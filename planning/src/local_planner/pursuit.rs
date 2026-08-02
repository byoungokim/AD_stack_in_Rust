/// Pure-pursuit primary executor on the global Hybrid A* path.
///
/// Eleven instrumented gauntlet attempts established that sampling-based DWA
/// with constant-curvature rollouts cannot execute the slalom weave fluidly:
/// each apex S-transition needs a counter-steer that no single (v, w) sample
/// represents, so the robot hitched at every apex, the stuck detector fired,
/// and recovery churned. Pure pursuit computes the EXACT arc to an adaptive
/// lookahead point on the (clearance-gradient-centered, curvature-capped)
/// global path — the apex S-transition is just the lookahead point crossing
/// the apex — and verifies that single arc with the SAME sub-stepped
/// collision + braking-extension machinery the DWA samples use
/// (`dwa::simulate_arc` + `dwa::speed_scaled_radius`). DWA remains the
/// fallback sampler when the pursuit arc is infeasible at every speed step.
use serde::Deserialize;

use super::dwa::{simulate_arc, speed_scaled_radius, wedged_allowance, DwaConfig};
use super::{Obstacle, RobotState, VelocityCommand};
use crate::global_planner::{PathWaypoint, SegmentDir};

#[derive(Debug, Clone, Deserialize)]
pub struct PursuitConfig {
    /// Lookahead gain (seconds of travel): L = clamp(k_v · |v|,
    /// lookahead_min, lookahead_max).
    #[serde(default = "default_k_v")]
    pub k_v: f64,
    /// Lower bound (m) of the adaptive lookahead — crawl/standstill stability.
    #[serde(default = "default_lookahead_min")]
    pub lookahead_min: f64,
    /// Upper bound (m) of the adaptive lookahead.
    #[serde(default = "default_lookahead_max")]
    pub lookahead_max: f64,
    /// Lateral-acceleration cap (m/s²): the curvature slow-down governor
    /// limits v ≤ sqrt(a_lat_max / |κ|) so tight path segments are taken
    /// slower — the same physics that makes DWA's tight samples slow.
    #[serde(default = "default_a_lat_max")]
    pub a_lat_max: f64,
    /// Speed cap (m/s, magnitude) while executing a planned REVERSE segment.
    /// Reverse is the blind direction (no forward sensor cone); planned
    /// reverses are short repositioning legs, not cruise.
    #[serde(default = "default_reverse_speed_cap")]
    pub reverse_speed_cap: f64,
}

fn default_k_v() -> f64 {
    1.0
}
fn default_lookahead_min() -> f64 {
    0.6
}
fn default_lookahead_max() -> f64 {
    2.5
}
fn default_a_lat_max() -> f64 {
    2.0
}
fn default_reverse_speed_cap() -> f64 {
    0.4
}

impl Default for PursuitConfig {
    fn default() -> Self {
        Self {
            k_v: default_k_v(),
            lookahead_min: default_lookahead_min(),
            lookahead_max: default_lookahead_max(),
            a_lat_max: default_a_lat_max(),
            reverse_speed_cap: default_reverse_speed_cap(),
        }
    }
}

impl PursuitConfig {
    /// Fail loudly on YAML values that would break the executor: a
    /// non-positive lookahead floor collapses the target search onto the
    /// robot (κ → ∞), an inverted [min, max] empties the clamp, a
    /// non-positive a_lat_max governs every curved segment to zero speed.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.k_v >= 0.0 && self.k_v.is_finite()) {
            return Err(format!(
                "pursuit.k_v must be >= 0 and finite, got {}",
                self.k_v
            ));
        }
        if !(self.lookahead_min > 0.0 && self.lookahead_min.is_finite()) {
            return Err(format!(
                "pursuit.lookahead_min must be > 0, got {}",
                self.lookahead_min
            ));
        }
        if !(self.lookahead_max >= self.lookahead_min && self.lookahead_max.is_finite()) {
            return Err(format!(
                "pursuit.lookahead_max ({}) must be >= pursuit.lookahead_min ({})",
                self.lookahead_max, self.lookahead_min
            ));
        }
        if !(self.a_lat_max > 0.0 && self.a_lat_max.is_finite()) {
            return Err(format!(
                "pursuit.a_lat_max must be > 0, got {}",
                self.a_lat_max
            ));
        }
        if !(self.reverse_speed_cap > 0.0 && self.reverse_speed_cap.is_finite()) {
            return Err(format!(
                "pursuit.reverse_speed_cap must be > 0, got {}",
                self.reverse_speed_cap
            ));
        }
        Ok(())
    }
}

/// Adaptive lookahead distance for the current speed:
/// L = clamp(k_v · |v|, lookahead_min, lookahead_max).
pub fn lookahead_distance(v: f64, config: &PursuitConfig) -> f64 {
    (config.k_v * v.abs()).clamp(config.lookahead_min, config.lookahead_max)
}

/// Control cycle of the 10Hz local-planner loop (matches DWA's window dt).
const CYCLE_DT: f64 = 0.1;

/// Speed-reduction steps tried when the full-speed pursuit arc fails its
/// rollout, as fractions of the governed speed, before the crawl attempt.
const SPEED_STEP_FRACTIONS: [f64; 4] = [1.0, 0.8, 0.6, 0.4];

/// Final speed step: the crawl (m/s). Matches the speed at/below which the
/// collision margin is fully scaled down (`MARGIN_SCALE_LOW_SPEED`).
const CRAWL_SPEED: f64 = 0.1;

/// Below this fraction of `lookahead_min`, the remaining path is too short to
/// pursue (κ = 2·sin α / d blows up on a vanishing chord) — the terminal
/// stop belongs to DWA's goal-distance scoring.
const MIN_TARGET_CHORD_FRACTION: f64 = 0.5;

/// Confidence for a verified pursuit command maps the worst rollout clearance
/// margin above the speed-scaled requirement onto [0.6, 0.9] — the same
/// collision-free band as DWA's 0.9, degrading toward (but staying above) the
/// arbitrator's degraded-response region for razor-thin verified arcs.
const CONFIDENCE_FLOOR: f64 = 0.6;
const CONFIDENCE_CEIL: f64 = 0.9;
/// Spare clearance (m) above the requirement that earns full confidence.
const CONFIDENCE_FULL_MARGIN_M: f64 = 0.3;

/// A verified pursuit command plus the maneuver flag the stuck detector
/// needs: `maneuver` is true while executing a planned REVERSE segment or the
/// scripted stop at a direction cusp — deliberately non-forward commands that
/// must not read as "no feasible forward plan".
#[derive(Debug, Clone)]
pub struct PursuitCommand {
    pub command: VelocityCommand,
    pub maneuver: bool,
}

/// Why pursuit produced no command this cycle. Carried in the `Err` of
/// `PursuitPlanner::compute` so the caller's fallback decisions — and the
/// hierarchy-fallback log when the scripted recovery reverse engages despite
/// a global path existing — are diagnostic instead of a bare `None`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PursuitDefer {
    /// Every speed step (including the crawl) failed the shared rollout
    /// verification: an obstacle blocks the pursuit arc.
    Blocked,
    /// The lookahead target lies behind the (virtual) robot — no pursuit arc
    /// in the run's travel direction exists.
    TargetBehind,
    /// The remaining path is too short to define a stable arc (terminal
    /// chord below the floor, or degenerate) — the endgame is DWA's.
    PathExhausted,
    /// No path to pursue.
    NoPath,
    /// The caller requested a stop (desired_speed <= 0); the accel-limited
    /// decel to exactly zero is DWA's tested behavior.
    StopRequested,
    /// The per-cycle accel window cannot reach any positive speed in the
    /// run's travel direction (still ramping out of opposing motion).
    ReverseRampOut,
}

impl std::fmt::Display for PursuitDefer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            PursuitDefer::Blocked => "Blocked",
            PursuitDefer::TargetBehind => "TargetBehind",
            PursuitDefer::PathExhausted => "PathExhausted",
            PursuitDefer::NoPath => "NoPath",
            PursuitDefer::StopRequested => "StopRequested",
            PursuitDefer::ReverseRampOut => "ReverseRampOut",
        })
    }
}

/// Remaining run distance below which a mid-path cusp counts as reached: the
/// executor stops and switches to the next segment from here. Shared with the
/// caller-side truncation (`active_run_truncation_index`), which drops the
/// exhausted run at the same radius — otherwise the centimeters of remnant
/// would "reactivate" as soon as the robot pulls away along the next run and
/// shuttle the executor between directions.
pub(crate) const CUSP_ARRIVE_M: f64 = 0.12;

/// Speeds below this (m/s, magnitude) count as stopped for the cusp
/// stop-and-switch (matches the behavior planner's stationary threshold).
const CUSP_STOP_SPEED: f64 = 0.05;

/// Determine the active (first) same-direction run of a path: returns the
/// inclusive end index and the run's travel direction. Waypoint `dir` is the
/// arrival direction and `path[0]` anchors the run, so the departure
/// direction from the anchor is `path[1].dir`.
pub(crate) fn active_run(path: &[PathWaypoint]) -> (usize, SegmentDir) {
    if path.len() < 2 {
        return (0, path.first().map(|w| w.dir).unwrap_or_default());
    }
    let dir = path[1].dir;
    let mut end = 1;
    while end + 1 < path.len() && path[end + 1].dir == dir {
        end += 1;
    }
    (end, dir)
}

pub struct PursuitPlanner {
    config: PursuitConfig,
    /// DWA dynamics + verification parameters: the pursuit command must obey
    /// the exact same executable envelope (max_curvature, max_angular_speed,
    /// accel limits) and pass the exact same rollout machinery as a DWA
    /// sample would.
    dwa: DwaConfig,
}

impl PursuitPlanner {
    pub fn new(config: PursuitConfig, dwa: DwaConfig) -> Self {
        Self { config, dwa }
    }

    /// Compute a verified pure-pursuit command on the active (possibly
    /// signed) path segment, or `Err(PursuitDefer)` naming WHY pursuit is
    /// infeasible (blocked arc at every speed step, target on the wrong
    /// side, path exhausted, or a stop requested) — the caller then falls
    /// back to DWA sampling (forward segments), a deliberate stop (reverse
    /// segments), or the scripted recovery machinery, and can log the
    /// deferral reason.
    ///
    /// Signed segments: a REVERSE run is executed with mirrored pure-pursuit
    /// geometry — the arc is computed for a virtual robot facing backwards
    /// (θ+π), whose forward kinematics are exactly the real robot's under a
    /// negative linear command — with speed capped at `reverse_speed_cap`.
    /// At a direction cusp the executor decelerates to a stop
    /// (`CUSP_ARRIVE_M` before the cusp point), holds the stop until
    /// (near-)stationary, then proceeds along the next run.
    ///
    /// `prev_cmd_v` is the SIGNED linear speed of the previously emitted
    /// command (from ANY executor); the output is accel-limited against it
    /// (max_acceleration / max_deceleration per cycle) so pursuit produces
    /// smooth ramps in both directions.
    pub fn compute(
        &self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
        prev_cmd_v: f64,
    ) -> Result<PursuitCommand, PursuitDefer> {
        if path.is_empty() {
            return Err(PursuitDefer::NoPath);
        }
        if desired_speed <= 0.0 {
            // A requested stop is DWA's job (accel-limited decel to exactly
            // zero); pursuing at zero speed is meaningless.
            return Err(PursuitDefer::StopRequested);
        }

        let (mut run_end, mut dir) = active_run(path);
        let mut run_start = 0usize;

        // Cusp arrival: when the active run is (nearly) exhausted and another
        // run follows, stop, then switch to the next run once stationary.
        if run_end + 1 < path.len() {
            let remaining = self.remaining_run_distance(state, &path[run_start..=run_end]);
            if remaining < CUSP_ARRIVE_M {
                let moving =
                    state.linear_vel.abs() > CUSP_STOP_SPEED || prev_cmd_v.abs() > CUSP_STOP_SPEED;
                if moving {
                    return Ok(self.stop_ramp(prev_cmd_v));
                }
                // Stopped at the cusp: the next run becomes active. The cusp
                // waypoint anchors it (same convention as `active_run`).
                run_start = run_end;
                let (next_end, next_dir) = active_run(&path[run_start..]);
                run_end = run_start + next_end;
                dir = next_dir;
            }
        }

        let run = &path[run_start..=run_end];
        let more_runs = run_end + 1 < path.len();
        let sign = dir.sign();

        // Still rolling AGAINST the active run's direction (e.g. main.rs
        // truncation flipped the run to the far side of a cusp while the
        // robot is still braking through it): scripted decel to zero — a
        // pursuit arc computed against the motion would be unverifiable.
        if sign * prev_cmd_v < -1e-6 || sign * state.linear_vel < -CUSP_STOP_SPEED {
            return Ok(self.stop_ramp(prev_cmd_v));
        }

        // Virtual robot for mirrored geometry: for reverse runs, a robot
        // facing θ+π moving forward at -linear_vel is kinematically identical
        // to the real robot under a negative linear command (θ' = ω in both).
        let vstate = if dir == SegmentDir::Reverse {
            RobotState {
                x: state.x,
                y: state.y,
                theta: normalize_angle(state.theta + std::f64::consts::PI),
                linear_vel: -state.linear_vel,
                angular_vel: state.angular_vel,
            }
        } else {
            state.clone()
        };

        // The short-chord floor only applies on the FINAL run (the terminal
        // stop is DWA's job). On a run with a successor the shrinking chord
        // is the CUSP APPROACH: keep pursuing the cusp point — the cusp stop
        // governor below bounds the speed, and the arrival branch above takes
        // over inside CUSP_ARRIVE_M.
        let (target, chord) = self
            .find_target(&vstate, run, !more_runs)
            .ok_or(PursuitDefer::PathExhausted)?;

        // Target bearing in the (virtual) robot frame.
        let dx = target.x - vstate.x;
        let dy = target.y - vstate.y;
        let lx = dx * vstate.theta.cos() + dy * vstate.theta.sin();
        let ly = -dx * vstate.theta.sin() + dy * vstate.theta.cos();
        if lx <= 0.0 {
            // Lookahead point behind the (virtual) robot: no pursuit arc in
            // the run's direction exists. DWA / recovery own this.
            return Err(PursuitDefer::TargetBehind);
        }

        // Classic pure pursuit: κ = 2·sin(α) / L toward the lookahead point,
        // clamped to the executable-curvature envelope (matching DWA's sample
        // filter — nothing downstream may clamp the command into an arc wider
        // than the verified one).
        let alpha = ly.atan2(lx);
        let kappa =
            (2.0 * alpha.sin() / chord).clamp(-self.dwa.max_curvature, self.dwa.max_curvature);

        // Speed cap: desired speed (magnitude), the platform limit, and the
        // reverse cap on reverse runs.
        let mut governed = desired_speed.clamp(0.0, self.dwa.max_speed);
        if dir == SegmentDir::Reverse {
            governed = governed.min(self.config.reverse_speed_cap);
        }
        // Curvature slow-down governor: v ≤ sqrt(a_lat_max / |κ|).
        if kappa.abs() > 1e-9 {
            governed = governed.min((self.config.a_lat_max / kappa.abs()).sqrt());
        }
        // Cusp stop governor: arrive at the cusp able to stop within the
        // remaining run distance at the braking limit.
        if more_runs {
            let remaining = self.remaining_run_distance(state, run);
            let stop_v =
                (2.0 * self.dwa.max_deceleration * (remaining - 0.5 * CUSP_ARRIVE_M).max(0.0))
                    .sqrt();
            governed = governed.min(stop_v);
            if governed <= 1e-6 {
                // Effectively at the cusp already — script the stop.
                return Ok(self.stop_ramp(prev_cmd_v));
            }
        }

        // Accel window against the previous command, in the run's travel
        // frame (prev_travel >= 0 after the direction guard above).
        let prev_travel = (sign * prev_cmd_v).max(0.0);
        let v_hi = (prev_travel + self.dwa.max_acceleration * CYCLE_DT).min(self.dwa.max_speed);
        if v_hi <= 0.0 {
            return Err(PursuitDefer::ReverseRampOut);
        }
        let v_lo = (prev_travel - self.dwa.max_deceleration * CYCLE_DT).clamp(0.0, v_hi);

        // Clearance/braking verification with speed step-down: try the
        // governed speed, then 80/60/40%, then the crawl, re-verifying each
        // step with the shared DWA rollout machinery. The rollout simulates
        // the ACTUAL SIGNED command (negative v on reverse runs), so the
        // swept arc, obstacle propagation, and braking extension all follow
        // the true motion.
        let maneuver = dir == SegmentDir::Reverse;
        let mut tried: [f64; 5] = [f64::NAN; 5];
        let candidates = SPEED_STEP_FRACTIONS
            .iter()
            .map(|f| governed * f)
            .chain(std::iter::once(CRAWL_SPEED));
        for (i, raw) in candidates.enumerate() {
            let v = raw.clamp(v_lo, v_hi);
            if v <= 1e-6 || tried.iter().any(|&t| (t - v).abs() < 1e-9) {
                continue; // no motion, or the accel window collapsed this step
            }
            tried[i] = v;
            let w = (kappa * v).clamp(-self.dwa.max_angular_speed, self.dwa.max_angular_speed);
            let v_cmd = sign * v;
            let roll = simulate_arc(&self.dwa, state, v_cmd, w, obstacles);
            let required = speed_scaled_radius(
                v_cmd,
                self.dwa.robot_radius,
                self.dwa.margin_low_speed_scale,
                self.dwa.high_speed_margin_gain,
            );
            // Per-obstacle acceptance with the wedged-start allowance
            // (mirrors `reverse_arc_blocker`'s "an already-wedged obstacle
            // cannot veto an improving maneuver"): an obstacle whose
            // clearance the CURRENT pose already violates only requires the
            // rollout never to come closer to it than the pose already is —
            // so the planned escape leg out of an inflation pocket is
            // executable. Every other obstacle keeps the exact standard
            // speed-scaled requirement.
            let worst_margin = roll
                .per_obstacle_min
                .iter()
                .zip(obstacles)
                .map(|(&d, obs)| {
                    d - wedged_allowance(state, obs, required, self.dwa.moving_obstacle_margin_gain)
                })
                .fold(f64::INFINITY, f64::min);
            if worst_margin >= -1e-9 {
                let margin = (worst_margin / CONFIDENCE_FULL_MARGIN_M).clamp(0.0, 1.0);
                return Ok(PursuitCommand {
                    command: VelocityCommand {
                        linear_x: v_cmd,
                        angular_z: w,
                        confidence: (CONFIDENCE_FLOOR
                            + (CONFIDENCE_CEIL - CONFIDENCE_FLOOR) * margin)
                            as f32,
                    },
                    maneuver,
                });
            }
        }
        Err(PursuitDefer::Blocked)
    }

    /// Scripted decel-to-zero at a direction cusp: magnitude shrinks by the
    /// braking limit per cycle, sign preserved, straight wheels. Part of
    /// planned execution — flagged as a maneuver so the stuck detector does
    /// not read the (near-)zero command as an infeasible plan.
    fn stop_ramp(&self, prev_cmd_v: f64) -> PursuitCommand {
        let mag = (prev_cmd_v.abs() - self.dwa.max_deceleration * CYCLE_DT).max(0.0);
        PursuitCommand {
            command: VelocityCommand {
                linear_x: prev_cmd_v.signum() * mag,
                angular_z: 0.0,
                confidence: 0.9,
            },
            maneuver: true,
        }
    }

    /// Distance (m) from the robot to the end of the active run: chord to the
    /// nearest run waypoint plus the run's arc length from there.
    fn remaining_run_distance(&self, state: &RobotState, run: &[PathWaypoint]) -> f64 {
        remaining_run_distance(run, state.x, state.y)
    }

    /// Choose the lookahead target: the first path waypoint at least the
    /// adaptive lookahead away, else the last waypoint (path shorter than the
    /// lookahead), else None when `terminal` and the remaining chord is too
    /// short to define a stable arc (mid-path cusp approaches keep pursuing —
    /// the cusp machinery owns the endgame there). Returns the target and the
    /// actual chord distance to it (pure pursuit uses the actual chord, not
    /// the nominal L).
    fn find_target<'p>(
        &self,
        state: &RobotState,
        path: &'p [PathWaypoint],
        terminal: bool,
    ) -> Option<(&'p PathWaypoint, f64)> {
        let l = lookahead_distance(state.linear_vel, &self.config);
        // Anchor the search at the waypoint nearest the robot and scan
        // FORWARD from there: on a winding path (the slalom S), an
        // already-passed waypoint can also lie ≥ L away, and a scan from the
        // path start would happily pursue it backwards.
        let dist_to =
            |wp: &PathWaypoint| ((wp.x - state.x).powi(2) + (wp.y - state.y).powi(2)).sqrt();
        let nearest_idx = path
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| dist_to(a).total_cmp(&dist_to(b)))
            .map(|(i, _)| i)?;
        let mut fallback: Option<(&PathWaypoint, f64)> = None;
        for wp in &path[nearest_idx..] {
            let dist = dist_to(wp);
            if dist >= l {
                return Some((wp, dist));
            }
            fallback = Some((wp, dist));
        }
        let (wp, dist) = fallback?;
        if terminal && dist < self.config.lookahead_min * MIN_TARGET_CHORD_FRACTION {
            return None;
        }
        if dist < 1e-6 {
            return None; // degenerate chord — κ = 2·sinα/d undefined
        }
        Some((wp, dist))
    }
}

/// Distance (m) from (x, y) to the end of a (sub)path: chord to the nearest
/// waypoint plus the arc length from there. Shared by the executor's cusp
/// logic and the caller-side truncation.
pub(crate) fn remaining_run_distance(run: &[PathWaypoint], x: f64, y: f64) -> f64 {
    let dist_to = |wp: &PathWaypoint| ((wp.x - x).powi(2) + (wp.y - y).powi(2)).sqrt();
    let Some(nearest_idx) = run
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| dist_to(a).total_cmp(&dist_to(b)))
        .map(|(i, _)| i)
    else {
        return 0.0;
    };
    let mut d = dist_to(&run[nearest_idx]);
    for w in run[nearest_idx..].windows(2) {
        d += ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
    }
    d
}

fn normalize_angle(a: f64) -> f64 {
    let mut v = a;
    while v > std::f64::consts::PI {
        v -= 2.0 * std::f64::consts::PI;
    }
    while v < -std::f64::consts::PI {
        v += 2.0 * std::f64::consts::PI;
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_path(len_m: f64, spacing: f64) -> Vec<PathWaypoint> {
        let n = (len_m / spacing) as usize;
        (1..=n)
            .map(|i| PathWaypoint {
                x: spacing * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect()
    }

    /// Circle of radius r tangent to +x at the origin, turning left, arc
    /// angle `span` rad, sampled every `step` rad.
    fn arc_path(r: f64, span: f64, step: f64) -> Vec<PathWaypoint> {
        let n = (span / step) as usize;
        (1..=n)
            .map(|i| {
                let t = step * i as f64;
                PathWaypoint {
                    x: r * t.sin(),
                    y: r * (1.0 - t.cos()),
                    theta: t,
                    steering: 0.0,
                    dir: Default::default(),
                }
            })
            .collect()
    }

    fn at_speed(v: f64) -> RobotState {
        RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: v,
            angular_vel: 0.0,
        }
    }

    #[test]
    fn test_pursuit_lookahead_scales_with_speed_within_bounds() {
        let cfg = PursuitConfig::default(); // k_v 1.0, [0.6, 2.5]
        assert_eq!(lookahead_distance(0.0, &cfg), 0.6);
        assert_eq!(lookahead_distance(0.3, &cfg), 0.6); // floored
        assert!((lookahead_distance(1.5, &cfg) - 1.5).abs() < 1e-12); // k_v·v
        assert!((lookahead_distance(2.2, &cfg) - 2.2).abs() < 1e-12);
        assert_eq!(lookahead_distance(5.0, &cfg), 2.5); // capped
        assert_eq!(lookahead_distance(-1.5, &cfg), 1.5); // |v|

        // Gain scales the slope: k_v = 0.5 halves the speed term.
        let cfg = PursuitConfig {
            k_v: 0.5,
            ..Default::default()
        };
        assert!((lookahead_distance(2.0, &cfg) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_pursuit_straight_path_runs_at_desired_speed() {
        // Straight path, no obstacles, cruising just below the target: the
        // governor must not bite (κ ≈ 0) and the command lands exactly on the
        // desired speed with a near-zero angular rate.
        let planner = PursuitPlanner::new(PursuitConfig::default(), DwaConfig::default());
        let cmd = planner
            .compute(&at_speed(1.95), &straight_path(10.0, 0.1), &[], 2.0, 1.95)
            .expect("open straight must be feasible")
            .command;
        assert!(
            (cmd.linear_x - 2.0).abs() < 1e-9,
            "straight path must run at desired speed, got {}",
            cmd.linear_x
        );
        assert!(cmd.angular_z.abs() < 1e-6);
        assert!(
            cmd.confidence >= 0.89,
            "open rollout must be full-confidence"
        );
    }

    #[test]
    fn test_pursuit_curvature_governor_caps_speed_on_tight_arc() {
        // Radius-0.5 arc → pursuit κ = 1/R = 2.0 exactly; the governor caps
        // v at sqrt(a_lat_max / κ) = sqrt(2.0 / 2.0) = 1.0 even though the
        // leg asks for 2.0. Robot already at 0.9 so the accel window
        // ([0.6, 1.15]) does not mask the governor.
        let planner = PursuitPlanner::new(PursuitConfig::default(), DwaConfig::default());
        let cmd = planner
            .compute(
                &at_speed(0.9),
                &arc_path(0.5, std::f64::consts::PI, 0.05),
                &[],
                2.0,
                0.9,
            )
            .expect("open arc must be feasible")
            .command;
        let kappa_cmd = cmd.angular_z / cmd.linear_x;
        assert!(
            cmd.linear_x <= (PursuitConfig::default().a_lat_max / kappa_cmd.abs()).sqrt() + 1e-6,
            "governor violated: v={} κ={}",
            cmd.linear_x,
            kappa_cmd
        );
        assert!(
            cmd.linear_x < 1.1,
            "tight arc must be governed well below the desired 2.0, got {}",
            cmd.linear_x
        );
        assert!(
            cmd.linear_x > 0.85,
            "governor must not collapse a feasible arc to a crawl, got {}",
            cmd.linear_x
        );
    }

    #[test]
    fn test_pursuit_output_inside_envelope_and_accel_limits() {
        // Across benign and adversarial geometries (goal nearly perpendicular,
        // standstill start, cruise), every emitted command must satisfy the
        // executable-curvature envelope, the angular-speed limit, and the
        // per-cycle accel window against the previous command.
        let dwa = DwaConfig::default();
        let planner = PursuitPlanner::new(PursuitConfig::default(), dwa.clone());
        let perpendicular: Vec<PathWaypoint> = (0..30)
            .map(|i| PathWaypoint {
                x: 0.4,
                y: 0.3 + 0.1 * i as f64,
                theta: std::f64::consts::FRAC_PI_2,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect();
        let cases: Vec<(RobotState, Vec<PathWaypoint>, f64, f64)> = vec![
            (at_speed(0.0), straight_path(10.0, 0.1), 2.2, 0.0),
            (at_speed(2.2), straight_path(10.0, 0.1), 2.2, 2.2),
            (at_speed(0.5), perpendicular, 2.2, 0.5),
            (
                at_speed(0.9),
                arc_path(0.5, std::f64::consts::PI, 0.05),
                2.2,
                0.9,
            ),
        ];
        for (state, path, desired, prev) in cases {
            let cmd = planner
                .compute(&state, &path, &[], desired, prev)
                .expect("open scene must be feasible")
                .command;
            assert!(
                cmd.angular_z.abs() <= cmd.linear_x * dwa.max_curvature + 1e-9,
                "curvature envelope violated: v={} w={}",
                cmd.linear_x,
                cmd.angular_z
            );
            assert!(cmd.angular_z.abs() <= dwa.max_angular_speed + 1e-9);
            assert!(
                cmd.linear_x <= prev + dwa.max_acceleration * 0.1 + 1e-9,
                "accel limit violated: prev={} cmd={}",
                prev,
                cmd.linear_x
            );
            assert!(
                cmd.linear_x >= (prev - dwa.max_deceleration * 0.1).max(0.0) - 1e-9,
                "decel limit violated: prev={} cmd={}",
                prev,
                cmd.linear_x
            );
            assert!(cmd.linear_x > 0.0 && cmd.linear_x <= dwa.max_speed + 1e-9);
        }
    }

    #[test]
    fn test_pursuit_steps_down_speed_before_wall() {
        // Wall across the path at x = 0.6: the full-speed and mid-fraction
        // arcs fail the shared rollout (horizon + braking extension), but the
        // crawl step still clears it — pursuit must emit a slow verified
        // command rather than defer to DWA.
        let planner = PursuitPlanner::new(PursuitConfig::default(), DwaConfig::default());
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(Obstacle::point(0.6, y));
            y += 0.05;
        }
        let cmd = planner
            .compute(&at_speed(0.3), &straight_path(10.0, 0.1), &wall, 0.5, 0.3)
            .expect("crawl toward a wall 0.6m out must still verify")
            .command;
        assert!(
            cmd.linear_x > 0.0 && cmd.linear_x <= 0.15,
            "expected a stepped-down crawl, got v={}",
            cmd.linear_x
        );
        assert!(cmd.confidence >= 0.6 - 1e-6);
    }

    #[test]
    fn test_pursuit_infeasible_when_every_speed_step_blocked() {
        // Wall at 0.35m: even the crawl's horizon+braking rollout ends inside
        // the required clearance — pursuit must return None (defer to DWA).
        let planner = PursuitPlanner::new(PursuitConfig::default(), DwaConfig::default());
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(Obstacle::point(0.35, y));
            y += 0.05;
        }
        assert_eq!(
            planner
                .compute(&at_speed(0.3), &straight_path(10.0, 0.1), &wall, 0.5, 0.3)
                .expect_err("fully blocked pursuit must defer to DWA"),
            PursuitDefer::Blocked,
        );
    }

    #[test]
    fn test_pursuit_defers_on_stop_request_and_rear_target() {
        // Every deferral carries its diagnostic reason (the hierarchy-
        // fallback log in main.rs prints it).
        let planner = PursuitPlanner::new(PursuitConfig::default(), DwaConfig::default());
        // desired_speed 0: the accel-limited decel to exactly zero is DWA's
        // tested behavior — pursuit must stand aside.
        assert_eq!(
            planner
                .compute(&at_speed(0.5), &straight_path(10.0, 0.1), &[], 0.0, 0.5)
                .unwrap_err(),
            PursuitDefer::StopRequested,
        );
        // Empty path.
        assert_eq!(
            planner
                .compute(&at_speed(0.5), &[], &[], 0.5, 0.5)
                .unwrap_err(),
            PursuitDefer::NoPath,
        );
        // Entire path behind the robot: no forward arc exists.
        let behind: Vec<PathWaypoint> = (1..=20)
            .map(|i| PathWaypoint {
                x: -0.1 * i as f64,
                y: 0.0,
                theta: std::f64::consts::PI,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect();
        assert_eq!(
            planner
                .compute(&at_speed(0.5), &behind, &[], 0.5, 0.5)
                .unwrap_err(),
            PursuitDefer::TargetBehind,
        );
        // Path exhausted (remaining chord below the stable-arc floor).
        let stub = vec![PathWaypoint {
            x: 0.1,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        assert_eq!(
            planner
                .compute(&at_speed(0.5), &stub, &[], 0.5, 0.5)
                .unwrap_err(),
            PursuitDefer::PathExhausted,
        );
    }

    #[test]
    fn test_pursuit_defer_reasons_format_for_logs() {
        // The fallback log prints these verbatim
        // ("scripted reverse engaged (pursuit=None reason=...)").
        assert_eq!(PursuitDefer::Blocked.to_string(), "Blocked");
        assert_eq!(PursuitDefer::TargetBehind.to_string(), "TargetBehind");
        assert_eq!(PursuitDefer::PathExhausted.to_string(), "PathExhausted");
        assert_eq!(PursuitDefer::NoPath.to_string(), "NoPath");
        assert_eq!(PursuitDefer::StopRequested.to_string(), "StopRequested");
        assert_eq!(PursuitDefer::ReverseRampOut.to_string(), "ReverseRampOut");
    }

    #[test]
    fn test_pursuit_wedged_start_accepts_only_clearance_improving_commands() {
        // (d) Wedged start: frontal cone (extent 0.15) 0.30m dead ahead —
        // net clearance 0.15m, BELOW even the crawl requirement (0.21 with
        // default radius 0.24). Under the old aggregate check every rollout
        // from this pose failed and the planned escape leg was unexecutable.
        // With the wedged-start allowance (mirroring reverse_arc_blocker):
        // - the planned REVERSE escape leg verifies — every rollout sample
        //   only moves AWAY from the wedging cone;
        // - the FORWARD path toward the same cone is still rejected — a
        //   clearance-DECREASING command gets no relief.
        use crate::global_planner::SegmentDir;
        let planner = PursuitPlanner::new(PursuitConfig::default(), DwaConfig::default());
        let state = at_speed(0.0);
        let cone = Obstacle {
            x: 0.30,
            y: 0.0,
            vx: 0.0,
            vy: 0.0,
            radius: 0.15,
        };

        let reverse_path: Vec<PathWaypoint> = (1..=20)
            .map(|i| PathWaypoint {
                x: -0.05 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Reverse,
            })
            .collect();
        let escape = planner
            .compute(&state, &reverse_path, std::slice::from_ref(&cone), 0.5, 0.0)
            .expect("clearance-improving reverse escape must verify from a wedged start");
        assert!(
            escape.command.linear_x < 0.0,
            "escape command must actually reverse, got v={}",
            escape.command.linear_x
        );
        assert!(escape.maneuver, "reverse escape leg is a planned maneuver");

        // Control: the forward path INTO the cone from the same wedged pose
        // is still infeasible at every speed step.
        assert_eq!(
            planner
                .compute(&state, &straight_path(10.0, 0.1), &[cone], 0.5, 0.0)
                .expect_err("a clearance-decreasing command must still be rejected"),
            PursuitDefer::Blocked,
        );
    }

    #[test]
    fn test_pursuit_config_validation() {
        assert!(PursuitConfig::default().validate().is_ok());
        assert!(PursuitConfig {
            lookahead_min: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(PursuitConfig {
            lookahead_max: 0.3, // < min
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(PursuitConfig {
            a_lat_max: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(PursuitConfig {
            k_v: f64::NAN,
            ..Default::default()
        }
        .validate()
        .is_err());
        // k_v = 0 is a valid constant-lookahead opt-out.
        assert!(PursuitConfig {
            k_v: 0.0,
            ..Default::default()
        }
        .validate()
        .is_ok());
        // A non-positive reverse cap would zero out every planned reverse.
        assert!(PursuitConfig {
            reverse_speed_cap: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
    }
}
