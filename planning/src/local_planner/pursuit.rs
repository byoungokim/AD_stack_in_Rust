/// Pure-pursuit primary executor on the global Hybrid A* path.
///
/// Eleven instrumented gauntlet attempts established that sampling-based DWA
/// with constant-curvature rollouts cannot execute the slalom weave fluidly:
/// each apex S-transition needs a counter-steer that no single (v, w) sample
/// represents, so the robot hitched at every apex, the stuck detector fired,
/// and recovery churned. Pure pursuit computes the EXACT arc to an adaptive
/// lookahead point on the (clearance-gradient-centered, curvature-capped)
/// global path — the apex S-transition is just the lookahead point crossing
/// the apex — and verifies that single arc with the shared sub-stepped
/// rollout machinery (`dwa::simulate_arc` for the static anticipatory sweep,
/// `dwa::simulate_committed_stop` for the truthful moving-obstacle check,
/// `dwa::speed_scaled_radius` for the requirement). DWA remains the fallback
/// sampler when the pursuit arc is infeasible at every speed step.
use serde::Deserialize;
use tracing::debug;

use super::dwa::{
    simulate_arc, simulate_committed_stop, speed_scaled_radius, wedged_allowance, DwaConfig,
};
use super::{Obstacle, RobotState, VelocityCommand};
use crate::global_planner::smoother::menger_curvature;
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
    /// Minimum turning speed (m/s): curvature-derived limits (instantaneous
    /// or profiled ahead) never command below this, so the robot keeps
    /// rolling through tight bends instead of stalling on a curvature cap.
    /// Genuine stops still go below it: cusp stop ramps, requested stops,
    /// and the rollout verification's speed step-down are all unaffected.
    #[serde(default = "default_v_turn_min")]
    pub v_turn_min: f64,
    /// Curvature-adaptive lookahead gain (dimensionless): L = clamp(L_base /
    /// (1 + gain·|κ_max_ahead|·L_base), lookahead_min, L_base), where
    /// κ_max_ahead is the tightest path curvature within L_base of arc.
    /// Long lookahead on straights (stability), short in bends (accurate
    /// tracking, no corner-cutting). 0 = speed-only lookahead (old behavior).
    #[serde(default = "default_curvature_lookahead_gain")]
    pub curvature_lookahead_gain: f64,
    /// Heading-error lookahead gain (dimensionless): after the target
    /// bearing α is known, one refinement pass re-finds the target at
    /// L' = clamp(L / (1 + gain·|α|), lookahead_min, L). The curvature-
    /// adaptive shrink reacts to PATH bends but not to the robot's own
    /// bearing error — post-recovery a straight path with a large α kept
    /// the long lookahead and produced wide, slowly-converging arcs (live:
    /// ~1.8m-radius weave lobes on the lane). Pulling the target in makes
    /// the arc turn at the platform's real radius, and the a_lat governor
    /// then brakes to turning speed automatically. 0 disables.
    #[serde(default = "default_heading_error_lookahead_gain")]
    pub heading_error_lookahead_gain: f64,
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
fn default_v_turn_min() -> f64 {
    0.2
}
fn default_curvature_lookahead_gain() -> f64 {
    0.6
}
fn default_heading_error_lookahead_gain() -> f64 {
    1.0
}

impl Default for PursuitConfig {
    fn default() -> Self {
        Self {
            k_v: default_k_v(),
            lookahead_min: default_lookahead_min(),
            lookahead_max: default_lookahead_max(),
            a_lat_max: default_a_lat_max(),
            reverse_speed_cap: default_reverse_speed_cap(),
            v_turn_min: default_v_turn_min(),
            curvature_lookahead_gain: default_curvature_lookahead_gain(),
            heading_error_lookahead_gain: default_heading_error_lookahead_gain(),
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
        // A non-positive minimum turning speed re-enables curvature stalls; a
        // huge one would let bends be entered at arbitrary speed. The upper
        // coherence bound against dwa.max_speed lives in
        // LocalPlannerConfig::validate (this struct cannot see the DWA limits).
        if !(self.v_turn_min > 0.0 && self.v_turn_min.is_finite()) {
            return Err(format!(
                "pursuit.v_turn_min must be > 0, got {}",
                self.v_turn_min
            ));
        }
        // Negative gain would GROW the lookahead in bends (and can cross the
        // pole of the shrink formula); 0 is the explicit speed-only opt-out.
        if !(self.curvature_lookahead_gain >= 0.0 && self.curvature_lookahead_gain.is_finite()) {
            return Err(format!(
                "pursuit.curvature_lookahead_gain must be >= 0 and finite, got {}",
                self.curvature_lookahead_gain
            ));
        }
        // Same polarity rule as the curvature gain: negative would GROW the
        // lookahead with bearing error; 0 is the explicit opt-out.
        if !(self.heading_error_lookahead_gain >= 0.0
            && self.heading_error_lookahead_gain.is_finite())
        {
            return Err(format!(
                "pursuit.heading_error_lookahead_gain must be >= 0 and finite, got {}",
                self.heading_error_lookahead_gain
            ));
        }
        Ok(())
    }
}

/// Speed-scaled base lookahead distance:
/// L_base = clamp(k_v · |v|, lookahead_min, lookahead_max).
pub fn lookahead_distance(v: f64, config: &PursuitConfig) -> f64 {
    (config.k_v * v.abs()).clamp(config.lookahead_min, config.lookahead_max)
}

/// Curvature-adaptive lookahead: the speed-scaled base shrinks with the
/// tightest curvature ahead of the robot within that base distance —
/// L = clamp(L_base / (1 + gain · |κ_max| · L_base), lookahead_min, L_base).
/// κ·L is the angle (rad) the base lookahead subtends on the bend, so the
/// shrink is dimensionless and monotonic: straights (κ_max ≈ 0) keep the
/// long, stable base; bends pull the target in so the pursuit arc tracks the
/// reference instead of chording across the corner (the corner-cutting that
/// fed the rollout verifier unverifiable arcs at the route's 90° entry).
/// gain = 0 restores the speed-only lookahead exactly.
pub(crate) fn curvature_scaled_lookahead(
    l_base: f64,
    kappa_max: f64,
    gain: f64,
    lookahead_min: f64,
) -> f64 {
    (l_base / (1.0 + gain * kappa_max.abs() * l_base)).clamp(lookahead_min.min(l_base), l_base)
}

/// Result of the single per-cycle curvature scan of the run ahead: the
/// anticipatory (backward-propagated) speed cap and the tightest |κ| within
/// the base lookahead (drives the curvature-adaptive lookahead shrink).
struct ProfileScan {
    speed_cap: f64,
    kappa_max_ahead: f64,
}

/// Control cycle of the 10Hz local-planner loop (matches DWA's window dt).
const CYCLE_DT: f64 = 0.1;

/// Minimum interval (ms) between per-rejection debug log lines: the speed
/// step-down loop can reject 5 arcs per 10Hz cycle, and a persistent blockage
/// repeats every cycle — without the limiter that is 50 lines/s of debug.
const BLOCKED_LOG_INTERVAL_MS: u64 = 500;

/// Rate limiter for the Blocked rejection debug log (process-wide; the log
/// is diagnostic, not per-planner state).
fn blocked_log_permitted() -> bool {
    use std::sync::atomic::{AtomicU64, Ordering};
    static LAST_MS: AtomicU64 = AtomicU64::new(0);
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);
    let last = LAST_MS.load(Ordering::Relaxed);
    now.saturating_sub(last) >= BLOCKED_LOG_INTERVAL_MS
        && LAST_MS
            .compare_exchange(last, now, Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
}

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

/// Which stage of the two-part rollout verification produced the binding
/// minimum for a rejected speed step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockedPhase {
    /// Static anticipatory sweep, within the fixed simulation horizon.
    Horizon,
    /// Static anticipatory sweep, in the braking extension past the horizon.
    BrakingExt,
    /// Truthful committed-stop rollout with full dynamic obstacle demands.
    CommittedStop,
}

impl BlockedPhase {
    fn as_str(&self) -> &'static str {
        match self {
            BlockedPhase::Horizon => "horizon",
            BlockedPhase::BrakingExt => "braking-ext",
            BlockedPhase::CommittedStop => "committed-stop",
        }
    }
}

/// Diagnostic record of one rejected verification attempt: which obstacle
/// bound (worst net margin after the wedged allowance), by how much, at what
/// commanded speed, and in which rollout phase (static horizon sweep, its
/// braking extension, or the committed-stop check). Carried inside
/// `PursuitDefer::Blocked` so the hierarchy-fallback log names the culprit
/// instead of a bare "Blocked".
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlockedStep {
    /// Commanded linear speed (signed) of the rejected step.
    pub speed_step: f64,
    /// Binding obstacle position (world frame) and speed estimate.
    pub obs_x: f64,
    pub obs_y: f64,
    pub obs_speed: f64,
    /// Worst net clearance to the binding obstacle along the failing check
    /// (surface distance minus dynamic demands), against `required`.
    pub net_clearance: f64,
    /// Required clearance for this obstacle after the wedged-start allowance.
    pub required: f64,
    /// Verification stage that produced the binding minimum.
    pub phase: BlockedPhase,
}

impl std::fmt::Display for BlockedStep {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "obs=({:.2},{:.2}) v_obs={:.2} net={:.3} req={:.3} at v={:.2} phase={}",
            self.obs_x,
            self.obs_y,
            self.obs_speed,
            self.net_clearance,
            self.required,
            self.speed_step,
            self.phase.as_str(),
        )
    }
}

/// Why pursuit produced no command this cycle. Carried in the `Err` of
/// `PursuitPlanner::compute` so the caller's fallback decisions — and the
/// hierarchy-fallback log when the scripted recovery reverse engages despite
/// a global path existing — are diagnostic instead of a bare `None`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PursuitDefer {
    /// Every speed step (including the crawl) failed the shared rollout
    /// verification: an obstacle blocks the pursuit arc. Carries the binding
    /// obstacle summary of the LAST (most conservative) attempted step —
    /// `None` only if the accel window let no step be attempted at all.
    Blocked(Option<BlockedStep>),
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
        match self {
            PursuitDefer::Blocked(Some(step)) => write!(f, "Blocked[{step}]"),
            PursuitDefer::Blocked(None) => f.write_str("Blocked"),
            PursuitDefer::TargetBehind => f.write_str("TargetBehind"),
            PursuitDefer::PathExhausted => f.write_str("PathExhausted"),
            PursuitDefer::NoPath => f.write_str("NoPath"),
            PursuitDefer::StopRequested => f.write_str("StopRequested"),
            PursuitDefer::ReverseRampOut => f.write_str("ReverseRampOut"),
        }
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

/// Anticipatory curvature profile: distance pad (m) added to the braking
/// distance when sizing the lookahead horizon along the tracked run.
const PROFILE_BRAKE_PAD_M: f64 = 0.5;
/// Anticipatory curvature profile: horizon floor (m) so slow approaches
/// still see the bend ahead.
const PROFILE_MIN_HORIZON_M: f64 = 3.0;
/// Curvature floor (1/m) below which a profile sample counts as straight
/// (avoids dividing a_lat_max by ~0).
const PROFILE_KAPPA_EPS: f64 = 1e-6;

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
    /// Ackermann wheelbase (m), used to invert the smoother's steering
    /// feed-forward back to path curvature (δ = atan(±κ·wb) ⇒ |κ| =
    /// |tan δ|/wb) for the anticipatory speed profile.
    wheelbase: f64,
}

impl PursuitPlanner {
    pub fn new(config: PursuitConfig, dwa: DwaConfig, wheelbase: f64) -> Self {
        debug_assert!(wheelbase > 0.0, "wheelbase must be positive");
        Self {
            config,
            dwa,
            wheelbase,
        }
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

        // Speed cap: desired speed (magnitude), the platform limit, and the
        // reverse cap on reverse runs.
        let mut governed = desired_speed.clamp(0.0, self.dwa.max_speed);
        if dir == SegmentDir::Reverse {
            governed = governed.min(self.config.reverse_speed_cap);
        }
        let leg_speed = governed;

        // Speed reference for the profile horizon and accel window: the
        // previous command in the run's travel frame (>= 0 after the
        // direction guard above), or the measured speed if it is higher.
        let prev_travel = (sign * prev_cmd_v).max(0.0);
        let v_now = prev_travel.max(vstate.linear_vel.max(0.0));

        // One curvature scan of the run ahead serves both governors: the
        // anticipatory speed cap (brake down to every upcoming curvature
        // limit in time) and the curvature-adaptive lookahead (long on
        // straights for stability, short in bends so the arc tracks the
        // reference instead of cutting the corner).
        let l_base = lookahead_distance(vstate.linear_vel, &self.config);
        let scan = self.curvature_profile(run, state.x, state.y, leg_speed, v_now, l_base);
        let lookahead = curvature_scaled_lookahead(
            l_base,
            scan.kappa_max_ahead,
            self.config.curvature_lookahead_gain,
            self.config.lookahead_min,
        );

        // The short-chord floor only applies on the FINAL run (the terminal
        // stop is DWA's job). On a run with a successor the shrinking chord
        // is the CUSP APPROACH: keep pursuing the cusp point — the cusp stop
        // governor below bounds the speed, and the arrival branch above takes
        // over inside CUSP_ARRIVE_M.
        let (target, chord) = self
            .find_target(&vstate, run, !more_runs, lookahead)
            .ok_or(PursuitDefer::PathExhausted)?;

        // Target bearing in the (virtual) robot frame.
        let rel = |px: f64, py: f64| {
            let dx = px - vstate.x;
            let dy = py - vstate.y;
            (
                dx * vstate.theta.cos() + dy * vstate.theta.sin(),
                -dx * vstate.theta.sin() + dy * vstate.theta.cos(),
            )
        };
        let (lx, ly) = rel(target.x, target.y);
        if lx <= 0.0 {
            // Lookahead point behind the (virtual) robot: no pursuit arc in
            // the run's direction exists. DWA / recovery own this.
            return Err(PursuitDefer::TargetBehind);
        }

        // Heading-error lookahead shrink, one refinement pass (see the
        // config doc): a large bearing error with a straight path ahead
        // kept the long lookahead and yielded a wide, slowly-converging
        // arc. Re-find the target at the α-shrunk lookahead; keep the
        // refined target only if it is still ahead of the virtual robot.
        let (mut lx, mut ly, mut chord) = (lx, ly, chord);
        let g = self.config.heading_error_lookahead_gain;
        let alpha0 = ly.atan2(lx);
        if g > 0.0 && alpha0.abs() > 0.1 {
            let l_short = (lookahead / (1.0 + g * alpha0.abs()))
                .clamp(self.config.lookahead_min.min(lookahead), lookahead);
            if l_short < chord {
                if let Some((t2, c2)) = self.find_target(&vstate, run, !more_runs, l_short) {
                    let (lx2, ly2) = rel(t2.x, t2.y);
                    if lx2 > 0.0 {
                        (lx, ly, chord) = (lx2, ly2, c2);
                    }
                }
            }
        }

        // Classic pure pursuit: κ = 2·sin(α) / L toward the lookahead point,
        // clamped to the executable-curvature envelope (matching DWA's sample
        // filter — nothing downstream may clamp the command into an arc wider
        // than the verified one).
        let alpha = ly.atan2(lx);
        let kappa =
            (2.0 * alpha.sin() / chord).clamp(-self.dwa.max_curvature, self.dwa.max_curvature);

        // Curvature slow-down governor, anticipatory: the classic
        // instantaneous cap v ≤ sqrt(a_lat_max / |κ|) on the CURRENT pursuit
        // arc is the s = 0 term of a braking-aware speed profile sampled
        // along the run ahead — arriving at a bend too fast cut the reference
        // line and got every rollout speed step rejected (Blocked → recovery
        // at the route's 90° entry turn). Curvature limits are floored at
        // v_turn_min (keep rolling through tight bends); a desired speed
        // below the floor is still respected.
        if kappa.abs() > 1e-9 {
            governed = governed.min(
                (self.config.a_lat_max / kappa.abs())
                    .sqrt()
                    .max(self.config.v_turn_min),
            );
        }
        governed = governed.min(scan.speed_cap);
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
        // frame.
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
        //
        // Each step passes a TWO-PART check, both against the speed-scaled
        // requirement with the wedged-start allowance:
        // 1. Static anticipatory sweep: the full `simulate_arc` horizon +
        //    braking extension against every obstacle held at its CURRENT
        //    position (zero velocity). For static scenes this is exactly the
        //    old verification.
        // 2. Truthful committed-stop rollout (`simulate_committed_stop`):
        //    the arc held for the command-commitment window, then a stop —
        //    with obstacle propagation, uncertainty inflation, and the
        //    moving-obstacle margin billed over that truthful timeline.
        // Part 2 replaces the old practice of billing full dynamic demands
        // over the fixed 1.5s horizon plus a crawl-speed braking extension:
        // that extrapolation charged a 0.1 m/s crawl with ~3 seconds of
        // obstacle evolution, so a tracked obstacle carrying a modest
        // (frequently spurious, yaw-skew-induced) velocity estimate vetoed
        // EVERY speed step of the route's entry turn from 1.4m away —
        // Blocked → recovery, the frozen-robot failure. The safety contract
        // is preserved: a command is emitted only if its truthful execution
        // (including braking to a stop along the commanded arc) stays
        // outside the required margins of every obstacle, statics checked
        // over the whole anticipatory sweep, movers where they will actually
        // be while the command can still be in effect. Later cycles
        // re-verify the shrinking gap every 100ms, so a genuinely
        // approaching object stops the robot while a stop is still
        // margin-clean.
        let maneuver = dir == SegmentDir::Reverse;
        let mut tried: [f64; 5] = [f64::NAN; 5];
        let mut last_blocked: Option<BlockedStep> = None;
        // Static anticipatory view: obstacles pinned at their current
        // positions. Cheap clone once per cycle; identical to `obstacles`
        // for untracked points.
        let static_obstacles: Vec<Obstacle> = obstacles
            .iter()
            .map(|o| Obstacle {
                vx: 0.0,
                vy: 0.0,
                ..o.clone()
            })
            .collect();
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
            let roll = simulate_arc(&self.dwa, state, v_cmd, w, &static_obstacles);
            let committed = simulate_committed_stop(&self.dwa, state, v_cmd, w, obstacles);
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
            // speed-scaled requirement. Both checks (static anticipatory
            // sweep, truthful committed stop) must clear their allowance.
            let mut worst_margin = f64::INFINITY;
            let mut binding: Option<BlockedStep> = None;
            for (i, obs) in obstacles.iter().enumerate() {
                let allow_static = wedged_allowance(
                    state,
                    &static_obstacles[i],
                    required,
                    self.dwa.moving_obstacle_margin_gain,
                );
                let allow_dyn =
                    wedged_allowance(state, obs, required, self.dwa.moving_obstacle_margin_gain);
                let m_static = roll.per_obstacle_min[i] - allow_static;
                let m_dyn = committed[i] - allow_dyn;
                let (margin, net, allow, phase) = if m_static <= m_dyn {
                    let phase =
                        if roll.per_obstacle_braking_min[i] <= roll.per_obstacle_min[i] + 1e-12 {
                            BlockedPhase::BrakingExt
                        } else {
                            BlockedPhase::Horizon
                        };
                    (m_static, roll.per_obstacle_min[i], allow_static, phase)
                } else {
                    (m_dyn, committed[i], allow_dyn, BlockedPhase::CommittedStop)
                };
                if margin < worst_margin {
                    worst_margin = margin;
                    binding = Some(BlockedStep {
                        speed_step: v_cmd,
                        obs_x: obs.x,
                        obs_y: obs.y,
                        obs_speed: obs.speed(),
                        net_clearance: net,
                        required: allow,
                        phase,
                    });
                }
            }
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
            // Rejection instrumentation: name the binding obstacle, its net
            // clearance vs. the requirement, and the failing rollout phase.
            // Rate-limited — the step-down loop rejects up to 5 arcs per
            // 10Hz cycle and this must not flood the debug log.
            if let Some(step) = &binding {
                last_blocked = Some(*step);
                if blocked_log_permitted() {
                    debug!("pursuit rollout rejected: {step}");
                }
            }
        }
        Err(PursuitDefer::Blocked(last_blocked))
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

    /// One curvature scan of the run ahead, from the robot's projection
    /// (nearest waypoint), serving both anticipatory governors.
    ///
    /// **Speed cap** — samples over a horizon of max(braking distance at
    /// `v_now` + pad, floor). Per sample s: v_limit(s) =
    /// clamp(sqrt(a_lat_max / |κ(s)|), v_turn_min, leg_speed) — and the
    /// permissible speed NOW is the backward-propagated min over samples of
    /// sqrt(v_limit(s)² + 2·max_deceleration·arc(s)), i.e. the robot must be
    /// able to brake down to every upcoming limit by the time it arrives
    /// there. One control cycle of travel is subtracted from each sample's
    /// arc distance: the command issued now stays in effect while the robot
    /// covers that distance, so without the shift the robot would cross each
    /// limit one cycle late.
    ///
    /// **κ_max_ahead** — the maximum |κ| over samples within `l_base` of arc
    /// (the speed-scaled base lookahead), feeding the curvature-adaptive
    /// lookahead shrink.
    ///
    /// Cusps are NOT part of this profile: the cusp stop governor in
    /// `compute` (profiling toward v = 0 at the cusp arc, below v_turn_min)
    /// stays the binding constraint near direction switches.
    fn curvature_profile(
        &self,
        run: &[PathWaypoint],
        x: f64,
        y: f64,
        leg_speed: f64,
        v_now: f64,
        l_base: f64,
    ) -> ProfileScan {
        let mut scan = ProfileScan {
            speed_cap: f64::INFINITY,
            kappa_max_ahead: 0.0,
        };
        let dist_to = |wp: &PathWaypoint| ((wp.x - x).powi(2) + (wp.y - y).powi(2)).sqrt();
        let Some(nearest) = run
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| dist_to(a).total_cmp(&dist_to(b)))
            .map(|(i, _)| i)
        else {
            return scan;
        };
        let horizon = (v_now * v_now / (2.0 * self.dwa.max_deceleration) + PROFILE_BRAKE_PAD_M)
            .max(PROFILE_MIN_HORIZON_M);
        let latency = v_now * CYCLE_DT;
        let mut arc = 0.0;
        for i in nearest..run.len() {
            if i > nearest {
                arc +=
                    ((run[i].x - run[i - 1].x).powi(2) + (run[i].y - run[i - 1].y).powi(2)).sqrt();
                if arc > horizon.max(l_base) {
                    break;
                }
            }
            let kappa = self.waypoint_curvature(run, i);
            if arc <= l_base && kappa > scan.kappa_max_ahead {
                scan.kappa_max_ahead = kappa;
            }
            if arc <= horizon {
                let v_limit = (self.config.a_lat_max / kappa.max(PROFILE_KAPPA_EPS))
                    .sqrt()
                    .max(self.config.v_turn_min)
                    .min(leg_speed);
                let s = (arc - latency).max(0.0);
                scan.speed_cap = scan
                    .speed_cap
                    .min((v_limit * v_limit + 2.0 * self.dwa.max_deceleration * s).sqrt());
            }
        }
        scan
    }

    /// |κ| (1/m) of the path at `run[i]`: the smoother's steering
    /// feed-forward inverted through the bicycle model when present
    /// (δ = atan(±κ·wheelbase)), else the 3-point discrete (Menger)
    /// curvature of the neighboring samples — raw A* fallback paths carry no
    /// steering. Endpoints without both neighbors count as straight.
    fn waypoint_curvature(&self, run: &[PathWaypoint], i: usize) -> f64 {
        let wp = &run[i];
        if wp.steering.abs() > 1e-6 {
            return (wp.steering.tan() / self.wheelbase).abs();
        }
        if i == 0 || i + 1 >= run.len() {
            return 0.0;
        }
        menger_curvature(
            (run[i - 1].x, run[i - 1].y),
            (run[i].x, run[i].y),
            (run[i + 1].x, run[i + 1].y),
        )
        .abs()
    }

    /// Choose the lookahead target: the first path waypoint at least the
    /// adaptive lookahead `l` (speed-scaled base shrunk by the curvature
    /// ahead) away, else the last waypoint (path shorter than the lookahead),
    /// else None when `terminal` and the remaining chord is too short to
    /// define a stable arc (mid-path cusp approaches keep pursuing — the cusp
    /// machinery owns the endgame there). Returns the target and the actual
    /// chord distance to it (pure pursuit uses the actual chord, not the
    /// nominal L).
    fn find_target<'p>(
        &self,
        state: &RobotState,
        path: &'p [PathWaypoint],
        terminal: bool,
        l: f64,
    ) -> Option<(&'p PathWaypoint, f64)> {
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

    /// Limo Pro wheelbase (m), matching `mpc.wheelbase`'s default.
    const TEST_WHEELBASE: f64 = 0.2;

    #[test]
    fn test_heading_error_shrink_tightens_convergence_arc() {
        // Robot 1.2m laterally OFF a straight reference, path ahead dead
        // straight (no curvature for the curvature-adaptive shrink to react
        // to), moving at speed. With the heading-error gain the target is
        // pulled in, the arc curvature tightens, and the a_lat governor
        // brakes; with gain 0 the long lookahead yields the old wide arc.
        let path = straight_path(6.0, 0.1);
        let state = RobotState {
            x: 0.0,
            y: 1.2,
            theta: 0.0,
            linear_vel: 2.0,
            angular_vel: 0.0,
        };
        let with_shrink = default_planner()
            .compute(&state, &path, &[], 2.0, 2.0)
            .expect("open scene must be pursuable");
        let no_shrink = PursuitPlanner::new(
            PursuitConfig {
                heading_error_lookahead_gain: 0.0,
                ..Default::default()
            },
            DwaConfig::default(),
            TEST_WHEELBASE,
        )
        .compute(&state, &path, &[], 2.0, 2.0)
        .expect("open scene must be pursuable");

        let kappa = |c: &PursuitCommand| (c.command.angular_z / c.command.linear_x.max(1e-9)).abs();
        assert!(
            kappa(&with_shrink) > kappa(&no_shrink) * 1.5,
            "shrunk lookahead must command a materially tighter arc: {} vs {}",
            kappa(&with_shrink),
            kappa(&no_shrink),
        );
        assert!(
            with_shrink.command.linear_x <= no_shrink.command.linear_x + 1e-9,
            "tighter arc must not be taken faster: {} vs {}",
            with_shrink.command.linear_x,
            no_shrink.command.linear_x,
        );
    }

    fn default_planner() -> PursuitPlanner {
        PursuitPlanner::new(
            PursuitConfig::default(),
            DwaConfig::default(),
            TEST_WHEELBASE,
        )
    }

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
        let planner = default_planner();
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
        let planner = default_planner();
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

    /// Straight run of `straight_m` along +x, then a left quarter-arc of
    /// radius `r` (κ = 1/r), arc sampled every ~5cm so the profile sees the
    /// bend entry crisply.
    fn bend_path(straight_m: f64, r: f64) -> Vec<PathWaypoint> {
        let mut path = straight_path(straight_m, 0.15);
        let step = 0.05 / r;
        let mut t = step;
        while t <= std::f64::consts::FRAC_PI_2 {
            path.push(PathWaypoint {
                x: straight_m + r * t.sin(),
                y: r * (1.0 - t.cos()),
                theta: t,
                steering: 0.0,
                dir: Default::default(),
            });
            t += step;
        }
        path
    }

    #[test]
    fn test_pursuit_brakes_ahead_of_bend_not_at_it() {
        // (a) + (c): 90° bend of κ = 2.0 starting at x = 2.0, desired 2.2.
        // Approach simulated open-loop along the straight: the commanded
        // speed must start dropping BEFORE the bend (a multi-cycle
        // anticipatory ramp), never fall faster than max_deceleration per
        // cycle, and enter the bend at most a whisker above
        // sqrt(a_lat_max/κ_bend) = 1.0. The old instantaneous-only governor
        // arrived at the bend at full speed, cut the reference line, and got
        // every rollout speed step rejected (Blocked → recovery).
        let planner = default_planner();
        let dwa = DwaConfig::default();
        let cfg = PursuitConfig::default();
        let path = bend_path(2.0, 0.5);
        let entry_limit = (cfg.a_lat_max / 2.0).sqrt(); // 1.0

        let (mut xpos, mut prev) = (0.0f64, 2.2f64);
        let mut cmds: Vec<f64> = Vec::new();
        while xpos < 2.0 {
            let state = RobotState {
                x: xpos,
                y: 0.0,
                theta: 0.0,
                linear_vel: prev,
                angular_vel: 0.0,
            };
            let v = planner
                .compute(&state, &path, &[], 2.2, prev)
                .expect("open approach must be feasible")
                .command
                .linear_x;
            cmds.push(v);
            xpos += v * CYCLE_DT;
            prev = v;
            assert!(cmds.len() < 200, "approach simulation runaway");
        }
        // Entry speed: the command computed at the first pose at/past the
        // bend entry (the profile's s = 0 sample is the bend itself there).
        let entry = planner
            .compute(
                &RobotState {
                    x: xpos,
                    y: 0.0,
                    theta: 0.0,
                    linear_vel: prev,
                    angular_vel: 0.0,
                },
                &path,
                &[],
                2.2,
                prev,
            )
            .expect("bend entry must be feasible")
            .command
            .linear_x;

        // Far from the bend the profile does not bite: full desired speed.
        assert!(
            (cmds[0] - 2.2).abs() < 1e-9,
            "profile must not slow the distant straight, got {}",
            cmds[0]
        );
        // Monotone, decel-bounded anticipatory ramp (the (c) criterion).
        for w in cmds.windows(2) {
            assert!(
                w[1] <= w[0] + 1e-9,
                "commanded speed rose during the bend approach: {:?}",
                w
            );
            assert!(
                w[1] >= w[0] - dwa.max_deceleration * CYCLE_DT - 1e-6,
                "profile demanded more than max_deceleration per cycle: {:?}",
                w
            );
        }
        let ramp_cycles = cmds.iter().filter(|v| **v < 2.2 - 1e-6).count();
        assert!(
            ramp_cycles >= 3,
            "no multi-cycle ramp BEFORE the bend: {:?}",
            cmds
        );
        assert!(
            entry <= entry_limit + 0.15,
            "entered the bend too fast: {} > {}",
            entry,
            entry_limit
        );
        assert!(
            entry >= cfg.v_turn_min,
            "entry speed {} fell below v_turn_min",
            entry
        );
    }

    #[test]
    fn test_profile_and_lookahead_are_identity_on_straights() {
        // (b) + (f): on a straight path the curvature scan finds κ_max = 0,
        // the anticipatory cap equals the leg speed (desired is commanded —
        // covered end-to-end by test_pursuit_straight_path_runs_at_desired_
        // speed), and the adaptive lookahead is EXACTLY the speed-scaled
        // base.
        let planner = default_planner();
        let cfg = PursuitConfig::default();
        let path = straight_path(10.0, 0.15);
        let l_base = lookahead_distance(2.2, &cfg);
        let scan = planner.curvature_profile(&path, 0.0, 0.0, 2.2, 2.2, l_base);
        assert_eq!(scan.kappa_max_ahead, 0.0);
        assert!(
            (scan.speed_cap - 2.2).abs() < 1e-9,
            "straight profile must cap at the leg speed, got {}",
            scan.speed_cap
        );
        assert_eq!(
            curvature_scaled_lookahead(
                l_base,
                scan.kappa_max_ahead,
                cfg.curvature_lookahead_gain,
                cfg.lookahead_min
            ),
            l_base
        );
        // gain = 0 is the exact speed-only opt-out on any curvature.
        assert_eq!(
            curvature_scaled_lookahead(l_base, 2.4, 0.0, cfg.lookahead_min),
            l_base
        );
    }

    #[test]
    fn test_lookahead_shrinks_inside_bends() {
        // (g): κ ≈ 2.0 within the base lookahead ⇒ the adaptive lookahead
        // pulls the target in (accurate tracking, no corner-cutting) but
        // never below lookahead_min.
        let planner = default_planner();
        let cfg = PursuitConfig::default();
        let path = arc_path(0.5, std::f64::consts::PI, 0.05);
        let l_base = cfg.lookahead_max; // cruise-speed base, 2.5
        let scan = planner.curvature_profile(&path, 0.0, 0.0, 2.2, 2.2, l_base);
        assert!(
            (scan.kappa_max_ahead - 2.0).abs() < 1e-6,
            "κ_max on the r=0.5 arc must be 2.0, got {}",
            scan.kappa_max_ahead
        );
        let l = curvature_scaled_lookahead(
            l_base,
            scan.kappa_max_ahead,
            cfg.curvature_lookahead_gain,
            cfg.lookahead_min,
        );
        assert!(
            l < 0.6 * l_base,
            "tight bend must shrink the lookahead: {} vs base {}",
            l,
            l_base
        );
        assert!(l >= cfg.lookahead_min - 1e-9);
    }

    #[test]
    fn test_waypoint_curvature_feed_forward_and_menger_fallback() {
        let planner = default_planner();
        // Straight geometry carrying a steering feed-forward of
        // atan(κ·wheelbase) with κ = 1.5: the feed-forward wins over the
        // (zero) discrete curvature.
        let mut ff = straight_path(3.0, 0.15);
        for wp in &mut ff {
            wp.steering = (1.5 * TEST_WHEELBASE).atan();
        }
        assert!((planner.waypoint_curvature(&ff, 5) - 1.5).abs() < 1e-9);
        // Zero steering falls back to 3-point Menger: exact on a circle
        // (r = 0.5 ⇒ κ = 2).
        let arc = arc_path(0.5, std::f64::consts::PI, 0.1);
        assert!((planner.waypoint_curvature(&arc, 3) - 2.0).abs() < 1e-9);
        // Endpoints without both neighbors count as straight.
        assert_eq!(planner.waypoint_curvature(&arc, 0), 0.0);
        assert_eq!(planner.waypoint_curvature(&arc, arc.len() - 1), 0.0);
    }

    #[test]
    fn test_v_turn_min_keeps_tight_bends_rolling() {
        // (d): a_lat_max dropped to 0.05 so the raw curvature limit on the
        // κ = 2.0 arc (sqrt(0.05/2) ≈ 0.16 m/s) falls BELOW v_turn_min: the
        // floor must win — the robot keeps rolling through the bend at
        // v_turn_min instead of stalling on curvature alone. (The rollout
        // collision verification may still step down/defer; that hierarchy
        // is unchanged and not in play in this open scene.)
        let cfg = PursuitConfig {
            a_lat_max: 0.05,
            ..Default::default()
        };
        let v_turn_min = cfg.v_turn_min;
        let planner = PursuitPlanner::new(cfg, DwaConfig::default(), TEST_WHEELBASE);
        let cmd = planner
            .compute(
                &at_speed(0.3),
                &arc_path(0.5, std::f64::consts::PI, 0.05),
                &[],
                2.0,
                0.3,
            )
            .expect("open arc must stay feasible")
            .command;
        assert!(
            (cmd.linear_x - v_turn_min).abs() < 1e-6,
            "curvature floor must bind at exactly v_turn_min, got {}",
            cmd.linear_x
        );
    }

    #[test]
    fn test_cusp_stop_ramp_binds_below_v_turn_min() {
        // (e): forward run ending in a cusp 0.2m ahead, then a reverse run.
        // With an exaggerated v_turn_min = 1.0 the cusp stop governor must
        // stay the binding constraint — the profile targets v = 0 at the
        // cusp arc, and the turning-speed floor must NOT lift it.
        let cfg = PursuitConfig {
            v_turn_min: 1.0,
            ..Default::default()
        };
        let v_turn_min = cfg.v_turn_min;
        let dwa = DwaConfig::default();
        let planner = PursuitPlanner::new(cfg, dwa.clone(), TEST_WHEELBASE);
        let mut path: Vec<PathWaypoint> = (1..=4)
            .map(|i| PathWaypoint {
                x: 0.05 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Forward,
            })
            .collect();
        for i in 1..=8 {
            path.push(PathWaypoint {
                x: 0.20 - 0.05 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Reverse,
            });
        }
        let cmd = planner
            .compute(&at_speed(1.2), &path, &[], 2.0, 1.2)
            .expect("cusp approach must be feasible")
            .command;
        // Remaining run distance from the origin is 0.20m.
        let stop_v = (2.0 * dwa.max_deceleration * (0.20 - 0.5 * CUSP_ARRIVE_M)).sqrt();
        assert!(
            cmd.linear_x <= stop_v + 1e-6,
            "cusp stop ramp violated: {} > {}",
            cmd.linear_x,
            stop_v
        );
        assert!(
            cmd.linear_x < v_turn_min,
            "v_turn_min must not lift the cusp stop ramp, got {}",
            cmd.linear_x
        );
        assert!(cmd.linear_x > 0.0);
    }

    #[test]
    fn test_pursuit_output_inside_envelope_and_accel_limits() {
        // Across benign and adversarial geometries (goal nearly perpendicular,
        // standstill start, cruise), every emitted command must satisfy the
        // executable-curvature envelope, the angular-speed limit, and the
        // per-cycle accel window against the previous command.
        let dwa = DwaConfig::default();
        let planner = PursuitPlanner::new(PursuitConfig::default(), dwa.clone(), TEST_WHEELBASE);
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
        let planner = default_planner();
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
        let planner = default_planner();
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(Obstacle::point(0.35, y));
            y += 0.05;
        }
        let err = planner
            .compute(&at_speed(0.3), &straight_path(10.0, 0.1), &wall, 0.5, 0.3)
            .expect_err("fully blocked pursuit must defer to DWA");
        let PursuitDefer::Blocked(Some(step)) = err else {
            panic!("expected Blocked with a binding-obstacle summary, got {err:?}");
        };
        // The binding obstacle must be named from the wall row (x = 0.35) with
        // a net clearance below the requirement.
        assert!(
            (step.obs_x - 0.35).abs() < 1e-9,
            "binding obstacle must be on the wall, got ({}, {})",
            step.obs_x,
            step.obs_y
        );
        assert!(
            step.net_clearance < step.required,
            "recorded rejection must show net < required: {step}"
        );
        assert_eq!(step.obs_speed, 0.0);
    }

    #[test]
    fn test_pursuit_defers_on_stop_request_and_rear_target() {
        // Every deferral carries its diagnostic reason (the hierarchy-
        // fallback log in main.rs prints it).
        let planner = default_planner();
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
        assert_eq!(PursuitDefer::Blocked(None).to_string(), "Blocked");
        // With a recorded rejection the summary names the binding obstacle,
        // the clearance-vs-requirement, the speed step, and the phase.
        let with_diag = PursuitDefer::Blocked(Some(BlockedStep {
            speed_step: 0.1,
            obs_x: 1.6,
            obs_y: 0.3,
            obs_speed: 0.5,
            net_clearance: 0.12,
            required: 0.215,
            phase: BlockedPhase::CommittedStop,
        }))
        .to_string();
        assert_eq!(
            with_diag,
            "Blocked[obs=(1.60,0.30) v_obs=0.50 net=0.120 req=0.215 at v=0.10 phase=committed-stop]"
        );
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
        let planner = default_planner();
        let state = at_speed(0.0);
        let cone = Obstacle {
            x: 0.30,
            y: 0.0,
            vx: 0.0,
            vy: 0.0,
            radius: 0.15,
            ..Default::default()
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
        assert!(matches!(
            planner
                .compute(&state, &straight_path(10.0, 0.1), &[cone], 0.5, 0.0)
                .expect_err("a clearance-decreasing command must still be rejected"),
            PursuitDefer::Blocked(_),
        ));
    }

    /// Scratch reproduction of the live entry-turn Blocked defect.
    /// Smoothed-path-like chain: left arc from (0.0, 0.267) θ=0 radius 0.383
    /// to heading north at x≈0.383, north leg to y≈2.1, right arc to east
    /// along y≈2.48. Feed-forward steering = atan(κ·wb) signed.
    fn entry_turn_path() -> Vec<PathWaypoint> {
        let r = 0.383;
        let steer = |kappa: f64| (kappa * TEST_WHEELBASE).atan();
        let mut path = Vec::new();
        // Entry left arc: center (0.0, 0.267 + r).
        let n = 12;
        for i in 1..=n {
            let t = std::f64::consts::FRAC_PI_2 * i as f64 / n as f64;
            path.push(PathWaypoint {
                x: r * t.sin(),
                y: 0.267 + r * (1.0 - t.cos()),
                theta: t,
                steering: steer(1.0 / r),
                dir: SegmentDir::Forward,
            });
        }
        // North leg x = r, y from 0.267+r to 2.1.
        let mut y = 0.267 + r + 0.1;
        while y <= 2.1 {
            path.push(PathWaypoint {
                x: r,
                y,
                theta: std::f64::consts::FRAC_PI_2,
                steering: 0.0,
                dir: SegmentDir::Forward,
            });
            y += 0.1;
        }
        // Top right arc: center (2r, 2.1), θ: π/2 → 0 (turning right/east).
        for i in 1..=n {
            let t = std::f64::consts::FRAC_PI_2 * i as f64 / n as f64;
            path.push(PathWaypoint {
                x: r + r * (1.0 - t.cos()),
                y: 2.1 + r * t.sin(),
                theta: std::f64::consts::FRAC_PI_2 - t,
                steering: steer(-1.0 / r),
                dir: SegmentDir::Forward,
            });
        }
        // East leg along y = 2.1 + r.
        let mut x = 2.0 * r + 0.1;
        while x <= 4.0 {
            path.push(PathWaypoint {
                x,
                y: 2.1 + r,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Forward,
            });
            x += 0.1;
        }
        path
    }

    /// Ground-truth static scene: west arena fence x≈-1.35, north corridor
    /// wall y≈1.57 starting x≈0.8, cone cluster at (1.6, 0.3).
    fn entry_turn_obstacles(cone_v: (f64, f64), copies: usize) -> Vec<Obstacle> {
        let mut obs = Vec::new();
        for _ in 0..copies {
            let mut y = -0.5;
            while y <= 3.5 {
                obs.push(Obstacle::point(-1.35, y));
                y += 0.1;
            }
            let mut x = 0.8;
            while x <= 4.0 {
                obs.push(Obstacle::point(x, 1.57));
                x += 0.1;
            }
            obs.push(Obstacle {
                x: 1.6,
                y: 0.3,
                vx: cone_v.0,
                vy: cone_v.1,
                radius: 0.2,
                ..Default::default()
            });
        }
        obs
    }

    #[test]
    fn test_entry_turn_verifies_despite_phantom_tracked_velocity() {
        // Live-defect regression (route entry turn, obstacle gauntlet):
        // robot near spawn heading +x, reference turning north then east,
        // west fence / north corridor wall / slalom cone all with ample
        // ground-truth clearance — yet pursuit returned Blocked at EVERY
        // speed step, 12-16 cycles per attempt, forcing recovery. Root
        // cause: the tracked cone cluster carried a spurious (yaw-skew)
        // velocity estimate, and the old verification billed the full 1.5s
        // horizon PLUS a crawl-speed braking extension (0.15m at 0.1 m/s =
        // 1.5s more) of phantom obstacle propagation + uncertainty
        // inflation against every step — slowest steps billed the most, so
        // nothing verified. With the two-part check (static anticipatory
        // sweep + truthful committed-stop rollout) the turn verifies again.
        let planner = default_planner();
        let cfg = PursuitConfig::default();
        let path = entry_turn_path();

        // Ground-truth static scene: profile-limited speed at the turn.
        let statics = entry_turn_obstacles((0.0, 0.0), 3);
        let spawn = |v0: f64| RobotState {
            x: 0.2,
            y: 0.3,
            theta: 0.0,
            linear_vel: v0,
            angular_vel: 0.0,
        };
        let cmd = planner
            .compute(&spawn(0.8), &path, &statics, 1.5, 0.8)
            .expect("static ground-truth entry turn must verify")
            .command;
        assert!(
            cmd.linear_x >= 0.8,
            "static scene must verify at profile-limited speed, got {}",
            cmd.linear_x
        );

        // Phantom velocity on the tracked cone (the live ingredient): the
        // entry turn must still verify at >= v_turn_min instead of
        // freezing into recovery.
        let phantom = entry_turn_obstacles((-0.3, 0.0), 1);
        for (v0, prev) in [(0.0, 0.0), (0.2, 0.2), (0.3, 0.3)] {
            let cmd = planner
                .compute(&spawn(v0), &path, &phantom, 1.5, prev)
                .unwrap_or_else(|e| {
                    panic!("entry turn must verify with a phantom-velocity cone (v0={v0}): {e}")
                })
                .command;
            assert!(
                cmd.linear_x > 0.0,
                "verified command must move (v0={v0}), got {}",
                cmd.linear_x
            );
            if v0 >= cfg.v_turn_min {
                assert!(
                    cmd.linear_x >= cfg.v_turn_min - 1e-9,
                    "rolling entry must stay at/above v_turn_min (v0={v0}), got {}",
                    cmd.linear_x
                );
            }
        }
        // Harsher phantom aimed near the robot: still no freeze — at least
        // the crawl verifies (its truthful stop is centimeters long).
        let harsh = entry_turn_obstacles((-0.5, 0.2), 1);
        let cmd = planner
            .compute(&spawn(0.2), &path, &harsh, 1.5, 0.2)
            .expect("harsh phantom must not freeze the entry turn")
            .command;
        assert!(cmd.linear_x > 0.0);

        // Safety contract control: a genuinely imminent mover — inside the
        // committed-stop envelope of every ladder step — is still rejected,
        // and the deferral names it with the committed-stop phase.
        let charging = vec![Obstacle {
            x: 1.2,
            y: 0.3,
            vx: -0.9,
            vy: 0.0,
            radius: 0.2,
            ..Default::default()
        }];
        let err = planner
            .compute(
                &at_speed(0.3),
                &straight_path(10.0, 0.1),
                &charging,
                0.5,
                0.3,
            )
            .expect_err("an imminent approaching obstacle must still block");
        let PursuitDefer::Blocked(Some(step)) = err else {
            panic!("expected Blocked with diagnostics, got {err:?}");
        };
        assert_eq!(step.phase, BlockedPhase::CommittedStop);
        assert!((step.obs_x - 1.2).abs() < 1e-9 && step.obs_speed > 0.8);
        assert!(step.net_clearance < step.required);
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
        // A non-positive turning-speed floor re-enables curvature stalls.
        for bad in [0.0, -0.1, f64::NAN] {
            assert!(PursuitConfig {
                v_turn_min: bad,
                ..Default::default()
            }
            .validate()
            .is_err());
        }
        // Negative / non-finite curvature-lookahead gain rejected; 0 is the
        // legal speed-only opt-out.
        for bad in [-0.1, f64::NAN, f64::INFINITY] {
            assert!(PursuitConfig {
                curvature_lookahead_gain: bad,
                ..Default::default()
            }
            .validate()
            .is_err());
        }
        assert!(PursuitConfig {
            curvature_lookahead_gain: 0.0,
            ..Default::default()
        }
        .validate()
        .is_ok());
    }
}
