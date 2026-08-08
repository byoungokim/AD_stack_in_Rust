/// Path-tracking MPC: joint speed + curvature optimization over a receding
/// horizon, the PRIMARY executor for forward runs.
///
/// Pure pursuit commits to ONE circular arc per cycle — its convergence onto
/// the reference is a geometric limit cycle (live: sinusoidal weave with
/// ~0.5m lateral amplitude down the whole lane), and its speed planning is
/// decoupled from steering, so exact stops land wherever the step-down
/// happens to end. This controller instead samples candidate CONTROL
/// SEQUENCES (two-phase curvature × three accel profiles), forward-integrates
/// each, and scores them on:
///   - lateral offset to the run polyline (squared, per step) — the user
///     requirement: offset stays minimal except when avoidance demands it
///     (avoidance lives upstream in the corridor/global planner; candidates
///     that collide are hard-rejected here),
///   - heading error against the local path tangent (damps the weave),
///   - tracking of a reference speed profile that ramps to ZERO exactly at
///     the stop point (goal or cusp) at the braking limit — speed planning
///     and steering are decided together,
///   - curvature-change effort (smoothness).
///
/// Candidates are collision-checked per sub-step against velocity-propagated,
/// uncertainty-inflated obstacles with the same speed-scaled clearance
/// requirement DWA and pursuit use, plus a braking extension from the horizon
/// end — an accepted command commits the robot only to space it has verified
/// it can stop inside.
///
/// The first (v, w) of the best feasible candidate is emitted. Reverse runs,
/// cusp switching, and the endgame inside `finish_m` of the run end remain
/// pure pursuit's (its cusp/arrival machinery is battle-tested); an
/// infeasible window (every candidate collides) returns None and the caller
/// falls through the existing pursuit → DWA → recovery hierarchy.
use serde::Deserialize;

use super::dwa::{speed_scaled_radius, DwaConfig, ROBOT_FOOTPRINT_RADIUS};
use super::{Obstacle, RobotState, TrajPoint, VelocityCommand};
use crate::global_planner::smoother::menger_curvature;
use crate::global_planner::PathWaypoint;

#[derive(Debug, Clone, Deserialize)]
pub struct TrackingMpcConfig {
    /// Master switch: false = pursuit-primary hierarchy, exactly as before.
    #[serde(default = "default_enabled")]
    pub enabled: bool,
    /// Horizon steps (of `dt` each).
    #[serde(default = "default_horizon_steps")]
    pub horizon_steps: usize,
    /// Horizon step (s).
    #[serde(default = "default_dt")]
    pub dt: f64,
    /// First-phase duration (s): candidates hold κ1 this long, then blend
    /// toward the path-tangent feedback curvature for the remainder.
    #[serde(default = "default_phase1_s")]
    pub phase1_s: f64,
    /// Lateral-offset weight (per m², per step).
    #[serde(default = "default_w_lat")]
    pub w_lat: f64,
    /// Heading-error weight (per rad², per step).
    #[serde(default = "default_w_head")]
    pub w_head: f64,
    /// Speed-tracking weight (per (m/s)², per step).
    #[serde(default = "default_w_speed")]
    pub w_speed: f64,
    /// Curvature-change weight (per (1/m)², per step pair).
    #[serde(default = "default_w_dkappa")]
    pub w_dkappa: f64,
    /// Cycle-to-cycle curvature continuity weight (per (1/m)², once per
    /// candidate): near-tied candidates must not flip steer direction
    /// between cycles — the executed motion read as erratic heading churn
    /// that other actors cannot predict. Anchors each cycle's phase-1
    /// curvature to the previously chosen one.
    #[serde(default = "default_w_kappa_cont")]
    pub w_kappa_cont: f64,
    /// Hand the run end back to pursuit inside this distance (m): its cusp
    /// arrival / terminal machinery owns the last half meter.
    #[serde(default = "default_finish_m")]
    pub finish_m: f64,
    /// Lateral-acceleration cap (m/s²) of the anticipatory reference speed
    /// profile: per-vertex curvature caps v ≤ sqrt(a_lat_max/|κ|), then a
    /// backward pass at comfort deceleration so the robot brakes BEFORE
    /// every bend (replay t=3.5s: without this it flew into the 90° entry
    /// turn under full acceleration and overshot into a reverse).
    #[serde(default = "default_mpc_a_lat_max")]
    pub a_lat_max: f64,
}

fn default_enabled() -> bool {
    true
}
fn default_horizon_steps() -> usize {
    14
}
fn default_dt() -> f64 {
    0.1
}
fn default_phase1_s() -> f64 {
    0.4
}
fn default_w_lat() -> f64 {
    6.0
}
fn default_w_head() -> f64 {
    1.5
}
fn default_w_speed() -> f64 {
    1.0
}
fn default_w_dkappa() -> f64 {
    // 0.6: at 0.3 the smoothness term lost to lateral/heading gains often
    // enough that steering flipped visibly between cycles.
    0.6
}
fn default_w_kappa_cont() -> f64 {
    0.5
}
fn default_finish_m() -> f64 {
    0.5
}
fn default_mpc_a_lat_max() -> f64 {
    1.5
}

impl Default for TrackingMpcConfig {
    fn default() -> Self {
        Self {
            enabled: default_enabled(),
            horizon_steps: default_horizon_steps(),
            dt: default_dt(),
            phase1_s: default_phase1_s(),
            w_lat: default_w_lat(),
            w_head: default_w_head(),
            w_speed: default_w_speed(),
            w_dkappa: default_w_dkappa(),
            w_kappa_cont: default_w_kappa_cont(),
            finish_m: default_finish_m(),
            a_lat_max: default_mpc_a_lat_max(),
        }
    }
}

impl TrackingMpcConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.horizon_steps == 0 || self.horizon_steps > 100 {
            return Err(format!(
                "tracking_mpc.horizon_steps must be in 1..=100, got {}",
                self.horizon_steps
            ));
        }
        for (name, v) in [
            ("dt", self.dt),
            ("phase1_s", self.phase1_s),
            ("finish_m", self.finish_m),
            ("a_lat_max", self.a_lat_max),
        ] {
            if !(v > 0.0 && v.is_finite()) {
                return Err(format!("tracking_mpc.{name} must be > 0, got {v}"));
            }
        }
        for (name, v) in [
            ("w_lat", self.w_lat),
            ("w_head", self.w_head),
            ("w_speed", self.w_speed),
            ("w_dkappa", self.w_dkappa),
            ("w_kappa_cont", self.w_kappa_cont),
        ] {
            if !(v >= 0.0 && v.is_finite()) {
                return Err(format!("tracking_mpc.{name} must be >= 0, got {v}"));
            }
        }
        Ok(())
    }
}

/// First-phase curvature offsets sampled around the feedback seed, as
/// fractions of the executable envelope.
const KAPPA_OFFSETS: [f64; 7] = [-0.5, -0.25, -0.1, 0.0, 0.1, 0.25, 0.5];

/// Second-phase path-tangent feedback gains (see `feedback_kappa`): lateral
/// and heading gains of the small-signal tracking law used to seed and to
/// steer the tail of each candidate. k_head = 2·sqrt(k_lat) → critical
/// damping of the (e, ψ) error dynamics at unit speed.
const K_LAT: f64 = 3.0;
const K_HEAD: f64 = 3.46;

/// Bound on the heading-error term of the feedback law (rad): the law is a
/// small-signal design; beyond this the pursuit fallback (or the phase-1
/// sample spread) owns the gross reorientation.
const FEEDBACK_PSI_CLAMP: f64 = 0.8;

/// Braking-extension reaction margin (m), matching DWA's assumption.
const BRAKING_REACTION_MARGIN_M: f64 = 0.15;

/// Fraction of the physical deceleration limit used by the reference stop
/// ramp (see `v_ref`).
const COMFORT_DECEL_FACTOR: f64 = 0.7;

/// Sub-step collision spacing (m), matching DWA's assumption.
const COLLISION_CHECK_SPACING_M: f64 = 0.05;

/// Speed floor (m/s) of the curvature caps in the reference profile: tight
/// bends are rolled through at least this fast, never stalled on.
const PROFILE_V_FLOOR: f64 = 0.2;

pub struct TrackingMpc {
    config: TrackingMpcConfig,
    dwa: DwaConfig,
    /// Phase-1 curvature chosen last cycle (the continuity anchor).
    prev_kappa1: Option<f64>,
}

/// Projection of a point onto the run polyline.
struct Projection {
    /// Signed lateral offset (m): positive = left of the travel direction.
    e_lat: f64,
    /// Path tangent (world frame) at the projection.
    tangent: f64,
    /// Arc distance from the projection to the run end (m).
    s_remaining: f64,
}

fn project(run: &[PathWaypoint], x: f64, y: f64) -> Option<Projection> {
    if run.len() < 2 {
        return None;
    }
    // Cumulative lengths from each vertex to the end, one reverse pass.
    let mut best: Option<(f64, usize, f64)> = None; // (d2, seg index, t)
    for (i, w) in run.windows(2).enumerate() {
        let (ax, ay) = (w[0].x, w[0].y);
        let (bx, by) = (w[1].x, w[1].y);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 > 1e-12 {
            (((x - ax) * dx + (y - ay) * dy) / len2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (px, py) = (ax + t * dx, ay + t * dy);
        let d2 = (x - px).powi(2) + (y - py).powi(2);
        if best.is_none_or(|(bd2, _, _)| d2 < bd2) {
            best = Some((d2, i, t));
        }
    }
    let (_, seg, t) = best?;
    let (ax, ay) = (run[seg].x, run[seg].y);
    let (bx, by) = (run[seg + 1].x, run[seg + 1].y);
    let (dx, dy) = (bx - ax, by - ay);
    let seg_len = (dx * dx + dy * dy).sqrt();
    if seg_len < 1e-9 {
        return None;
    }
    let tangent = dy.atan2(dx);
    // Signed lateral: cross(segment dir, robot - proj).
    let (px, py) = (ax + t * dx, ay + t * dy);
    let e_lat = (dx * (y - py) - dy * (x - px)) / seg_len;
    let mut s_remaining = (1.0 - t) * seg_len;
    for w in run.windows(2).skip(seg + 1) {
        s_remaining += ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
    }
    Some(Projection {
        e_lat,
        tangent,
        s_remaining,
    })
}

fn wrap(a: f64) -> f64 {
    let mut a = a % (2.0 * std::f64::consts::PI);
    if a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    } else if a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Arc-length frame over the run polyline plus the anticipatory speed
/// profile: cumulative distance, per-segment tangent, and a per-vertex
/// speed cap (curvature-limited via a_lat_max, backward-propagated at the
/// comfort deceleration, zero at the end when the run is a stop point).
/// The Frenet reference for the quintic candidates; the arc family uses
/// the same profile so both brake before bends identically.
struct RefFrame {
    cum_s: Vec<f64>,
    /// Per-SEGMENT tangent (len n-1).
    tangent: Vec<f64>,
    total: f64,
    v_cap: Vec<f64>,
}

impl RefFrame {
    fn build(
        run: &[PathWaypoint],
        leg_speed: f64,
        a_lat_max: f64,
        comfort_decel: f64,
        stop_at_end: bool,
    ) -> Option<Self> {
        let n = run.len();
        if n < 2 {
            return None;
        }
        let xs: Vec<f64> = run.iter().map(|w| w.x).collect();
        let ys: Vec<f64> = run.iter().map(|w| w.y).collect();
        let mut cum_s = vec![0.0; n];
        let mut tangent = vec![0.0; n - 1];
        for i in 0..n - 1 {
            let (dx, dy) = (xs[i + 1] - xs[i], ys[i + 1] - ys[i]);
            cum_s[i + 1] = cum_s[i] + (dx * dx + dy * dy).sqrt();
            tangent[i] = dy.atan2(dx);
        }
        let total = cum_s[n - 1];
        if total < 1e-6 {
            return None;
        }
        // Curvature caps per vertex, then the backward comfort-braking pass.
        let mut v_cap = vec![leg_speed; n];
        for i in 1..n - 1 {
            let k = menger_curvature(
                (xs[i - 1], ys[i - 1]),
                (xs[i], ys[i]),
                (xs[i + 1], ys[i + 1]),
            );
            if k.abs() > 1e-6 {
                v_cap[i] = v_cap[i].min((a_lat_max / k.abs()).sqrt().max(PROFILE_V_FLOOR));
            }
        }
        if stop_at_end {
            v_cap[n - 1] = 0.0;
        }
        for i in (0..n - 1).rev() {
            let ds = cum_s[i + 1] - cum_s[i];
            v_cap[i] = v_cap[i].min((v_cap[i + 1].powi(2) + 2.0 * comfort_decel * ds).sqrt());
        }
        Some(Self {
            cum_s,
            tangent,
            total,
            v_cap,
        })
    }

    /// Segment index containing arc length `s` (clamped).
    fn seg_at(&self, s: f64) -> usize {
        let s = s.clamp(0.0, self.total);
        match self
            .cum_s
            .binary_search_by(|v| v.partial_cmp(&s).unwrap_or(std::cmp::Ordering::Equal))
        {
            Ok(i) => i.min(self.tangent.len() - 1),
            Err(i) => (i.saturating_sub(1)).min(self.tangent.len() - 1),
        }
    }

    /// Anticipatory speed cap at arc length `s` (linear interpolation).
    fn cap_at(&self, s: f64) -> f64 {
        let s = s.clamp(0.0, self.total);
        let i = self.seg_at(s);
        let seg_len = (self.cum_s[i + 1] - self.cum_s[i]).max(1e-9);
        let t = ((s - self.cum_s[i]) / seg_len).clamp(0.0, 1.0);
        self.v_cap[i] + t * (self.v_cap[i + 1] - self.v_cap[i])
    }
}

impl TrackingMpc {
    pub fn new(config: TrackingMpcConfig, dwa: DwaConfig) -> Self {
        Self {
            config,
            dwa,
            prev_kappa1: None,
        }
    }

    /// Path-tangent feedback curvature at (x, y, θ): the small-signal law
    /// κ = −k_lat·e − k_head·ψ (steer right when left of / rotated left of
    /// the reference), clamped to the executable envelope. Used to seed the
    /// phase-1 samples and to steer every candidate's phase-2 tail.
    fn feedback_kappa(&self, run: &[PathWaypoint], x: f64, y: f64, theta: f64) -> f64 {
        let Some(p) = project(run, x, y) else {
            return 0.0;
        };
        let psi = wrap(theta - p.tangent).clamp(-FEEDBACK_PSI_CLAMP, FEEDBACK_PSI_CLAMP);
        (-K_LAT * p.e_lat - K_HEAD * psi).clamp(-self.dwa.max_curvature, self.dwa.max_curvature)
    }

    /// Compute the next command. None: infeasible window or out-of-scope
    /// geometry (caller falls through to pursuit → DWA → recovery).
    ///
    /// `stop_at_end`: the run end is a genuine stop point (final mission run
    /// or cusp) rather than a truncation artifact.
    pub fn compute(
        &mut self,
        state: &RobotState,
        run: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
        prev_cmd_v: f64,
        stop_at_end: bool,
    ) -> Option<(VelocityCommand, Vec<TrajPoint>)> {
        if !self.config.enabled || run.len() < 2 {
            return None;
        }
        // Still rolling AGAINST the run direction (braking through a cusp
        // that main.rs truncation already flipped): pursuit's direction
        // guard owns the stop ramp — commanding forward here would jump the
        // accel envelope.
        if prev_cmd_v < -1e-6 || state.linear_vel < -0.05 {
            return None;
        }
        let proj = project(run, state.x, state.y)?;
        if proj.s_remaining < self.config.finish_m {
            return None; // pursuit's arrival machinery owns the endgame
        }
        let leg_speed = desired_speed.clamp(0.0, self.dwa.max_speed);
        if leg_speed <= 0.0 {
            return None;
        }

        let dt = self.config.dt;
        let steps = self.config.horizon_steps;
        let phase1_steps = ((self.config.phase1_s / dt).round() as usize).clamp(1, steps);
        let v0 = prev_cmd_v.max(0.0).max(state.linear_vel.max(0.0));

        // Reference frame + anticipatory speed profile (curvature-braking
        // and terminal stop encoded once, shared by both families).
        let frame = RefFrame::build(
            run,
            leg_speed,
            self.config.a_lat_max,
            COMFORT_DECEL_FACTOR * self.dwa.max_deceleration,
            stop_at_end,
        )?;

        // Two-phase constant-curvature arc candidates.
        let seed = self.feedback_kappa(run, state.x, state.y, state.theta);
        let mut best: Option<(f64, VelocityCommand, f64, Vec<TrajPoint>)> = None;
        for offset in KAPPA_OFFSETS {
            let kappa1 = (seed + offset * self.dwa.max_curvature)
                .clamp(-self.dwa.max_curvature, self.dwa.max_curvature);
            // Cycle-to-cycle continuity: anchor the candidate's phase-1
            // curvature to last cycle's winner so near-ties do not flip the
            // steer direction (erratic, unpredictable heading churn).
            let cont = self
                .prev_kappa1
                .map_or(0.0, |pk| self.config.w_kappa_cont * (kappa1 - pk).powi(2));
            // Accel profiles: push toward the reference, hold, brake.
            for accel in [self.dwa.max_acceleration, 0.0, -self.dwa.max_deceleration] {
                if let Some((cost, first, traj)) = self.rollout(
                    state,
                    run,
                    obstacles,
                    &frame,
                    v0,
                    kappa1,
                    accel,
                    dt,
                    steps,
                    phase1_steps,
                ) {
                    let total = cost + cont;
                    if best.as_ref().is_none_or(|(c, _, _, _)| total < *c) {
                        best = Some((total, first, kappa1, traj));
                    }
                }
            }
        }
        match best {
            Some((_, cmd, k1, traj)) => {
                self.prev_kappa1 = Some(k1);
                Some((cmd, traj))
            }
            None => {
                self.prev_kappa1 = None;
                None
            }
        }
    }

    /// Integrate one candidate; returns (cost, first-step command) or None
    /// when any step collides or leaves the executable envelope.
    #[allow(clippy::too_many_arguments)]
    fn rollout(
        &self,
        state: &RobotState,
        run: &[PathWaypoint],
        obstacles: &[Obstacle],
        frame: &RefFrame,
        v0: f64,
        kappa1: f64,
        accel1: f64,
        dt: f64,
        steps: usize,
        phase1_steps: usize,
    ) -> Option<(f64, VelocityCommand, Vec<TrajPoint>)> {
        let (mut x, mut y, mut theta, mut v) = (state.x, state.y, state.theta, v0);
        let mut cost = 0.0;
        let mut prev_kappa = kappa1;
        let mut first: Option<VelocityCommand> = None;
        let mut t = 0.0;
        let (mut last_v, mut last_kappa) = (v0, kappa1);
        let mut traj: Vec<TrajPoint> = Vec::with_capacity(steps);

        for i in 0..steps {
            let proj = project(run, x, y)?;
            let vr = frame.cap_at(frame.total - proj.s_remaining);
            // Speed update: phase-1 candidates apply their accel; the tail
            // tracks the reference profile at the accel limit. Never exceed
            // the reference (the profile already encodes the stop ramp and
            // the anticipatory cap is enforced by the caller's leg_speed).
            let a = if i < phase1_steps {
                accel1
            } else {
                ((vr - v) / dt).clamp(-self.dwa.max_deceleration, self.dwa.max_acceleration)
            };
            // Cap at the reference, but never demand a drop steeper than the
            // deceleration limit — a hard clamp to vr broke the accel
            // envelope when the reference stepped down.
            let prev_step_v = v;
            let cap = vr.max(v - self.dwa.max_deceleration * dt).max(0.0);
            v = (v + a * dt).clamp(0.0, cap);
            // HARD stop feasibility: above the comfort stop ramp, a step is
            // feasible ONLY while braking at the full physical rate. Without
            // this, "cruise now, brake in the tail" stayed feasible every
            // cycle and the receding horizon procrastinated braking to the
            // last moment (live: coasted to the handoff at 1.64 m/s, needing
            // 2.7 m/s² to avoid overrunning the stop point).
            // (The reference now also dips before BENDS, so this constraint
            // additionally forces on-time braking into curvature.)
            if v > vr + 1e-6 && v > prev_step_v - self.dwa.max_deceleration * dt * 0.99 + 1e-9 {
                return None;
            }
            // Curvature: phase-1 holds κ1, the tail follows the feedback law
            // re-evaluated at the predicted pose (piecewise arcs — this is
            // what a single pursuit circle cannot represent).
            let kappa = if i < phase1_steps {
                kappa1
            } else {
                self.feedback_kappa(run, x, y, theta)
            };
            let w = (kappa * v).clamp(-self.dwa.max_angular_speed, self.dwa.max_angular_speed);
            if first.is_none() {
                first = Some(VelocityCommand {
                    linear_x: v,
                    angular_z: w,
                    confidence: 0.9,
                });
            }

            // Sub-stepped integration + collision check (DWA-equivalent).
            let n_sub = ((v * dt / COLLISION_CHECK_SPACING_M).ceil() as usize).max(1);
            let sub = dt / n_sub as f64;
            let required = speed_scaled_radius(
                v,
                self.dwa.robot_radius,
                self.dwa.margin_low_speed_scale,
                self.dwa.high_speed_margin_gain,
            );
            for _ in 0..n_sub {
                x += v * theta.cos() * sub;
                y += v * theta.sin() * sub;
                theta += w * sub;
                t += sub;
                if !clear_at(
                    obstacles,
                    x,
                    y,
                    t,
                    required,
                    self.dwa.moving_obstacle_margin_gain,
                ) {
                    return None;
                }
            }

            let proj2 = project(run, x, y)?;
            let psi = wrap(theta - proj2.tangent);
            cost += self.config.w_lat * proj2.e_lat * proj2.e_lat
                + self.config.w_head * psi * psi
                + self.config.w_speed * (v - vr) * (v - vr)
                + self.config.w_dkappa * (kappa - prev_kappa) * (kappa - prev_kappa);
            prev_kappa = kappa;
            (last_v, last_kappa) = (v, kappa);
            traj.push(TrajPoint { x, y, theta });
        }

        // Braking extension: from the horizon end, a reaction margin at the
        // final speed then a max-deceleration stop along the final curvature
        // must stay clear — the sequence commits only to verified space.
        if last_v > 1e-6 {
            let mut v = last_v;
            let w = (last_kappa * v).clamp(-self.dwa.max_angular_speed, self.dwa.max_angular_speed);
            let mut travel = -BRAKING_REACTION_MARGIN_M;
            let required = speed_scaled_radius(
                last_v,
                self.dwa.robot_radius,
                self.dwa.margin_low_speed_scale,
                self.dwa.high_speed_margin_gain,
            );
            while v > 1e-3 {
                let sub = (COLLISION_CHECK_SPACING_M / v).min(0.05);
                x += v * theta.cos() * sub;
                y += v * theta.sin() * sub;
                theta += w * sub;
                t += sub;
                travel += v * sub;
                if travel > 0.0 {
                    v = (v - self.dwa.max_deceleration * sub).max(0.0);
                }
                if !clear_at(
                    obstacles,
                    x,
                    y,
                    t,
                    required,
                    self.dwa.moving_obstacle_margin_gain,
                ) {
                    return None;
                }
            }
        }

        first.map(|f| (cost, f, traj))
    }
}

/// Clearance check at (x, y) and lookahead time `t` against every obstacle,
/// velocity-propagated and uncertainty-inflated, with the caller's
/// speed-scaled requirement — identical semantics to DWA's sample filter.
fn clear_at(
    obstacles: &[Obstacle],
    x: f64,
    y: f64,
    t: f64,
    required: f64,
    moving_margin_gain: f64,
) -> bool {
    debug_assert!(required >= ROBOT_FOOTPRINT_RADIUS);
    for obs in obstacles {
        let d = obs.net_distance_at(x, y, t) - moving_margin_gain * obs.speed();
        if d < required {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::global_planner::SegmentDir;

    fn straight_run(len_m: f64, spacing: f64) -> Vec<PathWaypoint> {
        let n = (len_m / spacing) as usize;
        (0..=n)
            .map(|i| PathWaypoint {
                x: spacing * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Forward,
            })
            .collect()
    }

    fn mpc() -> TrackingMpc {
        TrackingMpc::new(TrackingMpcConfig::default(), DwaConfig::default())
    }

    /// Closed-loop: integrate compute() at 10Hz with a kinematic unicycle.
    /// Returns (max overshoot past the reference after the first crossing,
    /// final state, final commanded v).
    fn simulate(
        mut state: RobotState,
        run: &[PathWaypoint],
        cycles: usize,
    ) -> (f64, RobotState, f64) {
        let mut mpc = mpc();
        let mut prev_v = state.linear_vel;
        let mut crossed = state.y < 0.0;
        let mut overshoot: f64 = 0.0;
        for _ in 0..cycles {
            let Some((cmd, _)) = mpc.compute(&state, run, &[], 2.0, prev_v, true) else {
                break;
            };
            let (v, w) = (cmd.linear_x, cmd.angular_z);
            for _ in 0..5 {
                state.x += v * state.theta.cos() * 0.02;
                state.y += v * state.theta.sin() * 0.02;
                state.theta += w * 0.02;
            }
            state.linear_vel = v;
            state.angular_vel = w;
            prev_v = v;
            if state.y < 0.0 {
                crossed = true;
            }
            if crossed {
                overshoot = overshoot.max(-state.y);
            }
        }
        (overshoot, state, prev_v)
    }

    #[test]
    fn test_mpc_converges_with_small_overshoot() {
        // 1m left of a straight reference at speed: the tracked convergence
        // must cross with bounded overshoot — the pursuit limit cycle this
        // controller replaces oscillated at ~0.5m amplitude indefinitely.
        let state = RobotState {
            x: 0.0,
            y: 1.0,
            theta: 0.0,
            linear_vel: 1.2,
            angular_vel: 0.0,
        };
        let run = straight_run(20.0, 0.25);
        let (overshoot, end, _) = simulate(state, &run, 80);
        assert!(
            overshoot < 0.15,
            "overshoot {overshoot:.3}m exceeds the 0.15m tracking bound"
        );
        assert!(
            end.y.abs() < 0.12,
            "must settle onto the reference, ended at e={:.3}",
            end.y
        );
        assert!(
            end.x > 4.0,
            "must make progress along the run, x={:.2}",
            end.x
        );
    }

    #[test]
    fn test_mpc_stops_at_run_end() {
        // Rolling at 1.5 m/s toward a 4m run end that is a genuine stop
        // point: the speed profile must ramp the command to (near) zero by
        // the run end — the naive step-down stopped wherever it happened to.
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 1.5,
            angular_vel: 0.0,
        };
        let run = straight_run(4.0, 0.25);
        let (_, end, last_v) = simulate(state, &run, 100);
        // MPC hands the last finish_m (0.5m) to pursuit; within its own
        // authority it must have (a) not overrun the stop point, (b) begun
        // braking, and (c) arrived at the handoff no hotter than the
        // PHYSICAL braking envelope evaluated just outside the handoff —
        // pursuit's endgame can then stop inside the remaining distance.
        assert!(
            end.x <= 4.0 + 0.05,
            "must not overrun the stop point, x={:.2}",
            end.x
        );
        assert!(end.x >= 3.0, "must reach the handoff zone, x={:.2}", end.x);
        let dec = DwaConfig::default().max_deceleration;
        let envelope = (2.0 * dec * 0.55).sqrt();
        assert!(
            last_v <= envelope + 0.05,
            "handoff speed {last_v:.2} exceeds the physical stop envelope {envelope:.2}"
        );
        assert!(last_v < 1.9, "braking must have begun (v={last_v:.2})");
    }

    #[test]
    fn test_mpc_rejects_blocked_window_and_respects_disable() {
        // A wall dead ahead inside the horizon: every candidate collides →
        // None (the caller falls through to pursuit/DWA/recovery).
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 1.0,
            angular_vel: 0.0,
        };
        let run = straight_run(6.0, 0.25);
        let mut wall = Vec::new();
        let mut yy = -1.0;
        while yy <= 1.0 {
            wall.push(Obstacle::point(0.5, yy));
            yy += 0.05;
        }
        assert!(mpc().compute(&state, &run, &wall, 1.5, 1.0, true).is_none());

        // Disabled config: always None regardless of scene.
        let mut off = TrackingMpc::new(
            TrackingMpcConfig {
                enabled: false,
                ..Default::default()
            },
            DwaConfig::default(),
        );
        assert!(off.compute(&state, &run, &[], 1.5, 1.0, true).is_none());
    }

    #[test]
    fn test_config_validation() {
        assert!(TrackingMpcConfig::default().validate().is_ok());
        assert!(TrackingMpcConfig {
            horizon_steps: 0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(TrackingMpcConfig {
            dt: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(TrackingMpcConfig {
            w_lat: -1.0,
            ..Default::default()
        }
        .validate()
        .is_err());
    }
}
