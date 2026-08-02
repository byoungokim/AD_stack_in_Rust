/// Dynamic Window Approach (DWA) local planner.
///
/// Samples velocity pairs (v, w) within a dynamic window of feasible
/// accelerations, simulates forward, scores trajectories, selects best.
/// Runs at 10Hz — very lightweight for Orin Nano.
use serde::Deserialize;

use super::{Obstacle, RobotState, VelocityCommand};
use crate::global_planner::PathWaypoint;

#[derive(Debug, Clone, Deserialize)]
pub struct DwaConfig {
    // Velocity limits
    #[serde(default = "default_max_speed")]
    pub max_speed: f64, // m/s
    #[serde(default = "default_max_angular")]
    pub max_angular_speed: f64, // rad/s
    #[serde(default = "default_max_accel")]
    pub max_acceleration: f64, // m/s^2
    /// Braking deceleration (m/s^2) assumed by the braking-aware feasibility
    /// check. Must not exceed the arbitrator envelope's max_deceleration —
    /// DWA would then assume stopping power the envelope never grants.
    #[serde(default = "default_max_decel")]
    pub max_deceleration: f64, // m/s^2
    #[serde(default = "default_max_angular_accel")]
    pub max_angular_accel: f64, // rad/s^2

    // Simulation
    #[serde(default = "default_sim_time")]
    pub sim_time: f64, // seconds to simulate forward
    #[serde(default = "default_sim_dt")]
    pub sim_dt: f64, // simulation time step
    #[serde(default = "default_v_samples")]
    pub v_samples: usize, // number of linear velocity samples
    #[serde(default = "default_w_samples")]
    pub w_samples: usize, // number of angular velocity samples

    // Scoring weights
    #[serde(default = "default_heading_weight")]
    pub heading_weight: f64,
    #[serde(default = "default_distance_weight")]
    pub distance_weight: f64,
    #[serde(default = "default_velocity_weight")]
    pub velocity_weight: f64,
    #[serde(default = "default_obstacle_weight")]
    pub obstacle_weight: f64,
    /// Command-continuity term: penalizes |v - v_prev| (normalized by
    /// max_speed) so the chosen velocity doesn't jump cycle-to-cycle when
    /// several corridors score nearly equally. Modest by design — it breaks
    /// ties, it must not override obstacle or heading preferences.
    #[serde(default = "default_continuity_weight")]
    pub weight_continuity: f64,

    // Safety
    #[serde(default = "default_robot_radius")]
    pub robot_radius: f64, // meters, for collision checking
    /// Recovery phase A margin relaxation: the margin ABOVE the raw robot
    /// footprint (robot_radius - ROBOT_FOOTPRINT_RADIUS) is scaled by this
    /// factor when retrying a stuck plan, never shrinking below the footprint
    /// itself. 1.0 = no relaxation.
    #[serde(default = "default_recovery_margin_scale")]
    pub recovery_margin_scale: f64,
    /// Speed-dependent margin: fraction of the tunable margin (collision
    /// radius above the physical footprint) required at crawl speeds
    /// (≤ 0.1 m/s), lerping up to the full margin at ≥ 0.3 m/s. Slow motion
    /// is allowed closer to obstacles (little kinetic energy, latency error
    /// small); cruise motion is pushed further out. 1.0 = no speed scaling.
    #[serde(default = "default_margin_low_speed_scale")]
    pub margin_low_speed_scale: f64,

    /// Extra required clearance per m/s of OBSTACLE speed (seconds): a moving
    /// obstacle demands `gain × |obstacle velocity|` meters beyond the
    /// robot's own speed-scaled radius. The speed-scaled margin above scales
    /// only with ROBOT speed, so a crawling robot was allowed as close to a
    /// 0.75 m/s pedestrian as to a cone (measured -0.030m ground-truth
    /// clearance brushing a crossing pedestrian at crawl). At the default
    /// 0.4, that pedestrian adds 0.30m; static obstacles (|v| ≈ 0) add
    /// nothing, preserving crawl relief in tight static gaps. Applied
    /// identically in DWA sample checks, standstill-escape profiles, the
    /// obstacle-distance scoring term (so scoring and feasibility agree),
    /// and the recovery reverse swept check.
    #[serde(default = "default_moving_obstacle_margin_gain")]
    pub moving_obstacle_margin_gain: f64,

    /// High-speed margin growth (m per m/s above 1.0 m/s of ROBOT speed):
    /// extends the speed-dependent margin ABOVE cruise. The crawl→cruise lerp
    /// tops out at 0.3 m/s, sized for the old 1.0 m/s ceiling; at 2.2 m/s the
    /// braking distance is ~0.8m and perception latency eats real distance,
    /// so samples above 1.0 m/s require an extra
    /// `high_speed_margin_gain × (v − 1.0)` of clearance on top of the full
    /// margin (~+0.07m at 2.2 with the default 0.06). 0.0 disables.
    #[serde(default = "default_high_speed_margin_gain")]
    pub high_speed_margin_gain: f64,

    /// Executable-curvature envelope (1/m): sampled (v, w) pairs with
    /// |w| > v * max_curvature are skipped. Must not exceed the arbitrator's
    /// safety-envelope max_curvature nor the Ackermann steering limit
    /// tan(max_steering)/wheelbase (2.6 for the Limo Pro) — otherwise DWA
    /// verifies a trajectory, downstream clamps the command, and the real
    /// arc swings wider than the verified one, clipping the inside of turns.
    #[serde(default = "default_max_curvature")]
    pub max_curvature: f64, // 1/m
}

// Dynamics chain for the >1.5 m/s gauntlet target: DWA limits match the
// arbitrator safety envelope exactly (2.2 / 2.5 / 4.5) so nothing DWA
// verifies is clamped downstream into an unverified arc, and the curvature
// envelope stays consistent: max_curvature 2.0 × 2.2 m/s = 4.4 rad/s ≤ 4.5.
fn default_max_speed() -> f64 {
    2.2
}
fn default_max_angular() -> f64 {
    4.5
}
fn default_max_accel() -> f64 {
    2.5
}
fn default_max_decel() -> f64 {
    // Matches the arbitrator envelope's max_deceleration.
    3.0
}
fn default_max_angular_accel() -> f64 {
    // Opens the w-window ±0.8 rad/s per 10Hz cycle: the full ±4.5 range is
    // reachable in ~0.6s, fast enough to set up slalom entries at 2.2 m/s.
    8.0
}
fn default_sim_time() -> f64 {
    1.5
}
fn default_sim_dt() -> f64 {
    0.1
}
// Sample counts sized for the larger windows: the one-cycle v-span is now
// 2·max_accel·dt = 0.5 m/s → 13 samples ≈ 0.042 m/s resolution; the w-span is
// 2·max_angular_accel·dt = 1.6 rad/s → 15 samples ≈ 0.114 rad/s (κ ≈ 0.05 1/m
// at 2.2 m/s — finer than the tracker follows). Both counts are deliberately
// ODD so the window midpoint (the current v / current w) lies exactly on the
// sample grid — the continuity term anchors there. 13×15 = 195 pairs, fewer
// than the old 11×21 = 231; the added per-sample cost is the substepping.
fn default_v_samples() -> usize {
    13
}
fn default_w_samples() -> usize {
    15
}
fn default_heading_weight() -> f64 {
    1.0
}
fn default_distance_weight() -> f64 {
    0.5
}
fn default_velocity_weight() -> f64 {
    0.3
}
fn default_obstacle_weight() -> f64 {
    2.0
}
fn default_continuity_weight() -> f64 {
    0.1
}
fn default_recovery_margin_scale() -> f64 {
    0.8
}
fn default_margin_low_speed_scale() -> f64 {
    0.4
}
fn default_robot_radius() -> f64 {
    // Circumscribed radius of the Limo Pro footprint is 0.19. 0.2 left ~1cm
    // of margin, which perception latency eats at corners (measured -0.076m
    // ground-truth clearance cutting a gate edge at 0.3 m/s); 0.25 combined
    // with obstacle persistence froze DWA in 1m gaps; 0.22 was the knife-edge
    // that served both regimes at once. With the speed-scaled margin
    // (`margin_low_speed_scale`) the single number no longer has to: the
    // corner-cutting failure was a cruise-speed failure and the frozen-gap
    // failure a crawl-speed one, so 0.24 pushes the cruise requirement OUT to
    // a real 5cm margin while the crawl requirement drops to ~0.21
    // (0.19 + 0.05·0.4), keeping tight gaps feasible at low speed — the knife
    // edge becomes a band. Ghost eviction of moving-object trails (main.rs)
    // additionally removes the persistence smear that made 0.25 freeze in
    // gaps, so the larger base no longer compounds with phantom fences.
    0.24
}

/// Circumscribed radius of the physical Limo Pro footprint (m). The recovery
/// margin relaxation must never check collisions below this — it is the robot
/// body, not a tunable margin.
pub const ROBOT_FOOTPRINT_RADIUS: f64 = 0.19;

/// Speeds below this (m/s) count as standstill for the committed-acceleration
/// escape (matches the behavior planner's stationary threshold).
const STANDSTILL_ESCAPE_SPEED: f64 = 0.05;

/// Speed at/below which the collision margin is fully scaled down to
/// `margin_low_speed_scale` (m/s). Matches the recovery crawl speed.
const MARGIN_SCALE_LOW_SPEED: f64 = 0.1;
/// Speed at/above which the full collision margin is required (m/s).
const MARGIN_SCALE_FULL_SPEED: f64 = 0.3;
/// Speed (m/s) above which `high_speed_margin_gain` starts growing the margin
/// further (the crawl→cruise lerp was sized for a 1.0 m/s ceiling).
const HIGH_SPEED_MARGIN_START: f64 = 1.0;

/// Maximum distance (m) between consecutive collision checks along a
/// simulated trajectory. Integrating at sim_dt alone strides `v·sim_dt`
/// (22cm at 2.2 m/s) — enough to step clean over a cone between two checks
/// (tunneling). Substepping caps the check spacing at 5cm for any sampled
/// speed.
const COLLISION_CHECK_SPACING_M: f64 = 0.05;

/// Reaction-time distance margin (m) added to the kinematic braking distance
/// v²/(2·max_deceleration) in the braking-aware feasibility check.
const BRAKING_REACTION_MARGIN_M: f64 = 0.15;

/// Substep count so that one `sim_dt` integration step at speed `v` is
/// collision-checked at least every `COLLISION_CHECK_SPACING_M` meters.
fn substeps_for(v: f64, sim_dt: f64) -> usize {
    ((v.abs() * sim_dt / COLLISION_CHECK_SPACING_M).ceil() as usize).max(1)
}

/// Speed-dependent collision radius for a sample at linear speed `v`:
/// the tunable margin above the physical footprint
/// (`base_radius - ROBOT_FOOTPRINT_RADIUS`) is scaled by a factor that lerps
/// from `low_speed_scale` at |v| ≤ 0.1 m/s up to 1.0 at |v| ≥ 0.3 m/s, and
/// above 1.0 m/s grows further by `high_speed_gain × (|v| − 1.0)` (braking
/// distance and perception-latency error both grow with speed). Never returns
/// less than the physical footprint. Used for both DWA sample checks and (at
/// the crawl point) the recovery reverse swept check, so slow motion gets a
/// consistent, tighter-but-real requirement everywhere.
pub fn speed_scaled_radius(
    v: f64,
    base_radius: f64,
    low_speed_scale: f64,
    high_speed_gain: f64,
) -> f64 {
    let margin = (base_radius - ROBOT_FOOTPRINT_RADIUS).max(0.0);
    let t = ((v.abs() - MARGIN_SCALE_LOW_SPEED)
        / (MARGIN_SCALE_FULL_SPEED - MARGIN_SCALE_LOW_SPEED))
        .clamp(0.0, 1.0);
    let scale = (low_speed_scale + (1.0 - low_speed_scale) * t).max(0.0);
    let high_speed_extra = high_speed_gain.max(0.0) * (v.abs() - HIGH_SPEED_MARGIN_START).max(0.0);
    ROBOT_FOOTPRINT_RADIUS + margin * scale + high_speed_extra
}
fn default_moving_obstacle_margin_gain() -> f64 {
    0.4
}
fn default_high_speed_margin_gain() -> f64 {
    0.06
}
fn default_max_curvature() -> f64 {
    // Matches the arbitrator's safety-envelope default and sits inside the
    // Limo Pro's Ackermann limit tan(0.48)/0.2 ≈ 2.6.
    2.0
}

impl Default for DwaConfig {
    fn default() -> Self {
        Self {
            max_speed: default_max_speed(),
            max_angular_speed: default_max_angular(),
            max_acceleration: default_max_accel(),
            max_deceleration: default_max_decel(),
            max_angular_accel: default_max_angular_accel(),
            sim_time: default_sim_time(),
            sim_dt: default_sim_dt(),
            v_samples: default_v_samples(),
            w_samples: default_w_samples(),
            heading_weight: default_heading_weight(),
            distance_weight: default_distance_weight(),
            velocity_weight: default_velocity_weight(),
            obstacle_weight: default_obstacle_weight(),
            weight_continuity: default_continuity_weight(),
            robot_radius: default_robot_radius(),
            recovery_margin_scale: default_recovery_margin_scale(),
            margin_low_speed_scale: default_margin_low_speed_scale(),
            moving_obstacle_margin_gain: default_moving_obstacle_margin_gain(),
            high_speed_margin_gain: default_high_speed_margin_gain(),
            max_curvature: default_max_curvature(),
        }
    }
}

impl DwaConfig {
    /// Fail loudly on YAML values that would break the planner: a zero or
    /// negative deceleration makes the braking-aware check divide by zero
    /// (infinite braking distance), zero samples empties the window, a
    /// non-positive sim step never advances the simulation.
    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [
            ("dwa.max_speed", self.max_speed),
            ("dwa.max_acceleration", self.max_acceleration),
            ("dwa.max_deceleration", self.max_deceleration),
            ("dwa.max_angular_speed", self.max_angular_speed),
            ("dwa.max_angular_accel", self.max_angular_accel),
            ("dwa.sim_time", self.sim_time),
            ("dwa.sim_dt", self.sim_dt),
            ("dwa.robot_radius", self.robot_radius),
            ("dwa.max_curvature", self.max_curvature),
        ] {
            if !(v > 0.0 && v.is_finite()) {
                return Err(format!("{name} must be > 0, got {v}"));
            }
        }
        if self.sim_dt > self.sim_time {
            return Err(format!(
                "dwa.sim_dt ({}) must not exceed dwa.sim_time ({})",
                self.sim_dt, self.sim_time
            ));
        }
        if self.v_samples < 2 || self.w_samples < 2 {
            return Err(format!(
                "dwa.v_samples/w_samples must be >= 2, got {}/{}",
                self.v_samples, self.w_samples
            ));
        }
        for (name, v) in [
            ("dwa.recovery_margin_scale", self.recovery_margin_scale),
            ("dwa.margin_low_speed_scale", self.margin_low_speed_scale),
            (
                "dwa.moving_obstacle_margin_gain",
                self.moving_obstacle_margin_gain,
            ),
            ("dwa.high_speed_margin_gain", self.high_speed_margin_gain),
        ] {
            if !(v >= 0.0 && v.is_finite()) {
                return Err(format!("{name} must be >= 0 and finite, got {v}"));
            }
        }
        Ok(())
    }
}

/// Simulated trajectory for scoring. `min_obstacle_dist` is the worst
/// per-obstacle clearance NET of the dynamic per-obstacle demands
/// (uncertainty-inflated extent and the moving-obstacle margin
/// `moving_obstacle_margin_gain × |v_obs|`), so the same number drives both
/// the feasibility check against the robot's speed-scaled radius and the
/// obstacle-distance scoring term — scoring and feasibility always agree.
struct SimTrajectory {
    v: f64,
    w: f64,
    end_x: f64,
    end_y: f64,
    end_theta: f64,
    min_obstacle_dist: f64,
    /// Worst net clearance along the braking EXTENSION of the trajectory:
    /// continuing the same arc from the horizon endpoint through a reaction
    /// margin plus a max_deceleration stop. The robot must always be able to
    /// stop inside checked-clear space — a horizon that ends closer to an
    /// obstacle than the braking distance is infeasible even if every
    /// horizon pose is collision-free. Infinite when the sample is at rest.
    braking_min_dist: f64,
    score: f64,
}

pub struct DwaPlanner {
    config: DwaConfig,
    /// Linear speed of the previously emitted command, for the continuity
    /// term. None before the first cycle (falls back to the measured speed).
    prev_cmd_v: Option<f64>,
}

impl DwaPlanner {
    pub fn new(config: DwaConfig) -> Self {
        Self {
            config,
            prev_cmd_v: None,
        }
    }

    /// Collision radius for recovery phase A: the margin above the physical
    /// footprint scaled by `recovery_margin_scale`, never below the footprint.
    pub fn relaxed_radius(&self) -> f64 {
        let margin = (self.config.robot_radius - ROBOT_FOOTPRINT_RADIUS).max(0.0);
        ROBOT_FOOTPRINT_RADIUS + margin * self.config.recovery_margin_scale
    }

    pub fn compute(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> VelocityCommand {
        self.compute_with_radius(
            state,
            path,
            obstacles,
            desired_speed,
            self.config.robot_radius,
        )
    }

    /// `compute` with an explicit collision radius — recovery phase A retries
    /// with `relaxed_radius()`. The radius only affects the collision check,
    /// never the scoring weights. Per sample, the effective requirement is
    /// additionally speed-scaled (`speed_scaled_radius`): the relaxation thus
    /// applies to the margin BEFORE speed scaling, and the two compose —
    /// a relaxed crawl checks the smallest radius, a full-speed normal sample
    /// the largest.
    pub fn compute_with_radius(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
        collision_radius: f64,
    ) -> VelocityCommand {
        let target_speed = desired_speed.clamp(0.0, self.config.max_speed);
        // Continuity reference: last emitted command, or the measured speed on
        // the first cycle (no jump from whatever the robot is already doing).
        let prev_v = self.prev_cmd_v.unwrap_or(state.linear_vel);

        // Find the local goal point on the path (lookahead)
        let goal = find_local_goal(state, path);

        // Compute dynamic window: the reachable set [cur - a·dt, cur + a·dt]
        // intersected with the physical limits [0, max_speed]. The target
        // speed caps the window as a preference, but never below the slowest
        // reachable speed — braking from above the target takes several
        // cycles at the accel limit, and clamping the window top below the
        // reachable floor would invert it (v_min > v_max) and make the
        // sample loop run *down* from v_min, which historically rewarded the
        // fastest sample when target_speed hit 0.
        let dt = 1.0 / 10.0; // control cycle
        let reach_min = (state.linear_vel - self.config.max_acceleration * dt).max(0.0);
        let reach_max = (state.linear_vel + self.config.max_acceleration * dt)
            .clamp(0.0, self.config.max_speed);
        let v_max = reach_max.min(target_speed.max(reach_min));
        let v_min = reach_min.min(v_max);
        debug_assert!(v_min <= v_max && v_min >= 0.0);
        let w_min = (state.angular_vel - self.config.max_angular_accel * dt)
            .max(-self.config.max_angular_speed);
        let w_max = (state.angular_vel + self.config.max_angular_accel * dt)
            .min(self.config.max_angular_speed);

        let v_step = if self.config.v_samples > 1 {
            (v_max - v_min) / (self.config.v_samples - 1) as f64
        } else {
            0.0
        };
        let w_step = if self.config.w_samples > 1 {
            (w_max - w_min) / (self.config.w_samples - 1) as f64
        } else {
            0.0
        };

        let mut best: Option<SimTrajectory> = None;
        // Whether ANY collision-free sample actually moved: distinguishes a
        // deadlocked window (everything collides except staying put) from a
        // legitimate choice to stop (e.g. already at the goal).
        let mut motion_feasible = false;

        for vi in 0..self.config.v_samples {
            let v = v_min + vi as f64 * v_step;

            for wi in 0..self.config.w_samples {
                let w = w_min + wi as f64 * w_step;

                // Only verify trajectories the steering can execute: pairs
                // outside the curvature envelope would be clamped downstream
                // and the real arc would differ from the simulated one.
                if w.abs() > v * self.config.max_curvature + 1e-9 {
                    continue;
                }

                // Simulate trajectory
                let traj = self.simulate(state, v, w, obstacles);

                // Skip if collision: the required clearance is speed-scaled —
                // crawl samples may pass closer than cruise samples. The
                // braking extension must ALSO clear: a trajectory that ends
                // closer to an obstacle than its stopping distance commits
                // the robot to space it has not verified (hard reject).
                let required = speed_scaled_radius(
                    v,
                    collision_radius,
                    self.config.margin_low_speed_scale,
                    self.config.high_speed_margin_gain,
                );
                if traj.min_obstacle_dist < required || traj.braking_min_dist < required {
                    continue;
                }
                if v > 1e-9 {
                    motion_feasible = true;
                }

                // Score trajectory
                let heading_score = heading_cost(traj.end_x, traj.end_y, traj.end_theta, &goal);
                let distance_score = distance_to_goal(traj.end_x, traj.end_y, &goal);
                let velocity_score = velocity_score(v, target_speed, self.config.max_speed);
                let obstacle_score = traj.min_obstacle_dist.min(3.0) / 3.0;
                // Continuity: near-tied corridors resolve toward the previous
                // command instead of jumping across the velocity window.
                let continuity_penalty = (v - prev_v).abs() / self.config.max_speed.max(0.1);

                let score = self.config.heading_weight * heading_score
                    - self.config.distance_weight * distance_score
                    + self.config.velocity_weight * velocity_score
                    + self.config.obstacle_weight * obstacle_score
                    - self.config.weight_continuity * continuity_penalty;

                let scored = SimTrajectory { score, ..traj };

                if best.as_ref().is_none_or(|b| scored.score > b.score) {
                    best = Some(scored);
                }
            }
        }

        let mut cmd = match best {
            Some(traj) => VelocityCommand {
                linear_x: traj.v,
                angular_z: traj.w,
                confidence: 0.9,
            },
            None => VelocityCommand {
                linear_x: 0.0,
                angular_z: 0.0,
                confidence: 0.1, // no feasible trajectory
            },
        };

        // Ackermann standstill trap: from v≈0 the one-cycle window tops out
        // at max_accel·dt (~0.05 m/s) and the curvature envelope pins
        // |w| ≤ v·κ_max, so every standard sample crawls ~7cm and turns <9°
        // over the whole horizon. When that window produces no feasible
        // MOTION while we are asked to move, evaluate committed-acceleration
        // escape profiles instead (constant curvature, accelerating at the
        // limit, envelope applied per-step so w grows with v). `motion_feasible`
        // keeps a legitimate stop (feasible motion existed but stopping scored
        // best, e.g. at the goal) from triggering the escape.
        if state.linear_vel.abs() < STANDSTILL_ESCAPE_SPEED
            && !motion_feasible
            && target_speed > 0.0
        {
            if let Some(escape) =
                self.standstill_escape(state, &goal, obstacles, target_speed, collision_radius)
            {
                cmd = escape;
            }
        }

        self.prev_cmd_v = Some(cmd.linear_x);
        cmd
    }

    /// Committed-acceleration escape from standstill: for each curvature
    /// sample, simulate accelerating at `max_acceleration` toward the capped
    /// target speed over the horizon, with the executable-curvature envelope
    /// applied per step (angular speed grows with the linear speed), and
    /// accept the profile only if it is collision-free against the
    /// velocity-propagated, uncertainty-inflated obstacles. The clearance
    /// requirement is speed-scaled per step (the profile accelerates, so the
    /// requirement grows along it). Returns the FIRST-CYCLE command
    /// consistent with the best profile (v = one-cycle acceleration step,
    /// w per the envelope at that v); confidence is mapped from the profile's
    /// worst clearance margin above the per-step requirement.
    ///
    /// Note: for purely static scenes this window is provably no larger than
    /// the standard one (a committed profile's first centimeters trace the
    /// same arc as a constant-(v,w) sample with the same curvature, and
    /// clearance can only shrink further along the arc). Its value is against
    /// MOVING obstacles: a slow sample lingers where an approaching object
    /// (plus its lookahead uncertainty growth) will be, while the committed
    /// profile outruns it. The static frontal trap is what the recovery
    /// reverse is for.
    fn standstill_escape(
        &self,
        state: &RobotState,
        goal: &PathWaypoint,
        obstacles: &[Obstacle],
        target_speed: f64,
        collision_radius: f64,
    ) -> Option<VelocityCommand> {
        let dt = self.config.sim_dt;
        let steps = (self.config.sim_time / dt) as usize;
        if steps == 0 {
            return None;
        }
        let n = self.config.w_samples.max(3);
        // (score, kappa, worst_margin)
        let mut best: Option<(f64, f64, f64)> = None;
        for i in 0..n {
            let kappa = -self.config.max_curvature
                + 2.0 * self.config.max_curvature * i as f64 / (n - 1) as f64;
            let (mut x, mut y, mut theta) = (state.x, state.y, state.theta);
            let mut v = state.linear_vel.max(0.0);
            let mut min_clear = f64::INFINITY;
            // Worst clearance ABOVE the per-step speed-scaled requirement:
            // the profile accelerates, so later steps demand more room.
            let mut worst_margin = f64::INFINITY;
            let mut t = 0.0;
            for _ in 0..steps {
                v = (v + self.config.max_acceleration * dt).min(target_speed);
                // Per-step envelope: w tracks the growing v at this curvature.
                let w = (kappa * v).clamp(
                    -self.config.max_angular_speed,
                    self.config.max_angular_speed,
                );
                let required = speed_scaled_radius(
                    v,
                    collision_radius,
                    self.config.margin_low_speed_scale,
                    self.config.high_speed_margin_gain,
                );
                // Sub-stepped like `simulate`: the profile accelerates toward
                // the (possibly 2.2 m/s) target, so a plain dt stride would
                // tunnel over thin obstacles late in the horizon.
                let n_sub = substeps_for(v, dt);
                let sub_dt = dt / n_sub as f64;
                for _ in 0..n_sub {
                    x += v * theta.cos() * sub_dt;
                    y += v * theta.sin() * sub_dt;
                    theta += w * sub_dt;
                    t += sub_dt;
                    for obs in obstacles {
                        let (ox, oy) = obs.position_at(t);
                        let d = ((x - ox).powi(2) + (y - oy).powi(2)).sqrt()
                            - obs.effective_radius_at(t)
                            - self.config.moving_obstacle_margin_gain * obs.speed();
                        if d < min_clear {
                            min_clear = d;
                        }
                        if d - required < worst_margin {
                            worst_margin = d - required;
                        }
                    }
                }
            }
            if worst_margin < 0.0 {
                continue; // profile collides somewhere along the horizon
            }
            let score = min_clear.min(1.0) + 0.2 * heading_cost(x, y, theta, goal);
            if best.is_none_or(|(s, _, _)| score > s) {
                best = Some((score, kappa, worst_margin));
            }
        }
        let (_, kappa, worst_margin) = best?;

        let cycle_dt = 0.1; // control cycle at the 10Hz local-planner rate
        let v1 =
            (state.linear_vel.max(0.0) + self.config.max_acceleration * cycle_dt).min(target_speed);
        let w1 = (kappa * v1).clamp(
            -self.config.max_angular_speed,
            self.config.max_angular_speed,
        );
        // Clearance margin above the per-step requirement → confidence in
        // [0.3, 0.9]; 0.3 (= the arbitrator's fallback_min_confidence floor)
        // for a razor-thin escape, 0.9 for ≥0.3m of spare clearance.
        let margin = (worst_margin / 0.3).clamp(0.0, 1.0);
        Some(VelocityCommand {
            linear_x: v1,
            angular_z: w1,
            confidence: (0.3 + 0.6 * margin) as f32,
        })
    }

    /// Simulate a trajectory forward with constant (v, w). Thin wrapper over
    /// the shared `simulate_arc` (also used by the pure-pursuit verifier).
    fn simulate(
        &self,
        state: &RobotState,
        v: f64,
        w: f64,
        obstacles: &[Obstacle],
    ) -> SimTrajectory {
        let roll = simulate_arc(&self.config, state, v, w, obstacles);
        SimTrajectory {
            v,
            w,
            end_x: roll.end_x,
            end_y: roll.end_y,
            end_theta: roll.end_theta,
            min_obstacle_dist: roll.min_obstacle_dist,
            braking_min_dist: roll.braking_min_dist,
            score: 0.0,
        }
    }

    /// Anchor the continuity term to a command emitted by another executor
    /// (pure pursuit). When DWA next runs as the fallback sampler, its
    /// |v − v_prev| tie-breaker must reference the command the robot is
    /// actually executing, not DWA's own — possibly long-stale — last
    /// emission.
    pub fn note_external_command(&mut self, v: f64) {
        self.prev_cmd_v = Some(v);
    }
}

/// Result of a constant-curvature arc rollout: endpoint pose plus the worst
/// net clearances along the horizon and along the braking extension (see
/// `SimTrajectory` for the exact semantics of the two distances).
pub struct ArcRollout {
    pub end_x: f64,
    pub end_y: f64,
    pub end_theta: f64,
    pub min_obstacle_dist: f64,
    pub braking_min_dist: f64,
    /// Worst net clearance PER OBSTACLE (input-slice order), taken over both
    /// the horizon and the braking extension. The aggregate minima above
    /// cannot say WHICH obstacle was closest; the pursuit verifier needs the
    /// per-obstacle number to apply the wedged-start allowance
    /// (`wedged_allowance`) — an obstacle the robot is already inside the
    /// requirement of must not veto a command that only moves away from it.
    pub per_obstacle_min: Vec<f64>,
}

/// Wedged-start clearance allowance for one obstacle: never require more net
/// clearance than the CURRENT pose already has — but never accept getting
/// closer than that either. Mirrors the `reverse_arc_blocker` rule
/// (`allow = threshold.min(d0)`) exactly, restated in the rollout's net-
/// clearance terms: subtracting the moving-obstacle margin from d0 here is
/// algebraically identical to adding it to the threshold there. For an
/// obstacle with full clearance (d0 >= required) this returns `required`
/// unchanged — the standard check.
pub fn wedged_allowance(
    state: &RobotState,
    obs: &Obstacle,
    required: f64,
    moving_margin_gain: f64,
) -> f64 {
    let d0 = ((state.x - obs.x).powi(2) + (state.y - obs.y).powi(2)).sqrt()
        - obs.radius
        - moving_margin_gain * obs.speed();
    required.min(d0)
}

/// Forward-simulate a constant (v, w) arc and report worst clearances.
///
/// This is the single collision-verification machinery shared by the DWA
/// sample loop and the pure-pursuit verifier — both executors accept a
/// command only on the exact same evidence.
///
/// Integration is SUB-STEPPED so consecutive collision checks are at most
/// `COLLISION_CHECK_SPACING_M` (5cm) apart at any sampled speed: at the
/// old plain sim_dt stride (v·sim_dt = 22cm at 2.2 m/s) a trajectory
/// could step clean over a cone between two checks and be accepted.
///
/// After the horizon, the same arc is extended through a braking profile
/// (`BRAKING_REACTION_MARGIN_M` of travel at v, then a
/// `max_deceleration`-limited stop at constant curvature) and the worst
/// clearance along that extension is reported separately
/// (`braking_min_dist`): the caller rejects samples that could not stop
/// inside checked-clear space.
pub fn simulate_arc(
    config: &DwaConfig,
    state: &RobotState,
    v: f64,
    w: f64,
    obstacles: &[Obstacle],
) -> ArcRollout {
    let mut x = state.x;
    let mut y = state.y;
    let mut theta = state.theta;
    let mut min_dist = f64::INFINITY;
    let mut per_obstacle_min = vec![f64::INFINITY; obstacles.len()];

    let steps = (config.sim_time / config.sim_dt) as usize;
    let n_sub = substeps_for(v, config.sim_dt);
    let sub_dt = config.sim_dt / n_sub as f64;

    let mut t = 0.0;
    for _ in 0..steps * n_sub {
        x += v * theta.cos() * sub_dt;
        y += v * theta.sin() * sub_dt;
        theta += w * sub_dt;
        t += sub_dt;

        // Check obstacle distances against velocity-propagated positions:
        // a crossing pedestrian is checked where it WILL be when the
        // robot gets there, not where it was at scan time. Distance is
        // to the object surface (its extent radius subtracted), further
        // reduced by the moving-obstacle margin — a fast obstacle demands
        // extra clearance regardless of how slowly the ROBOT moves.
        for (i, obs) in obstacles.iter().enumerate() {
            let (ox, oy) = obs.position_at(t);
            let d = ((x - ox).powi(2) + (y - oy).powi(2)).sqrt()
                - obs.effective_radius_at(t)
                - config.moving_obstacle_margin_gain * obs.speed();
            if d < min_dist {
                min_dist = d;
            }
            if d < per_obstacle_min[i] {
                per_obstacle_min[i] = d;
            }
        }
    }
    let (end_x, end_y, end_theta) = (x, y, theta);

    // Braking-aware extension: continue the SAME constant-curvature arc
    // from the horizon endpoint for the stopping distance
    // (reaction margin at v, then v²/(2·max_decel) of decelerating
    // travel), checked every ≤ COLLISION_CHECK_SPACING_M of arc length.
    // Obstacles keep propagating in time along the extension (elapsed
    // time per arc step follows the decaying speed profile).
    // Signed-v aware: a REVERSE command (v < 0, planned maneuver segments)
    // extends backwards along the same arc — the ratio w/v stays constant
    // through the braking profile, so per meter of (backward) travel the
    // heading changes by sigma·(w/v) and the position moves sigma·(cosθ,
    // sinθ). The old `v > 1e-9` guard silently skipped the braking check
    // for every reverse command.
    let mut braking_min = f64::INFINITY;
    if v.abs() > 1e-9 && !obstacles.is_empty() && config.max_deceleration > 0.0 {
        let sigma = v.signum();
        let kappa = w / v;
        let brake_dist = v * v / (2.0 * config.max_deceleration) + BRAKING_REACTION_MARGIN_M;
        let n = (brake_dist / COLLISION_CHECK_SPACING_M).ceil().max(1.0) as usize;
        let ds = brake_dist / n as f64;
        let mut s = 0.0;
        for _ in 0..n {
            s += ds;
            // Speed at arc position s: constant through the reaction
            // margin, then v² − 2a·s' beyond it (never below a slow
            // floor so the elapsed-time integral stays finite).
            let decel_s = (s - BRAKING_REACTION_MARGIN_M).max(0.0);
            let speed = (v * v - 2.0 * config.max_deceleration * decel_s)
                .max(0.0)
                .sqrt()
                .max(0.05);
            t += ds / speed;
            theta += sigma * kappa * ds;
            x += sigma * theta.cos() * ds;
            y += sigma * theta.sin() * ds;
            for (i, obs) in obstacles.iter().enumerate() {
                let (ox, oy) = obs.position_at(t);
                let d = ((x - ox).powi(2) + (y - oy).powi(2)).sqrt()
                    - obs.effective_radius_at(t)
                    - config.moving_obstacle_margin_gain * obs.speed();
                if d < braking_min {
                    braking_min = d;
                }
                if d < per_obstacle_min[i] {
                    per_obstacle_min[i] = d;
                }
            }
        }
    }

    ArcRollout {
        end_x,
        end_y,
        end_theta,
        min_obstacle_dist: min_dist,
        braking_min_dist: braking_min,
        per_obstacle_min,
    }
}

/// Velocity preference: 1.0 at the target speed, falling off linearly with
/// the deviation, normalized by the configured max speed (never by the
/// target — dividing by a zero target used to reward the fastest sample
/// exactly when the planner was asked to stop). Never prefers a sample
/// above the target over one at the target.
fn velocity_score(v: f64, target_speed: f64, max_speed: f64) -> f64 {
    1.0 - (v - target_speed).abs() / max_speed.max(0.1)
}

/// Lower bound (m) of the speed-scaled lookahead — the old fixed value,
/// preserved for crawl/low-speed stability.
const LOOKAHEAD_MIN_M: f64 = 1.0;
/// Upper bound (m) of the speed-scaled lookahead.
const LOOKAHEAD_MAX_M: f64 = 3.0;
/// Lookahead horizon in seconds of travel at the current speed.
const LOOKAHEAD_TIME_S: f64 = 1.3;

/// Find the nearest path waypoint ahead as the local goal.
///
/// The lookahead scales with the CURRENT robot speed: a fixed 1.0m goal point
/// is only 0.45s ahead at 2.2 m/s — the robot outruns its own goal point and
/// oscillates around it. 1.3s of travel, floored at the old 1.0m and capped
/// at 3.0m.
fn find_local_goal(state: &RobotState, path: &[PathWaypoint]) -> PathWaypoint {
    let lookahead =
        (LOOKAHEAD_TIME_S * state.linear_vel.abs()).clamp(LOOKAHEAD_MIN_M, LOOKAHEAD_MAX_M);

    for wp in path {
        let dist = ((wp.x - state.x).powi(2) + (wp.y - state.y).powi(2)).sqrt();
        if dist >= lookahead {
            return wp.clone();
        }
    }

    path.last().cloned().unwrap_or(PathWaypoint {
        x: state.x,
        y: state.y,
        theta: state.theta,
        steering: 0.0,
        dir: Default::default(),
    })
}

fn heading_cost(x: f64, y: f64, theta: f64, goal: &PathWaypoint) -> f64 {
    let goal_angle = (goal.y - y).atan2(goal.x - x);
    let diff = normalize_angle(goal_angle - theta).abs();
    1.0 - diff / std::f64::consts::PI // 1.0 = perfect heading, 0.0 = worst
}

fn distance_to_goal(x: f64, y: f64, goal: &PathWaypoint) -> f64 {
    ((x - goal.x).powi(2) + (y - goal.y).powi(2)).sqrt()
}

fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulate_arc_reverse_command_sweeps_and_brakes_backwards() {
        // A REVERSE command must sweep the arc backwards and — the old bug —
        // run the braking extension backwards too (the `v > 0` guard skipped
        // it entirely for negative v, leaving planned reverses without a
        // stopping-distance guarantee).
        let cfg = DwaConfig::default();
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: -0.3,
            angular_vel: 0.0,
        };
        // Obstacle BEHIND the robot, just past the reverse horizon (1.5s ×
        // 0.4 = 0.6m) but inside the braking extension (0.4²/(2·3.0) + 0.15m
        // ≈ 0.18m more); nothing ahead.
        let behind = vec![Obstacle::point(-0.75, 0.0)];
        let roll = simulate_arc(&cfg, &state, -0.4, 0.0, &behind);
        assert!(
            (roll.end_x - (-0.6)).abs() < 1e-6,
            "reverse rollout must end 0.6m back, ended at {}",
            roll.end_x
        );
        // Horizon never reaches the obstacle...
        assert!(roll.min_obstacle_dist > 0.14 && roll.min_obstacle_dist < 0.2);
        // ...but the braking extension, continuing BACKWARDS, must close on
        // it. (The old `v > 0` guard returned INFINITY here.)
        assert!(
            roll.braking_min_dist < 0.05,
            "reverse braking extension missed the obstacle behind: {}",
            roll.braking_min_dist
        );

        // Control: the same obstacle is irrelevant to a FORWARD command.
        let fwd = simulate_arc(&cfg, &state, 0.4, 0.0, &behind);
        assert!(fwd.min_obstacle_dist > 0.7);
        assert!(fwd.braking_min_dist > 0.7);
    }

    #[test]
    fn test_dwa_straight_no_obstacles() {
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![
            PathWaypoint {
                x: 1.0,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            },
            PathWaypoint {
                x: 2.0,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            },
        ];

        let cmd = planner.compute(&state, &path, &[], 0.5);
        assert!(cmd.linear_x > 0.0, "Should move forward");
        assert!(cmd.angular_z.abs() < 0.5, "Should be roughly straight");
    }

    #[test]
    fn test_dwa_propagates_moving_obstacles() {
        // A pedestrian at (0.75, -0.55) walking +y at 0.5 m/s crosses the
        // robot's straight line right when the robot would be there. Checked
        // statically (vy = 0) the straight arc clears it by >0.3m and is
        // chosen; checked against the propagated position it must be
        // rejected or evaded.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let crossing = Obstacle {
            x: 0.75,
            y: -0.55,
            vx: 0.0,
            vy: 0.5,
            radius: 0.15,
        };
        let mut parked = crossing.clone();
        parked.vy = 0.0;

        let cmd_static = planner.compute(&state, &path, &[parked], 0.5);
        let cmd_moving = planner.compute(&state, &path, &[crossing], 0.5);

        // Static: straight ahead at speed.
        assert!(cmd_static.linear_x > 0.3);
        assert!(cmd_static.angular_z.abs() < 0.15);
        // Moving: the same arc now collides mid-simulation; the planner must
        // change something (slow down and/or steer away).
        let evaded =
            cmd_moving.linear_x < cmd_static.linear_x - 0.05 || cmd_moving.angular_z.abs() > 0.2;
        assert!(
            evaded,
            "planner ignored the crossing pedestrian: static=({:.2},{:.2}) moving=({:.2},{:.2})",
            cmd_static.linear_x, cmd_static.angular_z, cmd_moving.linear_x, cmd_moving.angular_z
        );
    }

    #[test]
    fn test_dwa_respects_obstacle_extent_radius() {
        // A 0.3m-radius object whose CENTER clears the path by 0.4m: a point
        // check passes, a surface check must not.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.4,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let fat = Obstacle {
            x: 0.8,
            y: 0.4,
            vx: 0.0,
            vy: 0.0,
            radius: 0.3,
        };
        let cmd = planner.compute(&state, &path, &[fat], 0.5);
        let point = Obstacle::point(0.8, 0.4);
        let cmd_point = planner.compute(&state, &path, &[point], 0.5);
        // Point version: straight through. Extent version: must deviate.
        assert!(cmd_point.angular_z.abs() < 0.15);
        let evaded = cmd.linear_x < cmd_point.linear_x - 0.05 || cmd.angular_z.abs() >= 0.15;
        assert!(
            evaded,
            "extent radius ignored: ({:.2},{:.2})",
            cmd.linear_x, cmd.angular_z
        );
    }

    #[test]
    fn test_dwa_output_stays_inside_curvature_envelope() {
        // Goal 90° to the side tempts a sharp turn; the command must still be
        // executable: |w| <= v * max_curvature, so nothing downstream clamps
        // it into a wider-than-verified arc.
        let config = DwaConfig::default();
        let max_curvature = config.max_curvature;
        let mut planner = DwaPlanner::new(config);
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.4,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 0.0,
            y: 2.0,
            theta: std::f64::consts::FRAC_PI_2,
            steering: 0.0,
            dir: Default::default(),
        }];

        let cmd = planner.compute(&state, &path, &[], 0.5);
        assert!(
            cmd.angular_z.abs() <= cmd.linear_x * max_curvature + 1e-6,
            "command (v={}, w={}) exceeds curvature envelope {}",
            cmd.linear_x,
            cmd.angular_z,
            max_curvature
        );
    }

    #[test]
    fn test_dwa_stationary_cannot_spin_in_place() {
        // Ackermann steering cannot rotate at v=0: from standstill the
        // planner must not emit a pure-rotation command.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: -1.0, // goal behind the robot
            y: 0.5,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];

        let cmd = planner.compute(&state, &path, &[], 0.5);
        assert!(
            cmd.angular_z.abs() <= cmd.linear_x * DwaConfig::default().max_curvature + 1e-6,
            "spin-in-place command is not executable by Ackermann steering"
        );
    }

    #[test]
    fn test_dwa_avoids_obstacle() {
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 2.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let obstacles = vec![Obstacle::point(0.5, 0.0)];

        let cmd = planner.compute(&state, &path, &obstacles, 0.5);
        // Should steer away from obstacle
        assert!(
            cmd.angular_z.abs() > 0.01 || cmd.linear_x < 0.1,
            "Should avoid obstacle by turning or slowing"
        );
    }

    #[test]
    fn test_dwa_zero_target_decelerates_to_stop() {
        // Regression: with target_speed=0 while moving, the old window
        // inverted (v_max=0 < v_min=cur-a·dt) and the sample loop, seeded at
        // v_min and normalized by the zero target, picked the FASTEST sample.
        // Asked to stop, the planner must brake at the accel limit and reach
        // exactly zero.
        let config = DwaConfig::default();
        let max_dv = config.max_acceleration * 0.1; // per 10Hz cycle
        let mut planner = DwaPlanner::new(config);
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];

        let mut state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };

        // 0.5 m/s at 0.05 m/s per cycle = 10 cycles; allow a couple extra.
        for cycle in 0..12 {
            let cmd = planner.compute(&state, &path, &[], 0.0);
            assert!(
                cmd.linear_x <= state.linear_vel + 1e-9,
                "cycle {}: command {} accelerates past current speed {}",
                cycle,
                cmd.linear_x,
                state.linear_vel
            );
            if state.linear_vel > 0.0 {
                assert!(
                    cmd.linear_x < state.linear_vel,
                    "cycle {}: command {} does not decelerate from {}",
                    cycle,
                    cmd.linear_x,
                    state.linear_vel
                );
            }
            assert!(
                state.linear_vel - cmd.linear_x <= max_dv + 1e-9,
                "cycle {}: deceleration {} exceeds accel limit {}",
                cycle,
                state.linear_vel - cmd.linear_x,
                max_dv
            );
            state.linear_vel = cmd.linear_x;
            state.angular_vel = cmd.angular_z;
            if state.linear_vel == 0.0 {
                break;
            }
        }
        assert_eq!(
            state.linear_vel, 0.0,
            "did not reach a full stop within the accel-limited horizon"
        );
    }

    #[test]
    fn test_dwa_velocity_score_never_prefers_exceeding_target() {
        // The score must peak at the target and be monotonically
        // non-increasing above it, for any target including 0.
        let max_speed = 1.0;
        for &target in &[0.0, 0.2, 0.5, 1.0] {
            let peak = velocity_score(target, target, max_speed);
            let mut v = target;
            let mut prev = peak;
            while v < max_speed {
                v = (v + 0.05).min(max_speed);
                let s = velocity_score(v, target, max_speed);
                assert!(
                    s <= peak + 1e-12 && s <= prev + 1e-12,
                    "score prefers v={} over target {} (score {} vs peak {})",
                    v,
                    target,
                    s,
                    peak
                );
                prev = s;
            }
        }
    }

    #[test]
    fn test_dwa_command_never_exceeds_target_when_at_or_below_it() {
        // Cruising at the target with a far goal: the distance reward must
        // not push the command above the desired speed.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 10.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let cmd = planner.compute(&state, &path, &[], 0.5);
        assert!(
            cmd.linear_x <= 0.5 + 1e-9,
            "command {} exceeds desired speed 0.5",
            cmd.linear_x
        );
    }

    #[test]
    fn test_dwa_empty_path() {
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState::default();

        let cmd = planner.compute(&state, &[], &[], 0.5);
        assert_eq!(cmd.linear_x, 0.0);
    }

    #[test]
    fn test_dwa_continuity_prefers_velocity_near_previous_command() {
        // Straight corridor with velocity/distance preferences zeroed: every
        // (v, w=0) sample in the window scores identically on heading and
        // obstacle terms. Only the continuity term breaks the tie, so the
        // chosen v must sit at the previous command, not at the window edge
        // (which the first-best tie resolution would otherwise pick).
        let config = DwaConfig {
            velocity_weight: 0.0,
            distance_weight: 0.0,
            weight_continuity: 0.1,
            ..Default::default()
        };
        let mut planner = DwaPlanner::new(config);
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 10.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];

        // First cycle: continuity reference is the measured speed (0.5); the
        // window is [0.45, 0.55], so the command must stay at 0.5.
        let cmd = planner.compute(&state, &path, &[], 1.0);
        assert!(
            (cmd.linear_x - 0.5).abs() < 1e-6,
            "expected v near previous 0.5, got {}",
            cmd.linear_x
        );

        // Without the continuity term the tie resolves to the window edge —
        // proving the term (not something else) anchors the choice.
        let mut free = DwaPlanner::new(DwaConfig {
            velocity_weight: 0.0,
            distance_weight: 0.0,
            weight_continuity: 0.0,
            ..Default::default()
        });
        let cmd_free = free.compute(&state, &path, &[], 1.0);
        assert!(
            (cmd_free.linear_x - 0.5).abs() > 1e-6,
            "control experiment: ties should NOT land on 0.5 without continuity"
        );

        // Second cycle: the reference is the emitted command, keeping the
        // velocity anchored cycle-to-cycle.
        let state2 = RobotState {
            linear_vel: cmd.linear_x,
            ..state
        };
        let cmd2 = planner.compute(&state2, &path, &[], 1.0);
        assert!(
            (cmd2.linear_x - cmd.linear_x).abs() < 1e-6,
            "command jumped from {} to {} between equal-scoring cycles",
            cmd.linear_x,
            cmd2.linear_x
        );
    }

    #[test]
    fn test_dwa_standstill_escape_outruns_closing_obstacle() {
        // Standstill with an obstacle bearing down along the robot's axis
        // from behind and the corridor ahead open. Every standard sample
        // (≤0.05 m/s → ≤7.5cm of travel) lingers inside the approaching
        // obstacle's uncertainty-inflated sweep → zero feasible trajectories.
        // A committed-acceleration profile sprints clear ahead: the escape
        // must be found and its first-cycle command must be the one-cycle
        // accel step with the envelope-consistent angular rate.
        //
        // Geometry adapted for the moving-obstacle margin: the closer now
        // demands an extra 0.4 * |v| of clearance on top of its uncertainty
        // inflation, so the original 0.3 m/s closer starting 0.45m behind is
        // legitimately uncatchable. A 0.2 m/s closer from 0.6m back keeps
        // the test's intent — standard window blocked, escape outruns it —
        // with the stricter requirement.
        let config = DwaConfig::default();
        let max_curvature = config.max_curvature;
        let accel_step = config.max_acceleration * 0.1; // 0.05 m/s
        let mut planner = DwaPlanner::new(config);
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let closing = Obstacle {
            x: -0.6,
            y: 0.0,
            vx: 0.2,
            vy: 0.0,
            radius: 0.0,
        };

        let cmd = planner.compute(&state, &path, &[closing], 0.5);
        assert!(
            cmd.linear_x > 0.0,
            "standstill escape not found: v={} conf={}",
            cmd.linear_x,
            cmd.confidence
        );
        assert!(
            (cmd.linear_x - accel_step).abs() < 1e-9,
            "escape first-cycle speed {} must be the one-cycle accel step {}",
            cmd.linear_x,
            accel_step
        );
        assert!(
            cmd.angular_z.abs() <= cmd.linear_x * max_curvature + 1e-9,
            "escape command must stay inside the curvature envelope"
        );
        assert!(
            cmd.confidence >= 0.3,
            "escape confidence {} must clear the arbitrator fallback floor",
            cmd.confidence
        );
    }

    #[test]
    fn test_dwa_standstill_boxed_in_stays_infeasible() {
        // Fully boxed in: a ring of obstacles at 0.2m (inside even the
        // crawl-scaled requirement 0.19 + 0.05·0.4 = 0.21) in every
        // direction. Even committed-acceleration profiles collide immediately
        // — the planner must still report infeasibility, never invent an
        // escape.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let ring: Vec<Obstacle> = (0..12)
            .map(|i| {
                let a = i as f64 * std::f64::consts::TAU / 12.0;
                Obstacle::point(0.2 * a.cos(), 0.2 * a.sin())
            })
            .collect();

        let cmd = planner.compute(&state, &path, &ring, 0.5);
        assert_eq!(cmd.linear_x, 0.0, "boxed-in must not move");
        assert!(
            cmd.confidence <= 0.1 + 1e-6,
            "boxed-in must stay infeasible"
        );
    }

    #[test]
    fn test_dwa_standstill_static_frontal_trap_needs_reverse_not_escape() {
        // The Ackermann standstill trap, pinned as a regression: a static
        // obstacle (extent 0.1m) 0.3m dead ahead blocks the whole standard
        // window AND every committed-acceleration profile — with κ ≤ 2.0 the
        // tightest arc still passes within ~0.08m of the obstacle center.
        // Forward escape from a close static frontal obstacle is structurally
        // impossible; the recovery reverse is the correct answer, and the
        // planner must keep saying "infeasible" rather than creep forward.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let cone = Obstacle {
            x: 0.3,
            y: 0.0,
            vx: 0.0,
            vy: 0.0,
            radius: 0.1,
        };
        let cmd = planner.compute(&state, &path, &[cone], 0.5);
        assert_eq!(cmd.linear_x, 0.0);
        assert!(cmd.confidence <= 0.1 + 1e-6);
    }

    #[test]
    fn test_relaxed_radius_scales_margin_never_below_footprint() {
        // Default: 0.19 footprint + 0.05 margin * 0.8 = 0.23.
        let planner = DwaPlanner::new(DwaConfig::default());
        let relaxed = planner.relaxed_radius();
        assert!(relaxed < DwaConfig::default().robot_radius);
        assert!((relaxed - (0.19 + 0.05 * 0.8)).abs() < 1e-9);

        // Even a zero scale never checks below the physical footprint.
        let planner = DwaPlanner::new(DwaConfig {
            recovery_margin_scale: 0.0,
            ..Default::default()
        });
        assert!((planner.relaxed_radius() - ROBOT_FOOTPRINT_RADIUS).abs() < 1e-9);

        // robot_radius configured below the footprint: never negative margin.
        let planner = DwaPlanner::new(DwaConfig {
            robot_radius: 0.1,
            ..Default::default()
        });
        assert!(planner.relaxed_radius() >= ROBOT_FOOTPRINT_RADIUS);
    }

    #[test]
    fn test_dwa_relaxed_radius_unlocks_tight_gap() {
        // A gap that is infeasible at the full collision radius but feasible
        // at the relaxed one: recovery phase A must find a plan where the
        // normal planner reports infeasibility. Margin relaxation composes
        // with speed scaling, so the discriminating band lies between the
        // CRAWL-scaled strict radius (0.19 + 0.05·0.4 = 0.21) and the relaxed
        // one; scale 0.0 relaxes to the raw footprint (0.19) for a clean
        // 1cm band on each side.
        let config = DwaConfig {
            recovery_margin_scale: 0.0,
            ..Default::default()
        };
        let mut planner = DwaPlanner::new(config);
        assert!((planner.relaxed_radius() - ROBOT_FOOTPRINT_RADIUS).abs() < 1e-9);
        let half_gap = 0.2; // between relaxed 0.19 and crawl-scaled strict 0.21

        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.1,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        // Wall of point obstacles ahead with clearance half_gap from every
        // straight/curved escape — box the robot in a corridor of width
        // 2*half_gap around the x axis.
        let mut obstacles = Vec::new();
        let mut x = -0.5;
        while x <= 3.0 {
            obstacles.push(Obstacle::point(x, half_gap));
            obstacles.push(Obstacle::point(x, -half_gap));
            x += 0.05;
        }

        let strict = planner.compute(&state, &path, &obstacles, 0.1);
        assert!(
            strict.confidence <= 0.1 + 1e-6 && strict.linear_x == 0.0,
            "corridor should be infeasible at full radius, got v={} conf={}",
            strict.linear_x,
            strict.confidence
        );

        let relaxed_cmd =
            planner.compute_with_radius(&state, &path, &obstacles, 0.1, planner.relaxed_radius());
        assert!(
            relaxed_cmd.confidence > 0.5 && relaxed_cmd.linear_x > 0.0,
            "relaxed margin should unlock the corridor, got v={} conf={}",
            relaxed_cmd.linear_x,
            relaxed_cmd.confidence
        );
    }

    #[test]
    fn test_dwa_moving_obstacle_margin_rejects_crawl_pass() {
        // The brushed-pedestrian regression: a crawling robot (crawl-scaled
        // requirement 0.21) passing ~0.25-0.3m from a 0.75 m/s obstacle. The
        // robot-speed-scaled margin alone accepts that pass — crawl relief
        // applied to a pedestrian. The moving-obstacle margin (0.4 * 0.75 =
        // 0.30m extra) must reject every sample; the identical geometry with
        // the obstacle static stays accepted (crawl relief for cones).
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let crawl_state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.1,
            angular_vel: 0.0,
        };
        let make_obs = |vx: f64| Obstacle {
            x: 0.12,
            y: 0.25,
            vx,
            vy: 0.0,
            radius: 0.0,
        };

        // Static: 0.25m clearance > 0.21 crawl requirement — accepted.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let cmd = planner.compute(&crawl_state, &path, &[make_obs(0.0)], 0.1);
        assert!(
            cmd.linear_x > 0.0,
            "crawl past a STATIC obstacle at 0.25m must stay feasible, got v={} conf={}",
            cmd.linear_x,
            cmd.confidence
        );

        // Moving at 0.75 m/s: same geometry, requirement grows by 0.30m —
        // every sample in the crawl window must be rejected.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let cmd = planner.compute(&crawl_state, &path, &[make_obs(0.75)], 0.1);
        assert!(
            cmd.linear_x == 0.0 && cmd.confidence <= 0.1 + 1e-6,
            "crawl past a 0.75 m/s obstacle at 0.25m must be rejected, got v={} conf={}",
            cmd.linear_x,
            cmd.confidence
        );

        // Control experiment: gain 0 restores the old speed-blind acceptance,
        // proving the gain (not inflation or propagation) is the discriminator.
        let mut planner = DwaPlanner::new(DwaConfig {
            moving_obstacle_margin_gain: 0.0,
            ..Default::default()
        });
        let cmd = planner.compute(&crawl_state, &path, &[make_obs(0.75)], 0.1);
        assert!(
            cmd.linear_x > 0.0,
            "with gain 0 the old behavior must accept the pass, got v={}",
            cmd.linear_x
        );
    }

    #[test]
    fn test_dwa_scoring_prefers_side_away_from_mover() {
        // Static cone left, slow mover right (approaching head-on), both
        // starting 0.5m off the centerline well ahead — every sample in the
        // window is feasible, so only SCORING decides. The mover's reduced
        // net clearance (moving-obstacle margin + uncertainty inflation)
        // must pull the chosen trajectory toward the static side. Heading
        // weight is reduced so the obstacle preference is observable within
        // the one-cycle steering window; a both-static control pins that the
        // asymmetry comes from the mover, not the layout.
        let config = DwaConfig {
            heading_weight: 0.2,
            ..Default::default()
        };
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let cone = Obstacle {
            x: 1.2,
            y: 0.5,
            vx: 0.0,
            vy: 0.0,
            radius: 0.1,
        };
        let mut mover = Obstacle {
            x: 1.2,
            y: -0.5,
            vx: -0.2, // closing on the robot along -x
            vy: 0.0,
            radius: 0.1,
        };

        let mut planner = DwaPlanner::new(config.clone());
        let cmd = planner.compute(&state, &path, &[cone.clone(), mover.clone()], 0.5);
        assert!(cmd.linear_x > 0.0, "scene must stay feasible");
        assert!(
            cmd.angular_z > 0.05,
            "scoring must steer away from the mover (left), got w={}",
            cmd.angular_z
        );

        // Both static: symmetric scene, no sustained pull to either side.
        mover.vx = 0.0;
        let mut planner = DwaPlanner::new(config);
        let cmd = planner.compute(&state, &path, &[cone, mover], 0.5);
        assert!(
            cmd.angular_z.abs() < 0.05,
            "static-symmetric control must stay near straight, got w={}",
            cmd.angular_z
        );
    }

    #[test]
    fn test_speed_scaled_radius_lerps_crawl_to_cruise() {
        let base = DwaConfig::default().robot_radius; // 0.24
        let low = DwaConfig::default().margin_low_speed_scale; // 0.4
        let gain = DwaConfig::default().high_speed_margin_gain; // 0.06

        // Crawl (≤ 0.1 m/s): footprint + 0.4 of the 5cm margin = 0.21.
        assert!((speed_scaled_radius(0.0, base, low, gain) - 0.21).abs() < 1e-9);
        assert!((speed_scaled_radius(0.1, base, low, gain) - 0.21).abs() < 1e-9);
        // Cruise (0.3 ..= 1.0 m/s): the full base radius; the high-speed
        // growth only starts ABOVE 1.0 m/s.
        assert!((speed_scaled_radius(0.3, base, low, gain) - base).abs() < 1e-9);
        assert!((speed_scaled_radius(1.0, base, low, gain) - base).abs() < 1e-9);
        // Midpoint lerp: 0.19 + 0.05 * 0.7 = 0.225.
        assert!((speed_scaled_radius(0.2, base, low, gain) - 0.225).abs() < 1e-9);
        // Monotone in |v| and never below the physical footprint.
        let mut prev = 0.0;
        let mut v = 0.0;
        while v <= 2.5 {
            let r = speed_scaled_radius(v, base, low, gain);
            assert!(r >= prev && r >= ROBOT_FOOTPRINT_RADIUS);
            prev = r;
            v += 0.02;
        }
        // Reverse speeds scale by magnitude (reverse crawl = crawl).
        assert!((speed_scaled_radius(-0.1, base, low, gain) - 0.21).abs() < 1e-9);
    }

    #[test]
    fn test_speed_scaled_radius_high_speed_margin_growth() {
        let base = DwaConfig::default().robot_radius; // 0.24
        let low = DwaConfig::default().margin_low_speed_scale; // 0.4
        let gain = DwaConfig::default().high_speed_margin_gain; // 0.06

        // Above 1.0 m/s the requirement grows by gain·(v − 1.0) on top of the
        // full margin: +0.03 at 1.5, +0.072 at 2.2.
        assert!((speed_scaled_radius(1.5, base, low, gain) - (base + 0.03)).abs() < 1e-9);
        assert!((speed_scaled_radius(2.2, base, low, gain) - (base + 0.072)).abs() < 1e-9);
        // Gain 0 disables the growth (control) — cruise radius everywhere.
        assert!((speed_scaled_radius(2.2, base, low, 0.0) - base).abs() < 1e-9);
        // Reverse magnitude symmetry holds above cruise too.
        assert!(
            (speed_scaled_radius(-2.2, base, low, gain)
                - speed_scaled_radius(2.2, base, low, gain))
            .abs()
                < 1e-12
        );
    }

    #[test]
    fn test_dwa_crawl_accepts_clearance_that_cruise_rejects() {
        // The same 0.215m-half-width corridor: a crawl sample (requirement
        // 0.21) must pass, a cruise sample (requirement 0.24) must not. The
        // knife edge is now a band: slow = allowed closer, fast = pushed out.
        let half_gap = 0.215;
        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let mut obstacles = Vec::new();
        let mut x = -0.5;
        while x <= 3.0 {
            obstacles.push(Obstacle::point(x, half_gap));
            obstacles.push(Obstacle::point(x, -half_gap));
            x += 0.05;
        }

        // Crawling at 0.1 m/s, asked to crawl: feasible.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let crawl_state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.1,
            angular_vel: 0.0,
        };
        let crawl = planner.compute(&crawl_state, &path, &obstacles, 0.1);
        assert!(
            crawl.linear_x > 0.0 && crawl.confidence > 0.5,
            "crawl through 0.215m clearance must be feasible, got v={} conf={}",
            crawl.linear_x,
            crawl.confidence
        );

        // Cruising at 0.5 m/s: every sample in the one-cycle window
        // ([0.25, 0.5] at the 2.5 m/s² accel limit) requires at least the
        // 0.25 m/s-scaled radius (≈ 0.2325 > 0.215) — infeasible.
        let mut planner = DwaPlanner::new(DwaConfig::default());
        let cruise_state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };
        let cruise = planner.compute(&cruise_state, &path, &obstacles, 0.5);
        assert!(
            cruise.linear_x == 0.0 && cruise.confidence <= 0.1 + 1e-6,
            "cruise sample below the speed-scaled clearance must be rejected, got v={} conf={}",
            cruise.linear_x,
            cruise.confidence
        );
    }

    #[test]
    fn test_dwa_substep_catches_cone_the_coarse_stride_tunnels_over() {
        // Tunneling regression at speed: at 2.0 m/s the plain sim_dt stride
        // checks poses 0.2m apart. A point cone at (0.5, 0.18) with a 0.19
        // collision radius (margins zeroed via config) has a danger chord of
        // only ~0.12m on the straight path — the coarse checks at x = 0.4 and
        // 0.6 both clear it (d ≈ 0.206 > 0.19) while the true path passes at
        // 0.18 < 0.19 through the middle. Sub-stepped checking (≤ 5cm
        // spacing) lands a pose at x ≈ 0.5 and must reject the straight
        // cruise sample; the planner steers away from the cone instead.
        let config = DwaConfig {
            robot_radius: ROBOT_FOOTPRINT_RADIUS, // margin 0: requirement 0.19 at all speeds
            high_speed_margin_gain: 0.0,
            ..Default::default()
        };
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 2.0,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 8.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let cone = Obstacle::point(0.5, 0.18);

        // Control: the same cone 2cm further out (0.21 > 0.19 requirement)
        // clears even the sub-stepped check — straight at speed.
        let mut planner = DwaPlanner::new(config.clone());
        let clear = planner.compute(&state, &path, &[Obstacle::point(0.5, 0.21)], 2.0);
        assert!(
            clear.linear_x > 1.9 && clear.angular_z.abs() < 0.05,
            "control: cone outside the radius must not deflect the cruise, got ({:.2},{:.2})",
            clear.linear_x,
            clear.angular_z
        );

        // The tunnel-bait cone: with coarse-only checking the straight
        // 2.0 m/s sample scores best (velocity term peaks at the target) and
        // drives straight through the cone. Sub-stepping must reject every
        // near-straight sample; the planner steers away (negative w — the
        // cone is on the left) or sheds real speed.
        let mut planner = DwaPlanner::new(config);
        let cmd = planner.compute(&state, &path, &[cone], 2.0);
        assert!(
            cmd.angular_z < -0.05 || cmd.linear_x < 1.5,
            "coarse-stride tunneling: planner kept the straight cruise ({:.2},{:.2}) \
             through a cone 0.18m off the path centerline",
            cmd.linear_x,
            cmd.angular_z
        );
    }

    #[test]
    fn test_dwa_braking_distance_rejects_endpoint_too_close_to_wall() {
        // Braking-aware feasibility: cruising at 2.2 m/s toward a wall at
        // x = 3.8m. The 1.5s horizon ends at 3.3m — geometrically clear of
        // the wall by 0.5m (> the ~0.31m high-speed requirement) — but the
        // stopping distance is 2.2²/(2·3.0) + 0.15 ≈ 0.96m: the robot cannot
        // stop inside checked-clear space, so the straight cruise sample must
        // be REJECTED (hard check) and the command must not remain a straight
        // continuation at cruise speed.
        //
        // obstacle_weight is zeroed so the obstacle-distance SCORING term
        // cannot mask the hard check: with it, the planner would steer away
        // from the wall on preference alone and the test could not tell a
        // soft preference from the mandatory braking-feasibility rejection.
        let config = DwaConfig {
            obstacle_weight: 0.0,
            ..Default::default()
        };
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 2.2,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 10.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let wall = |x0: f64| -> Vec<Obstacle> {
            let mut w = Vec::new();
            let mut y = -2.0;
            while y <= 2.0 {
                w.push(Obstacle::point(x0, y));
                y += 0.1;
            }
            w
        };

        // Control: the same wall at 6.0m leaves the full stopping distance
        // beyond the horizon — straight cruise stays feasible.
        let mut planner = DwaPlanner::new(config.clone());
        let clear = planner.compute(&state, &path, &wall(6.0), 2.2);
        assert!(
            clear.linear_x > 1.9 && clear.angular_z.abs() < 0.05,
            "control: wall beyond horizon+braking distance must not slow the cruise, \
             got ({:.2},{:.2})",
            clear.linear_x,
            clear.angular_z
        );

        // Wall at 3.8m: the straight sample passes the geometric horizon
        // check but fails the braking extension. The planner must brake
        // and/or arc away — anything but continuing straight at cruise.
        let mut planner = DwaPlanner::new(config);
        let cmd = planner.compute(&state, &path, &wall(3.8), 2.2);
        assert!(
            cmd.linear_x < 2.0 || cmd.angular_z.abs() > 0.3,
            "endpoint inside braking distance of the wall must not be driven straight \
             at cruise, got ({:.2},{:.2})",
            cmd.linear_x,
            cmd.angular_z
        );
    }

    #[test]
    fn test_local_goal_lookahead_scales_with_speed() {
        // Waypoints every 0.5m along +x. The chosen local goal is the first
        // waypoint at/beyond the lookahead: 1.0m floor when slow, 1.3s of
        // travel at speed (2.86m at 2.2 m/s), capped at 3.0m.
        let path: Vec<PathWaypoint> = (1..=10)
            .map(|i| PathWaypoint {
                x: 0.5 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect();
        let at_speed = |v: f64| RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: v,
            angular_vel: 0.0,
        };

        // Standstill / crawl: the old 1.0m behavior is preserved.
        assert_eq!(find_local_goal(&at_speed(0.0), &path).x, 1.0);
        assert_eq!(find_local_goal(&at_speed(0.5), &path).x, 1.0);
        // At 1.0 m/s: 1.3m lookahead → first waypoint at 1.5m.
        assert_eq!(find_local_goal(&at_speed(1.0), &path).x, 1.5);
        // At 2.2 m/s: 2.86m → 3.0m waypoint (no longer outrun in 0.45s).
        assert_eq!(find_local_goal(&at_speed(2.2), &path).x, 3.0);
        // Cap at 3.0m even for absurd speeds; reverse uses |v|.
        assert_eq!(find_local_goal(&at_speed(10.0), &path).x, 3.0);
        assert_eq!(find_local_goal(&at_speed(-2.2), &path).x, 3.0);
    }

    #[test]
    fn test_dwa_config_validation() {
        assert!(DwaConfig::default().validate().is_ok());
        // Zero deceleration would give the braking check an infinite
        // stopping distance.
        assert!(DwaConfig {
            max_deceleration: 0.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(DwaConfig {
            max_speed: -1.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(DwaConfig {
            sim_dt: 2.0, // > sim_time
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(DwaConfig {
            v_samples: 1,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(DwaConfig {
            high_speed_margin_gain: -0.1,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(DwaConfig {
            moving_obstacle_margin_gain: f64::NAN,
            ..Default::default()
        }
        .validate()
        .is_err());
        // Zero gains are valid explicit opt-outs.
        assert!(DwaConfig {
            high_speed_margin_gain: 0.0,
            moving_obstacle_margin_gain: 0.0,
            ..Default::default()
        }
        .validate()
        .is_ok());
    }
}
