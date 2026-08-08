/// Local planner: tracking-MPC primary executor on forward runs, pure
/// pursuit beneath it (reverse runs, cusp/terminal endgame, MPC-infeasible
/// windows), DWA as the reactive fallback sampler, scripted recovery last.
///
/// The tracking MPC (`tracking_mpc`) optimizes speed and curvature JOINTLY
/// over a receding horizon — piecewise arcs instead of pursuit's single
/// circle, with a reference speed profile that ramps to zero exactly at the
/// stop point. Pure pursuit executes signed runs (cusps included) verified
/// with the shared rollout machinery; DWA samples reactively when both
/// defer; the legacy SimpleMpc (`mpc`) remains for the extreme-curvature
/// trigger. All run inside the 10Hz local-planner loop.
pub mod dwa;
pub mod mpc;
pub mod pursuit;
pub mod tracking_mpc;

use serde::Deserialize;
use tracing::debug;

pub use pursuit::PursuitDefer;

use crate::global_planner::{PathWaypoint, SegmentDir};

/// Velocity command output from the local planner.
#[derive(Debug, Clone, Default)]
pub struct VelocityCommand {
    pub linear_x: f64,   // m/s
    pub angular_z: f64,  // rad/s
    pub confidence: f32, // [0.0, 1.0]
}

/// Pose along a predicted trajectory.
#[derive(Debug, Clone, Default)]
pub struct TrajPoint {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// Which executor produced a local-plan command, exposed for the cycle log
/// (exec=pursuit|dwa|mpc|scripted) and post-run analysis.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Executor {
    /// Pure pursuit on the global path (primary).
    Pursuit,
    /// DWA sampling (fallback; also the deliberate-stop path).
    #[default]
    Dwa,
    /// MPC fallback for extreme path curvature.
    Mpc,
    /// Scripted recovery command (reverse / hold).
    Scripted,
}

impl std::fmt::Display for Executor {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            Executor::Pursuit => "pursuit",
            Executor::Dwa => "dwa",
            Executor::Mpc => "mpc",
            Executor::Scripted => "scripted",
        })
    }
}

/// Local planner output: the velocity command plus the predicted trajectory
/// obtained by forward-integrating that command from the current state. The
/// trajectory is forwarded to the tracker (feed-forward) and published on CH10
/// for visualization.
#[derive(Debug, Clone, Default)]
pub struct LocalPlan {
    pub command: VelocityCommand,
    pub trajectory: Vec<TrajPoint>,
    /// Which executor produced `command`.
    pub executor: Executor,
    /// The command is part of a PLANNED maneuver (reverse-segment execution
    /// or the scripted stop at a direction cusp). The stuck detector must not
    /// count these deliberately non-forward commands as "no feasible plan".
    pub planned_maneuver: bool,
}

/// Robot state for local planning.
#[derive(Debug, Clone, Default)]
pub struct RobotState {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub linear_vel: f64,
    pub angular_vel: f64,
}

/// Obstacle from perception: an untracked point sample (radius 0, zero
/// velocity — walls, sector-sampled returns), a tracked circle, or a
/// tracked ORIENTED RECTANGLE (both half-extents > 0). The rectangle cures
/// the circular over-approximation that closed narrow passages between box
/// obstacles (a cube's circumscribed circle overstates its half-width by up
/// to 41%); `radius` stays populated as the conservative fallback.
#[derive(Debug, Clone, Default)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub radius: f64,
    /// Oriented-rectangle half extent along the box's own x axis (m);
    /// active only when BOTH half extents are > 0.
    pub half_x: f64,
    /// Oriented-rectangle half extent along the box's own y axis (m).
    pub half_y: f64,
    /// Box heading in world frame (radians).
    pub heading: f64,
}

impl Obstacle {
    /// Untracked point sample.
    #[allow(dead_code)] // convenience constructor used by unit tests
    pub fn point(x: f64, y: f64) -> Self {
        Self {
            x,
            y,
            ..Default::default()
        }
    }

    /// Tracked oriented-rectangle obstacle (test/construction convenience).
    #[allow(dead_code)]
    pub fn boxed(x: f64, y: f64, half_x: f64, half_y: f64, heading: f64) -> Self {
        Self {
            x,
            y,
            // Conservative circular fallback for any consumer that ignores
            // the rectangle: the circumscribed radius.
            radius: (half_x * half_x + half_y * half_y).sqrt(),
            half_x,
            half_y,
            heading,
            ..Default::default()
        }
    }

    /// Position propagated along the velocity estimate by `t` seconds.
    pub fn position_at(&self, t: f64) -> (f64, f64) {
        (self.x + self.vx * t, self.y + self.vy * t)
    }

    /// Speed magnitude (m/s) of the velocity estimate; 0 for untracked points.
    pub fn speed(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy).sqrt()
    }

    /// Signed distance (m) from point (px, py) to this obstacle's boundary
    /// at lookahead `t`: velocity-propagated and uncertainty-inflated, shape
    /// aware (oriented rectangle when half extents are set, else the circle).
    /// Negative inside. The single geometry every collision check shares —
    /// callers subtract their own margins (robot requirement, moving margin).
    pub fn net_distance_at(&self, px: f64, py: f64, t: f64) -> f64 {
        let (ox, oy) = self.position_at(t);
        let grow = PREDICTION_UNCERTAINTY * self.speed() * t;
        if self.half_x > 0.0 && self.half_y > 0.0 {
            let (dx, dy) = (px - ox, py - oy);
            let (c, s) = (self.heading.cos(), self.heading.sin());
            let bx = (dx * c + dy * s).abs() - (self.half_x + grow);
            let by = (-dx * s + dy * c).abs() - (self.half_y + grow);
            let outside = (bx.max(0.0).powi(2) + by.max(0.0).powi(2)).sqrt();
            let inside = bx.max(by).min(0.0);
            outside + inside
        } else {
            ((px - ox).powi(2) + (py - oy).powi(2)).sqrt() - (self.radius + grow)
        }
    }
}

/// Meters of prediction uncertainty per meter of predicted travel.
const PREDICTION_UNCERTAINTY: f64 = 0.4;

#[derive(Debug, Clone, Deserialize)]
pub struct LocalPlannerConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default)]
    pub dwa: dwa::DwaConfig,
    #[serde(default)]
    pub mpc: mpc::MpcConfig,
    #[serde(default)]
    pub pursuit: pursuit::PursuitConfig,
    #[serde(default)]
    pub tracking_mpc: tracking_mpc::TrackingMpcConfig,
    #[serde(default = "default_mpc_trigger_curvature")]
    pub mpc_trigger_curvature: f64, // use MPC when path curvature exceeds this
    /// Pipeline delay (s) between the pose the planner sees and the moment
    /// its command reaches the chassis: ~1 perception cycle of pose age +
    /// 1 planning cycle + 1 control cycle at 10Hz each. The local-planning
    /// input state is forward-projected along its measured twist by this
    /// much, so commands are chosen for the pose the robot will occupy when
    /// they bite instead of the pose it left behind (at 2 m/s the stale pose
    /// is 0.4-0.8m behind reality — a systematic tracking gap). 0 disables.
    #[serde(default = "default_actuation_delay_s")]
    pub actuation_delay_s: f64,
}

fn default_rate_hz() -> u32 {
    10
}
fn default_mpc_trigger_curvature() -> f64 {
    // DWA owns normal avoidance (gauntlet-tested); MPC only takes over for
    // genuinely extreme curvature. 1.5 handed the tight slalom to MPC, whose
    // cost-pinned confidence then deadlocked the arbitrator gate.
    2.2
} // 1/m
fn default_actuation_delay_s() -> f64 {
    0.2
}

/// Forward-project a robot state along its measured twist by `tau` seconds
/// (constant-curvature arc; straight-line below the angular-rate epsilon).
/// The actuation-delay compensation for the local-planning input.
pub fn project_state(s: &RobotState, tau: f64) -> RobotState {
    if tau <= 0.0 {
        return s.clone();
    }
    let (v, w) = (s.linear_vel, s.angular_vel);
    let (x, y, theta) = if w.abs() < 1e-6 {
        (
            s.x + v * s.theta.cos() * tau,
            s.y + v * s.theta.sin() * tau,
            s.theta,
        )
    } else {
        let r = v / w;
        (
            s.x + r * ((s.theta + w * tau).sin() - s.theta.sin()),
            s.y - r * ((s.theta + w * tau).cos() - s.theta.cos()),
            s.theta + w * tau,
        )
    };
    RobotState {
        x,
        y,
        theta,
        linear_vel: v,
        angular_vel: w,
    }
}

impl Default for LocalPlannerConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            dwa: dwa::DwaConfig::default(),
            mpc: mpc::MpcConfig::default(),
            pursuit: pursuit::PursuitConfig::default(),
            tracking_mpc: tracking_mpc::TrackingMpcConfig::default(),
            mpc_trigger_curvature: default_mpc_trigger_curvature(),
            actuation_delay_s: default_actuation_delay_s(),
        }
    }
}

impl LocalPlannerConfig {
    /// Startup validation (fail loud on nonsense YAML); the DWA and pursuit
    /// numbers are the safety-relevant ones, plus the cross-struct coherence
    /// checks neither struct can perform alone.
    pub fn validate(&self) -> Result<(), String> {
        self.dwa.validate()?;
        self.pursuit.validate()?;
        self.tracking_mpc.validate()?;
        // Pursuit inverts the smoother's steering feed-forward through the
        // bicycle model (|κ| = |tan δ| / wheelbase) for the anticipatory
        // curvature profile — a non-positive wheelbase poisons every κ.
        if !(self.mpc.wheelbase > 0.0 && self.mpc.wheelbase.is_finite()) {
            return Err(format!(
                "mpc.wheelbase must be > 0 (pursuit curvature profile depends on it), got {}",
                self.mpc.wheelbase
            ));
        }
        // A turning-speed floor above the platform limit would ask curvature
        // governors to hold a speed the dynamics cannot deliver.
        if self.pursuit.v_turn_min > self.dwa.max_speed {
            return Err(format!(
                "pursuit.v_turn_min ({}) must be <= dwa.max_speed ({})",
                self.pursuit.v_turn_min, self.dwa.max_speed
            ));
        }
        // Delay compensation beyond a second would be extrapolating the pose
        // past any believable pipeline latency (and past obstacle validity).
        if !(0.0..=1.0).contains(&self.actuation_delay_s) || !self.actuation_delay_s.is_finite() {
            return Err(format!(
                "actuation_delay_s must be in [0, 1], got {}",
                self.actuation_delay_s
            ));
        }
        Ok(())
    }
}

/// Unified local planner: tracking-MPC primary + pursuit + DWA fallback.
pub struct LocalPlanner {
    config: LocalPlannerConfig,
    dwa_planner: dwa::DwaPlanner,
    mpc_planner: mpc::SimpleMpc,
    pursuit_planner: pursuit::PursuitPlanner,
    tracking_mpc: tracking_mpc::TrackingMpc,
    use_mpc: bool,
    /// Linear speed of the last emitted command from ANY executor — the
    /// accel-limit reference for the pursuit ramp (a scripted reverse or a
    /// DWA cycle must not let the next pursuit command jump discontinuously).
    prev_cmd_v: Option<f64>,
}

impl LocalPlanner {
    pub fn new(config: LocalPlannerConfig) -> Self {
        let dwa_planner = dwa::DwaPlanner::new(config.dwa.clone());
        let mpc_planner = mpc::SimpleMpc::new(config.mpc.clone());
        let pursuit_planner = pursuit::PursuitPlanner::new(
            config.pursuit.clone(),
            config.dwa.clone(),
            // Ackermann wheelbase for inverting the smoother's steering
            // feed-forward back to curvature in the anticipatory profile.
            config.mpc.wheelbase,
        );
        let tracking_mpc =
            tracking_mpc::TrackingMpc::new(config.tracking_mpc.clone(), config.dwa.clone());
        Self {
            config,
            dwa_planner,
            mpc_planner,
            pursuit_planner,
            tracking_mpc,
            use_mpc: false,
            prev_cmd_v: None,
        }
    }

    /// Install (or clear) the roadmap reference corridor for the DWA
    /// fallback sampler — pursuit follows the global path (already
    /// corridor-constrained at the source) and needs no bound of its own.
    /// Called once per planning cycle; None whenever no route is active.
    pub fn set_corridor(&mut self, corridor: Option<crate::global_planner::Corridor>) {
        self.dwa_planner.set_corridor(corridor);
    }

    /// Compute a velocity command + predicted trajectory.
    /// Empty path → zero command, empty trajectory.
    ///
    /// Arbitration: if a global path exists and pure pursuit yields a
    /// rollout-verified command, use it. Otherwise fall back to the existing
    /// DWA sampling (or MPC beyond the curvature trigger), then the recovery
    /// machinery as before (driven by the caller).
    #[allow(dead_code)] // plan-only entry point kept for unit tests
    pub fn compute(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> LocalPlan {
        self.compute_with_defer(state, path, obstacles, desired_speed)
            .0
    }

    /// `compute` additionally reporting the pursuit deferral: `Some(reason)`
    /// exactly when the PRIMARY executor was attempted and fell through to a
    /// fallback this cycle (None: pursuit drove, or there was no path to
    /// attempt). The caller's stale-heading streak needs the reason from
    /// EVERY cycle — reporting it only from the recovery Reverse branch let
    /// interleaved phases reset the count and kept a heading-stale path
    /// alive indefinitely.
    pub fn compute_with_defer(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> (LocalPlan, Option<PursuitDefer>) {
        if path.is_empty() {
            // A deliberate stop (no path to follow, e.g. Idle) is a
            // high-confidence command. Low confidence is reserved for "could
            // not find a feasible trajectory", which the arbitrator escalates
            // to an emergency stop.
            return (
                LocalPlan {
                    command: VelocityCommand {
                        linear_x: 0.0,
                        angular_z: 0.0,
                        confidence: 1.0,
                    },
                    trajectory: Vec::new(),
                    executor: Executor::Dwa,
                    planned_maneuver: false,
                },
                None,
            );
        }

        // Tracking MPC first on FORWARD runs: joint speed+curvature
        // optimization onto the path (piecewise arcs, exact-stop speed
        // profile). It declines (None) on reverse runs, inside the run-end
        // handoff zone, and on infeasible windows — pursuit owns those.
        let (run_end, run_dir) = pursuit::active_run(path);
        if run_dir == SegmentDir::Forward {
            let prev_v = self.prev_cmd_v.unwrap_or(state.linear_vel);
            if let Some((cmd, traj)) = self.tracking_mpc.compute(
                state,
                &path[..=run_end],
                obstacles,
                desired_speed,
                prev_v,
                // The path end is always a plan goal (mission pose or route
                // carrot): braking to it is safe, and the carrot hops ahead
                // before the ramp binds mid-route.
                true,
            ) {
                self.use_mpc = false;
                self.dwa_planner.note_external_command(cmd.linear_x);
                self.prev_cmd_v = Some(cmd.linear_x);
                // Publish the PLANNED sequence, not a constant-arc rollout
                // of the first command: the polynomial plan is what the
                // tracker should feed-forward and what the visualizer must
                // show (the constant arc read as "circular planning" and
                // hid the actual quintic from every downstream consumer).
                return (
                    LocalPlan {
                        command: cmd,
                        trajectory: traj,
                        executor: Executor::Mpc,
                        planned_maneuver: false,
                    },
                    None,
                );
            }
        }

        // Pure pursuit next: the exact arc onto the global path (signed
        // segments included), verified (and speed-stepped) by the shared
        // rollout machinery.
        let defer = match self.compute_pursuit(state, path, obstacles, desired_speed) {
            Ok(plan) => return (plan, None),
            Err(reason) => reason,
        };

        // Pursuit failed. On a REVERSE segment there is no fallback sampler:
        // DWA and MPC are forward-only and would chase a local goal behind
        // the robot. Emit the same low-confidence stop DWA emits when its
        // window is infeasible, so the stuck detector and recovery machinery
        // see the blockage exactly as they would a forward one.
        if active_segment_reverse(path) {
            debug!("local exec=pursuit reverse segment blocked — infeasible stop");
            return (
                self.finish(
                    state,
                    VelocityCommand {
                        linear_x: 0.0,
                        angular_z: 0.0,
                        confidence: 0.1,
                    },
                    Executor::Pursuit,
                ),
                Some(defer),
            );
        }

        // Determine if MPC should take over based on path curvature
        self.use_mpc = self.should_use_mpc(state, path);

        let (command, executor) = if self.use_mpc {
            (
                self.mpc_planner
                    .compute(state, path, obstacles, desired_speed),
                Executor::Mpc,
            )
        } else {
            (
                self.dwa_planner
                    .compute(state, path, obstacles, desired_speed),
                Executor::Dwa,
            )
        };
        debug!(
            "local exec={} v={:.2} w={:.2} conf={:.2}",
            executor, command.linear_x, command.angular_z, command.confidence
        );

        (self.finish(state, command, executor), Some(defer))
    }

    /// Pursuit-only attempt on the current global path — the PRIMARY
    /// executor, exposed separately so the Recovery command selection in
    /// main.rs can execute a planned escape (reverse legs included) BEFORE
    /// any relaxed/scripted fallback. On success the plan carries the
    /// planned-maneuver flag for the stuck detector; on failure the deferral
    /// reason says why (diagnostic hierarchy-fallback logging).
    pub fn compute_pursuit(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> Result<LocalPlan, PursuitDefer> {
        let prev_v = self.prev_cmd_v.unwrap_or(state.linear_vel);
        let pursuit::PursuitCommand { command, maneuver } =
            self.pursuit_planner
                .compute(state, path, obstacles, desired_speed, prev_v)?;
        debug!(
            "local exec=pursuit v={:.2} w={:.2} conf={:.2} maneuver={}",
            command.linear_x, command.angular_z, command.confidence, maneuver
        );
        self.use_mpc = false;
        // Keep DWA's continuity anchor on the executing command so a later
        // fallback cycle doesn't tie-break against a stale speed.
        self.dwa_planner.note_external_command(command.linear_x);
        Ok(self.finish_flagged(state, command, Executor::Pursuit, maneuver))
    }

    /// Record the emitted command (pursuit accel-limit reference) and attach
    /// the forward-integrated trajectory.
    fn finish(
        &mut self,
        state: &RobotState,
        command: VelocityCommand,
        executor: Executor,
    ) -> LocalPlan {
        self.finish_flagged(state, command, executor, false)
    }

    /// `finish` carrying the planned-maneuver flag through to the output.
    fn finish_flagged(
        &mut self,
        state: &RobotState,
        command: VelocityCommand,
        executor: Executor,
        planned_maneuver: bool,
    ) -> LocalPlan {
        self.prev_cmd_v = Some(command.linear_x);
        let trajectory = rollout(
            state,
            &command,
            self.config.dwa.sim_time,
            self.config.dwa.sim_dt,
        );
        LocalPlan {
            command,
            trajectory,
            executor,
            planned_maneuver,
        }
    }

    /// Recovery phase A: retry forward planning with the DWA obstacle margin
    /// relaxed (`dwa.recovery_margin_scale` applied to the margin above the
    /// physical footprint, never below it) at crawl speed. Always DWA — MPC
    /// has no business in a scripted recovery crawl.
    pub fn compute_relaxed(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> LocalPlan {
        if path.is_empty() {
            return LocalPlan {
                command: VelocityCommand {
                    linear_x: 0.0,
                    angular_z: 0.0,
                    confidence: 1.0,
                },
                trajectory: Vec::new(),
                executor: Executor::Dwa,
                planned_maneuver: false,
            };
        }
        // No forward sampler exists on a REVERSE active run (same guard as
        // `compute`): a relaxed DWA would chase a local goal behind the
        // robot. Emit the infeasible stop so the recovery machinery sees the
        // blockage — pursuit (tried first by the caller) owns reverse legs.
        if active_segment_reverse(path) {
            debug!("local relaxed retry on reverse segment — infeasible stop");
            return self.finish(
                state,
                VelocityCommand {
                    linear_x: 0.0,
                    angular_z: 0.0,
                    confidence: 0.1,
                },
                Executor::Dwa,
            );
        }
        self.use_mpc = false;
        let radius = self.dwa_planner.relaxed_radius();
        let command =
            self.dwa_planner
                .compute_with_radius(state, path, obstacles, desired_speed, radius);
        self.finish(state, command, Executor::Dwa)
    }

    /// Wrap a scripted command (recovery reverse / hold) as a LocalPlan with
    /// its forward-integrated trajectory for CH10 visualization.
    pub fn scripted_plan(&mut self, state: &RobotState, command: VelocityCommand) -> LocalPlan {
        self.finish(state, command, Executor::Scripted)
    }

    /// Check if MPC should be used (tight curvature ahead).
    /// Never called for reverse segments (`compute` short-circuits them).
    fn should_use_mpc(&self, _state: &RobotState, path: &[PathWaypoint]) -> bool {
        // Look at the next few waypoints for sharp turns
        let lookahead = 5.min(path.len());
        for i in 1..lookahead {
            let dtheta = (path[i].theta - path[i - 1].theta).abs();
            let dist =
                ((path[i].x - path[i - 1].x).powi(2) + (path[i].y - path[i - 1].y).powi(2)).sqrt();

            if dist > 0.01 {
                let curvature = dtheta / dist;
                if curvature > self.config.mpc_trigger_curvature {
                    return true;
                }
            }
        }
        false
    }
}

/// True when the path's active (first) direction run is a reverse segment —
/// the case with no forward-sampler fallback.
fn active_segment_reverse(path: &[PathWaypoint]) -> bool {
    pursuit::active_run(path).1 == SegmentDir::Reverse
}

/// Per-cycle truncation point for the retained global path: the waypoint
/// nearest (x, y) WITHIN the active (first) direction run only. Later runs
/// of a maneuver path may pass arbitrarily close to the robot's current pose
/// (reverse 0.5m, then forward back through the start area); they are
/// execution FUTURE and must never absorb the truncation — a global nearest
/// search would amputate the not-yet-executed reverse leg on cycle one.
pub fn active_run_truncation_index(path: &[PathWaypoint], x: f64, y: f64) -> usize {
    if path.is_empty() {
        return 0;
    }
    let (run_end, _) = pursuit::active_run(path);
    // Cusp arrival: once the active run's remaining distance is inside the
    // executor's arrival radius (and another run follows), drop the run
    // entirely — the cusp waypoint anchors the next run. Leaving the last
    // centimeters in place would flip the active run back to the exhausted
    // direction as soon as the robot pulls away along the next run.
    if run_end + 1 < path.len()
        && pursuit::remaining_run_distance(&path[..=run_end], x, y) < pursuit::CUSP_ARRIVE_M
    {
        return run_end;
    }
    crate::global_planner::nearest_waypoint_index(&path[..=run_end], x, y)
}

/// An obstacle refusing a reverse maneuver, for operator-facing logs.
#[derive(Debug, Clone, Copy)]
pub struct RearBlocker {
    pub x: f64,
    pub y: f64,
    /// Minimum distance from the robot center along the swept reverse path to
    /// the obstacle surface (m).
    pub surface_dist: f64,
}

/// Sample spacing (m of reversed arc length) for the swept reverse check.
const REVERSE_SWEEP_STEP_M: f64 = 0.02;

/// Swept clearance check for a reverse escape arc.
///
/// Samples poses along the path traced by reversing with signed curvature
/// `kappa` (rad per meter reversed; 0 = straight; equals `angular_z /
/// |linear_x|` of the scripted command) for `reverse_dist` meters. For every
/// obstacle, every sampled pose must keep `robot_radius + margin` clearance
/// to its surface — OR at least never come closer to it than the START pose
/// already is. That second clause is the fix for the gauntlet freeze: the
/// robot's CURRENT pose is not part of the swept corridor, so the frontal
/// obstacle the robot is wedged against (already inside the threshold of the
/// robot center) cannot veto the escape, because backing up strictly
/// increases clearance to it. The old segment check clamped onto the robot
/// center and let exactly that obstacle refuse the reverse.
///
/// Obstacles are checked at their current positions (the reverse is short
/// and slow), but a MOVING obstacle raises its per-obstacle threshold by
/// `moving_margin_gain × |v_obs|` — the same moving-obstacle margin as the
/// forward DWA checks, so a pedestrian near the reverse corridor vetoes the
/// escape earlier than a cone at the same distance.
/// Returns the worst blocking obstacle, or None when the arc is clear.
pub fn reverse_arc_blocker(
    state: &RobotState,
    obstacles: &[Obstacle],
    robot_radius: f64,
    margin: f64,
    moving_margin_gain: f64,
    reverse_dist: f64,
    kappa: f64,
) -> Option<RearBlocker> {
    if reverse_dist <= 0.0 {
        return None;
    }
    let n = (reverse_dist / REVERSE_SWEEP_STEP_M).ceil().max(1.0) as usize;
    let ds = reverse_dist / n as f64;
    let mut worst: Option<RearBlocker> = None;
    for obs in obstacles {
        let threshold = robot_radius + margin + moving_margin_gain * obs.speed();
        let d0 = obs.net_distance_at(state.x, state.y, 0.0);
        // Wedged-start allowance: never require more clearance than the start
        // pose already has (but never accept getting closer than that either).
        let allow = threshold.min(d0);
        let (mut x, mut y, mut th) = (state.x, state.y, state.theta);
        let mut min_d = f64::INFINITY;
        for _ in 0..n {
            th += kappa * ds;
            x -= th.cos() * ds;
            y -= th.sin() * ds;
            let d = obs.net_distance_at(x, y, 0.0);
            if d < min_d {
                min_d = d;
            }
        }
        if min_d < allow - 1e-9 && worst.is_none_or(|w| min_d < w.surface_dist) {
            worst = Some(RearBlocker {
                x: obs.x,
                y: obs.y,
                surface_dist: min_d,
            });
        }
    }
    worst
}

/// True when a straight reverse of `reverse_dist` meters is clear (see
/// `reverse_arc_blocker` for the exact swept-clearance semantics).
#[allow(dead_code)] // straight-reverse convenience wrapper; exercised by unit tests
pub fn rear_corridor_clear(
    state: &RobotState,
    obstacles: &[Obstacle],
    robot_radius: f64,
    margin: f64,
    moving_margin_gain: f64,
    reverse_dist: f64,
) -> bool {
    reverse_arc_blocker(
        state,
        obstacles,
        robot_radius,
        margin,
        moving_margin_gain,
        reverse_dist,
        0.0,
    )
    .is_none()
}

/// Obstacles farther than this (surface distance) never steer the escape arc.
const FRONTAL_RANGE_M: f64 = 1.0;

/// Plan the recovery reverse command: an arc that rotates the nose AWAY from
/// the nearest front-blocking obstacle, swept-checked along the actual arc.
///
/// The nearest obstacle in the frontal half-plane (within `FRONTAL_RANGE_M`)
/// picks the side: obstacle to the LEFT → reverse with `angular_z < 0` so the
/// nose sweeps right, away from it. Candidates are tried in preference order
/// (steered-away, straight, steered-toward) and the first arc that clears the
/// swept check wins. With no frontal obstacle to identify a side, only the
/// straight reverse is considered. `|angular_z| = speed * max_curvature`, the
/// symmetric reverse form of the executable-curvature envelope.
// The parameters are the escape's full physical contract (robot geometry,
// margins, sweep length, kinematics); bundling them into a one-off struct
// would only rename the same eight numbers.
#[allow(clippy::too_many_arguments)]
pub fn plan_reverse_escape(
    state: &RobotState,
    obstacles: &[Obstacle],
    robot_radius: f64,
    margin: f64,
    moving_margin_gain: f64,
    reverse_dist: f64,
    speed: f64,
    max_curvature: f64,
) -> Result<VelocityCommand, RearBlocker> {
    debug_assert!(speed > 0.0);
    let mut side = 0.0f64;
    let mut nearest = f64::INFINITY;
    for obs in obstacles {
        let (dx, dy) = (obs.x - state.x, obs.y - state.y);
        let lx = dx * state.theta.cos() + dy * state.theta.sin();
        let ly = -dx * state.theta.sin() + dy * state.theta.cos();
        if lx <= 0.0 {
            continue; // not frontal
        }
        let d = obs.net_distance_at(state.x, state.y, 0.0);
        if d < FRONTAL_RANGE_M && d < nearest && ly.abs() > 1e-9 {
            nearest = d;
            side = ly.signum();
        }
    }

    let w_mag = speed * max_curvature;
    let candidates: &[f64] = if side != 0.0 {
        &[-side * w_mag, 0.0, side * w_mag]
    } else {
        &[0.0]
    };

    let mut worst: Option<RearBlocker> = None;
    for &w in candidates {
        let kappa = w / speed;
        match reverse_arc_blocker(
            state,
            obstacles,
            robot_radius,
            margin,
            moving_margin_gain,
            reverse_dist,
            kappa,
        ) {
            None => {
                return Ok(VelocityCommand {
                    linear_x: -speed,
                    angular_z: w,
                    confidence: 0.9,
                })
            }
            Some(b) => {
                if worst.is_none_or(|prev| b.surface_dist < prev.surface_dist) {
                    worst = Some(b);
                }
            }
        }
    }
    // candidates is never empty, so a blocker was recorded.
    Err(worst.expect("blocked escape must record a blocker"))
}

/// Forward-integrate a (v, ω) command from `state` over `horizon` at step `dt`.
/// Stops early on NaN/Inf to avoid poisoning the published trajectory.
fn rollout(state: &RobotState, cmd: &VelocityCommand, horizon: f64, dt: f64) -> Vec<TrajPoint> {
    if horizon <= 0.0 || dt <= 0.0 {
        return Vec::new();
    }
    let steps = (horizon / dt) as usize;
    let mut out = Vec::with_capacity(steps);
    let (mut x, mut y, mut theta) = (state.x, state.y, state.theta);
    for _ in 0..steps {
        x += cmd.linear_x * theta.cos() * dt;
        y += cmd.linear_x * theta.sin() * dt;
        theta += cmd.angular_z * dt;
        if !x.is_finite() || !y.is_finite() || !theta.is_finite() {
            break;
        }
        out.push(TrajPoint { x, y, theta });
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rollout_straight_line_advances_along_x() {
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let cmd = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let traj = rollout(&state, &cmd, 1.0, 0.1);
        assert_eq!(traj.len(), 10);
        // After 1s at 0.5 m/s, x should be ~0.5.
        let last = traj.last().unwrap();
        assert!((last.x - 0.5).abs() < 1e-9);
        assert!(last.y.abs() < 1e-9);
        assert!(last.theta.abs() < 1e-9);
    }

    #[test]
    fn rollout_pure_rotation_keeps_position() {
        let state = RobotState {
            x: 1.0,
            y: 2.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let cmd = VelocityCommand {
            linear_x: 0.0,
            angular_z: 1.0,
            confidence: 0.9,
        };
        let traj = rollout(&state, &cmd, 1.0, 0.1);
        let last = traj.last().unwrap();
        assert!((last.x - 1.0).abs() < 1e-9);
        assert!((last.y - 2.0).abs() < 1e-9);
        assert!((last.theta - 1.0).abs() < 1e-9);
    }

    #[test]
    fn empty_path_is_confident_stop_not_planner_failure() {
        // Idle (no global path) must publish a confident zero command; a low
        // confidence here would trip the arbitrator's fallback E-stop while
        // the robot is merely waiting for a goal.
        let mut planner = LocalPlanner::new(LocalPlannerConfig::default());
        let plan = planner.compute(&RobotState::default(), &[], &[], 0.5);
        assert_eq!(plan.command.linear_x, 0.0);
        assert_eq!(plan.command.angular_z, 0.0);
        assert!(plan.command.confidence >= 0.9);
        assert!(plan.trajectory.is_empty());
    }

    #[test]
    fn rear_corridor_blocked_by_obstacle_behind() {
        // Robot facing +x; obstacle 0.2m directly behind: reverse must be
        // vetoed.
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let behind = vec![Obstacle::point(-0.2, 0.0)];
        assert!(!rear_corridor_clear(&state, &behind, 0.22, 0.05, 0.4, 0.3));

        // An obstacle just past the corridor end but inside the inflation
        // radius still blocks (the robot body sweeps to -0.3).
        let at_end = vec![Obstacle::point(-0.45, 0.0)];
        assert!(!rear_corridor_clear(&state, &at_end, 0.22, 0.05, 0.4, 0.3));

        // A fat obstacle whose CENTER is laterally outside the corridor but
        // whose surface reaches in: blocked (surface distance, not center).
        let fat = vec![Obstacle {
            x: -0.2,
            y: 0.5,
            vx: 0.0,
            vy: 0.0,
            radius: 0.3,
            ..Default::default()
        }];
        assert!(!rear_corridor_clear(&state, &fat, 0.22, 0.05, 0.4, 0.3));
    }

    #[test]
    fn rear_corridor_clear_when_obstacles_ahead_or_offset() {
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        // Obstacle in FRONT (the thing we are backing away from): clear.
        let front = vec![Obstacle::point(0.3, 0.0)];
        assert!(rear_corridor_clear(&state, &front, 0.22, 0.05, 0.4, 0.3));
        // Laterally offset well outside robot_radius + margin: clear.
        let side = vec![Obstacle::point(-0.2, 0.6)];
        assert!(rear_corridor_clear(&state, &side, 0.22, 0.05, 0.4, 0.3));
        // Far behind, beyond the swept distance + inflation: clear.
        let far = vec![Obstacle::point(-1.0, 0.0)];
        assert!(rear_corridor_clear(&state, &far, 0.22, 0.05, 0.4, 0.3));
        // No obstacles at all: clear.
        assert!(rear_corridor_clear(&state, &[], 0.22, 0.05, 0.4, 0.3));

        // Heading matters: rotate the robot 180° and the "front" obstacle is
        // now behind it.
        let flipped = RobotState {
            theta: std::f64::consts::PI,
            ..state
        };
        assert!(!rear_corridor_clear(&flipped, &front, 0.22, 0.05, 0.4, 0.3));
    }

    #[test]
    fn rear_corridor_clear_when_wedged_against_frontal_cone() {
        // Regression for the Gazebo gauntlet freeze: robot at origin heading
        // +x, wedged against a tracked cone (extent radius 0.12) 0.35m ahead-
        // left at 45°, centerline wall points 0.6m to the RIGHT-REAR running
        // parallel to the heading, rear corridor straight back open for 1.5m.
        // The old segment check measured the frontal cone against the robot
        // CENTER (surface distance 0.35 - 0.12 = 0.23 < 0.27 threshold) and
        // refused the reverse 13 times in a row. Backing up strictly moves
        // AWAY from that cone: the corridor must be CLEAR.
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let bearing = std::f64::consts::FRAC_PI_4;
        let mut obstacles = vec![Obstacle {
            x: 0.35 * bearing.cos(),
            y: 0.35 * bearing.sin(),
            vx: 0.0,
            vy: 0.0,
            radius: 0.12,
            ..Default::default()
        }];
        // Wall points 0.6m laterally on the right, side-rear, parallel to +x.
        let mut x = -1.5;
        while x <= 0.5 {
            obstacles.push(Obstacle::point(x, -0.6));
            x += 0.1;
        }
        assert!(
            rear_corridor_clear(&state, &obstacles, 0.22, 0.05, 0.4, 0.3),
            "open rear corridor vetoed by the frontal cone the robot is escaping from"
        );
    }

    #[test]
    fn reverse_escape_steers_nose_away_from_frontal_obstacle() {
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let max_curvature = 2.0;
        let speed = 0.1;

        // Frontal obstacle on the LEFT: reverse arc must rotate the nose
        // right (angular_z < 0) within the reverse curvature envelope.
        let left = vec![Obstacle::point(0.3, 0.2)];
        let cmd = plan_reverse_escape(&state, &left, 0.22, 0.05, 0.4, 0.3, speed, max_curvature)
            .expect("open rear must yield an escape");
        assert_eq!(cmd.linear_x, -speed);
        assert!(
            cmd.angular_z < 0.0,
            "nose must rotate away from left obstacle"
        );
        assert!(cmd.angular_z.abs() <= speed * max_curvature + 1e-9);

        // Frontal obstacle on the RIGHT: mirrored.
        let right = vec![Obstacle::point(0.3, -0.2)];
        let cmd = plan_reverse_escape(&state, &right, 0.22, 0.05, 0.4, 0.3, speed, max_curvature)
            .expect("open rear must yield an escape");
        assert!(
            cmd.angular_z > 0.0,
            "nose must rotate away from right obstacle"
        );

        // No frontal obstacle identifying a side: straight reverse.
        let none: Vec<Obstacle> = Vec::new();
        let cmd = plan_reverse_escape(&state, &none, 0.22, 0.05, 0.4, 0.3, speed, max_curvature)
            .expect("empty world must yield an escape");
        assert_eq!(cmd.angular_z, 0.0, "no side identified => straight reverse");
    }

    #[test]
    fn reverse_escape_sweeps_actual_arc_and_falls_back_to_straight() {
        // Frontal obstacle on the LEFT prefers the back-left arc (nose right).
        // An obstacle sitting ON that arc — but 0.30m laterally off the
        // straight corridor, i.e. clear of a straight-rectangle check — must
        // reject the arc; the planner then falls back to the straight reverse.
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let obstacles = vec![
            Obstacle::point(0.3, 0.2),   // frontal-left: prefer arc with w < 0
            Obstacle::point(-0.25, 0.3), // on the back-left arc path
        ];
        let cmd = plan_reverse_escape(&state, &obstacles, 0.22, 0.05, 0.4, 0.3, 0.1, 2.0)
            .expect("straight corridor is open");
        assert_eq!(
            cmd.angular_z, 0.0,
            "arc blocked by swept check must fall back to straight reverse"
        );

        // Fully blocked rear: refusal must name the blocking obstacle.
        let boxed = vec![
            Obstacle::point(0.3, 0.2),
            Obstacle::point(-0.2, 0.0), // dead behind
        ];
        let blocker = plan_reverse_escape(&state, &boxed, 0.22, 0.05, 0.4, 0.3, 0.1, 2.0)
            .expect_err("blocked rear must refuse");
        assert!((blocker.x - (-0.2)).abs() < 1e-9);
        assert!(blocker.surface_dist < 0.27);
    }

    #[test]
    fn reverse_sweep_moving_obstacle_demands_wider_berth() {
        // An obstacle 0.35m laterally off the swept reverse corridor: static
        // it clears the 0.22 + 0.05 = 0.27 threshold, but moving at 0.75 m/s
        // it raises its own threshold by 0.4 * 0.75 = 0.30 to 0.57 and must
        // veto the same reverse. Crawl relief near static geometry is
        // preserved; a pedestrian near the corridor is not granted it.
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.0,
            angular_vel: 0.0,
        };
        let static_side = vec![Obstacle::point(-0.2, 0.35)];
        assert!(
            rear_corridor_clear(&state, &static_side, 0.22, 0.05, 0.4, 0.3),
            "static obstacle 0.35m off the corridor must not veto the reverse"
        );
        let moving_side = vec![Obstacle {
            x: -0.2,
            y: 0.35,
            vx: 0.0,
            vy: 0.75,
            radius: 0.0,
            ..Default::default()
        }];
        assert!(
            !rear_corridor_clear(&state, &moving_side, 0.22, 0.05, 0.4, 0.3),
            "0.75 m/s obstacle at the same distance must veto the reverse"
        );
        // Gain 0 restores the old speed-blind behavior (control experiment).
        assert!(rear_corridor_clear(
            &state,
            &moving_side,
            0.22,
            0.05,
            0.0,
            0.3
        ));
    }

    #[test]
    fn obstacle_speed_magnitude() {
        assert_eq!(Obstacle::point(1.0, 2.0).speed(), 0.0);
        let ped = Obstacle {
            x: 0.0,
            y: 0.0,
            vx: 0.45,
            vy: -0.6,
            radius: 0.15,
            ..Default::default()
        };
        assert!((ped.speed() - 0.75).abs() < 1e-12);
    }

    /// Synthetic S-curve: two joined quarter-circle arcs of radius 1.0m —
    /// left from the origin (heading +x) to (1,1) heading +y, then right to
    /// (2,2) heading +x. Sampled every 0.05 rad (~5cm).
    fn s_curve_path() -> Vec<PathWaypoint> {
        let mut path = Vec::new();
        let step = 0.05;
        let quarter = std::f64::consts::FRAC_PI_2;
        let mut t = step;
        while t <= quarter {
            path.push(PathWaypoint {
                x: t.sin(),
                y: 1.0 - t.cos(),
                theta: t,
                steering: 0.0,
                dir: Default::default(),
            });
            t += step;
        }
        let mut t = step;
        while t <= quarter {
            path.push(PathWaypoint {
                x: 2.0 - t.cos(),
                y: 1.0 + t.sin(),
                theta: quarter - t,
                steering: 0.0,
                dir: Default::default(),
            });
            t += step;
        }
        path
    }

    /// Pursuit-focused tests: disable the tracking-MPC primary so the
    /// pursuit behaviors under test stay observable.
    fn pursuit_only() -> LocalPlannerConfig {
        let mut cfg = LocalPlannerConfig::default();
        cfg.tracking_mpc.enabled = false;
        cfg
    }

    #[test]
    fn s_curve_pursuit_tracks_apex_without_zero_speed() {
        // The apex-hitch regression test: eleven gauntlet attempts showed
        // constant-arc DWA sampling hitching at every slalom apex (the
        // S-transition counter-steer is not representable as one (v, w)
        // sample), churning the stuck detector. Pure pursuit must drive the
        // synthetic S end-to-end in an obstacle-free corridor WITHOUT EVER
        // commanding zero speed, staying on the pursuit executor throughout.
        let mut planner = LocalPlanner::new(pursuit_only());
        let path = s_curve_path();
        let goal = path.last().unwrap().clone();
        let mut state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };
        let dt = 0.1;
        let mut reached = false;
        for cycle in 0..120 {
            let dist_to_goal = ((state.x - goal.x).powi(2) + (state.y - goal.y).powi(2)).sqrt();
            if dist_to_goal < 0.65 {
                reached = true;
                break;
            }
            let plan = planner.compute(&state, &path, &[], 1.5);
            assert_eq!(
                plan.executor,
                Executor::Pursuit,
                "cycle {}: expected the pursuit executor on an open S-curve",
                cycle
            );
            assert!(
                plan.command.linear_x > 0.05,
                "cycle {}: apex hitch — commanded near-zero speed {} at ({:.2},{:.2})",
                cycle,
                plan.command.linear_x,
                state.x,
                state.y
            );
            assert!(
                plan.command.angular_z.abs()
                    <= plan.command.linear_x * LocalPlannerConfig::default().dwa.max_curvature
                        + 1e-9,
                "cycle {}: command outside the curvature envelope",
                cycle
            );
            // Kinematic closed loop at the 10Hz cycle.
            state.x += plan.command.linear_x * state.theta.cos() * dt;
            state.y += plan.command.linear_x * state.theta.sin() * dt;
            state.theta += plan.command.angular_z * dt;
            state.linear_vel = plan.command.linear_x;
            state.angular_vel = plan.command.angular_z;
        }
        assert!(
            reached,
            "S-curve not completed: stalled at ({:.2},{:.2})",
            state.x, state.y
        );
    }

    /// Distance from point (px, py) to the segment a→b.
    fn point_segment_dist(px: f64, py: f64, a: &PathWaypoint, b: &PathWaypoint) -> f64 {
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let len2 = dx * dx + dy * dy;
        let t = if len2 < 1e-12 {
            0.0
        } else {
            (((px - a.x) * dx + (py - a.y) * dy) / len2).clamp(0.0, 1.0)
        };
        let (cx, cy) = (a.x + t * dx, a.y + t * dy);
        ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
    }

    /// Closed-loop S-curve run at the given curvature-lookahead gain,
    /// returning the maximum cross-track error (m) to the reference polyline.
    fn s_curve_max_cross_track(gain: f64) -> f64 {
        let mut cfg = pursuit_only();
        cfg.pursuit.curvature_lookahead_gain = gain;
        let mut planner = LocalPlanner::new(cfg);
        let path = s_curve_path();
        let goal = path.last().unwrap().clone();
        let mut state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.5,
            angular_vel: 0.0,
        };
        let dt = 0.1;
        let mut worst = 0.0f64;
        for _ in 0..120 {
            if ((state.x - goal.x).powi(2) + (state.y - goal.y).powi(2)).sqrt() < 0.65 {
                break;
            }
            let plan = planner.compute(&state, &path, &[], 1.5);
            state.x += plan.command.linear_x * state.theta.cos() * dt;
            state.y += plan.command.linear_x * state.theta.sin() * dt;
            state.theta += plan.command.angular_z * dt;
            state.linear_vel = plan.command.linear_x;
            state.angular_vel = plan.command.angular_z;
            let cross = path
                .windows(2)
                .map(|w| point_segment_dist(state.x, state.y, &w[0], &w[1]))
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(cross);
        }
        worst
    }

    #[test]
    fn s_curve_curvature_lookahead_reduces_cross_track_error() {
        // (h) The curvature-adaptive lookahead's observable payoff: with the
        // shipped gain the S-curve closed loop must track the reference
        // strictly tighter than the speed-only control run (gain = 0), whose
        // long cruise lookahead chords across the r = 1.0 arcs.
        let gain = LocalPlannerConfig::default()
            .pursuit
            .curvature_lookahead_gain;
        assert!(gain > 0.0, "shipped default must enable the shrink");
        let adaptive = s_curve_max_cross_track(gain);
        let speed_only = s_curve_max_cross_track(0.0);
        assert!(
            adaptive < speed_only - 0.05,
            "curvature-adaptive lookahead must cut cross-track error: adaptive {:.3} vs speed-only {:.3}",
            adaptive,
            speed_only
        );
    }

    #[test]
    fn local_planner_config_cross_validation() {
        assert!(LocalPlannerConfig::default().validate().is_ok());
        // A turning-speed floor above the platform speed limit is a typo.
        let mut cfg = LocalPlannerConfig::default();
        cfg.pursuit.v_turn_min = cfg.dwa.max_speed + 0.1;
        assert!(cfg.validate().is_err());
        // Pursuit inverts steering through the wheelbase — 0 poisons every κ.
        let mut cfg = LocalPlannerConfig::default();
        cfg.mpc.wheelbase = 0.0;
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn blocked_pursuit_defers_to_dwa_executor() {
        // Wall dead ahead at 0.35m spanning the corridor: pursuit steps down
        // through every speed fraction and the crawl, all fail the shared
        // rollout, and arbitration must hand the cycle to DWA (exec=dwa) —
        // whose own infeasibility verdict then feeds the recovery machinery
        // exactly as before pursuit existed.
        let mut planner = LocalPlanner::new(pursuit_only());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path: Vec<PathWaypoint> = (1..=30)
            .map(|i| PathWaypoint {
                x: 0.1 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect();
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(Obstacle::point(0.35, y));
            y += 0.05;
        }
        let plan = planner.compute(&state, &path, &wall, 0.5);
        assert_eq!(
            plan.executor,
            Executor::Dwa,
            "blocked pursuit must defer to the DWA fallback"
        );

        // Control: the same scene without the wall is pursuit's.
        let open = planner.compute(&state, &path, &[], 0.5);
        assert_eq!(open.executor, Executor::Pursuit);
        assert!(open.command.linear_x > 0.0);
    }

    #[test]
    fn net_distance_shape_awareness() {
        // Axis-aligned box 0.4x0.2 at origin: face distances, corner
        // distance, inside sign, and the circle fallback.
        let b = Obstacle::boxed(0.0, 0.0, 0.2, 0.1, 0.0);
        assert!((b.net_distance_at(0.5, 0.0, 0.0) - 0.3).abs() < 1e-9);
        assert!((b.net_distance_at(0.0, 0.5, 0.0) - 0.4).abs() < 1e-9);
        let corner = ((0.3f64).powi(2) + (0.2f64).powi(2)).sqrt();
        assert!((b.net_distance_at(0.5, 0.3, 0.0) - corner).abs() < 1e-9);
        assert!(b.net_distance_at(0.0, 0.0, 0.0) < 0.0, "inside is negative");
        // Rotated 90°: the long axis now lies along world y.
        let r = Obstacle::boxed(0.0, 0.0, 0.2, 0.1, std::f64::consts::FRAC_PI_2);
        assert!((r.net_distance_at(0.5, 0.0, 0.0) - 0.4).abs() < 1e-9);
        assert!((r.net_distance_at(0.0, 0.5, 0.0) - 0.3).abs() < 1e-9);
        // Circle fallback (no half extents): distance minus radius.
        let c = Obstacle {
            x: 0.0,
            y: 0.0,
            radius: 0.25,
            ..Default::default()
        };
        assert!((c.net_distance_at(1.0, 0.0, 0.0) - 0.75).abs() < 1e-9);
        // The box's circumscribed-circle fallback radius overstates the
        // face clearance — the exact overshoot the rectangle removes.
        assert!(b.radius > 0.2 && b.radius < 0.23);
    }

    #[test]
    fn project_state_arc_and_straight() {
        // Straight: 1.5 m/s for 0.2s along +x.
        let s = RobotState {
            x: 1.0,
            y: 2.0,
            theta: 0.0,
            linear_vel: 1.5,
            angular_vel: 0.0,
        };
        let p = project_state(&s, 0.2);
        assert!((p.x - 1.3).abs() < 1e-9 && (p.y - 2.0).abs() < 1e-9);
        // Arc: quarter circle check — v=1, w=PI/2 over 1s turns 90° with
        // radius 2/PI; endpoint at (r, r) from a +x start.
        let s = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 1.0,
            angular_vel: std::f64::consts::FRAC_PI_2,
        };
        let p = project_state(&s, 1.0);
        let r = 1.0 / std::f64::consts::FRAC_PI_2;
        assert!((p.x - r).abs() < 1e-9, "arc x {} vs {}", p.x, r);
        assert!((p.y - r).abs() < 1e-9, "arc y {} vs {}", p.y, r);
        assert!((p.theta - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        // Zero delay: identity.
        let p = project_state(&s, 0.0);
        assert_eq!((p.x, p.y), (0.0, 0.0));
    }

    #[test]
    fn scripted_and_relaxed_plans_carry_executor_labels() {
        let mut planner = LocalPlanner::new(LocalPlannerConfig::default());
        let state = RobotState::default();
        let scripted = planner.scripted_plan(
            &state,
            VelocityCommand {
                linear_x: -0.1,
                angular_z: 0.0,
                confidence: 0.9,
            },
        );
        assert_eq!(scripted.executor, Executor::Scripted);

        let path = vec![PathWaypoint {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let relaxed = planner.compute_relaxed(&state, &path, &[], 0.1);
        assert_eq!(relaxed.executor, Executor::Dwa);

        // Executor names as the cycle log prints them.
        assert_eq!(Executor::Pursuit.to_string(), "pursuit");
        assert_eq!(Executor::Dwa.to_string(), "dwa");
        assert_eq!(Executor::Mpc.to_string(), "mpc");
        assert_eq!(Executor::Scripted.to_string(), "scripted");
    }

    /// (e) Two-segment maneuver path: reverse 0.5m straight back, cusp,
    /// then forward through a radius-1.0 quarter-turn. Executed closed-loop
    /// in a kinematic sim with the main-loop truncation applied each cycle:
    /// the robot must reverse, stop at the cusp, then proceed forward — and
    /// never exceed the envelope (reverse cap, curvature, accel limits).
    #[test]
    fn two_segment_reverse_then_forward_closed_loop() {
        use crate::global_planner::SegmentDir;

        let cfg = pursuit_only();
        let (max_acc, max_dec, max_kappa, max_speed) = (
            cfg.dwa.max_acceleration,
            cfg.dwa.max_deceleration,
            cfg.dwa.max_curvature,
            cfg.dwa.max_speed,
        );
        let reverse_cap = cfg.pursuit.reverse_speed_cap;
        let mut planner = LocalPlanner::new(cfg);

        let mut path: Vec<PathWaypoint> = Vec::new();
        for i in 1..=10 {
            path.push(PathWaypoint {
                x: -0.05 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Reverse,
            });
        }
        // Forward quarter-arc, radius 1.0, from (-0.5, 0) heading 0 turning
        // left to (0.5, 1.0) heading π/2.
        let mut t = 0.05;
        while t <= std::f64::consts::FRAC_PI_2 {
            path.push(PathWaypoint {
                x: -0.5 + t.sin(),
                y: 1.0 - t.cos(),
                theta: t,
                steering: 0.0,
                dir: SegmentDir::Forward,
            });
            t += 0.05;
        }
        let goal = path.last().unwrap().clone();

        let mut state = RobotState::default();
        let mut wpath = path.clone();
        let dt = 0.1;
        let mut prev_v = 0.0f64;
        let (mut reversed, mut stopped_at_cusp, mut forward_after_stop) = (false, false, false);
        let mut reached = false;

        for cycle in 0..300 {
            // Main-loop-style per-cycle truncation (active run scoped).
            let near = active_run_truncation_index(&wpath, state.x, state.y);
            if near > 0 {
                wpath.drain(..near);
            }
            let d_goal = ((state.x - goal.x).powi(2) + (state.y - goal.y).powi(2)).sqrt();
            if forward_after_stop && d_goal < 0.4 {
                reached = true;
                break;
            }

            let plan = planner.compute(&state, &wpath, &[], 1.0);
            let (v, w) = (plan.command.linear_x, plan.command.angular_z);

            // Envelope: speed, curvature, angular rate, accel continuity.
            assert!(v.abs() <= max_speed + 1e-9, "cycle {cycle}: |v|={v}");
            assert!(
                w.abs() <= v.abs() * max_kappa + 1e-9,
                "cycle {cycle}: curvature envelope violated v={v} w={w}"
            );
            assert!(
                (v - prev_v).abs() <= max_acc.max(max_dec) * dt + 1e-6,
                "cycle {cycle}: accel jump {prev_v} -> {v}"
            );

            if v < -0.05 {
                assert!(
                    v >= -reverse_cap - 1e-9,
                    "cycle {cycle}: reverse cap violated v={v}"
                );
                assert_eq!(plan.executor, Executor::Pursuit, "cycle {cycle}");
                assert!(plan.planned_maneuver, "cycle {cycle}: reverse not flagged");
                assert!(
                    !forward_after_stop,
                    "cycle {cycle}: reverse after forward leg"
                );
                reversed = true;
            }
            if reversed && !forward_after_stop && v.abs() < 1e-9 {
                stopped_at_cusp = true;
            }
            if stopped_at_cusp && v > 0.05 {
                forward_after_stop = true;
            }

            state.x += v * state.theta.cos() * dt;
            state.y += v * state.theta.sin() * dt;
            state.theta += w * dt;
            state.linear_vel = v;
            state.angular_vel = w;
            prev_v = v;
        }
        assert!(reversed, "robot never executed the reverse segment");
        assert!(stopped_at_cusp, "robot never stopped at the cusp");
        assert!(
            forward_after_stop,
            "robot never proceeded forward after the cusp"
        );
        assert!(
            reached,
            "goal not reached: stalled at ({:.2},{:.2})",
            state.x, state.y
        );
    }

    #[test]
    fn blocked_reverse_segment_stops_instead_of_dwa() {
        // A blocked REVERSE segment has no forward-sampler fallback: DWA/MPC
        // would chase a local goal behind the robot. The local planner must
        // emit the DWA-style infeasible stop (near-zero, low confidence, NOT
        // flagged as a maneuver) so the stuck detector and recovery see the
        // blockage.
        use crate::global_planner::SegmentDir;
        let mut planner = LocalPlanner::new(LocalPlannerConfig::default());
        let path: Vec<PathWaypoint> = (1..=20)
            .map(|i| PathWaypoint {
                x: -0.05 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Reverse,
            })
            .collect();
        // Wall right behind the robot, blocking every reverse speed step.
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(Obstacle::point(-0.3, y));
            y += 0.05;
        }
        let plan = planner.compute(&RobotState::default(), &path, &wall, 0.5);
        assert_eq!(plan.executor, Executor::Pursuit);
        assert_eq!(plan.command.linear_x, 0.0);
        assert!(
            plan.command.confidence <= 0.2,
            "blockage must read infeasible"
        );
        assert!(!plan.planned_maneuver);

        // Control: without the wall the reverse segment executes (negative
        // command, flagged as a planned maneuver).
        let open = planner.compute(&RobotState::default(), &path, &[], 0.5);
        assert!(open.command.linear_x < 0.0);
        assert!(open.planned_maneuver);
    }

    #[test]
    fn truncation_is_scoped_to_the_active_run() {
        // The forward leg of a maneuver path passes back through the robot's
        // start area; the truncation must keep the not-yet-executed reverse
        // leg instead of snapping to the spatially-nearest forward waypoint.
        use crate::global_planner::SegmentDir;
        let mut path: Vec<PathWaypoint> = Vec::new();
        for i in 1..=10 {
            path.push(PathWaypoint {
                x: -0.05 * i as f64,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Reverse,
            });
        }
        for i in 0..=20 {
            path.push(PathWaypoint {
                x: -0.5 + 0.1 * i as f64,
                y: 0.02, // passes 2cm from the robot start pose
                theta: 0.0,
                steering: 0.0,
                dir: SegmentDir::Forward,
            });
        }
        // Robot at the origin: the nearest waypoint GLOBALLY is on the
        // forward leg (0.0, 0.02); scoped truncation must pick index 0 of
        // the reverse run instead.
        assert_eq!(active_run_truncation_index(&path, 0.0, 0.0), 0);
        // Robot mid-reverse: truncates within the reverse run.
        assert_eq!(active_run_truncation_index(&path, -0.26, 0.0), 4);
        // Empty path: 0.
        assert_eq!(active_run_truncation_index(&[], 0.0, 0.0), 0);
    }

    #[test]
    fn rollout_empty_horizon_is_empty() {
        let state = RobotState::default();
        let cmd = VelocityCommand {
            linear_x: 1.0,
            angular_z: 0.0,
            confidence: 0.9,
        };
        assert!(rollout(&state, &cmd, 0.0, 0.1).is_empty());
    }
}
