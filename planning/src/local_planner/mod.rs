/// Local planner: DWA primary + simplified MPC fallback.
///
/// DWA runs at 10Hz for reactive obstacle avoidance.
/// MPC activates for tight maneuvers (parking, U-turns) where DWA's
/// short horizon is insufficient.
pub mod dwa;
pub mod mpc;

use serde::Deserialize;

use crate::global_planner::PathWaypoint;

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

/// Local planner output: the velocity command plus the predicted trajectory
/// obtained by forward-integrating that command from the current state. The
/// trajectory is forwarded to the tracker (feed-forward) and published on CH10
/// for visualization.
#[derive(Debug, Clone, Default)]
pub struct LocalPlan {
    pub command: VelocityCommand,
    pub trajectory: Vec<TrajPoint>,
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

/// Obstacle from perception: either an untracked point sample (radius 0,
/// zero velocity — walls, sector-sampled returns) or a tracked object with
/// an extent radius and a world-frame velocity estimate.
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub radius: f64,
}

impl Obstacle {
    /// Untracked point sample.
    #[allow(dead_code)] // convenience constructor used by unit tests
    pub fn point(x: f64, y: f64) -> Self {
        Self {
            x,
            y,
            vx: 0.0,
            vy: 0.0,
            radius: 0.0,
        }
    }

    /// Position propagated along the velocity estimate by `t` seconds.
    pub fn position_at(&self, t: f64) -> (f64, f64) {
        (self.x + self.vx * t, self.y + self.vy * t)
    }

    /// Extent radius grown by prediction uncertainty at lookahead `t`:
    /// constant-velocity prediction is exact on straight legs and wrong when
    /// the object turns or reverses, so the possible deviation grows with
    /// how far the object travels in the prediction window. Measured: a
    /// pedestrian turning a corner mid-prediction was clipped at -0.128m
    /// ground truth without this.
    pub fn effective_radius_at(&self, t: f64) -> f64 {
        let speed = (self.vx * self.vx + self.vy * self.vy).sqrt();
        self.radius + PREDICTION_UNCERTAINTY * speed * t
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
    #[serde(default = "default_mpc_trigger_curvature")]
    pub mpc_trigger_curvature: f64, // use MPC when path curvature exceeds this
}

fn default_rate_hz() -> u32 {
    10
}
fn default_mpc_trigger_curvature() -> f64 {
    1.5
} // 1/m

impl Default for LocalPlannerConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            dwa: dwa::DwaConfig::default(),
            mpc: mpc::MpcConfig::default(),
            mpc_trigger_curvature: default_mpc_trigger_curvature(),
        }
    }
}

/// Unified local planner with DWA + MPC fallback.
pub struct LocalPlanner {
    config: LocalPlannerConfig,
    dwa_planner: dwa::DwaPlanner,
    mpc_planner: mpc::SimpleMpc,
    use_mpc: bool,
}

impl LocalPlanner {
    pub fn new(config: LocalPlannerConfig) -> Self {
        let dwa_planner = dwa::DwaPlanner::new(config.dwa.clone());
        let mpc_planner = mpc::SimpleMpc::new(config.mpc.clone());
        Self {
            config,
            dwa_planner,
            mpc_planner,
            use_mpc: false,
        }
    }

    /// Compute a velocity command + predicted trajectory.
    /// Empty path → zero command, empty trajectory.
    pub fn compute(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> LocalPlan {
        if path.is_empty() {
            return LocalPlan::default();
        }

        // Determine if MPC should take over based on path curvature
        self.use_mpc = self.should_use_mpc(state, path);

        let command = if self.use_mpc {
            self.mpc_planner
                .compute(state, path, obstacles, desired_speed)
        } else {
            self.dwa_planner
                .compute(state, path, obstacles, desired_speed)
        };

        let trajectory = rollout(
            state,
            &command,
            self.config.dwa.sim_time,
            self.config.dwa.sim_dt,
        );

        LocalPlan {
            command,
            trajectory,
        }
    }

    /// Check if MPC should be used (tight curvature ahead).
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

    pub fn is_using_mpc(&self) -> bool {
        self.use_mpc
    }
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
