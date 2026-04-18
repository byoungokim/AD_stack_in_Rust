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
    pub angular_z: f64,   // rad/s
    pub confidence: f32,  // [0.0, 1.0]
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

/// Obstacle as a 2D point (from perception).
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub x: f64,
    pub y: f64,
}

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

fn default_rate_hz() -> u32 { 10 }
fn default_mpc_trigger_curvature() -> f64 { 1.5 } // 1/m

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

    /// Compute a velocity command given robot state, path, and obstacles.
    pub fn compute(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> VelocityCommand {
        if path.is_empty() {
            return VelocityCommand::default();
        }

        // Determine if MPC should take over based on path curvature
        self.use_mpc = self.should_use_mpc(state, path);

        if self.use_mpc {
            self.mpc_planner.compute(state, path, obstacles, desired_speed)
        } else {
            self.dwa_planner.compute(state, path, obstacles, desired_speed)
        }
    }

    /// Check if MPC should be used (tight curvature ahead).
    fn should_use_mpc(&self, _state: &RobotState, path: &[PathWaypoint]) -> bool {
        // Look at the next few waypoints for sharp turns
        let lookahead = 5.min(path.len());
        for i in 1..lookahead {
            let dtheta = (path[i].theta - path[i - 1].theta).abs();
            let dist = ((path[i].x - path[i - 1].x).powi(2)
                + (path[i].y - path[i - 1].y).powi(2))
            .sqrt();

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
