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

    // Safety
    #[serde(default = "default_robot_radius")]
    pub robot_radius: f64, // meters, for collision checking

    /// Executable-curvature envelope (1/m): sampled (v, w) pairs with
    /// |w| > v * max_curvature are skipped. Must not exceed the arbitrator's
    /// safety-envelope max_curvature nor the Ackermann steering limit
    /// tan(max_steering)/wheelbase (2.6 for the Limo Pro) — otherwise DWA
    /// verifies a trajectory, downstream clamps the command, and the real
    /// arc swings wider than the verified one, clipping the inside of turns.
    #[serde(default = "default_max_curvature")]
    pub max_curvature: f64, // 1/m
}

fn default_max_speed() -> f64 {
    1.0
}
fn default_max_angular() -> f64 {
    1.5
}
fn default_max_accel() -> f64 {
    0.5
}
fn default_max_angular_accel() -> f64 {
    2.0
}
fn default_sim_time() -> f64 {
    1.5
}
fn default_sim_dt() -> f64 {
    0.1
}
fn default_v_samples() -> usize {
    11
}
fn default_w_samples() -> usize {
    21
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
fn default_robot_radius() -> f64 {
    0.2
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
            max_angular_accel: default_max_angular_accel(),
            sim_time: default_sim_time(),
            sim_dt: default_sim_dt(),
            v_samples: default_v_samples(),
            w_samples: default_w_samples(),
            heading_weight: default_heading_weight(),
            distance_weight: default_distance_weight(),
            velocity_weight: default_velocity_weight(),
            obstacle_weight: default_obstacle_weight(),
            robot_radius: default_robot_radius(),
            max_curvature: default_max_curvature(),
        }
    }
}

/// Simulated trajectory for scoring.
struct SimTrajectory {
    v: f64,
    w: f64,
    end_x: f64,
    end_y: f64,
    end_theta: f64,
    min_obstacle_dist: f64,
    score: f64,
}

pub struct DwaPlanner {
    config: DwaConfig,
}

impl DwaPlanner {
    pub fn new(config: DwaConfig) -> Self {
        Self { config }
    }

    pub fn compute(
        &self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> VelocityCommand {
        let target_speed = desired_speed.min(self.config.max_speed);

        // Find the local goal point on the path (lookahead)
        let goal = find_local_goal(state, path);

        // Compute dynamic window
        let dt = 1.0 / 10.0; // control cycle
        let v_min = (state.linear_vel - self.config.max_acceleration * dt).max(0.0);
        let v_max = (state.linear_vel + self.config.max_acceleration * dt).min(target_speed);
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

                // Skip if collision
                if traj.min_obstacle_dist < self.config.robot_radius {
                    continue;
                }

                // Score trajectory
                let heading_score = heading_cost(traj.end_x, traj.end_y, traj.end_theta, &goal);
                let distance_score = distance_to_goal(traj.end_x, traj.end_y, &goal);
                let velocity_score = v / target_speed.max(0.1);
                let obstacle_score = traj.min_obstacle_dist.min(3.0) / 3.0;

                let score = self.config.heading_weight * heading_score
                    - self.config.distance_weight * distance_score
                    + self.config.velocity_weight * velocity_score
                    + self.config.obstacle_weight * obstacle_score;

                let scored = SimTrajectory { score, ..traj };

                if best.as_ref().is_none_or(|b| scored.score > b.score) {
                    best = Some(scored);
                }
            }
        }

        match best {
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
        }
    }

    /// Simulate a trajectory forward with constant (v, w).
    fn simulate(
        &self,
        state: &RobotState,
        v: f64,
        w: f64,
        obstacles: &[Obstacle],
    ) -> SimTrajectory {
        let mut x = state.x;
        let mut y = state.y;
        let mut theta = state.theta;
        let mut min_dist = f64::INFINITY;

        let steps = (self.config.sim_time / self.config.sim_dt) as usize;

        for _ in 0..steps {
            x += v * theta.cos() * self.config.sim_dt;
            y += v * theta.sin() * self.config.sim_dt;
            theta += w * self.config.sim_dt;

            // Check obstacle distances
            for obs in obstacles {
                let d = ((x - obs.x).powi(2) + (y - obs.y).powi(2)).sqrt();
                if d < min_dist {
                    min_dist = d;
                }
            }
        }

        SimTrajectory {
            v,
            w,
            end_x: x,
            end_y: y,
            end_theta: theta,
            min_obstacle_dist: min_dist,
            score: 0.0,
        }
    }
}

/// Find the nearest path waypoint ahead as the local goal.
fn find_local_goal(state: &RobotState, path: &[PathWaypoint]) -> PathWaypoint {
    let lookahead = 1.0; // meters

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
    fn test_dwa_straight_no_obstacles() {
        let planner = DwaPlanner::new(DwaConfig::default());
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
            },
            PathWaypoint {
                x: 2.0,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
            },
        ];

        let cmd = planner.compute(&state, &path, &[], 0.5);
        assert!(cmd.linear_x > 0.0, "Should move forward");
        assert!(cmd.angular_z.abs() < 0.5, "Should be roughly straight");
    }

    #[test]
    fn test_dwa_output_stays_inside_curvature_envelope() {
        // Goal 90° to the side tempts a sharp turn; the command must still be
        // executable: |w| <= v * max_curvature, so nothing downstream clamps
        // it into a wider-than-verified arc.
        let config = DwaConfig::default();
        let max_curvature = config.max_curvature;
        let planner = DwaPlanner::new(config);
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
        let planner = DwaPlanner::new(DwaConfig::default());
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
        }];

        let cmd = planner.compute(&state, &path, &[], 0.5);
        assert!(
            cmd.angular_z.abs() <= cmd.linear_x * DwaConfig::default().max_curvature + 1e-6,
            "spin-in-place command is not executable by Ackermann steering"
        );
    }

    #[test]
    fn test_dwa_avoids_obstacle() {
        let planner = DwaPlanner::new(DwaConfig::default());
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
        }];
        let obstacles = vec![Obstacle { x: 0.5, y: 0.0 }];

        let cmd = planner.compute(&state, &path, &obstacles, 0.5);
        // Should steer away from obstacle
        assert!(
            cmd.angular_z.abs() > 0.01 || cmd.linear_x < 0.1,
            "Should avoid obstacle by turning or slowing"
        );
    }

    #[test]
    fn test_dwa_empty_path() {
        let planner = DwaPlanner::new(DwaConfig::default());
        let state = RobotState::default();

        let cmd = planner.compute(&state, &[], &[], 0.5);
        assert_eq!(cmd.linear_x, 0.0);
    }
}
