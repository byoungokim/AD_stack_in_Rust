/// Simplified MPC (Model Predictive Control) for tight maneuvers.
///
/// Uses a short-horizon iterative optimizer with bicycle model dynamics.
/// Activates when the DWA's short horizon is insufficient (parking, U-turns,
/// sharp curves). Runs at 10Hz with a 5-step horizon.
use serde::Deserialize;

use super::{Obstacle, RobotState, VelocityCommand};
use crate::global_planner::PathWaypoint;

#[derive(Debug, Clone, Deserialize)]
pub struct MpcConfig {
    #[serde(default = "default_horizon")]
    pub horizon: usize, // number of prediction steps
    #[serde(default = "default_dt")]
    pub dt: f64, // seconds per step
    #[serde(default = "default_max_speed")]
    pub max_speed: f64,
    #[serde(default = "default_max_angular")]
    pub max_angular_speed: f64,
    #[serde(default = "default_wheelbase")]
    pub wheelbase: f64,
    #[serde(default = "default_max_steering")]
    pub max_steering_angle: f64,
    #[serde(default = "default_iterations")]
    pub iterations: usize, // optimization iterations
    #[serde(default = "default_robot_radius")]
    pub robot_radius: f64,
    /// Executable-curvature envelope (1/m): the output command is clamped to
    /// |w| <= v * max_curvature — the same envelope DWA enforces — so MPC can
    /// never command an arc the Ackermann steering cannot execute. Keep in
    /// sync with `dwa.max_curvature` and the arbitrator safety envelope.
    #[serde(default = "default_max_curvature")]
    pub max_curvature: f64,

    // Cost weights
    #[serde(default = "default_w_pos")]
    pub weight_position: f64,
    #[serde(default = "default_w_heading")]
    pub weight_heading: f64,
    #[serde(default = "default_w_velocity")]
    pub weight_velocity: f64,
    #[serde(default = "default_w_steer_rate")]
    pub weight_steer_rate: f64,
    #[serde(default = "default_w_obstacle")]
    pub weight_obstacle: f64,
}

fn default_horizon() -> usize {
    5
}
fn default_dt() -> f64 {
    0.2
}
fn default_max_speed() -> f64 {
    1.0
}
fn default_max_angular() -> f64 {
    1.5
}
fn default_wheelbase() -> f64 {
    0.2
}
fn default_max_steering() -> f64 {
    0.48
}
fn default_iterations() -> usize {
    10
}
fn default_robot_radius() -> f64 {
    0.2
}
fn default_max_curvature() -> f64 {
    // Matches DWA / arbitrator safety envelope; inside the Ackermann limit
    // tan(0.48)/0.2 ≈ 2.6.
    2.0
}
fn default_w_pos() -> f64 {
    5.0
}
fn default_w_heading() -> f64 {
    2.0
}
fn default_w_velocity() -> f64 {
    1.0
}
fn default_w_steer_rate() -> f64 {
    3.0
}
fn default_w_obstacle() -> f64 {
    10.0
}

impl Default for MpcConfig {
    fn default() -> Self {
        Self {
            horizon: default_horizon(),
            dt: default_dt(),
            max_speed: default_max_speed(),
            max_angular_speed: default_max_angular(),
            wheelbase: default_wheelbase(),
            max_steering_angle: default_max_steering(),
            iterations: default_iterations(),
            robot_radius: default_robot_radius(),
            max_curvature: default_max_curvature(),
            weight_position: default_w_pos(),
            weight_heading: default_w_heading(),
            weight_velocity: default_w_velocity(),
            weight_steer_rate: default_w_steer_rate(),
            weight_obstacle: default_w_obstacle(),
        }
    }
}

/// Control input for one MPC step.
#[derive(Debug, Clone)]
struct ControlInput {
    velocity: f64, // m/s
    steering: f64, // radians
}

pub struct SimpleMpc {
    config: MpcConfig,
    prev_controls: Vec<ControlInput>,
}

impl SimpleMpc {
    pub fn new(config: MpcConfig) -> Self {
        let horizon = config.horizon;
        Self {
            config,
            prev_controls: vec![
                ControlInput {
                    velocity: 0.0,
                    steering: 0.0
                };
                horizon
            ],
        }
    }

    pub fn compute(
        &mut self,
        state: &RobotState,
        path: &[PathWaypoint],
        obstacles: &[Obstacle],
        desired_speed: f64,
    ) -> VelocityCommand {
        let target_speed = desired_speed.min(self.config.max_speed);

        // Find reference points on path for each horizon step
        let refs = self.find_reference_points(state, path);

        // Warm-start: shift previous solution
        let mut controls = self.prev_controls.clone();
        controls.rotate_left(1);
        let fill = controls
            .get(controls.len().saturating_sub(2))
            .cloned()
            .unwrap_or(ControlInput {
                velocity: target_speed,
                steering: 0.0,
            });
        if let Some(last) = controls.last_mut() {
            *last = fill;
        }

        // Iterative optimization (simplified gradient-free: random perturbation)
        let mut best_cost = self.evaluate_cost(state, &controls, &refs, obstacles);

        for _iter in 0..self.config.iterations {
            let mut candidate = controls.clone();

            // Perturb each control input slightly
            for ctrl in candidate.iter_mut() {
                let v_perturb = (fastrand() - 0.5) * 0.1;
                let s_perturb = (fastrand() - 0.5) * 0.1;

                ctrl.velocity = (ctrl.velocity + v_perturb).clamp(0.0, target_speed);
                ctrl.steering = (ctrl.steering + s_perturb).clamp(
                    -self.config.max_steering_angle,
                    self.config.max_steering_angle,
                );
            }

            let cost = self.evaluate_cost(state, &candidate, &refs, obstacles);
            if cost < best_cost {
                best_cost = cost;
                controls = candidate;
            }
        }

        // Confidence is FEASIBILITY-based: how much clearance the chosen
        // trajectory keeps to the nearest obstacle. Deriving it from the cost
        // sum pinned confidence to ~0.1 near obstacles (the 1/(dist+0.01)
        // term is unbounded) and deadlocked the arbitrator gate mid-slalom
        // even though the trajectory was collision-free.
        let min_clearance = self.trajectory_clearance(state, &controls, obstacles);
        let confidence = feasibility_confidence(min_clearance, self.config.robot_radius);

        // Apply first control
        let first_vel = controls[0].velocity;
        let first_steer = controls[0].steering;
        let angular_z = first_vel * first_steer.tan() / self.config.wheelbase;

        self.prev_controls = controls;

        // Executable-curvature envelope, same as DWA: never command an arc
        // tighter than |w| = v * max_curvature — downstream would clamp it and
        // the real arc would swing wider than the optimized one.
        let w_curv_limit = first_vel.abs() * self.config.max_curvature;
        let angular_z = angular_z
            .clamp(
                -self.config.max_angular_speed,
                self.config.max_angular_speed,
            )
            .clamp(-w_curv_limit, w_curv_limit);

        VelocityCommand {
            linear_x: first_vel,
            angular_z,
            confidence,
        }
    }

    /// Minimum clearance (m) between the robot BODY and any obstacle surface
    /// along the trajectory produced by `controls`: distance to the obstacle
    /// surface minus the robot radius. Negative means the trajectory is in
    /// collision; +INFINITY when there are no obstacles.
    fn trajectory_clearance(
        &self,
        state: &RobotState,
        controls: &[ControlInput],
        obstacles: &[Obstacle],
    ) -> f64 {
        let mut x = state.x;
        let mut y = state.y;
        let mut theta = state.theta;
        let mut min_clearance = f64::INFINITY;

        for (i, ctrl) in controls.iter().enumerate() {
            x += ctrl.velocity * theta.cos() * self.config.dt;
            y += ctrl.velocity * theta.sin() * self.config.dt;
            theta += (ctrl.velocity / self.config.wheelbase) * ctrl.steering.tan() * self.config.dt;

            let t = (i + 1) as f64 * self.config.dt;
            for obs in obstacles {
                let (ox, oy) = obs.position_at(t);
                let clearance = ((x - ox).powi(2) + (y - oy).powi(2)).sqrt()
                    - obs.effective_radius_at(t)
                    - self.config.robot_radius;
                if clearance < min_clearance {
                    min_clearance = clearance;
                }
            }
        }

        min_clearance
    }

    /// Evaluate cost of a control sequence.
    fn evaluate_cost(
        &self,
        state: &RobotState,
        controls: &[ControlInput],
        refs: &[PathWaypoint],
        obstacles: &[Obstacle],
    ) -> f64 {
        let mut x = state.x;
        let mut y = state.y;
        let mut theta = state.theta;
        let mut prev_steer = 0.0;
        let mut total_cost = 0.0;

        for (i, ctrl) in controls.iter().enumerate() {
            // Bicycle model forward simulation
            x += ctrl.velocity * theta.cos() * self.config.dt;
            y += ctrl.velocity * theta.sin() * self.config.dt;
            theta += (ctrl.velocity / self.config.wheelbase) * ctrl.steering.tan() * self.config.dt;

            // Position cost (track reference)
            if let Some(ref_pt) = refs.get(i) {
                let pos_err = (x - ref_pt.x).powi(2) + (y - ref_pt.y).powi(2);
                total_cost += self.config.weight_position * pos_err;

                let heading_err = normalize_angle(theta - ref_pt.theta).powi(2);
                total_cost += self.config.weight_heading * heading_err;
            }

            // Velocity cost (penalize deviation from reference speed)
            let ref_speed = refs.get(i).map_or(0.0, |r| {
                // Estimate desired speed from spacing between reference points
                if i > 0 {
                    let dx = r.x - refs.get(i - 1).map_or(r.x, |prev| prev.x);
                    let dy = r.y - refs.get(i - 1).map_or(r.y, |prev| prev.y);
                    (dx * dx + dy * dy).sqrt() / self.config.dt
                } else {
                    self.config.max_speed * 0.5 // default reference speed
                }
            });
            let vel_err = (ctrl.velocity - ref_speed).powi(2);
            total_cost += self.config.weight_velocity * vel_err;

            // Steering rate cost (smooth steering)
            let steer_rate = (ctrl.steering - prev_steer).powi(2);
            total_cost += self.config.weight_steer_rate * steer_rate;
            prev_steer = ctrl.steering;

            // Obstacle cost (velocity-propagated surface distance)
            let t = (i + 1) as f64 * self.config.dt;
            for obs in obstacles {
                let (ox, oy) = obs.position_at(t);
                let dist =
                    ((x - ox).powi(2) + (y - oy).powi(2)).sqrt() - obs.effective_radius_at(t);
                if dist < self.config.robot_radius * 2.0 {
                    total_cost += self.config.weight_obstacle / (dist.max(0.0) + 0.01);
                }
            }
        }

        total_cost
    }

    /// Find reference points along the path for each horizon step.
    fn find_reference_points(
        &self,
        state: &RobotState,
        path: &[PathWaypoint],
    ) -> Vec<PathWaypoint> {
        let mut refs = Vec::with_capacity(self.config.horizon);

        // Estimate distance traveled per step
        let step_dist = 0.5 * self.config.max_speed * self.config.dt;
        let mut accumulated = 0.0;
        let mut path_idx = 0;

        // Find nearest path point
        let mut min_dist = f64::INFINITY;
        for (i, wp) in path.iter().enumerate() {
            let d = (wp.x - state.x).powi(2) + (wp.y - state.y).powi(2);
            if d < min_dist {
                min_dist = d;
                path_idx = i;
            }
        }

        for _ in 0..self.config.horizon {
            accumulated += step_dist;
            while path_idx + 1 < path.len() {
                let seg_len = ((path[path_idx + 1].x - path[path_idx].x).powi(2)
                    + (path[path_idx + 1].y - path[path_idx].y).powi(2))
                .sqrt();
                if accumulated > seg_len {
                    accumulated -= seg_len;
                    path_idx += 1;
                } else {
                    break;
                }
            }
            refs.push(path[path_idx.min(path.len() - 1)].clone());
        }

        refs
    }
}

/// Map trajectory clearance to confidence: a colliding trajectory (clearance
/// < 0) is 0.1 — the arbitrator's degraded response takes over. A collision-
/// free trajectory scales 0.6..0.9 with clearance, saturating at one robot
/// radius of margin: grazing past at zero margin is trustworthy-but-tight
/// (0.6), a full radius of spare room (or an empty field) is 0.9.
fn feasibility_confidence(min_clearance: f64, robot_radius: f64) -> f32 {
    if min_clearance < 0.0 {
        return 0.1;
    }
    let margin_scale = (min_clearance / robot_radius.max(1e-6)).min(1.0);
    (0.6 + 0.3 * margin_scale) as f32
}

/// Simple deterministic pseudo-random in [0, 1) for perturbation.
fn fastrand() -> f64 {
    use std::cell::Cell;
    thread_local! {
        static STATE: Cell<u32> = const { Cell::new(12345) };
    }
    STATE.with(|s| {
        let mut x = s.get();
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        s.set(x);
        (x as f64) / (u32::MAX as f64)
    })
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
    fn test_mpc_straight() {
        let mut mpc = SimpleMpc::new(MpcConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![
            PathWaypoint {
                x: 0.5,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            },
            PathWaypoint {
                x: 1.0,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            },
            PathWaypoint {
                x: 1.5,
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

        let cmd = mpc.compute(&state, &path, &[], 0.5);
        assert!(cmd.linear_x > 0.0, "MPC should produce forward velocity");
    }

    #[test]
    fn test_mpc_obstacle_penalty() {
        let mut mpc = SimpleMpc::new(MpcConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 1.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        let obstacles = vec![Obstacle::point(0.3, 0.0)];

        let cmd = mpc.compute(&state, &path, &obstacles, 0.5);
        // Should slow down or steer to avoid
        assert!(cmd.linear_x < 0.5 || cmd.angular_z.abs() > 0.01);
    }

    #[test]
    fn test_mpc_confidence_high_when_trajectory_clear() {
        // Open field: feasibility-based confidence must be HIGH even though
        // tracking/velocity cost terms are non-zero. The old cost-derived
        // formula (1 - cost/100) reported near-arbitrary values here.
        let mut mpc = SimpleMpc::new(MpcConfig::default());
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

        // No obstacles at all: full margin confidence.
        let cmd = mpc.compute(&state, &path, &[], 0.5);
        assert!(
            (cmd.confidence - 0.9).abs() < 1e-6,
            "clear trajectory must be high-confidence, got {}",
            cmd.confidence
        );

        // A far-away obstacle (2m off to the side) must not tank confidence.
        let far = Obstacle::point(1.0, 2.0);
        let cmd = mpc.compute(&state, &path, &[far], 0.5);
        assert!(
            cmd.confidence >= 0.6,
            "collision-free trajectory near an irrelevant obstacle got {}",
            cmd.confidence
        );
    }

    #[test]
    fn test_mpc_confidence_low_when_trajectory_collides() {
        // Robot boxed in: an obstacle dead ahead within the robot radius means
        // every forward trajectory collides -> confidence 0.1.
        let mut mpc = SimpleMpc::new(MpcConfig::default());
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![PathWaypoint {
            x: 1.0,
            y: 0.0,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }];
        // Wall of obstacles right in front — no clearance anywhere.
        let wall: Vec<Obstacle> = (-3..=3)
            .map(|i| Obstacle::point(0.15, i as f64 * 0.1))
            .collect();

        let cmd = mpc.compute(&state, &path, &wall, 0.5);
        assert!(
            (cmd.confidence - 0.1).abs() < 1e-6,
            "colliding trajectory must report 0.1, got {}",
            cmd.confidence
        );
    }

    #[test]
    fn test_feasibility_confidence_mapping() {
        // Collision -> 0.1.
        assert_eq!(feasibility_confidence(-0.05, 0.2), 0.1);
        // Zero clearance (grazing) -> 0.6.
        assert!((feasibility_confidence(0.0, 0.2) - 0.6).abs() < 1e-6);
        // One robot radius of margin (or more) saturates at 0.9.
        assert!((feasibility_confidence(0.2, 0.2) - 0.9).abs() < 1e-6);
        assert!((feasibility_confidence(10.0, 0.2) - 0.9).abs() < 1e-6);
        // Monotone in between.
        assert!(feasibility_confidence(0.05, 0.2) < feasibility_confidence(0.15, 0.2));
    }

    #[test]
    fn test_mpc_output_stays_inside_curvature_envelope() {
        // Path bending hard 90° to the side tempts maximum steering; the
        // emitted command must satisfy |w| <= v * max_curvature so downstream
        // clamps never widen the real arc past the optimized one.
        let config = MpcConfig::default();
        let max_curvature = config.max_curvature;
        let mut mpc = SimpleMpc::new(config);
        let state = RobotState {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
            linear_vel: 0.3,
            angular_vel: 0.0,
        };
        let path = vec![
            PathWaypoint {
                x: 0.1,
                y: 0.1,
                theta: 0.8,
                steering: 0.0,
                dir: Default::default(),
            },
            PathWaypoint {
                x: 0.1,
                y: 0.5,
                theta: std::f64::consts::FRAC_PI_2,
                steering: 0.0,
                dir: Default::default(),
            },
            PathWaypoint {
                x: 0.1,
                y: 1.0,
                theta: std::f64::consts::FRAC_PI_2,
                steering: 0.0,
                dir: Default::default(),
            },
        ];

        for _ in 0..5 {
            let cmd = mpc.compute(&state, &path, &[], 0.5);
            assert!(
                cmd.angular_z.abs() <= cmd.linear_x.abs() * max_curvature + 1e-9,
                "MPC command (v={}, w={}) exceeds curvature envelope {}",
                cmd.linear_x,
                cmd.angular_z,
                max_curvature
            );
        }
    }
}
