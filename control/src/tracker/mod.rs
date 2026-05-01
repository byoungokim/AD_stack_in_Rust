//! Trajectory tracker: converts a planned trajectory or velocity command
//! into a motor command for the chassis.
//!
//! Supports two algorithms:
//! - Pure Pursuit: geometric path tracking with lookahead distance
//! - Stanley: heading + cross-track error controller
//!
//! Stub awaiting wiring from `main.rs`; kept dead-code-allowed.
#![allow(dead_code)]

use serde::Deserialize;

use crate::kinematics::OdomPose;
use limo_hal::MotorCommand;

#[derive(Debug, Clone, Deserialize)]
pub struct TrackerConfig {
    #[serde(default = "default_algorithm")]
    pub algorithm: String, // "pure_pursuit" or "stanley"
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default)]
    pub pure_pursuit: PurePursuitConfig,
    #[serde(default)]
    pub stanley: StanleyConfig,
}

fn default_algorithm() -> String {
    "pure_pursuit".into()
}
fn default_rate_hz() -> u32 {
    10
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            algorithm: default_algorithm(),
            rate_hz: default_rate_hz(),
            pure_pursuit: PurePursuitConfig::default(),
            stanley: StanleyConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct PurePursuitConfig {
    #[serde(default = "default_lookahead")]
    pub lookahead_distance: f64,
    #[serde(default = "default_min_lookahead")]
    pub min_lookahead: f64,
    #[serde(default = "default_max_lookahead")]
    pub max_lookahead: f64,
}

fn default_lookahead() -> f64 {
    0.5
}
fn default_min_lookahead() -> f64 {
    0.2
}
fn default_max_lookahead() -> f64 {
    1.5
}

impl Default for PurePursuitConfig {
    fn default() -> Self {
        Self {
            lookahead_distance: default_lookahead(),
            min_lookahead: default_min_lookahead(),
            max_lookahead: default_max_lookahead(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct StanleyConfig {
    #[serde(default = "default_k_gain")]
    pub k_gain: f64,
    #[serde(default = "default_k_soft")]
    pub k_soft: f64,
}

fn default_k_gain() -> f64 {
    0.5
}
fn default_k_soft() -> f64 {
    1.0
}

impl Default for StanleyConfig {
    fn default() -> Self {
        Self {
            k_gain: default_k_gain(),
            k_soft: default_k_soft(),
        }
    }
}

/// A waypoint in the trajectory to follow.
#[derive(Clone, Debug)]
pub struct TrajectoryPoint {
    pub x: f64,
    pub y: f64,
    pub heading: f64, // radians
    pub speed: f64,   // m/s
}

pub struct TrajectoryTracker {
    config: TrackerConfig,
    wheelbase: f64,
    trajectory: Vec<TrajectoryPoint>,
    nearest_idx: usize,
}

impl TrajectoryTracker {
    pub fn new(config: TrackerConfig, wheelbase: f64) -> Self {
        Self {
            config,
            wheelbase,
            trajectory: Vec::new(),
            nearest_idx: 0,
        }
    }

    /// Update the trajectory to follow.
    pub fn set_trajectory(&mut self, trajectory: Vec<TrajectoryPoint>) {
        self.trajectory = trajectory;
        self.nearest_idx = 0;
    }

    /// Check if the tracker has a trajectory to follow.
    // Public API used by the control loop to gate compute() once CH2 trajectories arrive.
    #[allow(dead_code)]
    pub fn has_trajectory(&self) -> bool {
        !self.trajectory.is_empty()
    }

    /// Compute a motor command given the current pose.
    /// Returns None if no trajectory or trajectory is complete.
    pub fn compute(&mut self, pose: &OdomPose) -> Option<MotorCommand> {
        if self.trajectory.is_empty() {
            return None;
        }

        match self.config.algorithm.as_str() {
            "stanley" => self.stanley_control(pose),
            _ => self.pure_pursuit_control(pose),
        }
    }

    /// Pure pursuit controller.
    fn pure_pursuit_control(&mut self, pose: &OdomPose) -> Option<MotorCommand> {
        // Find nearest point
        self.update_nearest(pose);

        // Compute adaptive lookahead based on speed
        let target_speed = self.trajectory[self.nearest_idx].speed;
        let lookahead = (self.config.pure_pursuit.lookahead_distance * target_speed.abs().max(0.1))
            .clamp(
                self.config.pure_pursuit.min_lookahead,
                self.config.pure_pursuit.max_lookahead,
            );

        // Find lookahead point on trajectory
        let lookahead_pt = self.find_lookahead_point(pose, lookahead)?;

        // Compute curvature to lookahead point
        let dx = lookahead_pt.x - pose.x;
        let dy = lookahead_pt.y - pose.y;

        // Transform to robot frame
        let local_x = dx * pose.theta.cos() + dy * pose.theta.sin();
        let local_y = -dx * pose.theta.sin() + dy * pose.theta.cos();

        let l_sq = local_x * local_x + local_y * local_y;
        if l_sq < 1e-6 {
            return Some(MotorCommand {
                linear_vel: target_speed,
                angular_vel: 0.0,
            });
        }

        // Curvature = 2 * y / L^2
        let curvature = 2.0 * local_y / l_sq;

        Some(MotorCommand {
            linear_vel: target_speed,
            angular_vel: target_speed * curvature,
        })
    }

    /// Stanley controller.
    fn stanley_control(&mut self, pose: &OdomPose) -> Option<MotorCommand> {
        self.update_nearest(pose);

        if self.nearest_idx >= self.trajectory.len() {
            return None;
        }
        let nearest = &self.trajectory[self.nearest_idx];
        let target_speed = nearest.speed;

        // Heading error
        let heading_err = normalize_angle(nearest.heading - pose.theta);

        // Cross-track error
        let dx = pose.x - nearest.x;
        let dy = pose.y - nearest.y;
        let cross_track = -dx * nearest.heading.sin() + dy * nearest.heading.cos();

        // Stanley control law: delta = heading_err + atan2(k * cte, v + k_soft)
        let stanley_term = (self.config.stanley.k_gain * cross_track)
            .atan2(target_speed.abs() + self.config.stanley.k_soft);

        let angular_vel = (heading_err + stanley_term) * target_speed / self.wheelbase;

        Some(MotorCommand {
            linear_vel: target_speed,
            angular_vel,
        })
    }

    /// Update nearest trajectory index.
    fn update_nearest(&mut self, pose: &OdomPose) {
        let mut min_dist = f64::INFINITY;
        let search_start = self.nearest_idx.saturating_sub(2);
        let search_end = (self.nearest_idx + 20).min(self.trajectory.len());

        for i in search_start..search_end {
            let pt = &self.trajectory[i];
            let dist = (pt.x - pose.x).powi(2) + (pt.y - pose.y).powi(2);
            if dist < min_dist {
                min_dist = dist;
                self.nearest_idx = i;
            }
        }
    }

    /// Find the first trajectory point beyond the lookahead distance.
    fn find_lookahead_point(&self, pose: &OdomPose, lookahead: f64) -> Option<TrajectoryPoint> {
        let la_sq = lookahead * lookahead;

        for i in self.nearest_idx..self.trajectory.len() {
            let pt = &self.trajectory[i];
            let dist_sq = (pt.x - pose.x).powi(2) + (pt.y - pose.y).powi(2);
            if dist_sq >= la_sq {
                return Some(pt.clone());
            }
        }

        // If no point is far enough, use the last point
        self.trajectory.last().cloned()
    }
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

    fn make_straight_trajectory() -> Vec<TrajectoryPoint> {
        (0..50)
            .map(|i| TrajectoryPoint {
                x: i as f64 * 0.1,
                y: 0.0,
                heading: 0.0,
                speed: 0.5,
            })
            .collect()
    }

    #[test]
    fn test_pure_pursuit_straight() {
        let mut tracker = TrajectoryTracker::new(TrackerConfig::default(), 0.2);
        tracker.set_trajectory(make_straight_trajectory());

        let pose = OdomPose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let cmd = tracker.compute(&pose).unwrap();

        assert!(cmd.linear_vel > 0.0);
        assert!(cmd.angular_vel.abs() < 0.1); // nearly straight
    }

    #[test]
    fn test_pure_pursuit_offset() {
        let mut tracker = TrajectoryTracker::new(TrackerConfig::default(), 0.2);
        tracker.set_trajectory(make_straight_trajectory());

        // Robot is offset to the left
        let pose = OdomPose {
            x: 0.0,
            y: 0.3,
            theta: 0.0,
        };
        let cmd = tracker.compute(&pose).unwrap();

        // Should steer right (negative angular for positive y offset)
        assert!(cmd.angular_vel < 0.0);
    }

    #[test]
    fn test_no_trajectory() {
        let mut tracker = TrajectoryTracker::new(TrackerConfig::default(), 0.2);
        let pose = OdomPose::default();
        assert!(tracker.compute(&pose).is_none());
    }
}
