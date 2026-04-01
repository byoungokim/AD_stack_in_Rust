/// Kinematics engine: converts velocity commands to wheel-level commands
/// and computes odometry from wheel feedback.
///
/// Supports Ackermann and differential drive modes for the Limo Pro.
use serde::Deserialize;

use crate::chassis::{ChassisFeedback, MotorCommand};

#[derive(Debug, Clone, Deserialize)]
pub struct KinematicsConfig {
    #[serde(default = "default_mode")]
    pub mode: String, // "ackermann" or "differential"
    #[serde(default = "default_wheelbase")]
    pub wheelbase: f64, // meters
    #[serde(default = "default_track_width")]
    pub track_width: f64, // meters
    #[serde(default = "default_wheel_radius")]
    pub wheel_radius: f64, // meters
    #[serde(default = "default_max_steering")]
    pub max_steering_angle: f64, // radians
}

fn default_mode() -> String { "ackermann".into() }
fn default_wheelbase() -> f64 { 0.2 }
fn default_track_width() -> f64 { 0.172 }
fn default_wheel_radius() -> f64 { 0.045 }
fn default_max_steering() -> f64 { 0.48 }

impl Default for KinematicsConfig {
    fn default() -> Self {
        Self {
            mode: default_mode(),
            wheelbase: default_wheelbase(),
            track_width: default_track_width(),
            wheel_radius: default_wheel_radius(),
            max_steering_angle: default_max_steering(),
        }
    }
}

/// 2D pose for odometry tracking.
#[derive(Clone, Debug, Default)]
pub struct OdomPose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// 2D velocity.
#[derive(Clone, Debug, Default)]
pub struct OdomVelocity {
    pub linear_x: f64,
    pub angular_z: f64,
}

pub struct KinematicsEngine {
    config: KinematicsConfig,
    pose: OdomPose,
}

impl KinematicsEngine {
    pub fn new(config: KinematicsConfig) -> Self {
        Self {
            config,
            pose: OdomPose::default(),
        }
    }

    /// Clamp a velocity command to hardware limits.
    pub fn clamp_command(&self, cmd: &MotorCommand) -> MotorCommand {
        let max_angular = if self.config.mode == "ackermann" {
            // Max angular vel from max steering angle: v * tan(delta) / L
            // Use a reasonable max linear vel for the limit
            1.0_f64 * self.config.max_steering_angle.tan() / self.config.wheelbase
        } else {
            // Differential: limited by track width and wheel speed
            3.0 // rad/s, generous limit
        };

        MotorCommand {
            linear_vel: cmd.linear_vel.clamp(-1.0, 1.0),
            angular_vel: cmd.angular_vel.clamp(-max_angular, max_angular),
        }
    }

    /// Convert a (linear, angular) velocity command to a steering angle (Ackermann).
    /// Returns the steering angle in radians.
    pub fn velocity_to_steering(&self, cmd: &MotorCommand) -> f64 {
        if cmd.linear_vel.abs() < 1e-6 {
            return 0.0;
        }
        let steering = (cmd.angular_vel * self.config.wheelbase / cmd.linear_vel).atan();
        steering.clamp(-self.config.max_steering_angle, self.config.max_steering_angle)
    }

    /// Update odometry from wheel feedback.
    /// Returns the updated pose and velocity.
    pub fn update_odometry(
        &mut self,
        feedback: &ChassisFeedback,
        dt: f64,
    ) -> (OdomPose, OdomVelocity) {
        let (linear_vel, angular_vel) = match self.config.mode.as_str() {
            "ackermann" => self.ackermann_odom(feedback),
            _ => self.differential_odom(feedback),
        };

        // Integrate pose
        self.pose.x += linear_vel * self.pose.theta.cos() * dt;
        self.pose.y += linear_vel * self.pose.theta.sin() * dt;
        self.pose.theta += angular_vel * dt;

        // Normalize theta to [-pi, pi]
        self.pose.theta = normalize_angle(self.pose.theta);

        let velocity = OdomVelocity {
            linear_x: linear_vel,
            angular_z: angular_vel,
        };

        (self.pose.clone(), velocity)
    }

    /// Compute velocity from wheel RPMs using differential drive model.
    fn differential_odom(&self, feedback: &ChassisFeedback) -> (f64, f64) {
        let left_vel = rpm_to_vel(feedback.left_wheel_rpm, self.config.wheel_radius);
        let right_vel = rpm_to_vel(feedback.right_wheel_rpm, self.config.wheel_radius);

        let linear = (left_vel + right_vel) / 2.0;
        let angular = (right_vel - left_vel) / self.config.track_width;

        (linear, angular)
    }

    /// Compute velocity from wheel RPMs + steering angle using Ackermann model.
    fn ackermann_odom(&self, feedback: &ChassisFeedback) -> (f64, f64) {
        let left_vel = rpm_to_vel(feedback.left_wheel_rpm, self.config.wheel_radius);
        let right_vel = rpm_to_vel(feedback.right_wheel_rpm, self.config.wheel_radius);

        let linear = (left_vel + right_vel) / 2.0;
        let angular = linear * (feedback.steering_angle as f64).tan() / self.config.wheelbase;

        (linear, angular)
    }

    pub fn pose(&self) -> &OdomPose {
        &self.pose
    }

    pub fn reset_pose(&mut self) {
        self.pose = OdomPose::default();
    }
}

fn rpm_to_vel(rpm: f32, wheel_radius: f64) -> f64 {
    (rpm as f64) * 2.0 * std::f64::consts::PI * wheel_radius / 60.0
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
    fn test_differential_odom_straight() {
        let config = KinematicsConfig {
            mode: "differential".into(),
            ..Default::default()
        };
        let mut engine = KinematicsEngine::new(config);

        // Both wheels at same RPM → straight line
        let fb = ChassisFeedback {
            left_wheel_rpm: 100.0,
            right_wheel_rpm: 100.0,
            ..Default::default()
        };

        let (pose, vel) = engine.update_odometry(&fb, 0.1);
        assert!(vel.linear_x > 0.0);
        assert!(vel.angular_z.abs() < 1e-10);
        assert!(pose.x > 0.0);
        assert!(pose.y.abs() < 1e-10);
    }

    #[test]
    fn test_differential_odom_turn() {
        let config = KinematicsConfig {
            mode: "differential".into(),
            ..Default::default()
        };
        let mut engine = KinematicsEngine::new(config);

        // Right wheel faster → turn left
        let fb = ChassisFeedback {
            left_wheel_rpm: 50.0,
            right_wheel_rpm: 100.0,
            ..Default::default()
        };

        let (_pose, vel) = engine.update_odometry(&fb, 0.1);
        assert!(vel.angular_z > 0.0); // positive = CCW = left turn
    }

    #[test]
    fn test_clamp_command() {
        let engine = KinematicsEngine::new(KinematicsConfig::default());
        let cmd = MotorCommand {
            linear_vel: 5.0,
            angular_vel: 10.0,
        };
        let clamped = engine.clamp_command(&cmd);
        assert!(clamped.linear_vel <= 1.0);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(4.0) - (4.0 - 2.0 * std::f64::consts::PI)).abs() < 1e-10);
        assert!((normalize_angle(-4.0) - (-4.0 + 2.0 * std::f64::consts::PI)).abs() < 1e-10);
    }
}
