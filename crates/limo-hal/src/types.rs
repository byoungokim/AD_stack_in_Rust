/// Shared sensor and control data types.
///
/// These types define the boundary between platform-specific hardware
/// and platform-agnostic AD algorithms. All HAL implementations produce
/// and consume these types.
use nalgebra::Vector3;

// ======================== Sensor Types ========================

/// Raw camera frame.
#[derive(Clone)]
pub struct CameraFrame {
    pub timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub data: Vec<u8>,
    pub sequence: u32,
}

/// Single LiDAR scan.
#[derive(Clone)]
pub struct LidarScan {
    pub timestamp_ns: u64,
    pub angle_min: f32,
    pub angle_max: f32,
    pub angle_increment: f32,
    pub range_min: f32,
    pub range_max: f32,
    pub ranges: Vec<f32>,
    pub intensities: Vec<f32>,
    pub sequence: u32,
}

/// Single IMU measurement.
#[derive(Clone, Debug)]
pub struct ImuReading {
    pub timestamp_ns: u64,
    pub linear_acceleration: Vector3<f64>,
    pub angular_velocity: Vector3<f64>,
    pub orientation_euler: Vector3<f64>,
    pub sequence: u32,
}

// ======================== Localization Types ========================

/// 2D robot pose.
#[derive(Clone, Debug, Default)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// 2D velocity.
#[derive(Clone, Debug, Default)]
pub struct Twist2D {
    pub linear_x: f64,
    pub linear_y: f64,
    pub angular_z: f64,
}

/// Fused sensor state from EKF.
#[derive(Clone, Debug)]
pub struct FusedState {
    pub timestamp_ns: u64,
    pub pose: Pose2D,
    pub velocity: Twist2D,
}

// ======================== Control Types ========================

/// Motor command to send to the vehicle.
#[derive(Clone, Debug, Default)]
pub struct MotorCommand {
    pub linear_vel: f64,
    pub angular_vel: f64,
}

/// Feedback from the vehicle chassis.
#[derive(Clone, Debug, Default)]
pub struct ChassisFeedback {
    pub left_wheel_rpm: f32,
    pub right_wheel_rpm: f32,
    pub steering_angle: f32,
    pub battery_voltage: f32,
    pub error_code: u32,
    pub timestamp_ns: u64,
}
