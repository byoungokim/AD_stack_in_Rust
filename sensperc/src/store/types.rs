/// Sensor data types used within the sensperc process.
use nalgebra::Vector3;

/// Raw camera frame.
#[derive(Clone)]
pub struct CameraFrame {
    pub timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub encoding: String, // "bgr8", "rgb8", "jpeg"
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
#[derive(Clone)]
pub struct ImuReading {
    pub timestamp_ns: u64,
    pub linear_acceleration: Vector3<f64>, // m/s^2
    pub angular_velocity: Vector3<f64>,    // rad/s
    pub orientation_euler: Vector3<f64>,   // roll, pitch, yaw in radians
    pub sequence: u32,
}

/// 2D robot pose.
#[derive(Clone, Debug, Default)]
pub struct Pose2D {
    pub x: f64,     // meters
    pub y: f64,     // meters
    pub theta: f64, // radians
}

/// 2D velocity.
#[derive(Clone, Debug, Default)]
pub struct Twist2D {
    pub linear_x: f64,  // m/s
    pub linear_y: f64,  // m/s
    pub angular_z: f64,  // rad/s
}

/// Fused sensor state from EKF.
#[derive(Clone, Debug)]
pub struct FusedState {
    pub timestamp_ns: u64,
    pub pose: Pose2D,
    pub velocity: Twist2D,
}
