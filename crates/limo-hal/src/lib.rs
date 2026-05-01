pub mod dummy;
pub mod limo_hw;
pub mod protocols;
pub mod sim_zmq;
/// Limo Drive Hardware Abstraction Layer.
///
/// Defines traits that abstract sensor input and vehicle control,
/// enabling the same AD algorithms to run on any platform:
/// - Real Limo Pro hardware (V4L2, serial)
/// - Gazebo / Isaac Sim via ZMQ bridge
/// - Dummy synthetic data for testing
///
/// # Usage
/// ```ignore
/// let source: Box<dyn SensorSource> = match mode {
///     Mode::Real => Box::new(LimoHwSensorSource::new(config)),
///     Mode::Sim  => Box::new(SimZmqSensorSource::new(config)),
///     Mode::Test => Box::new(DummySensorSource::new()),
/// };
/// ```
pub mod types;

pub use types::*;

use anyhow::Result;

/// Sensor data source trait.
///
/// Abstracts where sensor data comes from. The SensPerc process
/// calls these methods without knowing if data comes from real
/// hardware, a simulator, or a test generator.
pub trait SensorSource: Send {
    /// Initialize and start the sensor source.
    fn start(&mut self) -> Result<()>;

    /// Stop the sensor source and release resources.
    fn stop(&mut self);

    /// Try to receive a camera frame (non-blocking).
    fn recv_camera(&mut self) -> Option<CameraFrame>;

    /// Try to receive a LiDAR scan (non-blocking).
    fn recv_lidar(&mut self) -> Option<LidarScan>;

    /// Try to receive an IMU reading (non-blocking).
    fn recv_imu(&mut self) -> Option<ImuReading>;

    /// Try to receive a pose estimate with confidence (non-blocking).
    /// Returns (pose, confidence) where confidence is [0.0, 1.0].
    /// Sources: sim ground truth (1.0), SLAM (0.8), odometry (0.6).
    fn recv_pose(&mut self) -> Option<(Pose2D, f32)>;

    /// Try to receive a velocity estimate (non-blocking).
    fn recv_velocity(&mut self) -> Option<Twist2D>;

    /// Name of this source for logging.
    fn name(&self) -> &str;

    /// Number of errors encountered since start. Consumers can monitor
    /// this to detect hardware faults (increasing count = degraded).
    fn error_count(&self) -> u64 {
        0
    }

    /// Whether the source is healthy (receiving data without errors).
    fn is_healthy(&self) -> bool {
        true
    }
}

/// Vehicle controller trait.
///
/// Abstracts how motor commands are sent to the vehicle and how
/// chassis feedback is received. The Control process calls these
/// methods without knowing if it's driving real motors or a sim.
pub trait VehicleController: Send {
    /// Initialize and start the vehicle controller.
    fn start(&mut self) -> Result<()>;

    /// Stop the controller and release resources.
    fn stop(&mut self);

    /// Send a motor command to the vehicle.
    fn send_command(&mut self, cmd: &MotorCommand) -> Result<()>;

    /// Try to receive chassis feedback (non-blocking).
    fn recv_feedback(&mut self) -> Option<ChassisFeedback>;

    /// Name of this controller for logging.
    fn name(&self) -> &str;

    /// Number of send/recv errors since start.
    fn error_count(&self) -> u64 {
        0
    }

    /// Whether the controller is healthy (hardware responsive).
    fn is_healthy(&self) -> bool {
        true
    }
}
