/// Sensor drivers for the SensPerc process.
///
/// Each driver runs as a dedicated thread, reading from hardware
/// and pushing data into the SensorStore ring buffers.
pub mod camera;
pub mod imu;
pub mod lidar;
