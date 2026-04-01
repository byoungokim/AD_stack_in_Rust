/// IMU driver: reads orientation/acceleration data via serial and pushes to SensorStore.
///
/// Supports a generic binary IMU protocol (common on low-cost 6/9-DOF IMUs).
/// Falls back to dummy data generation when serial is unavailable.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use nalgebra::Vector3;
use serde::Deserialize;
use tracing::{debug, error, info, warn};

use crate::store::types::ImuReading;
use crate::store::SensorStore;

#[derive(Debug, Clone, Deserialize)]
pub struct ImuConfig {
    #[serde(default = "default_device")]
    pub device: String,
    #[serde(default = "default_baudrate")]
    pub baudrate: u32,
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
}

fn default_device() -> String { "/dev/ttyUSB1".into() }
fn default_baudrate() -> u32 { 115200 }
fn default_rate_hz() -> u32 { 100 }

impl Default for ImuConfig {
    fn default() -> Self {
        Self {
            device: default_device(),
            baudrate: default_baudrate(),
            rate_hz: default_rate_hz(),
        }
    }
}

pub struct ImuDriver {
    config: ImuConfig,
    store: Arc<SensorStore>,
    running: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl ImuDriver {
    pub fn new(store: Arc<SensorStore>, config: ImuConfig) -> Self {
        Self {
            config,
            store,
            running: Arc::new(AtomicBool::new(false)),
            thread: None,
        }
    }

    pub fn start(&mut self) -> Result<()> {
        if self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        let config = self.config.clone();
        let store = Arc::clone(&self.store);
        let running = Arc::clone(&self.running);

        running.store(true, Ordering::Release);

        let handle = thread::Builder::new()
            .name("ImuDriver".into())
            .spawn(move || {
                if let Err(e) = imu_loop(&config, &store, &running) {
                    error!("ImuDriver error: {:#}", e);
                }
                running.store(false, Ordering::Release);
            })
            .context("Failed to spawn ImuDriver thread")?;

        self.thread = Some(handle);
        info!(
            "ImuDriver started: {} @ {} baud, {}Hz",
            self.config.device, self.config.baudrate, self.config.rate_hz
        );
        Ok(())
    }

    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
        info!("ImuDriver stopped");
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }
}

impl Drop for ImuDriver {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Try serial, fall back to dummy.
fn imu_loop(
    config: &ImuConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    match try_serial_imu_loop(config, store, running) {
        Ok(()) => Ok(()),
        Err(e) => {
            warn!("Serial IMU unavailable ({}), falling back to dummy", e);
            dummy_imu_loop(config, store, running)
        }
    }
}

/// Read IMU data from serial port.
fn try_serial_imu_loop(
    config: &ImuConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    let port = serialport::new(&config.device, config.baudrate)
        .timeout(Duration::from_millis(100))
        .open()
        .context(format!(
            "Failed to open IMU serial: {} @ {}",
            config.device, config.baudrate
        ))?;

    info!("IMU serial port opened: {}", config.device);

    let mut reader = std::io::BufReader::new(port);
    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut sequence: u32 = 0;

    while running.load(Ordering::Acquire) {
        let read_start = Instant::now();

        match read_imu_frame(&mut reader) {
            Ok(reading) => {
                let imu = ImuReading {
                    timestamp_ns: now_ns(),
                    linear_acceleration: reading.0,
                    angular_velocity: reading.1,
                    orientation_euler: reading.2,
                    sequence,
                };

                store.push_imu_reading(imu);
                sequence += 1;

                if sequence % (config.rate_hz * 10) == 0 {
                    debug!("ImuDriver: {} readings captured", sequence);
                }
            }
            Err(e) => {
                warn!("IMU read failed: {}", e);
            }
        }

        let elapsed = read_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    Ok(())
}

/// Read a single IMU data frame from serial.
///
/// Generic binary protocol: header (0x55, 0x51-0x53) + 8 bytes payload.
/// Packet types: 0x51 = acceleration, 0x52 = angular velocity, 0x53 = orientation.
/// Real implementations should match the specific IMU's protocol (WT901, BNO055, etc.).
fn read_imu_frame(
    reader: &mut std::io::BufReader<Box<dyn serialport::SerialPort>>,
) -> Result<(Vector3<f64>, Vector3<f64>, Vector3<f64>)> {
    use std::io::Read;

    // Read 3 packets (accel + gyro + orientation)
    let mut accel = Vector3::zeros();
    let mut gyro = Vector3::zeros();
    let mut euler = Vector3::zeros();

    for _ in 0..3 {
        // Find header byte 0x55
        let mut header = [0u8; 2];
        reader.read_exact(&mut header)?;

        let mut payload = [0u8; 8];
        reader.read_exact(&mut payload)?;

        match header[1] {
            0x51 => {
                // Acceleration: 3x int16 in units of g/32768
                let ax = i16::from_le_bytes([payload[0], payload[1]]) as f64 / 32768.0 * 16.0 * 9.81;
                let ay = i16::from_le_bytes([payload[2], payload[3]]) as f64 / 32768.0 * 16.0 * 9.81;
                let az = i16::from_le_bytes([payload[4], payload[5]]) as f64 / 32768.0 * 16.0 * 9.81;
                accel = Vector3::new(ax, ay, az);
            }
            0x52 => {
                // Angular velocity: 3x int16 in units of deg/s / 32768 * 2000
                let gx = i16::from_le_bytes([payload[0], payload[1]]) as f64 / 32768.0 * 2000.0_f64.to_radians();
                let gy = i16::from_le_bytes([payload[2], payload[3]]) as f64 / 32768.0 * 2000.0_f64.to_radians();
                let gz = i16::from_le_bytes([payload[4], payload[5]]) as f64 / 32768.0 * 2000.0_f64.to_radians();
                gyro = Vector3::new(gx, gy, gz);
            }
            0x53 => {
                // Euler angles: 3x int16 in units of deg/32768 * 180
                let roll  = i16::from_le_bytes([payload[0], payload[1]]) as f64 / 32768.0 * std::f64::consts::PI;
                let pitch = i16::from_le_bytes([payload[2], payload[3]]) as f64 / 32768.0 * std::f64::consts::PI;
                let yaw   = i16::from_le_bytes([payload[4], payload[5]]) as f64 / 32768.0 * std::f64::consts::PI;
                euler = Vector3::new(roll, pitch, yaw);
            }
            _ => {} // skip unknown packet types
        }
    }

    Ok((accel, gyro, euler))
}

/// Generate dummy IMU data for development.
fn dummy_imu_loop(
    config: &ImuConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    info!("ImuDriver using dummy data generator");

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut sequence: u32 = 0;
    let start_time = Instant::now();

    while running.load(Ordering::Acquire) {
        let read_start = Instant::now();
        let t = start_time.elapsed().as_secs_f64();

        // Simulate a stationary robot with slight vibrations
        let accel = Vector3::new(
            0.02 * (t * 5.0).sin(),  // slight vibration
            0.01 * (t * 7.0).cos(),
            9.81 + 0.01 * (t * 3.0).sin(), // gravity + vibration
        );

        let gyro = Vector3::new(
            0.001 * (t * 2.0).sin(),  // very small rotational noise
            0.001 * (t * 3.0).cos(),
            0.0,
        );

        let euler = Vector3::new(
            0.01 * (t * 0.5).sin(),  // slight roll
            0.005 * (t * 0.3).cos(), // slight pitch
            0.0,                      // no yaw drift
        );

        let reading = ImuReading {
            timestamp_ns: now_ns(),
            linear_acceleration: accel,
            angular_velocity: gyro,
            orientation_euler: euler,
            sequence,
        };

        store.push_imu_reading(reading);
        sequence += 1;

        if sequence % (config.rate_hz * 10) == 0 {
            debug!("ImuDriver (dummy): {} readings generated", sequence);
        }

        let elapsed = read_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    Ok(())
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
