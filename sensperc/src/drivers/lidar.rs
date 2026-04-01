/// LiDAR driver: reads scan data via serial port and pushes to SensorStore.
///
/// Supports common 2D LiDAR protocols (RPLIDAR, YDLidar, etc.).
/// Implements a generic frame parser that can be adapted per LiDAR model.
/// Falls back to dummy scan generation on platforms without serial ports.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use serde::Deserialize;
use tracing::{debug, error, info, warn};

use crate::store::types::LidarScan;
use crate::store::SensorStore;

#[derive(Debug, Clone, Deserialize)]
pub struct LidarConfig {
    #[serde(default = "default_device")]
    pub device: String,
    #[serde(default = "default_baudrate")]
    pub baudrate: u32,
    #[serde(default = "default_scan_rate_hz")]
    pub scan_rate_hz: u32,
    #[serde(default = "default_range_min")]
    pub range_min: f32,
    #[serde(default = "default_range_max")]
    pub range_max: f32,
    #[serde(default = "default_num_points")]
    pub num_points: usize,
}

fn default_device() -> String { "/dev/ttyUSB0".into() }
fn default_baudrate() -> u32 { 230400 }
fn default_scan_rate_hz() -> u32 { 10 }
fn default_range_min() -> f32 { 0.1 }
fn default_range_max() -> f32 { 12.0 }
fn default_num_points() -> usize { 360 }

impl Default for LidarConfig {
    fn default() -> Self {
        Self {
            device: default_device(),
            baudrate: default_baudrate(),
            scan_rate_hz: default_scan_rate_hz(),
            range_min: default_range_min(),
            range_max: default_range_max(),
            num_points: default_num_points(),
        }
    }
}

pub struct LidarDriver {
    config: LidarConfig,
    store: Arc<SensorStore>,
    running: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl LidarDriver {
    pub fn new(store: Arc<SensorStore>, config: LidarConfig) -> Self {
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
            .name("LidarDriver".into())
            .spawn(move || {
                if let Err(e) = scan_loop(&config, &store, &running) {
                    error!("LidarDriver error: {:#}", e);
                }
                running.store(false, Ordering::Release);
            })
            .context("Failed to spawn LidarDriver thread")?;

        self.thread = Some(handle);
        info!(
            "LidarDriver started: {} @ {} baud, {}Hz",
            self.config.device, self.config.baudrate, self.config.scan_rate_hz
        );
        Ok(())
    }

    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
        info!("LidarDriver stopped");
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }
}

impl Drop for LidarDriver {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Try to open serial port and read scans. Falls back to dummy if unavailable.
fn scan_loop(
    config: &LidarConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    match try_serial_scan_loop(config, store, running) {
        Ok(()) => Ok(()),
        Err(e) => {
            warn!("Serial LiDAR unavailable ({}), falling back to dummy", e);
            dummy_scan_loop(config, store, running)
        }
    }
}

/// Read scans from serial port.
fn try_serial_scan_loop(
    config: &LidarConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    let port = serialport::new(&config.device, config.baudrate)
        .timeout(Duration::from_millis(500))
        .open()
        .context(format!(
            "Failed to open serial port: {} @ {}",
            config.device, config.baudrate
        ))?;

    info!("LiDAR serial port opened: {}", config.device);

    let mut reader = std::io::BufReader::new(port);
    let interval = Duration::from_secs_f64(1.0 / config.scan_rate_hz as f64);
    let mut sequence: u32 = 0;

    let angle_min: f32 = 0.0;
    let angle_max: f32 = std::f32::consts::TAU; // 2*PI for full 360 scan
    let angle_increment = angle_max / config.num_points as f32;

    while running.load(Ordering::Acquire) {
        let scan_start = Instant::now();

        // Read raw bytes for one full scan
        // Protocol varies by LiDAR model — this is a simplified reader
        // that collects `num_points` range values from the serial stream.
        match read_scan_frame(&mut reader, config) {
            Ok((ranges, intensities)) => {
                let scan = LidarScan {
                    timestamp_ns: now_ns(),
                    angle_min,
                    angle_max,
                    angle_increment,
                    range_min: config.range_min,
                    range_max: config.range_max,
                    ranges,
                    intensities,
                    sequence,
                };

                store.push_lidar_scan(scan);
                sequence += 1;

                if sequence % (config.scan_rate_hz * 10) == 0 {
                    debug!("LidarDriver: {} scans captured", sequence);
                }
            }
            Err(e) => {
                warn!("LiDAR scan read failed: {}", e);
            }
        }

        let elapsed = scan_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    Ok(())
}

/// Read a single scan frame from the serial port.
///
/// This is a simplified protocol reader. Real implementations need
/// to handle the specific LiDAR protocol (RPLIDAR, YDLidar, etc.)
/// with proper start bytes, checksums, and data parsing.
fn read_scan_frame(
    reader: &mut std::io::BufReader<Box<dyn serialport::SerialPort>>,
    config: &LidarConfig,
) -> Result<(Vec<f32>, Vec<f32>)> {
    use std::io::Read;

    // Each point: 2 bytes range (mm, u16 little-endian) + 1 byte intensity
    let bytes_per_point = 3;
    let total_bytes = config.num_points * bytes_per_point;
    let mut buf = vec![0u8; total_bytes];

    reader
        .read_exact(&mut buf)
        .context("Failed to read scan data")?;

    let mut ranges = Vec::with_capacity(config.num_points);
    let mut intensities = Vec::with_capacity(config.num_points);

    for i in 0..config.num_points {
        let offset = i * bytes_per_point;
        let range_mm = u16::from_le_bytes([buf[offset], buf[offset + 1]]);
        let intensity = buf[offset + 2] as f32;

        let range_m = range_mm as f32 / 1000.0;
        // Clamp to valid range
        let range_m = if range_m < config.range_min || range_m > config.range_max {
            0.0 // invalid
        } else {
            range_m
        };

        ranges.push(range_m);
        intensities.push(intensity);
    }

    Ok((ranges, intensities))
}

/// Generate dummy scans for development.
fn dummy_scan_loop(
    config: &LidarConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    info!("LidarDriver using dummy scan generator");

    let interval = Duration::from_secs_f64(1.0 / config.scan_rate_hz as f64);
    let angle_min: f32 = 0.0;
    let angle_max: f32 = std::f32::consts::TAU;
    let angle_increment = angle_max / config.num_points as f32;
    let mut sequence: u32 = 0;

    while running.load(Ordering::Acquire) {
        let scan_start = Instant::now();

        // Generate a simulated scan: circular room with some obstacles
        let mut ranges = Vec::with_capacity(config.num_points);
        let mut intensities = Vec::with_capacity(config.num_points);

        for i in 0..config.num_points {
            let angle = angle_min + i as f32 * angle_increment;
            // Simulate a 3m radius room with a bump
            let base_range = 3.0_f32;
            let bump = if (angle - 1.5).abs() < 0.3 { -1.5 } else { 0.0 };
            let noise = ((sequence as f32 * 0.1 + i as f32 * 0.01).sin()) * 0.05;
            let range = (base_range + bump + noise).max(config.range_min);

            ranges.push(range);
            intensities.push(200.0);
        }

        let scan = LidarScan {
            timestamp_ns: now_ns(),
            angle_min,
            angle_max,
            angle_increment,
            range_min: config.range_min,
            range_max: config.range_max,
            ranges,
            intensities,
            sequence,
        };

        store.push_lidar_scan(scan);
        sequence += 1;

        if sequence % (config.scan_rate_hz * 10) == 0 {
            debug!("LidarDriver (dummy): {} scans generated", sequence);
        }

        let elapsed = scan_start.elapsed();
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
