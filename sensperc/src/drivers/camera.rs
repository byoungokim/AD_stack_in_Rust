/// Camera driver: captures frames via V4L2 and pushes to SensorStore.
///
/// Runs as a dedicated thread at configured FPS (default 30Hz).
/// Uses the v4l crate for V4L2 access on Linux/Jetson.
/// Falls back to a dummy generator for development on non-Linux platforms.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::Deserialize;
use tracing::{debug, error, info, warn};

use crate::store::types::CameraFrame;
use crate::store::SensorStore;

#[derive(Debug, Clone, Deserialize)]
pub struct CameraConfig {
    #[serde(default = "default_device")]
    pub device: String,
    #[serde(default = "default_width")]
    pub width: u32,
    #[serde(default = "default_height")]
    pub height: u32,
    #[serde(default = "default_fps")]
    pub fps: u32,
    #[serde(default = "default_encoding")]
    pub encoding: String,
}

fn default_device() -> String { "/dev/video0".into() }
fn default_width() -> u32 { 640 }
fn default_height() -> u32 { 480 }
fn default_fps() -> u32 { 30 }
fn default_encoding() -> String { "bgr8".into() }

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            device: default_device(),
            width: default_width(),
            height: default_height(),
            fps: default_fps(),
            encoding: default_encoding(),
        }
    }
}

pub struct CameraDriver {
    config: CameraConfig,
    store: Arc<SensorStore>,
    running: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl CameraDriver {
    pub fn new(store: Arc<SensorStore>, config: CameraConfig) -> Self {
        Self {
            config,
            store,
            running: Arc::new(AtomicBool::new(false)),
            thread: None,
        }
    }

    /// Start the camera capture thread.
    pub fn start(&mut self) -> Result<()> {
        if self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        let config = self.config.clone();
        let store = Arc::clone(&self.store);
        let running = Arc::clone(&self.running);

        running.store(true, Ordering::Release);

        let handle = thread::Builder::new()
            .name("CameraDriver".into())
            .spawn(move || {
                if let Err(e) = capture_loop(&config, &store, &running) {
                    error!("CameraDriver error: {:#}", e);
                }
                running.store(false, Ordering::Release);
            })
            .context("Failed to spawn CameraDriver thread")?;

        self.thread = Some(handle);
        info!(
            "CameraDriver started: {} @ {}x{} {}Hz",
            self.config.device, self.config.width, self.config.height, self.config.fps
        );
        Ok(())
    }

    /// Stop the camera capture thread.
    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
        info!("CameraDriver stopped");
    }

    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }
}

impl Drop for CameraDriver {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Main capture loop.
#[cfg(target_os = "linux")]
fn capture_loop(
    config: &CameraConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    use v4l::buffer::Type;
    use v4l::io::mmap::Stream;
    use v4l::io::traits::CaptureStream;
    use v4l::video::Capture;
    use v4l::Device;
    use v4l::FourCC;

    let dev = Device::with_path(&config.device)
        .context(format!("Failed to open camera: {}", config.device))?;

    // Set format
    let mut fmt = dev.format()?;
    fmt.width = config.width;
    fmt.height = config.height;
    fmt.fourcc = FourCC::new(b"YUYV");
    dev.set_format(&fmt)?;

    let actual_fmt = dev.format()?;
    info!(
        "V4L2 camera opened: {}x{} fourcc={:?}",
        actual_fmt.width, actual_fmt.height, actual_fmt.fourcc
    );

    let mut stream = Stream::with_buffers(&dev, Type::VideoCapture, 4)
        .context("Failed to create V4L2 stream")?;

    let interval = Duration::from_secs_f64(1.0 / config.fps as f64);
    let mut sequence: u32 = 0;

    while running.load(Ordering::Acquire) {
        let frame_start = Instant::now();

        match stream.next() {
            Ok((buf, _meta)) => {
                let timestamp_ns = Instant::now()
                    .duration_since(Instant::now() - frame_start)
                    .as_nanos() as u64;

                // Convert YUYV to BGR8 (simple conversion)
                let bgr_data = yuyv_to_bgr8(buf, config.width, config.height);

                let frame = CameraFrame {
                    timestamp_ns: std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as u64,
                    width: config.width,
                    height: config.height,
                    encoding: config.encoding.clone(),
                    data: bgr_data,
                    sequence,
                };

                store.push_camera_frame(frame);
                sequence += 1;

                if sequence % (config.fps * 10) == 0 {
                    debug!("CameraDriver: {} frames captured", sequence);
                }
            }
            Err(e) => {
                warn!("Camera read failed: {}, retrying...", e);
                thread::sleep(Duration::from_millis(100));
            }
        }

        // Rate limiting
        let elapsed = frame_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    Ok(())
}

/// Fallback capture loop for non-Linux platforms (generates dummy frames).
#[cfg(not(target_os = "linux"))]
fn capture_loop(
    config: &CameraConfig,
    store: &Arc<SensorStore>,
    running: &AtomicBool,
) -> Result<()> {
    warn!("V4L2 not available — using dummy camera frame generator");

    let interval = Duration::from_secs_f64(1.0 / config.fps as f64);
    let frame_size = (config.width * config.height * 3) as usize; // BGR8
    let mut sequence: u32 = 0;

    while running.load(Ordering::Acquire) {
        let frame_start = Instant::now();

        // Generate a dummy frame (gray gradient)
        let mut data = vec![128u8; frame_size];
        // Add a moving pattern so frames are visibly different
        let offset = (sequence as usize * 3) % 256;
        for (i, pixel) in data.iter_mut().enumerate() {
            *pixel = ((i + offset) % 256) as u8;
        }

        let frame = CameraFrame {
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            width: config.width,
            height: config.height,
            encoding: config.encoding.clone(),
            data,
            sequence,
        };

        store.push_camera_frame(frame);
        sequence += 1;

        if sequence % (config.fps * 10) == 0 {
            debug!("CameraDriver (dummy): {} frames generated", sequence);
        }

        let elapsed = frame_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    Ok(())
}

/// Convert YUYV (YUV 4:2:2) to BGR8.
#[cfg(target_os = "linux")]
fn yuyv_to_bgr8(yuyv: &[u8], width: u32, height: u32) -> Vec<u8> {
    let num_pixels = (width * height) as usize;
    let mut bgr = vec![0u8; num_pixels * 3];

    for i in 0..(num_pixels / 2) {
        let y0 = yuyv[i * 4] as f32;
        let u = yuyv[i * 4 + 1] as f32 - 128.0;
        let y1 = yuyv[i * 4 + 2] as f32;
        let v = yuyv[i * 4 + 3] as f32 - 128.0;

        // YUV to BGR conversion
        let r0 = (y0 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g0 = (y0 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b0 = (y0 + 1.772 * u).clamp(0.0, 255.0) as u8;

        let r1 = (y1 + 1.402 * v).clamp(0.0, 255.0) as u8;
        let g1 = (y1 - 0.344 * u - 0.714 * v).clamp(0.0, 255.0) as u8;
        let b1 = (y1 + 1.772 * u).clamp(0.0, 255.0) as u8;

        bgr[i * 6] = b0;
        bgr[i * 6 + 1] = g0;
        bgr[i * 6 + 2] = r0;
        bgr[i * 6 + 3] = b1;
        bgr[i * 6 + 4] = g1;
        bgr[i * 6 + 5] = r1;
    }

    bgr
}
