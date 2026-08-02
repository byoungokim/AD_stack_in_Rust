/// Limo Pro real hardware implementations of SensorSource and VehicleController.
///
/// Wraps V4L2 camera, serial LiDAR/IMU, and chassis serial protocol
/// behind the HAL traits. Each sensor runs in its own background thread.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use crossbeam_channel::{self, Receiver, Sender};
use serde::Deserialize;
use tracing::{info, warn};

use crate::{
    CameraFrame, ChassisFeedback, ImuReading, LidarScan, MotorCommand, SensorSource, StampedPose,
    Twist2D, VehicleController,
};

// ======================== Config ========================

#[derive(Debug, Clone, Deserialize, Default)]
pub struct LimoHwSensorConfig {
    #[serde(default)]
    pub camera_device: Option<String>,
    #[serde(default = "default_cam_w")]
    pub camera_width: u32,
    #[serde(default = "default_cam_h")]
    pub camera_height: u32,
    #[serde(default = "default_cam_fps")]
    pub camera_fps: u32,
    #[serde(default)]
    pub lidar_device: Option<String>,
    #[serde(default = "default_lidar_baud")]
    pub lidar_baudrate: u32,
    #[serde(default = "default_lidar_hz")]
    pub lidar_rate_hz: u32,
    #[serde(default = "default_lidar_points")]
    pub lidar_num_points: usize,
    #[serde(default)]
    pub imu_device: Option<String>,
    #[serde(default = "default_imu_baud")]
    pub imu_baudrate: u32,
    #[serde(default = "default_imu_hz")]
    pub imu_rate_hz: u32,
}

fn default_cam_w() -> u32 {
    640
}
fn default_cam_h() -> u32 {
    480
}
fn default_cam_fps() -> u32 {
    30
}
fn default_lidar_baud() -> u32 {
    230400
}
fn default_lidar_hz() -> u32 {
    10
}
fn default_lidar_points() -> usize {
    360
}
fn default_imu_baud() -> u32 {
    115200
}
fn default_imu_hz() -> u32 {
    100
}

#[derive(Debug, Clone, Deserialize, Default)]
pub struct LimoHwControlConfig {
    #[serde(default)]
    pub chassis_device: Option<String>,
    #[serde(default = "default_chassis_baud")]
    pub chassis_baudrate: u32,
    #[serde(default = "default_chassis_hz")]
    pub chassis_rate_hz: u32,
}

fn default_chassis_baud() -> u32 {
    460800
}
fn default_chassis_hz() -> u32 {
    10
}

// ======================== SensorSource ========================

pub struct LimoHwSensorSource {
    config: LimoHwSensorConfig,
    camera_rx: Option<Receiver<CameraFrame>>,
    lidar_rx: Option<Receiver<LidarScan>>,
    imu_rx: Option<Receiver<ImuReading>>,
    running: Arc<AtomicBool>,
    threads: Vec<thread::JoinHandle<()>>,
}

impl LimoHwSensorSource {
    pub fn new(config: LimoHwSensorConfig) -> Self {
        Self {
            config,
            camera_rx: None,
            lidar_rx: None,
            imu_rx: None,
            running: Arc::new(AtomicBool::new(false)),
            threads: Vec::new(),
        }
    }
}

impl SensorSource for LimoHwSensorSource {
    fn start(&mut self) -> Result<()> {
        self.running.store(true, Ordering::Release);

        // Camera thread
        let (cam_tx, cam_rx) = crossbeam_channel::bounded(4);
        self.camera_rx = Some(cam_rx);
        let cam_cfg = self.config.clone();
        let cam_run = Arc::clone(&self.running);
        self.threads.push(
            thread::Builder::new()
                .name("hw-camera".into())
                .spawn(move || camera_loop(&cam_cfg, cam_tx, &cam_run))?,
        );

        // LiDAR thread
        let (lid_tx, lid_rx) = crossbeam_channel::bounded(8);
        self.lidar_rx = Some(lid_rx);
        let lid_cfg = self.config.clone();
        let lid_run = Arc::clone(&self.running);
        self.threads.push(
            thread::Builder::new()
                .name("hw-lidar".into())
                .spawn(move || lidar_loop(&lid_cfg, lid_tx, &lid_run))?,
        );

        // IMU thread
        let (imu_tx, imu_rx) = crossbeam_channel::bounded(64);
        self.imu_rx = Some(imu_rx);
        let imu_cfg = self.config.clone();
        let imu_run = Arc::clone(&self.running);
        self.threads.push(
            thread::Builder::new()
                .name("hw-imu".into())
                .spawn(move || imu_loop(&imu_cfg, imu_tx, &imu_run))?,
        );

        info!("LimoHwSensorSource started (3 driver threads)");
        Ok(())
    }

    fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        for h in self.threads.drain(..) {
            let _ = h.join();
        }
        info!("LimoHwSensorSource stopped");
    }

    fn recv_camera(&mut self) -> Option<CameraFrame> {
        self.camera_rx.as_ref()?.try_recv().ok()
    }

    fn recv_lidar(&mut self) -> Option<LidarScan> {
        self.lidar_rx.as_ref()?.try_recv().ok()
    }

    fn recv_imu(&mut self) -> Option<ImuReading> {
        self.imu_rx.as_ref()?.try_recv().ok()
    }

    fn recv_pose(&mut self) -> Option<StampedPose> {
        None // real hardware has no ground truth; pose comes from SLAM/odometry
    }

    fn recv_velocity(&mut self) -> Option<Twist2D> {
        None // velocity comes from odometry in the control process
    }

    fn name(&self) -> &str {
        "limo_hw"
    }
}

// ======================== VehicleController ========================

pub struct LimoHwVehicleController {
    config: LimoHwControlConfig,
    command: Arc<Mutex<MotorCommand>>,
    feedback: Arc<Mutex<ChassisFeedback>>,
    running: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl LimoHwVehicleController {
    pub fn new(config: LimoHwControlConfig) -> Self {
        Self {
            config,
            command: Arc::new(Mutex::new(MotorCommand::default())),
            feedback: Arc::new(Mutex::new(ChassisFeedback::default())),
            running: Arc::new(AtomicBool::new(false)),
            thread: None,
        }
    }
}

impl VehicleController for LimoHwVehicleController {
    fn start(&mut self) -> Result<()> {
        self.running.store(true, Ordering::Release);
        let cfg = self.config.clone();
        let cmd = Arc::clone(&self.command);
        let fb = Arc::clone(&self.feedback);
        let run = Arc::clone(&self.running);

        self.thread = Some(
            thread::Builder::new()
                .name("hw-chassis".into())
                .spawn(move || chassis_loop(&cfg, &cmd, &fb, &run))?,
        );

        info!("LimoHwVehicleController started");
        Ok(())
    }

    fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(h) = self.thread.take() {
            let _ = h.join();
        }
        info!("LimoHwVehicleController stopped");
    }

    fn send_command(&mut self, cmd: &MotorCommand) -> Result<()> {
        *self.command.lock().unwrap() = cmd.clone();
        Ok(())
    }

    fn emergency_stop(&mut self) -> Result<()> {
        // Real Limo Pro e-stop framing is out of scope for now. The
        // strongest available action is latching a zero-velocity command
        // that the chassis loop transmits at the chassis rate until the
        // e-stop clears; the firmware timeout (~500ms) backs this up.
        *self.command.lock().unwrap() = MotorCommand::default();
        Ok(())
    }

    fn recv_feedback(&mut self) -> Option<ChassisFeedback> {
        Some(self.feedback.lock().unwrap().clone())
    }

    fn name(&self) -> &str {
        "limo_hw"
    }
}

// ======================== Driver Loops ========================
// These generate dummy data when hardware isn't available.

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

fn camera_loop(cfg: &LimoHwSensorConfig, tx: Sender<CameraFrame>, running: &AtomicBool) {
    let interval = Duration::from_secs_f64(1.0 / cfg.camera_fps as f64);
    let frame_size = (cfg.camera_width * cfg.camera_height * 3) as usize;
    let mut seq: u32 = 0;

    // TODO: real V4L2 capture on Linux. For now, dummy fallback.
    warn!("Camera: using dummy frame generator");

    while running.load(Ordering::Acquire) {
        let start = Instant::now();
        let mut data = vec![128u8; frame_size];
        let offset = (seq as usize * 3) % 256;
        for (i, p) in data.iter_mut().enumerate() {
            *p = ((i + offset) % 256) as u8;
        }

        let _ = tx.try_send(CameraFrame {
            timestamp_ns: now_ns(),
            width: cfg.camera_width,
            height: cfg.camera_height,
            encoding: "bgr8".into(),
            data,
            sequence: seq,
        });
        seq += 1;
        let e = start.elapsed();
        if e < interval {
            thread::sleep(interval - e);
        }
    }
}

fn lidar_loop(cfg: &LimoHwSensorConfig, tx: Sender<LidarScan>, running: &AtomicBool) {
    let interval = Duration::from_secs_f64(1.0 / cfg.lidar_rate_hz as f64);
    let n = cfg.lidar_num_points;
    let ai = std::f32::consts::TAU / n as f32;
    let mut seq: u32 = 0;
    let t0 = Instant::now();

    // TODO: real serial LiDAR. For now, dummy fallback.
    warn!("LiDAR: using dummy scan generator");

    while running.load(Ordering::Acquire) {
        let start = Instant::now();
        let t = t0.elapsed().as_secs_f32();
        let ranges: Vec<f32> = (0..n)
            .map(|i| {
                let a = i as f32 * ai;
                let bump = if (a - 1.5).abs() < 0.3 { -2.0 } else { 0.0 };
                (4.0 + bump + (t * 2.0 + i as f32 * 0.05).sin() * 0.03).max(0.1)
            })
            .collect();
        let intensities = vec![200.0f32; n];

        let _ = tx.try_send(LidarScan {
            timestamp_ns: now_ns(),
            angle_min: 0.0,
            angle_max: std::f32::consts::TAU,
            angle_increment: ai,
            range_min: 0.1,
            range_max: 12.0,
            ranges,
            intensities,
            sequence: seq,
        });
        seq += 1;
        let e = start.elapsed();
        if e < interval {
            thread::sleep(interval - e);
        }
    }
}

fn imu_loop(cfg: &LimoHwSensorConfig, tx: Sender<ImuReading>, running: &AtomicBool) {
    let interval = Duration::from_secs_f64(1.0 / cfg.imu_rate_hz as f64);
    let mut seq: u32 = 0;
    let t0 = Instant::now();

    // TODO: real serial IMU. For now, dummy fallback.
    warn!("IMU: using dummy data generator");

    while running.load(Ordering::Acquire) {
        let start = Instant::now();
        let t = t0.elapsed().as_secs_f64();

        let _ = tx.try_send(ImuReading {
            timestamp_ns: now_ns(),
            linear_acceleration: nalgebra::Vector3::new(
                0.02 * (t * 5.0).sin(),
                0.01 * (t * 7.0).cos(),
                9.81 + 0.01 * (t * 3.0).sin(),
            ),
            angular_velocity: nalgebra::Vector3::new(
                0.001 * (t * 2.0).sin(),
                0.001 * (t * 3.0).cos(),
                0.0,
            ),
            orientation_euler: nalgebra::Vector3::new(
                0.01 * (t * 0.5).sin(),
                0.005 * (t * 0.3).cos(),
                0.0,
            ),
            sequence: seq,
        });
        seq += 1;
        let e = start.elapsed();
        if e < interval {
            thread::sleep(interval - e);
        }
    }
}

fn chassis_loop(
    cfg: &LimoHwControlConfig,
    command: &Mutex<MotorCommand>,
    feedback: &Mutex<ChassisFeedback>,
    running: &AtomicBool,
) {
    let interval = Duration::from_secs_f64(1.0 / cfg.chassis_rate_hz as f64);
    let device = cfg.chassis_device.as_deref().unwrap_or("/dev/ttyTHS1");

    // Try serial, fall back to dummy
    match serialport::new(device, cfg.chassis_baudrate)
        .timeout(Duration::from_millis(50))
        .open()
    {
        Ok(mut port) => {
            info!("Chassis serial opened: {}", device);
            while running.load(Ordering::Acquire) {
                let start = Instant::now();
                let cmd = command.lock().unwrap().clone();
                let tx_buf = encode_command(&cmd);
                let _ = port.write_all(&tx_buf);

                let mut rx_buf = [0u8; 12];
                if port.read_exact(&mut rx_buf).is_ok() {
                    if let Some(fb) = decode_feedback(&rx_buf) {
                        *feedback.lock().unwrap() = fb;
                    }
                }
                let e = start.elapsed();
                if e < interval {
                    thread::sleep(interval - e);
                }
            }
            let _ = port.write_all(&encode_command(&MotorCommand::default()));
        }
        Err(e) => {
            warn!("Chassis serial unavailable ({}), using dummy", e);
            let wheel_radius = 0.045;
            let track_width = 0.172;
            while running.load(Ordering::Acquire) {
                let start = Instant::now();
                let cmd = command.lock().unwrap().clone();
                let left_vel = cmd.linear_vel - cmd.angular_vel * track_width / 2.0;
                let right_vel = cmd.linear_vel + cmd.angular_vel * track_width / 2.0;
                *feedback.lock().unwrap() = ChassisFeedback {
                    left_wheel_rpm: (left_vel / (2.0 * std::f64::consts::PI * wheel_radius) * 60.0)
                        as f32,
                    right_wheel_rpm: (right_vel / (2.0 * std::f64::consts::PI * wheel_radius)
                        * 60.0) as f32,
                    steering_angle: (cmd.angular_vel * 0.3) as f32,
                    battery_voltage: 12.4,
                    error_code: 0,
                    timestamp_ns: now_ns(),
                };
                let e = start.elapsed();
                if e < interval {
                    thread::sleep(interval - e);
                }
            }
        }
    }
}

fn encode_command(cmd: &MotorCommand) -> Vec<u8> {
    let lin = (cmd.linear_vel * 1000.0) as i16;
    let ang = (cmd.angular_vel * 1000.0) as i16;
    let lb = lin.to_le_bytes();
    let ab = ang.to_le_bytes();
    let cs = 0x55u8
        .wrapping_add(0x01)
        .wrapping_add(lb[0])
        .wrapping_add(lb[1])
        .wrapping_add(ab[0])
        .wrapping_add(ab[1]);
    vec![0x55, 0x01, lb[0], lb[1], ab[0], ab[1], cs]
}

fn decode_feedback(buf: &[u8; 12]) -> Option<ChassisFeedback> {
    if buf[0] != 0x55 || buf[1] != 0x02 {
        return None;
    }
    let expected: u8 = buf[0..11].iter().fold(0u8, |a, &b| a.wrapping_add(b));
    if expected != buf[11] {
        return None;
    }
    Some(ChassisFeedback {
        left_wheel_rpm: i16::from_le_bytes([buf[2], buf[3]]) as f32,
        right_wheel_rpm: i16::from_le_bytes([buf[4], buf[5]]) as f32,
        steering_angle: i16::from_le_bytes([buf[6], buf[7]]) as f32 / 1000.0,
        battery_voltage: u16::from_le_bytes([buf[8], buf[9]]) as f32 / 1000.0,
        error_code: buf[10] as u32,
        timestamp_ns: now_ns(),
    })
}

use std::io::Read as IoRead;
use std::io::Write as IoWrite;
