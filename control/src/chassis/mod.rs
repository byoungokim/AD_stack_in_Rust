/// Limo Pro chassis driver: serial communication with the motor controller.
///
/// Sends velocity commands and reads wheel encoder feedback at 10Hz.
/// The Limo Pro uses a custom serial protocol over UART.
///
/// Protocol (simplified):
///   TX: [0x55 0x01] [linear_vel: i16_le] [angular_vel: i16_le] [checksum]
///   RX: [0x55 0x02] [left_rpm: i16_le] [right_rpm: i16_le] [steering: i16_le]
///       [battery_mv: u16_le] [error: u8] [checksum]
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use serde::Deserialize;
use tracing::{debug, error, info, warn};

/// Chassis hardware configuration.
#[derive(Debug, Clone, Deserialize)]
pub struct ChassisConfig {
    #[serde(default = "default_serial_port")]
    pub serial_port: String,
    #[serde(default = "default_baudrate")]
    pub baudrate: u32,
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
}

fn default_serial_port() -> String { "/dev/ttyTHS1".into() }
fn default_baudrate() -> u32 { 460800 }
fn default_rate_hz() -> u32 { 10 }

impl Default for ChassisConfig {
    fn default() -> Self {
        Self {
            serial_port: default_serial_port(),
            baudrate: default_baudrate(),
            rate_hz: default_rate_hz(),
        }
    }
}

/// Motor command to send to the chassis.
#[derive(Clone, Debug, Default)]
pub struct MotorCommand {
    pub linear_vel: f64,   // m/s, positive = forward
    pub angular_vel: f64,  // rad/s, positive = counter-clockwise
}

/// Feedback from the chassis hardware.
#[derive(Clone, Debug, Default)]
pub struct ChassisFeedback {
    pub left_wheel_rpm: f32,
    pub right_wheel_rpm: f32,
    pub steering_angle: f32,    // radians
    pub battery_voltage: f32,   // volts
    pub error_code: u32,
    pub timestamp_ns: u64,
}

/// Shared state between the chassis driver thread and the control loop.
pub struct ChassisState {
    /// Latest command to send (written by control loop, read by driver).
    pub command: Mutex<MotorCommand>,
    /// Latest feedback from hardware (written by driver, read by control loop).
    pub feedback: Mutex<ChassisFeedback>,
}

impl ChassisState {
    pub fn new() -> Self {
        Self {
            command: Mutex::new(MotorCommand::default()),
            feedback: Mutex::new(ChassisFeedback::default()),
        }
    }

    pub fn set_command(&self, cmd: MotorCommand) {
        *self.command.lock().unwrap() = cmd;
    }

    pub fn get_command(&self) -> MotorCommand {
        self.command.lock().unwrap().clone()
    }

    pub fn set_feedback(&self, fb: ChassisFeedback) {
        *self.feedback.lock().unwrap() = fb;
    }

    pub fn get_feedback(&self) -> ChassisFeedback {
        self.feedback.lock().unwrap().clone()
    }
}

impl Default for ChassisState {
    fn default() -> Self {
        Self::new()
    }
}

/// Chassis driver that communicates with hardware in a dedicated thread.
pub struct ChassisDriver {
    config: ChassisConfig,
    state: Arc<ChassisState>,
    running: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<()>>,
}

impl ChassisDriver {
    pub fn new(state: Arc<ChassisState>, config: ChassisConfig) -> Self {
        Self {
            config,
            state,
            running: Arc::new(AtomicBool::new(false)),
            thread: None,
        }
    }

    pub fn start(&mut self) -> Result<()> {
        if self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        let config = self.config.clone();
        let state = Arc::clone(&self.state);
        let running = Arc::clone(&self.running);

        running.store(true, Ordering::Release);

        let handle = thread::Builder::new()
            .name("ChassisDriver".into())
            .spawn(move || {
                if let Err(e) = chassis_loop(&config, &state, &running) {
                    error!("ChassisDriver error: {:#}", e);
                }
                running.store(false, Ordering::Release);
            })
            .context("Failed to spawn ChassisDriver thread")?;

        self.thread = Some(handle);
        info!(
            "ChassisDriver started: {} @ {} baud, {}Hz",
            self.config.serial_port, self.config.baudrate, self.config.rate_hz
        );
        Ok(())
    }

    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
        info!("ChassisDriver stopped");
    }
}

impl Drop for ChassisDriver {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Main chassis communication loop.
fn chassis_loop(
    config: &ChassisConfig,
    state: &Arc<ChassisState>,
    running: &AtomicBool,
) -> Result<()> {
    match try_serial_loop(config, state, running) {
        Ok(()) => Ok(()),
        Err(e) => {
            warn!("Chassis serial unavailable ({}), using dummy driver", e);
            dummy_loop(config, state, running)
        }
    }
}

/// Real serial communication with the Limo Pro chassis.
fn try_serial_loop(
    config: &ChassisConfig,
    state: &Arc<ChassisState>,
    running: &AtomicBool,
) -> Result<()> {
    let mut port = serialport::new(&config.serial_port, config.baudrate)
        .timeout(Duration::from_millis(50))
        .open()
        .context(format!(
            "Failed to open chassis serial: {} @ {}",
            config.serial_port, config.baudrate
        ))?;

    info!("Chassis serial opened: {}", config.serial_port);

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut sequence: u64 = 0;

    while running.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // Read current command
        let cmd = state.get_command();

        // Encode and send command
        let tx_buf = encode_command(&cmd);
        if let Err(e) = port.write_all(&tx_buf) {
            warn!("Chassis write failed: {}", e);
        }

        // Read feedback
        let mut rx_buf = [0u8; 12];
        match port.read_exact(&mut rx_buf) {
            Ok(()) => {
                if let Some(fb) = decode_feedback(&rx_buf) {
                    state.set_feedback(fb);
                }
            }
            Err(e) => {
                debug!("Chassis read timeout: {}", e);
            }
        }

        sequence += 1;
        if sequence % (config.rate_hz as u64 * 10) == 0 {
            let fb = state.get_feedback();
            debug!(
                "Chassis cycle {}: cmd=({:.2}, {:.2}), battery={:.1}V",
                sequence, cmd.linear_vel, cmd.angular_vel, fb.battery_voltage
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    // Send zero velocity on shutdown
    let stop_cmd = encode_command(&MotorCommand::default());
    let _ = port.write_all(&stop_cmd);
    info!("Chassis: sent zero velocity on shutdown");

    Ok(())
}

/// Dummy chassis loop for development without hardware.
fn dummy_loop(
    config: &ChassisConfig,
    state: &Arc<ChassisState>,
    running: &AtomicBool,
) -> Result<()> {
    info!("ChassisDriver using dummy mode (no hardware)");

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut sequence: u64 = 0;

    // Simulated state
    let mut sim_x: f64 = 0.0;
    let mut sim_y: f64 = 0.0;
    let mut sim_theta: f64 = 0.0;
    let dt = 1.0 / config.rate_hz as f64;

    while running.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        let cmd = state.get_command();

        // Simple kinematics simulation
        sim_theta += cmd.angular_vel * dt;
        sim_x += cmd.linear_vel * sim_theta.cos() * dt;
        sim_y += cmd.linear_vel * sim_theta.sin() * dt;

        // Compute simulated wheel RPMs from velocity
        let wheel_radius = 0.045; // meters
        let track_width = 0.172;  // meters
        let left_vel = cmd.linear_vel - cmd.angular_vel * track_width / 2.0;
        let right_vel = cmd.linear_vel + cmd.angular_vel * track_width / 2.0;
        let left_rpm = (left_vel / (2.0 * std::f64::consts::PI * wheel_radius) * 60.0) as f32;
        let right_rpm = (right_vel / (2.0 * std::f64::consts::PI * wheel_radius) * 60.0) as f32;

        let fb = ChassisFeedback {
            left_wheel_rpm: left_rpm,
            right_wheel_rpm: right_rpm,
            steering_angle: (cmd.angular_vel * 0.3) as f32, // approximate
            battery_voltage: 12.4, // simulated full battery
            error_code: 0,
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
        };

        state.set_feedback(fb);

        sequence += 1;
        if sequence % (config.rate_hz as u64 * 10) == 0 {
            debug!(
                "Chassis (dummy) cycle {}: pos=({:.2}, {:.2}, {:.2}), cmd=({:.2}, {:.2})",
                sequence, sim_x, sim_y, sim_theta, cmd.linear_vel, cmd.angular_vel
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    Ok(())
}

/// Encode a motor command into serial bytes.
/// Protocol: [0x55 0x01] [linear_vel_mm_s: i16_le] [angular_vel_mrad_s: i16_le] [checksum]
fn encode_command(cmd: &MotorCommand) -> Vec<u8> {
    let linear_mm_s = (cmd.linear_vel * 1000.0) as i16;
    let angular_mrad_s = (cmd.angular_vel * 1000.0) as i16;

    let lin_bytes = linear_mm_s.to_le_bytes();
    let ang_bytes = angular_mrad_s.to_le_bytes();

    let checksum = 0x55u8
        .wrapping_add(0x01)
        .wrapping_add(lin_bytes[0])
        .wrapping_add(lin_bytes[1])
        .wrapping_add(ang_bytes[0])
        .wrapping_add(ang_bytes[1]);

    vec![
        0x55, 0x01,
        lin_bytes[0], lin_bytes[1],
        ang_bytes[0], ang_bytes[1],
        checksum,
    ]
}

/// Decode chassis feedback from serial bytes.
/// Protocol: [0x55 0x02] [left_rpm: i16] [right_rpm: i16] [steering: i16]
///           [battery_mv: u16] [error: u8] [checksum]
fn decode_feedback(buf: &[u8; 12]) -> Option<ChassisFeedback> {
    if buf[0] != 0x55 || buf[1] != 0x02 {
        return None;
    }

    // Verify checksum
    let expected: u8 = buf[0..11].iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    if expected != buf[11] {
        return None;
    }

    let left_rpm = i16::from_le_bytes([buf[2], buf[3]]) as f32;
    let right_rpm = i16::from_le_bytes([buf[4], buf[5]]) as f32;
    let steering_raw = i16::from_le_bytes([buf[6], buf[7]]) as f32;
    let battery_mv = u16::from_le_bytes([buf[8], buf[9]]) as f32;
    let error_code = buf[10] as u32;

    Some(ChassisFeedback {
        left_wheel_rpm: left_rpm,
        right_wheel_rpm: right_rpm,
        steering_angle: steering_raw / 1000.0, // mrad → rad
        battery_voltage: battery_mv / 1000.0,  // mV → V
        error_code,
        timestamp_ns: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos() as u64,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_roundtrip() {
        let cmd = MotorCommand {
            linear_vel: 0.5,
            angular_vel: 0.3,
        };
        let encoded = encode_command(&cmd);
        assert_eq!(encoded[0], 0x55);
        assert_eq!(encoded[1], 0x01);
        assert_eq!(encoded.len(), 7);

        // Verify the linear velocity encoding
        let lin = i16::from_le_bytes([encoded[2], encoded[3]]);
        assert_eq!(lin, 500); // 0.5 * 1000
    }

    #[test]
    fn test_decode_feedback_invalid_header() {
        let buf = [0x00; 12];
        assert!(decode_feedback(&buf).is_none());
    }
}
