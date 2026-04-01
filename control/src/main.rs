/// Limo Drive — Control Process (Process 3)
///
/// Safety-critical process that owns chassis hardware communication.
/// Subscribes to CH2 (ControlCommand from Planning).
/// Publishes on CH3 (VehicleState to SensPerc + Planning).
///
/// Architecture:
///   [CH2: ControlCommand] ──> Tracker ──> Kinematics ──> ChassisDriver ──> Motors
///                              ChassisDriver ──> Kinematics (odom) ──> [CH3: VehicleState]
///                              Watchdog ──> EmergencyStop (overrides all)
mod chassis;
mod config;
mod kinematics;
mod proto;
mod tracker;
mod watchdog;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, error, info, warn};
use tracing_subscriber::EnvFilter;

use chassis::{ChassisDriver, ChassisState, MotorCommand};
use config::{load_config, ControlConfig};
use kinematics::KinematicsEngine;
use tracker::{TrajectoryPoint, TrajectoryTracker};
use watchdog::Watchdog;

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: Control Process Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/control.yaml".into());
    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!("Failed to load config from '{}': {}, using defaults", config_path, e);
        ControlConfig::default()
    });

    info!(
        "Config: chassis={}Hz, tracker={} {}Hz, kinematics={}, state_pub={}Hz",
        config.chassis.rate_hz,
        config.tracker.algorithm,
        config.tracker.rate_hz,
        config.kinematics.mode,
        config.state_publisher.rate_hz,
    );

    ctrlc_handler();

    // Shared state
    let chassis_state = Arc::new(ChassisState::new());
    let estop_active = Arc::new(AtomicBool::new(false));

    // Start chassis driver thread
    let mut chassis_driver = ChassisDriver::new(
        Arc::clone(&chassis_state),
        config.chassis.clone(),
    );
    chassis_driver.start()?;

    // Initialize components
    let mut kinematics = KinematicsEngine::new(config.kinematics.clone());
    let mut tracker = TrajectoryTracker::new(
        config.tracker.clone(),
        config.kinematics.wheelbase,
    );
    let mut watchdog_monitor = Watchdog::new(
        config.watchdog.clone(),
        Arc::clone(&estop_active),
    );

    // TODO: Start ZMQ subscriber for CH2 (ControlCommand from Planning)
    // TODO: Start ZMQ publisher for CH3 (VehicleState)
    // For now, the control loop runs with the watchdog and chassis in dummy mode.

    info!("Control process running — entering main loop");

    let control_rate = config.chassis.rate_hz;
    let interval = Duration::from_secs_f64(1.0 / control_rate as f64);
    let dt = 1.0 / control_rate as f64;
    let mut cycle: u64 = 0;
    let mut last_state_pub = Instant::now();
    let state_pub_interval = Duration::from_secs_f64(1.0 / config.state_publisher.rate_hz as f64);

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // --- 1. Watchdog check ---
        if let Some(reason) = watchdog_monitor.check() {
            debug!("Watchdog triggered: {:?}", reason);
        }

        // --- 2. Read chassis feedback and update odometry ---
        let feedback = chassis_state.get_feedback();
        let (odom_pose, odom_vel) = kinematics.update_odometry(&feedback, dt);

        // Update watchdog with current speed
        watchdog_monitor.update_speed(odom_vel.linear_x);

        // --- 3. Compute motor command ---
        let motor_cmd = if estop_active.load(Ordering::Acquire) {
            // E-STOP: controlled deceleration
            let decel_vel = watchdog_monitor.deceleration_velocity(dt);
            MotorCommand {
                linear_vel: decel_vel,
                angular_vel: 0.0,
            }
        } else if tracker.has_trajectory() {
            // Normal: follow trajectory
            tracker
                .compute(&kinematics::OdomPose {
                    x: odom_pose.x,
                    y: odom_pose.y,
                    theta: odom_pose.theta,
                })
                .map(|cmd| kinematics.clamp_command(&cmd))
                .unwrap_or_default()
        } else {
            // No trajectory: hold position (zero velocity)
            MotorCommand::default()
        };

        // --- 4. Send to chassis ---
        chassis_state.set_command(motor_cmd);

        // --- 5. Publish VehicleState at configured rate ---
        if last_state_pub.elapsed() >= state_pub_interval {
            // TODO: Serialize VehicleState proto and publish on CH3
            // For now, just log periodically
            last_state_pub = Instant::now();
        }

        // --- Logging ---
        cycle += 1;
        if cycle % (control_rate as u64 * 5) == 0 {
            let fb = chassis_state.get_feedback();
            info!(
                "Control cycle {}: pose=({:.2}, {:.2}, {:.1}°) vel={:.2} m/s bat={:.1}V estop={}",
                cycle,
                odom_pose.x,
                odom_pose.y,
                odom_pose.theta.to_degrees(),
                odom_vel.linear_x,
                fb.battery_voltage,
                estop_active.load(Ordering::Acquire),
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    // Shutdown: send zero velocity
    info!("Shutting down Control...");
    chassis_state.set_command(MotorCommand::default());
    thread::sleep(Duration::from_millis(50)); // let chassis driver send it
    chassis_driver.stop();
    info!("=== Control Process Stopped ===");

    Ok(())
}

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
