/// Limo Drive — Control Process (Process 3)
///
/// Safety-critical process that owns vehicle actuation.
/// Uses the HAL VehicleController trait — works with any platform
/// (Limo Pro hardware, Gazebo, Isaac Sim, dummy test).
///
/// Subscribes to CH2 (ControlCommand from Planning).
/// Publishes on CH3 (VehicleState to SensPerc + Planning).
mod config;
mod kinematics;
mod tracker;
mod watchdog;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use config::{load_config, ControlConfig};
use kinematics::KinematicsEngine;
use tracker::TrajectoryTracker;
use watchdog::Watchdog;

use limo_hal::dummy::DummyVehicleController;
use limo_hal::limo_hw::{LimoHwControlConfig, LimoHwVehicleController};
use limo_hal::sim_zmq::{SimAckermannConfig, SimZmqVehicleController};
use limo_hal::{MotorCommand, VehicleController};
use limo_transport::{Channel, HeartbeatManager, Publisher, Subscriber};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: Control Process Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/control.yaml".into());
    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!(
            "Failed to load config from '{}': {}, using defaults",
            config_path, e
        );
        ControlConfig::default()
    });

    if let Err(e) = config.kinematics.validate() {
        anyhow::bail!("Invalid control config: {}", e);
    }

    let sim_mode =
        std::env::args().any(|a| a == "--sim") || std::env::var("LIMO_SIM").is_ok_and(|v| v == "1");
    let dummy_mode = std::env::args().any(|a| a == "--dummy");

    info!(
        "Config: tracker={} {}Hz, kinematics={}, state_pub={}Hz",
        config.tracker.algorithm,
        config.tracker.rate_hz,
        config.kinematics.mode,
        config.state_publisher.rate_hz,
    );

    ctrlc_handler();

    // --- ZMQ setup ---
    let zmq_ctx = zmq::Context::new();

    let mut ch3_pub = Publisher::bind(
        &zmq_ctx,
        &config.transport.ch3_endpoint,
        Channel::VehicleState.topic(),
    )?;
    let mut ch2_sub = Subscriber::connect(
        &zmq_ctx,
        &config.transport.ch2_endpoint,
        Channel::ControlCommand.topic(),
    )?;

    info!(
        "ZMQ: pub CH3={}, sub CH2={}",
        config.transport.ch3_endpoint, config.transport.ch2_endpoint
    );

    // --- Select vehicle controller via HAL ---
    let mut controller: Box<dyn VehicleController> = if sim_mode {
        if config.sim_faults.is_active() {
            info!(
                "Platform: SimZmq (CH6/CH7) + fault injection (feedback_drop={:.2} seed={})",
                config.sim_faults.feedback_drop_rate, config.sim_faults.seed,
            );
        } else {
            info!("Platform: SimZmq (CH6/CH7)");
        }
        Box::new(SimZmqVehicleController::with_config(
            SimAckermannConfig {
                wheelbase: config.kinematics.wheelbase,
                max_steering_angle: config.kinematics.max_steering_angle,
                track_width: config.kinematics.track_width,
                wheel_radius: config.kinematics.wheel_radius,
            },
            config.sim_faults.clone(),
        ))
    } else if dummy_mode {
        info!("Platform: Dummy (simulated kinematics)");
        Box::new(DummyVehicleController::new())
    } else {
        info!("Platform: Limo Pro hardware");
        Box::new(LimoHwVehicleController::new(LimoHwControlConfig::default()))
    };

    controller.start()?;
    info!("VehicleController '{}' started", controller.name());

    // Initialize components
    let estop_active = Arc::new(AtomicBool::new(false));
    let mut kinematics = KinematicsEngine::new(config.kinematics.clone());
    let _tracker = TrajectoryTracker::new(config.tracker.clone(), config.kinematics.wheelbase);
    let mut watchdog_monitor = Watchdog::new(config.watchdog.clone(), Arc::clone(&estop_active));

    // Start heartbeat
    let mut heartbeat = HeartbeatManager::start("control")?;

    info!("Control process running — entering main loop");

    let control_rate = config.state_publisher.rate_hz.max(10);
    let interval = Duration::from_secs_f64(1.0 / control_rate as f64);
    let dt = 1.0 / control_rate as f64;
    let mut cycle: u64 = 0;
    let mut last_state_pub = Instant::now();
    let state_pub_interval = Duration::from_secs_f64(1.0 / config.state_publisher.rate_hz as f64);

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // --- 1. Check for incoming ControlCommand (non-blocking) ---
        match ch2_sub.recv::<limo_proto::ControlCommand>(Duration::from_millis(1)) {
            Ok(Some(cmd)) => {
                watchdog_monitor.notify_command_received();
                if cmd.emergency_stop {
                    watchdog_monitor.trigger_estop(watchdog::EstopReason::ExplicitRequest);
                } else if let Some(limo_proto::control_command::Command::VelocityCmd(twist)) =
                    cmd.command
                {
                    let motor = MotorCommand {
                        linear_vel: twist.linear_x,
                        angular_vel: twist.angular_z,
                    };
                    let clamped = kinematics.clamp_command(&motor);
                    let _ = controller.send_command(&clamped);
                }
            }
            Ok(None) => {}
            Err(e) => {
                debug!("CH2 recv error: {:#}", e);
            }
        }

        // --- 2. Feed heartbeat status into watchdog ---
        let hb_health = heartbeat.peer_health();
        for peer in &["sensperc", "planning"] {
            if hb_health.status(peer) == limo_transport::PeerStatus::Nominal
                || hb_health.status(peer) == limo_transport::PeerStatus::Warn
            {
                watchdog_monitor.notify_heartbeat(peer);
            }
        }

        // --- 3. Watchdog check ---
        if let Some(reason) = watchdog_monitor.check() {
            debug!("Watchdog triggered: {:?}", reason);
        }

        // --- 4. Read feedback from controller and update odometry ---
        let feedback = controller.recv_feedback().unwrap_or_default();
        let (odom_pose, odom_vel) =
            kinematics.update_odometry(&kinematics::to_kinematics_feedback(&feedback), dt);

        watchdog_monitor.update_speed(odom_vel.linear_x);

        // --- 5. E-stop override ---
        if estop_active.load(Ordering::Acquire) {
            let decel_vel = watchdog_monitor.deceleration_velocity(dt);
            let _ = controller.send_command(&MotorCommand {
                linear_vel: decel_vel,
                angular_vel: 0.0,
            });
        }

        // --- 6. Publish VehicleState on CH3 ---
        if last_state_pub.elapsed() >= state_pub_interval {
            let vehicle_state = limo_proto::VehicleState {
                header: Some(limo_proto::Header {
                    timestamp_ns: now_ns(),
                    sequence: cycle as u32,
                    frame_id: "odom".into(),
                }),
                odometry_pose: Some(limo_proto::Pose2D {
                    x: odom_pose.x,
                    y: odom_pose.y,
                    theta: odom_pose.theta,
                }),
                odometry_velocity: Some(limo_proto::Twist2D {
                    linear_x: odom_vel.linear_x,
                    linear_y: 0.0,
                    angular_z: odom_vel.angular_z,
                }),
                steering_angle: feedback.steering_angle,
                drive_mode: limo_proto::DriveMode::DriveAckermann as i32,
                battery_voltage: feedback.battery_voltage,
                ctrl_status: if estop_active.load(Ordering::Acquire) {
                    limo_proto::ControllerStatus::CtrlEstop as i32
                } else {
                    limo_proto::ControllerStatus::CtrlActive as i32
                },
            };

            if let Err(e) = ch3_pub.publish(&vehicle_state) {
                warn!("Failed to publish VehicleState: {:#}", e);
            }
            last_state_pub = Instant::now();
        }

        // --- Logging ---
        cycle += 1;
        if cycle.is_multiple_of(control_rate as u64 * 5) {
            info!(
                "Control cycle {}: pose=({:.2}, {:.2}, {:.1}°) vel={:.2} m/s bat={:.1}V estop={} ch3={}",
                cycle, odom_pose.x, odom_pose.y, odom_pose.theta.to_degrees(),
                odom_vel.linear_x, feedback.battery_voltage,
                estop_active.load(Ordering::Acquire), ch3_pub.msg_count(),
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    // Shutdown
    info!("Shutting down Control...");
    let _ = controller.send_command(&MotorCommand::default());
    thread::sleep(Duration::from_millis(50));
    controller.stop();
    heartbeat.stop();
    info!("=== Control Process Stopped ===");

    Ok(())
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
