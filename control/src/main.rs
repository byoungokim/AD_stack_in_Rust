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
mod tracker;
mod watchdog;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use chassis::{ChassisDriver, ChassisState, MotorCommand};
use config::{load_config, ControlConfig};
use kinematics::KinematicsEngine;
use tracker::TrajectoryTracker;
use watchdog::Watchdog;

use limo_transport::{Channel, HeartbeatManager, Publisher, Subscriber};

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

    let sim_mode = std::env::args().any(|a| a == "--sim")
        || std::env::var("LIMO_SIM").map_or(false, |v| v == "1");

    ctrlc_handler();

    // --- ZMQ setup ---
    let zmq_ctx = zmq::Context::new();

    // CH3 publisher: VehicleState → SensPerc + Planning
    let ch3_endpoint = config.transport.ch3_endpoint.as_str();
    let mut ch3_pub = Publisher::bind(&zmq_ctx, ch3_endpoint, Channel::VehicleState.topic())?;

    // CH2 subscriber: ControlCommand from Planning
    let ch2_endpoint = config.transport.ch2_endpoint.as_str();
    let mut ch2_sub = Subscriber::connect(&zmq_ctx, ch2_endpoint, Channel::ControlCommand.topic())?;

    // Sim bridge channels (CH6 sub, CH7 pub) — only in sim mode
    let mut ch6_sub: Option<Subscriber> = None;
    let mut ch7_pub: Option<Publisher> = None;

    if sim_mode {
        info!("SIM MODE: using CH6 (SimVehicleState) and CH7 (SimControl) instead of chassis");
        ch6_sub = Some(Subscriber::connect(
            &zmq_ctx,
            Channel::SimVehicleState.connect_endpoint(),
            Channel::SimVehicleState.topic(),
        )?);
        ch7_pub = Some(Publisher::bind(
            &zmq_ctx,
            Channel::SimControl.bind_endpoint(),
            Channel::SimControl.topic(),
        )?);
    }

    info!("ZMQ: publishing VehicleState on {}, subscribing ControlCommand on {}",
          ch3_endpoint, ch2_endpoint);

    // Shared state
    let chassis_state = Arc::new(ChassisState::new());
    let estop_active = Arc::new(AtomicBool::new(false));

    // Start chassis driver thread (only in real mode)
    let mut chassis_driver = if sim_mode {
        info!("SIM MODE: chassis driver disabled");
        None
    } else {
        let mut driver = ChassisDriver::new(
            Arc::clone(&chassis_state),
            config.chassis.clone(),
        );
        driver.start()?;
        Some(driver)
    };

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

    // Start heartbeat manager (publishes on :5572, subscribes to peers)
    let mut heartbeat = HeartbeatManager::start("control")?;

    info!("Control process running — entering main loop");

    let control_rate = config.chassis.rate_hz;
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
                } else if let Some(limo_proto::control_command::Command::VelocityCmd(twist)) = cmd.command {
                    // Direct velocity command
                    let motor = MotorCommand {
                        linear_vel: twist.linear_x,
                        angular_vel: twist.angular_z,
                    };
                    let clamped = kinematics.clamp_command(&motor);
                    chassis_state.set_command(clamped);
                }
                // TODO: handle trajectory_cmd variant
            }
            Ok(None) => {} // timeout, no message
            Err(e) => {
                debug!("CH2 recv error: {:#}", e);
            }
        }

        // --- 2. Feed peer heartbeat status into watchdog ---
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

        // --- 3. Read chassis feedback and update odometry ---
        // In sim mode, update chassis_state from CH6 (SimVehicleState)
        if let Some(ref mut sub) = ch6_sub {
            if let Ok(Some(sim_vs)) = sub.recv::<limo_proto::SimVehicleState>(Duration::from_millis(1)) {
                let pose = sim_vs.pose.unwrap_or_default();
                let vel = sim_vs.velocity.unwrap_or_default();
                chassis_state.set_feedback(chassis::ChassisFeedback {
                    left_wheel_rpm: 0.0,
                    right_wheel_rpm: 0.0,
                    steering_angle: sim_vs.steering_angle,
                    battery_voltage: sim_vs.battery_voltage,
                    error_code: 0,
                    timestamp_ns: now_ns(),
                });
            }
        }

        let feedback = chassis_state.get_feedback();
        let (odom_pose, odom_vel) = kinematics.update_odometry(&feedback, dt);

        watchdog_monitor.update_speed(odom_vel.linear_x);

        // --- 4. E-stop override ---
        if estop_active.load(Ordering::Acquire) {
            let decel_vel = watchdog_monitor.deceleration_velocity(dt);
            chassis_state.set_command(MotorCommand {
                linear_vel: decel_vel,
                angular_vel: 0.0,
            });
        } else if tracker.has_trajectory() {
            if let Some(cmd) = tracker.compute(&kinematics::OdomPose {
                x: odom_pose.x,
                y: odom_pose.y,
                theta: odom_pose.theta,
            }) {
                let clamped = kinematics.clamp_command(&cmd);
                chassis_state.set_command(clamped);
            }
        }

        // --- 5. Forward control to sim (CH7) in sim mode ---
        if let Some(ref mut pub7) = ch7_pub {
            let motor_cmd = chassis_state.get_command();
            let sim_cmd = limo_proto::SimControlCommand {
                header: Some(limo_proto::Header {
                    timestamp_ns: now_ns(),
                    sequence: cycle as u32,
                    frame_id: "".into(),
                }),
                linear_velocity: motor_cmd.linear_vel as f32,
                angular_velocity: motor_cmd.angular_vel as f32,
                steering_angle: kinematics.velocity_to_steering(&motor_cmd) as f32,
                emergency_stop: estop_active.load(Ordering::Acquire),
            };
            let _ = pub7.publish(&sim_cmd);
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
        if cycle % (control_rate as u64 * 5) == 0 {
            info!(
                "Control cycle {}: pose=({:.2}, {:.2}, {:.1}°) vel={:.2} m/s bat={:.1}V estop={} ch3_sent={}",
                cycle,
                odom_pose.x,
                odom_pose.y,
                odom_pose.theta.to_degrees(),
                odom_vel.linear_x,
                feedback.battery_voltage,
                estop_active.load(Ordering::Acquire),
                ch3_pub.msg_count(),
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    // Shutdown
    info!("Shutting down Control...");
    chassis_state.set_command(MotorCommand::default());
    thread::sleep(Duration::from_millis(50));
    if let Some(mut driver) = chassis_driver { driver.stop(); }
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
