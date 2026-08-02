/// Limo Drive — Control Process (Process 3)
///
/// Safety-critical process that owns vehicle actuation.
/// Uses the HAL VehicleController trait — works with any platform
/// (Limo Pro hardware, Gazebo, Isaac Sim, dummy test).
///
/// Subscribes to CH2 (ControlCommand from Planning).
/// Publishes on CH3 (VehicleState to SensPerc + Planning).
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use limo_control::config::{load_config, ControlConfig};
use limo_control::control_loop::{build_vehicle_state, now_ns, ControlLoop};
use limo_control::tracker::TrajectoryTracker;

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
    let _tracker = TrajectoryTracker::new(config.tracker.clone(), config.kinematics.wheelbase);
    let mut ctrl_loop = ControlLoop::new(&config, Arc::clone(&estop_active));

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

        // --- 1. Drain CH2 to the NEWEST command (non-blocking) ---
        // Older queued messages are discarded: exactly one command is
        // evaluated — and at most one actuated — per cycle.
        let mut latest_cmd: Option<limo_proto::ControlCommand> = None;
        loop {
            match ch2_sub.recv::<limo_proto::ControlCommand>(Duration::ZERO) {
                Ok(Some(cmd)) => latest_cmd = Some(cmd),
                Ok(None) => break,
                Err(e) => {
                    debug!("CH2 recv error: {:#}", e);
                    break;
                }
            }
        }

        // --- 2. Feed heartbeat status into watchdog ---
        let hb_health = heartbeat.peer_health();
        for peer in &["sensperc", "planning"] {
            if hb_health.status(peer) == limo_transport::PeerStatus::Nominal
                || hb_health.status(peer) == limo_transport::PeerStatus::Warn
            {
                ctrl_loop.notify_heartbeat(peer);
            }
        }

        // --- 3. Run one safety-gated control cycle ---
        // Feedback/odometry, command age gate, watchdog check, then
        // exactly one actuation (e-stop override replaces the command).
        let out = ctrl_loop.run_cycle(latest_cmd, controller.as_mut(), dt, now_ns());

        // --- 4. Publish VehicleState on CH3 ---
        if last_state_pub.elapsed() >= state_pub_interval {
            let vehicle_state = build_vehicle_state(&out, cycle as u32, now_ns());
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
                cycle, out.odom_pose.x, out.odom_pose.y, out.odom_pose.theta.to_degrees(),
                out.odom_vel.linear_x, out.feedback.battery_voltage,
                out.estop_active, ch3_pub.msg_count(),
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

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
