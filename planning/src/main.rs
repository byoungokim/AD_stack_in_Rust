/// Limo Drive — Planning Process (Process 2)
///
/// All decision-making: behavior planning, global path planning (Hybrid A*),
/// local trajectory planning (DWA + MPC fallback), E2E inference, and
/// pipeline arbitration with safety envelope.
///
/// Subscribes:
///   CH1 (tcp:5551): WorldState from SensPerc
///   CH3 (tcp:5553): VehicleState from Control
/// Publishes:
///   CH2 (tcp:5552): ControlCommand to Control
mod arbitrator;
mod behavior;
mod config;
mod e2e;
mod global_planner;
mod local_planner;

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use arbitrator::Arbitrator;
use behavior::{BehaviorInput, BehaviorPlanner};
use config::{load_config, PlanningConfig};
use e2e::E2EInference;
use global_planner::{HybridAStar, OccupancyGrid, Pose, PathWaypoint};
use local_planner::{LocalPlanner, Obstacle, RobotState, VelocityCommand};

use limo_transport::{Channel, Publisher, Subscriber};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: Planning Process Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/planning.yaml".into());
    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!("Failed to load config from '{}': {}, using defaults", config_path, e);
        PlanningConfig::default()
    });

    info!(
        "Config: behavior={}Hz, global=Hybrid A*, local=DWA+MPC {}Hz, arb={}Hz, e2e={}",
        config.behavior.rate_hz,
        config.local_planner.rate_hz,
        config.arbitrator.rate_hz,
        config.e2e.enabled,
    );

    ctrlc_handler();

    // Start heartbeat manager
    let mut heartbeat = limo_transport::HeartbeatManager::start("planning")?;

    // --- ZMQ setup ---
    let zmq_ctx = zmq::Context::new();

    // CH1 subscriber: WorldState from SensPerc
    let mut ch1_sub = Subscriber::connect(
        &zmq_ctx,
        &config.transport.ch1_endpoint,
        Channel::WorldState.topic(),
    )?;

    // CH3 subscriber: VehicleState from Control
    let mut ch3_sub = Subscriber::connect(
        &zmq_ctx,
        &config.transport.ch3_endpoint,
        Channel::VehicleState.topic(),
    )?;

    // CH2 publisher: ControlCommand to Control
    let mut ch2_pub = Publisher::bind(
        &zmq_ctx,
        &config.transport.ch2_endpoint,
        Channel::ControlCommand.topic(),
    )?;

    info!(
        "ZMQ: sub CH1={}, sub CH3={}, pub CH2={}",
        config.transport.ch1_endpoint,
        config.transport.ch3_endpoint,
        config.transport.ch2_endpoint,
    );

    // --- Initialize components ---
    let mut behavior = BehaviorPlanner::new(config.behavior.clone());
    let global = HybridAStar::new(config.global_planner.clone());
    let mut local = LocalPlanner::new(config.local_planner.clone());
    let mut arb = Arbitrator::new(config.arbitrator.clone());
    let e2e_engine = E2EInference::new(config.e2e.clone());

    // State
    let mut global_path: Vec<PathWaypoint> = Vec::new();
    let mut last_global_plan = Instant::now();
    let global_plan_interval = Duration::from_secs(1); // 1Hz

    let local_rate = config.local_planner.rate_hz;
    let interval = Duration::from_secs_f64(1.0 / local_rate as f64);
    let dt = 1.0 / local_rate as f64;
    let mut cycle: u64 = 0;

    // Default empty grid (will be updated from WorldState)
    let mut grid = OccupancyGrid::new(200, 200, 0.05, -5.0, -5.0);

    info!("Planning process running — entering main loop");

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // --- 1. Receive WorldState from SensPerc (CH1) ---
        let world_state = match ch1_sub.recv::<limo_proto::WorldState>(Duration::from_millis(1)) {
            Ok(Some(ws)) => Some(ws),
            Ok(None) => None,
            Err(e) => { debug!("CH1 recv error: {:#}", e); None }
        };

        // --- 2. Receive VehicleState from Control (CH3) ---
        let vehicle_state = match ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(1)) {
            Ok(Some(vs)) => Some(vs),
            Ok(None) => None,
            Err(e) => { debug!("CH3 recv error: {:#}", e); None }
        };

        // --- 3. Build robot state ---
        let robot_state = build_robot_state(&world_state, &vehicle_state);
        let obstacles = extract_obstacles(&world_state);

        // --- 4. Behavior planner ---
        let behavior_input = BehaviorInput {
            robot_x: robot_state.x,
            robot_y: robot_state.y,
            robot_theta: robot_state.theta,
            localization_confidence: world_state.as_ref()
                .map(|ws| ws.localization_confidence)
                .unwrap_or(0.0),
            nearest_obstacle_distance: obstacles.iter()
                .map(|o| ((o.x - robot_state.x).powi(2) + (o.y - robot_state.y).powi(2)).sqrt())
                .fold(f64::INFINITY, f64::min),
            emergency_stop: false,
        };

        let behavior_out = behavior.update(&behavior_input);

        // --- 5. Global planner (1Hz) ---
        if behavior_out.replan_requested && last_global_plan.elapsed() >= global_plan_interval {
            // TODO: use actual goal from behavior/external command
            let start = Pose {
                x: robot_state.x,
                y: robot_state.y,
                theta: robot_state.theta,
            };
            let goal = Pose { x: 5.0, y: 0.0, theta: 0.0 }; // placeholder

            if let Some(path) = global.plan(&start, &goal, &grid) {
                global_path = path;
                debug!("Global path found: {} waypoints", global_path.len());
            } else {
                debug!("Global planner: no path found");
            }
            last_global_plan = Instant::now();
        }

        // --- 6. Local planner (10Hz) ---
        let trad_cmd = local.compute(
            &robot_state,
            &global_path,
            &obstacles,
            behavior_out.desired_speed,
        );

        // --- 7. E2E inference (if enabled) ---
        let e2e_cmd = if e2e_engine.is_enabled() {
            e2e_engine.infer(&[]) // TODO: pass actual sensor data
        } else {
            None
        };

        // --- 8. Arbitrator ---
        let arb_out = if behavior_out.state == behavior::DrivingState::EmergencyStop {
            arb.emergency_stop()
        } else {
            arb.arbitrate(&trad_cmd, e2e_cmd.as_ref(), dt)
        };

        // --- 9. Publish ControlCommand on CH2 ---
        let control_cmd = limo_proto::ControlCommand {
            header: Some(limo_proto::Header {
                timestamp_ns: now_ns(),
                sequence: cycle as u32,
                frame_id: "".into(),
            }),
            source: match arb_out.source {
                arbitrator::PipelineMode::Traditional => limo_proto::PipelineSource::SourceTraditional as i32,
                arbitrator::PipelineMode::E2E => limo_proto::PipelineSource::SourceE2e as i32,
                arbitrator::PipelineMode::Shadow => limo_proto::PipelineSource::SourceTraditional as i32,
            },
            command: Some(limo_proto::control_command::Command::VelocityCmd(
                limo_proto::Twist2D {
                    linear_x: arb_out.command.linear_x,
                    linear_y: 0.0,
                    angular_z: arb_out.command.angular_z,
                },
            )),
            confidence: arb_out.command.confidence,
            emergency_stop: arb_out.emergency_stop,
        };

        if let Err(e) = ch2_pub.publish(&control_cmd) {
            warn!("Failed to publish ControlCommand: {:#}", e);
        }

        // --- Logging ---
        cycle += 1;
        if cycle % (local_rate as u64 * 5) == 0 {
            info!(
                "Planning cycle {}: state={:?} speed={:.2} path={} mpc={} ch2_sent={} clipped={}",
                cycle,
                behavior_out.state,
                arb_out.command.linear_x,
                global_path.len(),
                local.is_using_mpc(),
                ch2_pub.msg_count(),
                arb_out.safety_clipped,
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    heartbeat.stop();
    info!("=== Planning Process Stopped ===");
    Ok(())
}

fn build_robot_state(
    world: &Option<limo_proto::WorldState>,
    vehicle: &Option<limo_proto::VehicleState>,
) -> RobotState {
    // Prefer world state pose (from SLAM), fallback to vehicle odometry
    if let Some(ws) = world {
        if let Some(pose) = &ws.robot_pose {
            let vel = ws.robot_velocity.as_ref();
            return RobotState {
                x: pose.x, y: pose.y, theta: pose.theta,
                linear_vel: vel.map(|v| v.linear_x).unwrap_or(0.0),
                angular_vel: vel.map(|v| v.angular_z).unwrap_or(0.0),
            };
        }
    }

    if let Some(vs) = vehicle {
        if let Some(pose) = &vs.odometry_pose {
            let vel = vs.odometry_velocity.as_ref();
            return RobotState {
                x: pose.x, y: pose.y, theta: pose.theta,
                linear_vel: vel.map(|v| v.linear_x).unwrap_or(0.0),
                angular_vel: vel.map(|v| v.angular_z).unwrap_or(0.0),
            };
        }
    }

    RobotState::default()
}

fn extract_obstacles(world: &Option<limo_proto::WorldState>) -> Vec<Obstacle> {
    let mut obstacles = Vec::new();
    if let Some(ws) = world {
        if let Some(dets) = &ws.detections {
            for det in &dets.detections {
                if let Some(pos) = &det.position_world {
                    obstacles.push(Obstacle { x: pos.x, y: pos.y });
                }
            }
        }
    }
    obstacles
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
