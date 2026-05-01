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
use global_planner::{HybridAStar, OccupancyGrid, PathWaypoint, Pose};
use local_planner::{LocalPlanner, Obstacle, RobotState};

use limo_transport::{Channel, Publisher, Subscriber};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: Planning Process Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/planning.yaml".into());
    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!(
            "Failed to load config from '{}': {}, using defaults",
            config_path, e
        );
        PlanningConfig::default()
    });

    if let Err(e) = config.arbitrator.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }

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

    // CH8 subscriber: ScenarioCommand from Scenario Manager
    let mut ch8_sub = Subscriber::connect(
        &zmq_ctx,
        Channel::ScenarioCommand.connect_endpoint(),
        Channel::ScenarioCommand.topic(),
    )?;

    // CH9 publisher: ScenarioStatus feedback
    let mut ch9_pub = Publisher::bind(
        &zmq_ctx,
        Channel::ScenarioStatus.bind_endpoint(),
        Channel::ScenarioStatus.topic(),
    )?;

    // CH10 publisher: PlannedPath for visualization
    let mut ch10_pub = Publisher::bind(
        &zmq_ctx,
        Channel::PlannedPath.bind_endpoint(),
        Channel::PlannedPath.topic(),
    )?;

    info!(
        "ZMQ: sub CH1={}, sub CH3={}, pub CH2={}, sub CH8, pub CH9",
        config.transport.ch1_endpoint, config.transport.ch3_endpoint, config.transport.ch2_endpoint,
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

    // Scenario state
    let mut scenario_waypoints: Vec<limo_proto::NavigationGoal> = Vec::new();
    let mut scenario_type = limo_proto::ScenarioType::ScenarioNone as i32;
    let mut current_wp_index: usize = 0;
    let mut scenario_active = false;
    let mut scenario_speed_limit: f64 = 0.5;

    let local_rate = config.local_planner.rate_hz;
    let interval = Duration::from_secs_f64(1.0 / local_rate as f64);
    let dt = 1.0 / local_rate as f64;
    let mut cycle: u64 = 0;

    // Default empty grid (will be updated from WorldState)
    // 400x400 at 0.1m = 40m×40m, covers from -20 to +20
    let mut grid = OccupancyGrid::new(400, 400, 0.1, -20.0, -20.0);

    info!("Planning process running — entering main loop");

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // --- 1. Receive WorldState from SensPerc (CH1) ---
        let world_state = match ch1_sub.recv::<limo_proto::WorldState>(Duration::from_millis(1)) {
            Ok(Some(ws)) => Some(ws),
            Ok(None) => None,
            Err(e) => {
                debug!("CH1 recv error: {:#}", e);
                None
            }
        };

        // --- 2. Receive VehicleState from Control (CH3) ---
        let vehicle_state = match ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(1))
        {
            Ok(Some(vs)) => Some(vs),
            Ok(None) => None,
            Err(e) => {
                debug!("CH3 recv error: {:#}", e);
                None
            }
        };

        // --- 3. Receive ScenarioCommand from CH8 ---
        if let Ok(Some(cmd)) = ch8_sub.recv::<limo_proto::ScenarioCommand>(Duration::from_millis(0))
        {
            if cmd.start && scenario_active && cmd.waypoints.len() == scenario_waypoints.len() {
                // Skip duplicate start commands (re-sends from scenario manager)
            } else if cmd.start {
                scenario_waypoints = cmd.waypoints.clone();
                scenario_type = cmd.r#type;
                current_wp_index = 0;
                scenario_active = !scenario_waypoints.is_empty();
                if cmd.global_speed_limit > 0.0 {
                    scenario_speed_limit = cmd.global_speed_limit as f64;
                }

                if scenario_active {
                    let wp = &scenario_waypoints[0];
                    if let Some(pose) = &wp.goal_pose {
                        behavior.set_goal(behavior::Goal {
                            x: pose.x,
                            y: pose.y,
                            theta: pose.theta,
                        });
                        info!(
                            "Scenario started: type={}, {} waypoints, first='{}' ({:.1},{:.1})",
                            cmd.r#type,
                            scenario_waypoints.len(),
                            wp.label,
                            pose.x,
                            pose.y
                        );
                    }
                }
            } else {
                scenario_active = false;
                behavior.clear_goal();
                info!("Scenario stopped");
            }
        }

        // --- 4. Build robot state and update grid ---
        let robot_state = build_robot_state(&world_state, &vehicle_state);
        let obstacles = extract_obstacles(&world_state);

        // Populate occupancy grid from detected obstacles
        // Clear grid each cycle and re-populate (rolling local map)
        grid.data.fill(0);
        for obs in &obstacles {
            // Mark obstacle cells (inflate by robot radius for safety)
            let inflate = 0.3; // meters
            let steps = (inflate / grid.resolution) as i32;
            for dx in -steps..=steps {
                for dy in -steps..=steps {
                    grid.set_occupied(
                        obs.x + dx as f64 * grid.resolution,
                        obs.y + dy as f64 * grid.resolution,
                    );
                }
            }
        }

        // Also populate from WorldState local_map if available
        if let Some(ws) = &world_state {
            if let Some(map) = &ws.local_map {
                if map.width > 0 && map.height > 0 {
                    let origin = map.origin.as_ref();
                    let ox = origin.map(|o| o.x).unwrap_or(0.0);
                    let oy = origin.map(|o| o.y).unwrap_or(0.0);
                    for gy in 0..map.height {
                        for gx in 0..map.width {
                            let idx = (gy * map.width + gx) as usize;
                            if idx < map.data.len() && map.data[idx] >= 50 {
                                let wx = ox + gx as f64 * map.resolution as f64;
                                let wy = oy + gy as f64 * map.resolution as f64;
                                grid.set_occupied(wx, wy);
                            }
                        }
                    }
                }
            }
        }

        // --- 5. Behavior planner ---
        let behavior_input = BehaviorInput {
            robot_x: robot_state.x,
            robot_y: robot_state.y,
            robot_theta: robot_state.theta,
            localization_confidence: world_state
                .as_ref()
                .map(|ws| ws.localization_confidence)
                .unwrap_or(0.0),
            nearest_obstacle_distance: obstacles
                .iter()
                .map(|o| ((o.x - robot_state.x).powi(2) + (o.y - robot_state.y).powi(2)).sqrt())
                .fold(f64::INFINITY, f64::min),
            emergency_stop: false,
        };

        let behavior_out = behavior.update(&behavior_input);

        // --- 5a. Advance scenario waypoints on goal reached ---
        if scenario_active && behavior_out.state == behavior::DrivingState::GoalReached {
            current_wp_index += 1;

            let is_patrol = scenario_type == limo_proto::ScenarioType::ScenarioPatrol as i32;

            if current_wp_index >= scenario_waypoints.len() {
                if is_patrol {
                    current_wp_index = 0; // loop
                    info!("Patrol: looping back to first waypoint");
                } else {
                    scenario_active = false;
                    behavior.clear_goal();
                    info!(
                        "Scenario complete: all {} waypoints reached",
                        scenario_waypoints.len()
                    );
                }
            }

            if scenario_active {
                let wp = &scenario_waypoints[current_wp_index];
                if let Some(pose) = &wp.goal_pose {
                    behavior.set_goal(behavior::Goal {
                        x: pose.x,
                        y: pose.y,
                        theta: pose.theta,
                    });
                    info!(
                        "Advancing to waypoint {}/{}: '{}' ({:.1},{:.1})",
                        current_wp_index + 1,
                        scenario_waypoints.len(),
                        wp.label,
                        pose.x,
                        pose.y
                    );
                }
            }
        }

        // --- 6. Global planner (1Hz) ---
        if behavior_out.replan_requested && last_global_plan.elapsed() >= global_plan_interval {
            let start = Pose {
                x: robot_state.x,
                y: robot_state.y,
                theta: robot_state.theta,
            };

            // Use current scenario waypoint as goal
            let goal = if scenario_active && current_wp_index < scenario_waypoints.len() {
                let wp = &scenario_waypoints[current_wp_index];
                let pose = wp.goal_pose.as_ref().unwrap();
                Pose {
                    x: pose.x,
                    y: pose.y,
                    theta: pose.theta,
                }
            } else {
                Pose {
                    x: robot_state.x,
                    y: robot_state.y,
                    theta: robot_state.theta,
                }
            };

            if let Some(path) = global.plan(&start, &goal, &grid) {
                global_path = path;
                debug!("Global path found: {} waypoints", global_path.len());
            } else {
                debug!("Global planner: no path found");
            }
            last_global_plan = Instant::now();
        }

        // --- 7. Local planner (10Hz) ---
        let desired_speed = behavior_out.desired_speed.min(scenario_speed_limit);
        let local_plan = local.compute(&robot_state, &global_path, &obstacles, desired_speed);
        let trad_cmd = local_plan.command.clone();

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
        let control_cmd = arbitrator::encode_control_command(&arb_out, cycle as u32, now_ns());

        if let Err(e) = ch2_pub.publish(&control_cmd) {
            warn!("Failed to publish ControlCommand: {:#}", e);
        }

        // --- 10. Publish ScenarioStatus on CH9 ---
        if scenario_active || !scenario_waypoints.is_empty() {
            let dist_to_goal = if scenario_active && current_wp_index < scenario_waypoints.len() {
                let wp = &scenario_waypoints[current_wp_index];
                if let Some(pose) = &wp.goal_pose {
                    ((robot_state.x - pose.x).powi(2) + (robot_state.y - pose.y).powi(2)).sqrt()
                } else {
                    0.0
                }
            } else {
                0.0
            };

            let status = limo_proto::ScenarioStatus {
                header: Some(limo_proto::Header {
                    timestamp_ns: now_ns(),
                    sequence: cycle as u32,
                    frame_id: "".into(),
                }),
                active_scenario: scenario_type,
                current_waypoint_index: current_wp_index as u32,
                total_waypoints: scenario_waypoints.len() as u32,
                distance_to_goal: dist_to_goal as f32,
                goal_reached: behavior_out.state == behavior::DrivingState::GoalReached,
                scenario_complete: !scenario_active
                    && !scenario_waypoints.is_empty()
                    && current_wp_index >= scenario_waypoints.len(),
                active_label: scenario_waypoints
                    .get(current_wp_index)
                    .map(|wp| wp.label.clone())
                    .unwrap_or_default(),
            };
            let _ = ch9_pub.publish(&status);
        }

        // --- 11. Publish PlannedPath on CH10 (for visualization) ---
        {
            let planned_path = limo_proto::PlannedPath {
                header: Some(limo_proto::Header {
                    timestamp_ns: now_ns(),
                    sequence: cycle as u32,
                    frame_id: "world".into(),
                }),
                global_path: global_path
                    .iter()
                    .map(|wp| limo_proto::Pose2D {
                        x: wp.x,
                        y: wp.y,
                        theta: wp.theta,
                    })
                    .collect(),
                local_trajectory: local_plan
                    .trajectory
                    .iter()
                    .map(|p| limo_proto::Pose2D {
                        x: p.x,
                        y: p.y,
                        theta: p.theta,
                    })
                    .collect(),
                current_goal: if scenario_active && current_wp_index < scenario_waypoints.len() {
                    scenario_waypoints[current_wp_index].goal_pose
                } else {
                    None
                },
                goal_label: scenario_waypoints
                    .get(current_wp_index)
                    .map(|wp| wp.label.clone())
                    .unwrap_or_default(),
                scenario_waypoints: scenario_waypoints
                    .iter()
                    .filter_map(|wp| wp.goal_pose)
                    .collect(),
                waypoint_labels: scenario_waypoints
                    .iter()
                    .map(|wp| wp.label.clone())
                    .collect(),
                current_waypoint_index: current_wp_index as u32,
                robot_pose: Some(limo_proto::Pose2D {
                    x: robot_state.x,
                    y: robot_state.y,
                    theta: robot_state.theta,
                }),
                robot_speed: arb_out.command.linear_x as f32,
                behavior_state: format!("{:?}", behavior_out.state),
            };
            let _ = ch10_pub.publish(&planned_path);
        }

        // --- Logging ---
        cycle += 1;
        if cycle.is_multiple_of(local_rate as u64 * 5) {
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
                x: pose.x,
                y: pose.y,
                theta: pose.theta,
                linear_vel: vel.map(|v| v.linear_x).unwrap_or(0.0),
                angular_vel: vel.map(|v| v.angular_z).unwrap_or(0.0),
            };
        }
    }

    if let Some(vs) = vehicle {
        if let Some(pose) = &vs.odometry_pose {
            let vel = vs.odometry_velocity.as_ref();
            return RobotState {
                x: pose.x,
                y: pose.y,
                theta: pose.theta,
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
