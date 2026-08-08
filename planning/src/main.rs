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
mod roadmap;

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, info, warn};
use tracing_subscriber::EnvFilter;

use arbitrator::Arbitrator;
use behavior::{BehaviorInput, BehaviorPlanner, RecoveryPhase};
use config::{load_config, PlanningConfig};
use e2e::E2EInference;
use global_planner::{
    path_cost, path_remains_valid_with_escape, smoother, start_escape_zone, Corridor,
    CostPenalties, HybridAStar, OccupancyGrid, PathWaypoint, PhysicalObstacle, Pose,
};
use local_planner::dwa::speed_scaled_radius;
use local_planner::{
    active_run_truncation_index, plan_reverse_escape, LocalPlan, LocalPlanner, Obstacle,
    PursuitDefer, RearBlocker, RobotState, VelocityCommand,
};

use limo_transport::subscriber::BackgroundSubscriber;
use limo_transport::{Channel, Publisher, Subscriber};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

/// Recovery phase B reverse crawl speed (m/s).
const RECOVERY_REVERSE_SPEED: f64 = 0.1;
/// Extra clearance (m) beyond the robot radius required of the rear corridor
/// before the recovery reverse is allowed.
const REAR_CLEAR_MARGIN: f64 = 0.05;
/// Linear speeds below this (m/s) count as a zero command when classifying
/// local-plan feasibility for the stuck detector.
const PLAN_ZERO_SPEED: f64 = 0.02;
/// Distance the robot must close toward the route leg goal to count as
/// progress for the blocked-link watchdog (absorbs pose jitter).
const LEG_PROGRESS_EPS_M: f64 = 0.05;
/// Consecutive pursuit TargetBehind deferrals (10Hz cycles) before the
/// retained global path is declared heading-stale and dropped. See the
/// stale-heading invalidation in the main loop for the transient-vs-stale
/// calibration.
const TARGET_BEHIND_DROP_CYCLES: u32 = 5;
/// Consecutive pursuit Blocked deferrals (10Hz cycles) before the retained
/// global path is declared clearance-stale (wall-hugging) and dropped.
/// Longer than the TargetBehind window: transient Blocked bursts also occur
/// against legitimately narrow passages where the path is fine and a mover
/// or noise spike clears within a few cycles.
const BLOCKED_DROP_CYCLES: u32 = 8;
/// Minimum retained-path length (waypoints, ~0.25m spacing) for the
/// blocked-streak drop: shorter paths are final approaches whose replan
/// cannot differ (the goal pinch IS the blockage) — recovery and the
/// endgame direct approach own those.
const BLOCKED_DROP_MIN_WAYPOINTS: usize = 8;
/// Consecutive failed replans (4Hz) before an A* failure counts as leg
/// obstruction for the blocked-link watchdog — one transient failure
/// during a maneuver flipped whole routes (replay t≈13s, t≈35s).
const ASTAR_BLAME_MIN_FAILS: u32 = 4;
/// Lateral distance (m) from the robot to the retained global path's nearest
/// waypoint beyond which the path loses its hysteresis retention privilege
/// (see the pose-consistency check at the replan decision).
const PATH_OFFSET_INVALIDATE_M: f64 = 0.5;
/// Endgame direct approach: within this distance of the mission goal, an
/// EMPTY global path is handed to DWA as a single-waypoint path at the goal
/// instead of the deliberate confident stop. Hybrid A* can fail persistently
/// when the goal sits inside hard inflation (cluttered plaza), and the
/// confident stop never trips the stuck detector — the live run parked
/// 0.81m short of a 0.35m tolerance forever. DWA's sampling is fully
/// collision-checked (crawl requirement below A*'s hard inflation), so it
/// closes tolerance-scale gaps or reports infeasible honestly.
const GOAL_DIRECT_APPROACH_M: f64 = 2.0;
/// Margin (m) the reference-corridor sub-polyline extends past the robot's
/// route projection and past the leg goal, so plan start/end (and the RS
/// goal tail) sit comfortably inside the tube.
const CORRIDOR_END_MARGIN_M: f64 = 0.5;

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
    let mut config = load_config(&config_path).unwrap_or_else(|e| {
        warn!(
            "Failed to load config from '{}': {}, using defaults",
            config_path, e
        );
        PlanningConfig::default()
    });

    if let Err(e) = config.arbitrator.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }
    if let Err(e) = config.fault_tolerance.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }
    if let Err(e) = config.behavior.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }
    if let Err(e) = config.global_planner.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }
    if let Err(e) = config.local_planner.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }
    if let Err(e) = config.roadmap.validate() {
        anyhow::bail!("Invalid planning config: {}", e);
    }
    // Cross-config coherence: DWA must never verify dynamics the arbitrator
    // envelope then clamps into an unverified command (clipped arcs swing
    // wider than the checked ones).
    if config.local_planner.dwa.max_speed > config.arbitrator.safety.max_speed {
        anyhow::bail!(
            "Invalid planning config: dwa.max_speed ({}) exceeds safety.max_speed ({})",
            config.local_planner.dwa.max_speed,
            config.arbitrator.safety.max_speed
        );
    }
    if config.local_planner.dwa.max_deceleration > config.arbitrator.safety.max_deceleration {
        anyhow::bail!(
            "Invalid planning config: dwa.max_deceleration ({}) assumes more braking than \
             safety.max_deceleration ({}) grants",
            config.local_planner.dwa.max_deceleration,
            config.arbitrator.safety.max_deceleration
        );
    }

    // LIMO_CORRIDOR_HALF_WIDTH overrides roadmap.corridor_half_width at
    // launch (same pattern as LIMO_ROADMAP_FILE): per-world corridor
    // tightness without editing the tuned planning.yaml. The sidewalk city
    // uses 0.65 so the hard tube (x corridor_hard_factor 1.2 = 0.78m)
    // matches the 1.5m sidewalk — the robot cannot legally leave sidewalks
    // or crosswalks. Applied before validate() so bad values fail loud.
    if let Ok(s) = std::env::var("LIMO_CORRIDOR_HALF_WIDTH") {
        let w: f64 = s
            .parse()
            .map_err(|_| anyhow::anyhow!("LIMO_CORRIDOR_HALF_WIDTH is not a number: '{s}'"))?;
        info!(
            "Corridor half-width override: {} -> {} (LIMO_CORRIDOR_HALF_WIDTH)",
            config.roadmap.corridor_half_width, w
        );
        config.roadmap.corridor_half_width = w;
        if let Err(e) = config.roadmap.validate() {
            anyhow::bail!("Invalid LIMO_CORRIDOR_HALF_WIDTH: {}", e);
        }
    }

    // Prior roadmap layer: standing global route knowledge. A configured
    // roadmap that fails to load is a startup error (fail loud), never a
    // silent fallback to direct goals. LIMO_ROADMAP_FILE overrides the
    // configured path so per-world roadmaps (gauntlet vs city) can be
    // selected at launch without editing the tuned planning.yaml.
    let roadmap_file =
        std::env::var("LIMO_ROADMAP_FILE").unwrap_or_else(|_| config.roadmap.file.clone());
    let mut roadmap: Option<roadmap::Roadmap> = if config.roadmap.enabled {
        match roadmap::Roadmap::load(
            &roadmap_file,
            Duration::from_secs_f64(config.roadmap.blocked_link_timeout_s),
        ) {
            Ok(rm) => {
                info!(
                    "Roadmap loaded: {} ({} nodes, {} links)",
                    roadmap_file,
                    rm.node_count(),
                    rm.link_count(),
                );
                Some(rm)
            }
            Err(e) => anyhow::bail!("Invalid roadmap '{}': {}", roadmap_file, e),
        }
    } else {
        None
    };

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

    // CH1 subscriber: WorldState from SensPerc. Background thread + drain to
    // newest each cycle so a backlog never makes planning act on old frames.
    let ch1_sub = BackgroundSubscriber::<limo_proto::WorldState>::start(
        &zmq_ctx,
        &config.transport.ch1_endpoint,
        Channel::WorldState.topic(),
        100,
    )?;

    // CH3 subscriber: VehicleState from Control
    let ch3_sub = BackgroundSubscriber::<limo_proto::VehicleState>::start(
        &zmq_ctx,
        &config.transport.ch3_endpoint,
        Channel::VehicleState.topic(),
        100,
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
    // 4 Hz: at 1 Hz a 2 m/s robot moved 2m between replans — it outran the
    // global path near newly-perceived obstacles. The chamfer-field A* costs
    // ~1ms per plan, so 4 Hz is cheap.
    let global_plan_interval = Duration::from_millis(250);
    // Path hysteresis: the goal (x, y, theta) the retained path was planned
    // for — a changed goal always forces replacement.
    let mut current_path_goal: Option<(f64, f64, f64)> = None;
    // Rate limiter for path-replacement INFO logs (replacements can happen at
    // the full 4 Hz replan rate during genuine topology changes).
    let mut last_replace_log = Instant::now() - Duration::from_secs(60);
    // Rate limiter for the hierarchy-fallback INFO log (scripted recovery
    // reverse engaging while a global path exists — countable in sim logs).
    let mut target_behind_streak: u32 = 0;
    let mut blocked_streak: u32 = 0;
    let mut astar_fail_streak: u32 = 0;
    let mut last_scripted_fallback_log = Instant::now() - Duration::from_secs(60);
    // Rate limiter for the blocked-link WARN: a persistently failing leg
    // re-declares the block every 10Hz cycle, which flooded run logs with
    // identical "blocked, rerouting" lines. Warn at most 1Hz; repeats in
    // between are counted and reported with the next emitted line.
    let mut last_block_warn = Instant::now() - Duration::from_secs(60);
    let mut block_warns_suppressed: u32 = 0;
    // A goal counts as "reached by the path end" within the A* acceptance
    // radius plus one cell of smoothing slack.
    let goal_end_tolerance = config.global_planner.xy_resolution * 2.0 + 0.1;

    // Roadmap route state: the standing-knowledge route toward the current
    // mission waypoint plus the leg-progress watchdog for blocked-link
    // detection. Always None when roadmap.enabled is false.
    let mut active_route: Option<ActiveRoute> = None;
    let mut last_route_log = Instant::now() - Duration::from_secs(60);
    let link_block_after = Duration::from_secs_f64(config.roadmap.link_block_after_s);
    // Reference corridor from the most recent global replan, re-installed
    // into the local planner each cycle while the route stays active (the
    // replan block runs at 4 Hz; between runs the tube is near-identical).
    let mut last_corridor: Option<Corridor> = None;

    // Scenario state
    let mut scenario_waypoints: Vec<limo_proto::NavigationGoal> = Vec::new();
    let mut scenario_type = limo_proto::ScenarioType::ScenarioNone as i32;
    let mut current_wp_index: usize = 0;
    let mut scenario_active = false;
    // No global scenario limit until one arrives on CH8: an unset limit
    // (proto 0.0 = "use default") must NOT cap cruise — the old 0.5 default
    // silently dragged every leg to 0.5 m/s.
    let mut scenario_speed_limit: f64 = f64::INFINITY;

    let local_rate = config.local_planner.rate_hz;
    let interval = Duration::from_secs_f64(1.0 / local_rate as f64);
    let dt = 1.0 / local_rate as f64;
    let mut cycle: u64 = 0;

    // Default empty grid (will be updated from WorldState)
    // 400x400 at 0.1m = 40m×40m, covers from -20 to +20.
    //
    // Out-of-bounds counts as BLOCKED in the collision check, so a roadmap
    // reaching past the box would make its far links permanently unplannable
    // (the city patrol wedged at x=20.0 — the grid's east wall — with an
    // empty street ahead, blocking link after link forever). Expand the
    // bounds to the union of the default box and the roadmap's node bbox +
    // margin: strictly more plannable space, identical behavior for courses
    // that already fit (gauntlet: x<=17.9).
    const GRID_RES_M: f64 = 0.1;
    const ROADMAP_GRID_MARGIN_M: f64 = 8.0;
    let (mut gmin_x, mut gmin_y, mut gmax_x, mut gmax_y) = (-20.0f64, -20.0f64, 20.0f64, 20.0f64);
    if let Some((bx0, by0, bx1, by1)) = roadmap.as_ref().and_then(|rm| rm.bounds()) {
        gmin_x = gmin_x.min(bx0 - ROADMAP_GRID_MARGIN_M);
        gmin_y = gmin_y.min(by0 - ROADMAP_GRID_MARGIN_M);
        gmax_x = gmax_x.max(bx1 + ROADMAP_GRID_MARGIN_M);
        gmax_y = gmax_y.max(by1 + ROADMAP_GRID_MARGIN_M);
    }
    let grid_w = ((gmax_x - gmin_x) / GRID_RES_M).ceil() as usize;
    let grid_h = ((gmax_y - gmin_y) / GRID_RES_M).ceil() as usize;
    info!(
        "Planning grid: {}x{} cells at {}m ({:.0}m x {:.0}m, origin ({:.1}, {:.1}))",
        grid_w,
        grid_h,
        GRID_RES_M,
        gmax_x - gmin_x,
        gmax_y - gmin_y,
        gmin_x,
        gmin_y
    );
    let mut grid = OccupancyGrid::new(grid_w, grid_h, GRID_RES_M, gmin_x, gmin_y);

    // Short-term obstacle persistence: plan against the union of the last few
    // perception cycles so a real obstacle can't vanish between replans when
    // one scan's sample drops it or scan/pose skew during fast yaw shifts it.
    // Depth 3 (~0.3s at 10Hz): deep enough that a single-cycle dropout can't
    // open a hole, shallow enough that smear from moving pedestrians doesn't
    // freeze DWA in 1m gaps (depth 5 froze the robot: 87% of cycles with no
    // feasible trajectory in the gauntlet slalom).
    let mut obstacle_memory = ObstacleMemory::new(3);

    // Tracked obstacles from the last received WorldState, for coasting
    // through a missed CH1 cycle — the world must never blink empty.
    let mut last_tracked: Vec<Obstacle> = Vec::new();
    let mut last_tracked_at = Instant::now();

    // Last-known inputs with receipt times: planning holds the last received
    // state through missed cycles instead of resetting to the origin, and the
    // receipt age drives the Layer-1 software E-stop (fault tolerance).
    let mut last_world: Option<limo_proto::WorldState> = None;
    let mut last_world_at: Option<Instant> = None;
    let mut last_vehicle: Option<limo_proto::VehicleState> = None;
    let mut last_vehicle_at: Option<Instant> = None;
    let mut stale_estop_active = false;
    let mut vehicle_stale_warned = false;

    // Previous cycle's local-plan quality, feeding the behavior planner's
    // stuck detector (the local planner runs after behavior each cycle).
    let mut prev_plan_infeasible = false;
    let mut prev_plan_feasible = false;
    // Collision radius for the reverse swept check: the reverse is a 0.1 m/s
    // crawl, so the crawl-scaled DWA margin is the consistent requirement
    // (REAR_CLEAR_MARGIN is added on top inside the sweep). With defaults:
    // 0.19 + 0.05·0.4 + 0.05 = 0.26 total — slow motion is allowed the same
    // tighter clearance in reverse as forward.
    let reverse_check_radius = speed_scaled_radius(
        RECOVERY_REVERSE_SPEED,
        config.local_planner.dwa.robot_radius,
        config.local_planner.dwa.margin_low_speed_scale,
        config.local_planner.dwa.high_speed_margin_gain,
    );
    // One-shot latch for the reverse-refused WARN (avoid 10Hz log spam).
    let mut rear_block_warned = false;
    let world_stale_after = Duration::from_millis(config.fault_tolerance.world_state_stale_ms);
    let vehicle_stale_after = Duration::from_millis(config.fault_tolerance.vehicle_state_stale_ms);

    info!("Planning process running — entering main loop");

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // --- 1. Receive WorldState from SensPerc (CH1), newest wins ---
        let world_is_fresh = match ch1_sub.try_recv_latest() {
            Some(ws) => {
                last_world = Some(ws);
                last_world_at = Some(Instant::now());
                true
            }
            None => false,
        };

        // --- 2. Receive VehicleState from Control (CH3), newest wins ---
        if let Some(vs) = ch3_sub.try_recv_latest() {
            last_vehicle = Some(vs);
            last_vehicle_at = Some(Instant::now());
        }

        // --- 2a. Layer-1 software E-stop on stale perception ---
        let world_age = last_world_at.map(|t| t.elapsed());
        let stale_estop = input_stale(world_age, world_stale_after);
        if stale_estop != stale_estop_active {
            if stale_estop {
                warn!(
                    "Software E-stop engaged: WorldState (CH1) {} (threshold {:?})",
                    world_age
                        .map(|a| format!("stale by {:?}", a))
                        .unwrap_or_else(|| "never received".into()),
                    world_stale_after,
                );
            } else {
                info!("Software E-stop released: WorldState (CH1) fresh again");
            }
            stale_estop_active = stale_estop;
        }

        // Vehicle-state staleness only degrades pose fallback quality; per the
        // degradation matrix the response is to alert the operator.
        let vehicle_stale = input_stale(last_vehicle_at.map(|t| t.elapsed()), vehicle_stale_after);
        if vehicle_stale != vehicle_stale_warned {
            if vehicle_stale {
                warn!(
                    "VehicleState (CH3) stale (threshold {:?}) — odometry fallback degraded",
                    vehicle_stale_after,
                );
            } else {
                info!("VehicleState (CH3) fresh again");
            }
            vehicle_stale_warned = vehicle_stale;
        }

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
                // Per-scenario limit: unset (<= 0) means "no scenario cap" —
                // reset any limit left over from a previous scenario.
                scenario_speed_limit = if cmd.global_speed_limit > 0.0 {
                    cmd.global_speed_limit as f64
                } else {
                    f64::INFINITY
                };

                if scenario_active {
                    let wp = &scenario_waypoints[0];
                    if let Some(pose) = &wp.goal_pose {
                        behavior.set_goal(behavior::Goal {
                            x: pose.x,
                            y: pose.y,
                            theta: pose.theta,
                            tolerance: waypoint_tolerance(wp),
                            speed: waypoint_speed(wp),
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
        // Always from the cached last-known inputs: a missed cycle must hold
        // the last pose, never silently reset the robot to the origin.
        let robot_state = build_robot_state(&last_world, &last_vehicle);
        let (static_obstacles, mut tracked_obstacles) = if world_is_fresh {
            extract_obstacles(&last_world)
        } else {
            // No new frame: don't re-ingest the cached one — persistence
            // would smear stale samples and tracked objects must coast below.
            (Vec::new(), Vec::new())
        };
        if world_is_fresh {
            last_tracked = tracked_obstacles.clone();
            last_tracked_at = Instant::now();
        } else if last_tracked_at.elapsed().as_secs_f64() < 0.5 {
            // CH1 miss this cycle: coast the last tracked set along its
            // velocity estimates instead of planning in an empty world.
            let dt = last_tracked_at.elapsed().as_secs_f64();
            tracked_obstacles = last_tracked
                .iter()
                .map(|o| Obstacle {
                    x: o.x + o.vx * dt,
                    y: o.y + o.vy * dt,
                    ..o.clone()
                })
                .collect();
        }
        // Only untracked point samples go through persistence: tracked
        // objects are continuous by construction (the tracker coasts through
        // misses), and smearing a moving object across frames would leave a
        // phantom trail behind it. The union additionally evicts stale
        // footprints of moving tracked objects (ghost fence, see
        // ObstacleMemory docs) — the mover's own returns land in the point
        // set too, and persisting them freezes everywhere it has been.
        obstacle_memory.push(static_obstacles);
        let mut obstacles =
            obstacle_memory.union_excluding_ghosts(&tracked_obstacles, Instant::now());
        obstacles.extend(tracked_obstacles);

        // Populate occupancy grid from detected obstacles
        // Clear grid each cycle and re-populate (rolling local map)
        grid.data.fill(0);
        for obs in &obstacles {
            // Mark obstacle cells (inflate by the DWA base collision radius,
            // plus the object's own extent for tracked clusters). The old
            // hardcoded 0.3 over-inflated: with 0.15m cone clusters it left
            // 0.3m of A*-free width in the slalom's 1.2m weave gaps — every
            // route became a marginal slit and the clearance field had no
            // room to center paths. Matching DWA's base radius keeps the two
            // planners' collision models consistent (soft clearance sits on
            // top for centering).
            let inflate = config.local_planner.dwa.robot_radius + obs.radius; // meters
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

        // Also populate from the last-known WorldState local_map if available
        if let Some(ws) = &last_world {
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

        // Start-pocket escape zone: when the robot's own cell lies inside the
        // hard inflation just painted above (wedged into an obstacle's
        // inflated zone) but its TRUE footprint overlaps no physical
        // obstacle, the global planner and the per-cycle path re-verification
        // relax collision checking to the true footprint within
        // `start_escape_radius` of the robot — so a planned escape maneuver
        // can exist instead of the search dying on an occupied start cell.
        // None whenever the robot is genuinely free (or physically
        // overlapping): everything then behaves exactly as before.
        let physical: Vec<PhysicalObstacle> = obstacles
            .iter()
            .map(|o| PhysicalObstacle {
                x: o.x,
                y: o.y,
                radius: o.radius,
            })
            .collect();
        let escape_zone = start_escape_zone(
            &grid,
            &physical,
            robot_state.x,
            robot_state.y,
            config.global_planner.start_escape_radius,
        );

        // --- 5. Behavior planner ---
        // Steered reverse escape: pick the arc that rotates the nose away
        // from the nearest frontal obstacle and swept-check that actual arc.
        // Ok(cmd) doubles as the scripted phase-B command; Err names the
        // blocking obstacle for the operator log. The swept distance is the
        // behavior planner's CURRENT escalating round target (previous
        // cycle's episode state — attempt counts change on round boundaries,
        // so one cycle of lag is immaterial).
        let reverse_escape = plan_reverse_escape(
            &robot_state,
            &obstacles,
            reverse_check_radius,
            REAR_CLEAR_MARGIN,
            config.local_planner.dwa.moving_obstacle_margin_gain,
            behavior.reverse_target_m(),
            RECOVERY_REVERSE_SPEED,
            // Half the executable curvature: full-κ reverse arcs rack up
            // ~57° of heading change per 0.5m burst — repeated bursts left
            // the robot facing backwards, and an Ackermann platform pays a
            // slow multi-point turn to recover heading. Escape needs lateral
            // offset, not rotation; κ/2 halves the heading debt per burst.
            config.local_planner.dwa.max_curvature * 0.5,
        );

        let behavior_input = BehaviorInput {
            robot_x: robot_state.x,
            robot_y: robot_state.y,
            robot_theta: robot_state.theta,
            localization_confidence: last_world
                .as_ref()
                .map(|ws| ws.localization_confidence)
                .unwrap_or(0.0),
            nearest_obstacle_distance: obstacles
                .iter()
                .map(|o| {
                    let d = ((o.x - robot_state.x).powi(2) + (o.y - robot_state.y).powi(2)).sqrt();
                    (d - o.radius).max(0.0)
                })
                .fold(f64::INFINITY, f64::min),
            emergency_stop: stale_estop,
            dt,
            robot_speed: robot_state.linear_vel,
            planner_infeasible: prev_plan_infeasible,
            planner_feasible: prev_plan_feasible,
            rear_clear: reverse_escape.is_ok(),
        };

        let behavior_out = behavior.update(&behavior_input);

        // WARN naming the blocking obstacle whenever reverse is refused during
        // Recovery (latched: once per continuous blocked stretch, not 10Hz).
        if behavior_out.state == behavior::DrivingState::Recovery {
            match &reverse_escape {
                Err(blocker) => {
                    if !rear_block_warned {
                        warn!(
                            "Recovery: reverse refused — blocking obstacle at ({:.2}, {:.2}), \
                             swept surface distance {:.3}m < required {:.3}m",
                            blocker.x,
                            blocker.y,
                            blocker.surface_dist,
                            reverse_check_radius + REAR_CLEAR_MARGIN,
                        );
                        rear_block_warned = true;
                    }
                }
                Ok(_) => rear_block_warned = false,
            }
        } else {
            rear_block_warned = false;
        }

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
                        tolerance: waypoint_tolerance(wp),
                        speed: waypoint_speed(wp),
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

        // --- 5b. Roadmap route maintenance ---
        // Recompute the graph route when the mission waypoint changes, the
        // robot deviates from the route polyline, or (via 6a clearing
        // active_route) the current link got blocked. With roadmap disabled
        // this whole layer is inert: active_route stays None and step 6
        // targets the mission waypoint directly, exactly as before.
        if let Some(rm) = roadmap.as_ref() {
            let mission_xy = scenario_waypoints
                .get(current_wp_index)
                .filter(|_| scenario_active)
                .and_then(|wp| wp.goal_pose.as_ref())
                .map(|p| (p.x, p.y));
            match mission_xy {
                None => active_route = None,
                Some(goal_xy) => {
                    let state = active_route.as_ref().map(|ar| {
                        let proj =
                            roadmap::project_onto_route(&ar.route, robot_state.x, robot_state.y);
                        (ar.mission_wp, proj.distance)
                    });
                    if let Some(reason) =
                        route_recompute_reason(state, current_wp_index, config.roadmap.deviation_m)
                    {
                        match rm.route((robot_state.x, robot_state.y), goal_xy, Instant::now()) {
                            Some(route) => {
                                if last_route_log.elapsed() >= Duration::from_secs(1) {
                                    info!(
                                        "Route ({}): {} — {:.1}m, est {:.1}s, min width {:.1}m",
                                        reason,
                                        route.describe(),
                                        route.length,
                                        route.travel_time,
                                        route.min_width(),
                                    );
                                    last_route_log = Instant::now();
                                }
                                active_route = Some(ActiveRoute {
                                    route,
                                    mission_wp: current_wp_index,
                                    goal_s: None,
                                    goal_link: 0,
                                    best_goal_dist: f64::INFINITY,
                                    progress_at: Instant::now(),
                                });
                            }
                            None => {
                                if last_route_log.elapsed() >= Duration::from_secs(1) {
                                    warn!(
                                        "Roadmap: no route to mission waypoint ({:.1},{:.1}); \
                                         falling back to direct goal",
                                        goal_xy.0, goal_xy.1,
                                    );
                                    last_route_log = Instant::now();
                                }
                                active_route = None;
                            }
                        }
                    }
                }
            }
        }

        // --- 5c. Route leg goal + link speed cap ---
        // Hysteretic carrot on the route polyline: the leg goal (fed to
        // Hybrid A* in step 6) hops in discrete ~leg_max_m steps so path
        // hysteresis stays effective. The speed cap is the link under the
        // robot's route projection; the leg-progress watchdog feeds 6a.
        let mut route_speed_cap = f64::INFINITY;
        let mut route_goal: Option<roadmap::RouteGoal> = None;
        // Robot's arc position on the active route (valid only while
        // route_goal is Some) — the lower end of the reference corridor.
        let mut route_robot_s = 0.0;
        if let Some(ar) = active_route.as_mut() {
            let proj = roadmap::project_onto_route(&ar.route, robot_state.x, robot_state.y);
            route_speed_cap = ar.route.waypoints[proj.segment].speed;
            route_robot_s = proj.s;
            let goal_s = roadmap::advance_goal_arc(
                &ar.route,
                proj.s,
                ar.goal_s,
                config.roadmap.leg_min_m,
                config.roadmap.leg_max_m,
            );
            if let Some(g) = roadmap::goal_at_arc(&ar.route, goal_s) {
                if ar.goal_s.is_none_or(|prev| (prev - g.s).abs() > 1e-6) {
                    // New leg goal: reset the leg-progress watchdog.
                    ar.goal_s = Some(g.s);
                    ar.best_goal_dist = f64::INFINITY;
                    ar.progress_at = Instant::now();
                }
                ar.goal_link = g.link;
                let d = ((robot_state.x - g.x).powi(2) + (robot_state.y - g.y).powi(2)).sqrt();
                if d + LEG_PROGRESS_EPS_M < ar.best_goal_dist {
                    ar.best_goal_dist = d;
                    ar.progress_at = Instant::now();
                }
                route_goal = Some(g);
            }
        }

        // --- 6. Global planner (4Hz) with smoothing + path hysteresis ---
        // Commitment: the CURRENT path is kept while it stays valid, so
        // partial commitment to a corridor persists through obstacle-estimate
        // jitter instead of flapping between topologies every replan.
        //
        // Per-cycle (10Hz): truncate the retained path to the waypoint
        // nearest the robot and re-verify the remainder against the LATEST
        // grid at hard inflation (≤5cm sub-steps).
        if !global_path.is_empty() {
            // Truncation is scoped to the ACTIVE direction run: a maneuver
            // path's later segments can pass back through space near the
            // robot (reverse 0.5m, then forward through where it started),
            // and a global nearest-waypoint search would amputate the
            // not-yet-executed reverse leg on cycle one.
            let near = active_run_truncation_index(&global_path, robot_state.x, robot_state.y);
            if near > 0 {
                global_path.drain(..near);
            }
        }
        let current_path_valid =
            path_remains_valid_with_escape(&global_path, &grid, escape_zone.as_ref());

        let mut astar_failed_on_route = false;
        if behavior_out.replan_requested && last_global_plan.elapsed() >= global_plan_interval {
            let start = Pose {
                x: robot_state.x,
                y: robot_state.y,
                theta: robot_state.theta,
            };

            // Mission goal = current scenario waypoint. A waypoint without a
            // goal_pose (malformed CH8 message) must not panic the planner;
            // hold position and warn instead.
            let mission = match scenario_waypoints
                .get(current_wp_index)
                .filter(|_| scenario_active)
            {
                Some(wp) => match wp.goal_pose.as_ref() {
                    Some(pose) => Some(Pose {
                        x: pose.x,
                        y: pose.y,
                        theta: pose.theta,
                    }),
                    None => {
                        warn!(
                            "Waypoint {}/{} '{}' has no goal_pose; holding position",
                            current_wp_index + 1,
                            scenario_waypoints.len(),
                            wp.label,
                        );
                        None
                    }
                },
                None => None,
            };
            // With an active roadmap route, A* targets the route's next leg
            // goal instead of the (possibly far) mission waypoint; the
            // route's final leg defers back to the exact mission pose.
            let goal = select_plan_goal(mission, route_goal.as_ref()).unwrap_or(Pose {
                x: robot_state.x,
                y: robot_state.y,
                theta: robot_state.theta,
            });

            // Reference corridor for the leg (active route only): the route
            // polyline from the robot's projection to the leg goal (small
            // margin both ends) becomes a soft-pull + hard-bound tube for
            // Hybrid A* and the smoother — metric planning FOLLOWS the
            // route, deviating only locally for obstacles. No route: None,
            // and the search is byte-for-byte the unconstrained one.
            let corridor = match (&active_route, &route_goal) {
                (Some(ar), Some(g)) => Corridor::new(
                    roadmap::sub_polyline(
                        &ar.route,
                        route_robot_s - CORRIDOR_END_MARGIN_M,
                        g.s + CORRIDOR_END_MARGIN_M,
                    ),
                    config.roadmap.corridor_half_width,
                    config.roadmap.corridor_cost_weight,
                    config.roadmap.corridor_hard_factor,
                )
                .map(|mut c| {
                    // Outside-start re-entry (see Corridor::allow_reentry_from):
                    // a rerouted reference may start far from the robot; the
                    // plan back INTO the tube must be possible.
                    c.allow_reentry_from(robot_state.x, robot_state.y);
                    c
                }),
                _ => None,
            };
            // The DWA fallback samples against the same tube (see
            // DwaPlanner::set_corridor) so a pursuit deferral doesn't send
            // the sampler chasing the carrot outside the route.
            last_corridor = corridor.clone();

            let goal_changed = current_path_goal.is_none_or(|(gx, gy, gt)| {
                goal_differs((gx, gy, gt), (goal.x, goal.y, goal.theta))
            });
            let end_reaches_goal = global_path.last().is_some_and(|end| {
                ((end.x - goal.x).powi(2) + (end.y - goal.y).powi(2)).sqrt() <= goal_end_tolerance
            });
            // Pose consistency (replay t=65.8s): hysteresis judged retained
            // paths by collision validity, goal identity, and cost — never
            // by whether the path still relates to where the robot IS. A
            // retained path a full meter lateral of the robot made every
            // executor arc back onto a stale reference (kinked, pose-blind
            // trajectories). Beyond PATH_OFFSET_INVALIDATE_M the retention
            // privilege is void — the fresh candidate, planned FROM the
            // current pose, replaces it unconditionally.
            let robot_near_path = global_path.iter().any(|w| {
                (w.x - robot_state.x).powi(2) + (w.y - robot_state.y).powi(2)
                    <= PATH_OFFSET_INVALIDATE_M * PATH_OFFSET_INVALIDATE_M
            });
            let current_ok =
                current_path_valid && !goal_changed && end_reaches_goal && robot_near_path;

            // Reference-direct following (roadmap.follow_route_directly):
            // the route polyline IS the global path — resampled fresh every
            // replan tick from the robot's projection to the leg goal. No
            // A*, no smoother, no hysteresis: the reference is stable by
            // construction, always starts at the robot's projection, and
            // obstacle avoidance lives in the local planner's quintic
            // lateral offsets. The whole class of "A* found no path" link
            // blame disappears with it.
            let route_direct = config.roadmap.follow_route_directly
                && active_route.is_some()
                && route_goal.is_some();
            if route_direct {
                if let (Some(ar), Some(g)) = (&active_route, &route_goal) {
                    let path = resample_polyline(
                        &roadmap::sub_polyline(&ar.route, route_robot_s, g.s),
                        ROUTE_PATH_SPACING_M,
                    );
                    if path.len() >= 2 {
                        global_path = path;
                        current_path_goal = Some((goal.x, goal.y, goal.theta));
                    } else {
                        // Degenerate leg (robot at the leg goal): the
                        // endgame direct approach owns the remainder.
                        global_path.clear();
                    }
                }
                astar_fail_streak = 0;
                last_global_plan = Instant::now();
            } else {
                // One chamfer build per replan, shared by A*, the smoother, and
                // both sides of the hysteresis cost comparison.
                let clearance = global.build_clearance(&grid);
                if let Some(raw) = global
                    .plan_with_corridor(
                        &start,
                        &goal,
                        &grid,
                        clearance.as_ref(),
                        escape_zone.as_ref(),
                        corridor.as_ref(),
                    )
                    .0
                {
                    let smooth_start = Instant::now();
                    let candidate = smoother::smooth_path(
                        &raw,
                        &grid,
                        clearance.as_ref(),
                        &config.global_planner,
                        config.local_planner.dwa.max_curvature,
                        escape_zone.as_ref(),
                        corridor.as_ref(),
                    );
                    debug!(
                        "smoother: {} raw -> {} waypoints in {:?}",
                        raw.len(),
                        candidate.len(),
                        smooth_start.elapsed(),
                    );
                    // Route-following observability: corridor mode + how far the
                    // produced plan strays from the reference at worst.
                    if let Some(c) = corridor.as_ref() {
                        let max_off = candidate
                            .iter()
                            .map(|w| c.offset(w.x, w.y))
                            .fold(0.0_f64, f64::max);
                        debug!(
                            "corridor active: leg max lateral offset {:.2}m (hard bound {:.2}m)",
                            max_off,
                            c.hard_bound(),
                        );
                    }

                    let weight = config.global_planner.clearance_cost_weight;
                    let decay = config.global_planner.clearance_decay_m;
                    // Both sides scored with the same direction penalties, so a
                    // shuttling maneuver path is judged by the full cost A*
                    // charged for it and a clean pure-forward candidate can
                    // displace it on the same 15% margin as any other.
                    let penalties = CostPenalties::from_config(&config.global_planner);
                    let decision = path_replace_decision(
                        current_ok,
                        goal_changed,
                        path_cost(&candidate, clearance.as_ref(), weight, decay, penalties),
                        path_cost(&global_path, clearance.as_ref(), weight, decay, penalties),
                        config.global_planner.path_improvement_threshold,
                    );

                    if let Some(reason) = decision {
                        if last_replace_log.elapsed() >= Duration::from_secs(1) {
                            info!(
                                "Global path replaced ({}): {} waypoints",
                                reason,
                                candidate.len()
                            );
                            last_replace_log = Instant::now();
                        } else {
                            debug!(
                                "Global path replaced ({}): {} waypoints",
                                reason,
                                candidate.len()
                            );
                        }
                        global_path = candidate;
                        current_path_goal = Some((goal.x, goal.y, goal.theta));
                    } else {
                        debug!(
                            "Global path retained: candidate not {}% better",
                            config.global_planner.path_improvement_threshold * 100.0
                        );
                    }
                } else {
                    // No new path. Keep whatever we have: even an invalidated
                    // path is safer to keep pointing along than an empty one (the
                    // executors collision-check every command at 10Hz; an empty
                    // path is a confident stop that would mask the blockage).
                    // Inside the corridor tube this is a much stronger signal the
                    // leg is genuinely obstructed — the blocked-link machinery
                    // (6a) escalates to a reroute; the tube is never widened.
                    debug!(
                        "Global planner: no path found{}",
                        if corridor.is_some() {
                            " (corridor-constrained: leg likely obstructed)"
                        } else {
                            ""
                        },
                    );
                    astar_failed_on_route = route_goal.is_some();
                }
                // Blame persistence: a SINGLE failed replan blamed the leg's
                // link instantly (replay t≈13s and t≈35s: transient failures
                // during the entry maneuver flipped the route to the slalom,
                // then flipped it back mid-corridor — wholesale plan reversals
                // from momentary planner hiccups). Only a STREAK of failures is
                // evidence of genuine obstruction.
                if astar_failed_on_route {
                    astar_fail_streak += 1;
                } else {
                    astar_fail_streak = 0;
                }
                last_global_plan = Instant::now();
            }
        }

        // --- 6a. Blocked-link detection on the active route ---
        // A leg is declared blocked when Hybrid A* found no path to its goal
        // or the robot made no progress toward it for link_block_after_s.
        // The link is temporarily excluded from routing and the route is
        // recomputed next cycle (5b sees active_route == None).
        //
        // PAUSED during Recovery: a wedged robot cannot make leg progress,
        // and blaming whatever link the leg goal happens to sit on produced
        // live route flapping — mid-recovery the watchdog declared the (un-
        // reached) next link blocked, swapped the wide lane route for the
        // 0.6m slalom, wedged there, blamed that, and oscillated. Recovery
        // time is not link time: the clock is refreshed while recovering so
        // it restarts from zero on exit, and transient escape-mode A*
        // failures are not treated as leg obstruction either.
        let in_recovery = behavior_out.state == behavior::DrivingState::Recovery;
        if in_recovery {
            if let Some(ar) = active_route.as_mut() {
                ar.progress_at = Instant::now();
            }
        }
        let block = (!in_recovery)
            .then(|| {
                active_route.as_ref().and_then(|ar| {
                    leg_block_cause(
                        astar_fail_streak >= ASTAR_BLAME_MIN_FAILS,
                        ar.progress_at.elapsed(),
                        link_block_after,
                    )
                    .map(|c| (ar.goal_link, c))
                })
            })
            .flatten();
        if let Some((link, cause)) = block {
            if let Some(rm) = roadmap.as_mut() {
                if last_block_warn.elapsed() >= Duration::from_secs(1) {
                    if block_warns_suppressed > 0 {
                        warn!(
                            "Link {} blocked ({}), rerouting ({} repeats suppressed)",
                            rm.link_name(link),
                            cause,
                            block_warns_suppressed
                        );
                    } else {
                        warn!("Link {} blocked ({}), rerouting", rm.link_name(link), cause);
                    }
                    last_block_warn = Instant::now();
                    block_warns_suppressed = 0;
                } else {
                    block_warns_suppressed += 1;
                }
                rm.report_blocked(link, Instant::now());
            }
            active_route = None;
        }

        // --- 7. Local planner (10Hz) ---
        // Command-source precedence (see `select_local_plan`): pursuit on
        // the CURRENT global path is primary even inside Recovery, so a
        // planned escape (reverse legs included) is EXECUTED rather than
        // shuffled over by scripted reverse/relaxed DWA. Scripted commands
        // are the last resort.
        // Per-leg desired speed: min(scenario leg speed via behavior, global
        // scenario limit, the active roadmap link's cruise cap).
        let desired_speed = behavior_out
            .desired_speed
            .min(scenario_speed_limit)
            .min(route_speed_cap);
        // DWA fallback follows the same route tube as the global planner;
        // cleared the moment no route is active (mission tail, blocked-link
        // reroute gap) so the sampler falls back to unconstrained.
        local.set_corridor(if active_route.is_some() {
            last_corridor.clone()
        } else {
            None
        });
        // Endgame direct approach (see `direct_approach_path`): an empty
        // path near the goal becomes a one-waypoint DWA path instead of the
        // deliberate stop that parked the robot 0.81m short of tolerance.
        let direct_goal = if global_path.is_empty() && behavior_out.recovery_phase.is_none() {
            direct_approach_path(
                scenario_active,
                &scenario_waypoints,
                current_wp_index,
                &robot_state,
                GOAL_DIRECT_APPROACH_M,
            )
            .map(|wp| vec![wp])
        } else {
            None
        };
        let exec_path: &[PathWaypoint] = direct_goal.as_deref().unwrap_or(&global_path);
        // Actuation-delay compensation: plan from the pose the robot will
        // occupy when the command reaches the chassis (see
        // `local_planner::project_state`), not the perception-aged one.
        let exec_state =
            local_planner::project_state(&robot_state, config.local_planner.actuation_delay_s);
        let (local_plan, scripted_fallback, pursuit_attempt) = select_local_plan(
            &mut local,
            behavior_out.recovery_phase,
            &exec_state,
            exec_path,
            &obstacles,
            desired_speed,
            &reverse_escape,
        );
        // Stale-heading invalidation: path hysteresis keeps a collision-valid
        // path even when a recovery rotation left its forward direction BEHIND
        // the robot — heading was never a validity criterion, so the stale
        // path survived every replan tick and pursuit deferred (TargetBehind)
        // forever. TARGET_BEHIND_DROP_CYCLES consecutive deferrals (counted
        // across ALL phases where pursuit is attempted; scripted cycles
        // neither count nor reset) drop the path; the next 4Hz replan plans
        // from the actual pose/heading (bidirectional A* + RS handles the
        // turn properly). 5 cycles = 0.5s: a genuinely stale heading defers
        // every cycle indefinitely so the drop still fires near-instantly,
        // while the 1-2 cycle TargetBehind TRANSIENTS of a hard weave no
        // longer kill a live path mid-drive (at 2 cycles the drops fired at
        // 1.2 m/s and forced escape-mode replans that amplified the weave).
        // Route-direct mode has NO retention to bust: the path regenerates
        // identically from the robot's projection every replan tick, so a
        // streak drop only starves the executors of a path for 250ms and
        // then meets the same geometry again (live: drop-regenerate loop at
        // 10Hz while recovery had nothing to work with). Reorientation onto
        // the reference is the executors' job there, not the invalidators'.
        let route_direct_active = config.roadmap.follow_route_directly && active_route.is_some();
        match pursuit_attempt {
            PursuitAttempt::Deferred(local_planner::PursuitDefer::TargetBehind) => {
                blocked_streak = 0;
                target_behind_streak += 1;
                if target_behind_streak >= TARGET_BEHIND_DROP_CYCLES
                    && !route_direct_active
                    && !global_path.is_empty()
                {
                    warn!(
                        "pursuit target behind for {} cycles — dropping stale path \
                         ({} waypoints) to force a heading-consistent replan",
                        target_behind_streak,
                        global_path.len()
                    );
                    global_path.clear();
                    target_behind_streak = 0;
                }
            }
            // Clearance-stale path: hysteresis can retain a path that hugs a
            // wall tip so closely every pursuit arc sweeps within millimeters
            // of it (live: net 0.193 vs req 0.195 at the channel entry, robot
            // a full meter away — no wedge allowance applies). A sustained
            // Blocked streak means the RETAINED path, not the world, is the
            // problem: drop it so the next replan routes through today's
            // clearance field from the actual pose.
            PursuitAttempt::Deferred(local_planner::PursuitDefer::Blocked(_)) => {
                target_behind_streak = 0;
                blocked_streak += 1;
                // Short paths are exempt: dropping a sub-2m final approach
                // cannot yield a materially different replan (the goal pinch
                // IS the blockage) — live it churned drop/replan every 0.8s
                // and reset pursuit each time. Recovery and the endgame
                // direct approach own short-path blockages.
                if blocked_streak >= BLOCKED_DROP_CYCLES
                    && !route_direct_active
                    && global_path.len() >= BLOCKED_DROP_MIN_WAYPOINTS
                {
                    warn!(
                        "pursuit blocked for {} cycles — dropping wall-hugging path \
                         ({} waypoints) to force a clearance-fresh replan",
                        blocked_streak,
                        global_path.len()
                    );
                    global_path.clear();
                    blocked_streak = 0;
                }
            }
            PursuitAttempt::Deferred(_) | PursuitAttempt::Succeeded => {
                target_behind_streak = 0;
                blocked_streak = 0;
            }
            PursuitAttempt::NotTried => {}
        }
        if let Some(reason) = scripted_fallback {
            // Hierarchy fallback: scripted reverse engaged even though a
            // global path exists. Rate-limited, tagged for post-run counting.
            if last_scripted_fallback_log.elapsed() >= Duration::from_secs(1) {
                info!(
                    "scripted reverse engaged (pursuit=None reason={}) with {} path waypoints \
                     present",
                    reason,
                    global_path.len(),
                );
                last_scripted_fallback_log = Instant::now();
            }
        }
        let trad_cmd = local_plan.command.clone();

        // Plan-quality flags for next cycle's stuck detector (see
        // `classify_plan`): planned maneuver execution (a pursuit reverse
        // segment or its cusp stop) is exempt from the "stuck" reading, while
        // the scripted RECOVERY reverse still never counts as feasible.
        (prev_plan_infeasible, prev_plan_feasible) = classify_plan(
            &trad_cmd,
            local_plan.planned_maneuver,
            config.arbitrator.fallback_min_confidence,
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
                // With an active roadmap route the waypoint list carries the
                // ROUTE polyline (labels = node ids, current index = the
                // vertex the leg goal heads toward) so the visualizer shows
                // the route; wire format unchanged. Without a route: the
                // mission waypoints, exactly as before.
                scenario_waypoints: match &active_route {
                    Some(ar) => ar
                        .route
                        .waypoints
                        .iter()
                        .map(|wp| limo_proto::Pose2D {
                            x: wp.x,
                            y: wp.y,
                            theta: 0.0,
                        })
                        .collect(),
                    None => scenario_waypoints
                        .iter()
                        .filter_map(|wp| wp.goal_pose)
                        .collect(),
                },
                waypoint_labels: match &active_route {
                    Some(ar) => ar
                        .route
                        .waypoints
                        .iter()
                        .map(|wp| wp.node_id.clone())
                        .collect(),
                    None => scenario_waypoints
                        .iter()
                        .map(|wp| wp.label.clone())
                        .collect(),
                },
                current_waypoint_index: match &active_route {
                    Some(ar) => {
                        roadmap::vertex_at_or_after(&ar.route, ar.goal_s.unwrap_or(0.0)) as u32
                    }
                    None => current_wp_index as u32,
                },
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
                "Planning cycle {}: state={:?}{} speed={:.2} conf={:.2} degraded={} path={} exec={} ch2_sent={} clipped={}",
                cycle,
                behavior_out.state,
                behavior_out
                    .recovery_phase
                    .map(|p| format!("({:?})", p))
                    .unwrap_or_default(),
                arb_out.command.linear_x,
                arb_out.command.confidence,
                arb.is_degraded(),
                global_path.len(),
                local_plan.executor,
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

/// Layer-1 software E-stop / staleness decision: an input is stale when it
/// has never arrived or its age exceeds the threshold. Applied to WorldState
/// (CH1) it means planning is blind and must command an emergency stop.
fn input_stale(age: Option<Duration>, threshold: Duration) -> bool {
    age.is_none_or(|a| a > threshold)
}

/// Stuck-detector classification of the cycle's local plan. Stuck means a
/// (near-)zero forward command while driving toward an active goal —
/// confidence is deliberately not consulted for infeasibility: a
/// confidently-stationary plan (e.g. an optimizer converging on "hold
/// still") is still zero progress. Exceptions, both flagged by the local
/// planner as `planned_maneuver`:
/// - a pursuit REVERSE-segment command (negative linear_x) is planned
///   progress → feasible, not infeasible;
/// - the scripted stop AT a direction cusp (near-zero linear_x) is planned
///   execution → not infeasible (and not "feasible forward" either).
///
/// The recovery machinery's own scripted reverse is NOT flagged and keeps
/// its old classification (never feasible — Recovery must not self-exit
/// while boxed in).
fn classify_plan(
    cmd: &VelocityCommand,
    planned_maneuver: bool,
    min_confidence: f32,
) -> (bool, bool) {
    let infeasible = cmd.linear_x.abs() < PLAN_ZERO_SPEED && !planned_maneuver;
    let moving_forward = cmd.linear_x > PLAN_ZERO_SPEED;
    let planned_reverse = planned_maneuver && cmd.linear_x < -PLAN_ZERO_SPEED;
    let feasible = (moving_forward || planned_reverse) && cmd.confidence >= min_confidence;
    (infeasible, feasible)
}

/// Command-source precedence for one 10Hz cycle.
///
/// Hierarchy inversion (live gauntlet histogram: pursuit drove ~1 sampled
/// cycle while Recovery scripted/relaxed drove 43/47 — planned escapes were
/// produced and then ignored): in Recovery(ForwardRetry) and
/// Recovery(Reverse), PURSUIT ON THE CURRENT GLOBAL PATH is attempted before
/// any relaxed/scripted fallback. From a wedge that path is an escape plan
/// with reverse legs, which pursuit executes as verified planned maneuvers
/// (wedged allowance included); the resulting command reads as
/// feasible/maneuver in `classify_plan`, so Recovery exits organically.
///
/// Fallback order preserved beneath pursuit:
/// - ForwardRetry: relaxed-margin DWA crawl (whose infeasible verdict still
///   drives the phase machine toward Reverse exactly as before);
/// - Reverse: the swept-checked scripted reverse burst — the true last
///   resort. When it engages even though a global path exists, the pursuit
///   deferral reason is returned so the caller can emit the rate-limited
///   hierarchy-fallback log;
/// - Hold: the confident zero command, unchanged (behavior commands zero
///   speed here; the periodic retry rounds surface as ForwardRetry, where
///   pursuit is primary).
///
/// The behavior state machine (states, stuck detection, episode accounting,
/// E-stop semantics) is untouched — only the command source changed.
fn select_local_plan(
    local: &mut LocalPlanner,
    recovery_phase: Option<RecoveryPhase>,
    robot_state: &RobotState,
    global_path: &[PathWaypoint],
    obstacles: &[Obstacle],
    desired_speed: f64,
    reverse_escape: &Result<VelocityCommand, RearBlocker>,
) -> (LocalPlan, Option<PursuitDefer>, PursuitAttempt) {
    match recovery_phase {
        Some(RecoveryPhase::ForwardRetry) => {
            match local.compute_pursuit(robot_state, global_path, obstacles, desired_speed) {
                Ok(plan) => (plan, None, PursuitAttempt::Succeeded),
                Err(reason) => (
                    local.compute_relaxed(robot_state, global_path, obstacles, desired_speed),
                    None,
                    PursuitAttempt::Deferred(reason),
                ),
            }
        }
        Some(RecoveryPhase::Reverse) => {
            match local.compute_pursuit(robot_state, global_path, obstacles, desired_speed) {
                Ok(plan) => (plan, None, PursuitAttempt::Succeeded),
                Err(reason) => {
                    // Behavior only reports Reverse when rear_clear was true
                    // this cycle, so the escape is Ok; fall back to a
                    // straight reverse defensively if that invariant breaks.
                    let cmd = reverse_escape.clone().unwrap_or(VelocityCommand {
                        linear_x: -RECOVERY_REVERSE_SPEED,
                        angular_z: 0.0,
                        confidence: 0.9,
                    });
                    let fallback = (!global_path.is_empty()).then_some(reason);
                    (
                        local.scripted_plan(robot_state, cmd),
                        fallback,
                        PursuitAttempt::Deferred(reason),
                    )
                }
            }
        }
        Some(RecoveryPhase::Hold) => (
            local.scripted_plan(
                robot_state,
                VelocityCommand {
                    linear_x: 0.0,
                    angular_z: 0.0,
                    confidence: 1.0,
                },
            ),
            None,
            PursuitAttempt::NotTried,
        ),
        None => {
            let (plan, defer) =
                local.compute_with_defer(robot_state, global_path, obstacles, desired_speed);
            let attempt = match defer {
                Some(reason) => PursuitAttempt::Deferred(reason),
                None if global_path.is_empty() => PursuitAttempt::NotTried,
                None => PursuitAttempt::Succeeded,
            };
            (plan, None, attempt)
        }
    }
}

/// Endgame direct-approach path (see `GOAL_DIRECT_APPROACH_M`): a
/// single-waypoint path at the active mission goal, produced only when the
/// global path is empty, no recovery is in progress, and the goal is within
/// `max_dist` of the robot. None everywhere else — the empty-path confident
/// stop remains the correct behavior for Idle and for far-away A* failures
/// (chasing a distant goal unchecked is what the corridor work eliminated).
fn direct_approach_path(
    scenario_active: bool,
    waypoints: &[limo_proto::NavigationGoal],
    current_wp: usize,
    robot: &RobotState,
    max_dist: f64,
) -> Option<PathWaypoint> {
    let wp = waypoints.get(current_wp).filter(|_| scenario_active)?;
    let pose = wp.goal_pose.as_ref()?;
    let dist = ((pose.x - robot.x).powi(2) + (pose.y - robot.y).powi(2)).sqrt();
    if dist > max_dist {
        return None;
    }
    // Tolerance-edge targeting: the mission only requires ENTERING the
    // tolerance disk, but aiming at the center forced a millimeter-scale
    // squeeze past the goal's nearest obstacle (live: goal ringed by plaza
    // clutter — recovery shuffled forward/backward against a pinch the
    // tolerance never asked it to cross). Aim at the nearest point
    // comfortably inside the disk instead.
    let pull = ((wp.goal_tolerance as f64) - TOLERANCE_EDGE_MARGIN_M)
        .max(0.0)
        .min(dist);
    let (tx, ty) = if dist > 1e-6 {
        (
            pose.x + (robot.x - pose.x) / dist * pull,
            pose.y + (robot.y - pose.y) / dist * pull,
        )
    } else {
        (pose.x, pose.y)
    };
    Some(PathWaypoint {
        x: tx,
        y: ty,
        theta: pose.theta,
        steering: 0.0,
        dir: Default::default(),
    })
}

/// Resample a route polyline into forward PathWaypoints at ~ROUTE_PATH_
/// SPACING_M spacing, theta = segment tangent (last point inherits it).
/// The reference-direct global path (roadmap.follow_route_directly).
fn resample_polyline(poly: &[(f64, f64)], spacing: f64) -> Vec<PathWaypoint> {
    if poly.len() < 2 {
        return Vec::new();
    }
    let mut out: Vec<PathWaypoint> = Vec::new();
    let mut carry = 0.0;
    for w in poly.windows(2) {
        let (ax, ay) = w[0];
        let (bx, by) = w[1];
        let (dx, dy) = (bx - ax, by - ay);
        let len = (dx * dx + dy * dy).sqrt();
        if len < 1e-9 {
            continue;
        }
        let theta = dy.atan2(dx);
        if out.is_empty() {
            out.push(PathWaypoint {
                x: ax,
                y: ay,
                theta,
                steering: 0.0,
                dir: Default::default(),
            });
        }
        let mut s = spacing - carry;
        while s < len {
            out.push(PathWaypoint {
                x: ax + dx / len * s,
                y: ay + dy / len * s,
                theta,
                steering: 0.0,
                dir: Default::default(),
            });
            s += spacing;
        }
        carry = len - (s - spacing);
    }
    let (lx, ly) = *poly.last().unwrap();
    let end_far = out
        .last()
        .is_none_or(|e| ((e.x - lx).powi(2) + (e.y - ly).powi(2)).sqrt() > spacing * 0.25);
    if end_far {
        let theta = out.last().map_or(0.0, |e| e.theta);
        out.push(PathWaypoint {
            x: lx,
            y: ly,
            theta,
            steering: 0.0,
            dir: Default::default(),
        });
    }
    out
}

/// Waypoint spacing (m) of the reference-direct global path.
const ROUTE_PATH_SPACING_M: f64 = 0.25;

/// How far INSIDE the goal tolerance disk the direct approach aims (m):
/// the scenario checks distance-to-center < tolerance, so stopping this
/// deep past the disk edge registers arrival with margin for pose noise.
const TOLERANCE_EDGE_MARGIN_M: f64 = 0.10;

/// Outcome of the pursuit attempt inside `select_local_plan`, for the
/// stale-heading streak: scripted cycles (Hold, empty path) carry no
/// evidence about the path's heading and must NOT reset the TargetBehind
/// count — the live failure was exactly the interleaved Hold/scripted
/// cycles zeroing the streak so "2 consecutive deferrals" never fired and
/// a heading-stale path survived recovery indefinitely.
#[derive(Debug, Clone, Copy, PartialEq)]
enum PursuitAttempt {
    NotTried,
    Succeeded,
    Deferred(PursuitDefer),
}

/// Roadmap route currently followed toward a mission waypoint, with the
/// hysteretic leg-goal position and the leg-progress watchdog state used
/// for blocked-link detection.
struct ActiveRoute {
    route: roadmap::Route,
    /// Scenario waypoint index this route serves.
    mission_wp: usize,
    /// Arc position of the current leg goal on the route (None until the
    /// first 5c pass after a (re)route).
    goal_s: Option<f64>,
    /// Link the current leg goal lies on — the link reported blocked when
    /// the leg stalls.
    goal_link: usize,
    /// Best (smallest) distance to the leg goal achieved so far.
    best_goal_dist: f64,
    /// Last time best_goal_dist improved (leg-progress watchdog).
    progress_at: Instant,
}

/// Why the current route leg's link is declared blocked this cycle, if at
/// all. `astar_failed` is "Hybrid A* found no path to the leg goal this
/// replan" — with the corridor constraint active that search ran inside the
/// route tube, so failure is a strong obstruction signal; the caller reports
/// the link blocked and reroutes (the tube is never widened).
fn leg_block_cause(
    astar_failed: bool,
    progress_age: Duration,
    block_after: Duration,
) -> Option<&'static str> {
    if astar_failed {
        Some("global planner found no path")
    } else if progress_age >= block_after {
        Some("no leg progress")
    } else {
        None
    }
}

/// Why the roadmap route must be recomputed this cycle, if at all.
/// `active` = (mission waypoint the route serves, current lateral deviation
/// from the route polyline) for the retained route, None when there is no
/// route (fresh goal, or 6a cleared it after blocking a link).
fn route_recompute_reason(
    active: Option<(usize, f64)>,
    current_wp: usize,
    deviation_limit: f64,
) -> Option<&'static str> {
    match active {
        None => Some("new goal"),
        Some((wp, _)) if wp != current_wp => Some("mission waypoint changed"),
        Some((_, dev)) if dev > deviation_limit => Some("deviated from route"),
        _ => None,
    }
}

/// The goal fed to Hybrid A*: the roadmap route's next leg goal when a
/// route is active (its FINAL leg defers to the exact mission pose so the
/// behavior planner's goal-reached check converges on the same point),
/// otherwise the mission waypoint itself. With `roadmap.enabled: false`
/// there is never a route goal, so this is exactly the old direct
/// mission-waypoint flow. None = no active mission → hold position.
fn select_plan_goal(
    mission: Option<Pose>,
    route_goal: Option<&roadmap::RouteGoal>,
) -> Option<Pose> {
    let mission = mission?;
    match route_goal {
        Some(g) if !g.is_final => Some(Pose {
            x: g.x,
            y: g.y,
            theta: g.heading,
        }),
        _ => Some(mission),
    }
}

/// Why the retained global path was replaced (hysteresis logging).
#[derive(Debug, Clone, Copy, PartialEq)]
enum PathReplaceReason {
    /// The retained path hit newly-perceived hard inflation (or there was no
    /// path / it no longer ends at the goal).
    Invalid,
    /// The navigation goal changed (waypoint advance, new scenario).
    GoalChanged,
    /// The candidate beats the retained path's remaining cost by at least the
    /// improvement threshold (fraction, e.g. 0.23 = 23% better).
    Better(f64),
}

impl std::fmt::Display for PathReplaceReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PathReplaceReason::Invalid => f.write_str("invalid"),
            PathReplaceReason::GoalChanged => f.write_str("goal"),
            PathReplaceReason::Better(frac) => write!(f, "better by {:.0}%", frac * 100.0),
        }
    }
}

/// Goal-change epsilon (m / rad): scenario waypoints are exact repeats
/// between cycles, so this only needs to absorb float noise.
const GOAL_EPS: f64 = 1e-6;

fn goal_differs(a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
    (a.0 - b.0).abs() > GOAL_EPS || (a.1 - b.1).abs() > GOAL_EPS || (a.2 - b.2).abs() > GOAL_EPS
}

/// Hysteresis core: whether the retained global path is replaced by the new
/// candidate, and why. An invalid current path (or a changed goal) is always
/// replaced; a VALID one only when the candidate is at least `threshold`
/// (fraction) cheaper than the current path's remaining cost. Jittered-but-
/// equivalent replans are rejected — this is what kills 4 Hz corridor
/// flapping on a noisy obstacle snapshot.
fn path_replace_decision(
    current_ok: bool,
    goal_changed: bool,
    cost_new: f64,
    cost_current: f64,
    threshold: f64,
) -> Option<PathReplaceReason> {
    if !current_ok {
        return Some(if goal_changed {
            PathReplaceReason::GoalChanged
        } else {
            PathReplaceReason::Invalid
        });
    }
    if !(cost_new.is_finite() && cost_current.is_finite()) || cost_current <= 0.0 {
        return None;
    }
    let improvement = 1.0 - cost_new / cost_current;
    (improvement >= threshold).then_some(PathReplaceReason::Better(improvement))
}

/// Build the robot state from the cached last-known inputs. Callers pass the
/// held (last received) messages, so a missed cycle keeps the last pose; the
/// origin default is only reachable before ANY input has ever arrived, and
/// the staleness E-stop keeps the robot stopped in that window.
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

/// Per-waypoint arrival tolerance from a scenario goal; None when the proto
/// field is unset (0.0) so the behavior planner falls back to its config.
fn waypoint_tolerance(wp: &limo_proto::NavigationGoal) -> Option<f64> {
    (wp.goal_tolerance > 0.0).then_some(wp.goal_tolerance as f64)
}

/// Per-waypoint desired speed from a scenario goal; None when the proto field
/// is unset (0.0) so the leg runs at the behavior planner's default_speed.
/// The behavior planner treats a Some value as a CAP on the leg's speed.
fn waypoint_speed(wp: &limo_proto::NavigationGoal) -> Option<f64> {
    (wp.desired_speed > 0.0).then_some(wp.desired_speed as f64)
}

/// Tracked obstacles slower than this (m/s) never evict persisted points:
/// a static or slow track's surroundings are real obstacles, not stale
/// footprints of a mover.
const GHOST_MIN_TRACK_SPEED: f64 = 0.15;
/// Association slack (m) beyond the tracked extent radius when matching a
/// persisted point to a mover's back-extrapolated position — absorbs tracker
/// centroid noise and the CH1-receipt-vs-scan timestamp skew.
const GHOST_EVICTION_EPS_M: f64 = 0.15;

/// A persisted detection frame with its ingestion time (the Instant the CH1
/// WorldState carrying it was received; CH1 latency is small relative to the
/// eviction slack, so receipt time stands in for capture time).
struct TimedFrame {
    at: Instant,
    obstacles: Vec<Obstacle>,
}

/// Rolling memory of recent obstacle detections. Planning runs against the
/// union of the last `depth` frames (~0.3 s at 10 Hz): an obstacle that drops
/// out of a single cycle still blocks trajectories, at the cost of ~depth×
/// points and slightly wider phantom footprints around moving obstacles.
/// Empty frames still advance the window, so stale points decay in `depth`
/// cycles if perception stops reporting them.
///
/// Ghost eviction: a moving object's lidar returns enter the per-frame point
/// sets, and the union would freeze every place the object has recently been
/// — a fence of stale points across free space (live gauntlet: a wandering
/// pedestrian's trail stalled the robot 0.6m from the goal with no physical
/// obstacle within 0.4m). `union_excluding_ghosts` drops any persisted point
/// that lies where a currently-tracked mover WAS when that point's frame was
/// captured (back-extrapolated along the track velocity); the mover itself is
/// already represented by the tracker's velocity-propagated cluster. Points
/// near static/slow tracks are kept — they are real obstacles.
struct ObstacleMemory {
    depth: usize,
    frames: std::collections::VecDeque<TimedFrame>,
}

impl ObstacleMemory {
    fn new(depth: usize) -> Self {
        Self {
            depth,
            frames: std::collections::VecDeque::with_capacity(depth),
        }
    }

    fn push(&mut self, frame: Vec<Obstacle>) {
        self.push_at(frame, Instant::now());
    }

    fn push_at(&mut self, frame: Vec<Obstacle>, at: Instant) {
        if self.frames.len() == self.depth {
            self.frames.pop_front();
        }
        self.frames.push_back(TimedFrame {
            at,
            obstacles: frame,
        });
    }

    /// Union of all persisted frames, minus stale footprints of moving
    /// tracked objects. For each frame (captured at t_k) and each tracked
    /// obstacle faster than `GHOST_MIN_TRACK_SPEED`, the track center is
    /// back-extrapolated to that frame's time (c_k = pos − v·(now − t_k));
    /// points within `radius + GHOST_EVICTION_EPS_M` of c_k are dropped.
    ///
    /// Chosen semantics for the overlap case: a genuinely static point
    /// captured while a mover passes over it IS evicted for those frames —
    /// acceptable because the tracked cluster covers that exact space while
    /// overlapping it, and once the mover walks on, fresh frames re-detect
    /// the point and it re-enters the union within one cycle.
    fn union_excluding_ghosts(&self, tracked: &[Obstacle], now: Instant) -> Vec<Obstacle> {
        let mut out = Vec::new();
        for frame in &self.frames {
            let dt = now.saturating_duration_since(frame.at).as_secs_f64();
            for p in &frame.obstacles {
                let is_ghost = tracked.iter().any(|trk| {
                    let speed = (trk.vx * trk.vx + trk.vy * trk.vy).sqrt();
                    if speed < GHOST_MIN_TRACK_SPEED {
                        return false;
                    }
                    let cx = trk.x - trk.vx * dt;
                    let cy = trk.y - trk.vy * dt;
                    let d = ((p.x - cx).powi(2) + (p.y - cy).powi(2)).sqrt();
                    d <= trk.radius + GHOST_EVICTION_EPS_M
                });
                if !is_ghost {
                    out.push(p.clone());
                }
            }
        }
        out
    }
}

/// Split detections into (untracked point samples, tracked objects).
/// Tracked objects carry extent and velocity from sensperc's cluster tracker.
fn extract_obstacles(world: &Option<limo_proto::WorldState>) -> (Vec<Obstacle>, Vec<Obstacle>) {
    let mut points = Vec::new();
    let mut tracked = Vec::new();
    if let Some(ws) = world {
        if let Some(dets) = &ws.detections {
            for det in &dets.detections {
                if let Some(pos) = &det.position_world {
                    let (vx, vy) = det
                        .velocity_world
                        .as_ref()
                        .map(|v| (v.linear_x, v.linear_y))
                        .unwrap_or((0.0, 0.0));
                    let obs = Obstacle {
                        x: pos.x,
                        y: pos.y,
                        vx,
                        vy,
                        radius: det.radius as f64,
                        // Oriented-rectangle extent when the tracker sent
                        // one (both half extents > 0); the circular radius
                        // above stays the conservative fallback.
                        half_x: det.half_extent_x as f64,
                        half_y: det.half_extent_y as f64,
                        heading: det.orientation as f64,
                    };
                    if det.track_id != 0 {
                        tracked.push(obs);
                    } else {
                        points.push(obs);
                    }
                }
            }
        }
    }
    (points, tracked)
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

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Obstacle {
        Obstacle::point(x, y)
    }

    #[test]
    fn obstacle_memory_survives_a_dropped_cycle() {
        // A cone seen at t0 but missing from t1's sample must still be
        // present in the planning set at t1.
        let mut mem = ObstacleMemory::new(5);
        mem.push(vec![pt(1.0, 2.0)]);
        mem.push(vec![]); // sampling dropped it this cycle
        let union = mem.union_excluding_ghosts(&[], Instant::now());
        assert!(union.iter().any(|o| o.x == 1.0 && o.y == 2.0));
    }

    #[test]
    fn obstacle_memory_decays_after_depth_cycles() {
        let mut mem = ObstacleMemory::new(3);
        mem.push(vec![pt(1.0, 2.0)]);
        for _ in 0..3 {
            mem.push(vec![]);
        }
        assert!(mem.union_excluding_ghosts(&[], Instant::now()).is_empty());
    }

    #[test]
    fn obstacle_memory_unions_all_frames() {
        let mut mem = ObstacleMemory::new(3);
        mem.push(vec![pt(1.0, 0.0)]);
        mem.push(vec![pt(2.0, 0.0)]);
        assert_eq!(mem.union_excluding_ghosts(&[], Instant::now()).len(), 2);
    }

    // ---- Ghost eviction (persistence trail of moving tracked objects) ----

    /// Tracked mover helper: pedestrian-like cluster.
    fn mover(x: f64, y: f64, vx: f64, vy: f64) -> Obstacle {
        Obstacle {
            x,
            y,
            vx,
            vy,
            radius: 0.15,
            ..Default::default()
        }
    }

    #[test]
    fn ghost_trail_of_moving_track_is_evicted() {
        // Pedestrian tracked at (2.0, 0.0) walking +x at 0.5 m/s. Frames from
        // 0.2s and 0.1s ago hold its stale returns at exactly its former
        // positions (x = 1.90, 1.95) — the picket-fence points that stalled
        // the live run. Back-extrapolating the track to each frame's time
        // must land on them and evict both.
        let t0 = Instant::now();
        let mut mem = ObstacleMemory::new(3);
        mem.push_at(vec![pt(1.90, 0.0)], t0);
        mem.push_at(vec![pt(1.95, 0.0)], t0 + Duration::from_millis(100));
        let tracked = vec![mover(2.0, 0.0, 0.5, 0.0)];
        let now = t0 + Duration::from_millis(200);
        let union = mem.union_excluding_ghosts(&tracked, now);
        assert!(
            union.is_empty(),
            "stale footprints of the mover must be evicted, kept: {:?}",
            union
        );
    }

    #[test]
    fn points_near_static_track_are_kept() {
        // Identical geometry, but the track is stationary (below the
        // GHOST_MIN_TRACK_SPEED gate): nearby persisted points are REAL
        // obstacles (e.g. clutter around a parked object) and must be kept.
        let t0 = Instant::now();
        let mut mem = ObstacleMemory::new(3);
        mem.push_at(vec![pt(1.90, 0.0)], t0);
        mem.push_at(vec![pt(1.95, 0.0)], t0 + Duration::from_millis(100));
        let parked = vec![mover(2.0, 0.0, 0.0, 0.0)];
        let now = t0 + Duration::from_millis(200);
        assert_eq!(mem.union_excluding_ghosts(&parked, now).len(), 2);

        // Slow drift under the speed gate behaves the same.
        let creeping = vec![mover(2.0, 0.0, 0.1, 0.0)];
        assert_eq!(mem.union_excluding_ghosts(&creeping, now).len(), 2);
    }

    #[test]
    fn static_cone_under_mover_is_covered_then_reappears() {
        // Documented overlap semantics: a genuinely static cone point in a
        // fresh frame at the mover's CURRENT position IS evicted (dt ≈ 0, so
        // the back-extrapolated center is the mover itself) — acceptable
        // because the tracked extent covers that exact space this cycle...
        let t0 = Instant::now();
        let mut mem = ObstacleMemory::new(3);
        let cone = pt(2.0, 0.02);
        mem.push_at(vec![cone.clone()], t0);
        let ped = mover(2.0, 0.0, 0.5, 0.0);
        assert!(mem
            .union_excluding_ghosts(std::slice::from_ref(&ped), t0)
            .is_empty());
        let d = ((cone.x - ped.x).powi(2) + (cone.y - ped.y).powi(2)).sqrt();
        assert!(
            d <= ped.radius,
            "eviction under the mover is only safe because the tracked extent covers the spot"
        );

        // ...and once the mover has walked past, the same cone in a fresh
        // frame is no longer near any back-extrapolated position: KEPT. The
        // old overlap frame is (correctly) still evicted as trail.
        let t1 = t0 + Duration::from_secs(2);
        mem.push_at(vec![cone.clone()], t1);
        let ped_far = mover(3.0, 0.0, 0.5, 0.0); // walked 1m on
        let union = mem.union_excluding_ghosts(&[ped_far], t1);
        assert_eq!(union.len(), 1, "re-detected static cone must survive");
        assert!((union[0].x - 2.0).abs() < 1e-9);
    }

    // ---- Layer-1 software E-stop (input staleness) ----

    #[test]
    fn input_stale_when_never_received() {
        // Before the first WorldState arrives, planning is blind: E-stop.
        assert!(input_stale(None, Duration::from_millis(300)));
    }

    #[test]
    fn input_fresh_within_threshold() {
        assert!(!input_stale(
            Some(Duration::from_millis(100)),
            Duration::from_millis(300)
        ));
        // Exactly at the threshold is still fresh.
        assert!(!input_stale(
            Some(Duration::from_millis(300)),
            Duration::from_millis(300)
        ));
    }

    #[test]
    fn input_stale_beyond_threshold() {
        assert!(input_stale(
            Some(Duration::from_millis(301)),
            Duration::from_millis(300)
        ));
    }

    // ---- build_robot_state hold-last-known behavior ----

    fn world_with_pose(x: f64, y: f64, theta: f64) -> limo_proto::WorldState {
        limo_proto::WorldState {
            robot_pose: Some(limo_proto::Pose2D { x, y, theta }),
            robot_velocity: Some(limo_proto::Twist2D {
                linear_x: 0.4,
                linear_y: 0.0,
                angular_z: 0.1,
            }),
            ..Default::default()
        }
    }

    fn vehicle_with_pose(x: f64, y: f64, theta: f64) -> limo_proto::VehicleState {
        limo_proto::VehicleState {
            odometry_pose: Some(limo_proto::Pose2D { x, y, theta }),
            odometry_velocity: Some(limo_proto::Twist2D {
                linear_x: 0.2,
                linear_y: 0.0,
                angular_z: 0.0,
            }),
            ..Default::default()
        }
    }

    #[test]
    fn build_robot_state_prefers_world_pose() {
        let state = build_robot_state(
            &Some(world_with_pose(3.0, 4.0, 0.5)),
            &Some(vehicle_with_pose(9.0, 9.0, 0.0)),
        );
        assert_eq!((state.x, state.y, state.theta), (3.0, 4.0, 0.5));
        assert_eq!(state.linear_vel, 0.4);
    }

    #[test]
    fn build_robot_state_holds_cached_world_on_missed_cycle() {
        // The main loop passes the CACHED last-known WorldState on a missed
        // CH1 cycle; the pose must be that held pose, not the origin.
        let cached = Some(world_with_pose(2.0, -1.0, 1.2));
        let state = build_robot_state(&cached, &None);
        assert_eq!((state.x, state.y, state.theta), (2.0, -1.0, 1.2));
    }

    #[test]
    fn build_robot_state_falls_back_to_vehicle_odometry() {
        let state = build_robot_state(&None, &Some(vehicle_with_pose(1.0, 2.0, 0.3)));
        assert_eq!((state.x, state.y, state.theta), (1.0, 2.0, 0.3));
        assert_eq!(state.linear_vel, 0.2);
    }

    #[test]
    fn waypoint_speed_maps_unset_to_none() {
        // Proto 0.0 (unset) and negative garbage → None (leg uses the
        // configured default_speed); positive values pass through as the
        // per-leg cap. The old wiring dropped the field entirely.
        let mut wp = limo_proto::NavigationGoal::default();
        assert_eq!(waypoint_speed(&wp), None);
        wp.desired_speed = -1.0;
        assert_eq!(waypoint_speed(&wp), None);
        wp.desired_speed = 1.5;
        assert_eq!(waypoint_speed(&wp), Some(1.5));
    }

    // ---- Path hysteresis / commitment ----

    fn straight_path(n: usize, spacing: f64) -> Vec<PathWaypoint> {
        (0..n)
            .map(|i| PathWaypoint {
                x: i as f64 * spacing,
                y: 0.0,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect()
    }

    #[test]
    fn invalid_current_path_triggers_replacement() {
        // A newly-perceived obstacle landing ON the retained path invalidates
        // it, and the decision must be replacement with reason "invalid" no
        // matter how the candidate's cost compares.
        let mut grid = OccupancyGrid::new(400, 400, 0.1, -20.0, -20.0);
        let path = straight_path(21, 0.15); // 3m straight line
        assert!(global_planner::path_remains_valid(&path, &grid));
        grid.set_occupied(1.57, 0.0); // mid-segment, between waypoints
        assert!(!global_planner::path_remains_valid(&path, &grid));

        let decision = path_replace_decision(false, false, 10.0, 3.0, 0.15);
        assert_eq!(decision, Some(PathReplaceReason::Invalid));
    }

    #[test]
    fn jittered_but_valid_path_is_retained() {
        // Obstacle-estimate jitter shifts the candidate's cost a few percent:
        // a valid current path on the same goal must be KEPT (no topology
        // flapping between 4 Hz replans).
        let grid = OccupancyGrid::new(400, 400, 0.1, -20.0, -20.0);
        let path = straight_path(21, 0.15);
        assert!(global_planner::path_remains_valid(&path, &grid));
        // 5% better than current: below the 15% threshold.
        assert_eq!(path_replace_decision(true, false, 2.85, 3.0, 0.15), None);
        // Slightly WORSE candidate (jitter the other way): also kept.
        assert_eq!(path_replace_decision(true, false, 3.1, 3.0, 0.15), None);
    }

    #[test]
    fn improvement_threshold_respected() {
        // Exactly at the threshold and beyond: replaced with the measured
        // improvement fraction; just below: retained.
        match path_replace_decision(true, false, 2.4, 3.0, 0.15) {
            Some(PathReplaceReason::Better(f)) => assert!((f - 0.2).abs() < 1e-9),
            other => panic!("20% better candidate must replace, got {:?}", other),
        }
        match path_replace_decision(true, false, 2.55, 3.0, 0.15) {
            Some(PathReplaceReason::Better(f)) => assert!((f - 0.15).abs() < 1e-9),
            other => panic!("threshold-exact candidate must replace, got {:?}", other),
        }
        assert_eq!(path_replace_decision(true, false, 2.6, 3.0, 0.15), None);
        // Degenerate current costs never force a swap via the cost branch.
        assert_eq!(path_replace_decision(true, false, 1.0, 0.0, 0.15), None);
        assert_eq!(
            path_replace_decision(true, false, f64::NAN, 3.0, 0.15),
            None
        );
    }

    #[test]
    fn hysteresis_cost_includes_direction_penalties() {
        // (f) The retained path shuttles (reverse leg + two cusps); a new
        // pure-forward candidate of similar geometric length must replace it
        // once the PENALIZED costs differ by >= 15% — and would NOT replace
        // it under the old length-only cost. Both sides scored by the same
        // `path_cost` the main loop uses.
        use global_planner::SegmentDir;
        let wp = |x: f64, y: f64, dir: SegmentDir| PathWaypoint {
            x,
            y,
            theta: 0.0,
            steering: 0.0,
            dir,
        };
        // Shuttle: forward 1.5m, reverse 0.5m, forward 1.5m = 3.5m length.
        let shuttle = vec![
            wp(0.0, 0.0, SegmentDir::Forward),
            wp(1.5, 0.0, SegmentDir::Forward),
            wp(1.0, 0.0, SegmentDir::Reverse),
            wp(2.5, 0.0, SegmentDir::Forward),
        ];
        // Pure forward detour: 3.4m, no cusps.
        let forward = vec![
            wp(0.0, 0.0, SegmentDir::Forward),
            wp(1.7, 0.0, SegmentDir::Forward),
            wp(3.4, 0.0, SegmentDir::Forward),
        ];
        let pen = CostPenalties {
            reverse_cost_multiplier: 2.0,
            direction_switch_penalty: 0.6,
        };
        let cost_shuttle = path_cost(&shuttle, None, 0.0, 0.0, pen);
        let cost_forward = path_cost(&forward, None, 0.0, 0.0, pen);
        // Shuttle: 1.5 + 0.5*2 + 1.5 + 2*0.6 = 5.2; forward: 3.4 -> 35% better.
        assert!((cost_shuttle - 5.2).abs() < 1e-9);
        assert!((cost_forward - 3.4).abs() < 1e-9);
        match path_replace_decision(true, false, cost_forward, cost_shuttle, 0.15) {
            Some(PathReplaceReason::Better(f)) => assert!(f > 0.15),
            other => panic!("pure-forward candidate must replace the shuttle, got {other:?}"),
        }

        // Control: under the un-penalized (old) cost the shuttle looks
        // SHORTER (3.5 vs 3.4 is only ~3% apart) and would be retained —
        // pinning that the penalties, not the geometry, drive the swap.
        let no_pen = CostPenalties::none();
        let old_shuttle = path_cost(&shuttle, None, 0.0, 0.0, no_pen);
        let old_forward = path_cost(&forward, None, 0.0, 0.0, no_pen);
        assert_eq!(
            path_replace_decision(true, false, old_forward, old_shuttle, 0.15),
            None,
            "without penalties the shuttle would have been kept"
        );
    }

    #[test]
    fn classify_plan_exempts_planned_maneuvers() {
        let cmd = |v: f64, conf: f32| VelocityCommand {
            linear_x: v,
            angular_z: 0.0,
            confidence: conf,
        };
        // Feasible forward plan: unchanged semantics.
        assert_eq!(classify_plan(&cmd(0.5, 0.9), false, 0.3), (false, true));
        // Genuine zero command: infeasible (stuck detector counts it).
        assert_eq!(classify_plan(&cmd(0.0, 0.1), false, 0.3), (true, false));
        // Planned pursuit REVERSE: progress — feasible, never infeasible.
        assert_eq!(classify_plan(&cmd(-0.3, 0.8), true, 0.3), (false, true));
        // Cusp stop (planned, near-zero): not infeasible, not feasible.
        assert_eq!(classify_plan(&cmd(0.0, 0.9), true, 0.3), (false, false));
        // Recovery scripted reverse (NOT flagged): old semantics — neither
        // infeasible nor feasible, Recovery cannot self-exit on it.
        assert_eq!(classify_plan(&cmd(-0.1, 0.9), false, 0.3), (false, false));
        // Low-confidence forward motion is not feasible.
        assert_eq!(classify_plan(&cmd(0.5, 0.2), false, 0.3), (false, false));
    }

    #[test]
    fn goal_change_forces_replacement() {
        assert_eq!(
            path_replace_decision(false, true, 3.0, 3.0, 0.15),
            Some(PathReplaceReason::GoalChanged)
        );
        assert!(goal_differs((1.0, 2.0, 0.0), (1.0, 2.5, 0.0)));
        assert!(!goal_differs((1.0, 2.0, 0.5), (1.0, 2.0, 0.5)));
    }

    #[test]
    fn replace_reason_log_formatting() {
        assert_eq!(PathReplaceReason::Invalid.to_string(), "invalid");
        assert_eq!(PathReplaceReason::GoalChanged.to_string(), "goal");
        assert_eq!(PathReplaceReason::Better(0.23).to_string(), "better by 23%");
    }

    #[test]
    fn build_robot_state_default_only_when_nothing_ever_received() {
        let state = build_robot_state(&None, &None);
        assert_eq!((state.x, state.y, state.theta), (0.0, 0.0, 0.0));
        // In this window input_stale(None, ..) forces the software E-stop,
        // so the origin pose is never acted on.
        assert!(input_stale(None, Duration::from_millis(300)));
    }

    // ---- Roadmap route-goal selection (main-loop glue) ----

    fn mission_pose() -> Pose {
        Pose {
            x: 17.9,
            y: 0.5,
            theta: 0.0,
        }
    }

    fn route_goal(x: f64, y: f64, heading: f64, is_final: bool) -> roadmap::RouteGoal {
        roadmap::RouteGoal {
            s: 0.0,
            x,
            y,
            heading,
            link: 0,
            is_final,
        }
    }

    #[test]
    fn select_plan_goal_without_route_is_the_old_direct_flow() {
        // roadmap.enabled: false → route_goal is always None → the goal fed
        // to Hybrid A* is exactly the mission waypoint, as today.
        let goal = select_plan_goal(Some(mission_pose()), None).unwrap();
        assert_eq!((goal.x, goal.y, goal.theta), (17.9, 0.5, 0.0));
        // No mission → no goal (caller holds position), route or not.
        assert!(select_plan_goal(None, None).is_none());
        assert!(select_plan_goal(None, Some(&route_goal(1.0, 2.0, 0.3, false))).is_none());
    }

    #[test]
    fn select_plan_goal_targets_route_leg_and_defers_final_to_mission() {
        // Mid-route: the leg goal (with the route tangent heading) replaces
        // the direct mission goal.
        let g = route_goal(3.0, 2.7, 0.1, false);
        let goal = select_plan_goal(Some(mission_pose()), Some(&g)).unwrap();
        assert_eq!((goal.x, goal.y, goal.theta), (3.0, 2.7, 0.1));
        // Final leg: the exact mission pose wins (so behavior's
        // goal-reached check and A* converge on the same point).
        let last = route_goal(17.88, 0.49, 1.0, true);
        let goal = select_plan_goal(Some(mission_pose()), Some(&last)).unwrap();
        assert_eq!((goal.x, goal.y, goal.theta), (17.9, 0.5, 0.0));
    }

    #[test]
    fn leg_block_cause_feeds_reroute_machinery() {
        // (c) glue: a corridor-constrained A* failure on the active route
        // (astar_failed_on_route) must declare the leg's link blocked THIS
        // cycle — the existing report_blocked + reroute path then fires
        // exactly as for the progress watchdog.
        let after = Duration::from_secs(10);
        assert_eq!(
            leg_block_cause(true, Duration::ZERO, after),
            Some("global planner found no path")
        );
        assert_eq!(
            leg_block_cause(false, Duration::from_secs(11), after),
            Some("no leg progress")
        );
        assert_eq!(leg_block_cause(false, Duration::from_secs(9), after), None);
    }

    #[test]
    fn route_recompute_triggers() {
        // No route yet → compute.
        assert_eq!(route_recompute_reason(None, 2, 1.5), Some("new goal"));
        // Route serves an older mission waypoint → recompute.
        assert_eq!(
            route_recompute_reason(Some((1, 0.2)), 2, 1.5),
            Some("mission waypoint changed")
        );
        // Robot pushed off the route (avoidance detour) beyond the limit →
        // deviation-triggered reroute.
        assert_eq!(
            route_recompute_reason(Some((2, 1.6)), 2, 1.5),
            Some("deviated from route")
        );
        // On-route, same mission waypoint → keep the route.
        assert_eq!(route_recompute_reason(Some((2, 1.4)), 2, 1.5), None);
    }

    // ---- Recovery command-source hierarchy (pursuit primary) ----

    use behavior::{BehaviorConfig, DrivingState, Goal};
    use local_planner::{Executor, LocalPlannerConfig};

    fn stationary() -> RobotState {
        RobotState::default()
    }

    fn scripted_escape_cmd() -> Result<VelocityCommand, RearBlocker> {
        Ok(VelocityCommand {
            linear_x: -RECOVERY_REVERSE_SPEED,
            angular_z: 0.0,
            confidence: 0.9,
        })
    }

    #[test]
    fn recovery_forward_retry_prefers_pursuit_on_open_path() {
        // Hierarchy inversion core: in Recovery(ForwardRetry) — which is also
        // how Hold's periodic retry rounds surface — an executable global
        // path must be driven by the PURSUIT executor, not the relaxed DWA
        // crawl, and the plan must classify as feasible so Recovery can exit
        // organically on the feasible-cycle streak.
        let mut local = LocalPlanner::new(LocalPlannerConfig::default());
        let path = straight_path(31, 0.1); // 3m open straight
        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::ForwardRetry),
            &stationary(),
            &path,
            &[],
            0.1,
            &scripted_escape_cmd(),
        );
        assert_eq!(plan.executor, Executor::Pursuit);
        assert!(plan.command.linear_x > 0.0);
        assert!(fallback.is_none());
        let (infeasible, feasible) = classify_plan(&plan.command, plan.planned_maneuver, 0.3);
        assert!(
            !infeasible && feasible,
            "pursuit command must feed the sticky recovery exit"
        );
    }

    #[test]
    fn recovery_pursuit_none_falls_back_relaxed_then_scripted() {
        // (b) Old precedence preserved BENEATH pursuit. With no path (no
        // goal) pursuit defers: ForwardRetry must attempt the relaxed DWA
        // (whose empty-path answer is the deliberate confident stop), the
        // Reverse phase must produce the scripted burst, Hold the confident
        // zero — and none of these count as a hierarchy fallback (there was
        // no plan being ignored).
        let mut local = LocalPlanner::new(LocalPlannerConfig::default());

        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::ForwardRetry),
            &stationary(),
            &[],
            &[],
            0.1,
            &scripted_escape_cmd(),
        );
        assert_eq!(
            plan.executor,
            Executor::Dwa,
            "relaxed DWA must be attempted"
        );
        assert_eq!(plan.command.linear_x, 0.0);
        assert!(
            plan.command.confidence >= 0.9,
            "empty-path stop is deliberate"
        );
        assert!(fallback.is_none());

        // Path-exhausted stub (remaining chord below pursuit's stable-arc
        // floor): pursuit defers, phase A still belongs to the relaxed DWA.
        let stub = straight_path(2, 0.1);
        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::ForwardRetry),
            &stationary(),
            &stub,
            &[],
            0.1,
            &scripted_escape_cmd(),
        );
        assert_eq!(plan.executor, Executor::Dwa);
        assert!(fallback.is_none());

        // Reverse phase without a path: scripted burst, no fallback reason.
        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::Reverse),
            &stationary(),
            &[],
            &[],
            0.1,
            &scripted_escape_cmd(),
        );
        assert_eq!(plan.executor, Executor::Scripted);
        assert!((plan.command.linear_x - (-RECOVERY_REVERSE_SPEED)).abs() < 1e-9);
        assert!(
            fallback.is_none(),
            "no path present => not a hierarchy fallback"
        );

        // Hold: confident zero command, unchanged.
        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::Hold),
            &stationary(),
            &[],
            &[],
            0.0,
            &scripted_escape_cmd(),
        );
        assert_eq!(plan.executor, Executor::Scripted);
        assert_eq!(plan.command.linear_x, 0.0);
        assert!(plan.command.confidence >= 0.99);
        assert!(fallback.is_none());
    }

    #[test]
    fn pursuit_attempt_reporting_feeds_the_stale_heading_streak() {
        // The stale-heading streak counts Deferred from ANY phase where
        // pursuit ran and holds (not resets) on scripted cycles. Pin the
        // per-branch reporting: ForwardRetry against a wall = Deferred
        // (previously reported as None, which reset the streak every
        // interleaved cycle and kept heading-stale paths alive), Hold =
        // NotTried, open-path phases = Succeeded.
        let mut local = LocalPlanner::new(LocalPlannerConfig::default());
        let path = straight_path(31, 0.1);
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(pt(0.25, y));
            y += 0.05;
        }
        let (_, _, attempt) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::ForwardRetry),
            &stationary(),
            &path,
            &wall,
            0.1,
            &scripted_escape_cmd(),
        );
        assert!(
            matches!(attempt, PursuitAttempt::Deferred(_)),
            "blocked ForwardRetry must report the deferral, got {attempt:?}"
        );

        let (_, _, attempt) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::Hold),
            &stationary(),
            &path,
            &wall,
            0.0,
            &scripted_escape_cmd(),
        );
        assert_eq!(
            attempt,
            PursuitAttempt::NotTried,
            "Hold carries no evidence"
        );

        let (_, _, attempt) = select_local_plan(
            &mut local,
            None,
            &stationary(),
            &path,
            &[],
            0.5,
            &scripted_escape_cmd(),
        );
        assert_eq!(attempt, PursuitAttempt::Succeeded);

        let (_, _, attempt) = select_local_plan(
            &mut local,
            None,
            &stationary(),
            &[],
            &[],
            0.5,
            &scripted_escape_cmd(),
        );
        assert_eq!(attempt, PursuitAttempt::NotTried, "no path, no evidence");
    }

    #[test]
    fn scripted_reverse_fallback_carries_pursuit_reason_when_path_exists() {
        // (c) A global path exists but a wall 0.25m ahead blocks every
        // pursuit speed step (even the accel-clamped crawl rolls inside the
        // crawl clearance requirement): the Reverse phase engages the
        // scripted burst AND surfaces the deferral reason for the
        // rate-limited "scripted reverse engaged (pursuit=None reason=...)"
        // log. Control: with the path removed, the same scripted burst
        // reports no fallback.
        let mut local = LocalPlanner::new(LocalPlannerConfig::default());
        let path = straight_path(31, 0.1);
        let mut wall = Vec::new();
        let mut y = -1.0;
        while y <= 1.0 {
            wall.push(pt(0.25, y));
            y += 0.05;
        }
        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::Reverse),
            &stationary(),
            &path,
            &wall,
            0.1,
            &scripted_escape_cmd(),
        );
        assert_eq!(plan.executor, Executor::Scripted);
        assert!(plan.command.linear_x < 0.0);
        assert!(
            matches!(fallback, Some(PursuitDefer::Blocked(Some(_)))),
            "fallback log must carry the pursuit deferral reason with its \
             binding-obstacle summary, got {fallback:?}"
        );

        // Same scene, no path: scripted engages silently (nothing ignored).
        let (plan, fallback, _) = select_local_plan(
            &mut local,
            Some(RecoveryPhase::Reverse),
            &stationary(),
            &[],
            &wall,
            0.1,
            &scripted_escape_cmd(),
        );
        assert_eq!(plan.executor, Executor::Scripted);
        assert!(fallback.is_none());
    }

    /// (a) Closed-loop wedged start on the compound pocket (tracked cone
    /// dead ahead inside its hard-inflation band, wall point samples
    /// flanking at y = ±0.235, rear physically open — the live-gauntlet
    /// wedge, mirroring `global_planner::escape_tests`): once the
    /// escape-relaxed global plan exists, Recovery's command source must be
    /// the PURSUIT executor on that plan. The robot backs out of the pocket,
    /// Recovery exits organically on feasible-maneuver cycles (before the
    /// 0.3m movement exit could fire), and the scripted reverse never
    /// engages.
    #[test]
    fn recovery_executes_escape_plan_via_pursuit_closed_loop() {
        // Compound-pocket fixture.
        let mut obstacles = vec![Obstacle {
            x: 0.38,
            y: 0.0,
            vx: 0.0,
            vy: 0.0,
            radius: 0.15,
            ..Default::default()
        }];
        let mut x = -0.2;
        while x <= 0.4 + 1e-9 {
            for y in [0.235, -0.235] {
                obstacles.push(pt(x, y));
            }
            x += 0.05;
        }
        let physical: Vec<PhysicalObstacle> = obstacles
            .iter()
            .map(|o| PhysicalObstacle {
                x: o.x,
                y: o.y,
                radius: o.radius,
            })
            .collect();
        // Hard inflation painted as in the live wedge (robot_radius 0.24 +
        // extent, circular blobs).
        let mut grid = OccupancyGrid::new(100, 100, 0.1, -5.0, -5.0);
        for obs in &physical {
            let blob = 0.24 + obs.radius;
            let mut dx = -(blob + 0.05);
            while dx <= blob + 0.05 {
                let mut dy = -(blob + 0.05);
                while dy <= blob + 0.05 {
                    if (dx * dx + dy * dy).sqrt() <= blob {
                        grid.set_occupied(obs.x + dx, obs.y + dy);
                    }
                    dy += 0.05;
                }
                dx += 0.05;
            }
        }

        let lp_cfg = LocalPlannerConfig::default();
        let mut local = LocalPlanner::new(lp_cfg.clone());
        let mut bp = BehaviorPlanner::new(BehaviorConfig::default());
        bp.set_goal(Goal {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
            tolerance: None,
            speed: None,
        });
        let reverse_check_radius = speed_scaled_radius(
            RECOVERY_REVERSE_SPEED,
            lp_cfg.dwa.robot_radius,
            lp_cfg.dwa.margin_low_speed_scale,
            lp_cfg.dwa.high_speed_margin_gain,
        );
        let nearest = |state: &RobotState, obstacles: &[Obstacle]| {
            obstacles
                .iter()
                .map(|o| {
                    let d = ((o.x - state.x).powi(2) + (o.y - state.y).powi(2)).sqrt();
                    (d - o.radius).max(0.0)
                })
                .fold(f64::INFINITY, f64::min)
        };

        let mut state = RobotState::default();
        let dt = 0.1;
        let (mut prev_infeasible, mut prev_feasible) = (false, false);

        // Phase 1 — the live sequence: wedged with NO plan yet. The empty
        // path's zero command reads infeasible; the stuck detector must walk
        // the behavior planner into Recovery.
        let mut entered = false;
        for _ in 0..100 {
            let reverse_escape = plan_reverse_escape(
                &state,
                &obstacles,
                reverse_check_radius,
                REAR_CLEAR_MARGIN,
                lp_cfg.dwa.moving_obstacle_margin_gain,
                bp.reverse_target_m(),
                RECOVERY_REVERSE_SPEED,
                lp_cfg.dwa.max_curvature * 0.5,
            );
            let out = bp.update(&BehaviorInput {
                robot_x: state.x,
                robot_y: state.y,
                robot_theta: state.theta,
                localization_confidence: 1.0,
                nearest_obstacle_distance: nearest(&state, &obstacles),
                emergency_stop: false,
                dt,
                robot_speed: state.linear_vel,
                planner_infeasible: prev_infeasible,
                planner_feasible: prev_feasible,
                rear_clear: reverse_escape.is_ok(),
            });
            let (plan, fallback, _) = select_local_plan(
                &mut local,
                out.recovery_phase,
                &state,
                &[],
                &obstacles,
                out.desired_speed,
                &reverse_escape,
            );
            assert!(fallback.is_none());
            (prev_infeasible, prev_feasible) =
                classify_plan(&plan.command, plan.planned_maneuver, 0.3);
            if out.state == DrivingState::Recovery {
                entered = true;
                break;
            }
        }
        assert!(entered, "stuck detector never entered Recovery");

        // Phase 2 — the escape-relaxed global replan produces the plan
        // main.rs would publish (raw A* + smoother under the escape zone).
        let planner = HybridAStar::new(global_planner::HybridAStarConfig::default());
        let gp_cfg = global_planner::HybridAStarConfig::default();
        let start = Pose {
            x: state.x,
            y: state.y,
            theta: state.theta,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };
        let zone = start_escape_zone(&grid, &physical, state.x, state.y, 0.6)
            .expect("wedged start must activate the escape zone");
        let clearance = planner.build_clearance(&grid);
        let raw = planner
            .plan_with_escape(&start, &goal, &grid, clearance.as_ref(), Some(&zone))
            .0
            .expect("escape plan must exist from the wedge");
        let mut path = smoother::smooth_path(
            &raw,
            &grid,
            clearance.as_ref(),
            &gp_cfg,
            lp_cfg.dwa.max_curvature,
            Some(&zone),
            None,
        );
        assert_eq!(
            path[1].dir,
            global_planner::SegmentDir::Reverse,
            "escape plan must lead with a reverse leg"
        );

        // Phase 3 — closed loop: pursuit must drive Recovery, the robot must
        // back out, and the scripted reverse must never engage.
        let mut recovery_pursuit_cycles = 0u32;
        let mut organic_exit = false;
        let mut escaped = false;
        for cycle in 0..600 {
            let near = active_run_truncation_index(&path, state.x, state.y);
            if near > 0 {
                path.drain(..near);
            }
            let reverse_escape = plan_reverse_escape(
                &state,
                &obstacles,
                reverse_check_radius,
                REAR_CLEAR_MARGIN,
                lp_cfg.dwa.moving_obstacle_margin_gain,
                bp.reverse_target_m(),
                RECOVERY_REVERSE_SPEED,
                lp_cfg.dwa.max_curvature * 0.5,
            );
            let out = bp.update(&BehaviorInput {
                robot_x: state.x,
                robot_y: state.y,
                robot_theta: state.theta,
                localization_confidence: 1.0,
                nearest_obstacle_distance: nearest(&state, &obstacles),
                emergency_stop: false,
                dt,
                robot_speed: state.linear_vel,
                planner_infeasible: prev_infeasible,
                planner_feasible: prev_feasible,
                rear_clear: reverse_escape.is_ok(),
            });
            assert_ne!(
                out.recovery_phase,
                Some(RecoveryPhase::Reverse),
                "cycle {cycle}: reached the scripted-reverse phase despite an executable escape plan"
            );
            let (plan, fallback, _) = select_local_plan(
                &mut local,
                out.recovery_phase,
                &state,
                &path,
                &obstacles,
                out.desired_speed,
                &reverse_escape,
            );
            assert!(
                fallback.is_none(),
                "cycle {cycle}: scripted reverse engaged with a path present"
            );
            if out.state == DrivingState::Recovery {
                assert_eq!(
                    plan.executor,
                    Executor::Pursuit,
                    "cycle {cycle}: Recovery command must come from the pursuit executor"
                );
                recovery_pursuit_cycles += 1;
            } else if recovery_pursuit_cycles > 0 && !organic_exit {
                // First exit after pursuit-driven Recovery cycles: net
                // displacement must still be below the 0.3m movement exit,
                // proving the exit came from feasible-maneuver cycles.
                let net = (state.x * state.x + state.y * state.y).sqrt();
                assert!(
                    net < 0.3,
                    "cycle {cycle}: recovery exit at {net:.2}m — expected the feasible-cycle \
                     exit before the movement exit"
                );
                organic_exit = true;
            }
            (prev_infeasible, prev_feasible) =
                classify_plan(&plan.command, plan.planned_maneuver, 0.3);

            state.x += plan.command.linear_x * state.theta.cos() * dt;
            state.y += plan.command.linear_x * state.theta.sin() * dt;
            state.theta += plan.command.angular_z * dt;
            state.linear_vel = plan.command.linear_x;
            state.angular_vel = plan.command.angular_z;

            if (state.x * state.x + state.y * state.y).sqrt() > 0.45 {
                escaped = true;
                break;
            }
        }
        assert!(recovery_pursuit_cycles > 0, "pursuit never drove Recovery");
        assert!(organic_exit, "Recovery never exited via feasible cycles");
        assert!(
            escaped,
            "robot never left the pocket (at {:.2},{:.2})",
            state.x, state.y
        );
    }
}
