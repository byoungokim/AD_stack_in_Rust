/// Limo Drive — Sensing & Perception Process (Process 1)
///
/// Owns all sensor input and perception computation.
/// Uses the HAL SensorSource trait — works with any platform
/// (Limo Pro hardware, Gazebo, Isaac Sim, dummy test data).
///
/// Publishes aggregated WorldState on CH1 (ZMQ PUB tcp:5551).
/// Subscribes to VehicleState on CH3 (for sensor fusion / EKF).
mod config;
mod perception;
mod slam;
mod store;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, error, info, warn};
use tracing_subscriber::EnvFilter;

use config::{load_config, SensPercConfig};
use store::SensorStore;

use limo_hal::dummy::DummySensorSource;
use limo_hal::limo_hw::{LimoHwSensorConfig, LimoHwSensorSource};
use limo_hal::sim_zmq::SimZmqSensorSource;
use limo_hal::SensorSource;
use limo_transport::{Channel, HeartbeatManager, Publisher, Subscriber};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: SensPerc Process Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/sensperc.yaml".into());
    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!(
            "Failed to load config from '{}': {}, using defaults",
            config_path, e
        );
        SensPercConfig::default()
    });

    let sim_mode =
        std::env::args().any(|a| a == "--sim") || std::env::var("LIMO_SIM").is_ok_and(|v| v == "1");
    let dummy_mode = std::env::args().any(|a| a == "--dummy");

    info!("Config loaded: aggregator={}Hz", config.aggregator.rate_hz);

    ctrlc_handler();

    // Start heartbeat manager
    let mut heartbeat = HeartbeatManager::start("sensperc")?;

    // --- Select sensor source via HAL ---
    let mut source: Box<dyn SensorSource> = if sim_mode {
        if config.sim_faults.is_active() {
            info!(
                "Platform: SimZmq (CH5) + fault injection (cam={:.2} lidar={:.2} imu={:.2} pose={:.2} vel={:.2} seed={})",
                config.sim_faults.camera_drop_rate,
                config.sim_faults.lidar_drop_rate,
                config.sim_faults.imu_drop_rate,
                config.sim_faults.pose_drop_rate,
                config.sim_faults.velocity_drop_rate,
                config.sim_faults.seed,
            );
        } else {
            info!("Platform: SimZmq (subscribing CH5)");
        }
        Box::new(SimZmqSensorSource::with_faults(config.sim_faults.clone()))
    } else if dummy_mode {
        info!("Platform: Dummy (synthetic data)");
        Box::new(DummySensorSource::new())
    } else {
        info!("Platform: Limo Pro hardware");
        Box::new(LimoHwSensorSource::new(LimoHwSensorConfig::default()))
    };

    source.start()?;
    info!("SensorSource '{}' started", source.name());

    // Create shared sensor store (for intra-process ring buffers)
    let store = Arc::new(SensorStore::new());

    // Start sensor reader thread (pumps HAL source → store)
    let reader_store = Arc::clone(&store);
    let reader_handle = thread::Builder::new()
        .name("SensorReader".into())
        .spawn(move || {
            sensor_reader_loop(source, &reader_store);
        })?;

    // Start SLAM thread (feature-based scan matching + occupancy grid)
    let slam_store = Arc::clone(&store);
    let slam_handle = thread::Builder::new()
        .name("SlamFrontend".into())
        .spawn(move || {
            slam::slam_loop(&slam_store, &SHUTDOWN);
        })?;

    // Start perception thread (YOLO object detection from camera)
    let perc_store = Arc::clone(&store);
    let model_path = "models/yolov8n.onnx".to_string();
    let perc_handle = thread::Builder::new()
        .name("Perception".into())
        .spawn(move || {
            perception::perception_loop(&perc_store, &SHUTDOWN, &model_path);
        })?;

    // Start aggregator loop (publishes WorldState on CH1, subscribes CH3)
    let agg_store = Arc::clone(&store);
    let agg_config = config.aggregator.clone();
    let agg_mount = config.lidar_mount.clone();
    let agg_handle = thread::Builder::new()
        .name("Aggregator".into())
        .spawn(move || {
            if let Err(e) = aggregator_loop(&agg_store, &agg_config, &agg_mount) {
                error!("Aggregator error: {:#}", e);
            }
        })?;

    // Main thread: monitor stats until shutdown
    let stats_interval = Duration::from_secs(5);
    let mut last_stats = Instant::now();

    while !SHUTDOWN.load(Ordering::Acquire) {
        thread::sleep(Duration::from_millis(100));

        if last_stats.elapsed() >= stats_interval {
            let stats = store.stats();
            info!(
                "SensorStore stats: camera={} frames, lidar={} scans, imu={} readings",
                stats.camera_frames, stats.lidar_scans, stats.imu_readings
            );
            last_stats = Instant::now();
        }
    }

    // Graceful shutdown
    info!("Shutting down SensPerc...");
    let _ = reader_handle.join();
    let _ = slam_handle.join();
    let _ = perc_handle.join();
    let _ = agg_handle.join();
    heartbeat.stop();
    info!("=== SensPerc Process Stopped ===");

    Ok(())
}

/// Reads from the HAL SensorSource and pushes into the SensorStore.
/// Runs in a dedicated thread, polling the source continuously.
fn sensor_reader_loop(mut source: Box<dyn SensorSource>, store: &Arc<SensorStore>) {
    info!("SensorReader started for source '{}'", source.name());

    while !SHUTDOWN.load(Ordering::Acquire) {
        let mut got_data = false;

        // Camera
        if let Some(frame) = source.recv_camera() {
            store.push_camera_frame(store::types::CameraFrame {
                timestamp_ns: frame.timestamp_ns,
                width: frame.width,
                height: frame.height,
                encoding: frame.encoding,
                data: frame.data,
                sequence: frame.sequence,
            });
            got_data = true;
        }

        // LiDAR: into the SLAM work queue AND the aggregator's latest slot
        if let Some(scan) = source.recv_lidar() {
            store.latest_scan.store(store::types::LidarScan {
                timestamp_ns: scan.timestamp_ns,
                angle_min: scan.angle_min,
                angle_max: scan.angle_max,
                angle_increment: scan.angle_increment,
                range_min: scan.range_min,
                range_max: scan.range_max,
                ranges: scan.ranges.clone(),
                intensities: scan.intensities.clone(),
                sequence: scan.sequence,
            });
            store.push_lidar_scan(store::types::LidarScan {
                timestamp_ns: scan.timestamp_ns,
                angle_min: scan.angle_min,
                angle_max: scan.angle_max,
                angle_increment: scan.angle_increment,
                range_min: scan.range_min,
                range_max: scan.range_max,
                ranges: scan.ranges,
                intensities: scan.intensities,
                sequence: scan.sequence,
            });
            got_data = true;
        }

        // IMU
        if let Some(imu) = source.recv_imu() {
            store.push_imu_reading(store::types::ImuReading {
                timestamp_ns: imu.timestamp_ns,
                linear_acceleration: imu.linear_acceleration,
                angular_velocity: imu.angular_velocity,
                orientation_euler: imu.orientation_euler,
                sequence: imu.sequence,
            });
            got_data = true;
        }

        // Pose (from sim ground truth or SLAM), stamped with the time it
        // describes so the aggregator can project scans with the scan-time
        // pose instead of the latest one.
        if let Some(stamped) = source.recv_pose() {
            store.push_stamped_pose(
                stamped.timestamp_ns,
                store::types::Pose2D {
                    x: stamped.pose.x,
                    y: stamped.pose.y,
                    theta: stamped.pose.theta,
                },
            );
            store.localization_confidence.store(stamped.confidence);
            got_data = true;
        }

        // Velocity
        if let Some(vel) = source.recv_velocity() {
            store.latest_velocity.store(store::types::Twist2D {
                linear_x: vel.linear_x,
                linear_y: vel.linear_y,
                angular_z: vel.angular_z,
            });
            got_data = true;
        }

        if !got_data {
            thread::sleep(Duration::from_millis(1)); // avoid busy-wait
        }
    }

    source.stop();
    info!("SensorReader stopped");
}

/// Aggregator loop: reads latest sensor data, subscribes CH3, publishes WorldState on CH1.
fn aggregator_loop(
    store: &Arc<SensorStore>,
    config: &config::AggregatorConfig,
    mount: &config::LidarMountConfig,
) -> Result<()> {
    let zmq_ctx = zmq::Context::new();

    let mut ch1_pub = Publisher::bind(&zmq_ctx, &config.ch1_endpoint, Channel::WorldState.topic())?;

    let ch3_connect = Channel::VehicleState.connect_endpoint();
    let mut ch3_sub = Subscriber::connect(&zmq_ctx, ch3_connect, Channel::VehicleState.topic())?;

    info!(
        "Aggregator started at {}Hz, CH1={}, CH3={}",
        config.rate_hz, config.ch1_endpoint, ch3_connect
    );

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut cycle: u64 = 0;

    let mut obstacle_tracker =
        perception::tracker::ClusterTracker::new(perception::tracker::TrackerConfig::default());
    let cluster_params = perception::tracker::ClusterParams::default();
    // Extent-gate rejections since the last periodic log (avoid per-scan spam).
    let mut structure_rejects: u64 = 0;
    let mut last_track_update = Instant::now();
    let mut last_scan_seq: Option<u32> = None;
    let mut prev_detections: Option<limo_proto::DetectionArray> = None;

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // Read VehicleState from CH3 (odometry fallback)
        if let Ok(Some(vs)) = ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(1)) {
            if let Some(pose) = &vs.odometry_pose {
                if store.latest_pose.age_secs() > 0.5 {
                    let ts = vs.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0);
                    store.push_stamped_pose(
                        ts,
                        store::types::Pose2D {
                            x: pose.x,
                            y: pose.y,
                            theta: pose.theta,
                        },
                    );
                    store.localization_confidence.store(0.6);
                }
            }
            if let Some(vel) = &vs.odometry_velocity {
                if store.latest_velocity.age_secs() > 0.5 {
                    store.latest_velocity.store(store::types::Twist2D {
                        linear_x: vel.linear_x,
                        linear_y: vel.linear_y,
                        angular_z: vel.angular_z,
                    });
                }
            }
        }

        // Read the latest scan non-destructively. The camera and imu ring
        // buffers belong to the perception and SLAM threads — popping them
        // here starved those consumers.
        let latest_lidar = store.latest_scan.load();
        let is_new_scan = latest_lidar
            .as_ref()
            .is_some_and(|s| Some(s.sequence) != last_scan_seq);
        if let Some(scan) = &latest_lidar {
            last_scan_seq = Some(scan.sequence);
        }

        let pose = store.latest_pose.load().unwrap_or_default();
        let velocity = store.latest_velocity.load().unwrap_or_default();
        let loc_confidence = store.localization_confidence.load().unwrap_or(0.0);

        // --- LiDAR obstacle detection: cluster, track, sample ---
        // 1. Valid nearby returns are clustered by adjacent-point distance.
        // 2. Compact clusters (cones, boxes, pedestrians) are tracked across
        //    scans for stable ids and velocity estimates.
        // 3. Everything else (walls, noise) keeps the nearest-return-per-
        //    sector point representation, so the closest obstacle in every
        //    direction always survives sampling.
        let detections = if !is_new_scan {
            // Same scan as last cycle (or none yet): re-emit the previous
            // detections instead of publishing an obstacle-free WorldState —
            // planning must never see the world blink empty.
            prev_detections.clone()
        } else if let Some(scan) = &latest_lidar {
            let angle_inc = scan_angle_increment(scan);

            // Project with the pose the robot had AT SCAN TIME (interpolated
            // from the stamped history), not the latest pose: during fast yaw
            // the skew displaces projected obstacles by omega*skew*range.
            let scan_pose = store
                .pose_at(scan.timestamp_ns)
                .unwrap_or_else(|| pose.clone());
            // Beams are measured from the lidar's mount pose on the chassis,
            // not from base_link center — an unmodeled mount offset displaces
            // every projected obstacle by exactly that offset.
            let origin = lidar_world_pose(&scan_pose, mount);

            // Valid returns in beam order, with world-frame coordinates.
            let returns = scan_world_returns(scan, origin, MAX_OBSTACLE_RANGE);
            let pts: Vec<(f64, f64)> = returns.iter().map(|&(_, _, x, y)| (x, y)).collect();
            let beam_ranges: Vec<f64> = returns.iter().map(|&(_, r, _, _)| r as f64).collect();

            // Range-gated (<= cluster max_range), range-adaptive clustering:
            // returns farther out only feed the sector points and the SLAM
            // grid — at long range the beam-arc spacing rivals the cluster
            // eps and wall segments would fragment into phantom objects.
            let clusters = perception::tracker::cluster_scan_returns(
                &pts,
                &beam_ranges,
                angle_inc as f64,
                &cluster_params,
            );
            // Extent gate: oversized clusters are structure (walls, gate
            // frames), never tracks. Shape gate: elongated slivers (grazing
            // wall fragments small enough to slip the extent gate) are
            // structure too. Both stay in the point representation below and
            // in the occupancy grid.
            let (compact, structure): (Vec<_>, Vec<_>) = clusters.into_iter().partition(|c| {
                c.radius <= perception::tracker::MAX_TRACK_EXTENT
                    && !c.is_wall_like(&cluster_params)
            });
            structure_rejects += structure.len() as u64;

            let mut in_compact = vec![false; pts.len()];
            for c in &compact {
                for &i in &c.point_indices {
                    in_compact[i] = true;
                }
            }

            let dt = last_track_update.elapsed().as_secs_f64();
            last_track_update = Instant::now();
            let tracked = obstacle_tracker.update(&compact, dt);

            // Static/wall points: everything not owned by a compact cluster.
            let static_returns: Vec<(f32, f32)> = returns
                .iter()
                .enumerate()
                .filter(|(i, _)| !in_compact[*i])
                .map(|(_, &(angle, range, _, _))| (angle, range))
                .collect();

            let mut dets: Vec<limo_proto::Detection> =
                nearest_per_sector(&static_returns, OBSTACLE_SECTORS)
                    .into_iter()
                    .map(|(angle, range)| {
                        let (wx, wy) = project_return(origin, angle, range);
                        limo_proto::Detection {
                            object_class: limo_proto::ObjectClass::ObjectObstacle as i32,
                            confidence: 0.8,
                            bbox_image: None,
                            position_world: Some(limo_proto::Point2D { x: wx, y: wy }),
                            distance: range,
                            velocity_world: None,
                            radius: 0.0,
                            track_id: 0,
                        }
                    })
                    .collect();

            for t in &tracked {
                let dist = ((t.x - pose.x).powi(2) + (t.y - pose.y).powi(2)).sqrt();
                dets.push(limo_proto::Detection {
                    object_class: limo_proto::ObjectClass::ObjectObstacle as i32,
                    confidence: 0.9,
                    bbox_image: None,
                    position_world: Some(limo_proto::Point2D { x: t.x, y: t.y }),
                    distance: dist as f32,
                    velocity_world: Some(limo_proto::Twist2D {
                        linear_x: t.vx,
                        linear_y: t.vy,
                        angular_z: 0.0,
                    }),
                    radius: t.radius as f32,
                    track_id: t.id,
                });
            }
            Some(limo_proto::DetectionArray {
                header: None,
                detections: dets,
            })
        } else {
            None
        };
        prev_detections = detections.clone();

        // Merge camera detections (from YOLO) if available
        let detections = if let Some(cam_dets) = store.latest_detections.load() {
            let mut all_dets = detections.map_or(vec![], |d| d.detections);
            for cd in &cam_dets {
                // Camera detections have bounding boxes but no world position
                // Estimate distance from bbox size (rough heuristic)
                let bbox_height = cd.y2 - cd.y1;
                let est_distance = if bbox_height > 10.0 {
                    (480.0 / bbox_height) * 0.5 // rough depth from bbox
                } else {
                    5.0
                };

                all_dets.push(limo_proto::Detection {
                    object_class: cd.class_id as i32,
                    confidence: cd.confidence,
                    bbox_image: Some(limo_proto::BoundingBox2D {
                        x_center: ((cd.x1 + cd.x2) / 2.0) as f64,
                        y_center: ((cd.y1 + cd.y2) / 2.0) as f64,
                        width: (cd.x2 - cd.x1) as f64,
                        height: (cd.y2 - cd.y1) as f64,
                        angle: 0.0,
                    }),
                    position_world: Some(limo_proto::Point2D {
                        x: pose.x + est_distance as f64 * pose.theta.cos(),
                        y: pose.y + est_distance as f64 * pose.theta.sin(),
                    }),
                    distance: est_distance,
                    velocity_world: None,
                    radius: 0.0,
                    track_id: 0,
                });
            }
            Some(limo_proto::DetectionArray {
                header: None,
                detections: all_dets,
            })
        } else {
            detections
        };

        // Compose and publish WorldState on CH1
        let world_state = limo_proto::WorldState {
            header: Some(limo_proto::Header {
                timestamp_ns: now_ns(),
                sequence: cycle as u32,
                frame_id: "world".into(),
            }),
            robot_pose: Some(limo_proto::Pose2D {
                x: pose.x,
                y: pose.y,
                theta: pose.theta,
            }),
            robot_velocity: Some(limo_proto::Twist2D {
                linear_x: velocity.linear_x,
                linear_y: velocity.linear_y,
                angular_z: velocity.angular_z,
            }),
            detections,
            lanes: None,
            local_map: store
                .slam_local_map
                .load()
                .map(|m| limo_proto::OccupancyGrid {
                    header: None,
                    width: m.width as u32,
                    height: m.height as u32,
                    resolution: m.resolution as f32,
                    origin: Some(limo_proto::Pose2D {
                        x: m.origin_x,
                        y: m.origin_y,
                        theta: 0.0,
                    }),
                    data: m.data,
                }),
            localization_confidence: loc_confidence,
        };

        if let Err(e) = ch1_pub.publish(&world_state) {
            warn!("Failed to publish WorldState: {:#}", e);
        }

        cycle += 1;
        if cycle.is_multiple_of(config.rate_hz as u64 * 10) {
            let stats = store.stats();
            info!(
                "Aggregator cycle {}: cam={}, lidar={}, imu={}, ch1_sent={}",
                cycle,
                stats.camera_frames,
                stats.lidar_scans,
                stats.imu_readings,
                ch1_pub.msg_count(),
            );
            if structure_rejects > 0 {
                debug!(
                    "Extent/shape gates rejected {} clusters since last report \
                     (structure stays in grid/sector points)",
                    structure_rejects
                );
                structure_rejects = 0;
            }
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    info!("Aggregator stopped");
    Ok(())
}

/// Obstacle detection range limit (meters) for lidar-derived detections.
// 3.0 starved the planner at speed: at 1.8 m/s that is a 1.7s horizon — the
// robot outran its own map (wedged into cones it discovered at margin
// distance). 8.0 gives DWA's braking check and the global planner room to
// react at the 2.2 m/s gauntlet target.
const MAX_OBSTACLE_RANGE: f32 = 8.0;
// Clustering does NOT run out to MAX_OBSTACLE_RANGE: cluster formation is
// range-gated and eps is range-adaptive (see
// `perception::tracker::ClusterParams`), and tracks are extent-gated at
// `perception::tracker::MAX_TRACK_EXTENT`. Returns beyond the cluster gate
// still feed the sector points below and the SLAM grid at the full 8 m.
/// Angular sectors for obstacle sampling. 72 sectors = 5° each: at the 3 m
/// range limit a sector spans 0.26 m of arc, under the planner's 0.3 m
/// inflation radius, so sampled wall points always merge into a continuous
/// inflated barrier.
const OBSTACLE_SECTORS: usize = 72;

/// Effective angle step of a scan: the reported increment when present,
/// otherwise derived from the angular span (gz convention: `num_points`
/// beams inclusive of both endpoints, step = span / (n - 1)).
fn scan_angle_increment(scan: &store::types::LidarScan) -> f32 {
    let num_points = scan.ranges.len();
    if scan.angle_increment > 0.0 {
        scan.angle_increment
    } else if num_points > 1 {
        (scan.angle_max - scan.angle_min) / (num_points - 1) as f32
    } else {
        0.0
    }
}

/// Planar world pose of the lidar: the robot pose composed with the
/// sensor's mount transform on the chassis (base_link → lidar).
///
/// The gz model (simulation/models/limo_pro/model.sdf) mounts the lidar at
/// the base center, so the sim mount is zero; hardware mounts configure
/// `lidar_mount` in sensperc.yaml.
fn lidar_world_pose(
    robot: &store::types::Pose2D,
    mount: &config::LidarMountConfig,
) -> (f64, f64, f64) {
    let (s, c) = robot.theta.sin_cos();
    (
        robot.x + mount.x * c - mount.y * s,
        robot.y + mount.x * s + mount.y * c,
        robot.theta + mount.yaw,
    )
}

/// World position of a single lidar return. `angle` is the beam angle in
/// the sensor frame: CCW, zero along the sensor's +x (forward) — the gz
/// gpu_lidar / RPLIDAR convention.
fn project_return(origin: (f64, f64, f64), angle: f32, range: f32) -> (f64, f64) {
    let a = origin.2 + angle as f64;
    (
        origin.0 + range as f64 * a.cos(),
        origin.1 + range as f64 * a.sin(),
    )
}

/// Project a scan's valid returns into the world frame from the lidar's
/// world pose. Returns `(beam_angle, range, world_x, world_y)` in beam
/// order; returns outside `[range_min, max_range]` are dropped.
fn scan_world_returns(
    scan: &store::types::LidarScan,
    origin: (f64, f64, f64),
    max_range: f32,
) -> Vec<(f32, f32, f64, f64)> {
    let angle_inc = scan_angle_increment(scan);
    scan.ranges
        .iter()
        .enumerate()
        .filter(|(_, &r)| r >= scan.range_min && r <= max_range)
        .map(|(i, &range)| {
            let angle = scan.angle_min + i as f32 * angle_inc;
            let (wx, wy) = project_return(origin, angle, range);
            (angle, range, wx, wy)
        })
        .collect()
}

/// Keep the nearest return per angular sector.
///
/// Guarantees the closest obstacle in EVERY direction survives sampling —
/// unlike uniform decimation, which can drop the return for the wall right
/// beside the robot while keeping farther points elsewhere in the scan.
/// Takes pre-filtered (beam angle, range) pairs and returns at most
/// `num_sectors` of them.
fn nearest_per_sector(returns: &[(f32, f32)], num_sectors: usize) -> Vec<(f32, f32)> {
    let mut nearest: Vec<Option<(f32, f32)>> = vec![None; num_sectors];
    let sector_width = std::f32::consts::TAU / num_sectors as f32;
    for &(angle, range) in returns {
        let idx = (angle.rem_euclid(std::f32::consts::TAU) / sector_width) as usize % num_sectors;
        if nearest[idx].is_none_or(|(_, r)| range < r) {
            nearest[idx] = Some((angle, range));
        }
    }
    nearest.into_iter().flatten().collect()
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
    use std::f32::consts::TAU;

    /// Build (angle, range) pairs from a 1°/beam scan the way the aggregator
    /// does: only returns within [0.1, 3.0] survive.
    fn pairs(ranges: &[f32], angle_min: f32) -> Vec<(f32, f32)> {
        ranges
            .iter()
            .enumerate()
            .filter(|(_, &r)| (0.1..=3.0).contains(&r))
            .map(|(i, &r)| (angle_min + i as f32 * (TAU / 360.0), r))
            .collect()
    }

    /// A 360-beam scan (1°/beam) with all ranges out of detection range.
    fn empty_scan() -> Vec<f32> {
        vec![10.0; 360]
    }

    #[test]
    fn nearest_wall_return_always_survives() {
        // Dense far returns everywhere + one close "wall" return at 90°.
        // Uniform decimation could drop it; sector sampling must not.
        let mut ranges = vec![2.9_f32; 360];
        ranges[90] = 0.4;
        let out = nearest_per_sector(&pairs(&ranges, 0.0), 72);
        assert!(out.iter().any(|&(_, r)| (r - 0.4).abs() < 1e-6));
        // and the sector containing 90° reports the wall, not a 2.9m point
        let sector_width = TAU / 72.0;
        let wall_sector = ((90.0_f32.to_radians()) / sector_width) as usize;
        let in_sector: Vec<_> = out
            .iter()
            .filter(|&&(a, _)| (a.rem_euclid(TAU) / sector_width) as usize == wall_sector)
            .collect();
        assert_eq!(in_sector.len(), 1);
        assert!((in_sector[0].1 - 0.4).abs() < 1e-6);
    }

    #[test]
    fn keeps_nearest_within_a_sector() {
        let mut ranges = empty_scan();
        ranges[11] = 2.0; // 11° and 13° both sit inside the 10°-15° sector
        ranges[13] = 1.0;
        let out = nearest_per_sector(&pairs(&ranges, 0.0), 72);
        assert_eq!(out.len(), 1);
        assert!((out[0].1 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn output_bounded_by_sector_count() {
        let ranges = vec![1.5_f32; 360]; // everything in range
        let out = nearest_per_sector(&pairs(&ranges, 0.0), 72);
        assert_eq!(out.len(), 72);
    }

    #[test]
    fn out_of_range_returns_filtered_before_sampling() {
        let mut ranges = empty_scan();
        ranges[0] = 0.05; // below range_min
        ranges[100] = 5.0; // above max
        let out = nearest_per_sector(&pairs(&ranges, 0.0), 72);
        assert!(out.is_empty());
    }

    // ---- Scan → world projection (mount pose + angle convention) ----

    fn scan(angle_min: f32, angle_max: f32, inc: f32, ranges: Vec<f32>) -> store::types::LidarScan {
        store::types::LidarScan {
            timestamp_ns: 0,
            angle_min,
            angle_max,
            angle_increment: inc,
            range_min: 0.1,
            range_max: 12.0,
            ranges,
            intensities: vec![],
            sequence: 0,
        }
    }

    fn pose2d(x: f64, y: f64, theta: f64) -> store::types::Pose2D {
        store::types::Pose2D { x, y, theta }
    }

    fn mount(x: f64, y: f64, yaw: f64) -> config::LidarMountConfig {
        config::LidarMountConfig { x, y, yaw }
    }

    /// Synthetic scan with a known mount offset: every projected return must
    /// land on its hand-computed world position to within ±3 cm.
    ///
    /// Geometry (all expected values derived by hand, not from the code
    /// under test): robot at (2, 1) heading +y (θ = π/2); lidar mounted
    /// 0.30 m forward and 0.10 m left of base center → sensor origin
    /// (2 - 0.10, 1 + 0.30) = (1.90, 1.30), sensor yaw π/2.
    #[test]
    fn projection_with_mount_offset_recovers_world_points_within_3cm() {
        let mut ranges = vec![20.0_f32; 360]; // 20 m: outside MAX_OBSTACLE_RANGE
        ranges[0] = 2.0; // sensor +x (= world +y) → (1.90, 3.30)
        ranges[45] = 1.5; // 45° CCW → world bearing 3π/4
        ranges[90] = 2.0; // 90° CCW → world -x → (-0.10, 1.30)
        ranges[180] = 2.0; // behind → world -y → (1.90, -0.70)
        let s = scan(0.0, TAU, TAU / 360.0, ranges);

        let robot = pose2d(2.0, 1.0, std::f64::consts::FRAC_PI_2);
        let origin = lidar_world_pose(&robot, &mount(0.30, 0.10, 0.0));
        let returns = scan_world_returns(&s, origin, MAX_OBSTACLE_RANGE);
        assert_eq!(returns.len(), 4, "only in-range beams survive filtering");

        let expected = [
            (1.90, 3.30),
            (0.839_34, 2.360_66), // (1.90, 1.30) + 1.5 * u(3π/4)
            (-0.10, 1.30),
            (1.90, -0.70),
        ];
        for (&(_, _, wx, wy), &(ex, ey)) in returns.iter().zip(expected.iter()) {
            assert!(
                (wx - ex).abs() < 0.03 && (wy - ey).abs() < 0.03,
                "projected ({wx:.3}, {wy:.3}) != expected ({ex:.3}, {ey:.3})"
            );
        }

        // Negative control — the defect class: ignoring the mount offset
        // (projecting from base center) displaces every return by the full
        // mount offset magnitude (~0.32 m here), far outside ±3 cm.
        let no_mount = lidar_world_pose(&robot, &mount(0.0, 0.0, 0.0));
        let bad = scan_world_returns(&s, no_mount, MAX_OBSTACLE_RANGE);
        let (_, _, bx, by) = bad[0];
        let err = ((bx - 1.90_f64).powi(2) + (by - 3.30_f64).powi(2)).sqrt();
        assert!(
            err > 0.25,
            "unmodeled mount offset must show as a systematic error, got {err:.3}"
        );
    }

    /// Beam angle convention: CCW, zero = sensor forward (gz gpu_lidar and
    /// RPLIDAR). A beam at +90° from a robot facing +x must land at +y.
    #[test]
    fn projection_angle_convention_is_ccw_zero_forward() {
        let s = scan(
            0.0,
            TAU,
            std::f32::consts::FRAC_PI_2,
            vec![1.0, 2.0, 3.0, 4.0],
        );
        let origin = lidar_world_pose(&pose2d(0.0, 0.0, 0.0), &mount(0.0, 0.0, 0.0));
        let returns = scan_world_returns(&s, origin, MAX_OBSTACLE_RANGE);
        let expected = [(1.0, 0.0), (0.0, 2.0), (-3.0, 0.0), (0.0, -4.0)];
        for (&(_, _, wx, wy), &(ex, ey)) in returns.iter().zip(expected.iter()) {
            assert!(
                (wx - ex).abs() < 0.03 && (wy - ey).abs() < 0.03,
                "CCW/zero-forward violated: ({wx:.3}, {wy:.3}) != ({ex}, {ey})"
            );
        }
    }

    /// A mount yaw rotates every beam: sensor twisted +90° on the chassis
    /// puts its forward beam at the robot's +y.
    #[test]
    fn projection_applies_mount_yaw() {
        let s = scan(0.0, TAU, TAU / 360.0, vec![1.0]);
        let origin = lidar_world_pose(
            &pose2d(0.0, 0.0, 0.0),
            &mount(0.0, 0.0, std::f64::consts::FRAC_PI_2),
        );
        let returns = scan_world_returns(&s, origin, MAX_OBSTACLE_RANGE);
        let (_, _, wx, wy) = returns[0];
        assert!(wx.abs() < 0.03 && (wy - 1.0).abs() < 0.03);
    }

    /// Missing angle_increment falls back to span / (n - 1) — the gz
    /// endpoint-inclusive convention.
    #[test]
    fn projection_derives_increment_from_span_when_missing() {
        let s = scan(0.0, std::f32::consts::PI, 0.0, vec![1.0, 1.0, 1.0]);
        let origin = lidar_world_pose(&pose2d(0.0, 0.0, 0.0), &mount(0.0, 0.0, 0.0));
        let returns = scan_world_returns(&s, origin, MAX_OBSTACLE_RANGE);
        let expected = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        for (&(_, _, wx, wy), &(ex, ey)) in returns.iter().zip(expected.iter()) {
            assert!((wx - ex).abs() < 0.03 && (wy - ey).abs() < 0.03);
        }
    }

    #[test]
    fn negative_angles_wrap_into_valid_sectors() {
        // Scan starting at -π (common lidar convention) must not panic or
        // alias sectors.
        let mut ranges = empty_scan();
        ranges[0] = 1.0; // beam at -π
        ranges[359] = 2.0; // beam just below +π — same physical direction band
        let out = nearest_per_sector(&pairs(&ranges, -std::f32::consts::PI), 72);
        assert!(!out.is_empty());
        assert!(out.iter().all(|&(_, r)| (0.1..=3.0).contains(&r)));
    }
}
