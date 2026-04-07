/// Limo Drive — Sensing & Perception Process (Process 1)
///
/// Owns all sensor hardware and perception computation.
/// Publishes aggregated WorldState on CH1 (ZMQ PUB tcp:5551).
/// Subscribes to VehicleState on CH3 (for sensor fusion / EKF).
///
/// Architecture:
///   CameraDriver (30Hz) ──┐
///   LidarDriver  (10Hz) ──┤── SensorStore ── Aggregator (10Hz) ──> CH1: WorldState
///   ImuDriver   (100Hz) ──┘
///   [CH3: VehicleState] ──> SensorStore (for fusion)
mod config;
mod drivers;
mod store;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{debug, error, info, warn};
use tracing_subscriber::EnvFilter;

use config::{load_config, SensPercConfig};
use drivers::camera::CameraDriver;
use drivers::imu::ImuDriver;
use drivers::lidar::LidarDriver;
use store::SensorStore;

use limo_transport::{Channel, Publisher, Subscriber};

/// Global shutdown flag, shared across all threads.
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: SensPerc Process Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/sensperc.yaml".into());
    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!("Failed to load config from '{}': {}, using defaults", config_path, e);
        SensPercConfig::default()
    });

    info!("Config loaded: camera={}Hz, lidar={}Hz, imu={}Hz, aggregator={}Hz",
          config.camera.fps, config.lidar.scan_rate_hz,
          config.imu.rate_hz, config.aggregator.rate_hz);

    let sim_mode = std::env::args().any(|a| a == "--sim")
        || std::env::var("LIMO_SIM").map_or(false, |v| v == "1");

    ctrlc_handler();

    // Create shared sensor store
    let store = Arc::new(SensorStore::new());

    // --- Sensor input: hardware drivers OR sim bridge ---
    let mut camera = None;
    let mut lidar = None;
    let mut imu = None;

    if sim_mode {
        info!("SIM MODE: subscribing CH5 (SimSensors) instead of hardware drivers");
        let sim_store = Arc::clone(&store);
        thread::Builder::new()
            .name("SimSensorSub".into())
            .spawn(move || {
                if let Err(e) = sim_sensor_loop(&sim_store) {
                    error!("SimSensor subscriber error: {:#}", e);
                }
            })?;
    } else {
        info!("REAL MODE: starting hardware sensor drivers");
        let mut cam = CameraDriver::new(Arc::clone(&store), config.camera.clone());
        let mut lid = LidarDriver::new(Arc::clone(&store), config.lidar.clone());
        let mut im = ImuDriver::new(Arc::clone(&store), config.imu.clone());
        cam.start()?;
        lid.start()?;
        im.start()?;
        camera = Some(cam);
        lidar = Some(lid);
        imu = Some(im);
    }

    // Start aggregator loop (publishes WorldState on CH1, subscribes CH3)
    let agg_store = Arc::clone(&store);
    let agg_config = config.aggregator.clone();
    let agg_handle = thread::Builder::new()
        .name("Aggregator".into())
        .spawn(move || {
            if let Err(e) = aggregator_loop(&agg_store, &agg_config) {
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
    if let Some(mut c) = camera { c.stop(); }
    if let Some(mut l) = lidar { l.stop(); }
    if let Some(mut i) = imu { i.stop(); }
    let _ = agg_handle.join();
    info!("=== SensPerc Process Stopped ===");

    Ok(())
}

/// Aggregator loop: reads latest sensor data, subscribes CH3, publishes WorldState on CH1.
fn aggregator_loop(
    store: &Arc<SensorStore>,
    config: &config::AggregatorConfig,
) -> Result<()> {
    // --- ZMQ setup ---
    let zmq_ctx = zmq::Context::new();

    // CH1 publisher: WorldState → Planning
    let mut ch1_pub = Publisher::bind(
        &zmq_ctx,
        &config.ch1_endpoint,
        Channel::WorldState.topic(),
    )?;

    // CH3 subscriber: VehicleState from Control (for sensor fusion)
    let ch3_connect = Channel::VehicleState.connect_endpoint();
    let mut ch3_sub = Subscriber::connect(
        &zmq_ctx,
        ch3_connect,
        Channel::VehicleState.topic(),
    )?;

    info!(
        "Aggregator started at {}Hz, CH1={}, CH3={}",
        config.rate_hz, config.ch1_endpoint, ch3_connect
    );

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut cycle: u64 = 0;

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // --- Read VehicleState from CH3 (non-blocking) ---
        // Use odometry as fallback localization when no SLAM or sim ground truth
        match ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(1)) {
            Ok(Some(vs)) => {
                if let Some(pose) = &vs.odometry_pose {
                    // Only use odometry if no higher-priority source (sim/SLAM) is fresh
                    if store.latest_pose.age_secs() > 0.5 {
                        store.latest_pose.store(store::types::Pose2D {
                            x: pose.x, y: pose.y, theta: pose.theta,
                        });
                        store.localization_confidence.store(0.6); // odometry = moderate
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
            Ok(None) => {}
            Err(e) => {
                debug!("CH3 recv error: {:#}", e);
            }
        }

        // --- Read latest sensor data ---
        let latest_camera = store.camera_buffer.pop_latest();
        let latest_lidar = store.lidar_buffer.pop_latest();
        let _latest_imu = store.imu_buffer.pop_latest();

        // --- Read localization from store (set by sim/SLAM/odometry) ---
        let pose = store.latest_pose.load().unwrap_or_default();
        let velocity = store.latest_velocity.load().unwrap_or_default();
        let loc_confidence = store.localization_confidence.load().unwrap_or(0.0);

        // --- Compose and publish WorldState on CH1 ---
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
            detections: if latest_camera.is_some() {
                // TODO: run object detection
                Some(limo_proto::DetectionArray {
                    header: None,
                    detections: vec![],
                })
            } else {
                None
            },
            lanes: None, // TODO: lane detection
            local_map: if latest_lidar.is_some() {
                // TODO: build from LiDAR
                Some(limo_proto::OccupancyGrid {
                    header: None,
                    width: 0,
                    height: 0,
                    resolution: 0.05,
                    origin: None,
                    data: vec![],
                })
            } else {
                None
            },
            localization_confidence: loc_confidence,
        };

        if let Err(e) = ch1_pub.publish(&world_state) {
            warn!("Failed to publish WorldState: {:#}", e);
        }

        cycle += 1;
        if cycle % (config.rate_hz as u64 * 10) == 0 {
            let stats = store.stats();
            info!(
                "Aggregator cycle {}: cam={}, lidar={}, imu={}, ch1_sent={}",
                cycle, stats.camera_frames, stats.lidar_scans, stats.imu_readings,
                ch1_pub.msg_count(),
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            thread::sleep(interval - elapsed);
        }
    }

    info!("Aggregator stopped");
    Ok(())
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}

/// Subscribe to CH5 (SimSensors from Isaac Sim bridge) and inject into SensorStore.
fn sim_sensor_loop(store: &Arc<SensorStore>) -> Result<()> {
    let zmq_ctx = zmq::Context::new();
    let mut ch5_sub = Subscriber::connect(
        &zmq_ctx,
        Channel::SimSensors.connect_endpoint(),
        Channel::SimSensors.topic(),
    )?;

    info!("SimSensor subscriber connected to {}", Channel::SimSensors.connect_endpoint());

    while !SHUTDOWN.load(Ordering::Acquire) {
        match ch5_sub.recv::<limo_proto::SimSensorData>(Duration::from_millis(50)) {
            Ok(Some(sim_data)) => {
                // Inject camera frame
                if !sim_data.camera_image.is_empty() {
                    let frame = store::types::CameraFrame {
                        timestamp_ns: sim_data.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0),
                        width: sim_data.camera_width,
                        height: sim_data.camera_height,
                        encoding: sim_data.camera_encoding.clone(),
                        data: sim_data.camera_image,
                        sequence: sim_data.header.as_ref().map(|h| h.sequence).unwrap_or(0),
                    };
                    store.push_camera_frame(frame);
                }

                // Inject LiDAR scan
                if let Some(scan) = sim_data.lidar_scan {
                    let lidar_scan = store::types::LidarScan {
                        timestamp_ns: sim_data.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0),
                        angle_min: scan.angle_min,
                        angle_max: scan.angle_max,
                        angle_increment: scan.angle_increment,
                        range_min: scan.range_min,
                        range_max: scan.range_max,
                        ranges: scan.ranges,
                        intensities: scan.intensities,
                        sequence: sim_data.header.as_ref().map(|h| h.sequence).unwrap_or(0),
                    };
                    store.push_lidar_scan(lidar_scan);
                }

                // Inject IMU reading
                if let Some(imu_data) = sim_data.imu {
                    let accel = imu_data.linear_acceleration.unwrap_or_default();
                    let gyro = imu_data.angular_velocity.unwrap_or_default();
                    let euler = imu_data.orientation_euler.unwrap_or_default();
                    let reading = store::types::ImuReading {
                        timestamp_ns: sim_data.header.as_ref().map(|h| h.timestamp_ns).unwrap_or(0),
                        linear_acceleration: nalgebra::Vector3::new(accel.x, accel.y, accel.z),
                        angular_velocity: nalgebra::Vector3::new(gyro.x, gyro.y, gyro.z),
                        orientation_euler: nalgebra::Vector3::new(euler.x, euler.y, euler.z),
                        sequence: sim_data.header.as_ref().map(|h| h.sequence).unwrap_or(0),
                    };
                    store.push_imu_reading(reading);
                }

                // Store ground truth pose/velocity from sim (highest priority)
                if let Some(gt_pose) = sim_data.ground_truth_pose {
                    store.latest_pose.store(store::types::Pose2D {
                        x: gt_pose.x,
                        y: gt_pose.y,
                        theta: gt_pose.theta,
                    });
                    store.localization_confidence.store(1.0); // ground truth = perfect
                }
                if let Some(gt_vel) = sim_data.ground_truth_velocity {
                    store.latest_velocity.store(store::types::Twist2D {
                        linear_x: gt_vel.linear_x,
                        linear_y: gt_vel.linear_y,
                        angular_z: gt_vel.angular_z,
                    });
                }
            }
            Ok(None) => {} // timeout
            Err(e) => {
                debug!("CH5 recv error: {:#}", e);
            }
        }
    }

    Ok(())
}

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
