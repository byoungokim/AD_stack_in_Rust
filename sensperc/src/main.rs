/// Limo Drive — Sensing & Perception Process (Process 1)
///
/// Owns all sensor input and perception computation.
/// Uses the HAL SensorSource trait — works with any platform
/// (Limo Pro hardware, Gazebo, Isaac Sim, dummy test data).
///
/// Publishes aggregated WorldState on CH1 (ZMQ PUB tcp:5551).
/// Subscribes to VehicleState on CH3 (for sensor fusion / EKF).
mod config;
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

use limo_hal::{SensorSource, Pose2D as HalPose2D, Twist2D as HalTwist2D};
use limo_hal::limo_hw::{LimoHwSensorSource, LimoHwSensorConfig};
use limo_hal::sim_zmq::SimZmqSensorSource;
use limo_hal::dummy::DummySensorSource;
use limo_transport::{Channel, HeartbeatManager, Publisher, Subscriber};

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

    let sim_mode = std::env::args().any(|a| a == "--sim")
        || std::env::var("LIMO_SIM").map_or(false, |v| v == "1");
    let dummy_mode = std::env::args().any(|a| a == "--dummy");

    info!("Config loaded: aggregator={}Hz", config.aggregator.rate_hz);

    ctrlc_handler();

    // Start heartbeat manager
    let mut heartbeat = HeartbeatManager::start("sensperc")?;

    // --- Select sensor source via HAL ---
    let mut source: Box<dyn SensorSource> = if sim_mode {
        info!("Platform: SimZmq (subscribing CH5)");
        Box::new(SimZmqSensorSource::new())
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
    let _ = reader_handle.join();
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
                timestamp_ns: frame.timestamp_ns, width: frame.width,
                height: frame.height, encoding: frame.encoding,
                data: frame.data, sequence: frame.sequence,
            });
            got_data = true;
        }

        // LiDAR
        if let Some(scan) = source.recv_lidar() {
            store.push_lidar_scan(store::types::LidarScan {
                timestamp_ns: scan.timestamp_ns, angle_min: scan.angle_min,
                angle_max: scan.angle_max, angle_increment: scan.angle_increment,
                range_min: scan.range_min, range_max: scan.range_max,
                ranges: scan.ranges, intensities: scan.intensities,
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

        // Pose (from sim ground truth or SLAM)
        if let Some((pose, confidence)) = source.recv_pose() {
            store.latest_pose.store(store::types::Pose2D {
                x: pose.x, y: pose.y, theta: pose.theta,
            });
            store.localization_confidence.store(confidence);
            got_data = true;
        }

        // Velocity
        if let Some(vel) = source.recv_velocity() {
            store.latest_velocity.store(store::types::Twist2D {
                linear_x: vel.linear_x, linear_y: vel.linear_y,
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
) -> Result<()> {
    let zmq_ctx = zmq::Context::new();

    let mut ch1_pub = Publisher::bind(
        &zmq_ctx, &config.ch1_endpoint, Channel::WorldState.topic(),
    )?;

    let ch3_connect = Channel::VehicleState.connect_endpoint();
    let mut ch3_sub = Subscriber::connect(
        &zmq_ctx, ch3_connect, Channel::VehicleState.topic(),
    )?;

    info!("Aggregator started at {}Hz, CH1={}, CH3={}", config.rate_hz, config.ch1_endpoint, ch3_connect);

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut cycle: u64 = 0;

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // Read VehicleState from CH3 (odometry fallback)
        if let Ok(Some(vs)) = ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(1)) {
            if let Some(pose) = &vs.odometry_pose {
                if store.latest_pose.age_secs() > 0.5 {
                    store.latest_pose.store(store::types::Pose2D {
                        x: pose.x, y: pose.y, theta: pose.theta,
                    });
                    store.localization_confidence.store(0.6);
                }
            }
            if let Some(vel) = &vs.odometry_velocity {
                if store.latest_velocity.age_secs() > 0.5 {
                    store.latest_velocity.store(store::types::Twist2D {
                        linear_x: vel.linear_x, linear_y: vel.linear_y,
                        angular_z: vel.angular_z,
                    });
                }
            }
        }

        // Read latest sensor data
        let latest_camera = store.camera_buffer.pop_latest();
        let latest_lidar = store.lidar_buffer.pop_latest();
        let _latest_imu = store.imu_buffer.pop_latest();

        let pose = store.latest_pose.load().unwrap_or_default();
        let velocity = store.latest_velocity.load().unwrap_or_default();
        let loc_confidence = store.localization_confidence.load().unwrap_or(0.0);

        // Compose and publish WorldState on CH1
        let world_state = limo_proto::WorldState {
            header: Some(limo_proto::Header {
                timestamp_ns: now_ns(), sequence: cycle as u32,
                frame_id: "world".into(),
            }),
            robot_pose: Some(limo_proto::Pose2D {
                x: pose.x, y: pose.y, theta: pose.theta,
            }),
            robot_velocity: Some(limo_proto::Twist2D {
                linear_x: velocity.linear_x, linear_y: velocity.linear_y,
                angular_z: velocity.angular_z,
            }),
            detections: if latest_camera.is_some() {
                Some(limo_proto::DetectionArray { header: None, detections: vec![] })
            } else { None },
            lanes: None,
            local_map: if latest_lidar.is_some() {
                Some(limo_proto::OccupancyGrid {
                    header: None, width: 0, height: 0, resolution: 0.05,
                    origin: None, data: vec![],
                })
            } else { None },
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
        if elapsed < interval { thread::sleep(interval - elapsed); }
    }

    info!("Aggregator stopped");
    Ok(())
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH).unwrap().as_nanos() as u64
}

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
