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

    ctrlc_handler();

    // Create shared sensor store
    let store = Arc::new(SensorStore::new());

    // Start sensor drivers
    let mut camera = CameraDriver::new(Arc::clone(&store), config.camera.clone());
    let mut lidar = LidarDriver::new(Arc::clone(&store), config.lidar.clone());
    let mut imu = ImuDriver::new(Arc::clone(&store), config.imu.clone());

    camera.start()?;
    lidar.start()?;
    imu.start()?;

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
    camera.stop();
    lidar.stop();
    imu.stop();
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
        match ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(1)) {
            Ok(Some(vs)) => {
                // Store for sensor fusion
                // TODO: feed into EKF when implemented
                debug!(
                    "CH3 VehicleState received: pose=({:.2}, {:.2}), bat={:.1}V",
                    vs.odometry_pose.as_ref().map(|p| p.x).unwrap_or(0.0),
                    vs.odometry_pose.as_ref().map(|p| p.y).unwrap_or(0.0),
                    vs.battery_voltage,
                );
            }
            Ok(None) => {} // timeout
            Err(e) => {
                debug!("CH3 recv error: {:#}", e);
            }
        }

        // --- Read latest sensor data ---
        let latest_camera = store.camera_buffer.pop_latest();
        let latest_lidar = store.lidar_buffer.pop_latest();
        let _latest_imu = store.imu_buffer.pop_latest();
        let _latest_fused = store.latest_fused_state.load();

        // --- Compose and publish WorldState on CH1 ---
        let world_state = limo_proto::WorldState {
            header: Some(limo_proto::Header {
                timestamp_ns: now_ns(),
                sequence: cycle as u32,
                frame_id: "world".into(),
            }),
            robot_pose: Some(limo_proto::Pose2D {
                x: 0.0, // TODO: from SLAM/localization
                y: 0.0,
                theta: 0.0,
            }),
            robot_velocity: Some(limo_proto::Twist2D {
                linear_x: 0.0, // TODO: from sensor fusion
                linear_y: 0.0,
                angular_z: 0.0,
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
            localization_confidence: 0.0, // TODO: from SLAM
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

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
