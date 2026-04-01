/// Limo Drive — Sensing & Perception Process (Process 1)
///
/// Owns all sensor hardware and perception computation.
/// Publishes aggregated WorldState on CH1 (ZMQ PUB tcp:5551).
///
/// Architecture:
///   CameraDriver (30Hz) ──┐
///   LidarDriver  (10Hz) ──┤── SensorStore ── Aggregator (10Hz) ──> CH1: WorldState
///   ImuDriver   (100Hz) ──┘
mod config;
mod drivers;
mod proto;
mod store;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{error, info, warn};
use tracing_subscriber::EnvFilter;

use config::{load_config, SensPercConfig};
use drivers::camera::CameraDriver;
use drivers::imu::ImuDriver;
use drivers::lidar::LidarDriver;
use store::SensorStore;

/// Global shutdown flag, shared across all threads.
static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: SensPerc Process Starting ===");

    // Load config
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

    // Register signal handler for graceful shutdown
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

    // Start aggregator loop (publishes WorldState on CH1)
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

/// Aggregator loop: reads latest sensor data and publishes WorldState on CH1.
///
/// For now, this is a stub that logs what it would publish.
/// Full implementation will serialize WorldState via Protobuf and publish via ZMQ.
fn aggregator_loop(
    store: &Arc<SensorStore>,
    config: &config::AggregatorConfig,
) -> Result<()> {
    info!("Aggregator started at {}Hz, CH1={}", config.rate_hz, config.ch1_endpoint);

    // TODO: Initialize ZMQ publisher for CH1 (WorldState)
    // let ctx = zmq::Context::new();
    // let publisher = ctx.socket(zmq::PUB)?;
    // publisher.bind(&config.ch1_endpoint)?;

    let interval = Duration::from_secs_f64(1.0 / config.rate_hz as f64);
    let mut cycle: u64 = 0;

    while !SHUTDOWN.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // Read latest data from all slots
        let _latest_camera = store.camera_buffer.pop_latest();
        let _latest_lidar = store.lidar_buffer.pop_latest();
        let _latest_imu = store.imu_buffer.pop_latest();
        let _latest_fused = store.latest_fused_state.load();

        // TODO: Compose WorldState protobuf message from latest data
        // TODO: Serialize and publish on CH1

        cycle += 1;
        if cycle % (config.rate_hz as u64 * 10) == 0 {
            let stats = store.stats();
            info!(
                "Aggregator cycle {}: cam={}, lidar={}, imu={}",
                cycle, stats.camera_frames, stats.lidar_scans, stats.imu_readings
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

/// Register Ctrl+C handler for graceful shutdown.
fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
