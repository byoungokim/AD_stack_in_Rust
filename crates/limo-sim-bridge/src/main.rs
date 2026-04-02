/// Limo Drive — Isaac Sim Bridge
///
/// Bidirectional ZMQ bridge between the Limo Drive stack and Isaac Sim.
///
/// Isaac Sim side (extension/script):
///   - Publishes sensor data → CH5 (SimSensors)
///   - Publishes vehicle state → CH6 (SimVehicleState)
///   - Subscribes control commands ← CH7 (SimControl)
///
/// This bridge can run as:
///   1. A standalone process alongside Isaac Sim
///   2. A test harness generating synthetic sim data (--dummy mode)
///
/// In production, Isaac Sim's Python extension connects directly to
/// these ZMQ ports. This binary provides a dummy simulator for testing
/// the full pipeline without Isaac Sim.
mod config;
mod dummy_sim;

use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

use config::{load_config, SimBridgeConfig};
use limo_transport::{Channel, Publisher, Subscriber};

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: Isaac Sim Bridge Starting ===");

    let config_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "config/sim_bridge.yaml".into());
    let dummy_mode = std::env::args().any(|a| a == "--dummy");

    let config = load_config(&config_path).unwrap_or_else(|e| {
        warn!("Config not found ({}), using defaults", e);
        SimBridgeConfig::default()
    });

    ctrlc_handler();

    let zmq_ctx = zmq::Context::new();

    if dummy_mode {
        info!("Running in DUMMY mode (no Isaac Sim required)");
        dummy_sim::run_dummy_sim(&zmq_ctx, &config, &SHUTDOWN)?;
    } else {
        info!("Running in BRIDGE mode (waiting for Isaac Sim connections)");
        run_bridge(&zmq_ctx, &config)?;
    }

    info!("=== Isaac Sim Bridge Stopped ===");
    Ok(())
}

/// Bridge mode: just bind the ZMQ ports and let Isaac Sim connect.
/// Also subscribes to CH7 (SimControl) and logs received commands.
fn run_bridge(ctx: &zmq::Context, config: &SimBridgeConfig) -> Result<()> {
    // CH7 subscriber: receive control commands from our stack
    let mut ch7_sub = Subscriber::connect(
        ctx,
        &config.ch7_endpoint_connect,
        Channel::SimControl.topic(),
    )?;

    info!(
        "Bridge ready — Isaac Sim should publish on ports {} (sensors) and {} (state)",
        config.ch5_port, config.ch6_port
    );
    info!(
        "Bridge subscribing control commands on {}",
        config.ch7_endpoint_connect
    );

    while !SHUTDOWN.load(Ordering::Acquire) {
        // Log incoming control commands
        match ch7_sub.recv::<limo_proto::SimControlCommand>(Duration::from_millis(100)) {
            Ok(Some(cmd)) => {
                tracing::debug!(
                    "CH7 SimControl: v={:.2} w={:.2} steer={:.2} estop={}",
                    cmd.linear_velocity,
                    cmd.angular_velocity,
                    cmd.steering_angle,
                    cmd.emergency_stop,
                );
            }
            Ok(None) => {}
            Err(e) => {
                tracing::debug!("CH7 recv error: {:#}", e);
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
