/// Limo Drive — Scenario Manager
///
/// Sends navigation goals to the Planning process via CH8.
/// Receives scenario status feedback via CH9.
///
/// Usage:
///   limo_scenario --preset square_patrol    # Run a built-in preset
///   limo_scenario --file scenario.yaml      # Load from YAML
///   limo_scenario --goal 3.0 2.0 0.0        # Single goal (x y theta)
///   limo_scenario --list                    # List available presets
mod scenarios;

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::Result;
use tracing::{info, warn};
use tracing_subscriber::EnvFilter;

use limo_transport::{Channel, Publisher, Subscriber};
use scenarios::presets;

static SHUTDOWN: AtomicBool = AtomicBool::new(false);

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    info!("=== Limo Drive: Scenario Manager ===");

    let args: Vec<String> = std::env::args().collect();

    // Parse command
    if args.iter().any(|a| a == "--list") {
        print_presets();
        return Ok(());
    }

    let scenario = if let Some(pos) = args.iter().position(|a| a == "--preset") {
        let name = args
            .get(pos + 1)
            .map(|s| s.as_str())
            .unwrap_or("straight_line");
        match name {
            "straight_line" => presets::straight_line(),
            "square_patrol" => presets::square_patrol(),
            "slalom" => presets::slalom(),
            "parking" => presets::parking(),
            "figure_eight" => presets::figure_eight(),
            _ => {
                warn!("Unknown preset '{}', using straight_line", name);
                presets::straight_line()
            }
        }
    } else if let Some(pos) = args.iter().position(|a| a == "--file") {
        let path = args.get(pos + 1).expect("--file requires a path");
        scenarios::load_scenario(path)?
    } else if let Some(pos) = args.iter().position(|a| a == "--goal") {
        let x: f64 = args
            .get(pos + 1)
            .expect("--goal requires x y theta")
            .parse()?;
        let y: f64 = args
            .get(pos + 2)
            .expect("--goal requires x y theta")
            .parse()?;
        let theta: f64 = args
            .get(pos + 3)
            .expect("--goal requires x y theta")
            .parse()?;
        scenarios::ScenarioDef {
            name: "single_goal".into(),
            scenario_type: "waypoint".into(),
            waypoints: vec![scenarios::WaypointDef {
                x,
                y,
                theta,
                tolerance: 0.15,
                speed: 0.5,
                label: "goal".into(),
            }],
            speed_limit: 0.5,
        }
    } else {
        info!("No scenario specified. Usage:");
        info!("  limo_scenario --preset <name>");
        info!("  limo_scenario --file <path.yaml>");
        info!("  limo_scenario --goal <x> <y> <theta>");
        info!("  limo_scenario --list");
        info!("");
        info!("Using default: straight_line");
        presets::straight_line()
    };

    info!(
        "Scenario: '{}' (type={}, {} waypoints, speed_limit={:.1} m/s)",
        scenario.name,
        scenario.scenario_type,
        scenario.waypoints.len(),
        scenario.speed_limit
    );

    for (i, wp) in scenario.waypoints.iter().enumerate() {
        info!(
            "  wp[{}] '{}': ({:.2}, {:.2}, {:.1}°) tol={:.2}m speed={:.1}m/s",
            i,
            wp.label,
            wp.x,
            wp.y,
            wp.theta.to_degrees(),
            wp.tolerance,
            wp.speed
        );
    }

    ctrlc_handler();

    // --- ZMQ setup ---
    let zmq_ctx = zmq::Context::new();

    let mut ch8_pub = Publisher::bind(
        &zmq_ctx,
        Channel::ScenarioCommand.bind_endpoint(),
        Channel::ScenarioCommand.topic(),
    )?;

    let mut ch9_sub = Subscriber::connect(
        &zmq_ctx,
        Channel::ScenarioStatus.connect_endpoint(),
        Channel::ScenarioStatus.topic(),
    )?;

    info!(
        "ZMQ: pub CH8={}, sub CH9={}",
        Channel::ScenarioCommand.bind_endpoint(),
        Channel::ScenarioStatus.connect_endpoint()
    );

    // Wait for Planning to connect
    std::thread::sleep(Duration::from_millis(500));

    // Send scenario command
    let cmd = scenario.to_command(0);
    ch8_pub.publish(&cmd)?;
    info!("Scenario command sent!");

    // Monitor status
    let start = Instant::now();
    let mut last_log = Instant::now();

    while !SHUTDOWN.load(Ordering::Acquire) {
        match ch9_sub.recv::<limo_proto::ScenarioStatus>(Duration::from_millis(500)) {
            Ok(Some(status)) => {
                if last_log.elapsed() >= Duration::from_secs(2) {
                    info!(
                        "Status: wp {}/{} dist={:.2}m label='{}' reached={} complete={}",
                        status.current_waypoint_index,
                        status.total_waypoints,
                        status.distance_to_goal,
                        status.active_label,
                        status.goal_reached,
                        status.scenario_complete,
                    );
                    last_log = Instant::now();
                }

                if status.scenario_complete {
                    info!(
                        "Scenario '{}' completed in {:.1}s!",
                        scenario.name,
                        start.elapsed().as_secs_f64()
                    );
                    break;
                }
            }
            Ok(None) => {} // timeout, re-send periodically
            Err(e) => {
                warn!("CH9 recv error: {:#}", e);
            }
        }

        // Re-send command periodically in case Planning wasn't ready
        if start.elapsed().as_secs().is_multiple_of(5) && ch8_pub.msg_count() < 10 {
            let cmd = scenario.to_command(ch8_pub.msg_count() as u32);
            let _ = ch8_pub.publish(&cmd);
        }
    }

    info!("=== Scenario Manager Stopped ===");
    Ok(())
}

fn print_presets() {
    println!("Available scenario presets:");
    println!("  straight_line  - Go 3m forward");
    println!("  square_patrol  - Drive in a 4m square loop");
    println!("  slalom         - Weave through 5 waypoints");
    println!("  parking        - Navigate to a parking spot");
    println!("  figure_eight   - Figure-8 pattern");
}

fn ctrlc_handler() {
    let _ = ctrlc::set_handler(move || {
        info!("Received Ctrl+C, shutting down...");
        SHUTDOWN.store(true, Ordering::Release);
    });
}
