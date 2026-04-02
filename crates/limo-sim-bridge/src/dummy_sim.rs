/// Dummy simulator: generates synthetic sensor data and vehicle state
/// for testing the full pipeline without Isaac Sim.
///
/// Simulates a Limo Pro driving in a simple environment:
/// - Camera: gradient test pattern
/// - LiDAR: circular room with one obstacle
/// - IMU: stationary with noise
/// - Vehicle state: simple kinematics from received control commands
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::Result;
use tracing::{debug, info};

use crate::config::SimBridgeConfig;
use limo_transport::{Channel, Publisher, Subscriber};

/// Simulated vehicle state.
struct SimState {
    x: f64,
    y: f64,
    theta: f64,
    linear_vel: f64,
    angular_vel: f64,
    steering_angle: f32,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            x: 0.0, y: 0.0, theta: 0.0,
            linear_vel: 0.0, angular_vel: 0.0,
            steering_angle: 0.0,
        }
    }
}

pub fn run_dummy_sim(
    ctx: &zmq::Context,
    config: &SimBridgeConfig,
    shutdown: &'static AtomicBool,
) -> Result<()> {
    let state = Arc::new(Mutex::new(SimState::default()));

    // CH5 publisher: SimSensors → SensPerc
    let ch5_bind = format!("tcp://*:{}", config.ch5_port);
    let mut ch5_pub = Publisher::bind(ctx, &ch5_bind, Channel::SimSensors.topic())?;

    // CH6 publisher: SimVehicleState → Control
    let ch6_bind = format!("tcp://*:{}", config.ch6_port);
    let mut ch6_pub = Publisher::bind(ctx, &ch6_bind, Channel::SimVehicleState.topic())?;

    // CH7 subscriber: SimControl from Control
    let mut ch7_sub = Subscriber::connect(
        ctx,
        &config.ch7_endpoint_connect,
        Channel::SimControl.topic(),
    )?;

    info!(
        "Dummy sim: sensors@{}Hz on port {}, state@{}Hz on port {}",
        config.dummy.sensor_rate_hz, config.ch5_port,
        config.dummy.state_rate_hz, config.ch6_port,
    );

    let sensor_interval = Duration::from_secs_f64(1.0 / config.dummy.sensor_rate_hz as f64);
    let state_interval = Duration::from_secs_f64(1.0 / config.dummy.state_rate_hz as f64);
    let physics_dt = 1.0 / config.dummy.state_rate_hz as f64;

    let mut last_sensor = Instant::now();
    let mut last_state = Instant::now();
    let mut sequence: u32 = 0;
    let start = Instant::now();

    while !shutdown.load(Ordering::Acquire) {
        let now = Instant::now();

        // --- Receive control commands (non-blocking) ---
        if let Ok(Some(cmd)) = ch7_sub.recv::<limo_proto::SimControlCommand>(Duration::from_millis(1)) {
            let mut s = state.lock().unwrap();
            if cmd.emergency_stop {
                s.linear_vel = 0.0;
                s.angular_vel = 0.0;
            } else {
                s.linear_vel = cmd.linear_velocity as f64;
                s.angular_vel = cmd.angular_velocity as f64;
                s.steering_angle = cmd.steering_angle;
            }
        }

        // --- Physics step ---
        {
            let mut s = state.lock().unwrap();
            s.x += s.linear_vel * s.theta.cos() * physics_dt;
            s.y += s.linear_vel * s.theta.sin() * physics_dt;
            s.theta += s.angular_vel * physics_dt;
            // Normalize theta
            while s.theta > std::f64::consts::PI { s.theta -= 2.0 * std::f64::consts::PI; }
            while s.theta < -std::f64::consts::PI { s.theta += 2.0 * std::f64::consts::PI; }
        }

        // --- Publish sensor data (CH5) ---
        if now.duration_since(last_sensor) >= sensor_interval {
            let s = state.lock().unwrap();
            let t = start.elapsed().as_secs_f64();

            let sensor_data = limo_proto::SimSensorData {
                header: Some(limo_proto::Header {
                    timestamp_ns: now_ns(),
                    sequence,
                    frame_id: "sim".into(),
                }),
                camera_image: generate_dummy_image(
                    config.dummy.image_width,
                    config.dummy.image_height,
                    sequence,
                ),
                camera_width: config.dummy.image_width,
                camera_height: config.dummy.image_height,
                camera_encoding: "rgb8".into(),
                lidar_scan: Some(generate_dummy_lidar(config.dummy.lidar_num_points, t)),
                imu: Some(generate_dummy_imu(t)),
                ground_truth_pose: Some(limo_proto::Pose2D {
                    x: s.x, y: s.y, theta: s.theta,
                }),
                ground_truth_velocity: Some(limo_proto::Twist2D {
                    linear_x: s.linear_vel,
                    linear_y: 0.0,
                    angular_z: s.angular_vel,
                }),
            };

            let _ = ch5_pub.publish(&sensor_data);
            last_sensor = now;
            sequence += 1;
        }

        // --- Publish vehicle state (CH6) ---
        if now.duration_since(last_state) >= state_interval {
            let s = state.lock().unwrap();

            let vehicle_state = limo_proto::SimVehicleState {
                header: Some(limo_proto::Header {
                    timestamp_ns: now_ns(),
                    sequence,
                    frame_id: "sim".into(),
                }),
                pose: Some(limo_proto::Pose2D {
                    x: s.x, y: s.y, theta: s.theta,
                }),
                velocity: Some(limo_proto::Twist2D {
                    linear_x: s.linear_vel,
                    linear_y: 0.0,
                    angular_z: s.angular_vel,
                }),
                steering_angle: s.steering_angle,
                battery_voltage: 12.6, // simulated full
                drive_mode: limo_proto::DriveMode::DriveAckermann as i32 as u32,
                collision_detected: false,
            };

            let _ = ch6_pub.publish(&vehicle_state);
            last_state = now;

            if sequence % (config.dummy.state_rate_hz * 5) == 0 {
                debug!(
                    "DummySim: pose=({:.2}, {:.2}, {:.1}°) vel=({:.2}, {:.2})",
                    s.x, s.y, s.theta.to_degrees(), s.linear_vel, s.angular_vel
                );
            }
        }

        thread::sleep(Duration::from_millis(1));
    }

    Ok(())
}

fn generate_dummy_image(width: u32, height: u32, frame: u32) -> Vec<u8> {
    let size = (width * height * 3) as usize;
    let mut data = vec![0u8; size];
    let offset = (frame as usize * 3) % 256;
    for (i, pixel) in data.iter_mut().enumerate() {
        *pixel = ((i + offset) % 256) as u8;
    }
    data
}

fn generate_dummy_lidar(num_points: u32, t: f64) -> limo_proto::LaserScan {
    let angle_increment = std::f32::consts::TAU / num_points as f32;
    let mut ranges = Vec::with_capacity(num_points as usize);
    let mut intensities = Vec::with_capacity(num_points as usize);

    for i in 0..num_points {
        let angle = i as f32 * angle_increment;
        let base = 4.0_f32;
        let bump = if (angle - 1.5).abs() < 0.3 { -2.0 } else { 0.0 };
        let noise = ((t as f32 * 2.0 + i as f32 * 0.05).sin()) * 0.03;
        ranges.push((base + bump + noise).max(0.1));
        intensities.push(200.0);
    }

    limo_proto::LaserScan {
        header: None,
        angle_min: 0.0,
        angle_max: std::f32::consts::TAU,
        angle_increment,
        range_min: 0.1,
        range_max: 12.0,
        ranges,
        intensities,
    }
}

fn generate_dummy_imu(t: f64) -> limo_proto::ImuReading {
    limo_proto::ImuReading {
        header: None,
        linear_acceleration: Some(limo_proto::Vector3 {
            x: 0.02 * (t * 5.0).sin(),
            y: 0.01 * (t * 7.0).cos(),
            z: 9.81 + 0.01 * (t * 3.0).sin(),
        }),
        angular_velocity: Some(limo_proto::Vector3 {
            x: 0.001 * (t * 2.0).sin(),
            y: 0.001 * (t * 3.0).cos(),
            z: 0.0,
        }),
        orientation_euler: Some(limo_proto::Vector3 {
            x: 0.01 * (t * 0.5).sin(),
            y: 0.005 * (t * 0.3).cos(),
            z: 0.0,
        }),
    }
}

fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
