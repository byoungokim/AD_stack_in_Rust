/// Integration tests for Isaac Sim bridge ZMQ channels (CH5, CH6, CH7).
///
/// Verifies that sim sensor data, sim vehicle state, and sim control
/// commands flow correctly through the ZMQ transport layer.
use std::thread;
use std::time::Duration;

use limo_transport::{Channel, Publisher, Subscriber};

fn test_ctx() -> zmq::Context {
    zmq::Context::new()
}

fn unique_endpoint(port: u16) -> String {
    format!("tcp://127.0.0.1:{}", port)
}

// --- Test 1: SimSensorData roundtrip (Isaac Sim → SensPerc via CH5) ---

#[test]
fn test_sim_sensor_data_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(16560);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::SimSensors.topic())
        .expect("Failed to bind CH5 publisher");
    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::SimSensors.topic())
        .expect("Failed to connect CH5 subscriber");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::SimSensorData {
        header: Some(limo_proto::Header {
            timestamp_ns: 1000000,
            sequence: 1,
            frame_id: "sim".into(),
        }),
        camera_image: vec![128u8; 640 * 480 * 3], // dummy RGB image
        camera_width: 640,
        camera_height: 480,
        camera_encoding: "rgb8".into(),
        lidar_scan: Some(limo_proto::LaserScan {
            header: None,
            angle_min: 0.0,
            angle_max: std::f32::consts::TAU,
            angle_increment: std::f32::consts::TAU / 360.0,
            range_min: 0.1,
            range_max: 12.0,
            ranges: vec![3.0; 360],
            intensities: vec![200.0; 360],
        }),
        imu: Some(limo_proto::ImuReading {
            header: None,
            linear_acceleration: Some(limo_proto::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 9.81,
            }),
            angular_velocity: Some(limo_proto::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            orientation_euler: Some(limo_proto::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
        }),
        ground_truth_pose: Some(limo_proto::Pose2D {
            x: 1.0,
            y: 2.0,
            theta: 0.5,
        }),
        ground_truth_velocity: Some(limo_proto::Twist2D {
            linear_x: 0.3,
            linear_y: 0.0,
            angular_z: 0.1,
        }),
    };

    publisher
        .publish(&msg)
        .expect("Failed to publish SimSensorData");

    let received: limo_proto::SimSensorData = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no SimSensorData received");

    // Verify fields
    assert_eq!(received.camera_width, 640);
    assert_eq!(received.camera_height, 480);
    assert_eq!(received.camera_encoding, "rgb8");
    assert_eq!(received.camera_image.len(), 640 * 480 * 3);

    let lidar = received.lidar_scan.unwrap();
    assert_eq!(lidar.ranges.len(), 360);

    let gt_pose = received.ground_truth_pose.unwrap();
    assert!((gt_pose.x - 1.0).abs() < 1e-10);
    assert!((gt_pose.y - 2.0).abs() < 1e-10);

    let imu = received.imu.unwrap();
    let accel = imu.linear_acceleration.unwrap();
    assert!((accel.z - 9.81).abs() < 1e-6);
}

// --- Test 2: SimVehicleState roundtrip (Isaac Sim → Control via CH6) ---

#[test]
fn test_sim_vehicle_state_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(16561);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::SimVehicleState.topic())
        .expect("Failed to bind CH6 publisher");
    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::SimVehicleState.topic())
        .expect("Failed to connect CH6 subscriber");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::SimVehicleState {
        header: Some(limo_proto::Header {
            timestamp_ns: 2000000,
            sequence: 5,
            frame_id: "sim".into(),
        }),
        pose: Some(limo_proto::Pose2D {
            x: 3.0,
            y: 1.0,
            theta: 1.2,
        }),
        velocity: Some(limo_proto::Twist2D {
            linear_x: 0.5,
            linear_y: 0.0,
            angular_z: 0.2,
        }),
        steering_angle: 0.15,
        battery_voltage: 12.6,
        drive_mode: 1, // Ackermann
        collision_detected: false,
    };

    publisher.publish(&msg).expect("Failed to publish");

    let received: limo_proto::SimVehicleState = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no SimVehicleState received");

    let pose = received.pose.unwrap();
    assert!((pose.x - 3.0).abs() < 1e-10);
    assert!((received.steering_angle - 0.15).abs() < 1e-6);
    assert!((received.battery_voltage - 12.6).abs() < 1e-6);
    assert!(!received.collision_detected);
}

// --- Test 3: SimControlCommand roundtrip (Control → Isaac Sim via CH7) ---

#[test]
fn test_sim_control_command_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(16562);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::SimControl.topic())
        .expect("Failed to bind CH7 publisher");
    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::SimControl.topic())
        .expect("Failed to connect CH7 subscriber");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::SimControlCommand {
        header: Some(limo_proto::Header {
            timestamp_ns: 3000000,
            sequence: 10,
            frame_id: "".into(),
        }),
        linear_velocity: 0.5,
        angular_velocity: 0.2,
        steering_angle: 0.15,
        emergency_stop: false,
    };

    publisher.publish(&msg).expect("Failed to publish");

    let received: limo_proto::SimControlCommand = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no SimControlCommand received");

    assert!((received.linear_velocity - 0.5).abs() < 1e-6);
    assert!((received.angular_velocity - 0.2).abs() < 1e-6);
    assert!((received.steering_angle - 0.15).abs() < 1e-6);
    assert!(!received.emergency_stop);
}

// --- Test 4: SimControlCommand emergency stop ---

#[test]
fn test_sim_emergency_stop() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(16563);

    let mut publisher =
        Publisher::bind(&ctx, &endpoint, Channel::SimControl.topic()).expect("Failed to bind");
    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::SimControl.topic())
        .expect("Failed to connect");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::SimControlCommand {
        header: Some(limo_proto::Header {
            timestamp_ns: 4000000,
            sequence: 0,
            frame_id: "".into(),
        }),
        linear_velocity: 0.0,
        angular_velocity: 0.0,
        steering_angle: 0.0,
        emergency_stop: true,
    };

    publisher.publish(&msg).expect("Failed to publish");

    let received: limo_proto::SimControlCommand = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout");

    assert!(received.emergency_stop);
    assert_eq!(received.linear_velocity, 0.0);
}

// --- Test 5: Full sim loop (CH6 → Control logic → CH7) simulated ---

#[test]
fn test_sim_full_loop() {
    let ctx = test_ctx();
    let ch6_endpoint = unique_endpoint(16564);
    let ch7_endpoint = unique_endpoint(16565);

    // Simulate Isaac Sim publishing vehicle state on CH6
    let mut sim_state_pub = Publisher::bind(&ctx, &ch6_endpoint, Channel::SimVehicleState.topic())
        .expect("Failed to bind CH6");

    // Simulate Isaac Sim subscribing control commands on CH7
    let mut sim_ctrl_sub = Subscriber::connect(&ctx, &ch7_endpoint, Channel::SimControl.topic())
        .expect("Failed to connect CH7");

    // Simulate Control process publishing commands on CH7
    let mut ctrl_cmd_pub = Publisher::bind(&ctx, &ch7_endpoint, Channel::SimControl.topic())
        .expect("Failed to bind CH7");

    thread::sleep(Duration::from_millis(100));

    // Isaac Sim publishes state
    let state = limo_proto::SimVehicleState {
        header: Some(limo_proto::Header {
            timestamp_ns: 5000000,
            sequence: 1,
            frame_id: "sim".into(),
        }),
        pose: Some(limo_proto::Pose2D {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }),
        velocity: Some(limo_proto::Twist2D {
            linear_x: 0.0,
            linear_y: 0.0,
            angular_z: 0.0,
        }),
        steering_angle: 0.0,
        battery_voltage: 12.6,
        drive_mode: 1,
        collision_detected: false,
    };
    sim_state_pub
        .publish(&state)
        .expect("Failed to publish state");

    // Control process publishes a command back
    let cmd = limo_proto::SimControlCommand {
        header: Some(limo_proto::Header {
            timestamp_ns: 5000000,
            sequence: 1,
            frame_id: "".into(),
        }),
        linear_velocity: 0.3,
        angular_velocity: 0.0,
        steering_angle: 0.0,
        emergency_stop: false,
    };
    // ZMQ slow-joiner: sim_ctrl_sub connected BEFORE ctrl_cmd_pub bound, so
    // the subscriber sits in a reconnect-retry cycle whose completion no
    // fixed settle sleep can guarantee on a loaded runner — a single publish
    // is silently dropped until the subscription lands (measured ~20% local
    // flake, intermittent CI failures). Republish until received, bounded.
    let mut received: Option<limo_proto::SimControlCommand> = None;
    for _ in 0..50 {
        ctrl_cmd_pub.publish(&cmd).expect("Failed to publish cmd");
        if let Some(msg) = sim_ctrl_sub
            .recv(Duration::from_millis(100))
            .expect("Recv error")
        {
            received = Some(msg);
            break;
        }
    }
    let received = received.expect("Timeout — sim didn't receive control command");

    assert!((received.linear_velocity - 0.3).abs() < 1e-6);
}

// --- Test 6: Sim channel topic isolation ---

#[test]
fn test_sim_channel_isolation() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(16566);

    // Publish on SimSensors topic
    let mut publisher =
        Publisher::bind(&ctx, &endpoint, Channel::SimSensors.topic()).expect("Failed to bind");

    // Subscribe on SimControl topic — should NOT receive
    let mut wrong_sub = Subscriber::connect(&ctx, &endpoint, Channel::SimControl.topic())
        .expect("Failed to connect");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::SimSensorData {
        header: Some(limo_proto::Header {
            timestamp_ns: 1111,
            sequence: 1,
            frame_id: "sim".into(),
        }),
        ..Default::default()
    };

    publisher.publish(&msg).expect("Failed to publish");

    let result = wrong_sub.recv::<limo_proto::SimControlCommand>(Duration::from_millis(300));
    assert!(
        result.unwrap().is_none(),
        "Should not receive on wrong sim topic"
    );
}
