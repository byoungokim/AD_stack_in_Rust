/// Integration tests for ZMQ pub/sub communication with Protobuf messages.
///
/// Tests verify that messages can be safely published and received
/// across threads using the limo-transport crate, simulating the
/// inter-process communication between SensPerc and Control.
use std::thread;
use std::time::Duration;

use limo_transport::subscriber::BackgroundSubscriber;
use limo_transport::{Channel, Publisher, Subscriber};

/// Helper: create a ZMQ context for testing.
fn test_ctx() -> zmq::Context {
    zmq::Context::new()
}

/// Helper: generate a unique endpoint to avoid port conflicts between tests.
fn unique_endpoint(port: u16) -> String {
    format!("tcp://127.0.0.1:{}", port)
}

// --- Test 1: VehicleState roundtrip (Control → SensPerc via CH3) ---

#[test]
fn test_vehicle_state_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15553);

    // Publisher (simulates Control process)
    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to bind publisher");

    // Subscriber (simulates SensPerc process)
    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to connect subscriber");

    // ZMQ needs a brief moment for the connection to establish
    thread::sleep(Duration::from_millis(100));

    // Create a VehicleState message
    let msg = limo_proto::VehicleState {
        header: Some(limo_proto::Header {
            timestamp_ns: 1234567890,
            sequence: 42,
            frame_id: "odom".into(),
        }),
        odometry_pose: Some(limo_proto::Pose2D {
            x: 1.5,
            y: 2.3,
            theta: 0.7,
        }),
        odometry_velocity: Some(limo_proto::Twist2D {
            linear_x: 0.5,
            linear_y: 0.0,
            angular_z: 0.1,
        }),
        steering_angle: 0.15,
        drive_mode: limo_proto::DriveMode::DriveAckermann as i32,
        battery_voltage: 12.4,
        ctrl_status: limo_proto::ControllerStatus::CtrlActive as i32,
    };

    // Publish
    publisher.publish(&msg).expect("Failed to publish");

    // Receive with timeout
    let received: limo_proto::VehicleState = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no message received");

    // Verify all fields
    let header = received.header.unwrap();
    assert_eq!(header.timestamp_ns, 1234567890);
    assert_eq!(header.sequence, 42);
    assert_eq!(header.frame_id, "odom");

    let pose = received.odometry_pose.unwrap();
    assert!((pose.x - 1.5).abs() < 1e-10);
    assert!((pose.y - 2.3).abs() < 1e-10);
    assert!((pose.theta - 0.7).abs() < 1e-10);

    let vel = received.odometry_velocity.unwrap();
    assert!((vel.linear_x - 0.5).abs() < 1e-10);
    assert!((vel.angular_z - 0.1).abs() < 1e-10);

    assert!((received.steering_angle - 0.15).abs() < 1e-6);
    assert!((received.battery_voltage - 12.4).abs() < 1e-6);
    assert_eq!(
        received.drive_mode,
        limo_proto::DriveMode::DriveAckermann as i32
    );
    assert_eq!(
        received.ctrl_status,
        limo_proto::ControllerStatus::CtrlActive as i32
    );
}

// --- Test 2: WorldState roundtrip (SensPerc → Planning via CH1) ---

#[test]
fn test_world_state_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15551);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::WorldState.topic())
        .expect("Failed to bind publisher");

    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::WorldState.topic())
        .expect("Failed to connect subscriber");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::WorldState {
        header: Some(limo_proto::Header {
            timestamp_ns: 9999,
            sequence: 1,
            frame_id: "world".into(),
        }),
        robot_pose: Some(limo_proto::Pose2D {
            x: 10.0,
            y: 20.0,
            theta: 1.57,
        }),
        robot_velocity: Some(limo_proto::Twist2D {
            linear_x: 0.3,
            linear_y: 0.0,
            angular_z: 0.0,
        }),
        detections: Some(limo_proto::DetectionArray {
            header: None,
            detections: vec![limo_proto::Detection {
                object_class: limo_proto::ObjectClass::ObjectPerson as i32,
                confidence: 0.95,
                bbox_image: None,
                position_world: Some(limo_proto::Point2D { x: 5.0, y: 3.0 }),
                distance: 4.2,
                velocity_world: None,
                radius: 0.0,
                track_id: 0,
            }],
        }),
        lanes: None,
        local_map: None,
        localization_confidence: 0.85,
    };

    publisher.publish(&msg).expect("Failed to publish");

    let received: limo_proto::WorldState = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no WorldState received");

    assert_eq!(received.header.unwrap().timestamp_ns, 9999);
    assert!((received.robot_pose.unwrap().x - 10.0).abs() < 1e-10);
    assert!((received.localization_confidence - 0.85).abs() < 1e-6);

    let dets = received.detections.unwrap();
    assert_eq!(dets.detections.len(), 1);
    assert_eq!(
        dets.detections[0].object_class,
        limo_proto::ObjectClass::ObjectPerson as i32
    );
    assert!((dets.detections[0].confidence - 0.95).abs() < 1e-6);
}

// --- Test 3: ControlCommand roundtrip (Planning → Control via CH2) ---

#[test]
fn test_control_command_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15552);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::ControlCommand.topic())
        .expect("Failed to bind publisher");

    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::ControlCommand.topic())
        .expect("Failed to connect subscriber");

    thread::sleep(Duration::from_millis(100));

    // Test velocity command
    let msg = limo_proto::ControlCommand {
        header: Some(limo_proto::Header {
            timestamp_ns: 5555,
            sequence: 10,
            frame_id: "".into(),
        }),
        source: limo_proto::PipelineSource::SourceTraditional as i32,
        command: Some(limo_proto::control_command::Command::VelocityCmd(
            limo_proto::Twist2D {
                linear_x: 0.5,
                linear_y: 0.0,
                angular_z: 0.2,
            },
        )),
        confidence: 0.99,
        emergency_stop: false,
    };

    publisher.publish(&msg).expect("Failed to publish");

    let received: limo_proto::ControlCommand = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no ControlCommand received");

    assert_eq!(received.header.unwrap().sequence, 10);
    assert!(!received.emergency_stop);
    assert!((received.confidence - 0.99).abs() < 1e-6);

    match received.command {
        Some(limo_proto::control_command::Command::VelocityCmd(twist)) => {
            assert!((twist.linear_x - 0.5).abs() < 1e-10);
            assert!((twist.angular_z - 0.2).abs() < 1e-10);
        }
        _ => panic!("Expected VelocityCmd variant"),
    }
}

// --- Test 4: Emergency stop command ---

#[test]
fn test_emergency_stop_command() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15555);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::ControlCommand.topic())
        .expect("Failed to bind publisher");

    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::ControlCommand.topic())
        .expect("Failed to connect subscriber");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::ControlCommand {
        header: Some(limo_proto::Header {
            timestamp_ns: 7777,
            sequence: 0,
            frame_id: "".into(),
        }),
        source: limo_proto::PipelineSource::SourceTraditional as i32,
        command: Some(limo_proto::control_command::Command::VelocityCmd(
            limo_proto::Twist2D {
                linear_x: 0.0,
                linear_y: 0.0,
                angular_z: 0.0,
            },
        )),
        confidence: 1.0,
        emergency_stop: true,
    };

    publisher.publish(&msg).expect("Failed to publish");

    let received: limo_proto::ControlCommand = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no e-stop command received");

    assert!(received.emergency_stop);
}

// --- Test 5: Multiple messages in sequence ---

#[test]
fn test_multiple_messages_sequence() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15556);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to bind publisher");

    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to connect subscriber");

    thread::sleep(Duration::from_millis(100));

    let num_messages = 50;

    for i in 0..num_messages {
        let msg = limo_proto::VehicleState {
            header: Some(limo_proto::Header {
                timestamp_ns: i as u64 * 1000,
                sequence: i,
                frame_id: "odom".into(),
            }),
            odometry_pose: Some(limo_proto::Pose2D {
                x: i as f64 * 0.1,
                y: 0.0,
                theta: 0.0,
            }),
            ..Default::default()
        };
        publisher.publish(&msg).expect("Failed to publish");
    }

    // Receive all messages and verify ordering
    let mut received_count = 0;
    let mut last_seq = 0u32;

    loop {
        match subscriber.recv::<limo_proto::VehicleState>(Duration::from_millis(500)) {
            Ok(Some(msg)) => {
                let seq = msg.header.unwrap().sequence;
                if received_count > 0 {
                    assert!(
                        seq > last_seq,
                        "Messages out of order: {} <= {}",
                        seq,
                        last_seq
                    );
                }
                last_seq = seq;
                received_count += 1;
            }
            Ok(None) => break, // timeout, done
            Err(e) => panic!("Recv error: {:#}", e),
        }
    }

    assert!(
        received_count > 0,
        "Expected to receive at least some messages, got 0"
    );
    assert_eq!(publisher.msg_count(), num_messages as u64);
}

// --- Test 6: Subscriber timeout (no publisher) ---

#[test]
fn test_subscriber_timeout() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15557);

    // Only create subscriber, no publisher
    // Need to bind something or connect will succeed but no messages
    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to connect subscriber");

    let start = std::time::Instant::now();
    let result = subscriber.recv::<limo_proto::VehicleState>(Duration::from_millis(200));

    assert!(start.elapsed() >= Duration::from_millis(150));
    assert!(result.unwrap().is_none(), "Expected timeout (None)");
}

// --- Test 7: Topic filtering (wrong topic receives nothing) ---

#[test]
fn test_topic_filtering() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15558);

    // Publisher sends on VehicleState topic
    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to bind publisher");

    // Subscriber listens on WorldState topic — should NOT receive
    let mut wrong_sub = Subscriber::connect(&ctx, &endpoint, Channel::WorldState.topic())
        .expect("Failed to connect subscriber");

    thread::sleep(Duration::from_millis(100));

    let msg = limo_proto::VehicleState {
        header: Some(limo_proto::Header {
            timestamp_ns: 1111,
            sequence: 1,
            frame_id: "test".into(),
        }),
        ..Default::default()
    };

    publisher.publish(&msg).expect("Failed to publish");

    // Should timeout because topic doesn't match
    let result = wrong_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(300));
    assert!(
        result.unwrap().is_none(),
        "Should not receive on wrong topic"
    );
}

// --- Test 8: Background subscriber ---

#[test]
fn test_background_subscriber() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15559);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::VehicleState.topic())
        .expect("Failed to bind publisher");

    let bg_sub = BackgroundSubscriber::<limo_proto::VehicleState>::start(
        &ctx,
        &endpoint,
        Channel::VehicleState.topic(),
        16,
    )
    .expect("Failed to start background subscriber");

    thread::sleep(Duration::from_millis(100));

    // Publish several messages
    for i in 0..10 {
        let msg = limo_proto::VehicleState {
            header: Some(limo_proto::Header {
                timestamp_ns: i * 100,
                sequence: i as u32,
                frame_id: "test".into(),
            }),
            battery_voltage: 12.0 + i as f32 * 0.1,
            ..Default::default()
        };
        publisher.publish(&msg).expect("Failed to publish");
        thread::sleep(Duration::from_millis(10));
    }

    // Wait for messages to arrive
    thread::sleep(Duration::from_millis(200));

    // Get latest — should be the most recent message
    let latest = bg_sub.try_recv_latest();
    assert!(
        latest.is_some(),
        "Expected at least one message from background subscriber"
    );

    let latest = latest.unwrap();
    // The latest should have a high sequence number
    assert!(latest.header.unwrap().sequence > 0);
}

// --- Test 9: Heartbeat roundtrip ---

#[test]
fn test_heartbeat_roundtrip() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15560);

    let mut publisher = Publisher::bind(&ctx, &endpoint, Channel::Heartbeat.topic())
        .expect("Failed to bind publisher");

    let mut subscriber = Subscriber::connect(&ctx, &endpoint, Channel::Heartbeat.topic())
        .expect("Failed to connect subscriber");

    thread::sleep(Duration::from_millis(100));

    let hb = limo_proto::Heartbeat {
        process_name: "control".into(),
        timestamp_ns: 123456,
        status: limo_proto::ProcessStatus::ProcessNominal as i32,
        sequence: 7,
    };

    publisher.publish(&hb).expect("Failed to publish heartbeat");

    let received: limo_proto::Heartbeat = subscriber
        .recv(Duration::from_secs(2))
        .expect("Recv error")
        .expect("Timeout — no heartbeat received");

    assert_eq!(received.process_name, "control");
    assert_eq!(received.timestamp_ns, 123456);
    assert_eq!(
        received.status,
        limo_proto::ProcessStatus::ProcessNominal as i32
    );
    assert_eq!(received.sequence, 7);
}

// --- Test 10: Cross-thread pub/sub (simulates inter-process) ---

#[test]
fn test_cross_thread_pubsub() {
    let ctx = test_ctx();
    let endpoint = unique_endpoint(15561);

    let pub_ctx = ctx.clone();
    let sub_ctx = ctx.clone();
    let ep = endpoint.clone();

    // Publisher thread (simulates Control process)
    let pub_handle = thread::spawn(move || {
        let mut publisher = Publisher::bind(&pub_ctx, &ep, Channel::VehicleState.topic())
            .expect("Failed to bind publisher");

        // Wait for subscriber
        thread::sleep(Duration::from_millis(200));

        for i in 0..20 {
            let msg = limo_proto::VehicleState {
                header: Some(limo_proto::Header {
                    timestamp_ns: i * 50_000_000, // 50ms intervals
                    sequence: i as u32,
                    frame_id: "odom".into(),
                }),
                odometry_pose: Some(limo_proto::Pose2D {
                    x: i as f64 * 0.05,
                    y: 0.0,
                    theta: 0.0,
                }),
                battery_voltage: 12.0,
                ..Default::default()
            };
            publisher.publish(&msg).expect("Failed to publish");
            thread::sleep(Duration::from_millis(50)); // 20Hz
        }
    });

    // Subscriber thread (simulates SensPerc process)
    let sub_handle = thread::spawn(move || {
        let mut subscriber =
            Subscriber::connect(&sub_ctx, &endpoint, Channel::VehicleState.topic())
                .expect("Failed to connect subscriber");

        let mut count = 0;
        let start = std::time::Instant::now();

        while start.elapsed() < Duration::from_secs(3) {
            match subscriber.recv::<limo_proto::VehicleState>(Duration::from_millis(200)) {
                Ok(Some(msg)) => {
                    let pose = msg.odometry_pose.unwrap();
                    assert!(pose.x >= 0.0); // x should be non-negative
                    count += 1;
                }
                Ok(None) => {
                    if count > 0 {
                        break; // publisher done
                    }
                }
                Err(_) => break,
            }
        }

        count
    });

    pub_handle.join().unwrap();
    let received = sub_handle.join().unwrap();

    assert!(
        received >= 10,
        "Expected at least 10 messages in cross-thread test, got {}",
        received,
    );
}
