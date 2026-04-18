//! Integration test for SimZmqVehicleController wire contract.
//!
//! Verifies that send_command(MotorCommand) produces a SimControlCommand on CH7
//! with steering_angle computed via the Ackermann bicycle model — i.e. the
//! steering value lands in the right protobuf field, not just in the unit test.

use std::time::Duration;

use limo_hal::sim_zmq::{SimAckermannConfig, SimZmqVehicleController};
use limo_hal::{MotorCommand, VehicleController};
use limo_transport::{Channel, Subscriber};

// One combined test — CH7 port is global, can't bind twice in parallel.
#[test]
fn send_command_publishes_correct_steering_on_ch7() {
    let ctx = zmq::Context::new();

    let mut ctrl = SimZmqVehicleController::with_kinematics(SimAckermannConfig {
        wheelbase: 0.2,
        max_steering_angle: 0.48,
    });
    ctrl.start().expect("start controller");

    let mut sub = Subscriber::connect(
        &ctx,
        Channel::SimControl.connect_endpoint(),
        Channel::SimControl.topic(),
    )
    .expect("connect CH7 subscriber");

    // Slow-joiner: wait for SUB→PUB subscription handshake.
    std::thread::sleep(Duration::from_millis(200));

    // Case 1: non-zero velocity → Ackermann steering = atan(ω·L / v).
    ctrl.send_command(&MotorCommand { linear_vel: 1.0, angular_vel: 0.5 })
        .expect("send_command #1");
    let msg1: limo_proto::SimControlCommand = sub
        .recv(Duration::from_secs(2))
        .expect("recv error")
        .expect("timeout — no CH7 message (case 1)");

    let expected = (0.5_f32 * 0.2 / 1.0).atan();
    assert!(
        (msg1.steering_angle - expected).abs() < 1e-3,
        "wire steering: got {}, expected ≈ {}",
        msg1.steering_angle, expected
    );
    assert!((msg1.linear_velocity - 1.0).abs() < 1e-6);
    assert!((msg1.angular_velocity - 0.5).abs() < 1e-6);
    assert!(!msg1.emergency_stop);

    // Case 2: zero linear velocity → zero steering (no divide-by-zero).
    ctrl.send_command(&MotorCommand { linear_vel: 0.0, angular_vel: 1.0 })
        .expect("send_command #2");
    let msg2: limo_proto::SimControlCommand = sub
        .recv(Duration::from_secs(2))
        .expect("recv error")
        .expect("timeout — no CH7 message (case 2)");

    assert_eq!(msg2.steering_angle, 0.0);

    // Sequence counter increments between sends.
    let seq1 = msg1.header.as_ref().unwrap().sequence;
    let seq2 = msg2.header.as_ref().unwrap().sequence;
    assert!(seq2 > seq1, "sequence did not increment: {} → {}", seq1, seq2);

    ctrl.stop();
}
