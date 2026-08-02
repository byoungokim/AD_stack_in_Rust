//! Safety-chain integration tests for the Control process.
//!
//! Each test publishes ControlCommand messages on a real ZMQ CH2 socket,
//! drives the extracted control loop (`ControlLoop::run_cycle`) exactly
//! like `main.rs` does, publishes the resulting VehicleState on a real
//! ZMQ CH3 socket, and asserts on what comes back — plus what reached
//! the (scripted) motors.
//!
//! Determinism: wall-clock command timestamps are injected per publish,
//! chassis feedback is scripted, and the only real-time dependencies are
//! the watchdog's Instant-based command timeout (tests use a short
//! configured timeout with generous bounded waits) and ZMQ delivery
//! (bounded handshakes/waits). Each test uses its own tcp ports.

use std::time::{Duration, Instant};

use limo_control::config::ControlConfig;
use limo_control::control_loop::{build_vehicle_state, now_ns, ControlLoop, CycleOutput};
use limo_control::kinematics::KinematicsConfig;
use limo_control::watchdog::WatchdogConfig;

use limo_hal::{ChassisFeedback, MotorCommand, VehicleController};
use limo_transport::{Channel, Publisher, Subscriber};

const DT: f64 = 0.05; // injected cycle time for the decel ramp profile
const WHEEL_RADIUS: f64 = 0.045; // matches KinematicsConfig::default()

// ======================== Scripted controller ========================

/// VehicleController with fully scripted feedback velocity, so tests
/// control exactly what "measured speed" the watchdog sees.
struct ScriptedController {
    /// Velocity the chassis reports (both wheels, straight line).
    feedback_vel: f64,
    last_cmd: MotorCommand,
    estop_calls: u64,
}

impl ScriptedController {
    fn new() -> Self {
        Self {
            feedback_vel: 0.0,
            last_cmd: MotorCommand::default(),
            estop_calls: 0,
        }
    }
}

impl VehicleController for ScriptedController {
    fn start(&mut self) -> anyhow::Result<()> {
        Ok(())
    }

    fn stop(&mut self) {}

    fn send_command(&mut self, cmd: &MotorCommand) -> anyhow::Result<()> {
        self.last_cmd = cmd.clone();
        Ok(())
    }

    fn emergency_stop(&mut self) -> anyhow::Result<()> {
        self.last_cmd = MotorCommand::default();
        self.estop_calls += 1;
        Ok(())
    }

    fn recv_feedback(&mut self) -> Option<ChassisFeedback> {
        let rpm = (self.feedback_vel * 60.0 / (2.0 * std::f64::consts::PI * WHEEL_RADIUS)) as f32;
        Some(ChassisFeedback {
            left_wheel_rpm: rpm,
            right_wheel_rpm: rpm,
            steering_angle: 0.0,
            battery_voltage: 12.4,
            error_code: 0,
            timestamp_ns: now_ns(),
        })
    }

    fn name(&self) -> &str {
        "scripted"
    }
}

// ======================== Test config ========================

fn test_config() -> ControlConfig {
    ControlConfig {
        watchdog: WatchdogConfig {
            command_timeout_ms: 100,
            deceleration_rate: 2.0,    // 0.1 m/s per 0.05s cycle
            heartbeat_dead_ms: 10_000, // keep PeerDead out of these tests
            max_command_age_ms: 300,
            stopped_speed_threshold: 0.05,
        },
        kinematics: KinematicsConfig {
            // Mirrors the SIM value in config/control.yaml; the code
            // default stays at the safe real-hardware 1.0 m/s.
            max_linear_vel: 2.5,
            ..KinematicsConfig::default()
        },
        ..ControlConfig::default()
    }
}

// ======================== ZMQ harness ========================

/// Test-side CH2 publisher (playing Planning) + control-side CH2
/// subscriber, and control-side CH3 publisher + test-side subscriber.
struct Harness {
    ch2_pub: Publisher,
    ch2_sub: Subscriber,
    ch3_pub: Publisher,
    ch3_sub: Subscriber,
    ctrl_loop: ControlLoop,
    seq: u32,
    cycle: u32,
}

impl Harness {
    /// `port` and `port + 1` must be unique per test (tests run in
    /// parallel threads within one binary).
    fn new(port: u16) -> Self {
        let ctx = zmq::Context::new();
        let ch2_endpoint_bind = format!("tcp://127.0.0.1:{}", port);
        let ch3_endpoint_bind = format!("tcp://127.0.0.1:{}", port + 1);

        let mut ch2_pub =
            Publisher::bind(&ctx, &ch2_endpoint_bind, Channel::ControlCommand.topic())
                .expect("bind CH2 pub");
        let mut ch2_sub =
            Subscriber::connect(&ctx, &ch2_endpoint_bind, Channel::ControlCommand.topic())
                .expect("connect CH2 sub");
        let mut ch3_pub = Publisher::bind(&ctx, &ch3_endpoint_bind, Channel::VehicleState.topic())
            .expect("bind CH3 pub");
        let mut ch3_sub =
            Subscriber::connect(&ctx, &ch3_endpoint_bind, Channel::VehicleState.topic())
                .expect("connect CH3 sub");

        // Slow-joiner handshake on both channels: probe until delivery
        // works, then drain. The CH2 probe uses an ancient timestamp so
        // even if a test later saw it, the age gate would reject it.
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let probe = limo_proto::ControlCommand {
                header: Some(limo_proto::Header {
                    timestamp_ns: 1,
                    sequence: 0,
                    frame_id: "probe".into(),
                }),
                ..Default::default()
            };
            ch2_pub.publish(&probe).expect("publish CH2 probe");
            if let Ok(Some(_)) =
                ch2_sub.recv::<limo_proto::ControlCommand>(Duration::from_millis(50))
            {
                break;
            }
            assert!(Instant::now() < deadline, "CH2 handshake timed out");
        }
        while let Ok(Some(_)) = ch2_sub.recv::<limo_proto::ControlCommand>(Duration::ZERO) {}

        loop {
            let probe = limo_proto::VehicleState::default();
            ch3_pub.publish(&probe).expect("publish CH3 probe");
            if let Ok(Some(_)) = ch3_sub.recv::<limo_proto::VehicleState>(Duration::from_millis(50))
            {
                break;
            }
            assert!(Instant::now() < deadline, "CH3 handshake timed out");
        }
        while let Ok(Some(_)) = ch3_sub.recv::<limo_proto::VehicleState>(Duration::ZERO) {}

        // Build the loop AFTER the handshake so the watchdog's command
        // timer starts at test scenario time.
        let config = test_config();
        let estop_flag = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let ctrl_loop = ControlLoop::new(&config, estop_flag);

        Self {
            ch2_pub,
            ch2_sub,
            ch3_pub,
            ch3_sub,
            ctrl_loop,
            seq: 0,
            cycle: 0,
        }
    }

    fn publish_cmd(
        &mut self,
        timestamp_ns: u64,
        emergency_stop: bool,
        command: Option<limo_proto::control_command::Command>,
    ) {
        self.seq += 1;
        let msg = limo_proto::ControlCommand {
            header: Some(limo_proto::Header {
                timestamp_ns,
                sequence: self.seq,
                frame_id: "".into(),
            }),
            source: 0,
            command,
            confidence: 1.0,
            emergency_stop,
        };
        self.ch2_pub.publish(&msg).expect("publish ControlCommand");
    }

    fn publish_velocity(&mut self, timestamp_ns: u64, linear: f64, angular: f64) {
        self.publish_cmd(
            timestamp_ns,
            false,
            Some(limo_proto::control_command::Command::VelocityCmd(
                limo_proto::Twist2D {
                    linear_x: linear,
                    linear_y: 0.0,
                    angular_z: angular,
                },
            )),
        );
    }

    fn publish_trajectory(&mut self, timestamp_ns: u64) {
        self.publish_cmd(
            timestamp_ns,
            false,
            Some(limo_proto::control_command::Command::TrajectoryCmd(
                limo_proto::Trajectory {
                    header: None,
                    points: vec![],
                },
            )),
        );
    }

    fn publish_estop(&mut self, timestamp_ns: u64) {
        self.publish_cmd(timestamp_ns, true, None);
    }

    /// Run one control cycle after waiting (bounded) for the most
    /// recently published command to arrive, draining to the newest —
    /// like main.rs. Sequence-matched so late handshake probes (seq 0)
    /// can never be mistaken for the command under test.
    fn step_expect_cmd(
        &mut self,
        controller: &mut dyn VehicleController,
    ) -> (CycleOutput, limo_proto::VehicleState) {
        let target = self.seq;
        let deadline = Instant::now() + Duration::from_secs(2);
        let latest = loop {
            if let Ok(Some(cmd)) = self
                .ch2_sub
                .recv::<limo_proto::ControlCommand>(Duration::from_millis(50))
            {
                let seq = cmd.header.as_ref().map(|h| h.sequence).unwrap_or(0);
                if seq == target {
                    break Some(cmd);
                }
                // Older message (e.g. handshake probe): drained/discarded.
            }
            assert!(
                Instant::now() < deadline,
                "expected ControlCommand seq {} on CH2 but it never arrived",
                target
            );
        };
        self.run_and_publish(latest, controller)
    }

    /// Run one control cycle with whatever is queued (possibly nothing).
    fn step_no_wait(
        &mut self,
        controller: &mut dyn VehicleController,
    ) -> (CycleOutput, limo_proto::VehicleState) {
        let mut latest = None;
        while let Ok(Some(cmd)) = self
            .ch2_sub
            .recv::<limo_proto::ControlCommand>(Duration::ZERO)
        {
            latest = Some(cmd);
        }
        self.run_and_publish(latest, controller)
    }

    fn run_and_publish(
        &mut self,
        latest: Option<limo_proto::ControlCommand>,
        controller: &mut dyn VehicleController,
    ) -> (CycleOutput, limo_proto::VehicleState) {
        let out = self.ctrl_loop.run_cycle(latest, controller, DT, now_ns());
        self.cycle += 1;
        let state = build_vehicle_state(&out, self.cycle, now_ns());
        self.ch3_pub.publish(&state).expect("publish VehicleState");
        // Sequence-matched: skip any late handshake probes (no header).
        let deadline = Instant::now() + Duration::from_secs(2);
        let received = loop {
            if let Ok(Some(s)) = self
                .ch3_sub
                .recv::<limo_proto::VehicleState>(Duration::from_millis(50))
            {
                if s.header.as_ref().map(|h| h.sequence) == Some(self.cycle) {
                    break s;
                }
            }
            assert!(
                Instant::now() < deadline,
                "expected VehicleState seq {} on CH3 but it never arrived",
                self.cycle
            );
        };
        (out, received)
    }
}

fn assert_estop_status(state: &limo_proto::VehicleState, estop: bool) {
    let expected = if estop {
        limo_proto::ControllerStatus::CtrlEstop as i32
    } else {
        limo_proto::ControllerStatus::CtrlActive as i32
    };
    assert_eq!(state.ctrl_status, expected, "CH3 ctrl_status mismatch");
}

// ======================== Tests ========================

/// (a) Command timeout ⇒ e-stop latches and the velocity ramps to zero
/// on the open-loop deceleration profile, then the HAL e-stop engages.
#[test]
fn command_timeout_triggers_estop_and_decel_ramp() {
    let mut h = Harness::new(46101);
    let mut ctrl = ScriptedController::new();

    // Fresh velocity command flows through to the motors.
    h.publish_velocity(now_ns(), 0.5, 0.0);
    let (out, state) = h.step_expect_cmd(&mut ctrl);
    assert!(!out.estop_active);
    assert_estop_status(&state, false);
    let sent = out.actuated.expect("velocity command should be actuated");
    assert!((sent.linear_vel - 0.5).abs() < 1e-9);
    assert!((ctrl.last_cmd.linear_vel - 0.5).abs() < 1e-9);

    // Chassis reports the vehicle moving at 0.5 m/s.
    ctrl.feedback_vel = 0.5;

    // Stop publishing: after command_timeout_ms the watchdog latches.
    std::thread::sleep(Duration::from_millis(130));
    let (out, state) = h.step_no_wait(&mut ctrl);
    assert!(
        out.estop_active,
        "e-stop should latch after command timeout"
    );
    assert!(h
        .ctrl_loop
        .watchdog()
        .latched_reason()
        .expect("latched reason")
        .is_command_timeout());
    assert_estop_status(&state, true);

    // Deceleration ramp: strictly decreasing, reaches zero, then the
    // HAL emergency_stop path engages (actuated == None).
    let mut prev = out
        .actuated
        .expect("ramp should send a decel command")
        .linear_vel;
    assert!(prev > 0.0 && prev < 0.5, "ramp starts below measured speed");
    let mut reached_zero = false;
    for _ in 0..20 {
        let (out, _state) = h.step_no_wait(&mut ctrl);
        assert!(out.estop_active);
        match out.actuated {
            Some(cmd) => {
                assert!(cmd.linear_vel < prev, "ramp must decrease monotonically");
                assert_eq!(cmd.angular_vel, 0.0);
                prev = cmd.linear_vel;
            }
            None => {
                reached_zero = true;
                break;
            }
        }
    }
    assert!(reached_zero, "ramp never completed");
    assert!(ctrl.estop_calls > 0, "HAL emergency_stop not invoked");
    assert_eq!(ctrl.last_cmd.linear_vel, 0.0);

    // Fresh commands resume ⇒ CommandTimeout auto-clears.
    ctrl.feedback_vel = 0.0;
    h.publish_velocity(now_ns(), 0.3, 0.0);
    let (out, state) = h.step_expect_cmd(&mut ctrl);
    assert!(!out.estop_active, "timeout e-stop should auto-clear");
    assert_estop_status(&state, false);
    assert!((out.actuated.unwrap().linear_vel - 0.3).abs() < 1e-9);
}

/// (b) Explicit e-stop latches; a stale queued command must neither
/// clear the latch nor move the motors.
#[test]
fn explicit_estop_latches_against_stale_command() {
    let mut h = Harness::new(46111);
    let mut ctrl = ScriptedController::new();
    ctrl.feedback_vel = 0.5; // vehicle is moving

    h.publish_estop(now_ns());
    let (out, state) = h.step_expect_cmd(&mut ctrl);
    assert!(out.estop_active);
    assert!(
        out.actuated.is_none(),
        "e-stop must replace the command path"
    );
    assert_estop_status(&state, true);
    assert_eq!(ctrl.estop_calls, 1);
    assert_eq!(ctrl.last_cmd.linear_vel, 0.0);

    // A stale command (1s old wall clock) arrives from the queue.
    let stale_ts = now_ns() - 1_000_000_000;
    h.publish_velocity(stale_ts, 0.8, 0.0);
    let (out, state) = h.step_expect_cmd(&mut ctrl);
    assert!(out.estop_active, "stale command must not clear the latch");
    assert!(out.actuated.is_none(), "stale command must not be actuated");
    assert_estop_status(&state, true);
    assert_eq!(
        ctrl.last_cmd.linear_vel, 0.0,
        "stale command reached the motors"
    );
    assert_eq!(ctrl.estop_calls, 2, "HAL e-stop asserted every cycle");
}

/// (c) Explicit e-stop clears only after a fresh emergency_stop=false
/// command arrives AND the measured velocity shows the robot stopped.
#[test]
fn explicit_estop_clears_only_after_fresh_release_and_stop() {
    let mut h = Harness::new(46121);
    let mut ctrl = ScriptedController::new();
    ctrl.feedback_vel = 0.5;

    // Latch the explicit e-stop while moving.
    h.publish_estop(now_ns());
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    assert!(out.estop_active);

    // Fresh release command, but the vehicle is still moving ⇒ held.
    h.publish_velocity(now_ns(), 0.2, 0.0);
    let (out, state) = h.step_expect_cmd(&mut ctrl);
    assert!(
        out.estop_active,
        "release must be refused while the vehicle is moving"
    );
    assert!(out.actuated.is_none());
    assert_eq!(ctrl.last_cmd.linear_vel, 0.0);
    assert_estop_status(&state, true);

    // Vehicle actually stopped + fresh release ⇒ cleared, command flows.
    ctrl.feedback_vel = 0.0;
    h.publish_velocity(now_ns(), 0.2, 0.0);
    let (out, state) = h.step_expect_cmd(&mut ctrl);
    assert!(!out.estop_active, "release should clear once stopped");
    assert_estop_status(&state, false);
    let sent = out.actuated.expect("released command should be actuated");
    assert!((sent.linear_vel - 0.2).abs() < 1e-9);
    assert!((ctrl.last_cmd.linear_vel - 0.2).abs() < 1e-9);
}

/// (d) TrajectoryCmd is not actionable: it must not feed the watchdog,
/// so a trajectory-only stream still ends in a command-timeout e-stop.
#[test]
fn trajectory_cmd_does_not_feed_watchdog() {
    let mut h = Harness::new(46131);
    let mut ctrl = ScriptedController::new();

    // Establish a normal velocity command first.
    h.publish_velocity(now_ns(), 0.3, 0.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    assert!(!out.estop_active);
    ctrl.feedback_vel = 0.3;

    // Stream fresh TrajectoryCmds well past the command timeout. They
    // are rejected (rate-limited warn) and must not reset the watchdog.
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(180) {
        h.publish_trajectory(now_ns());
        let (out, _) = h.step_expect_cmd(&mut ctrl);
        if let Some(cmd) = out.actuated {
            // Until the timeout fires the loop holds the last actuated
            // velocity — the trajectory itself is never actuated. After
            // it fires, only the decel ramp (angular 0) is sent.
            assert!(cmd.linear_vel <= 0.3 + 1e-9);
        }
        std::thread::sleep(Duration::from_millis(25));
    }

    let (out, state) = h.step_no_wait(&mut ctrl);
    assert!(
        out.estop_active,
        "watchdog must time out despite fresh TrajectoryCmds"
    );
    assert!(h
        .ctrl_loop
        .watchdog()
        .latched_reason()
        .expect("latched reason")
        .is_command_timeout());
    assert_estop_status(&state, true);
}

/// (e) The linear clamp honors the CONFIGURED max_linear_vel: 2.2 m/s
/// flows through untouched with the sim limit (2.5), and an over-limit
/// command is clamped to exactly the configured value — not the old
/// hardcoded 1.0.
#[test]
fn clamp_honors_configured_max_linear_vel_end_to_end() {
    let mut h = Harness::new(46141);
    let mut ctrl = ScriptedController::new();

    // 2.2 m/s is inside the configured 2.5 limit → reaches the motors as-is.
    h.publish_velocity(now_ns(), 2.2, 0.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    let sent = out.actuated.expect("velocity command should be actuated");
    assert!(
        (sent.linear_vel - 2.2).abs() < 1e-9,
        "2.2 m/s must not be capped (got {})",
        sent.linear_vel
    );
    assert!((ctrl.last_cmd.linear_vel - 2.2).abs() < 1e-9);

    // 3.0 m/s exceeds the configured limit → clamped to 2.5.
    h.publish_velocity(now_ns(), 3.0, 0.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    let sent = out.actuated.expect("velocity command should be actuated");
    assert!(
        (sent.linear_vel - 2.5).abs() < 1e-9,
        "over-limit command must clamp to configured max (got {})",
        sent.linear_vel
    );

    // Reverse direction clamps symmetrically.
    h.publish_velocity(now_ns(), -3.0, 0.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    assert!((out.actuated.unwrap().linear_vel + 2.5).abs() < 1e-9);
}

/// (f) The Ackermann angular clamp scales with the commanded speed:
/// tight at 0.2 m/s, wide at 2.2 m/s, and floored (not zero) at rest.
#[test]
fn angular_clamp_scales_with_commanded_speed() {
    let mut h = Harness::new(46151);
    let mut ctrl = ScriptedController::new();

    let kin = KinematicsConfig::default();
    let k = kin.max_steering_angle.tan() / kin.wheelbase; // rad/s per m/s

    // Slow: 0.2 m/s with a huge angular request → capped at 0.2·k (~0.52),
    // far below the old fixed-reference limit of 1.0·k (~2.6).
    h.publish_velocity(now_ns(), 0.2, 100.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    let sent = out.actuated.expect("actuated");
    assert!((sent.angular_vel - 0.2 * k).abs() < 1e-9);

    // Fast: 2.2 m/s → limit widens to 2.2·k (~5.7 rad/s); an executable
    // 4 rad/s request must pass untouched (the old limit capped it ~2.6).
    h.publish_velocity(now_ns(), 2.2, 4.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    let sent = out.actuated.expect("actuated");
    assert!(
        (sent.angular_vel - 4.0).abs() < 1e-9,
        "4 rad/s at 2.2 m/s is executable and must not be capped (got {})",
        sent.angular_vel
    );
    h.publish_velocity(now_ns(), 2.2, 100.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    assert!((out.actuated.unwrap().angular_vel - 2.2 * k).abs() < 1e-9);

    // Near-zero speed: the floor (0.1 m/s reference) keeps a nonzero
    // steering limit so recovery arcs from rest are not blocked.
    h.publish_velocity(now_ns(), 0.0, 100.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    let sent = out.actuated.expect("actuated");
    assert!((sent.angular_vel - 0.1 * k).abs() < 1e-9);
    assert!(sent.angular_vel > 0.0, "floor must keep steering authority");
}

/// (g) Command-timeout deceleration from 2.2 m/s: the open-loop ramp at
/// the configured 2.0 m/s² reaches zero in ~1.1 s of simulated time
/// (22 cycles at dt=0.05), then the HAL e-stop engages.
#[test]
fn timeout_decel_from_2p2_reaches_zero_in_about_1p1s() {
    let mut h = Harness::new(46161);
    let mut ctrl = ScriptedController::new();

    // Drive at 2.2 m/s with matching chassis feedback.
    h.publish_velocity(now_ns(), 2.2, 0.0);
    let (out, _) = h.step_expect_cmd(&mut ctrl);
    assert!((out.actuated.unwrap().linear_vel - 2.2).abs() < 1e-9);
    ctrl.feedback_vel = 2.2;

    // Let the watchdog time out.
    std::thread::sleep(Duration::from_millis(130));

    // The ramp is seeded from the MEASURED 2.2 m/s (speed-agnostic: no
    // hardcoded start speed) and steps down 0.1 m/s per 0.05 s cycle.
    let mut prev = 2.2;
    let mut ramp_cycles = 0u32;
    let mut reached_zero = false;
    for _ in 0..40 {
        let (out, _) = h.step_no_wait(&mut ctrl);
        assert!(out.estop_active, "e-stop must stay latched during ramp");
        ramp_cycles += 1;
        match out.actuated {
            Some(cmd) => {
                assert!(
                    cmd.linear_vel < prev && cmd.linear_vel > 0.0,
                    "ramp must decrease monotonically toward zero"
                );
                assert_eq!(cmd.angular_vel, 0.0);
                prev = cmd.linear_vel;
            }
            None => {
                reached_zero = true;
                break;
            }
        }
    }
    assert!(reached_zero, "decel ramp from 2.2 m/s never completed");
    assert!(ctrl.estop_calls > 0, "HAL emergency_stop not engaged");

    // 2.2 / 2.0 = 1.1 s → 22 cycles at dt=0.05. Allow one cycle of
    // profile rounding slack.
    let simulated_stop_time = f64::from(ramp_cycles) * DT;
    assert!(
        (1.0..=1.2).contains(&simulated_stop_time),
        "stop from 2.2 m/s took {:.2}s simulated (expected ≈1.1s)",
        simulated_stop_time
    );
}
