/// Control loop core: one cycle of the safety-gated command path.
///
/// Extracted from `main.rs` so the loop body is testable with injected
/// commands and wall-clock time. Cycle order is safety-relevant:
///
/// 1. Read chassis feedback, update odometry and the measured speed.
/// 2. Classify the newest (drained) CH2 command: age-gate it, then only
///    ACTIONABLE commands feed the watchdog (VelocityCmd or an explicit
///    e-stop request). TrajectoryCmd is rejected with a rate-limited
///    warning — an unhandled command type must not keep the watchdog fed
///    while the chassis holds its last velocity.
/// 3. Run the watchdog check (command timeout, peer heartbeats).
/// 4. Actuate exactly ONE command: the e-stop/deceleration override
///    replaces the planning command — a stale or late-queued message can
///    never reach the motors while the e-stop latch is active.
use std::time::{Duration, Instant};

use tracing::warn;

use limo_hal::{ChassisFeedback, MotorCommand, VehicleController};

use crate::config::ControlConfig;
use crate::kinematics::{self, KinematicsEngine, OdomPose, OdomVelocity};
use crate::watchdog::Watchdog;

/// Minimum interval between repeated warnings of the same kind.
const WARN_INTERVAL: Duration = Duration::from_secs(1);

/// What one control cycle did — for state publishing, logging, and tests.
pub struct CycleOutput {
    pub odom_pose: OdomPose,
    pub odom_vel: OdomVelocity,
    pub feedback: ChassisFeedback,
    pub estop_active: bool,
    /// The single command actuated this cycle. `None` means the HAL
    /// `emergency_stop()` path was invoked instead of a velocity command.
    pub actuated: Option<MotorCommand>,
}

pub struct ControlLoop {
    kinematics: KinematicsEngine,
    watchdog: Watchdog,
    max_command_age_ns: u64,
    /// Last command that was actuated; re-sent when no fresh command has
    /// arrived yet (the watchdog bounds this hold to command_timeout_ms).
    last_actuated: MotorCommand,
    last_traj_warn: Option<Instant>,
    last_stale_warn: Option<Instant>,
}

impl ControlLoop {
    pub fn new(
        config: &ControlConfig,
        estop_flag: std::sync::Arc<std::sync::atomic::AtomicBool>,
    ) -> Self {
        Self {
            kinematics: KinematicsEngine::new(config.kinematics.clone()),
            watchdog: Watchdog::new(config.watchdog.clone(), estop_flag),
            max_command_age_ns: config.watchdog.max_command_age_ms * 1_000_000,
            last_actuated: MotorCommand::default(),
            last_traj_warn: None,
            last_stale_warn: None,
        }
    }

    /// Forward a peer heartbeat observation to the watchdog.
    pub fn notify_heartbeat(&mut self, peer: &str) {
        self.watchdog.notify_heartbeat(peer);
    }

    pub fn watchdog(&self) -> &Watchdog {
        &self.watchdog
    }

    /// Run one control cycle.
    ///
    /// `latest_cmd` is the newest ControlCommand drained from CH2 this
    /// cycle (older queued messages must be discarded by the caller).
    /// `now_ns` is wall-clock time, injected for testability; the command
    /// age gate compares it against `header.timestamp_ns`.
    pub fn run_cycle(
        &mut self,
        latest_cmd: Option<limo_proto::ControlCommand>,
        controller: &mut dyn VehicleController,
        dt: f64,
        now_ns: u64,
    ) -> CycleOutput {
        // --- 1. Feedback → odometry → measured speed ---
        let feedback = controller.recv_feedback().unwrap_or_default();
        let (odom_pose, odom_vel) = self
            .kinematics
            .update_odometry(&kinematics::to_kinematics_feedback(&feedback), dt);
        self.watchdog.update_measured_speed(odom_vel.linear_x);

        // --- 2. Classify the newest command (age gate + actionability) ---
        let planned = latest_cmd.and_then(|cmd| self.classify_command(cmd, now_ns));

        // --- 3. Watchdog check (command timeout, peer heartbeats) ---
        self.watchdog.check();

        // --- 4. Actuate exactly one command ---
        let actuated = if let Some(reason) = self.watchdog.latched_reason() {
            if reason.is_command_timeout() {
                // Controlled deceleration ramp (open-loop from the speed
                // captured at e-stop entry); once at zero, assert the HAL
                // e-stop to hold the vehicle stopped.
                let v = self.watchdog.deceleration_velocity(dt);
                if v != 0.0 {
                    let cmd = MotorCommand {
                        linear_vel: v,
                        angular_vel: 0.0,
                    };
                    let _ = controller.send_command(&cmd);
                    self.last_actuated = cmd.clone();
                    Some(cmd)
                } else {
                    self.last_actuated = MotorCommand::default();
                    let _ = controller.emergency_stop();
                    None
                }
            } else {
                // ExplicitRequest / PeerDead: immediate hard stop via the
                // platform's strongest mechanism.
                self.last_actuated = MotorCommand::default();
                let _ = controller.emergency_stop();
                None
            }
        } else {
            let cmd = match planned {
                Some(motor) => self.kinematics.clamp_command(&motor),
                // No fresh command yet: re-send the last actuated command.
                // The watchdog bounds this hold to command_timeout_ms.
                None => self.last_actuated.clone(),
            };
            let _ = controller.send_command(&cmd);
            self.last_actuated = cmd.clone();
            Some(cmd)
        };

        CycleOutput {
            odom_pose,
            odom_vel,
            feedback,
            estop_active: self.watchdog.is_estop_active(),
            actuated,
        }
    }

    /// Apply the age gate and actionability rules to a received command.
    /// Returns the velocity command to actuate, if any. Only actionable,
    /// fresh commands feed the watchdog.
    fn classify_command(
        &mut self,
        cmd: limo_proto::ControlCommand,
        now_ns: u64,
    ) -> Option<MotorCommand> {
        // Age gate: a command without a header, or older than the
        // threshold, is treated as NOT received — it must not reset the
        // watchdog and must not be actuated.
        let fresh = cmd
            .header
            .as_ref()
            .is_some_and(|h| now_ns.saturating_sub(h.timestamp_ns) <= self.max_command_age_ns);
        if !fresh {
            if Self::should_warn(&mut self.last_stale_warn) {
                let age_ms = cmd
                    .header
                    .as_ref()
                    .map(|h| now_ns.saturating_sub(h.timestamp_ns) / 1_000_000)
                    .unwrap_or(u64::MAX);
                warn!(
                    "Ignoring stale ControlCommand (age {}ms > {}ms) — not fed to watchdog",
                    age_ms,
                    self.max_command_age_ns / 1_000_000
                );
            }
            return None;
        }

        if cmd.emergency_stop {
            // Explicit e-stop request: actionable, feeds the watchdog and
            // latches the ExplicitRequest reason.
            self.watchdog.notify_command_received(true);
            return None;
        }

        match cmd.command {
            Some(limo_proto::control_command::Command::VelocityCmd(twist)) => {
                self.watchdog.notify_command_received(false);
                Some(MotorCommand {
                    linear_vel: twist.linear_x,
                    angular_vel: twist.angular_z,
                })
            }
            Some(limo_proto::control_command::Command::TrajectoryCmd(_)) => {
                // Not actionable yet (tracker wiring pending): must NOT
                // feed the watchdog, or an unhandled trajectory stream
                // would hold the chassis at its last velocity forever.
                if Self::should_warn(&mut self.last_traj_warn) {
                    warn!(
                        "TrajectoryCmd received but tracker is not wired — \
                         command rejected (does not feed the watchdog)"
                    );
                }
                None
            }
            None => None,
        }
    }

    fn should_warn(last: &mut Option<Instant>) -> bool {
        let now = Instant::now();
        if last.is_none_or(|t| now.duration_since(t) >= WARN_INTERVAL) {
            *last = Some(now);
            true
        } else {
            false
        }
    }
}

/// Build the CH3 VehicleState message for one cycle's output.
pub fn build_vehicle_state(
    out: &CycleOutput,
    sequence: u32,
    now_ns: u64,
) -> limo_proto::VehicleState {
    limo_proto::VehicleState {
        header: Some(limo_proto::Header {
            timestamp_ns: now_ns,
            sequence,
            frame_id: "odom".into(),
        }),
        odometry_pose: Some(limo_proto::Pose2D {
            x: out.odom_pose.x,
            y: out.odom_pose.y,
            theta: out.odom_pose.theta,
        }),
        odometry_velocity: Some(limo_proto::Twist2D {
            linear_x: out.odom_vel.linear_x,
            linear_y: 0.0,
            angular_z: out.odom_vel.angular_z,
        }),
        steering_angle: out.feedback.steering_angle,
        drive_mode: limo_proto::DriveMode::DriveAckermann as i32,
        battery_voltage: out.feedback.battery_voltage,
        ctrl_status: if out.estop_active {
            limo_proto::ControllerStatus::CtrlEstop as i32
        } else {
            limo_proto::ControllerStatus::CtrlActive as i32
        },
    }
}

/// Wall-clock time in nanoseconds since the Unix epoch.
pub fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64
}
