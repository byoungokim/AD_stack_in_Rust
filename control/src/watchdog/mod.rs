/// Watchdog and emergency stop handler.
///
/// Monitors:
/// - Freshness of ControlCommand messages from Planning (CH2)
/// - Heartbeats from peer processes (CH0)
///
/// Triggers autonomous emergency stop when:
/// - No valid ControlCommand received for `command_timeout_ms`
/// - A critical peer (planning) is detected as dead
/// - Planning explicitly requests one (ControlCommand.emergency_stop)
///
/// The e-stop is LATCHED with the reason that triggered it, and each
/// reason has its own clear rule:
/// - `CommandTimeout`: auto-clears when fresh valid commands resume.
/// - `ExplicitRequest`: clears only when a fresh command with
///   `emergency_stop=false` arrives AND the measured |linear velocity|
///   shows the vehicle has actually stopped.
/// - `PeerDead`: clears only when the peer heartbeat is healthy again
///   AND fresh commands are arriving.
///
/// Staleness is the caller's contract: only commands that passed the
/// command-age gate may reach `notify_command_received`, so a single
/// stale queued message can never clear the latch.
///
/// The watchdog is the last line of software defense before the chassis
/// firmware timeout (~500ms) and the physical E-stop button.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use serde::Deserialize;
use tracing::{error, info, warn};

#[derive(Debug, Clone, Deserialize)]
pub struct WatchdogConfig {
    #[serde(default = "default_command_timeout_ms")]
    pub command_timeout_ms: u64,
    #[serde(default = "default_deceleration_rate")]
    pub deceleration_rate: f64, // m/s^2
    #[serde(default = "default_heartbeat_dead_ms")]
    pub heartbeat_dead_ms: u64,
    /// A ControlCommand whose header timestamp is older than this is
    /// treated as NOT received: it neither feeds the watchdog nor gets
    /// actuated. Both sides use wall clock today.
    #[serde(default = "default_max_command_age_ms")]
    pub max_command_age_ms: u64,
    /// |linear velocity| below which the vehicle counts as stopped —
    /// required before an explicit e-stop may be released.
    #[serde(default = "default_stopped_speed_threshold")]
    pub stopped_speed_threshold: f64, // m/s
}

fn default_command_timeout_ms() -> u64 {
    200
}
fn default_deceleration_rate() -> f64 {
    0.5
}
fn default_heartbeat_dead_ms() -> u64 {
    1000
}
fn default_max_command_age_ms() -> u64 {
    300
}
fn default_stopped_speed_threshold() -> f64 {
    0.05
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            command_timeout_ms: default_command_timeout_ms(),
            deceleration_rate: default_deceleration_rate(),
            heartbeat_dead_ms: default_heartbeat_dead_ms(),
            max_command_age_ms: default_max_command_age_ms(),
            stopped_speed_threshold: default_stopped_speed_threshold(),
        }
    }
}

/// Reason the emergency stop was triggered.
#[derive(Debug, Clone)]
pub enum EstopReason {
    CommandTimeout { age_ms: u64 },
    PeerDead { peer: String, age_ms: u64 },
    ExplicitRequest,
}

impl EstopReason {
    /// Severity ordering: a latched reason may only be replaced by a
    /// stricter one (stricter = harder to clear), never weakened.
    fn severity(&self) -> u8 {
        match self {
            EstopReason::CommandTimeout { .. } => 0,
            EstopReason::PeerDead { .. } => 1,
            EstopReason::ExplicitRequest => 2,
        }
    }

    pub fn is_command_timeout(&self) -> bool {
        matches!(self, EstopReason::CommandTimeout { .. })
    }
}

/// Watchdog monitors command freshness and peer health.
pub struct Watchdog {
    config: WatchdogConfig,

    /// Set to true when e-stop is active. Read by the control loop.
    estop_active: Arc<AtomicBool>,

    /// Timestamp of the last valid ControlCommand received.
    last_command_time: Instant,

    /// Timestamp of last heartbeat from each peer.
    peer_heartbeats: std::collections::HashMap<String, Instant>,

    /// Latched e-stop reason; None means no e-stop.
    latched: Option<EstopReason>,

    /// Open-loop deceleration ramp speed, seeded from the measured speed
    /// at e-stop entry and driven down by `deceleration_velocity`.
    ramp_speed: f64,

    /// Latest measured linear velocity from chassis feedback.
    measured_speed: f64,
}

impl Watchdog {
    pub fn new(config: WatchdogConfig, estop_active: Arc<AtomicBool>) -> Self {
        let now = Instant::now();
        let mut peer_heartbeats = std::collections::HashMap::new();
        // Initialize peers with current time (grace period on startup)
        peer_heartbeats.insert("sensperc".into(), now);
        peer_heartbeats.insert("planning".into(), now);

        Self {
            config,
            estop_active,
            last_command_time: now,
            peer_heartbeats,
            latched: None,
            ramp_speed: 0.0,
            measured_speed: 0.0,
        }
    }

    /// Notify that a FRESH, actionable ControlCommand was received.
    ///
    /// The caller must already have applied the command-age gate — stale
    /// messages must never reach this method. `emergency_stop` is the
    /// command's explicit e-stop flag.
    pub fn notify_command_received(&mut self, emergency_stop: bool) {
        self.last_command_time = Instant::now();

        if emergency_stop {
            self.trigger_estop(EstopReason::ExplicitRequest);
            return;
        }

        // Latch-clear rules, per reason.
        match &self.latched {
            Some(EstopReason::CommandTimeout { .. }) => {
                self.clear_estop("fresh commands resumed after timeout");
            }
            Some(EstopReason::ExplicitRequest) => {
                if self.measured_speed.abs() < self.config.stopped_speed_threshold {
                    self.clear_estop("explicit e-stop released and vehicle stopped");
                } else {
                    warn!(
                        "Watchdog: e-stop release requested but vehicle still moving \
                         ({:.3} m/s >= {:.3} m/s) — latch held",
                        self.measured_speed.abs(),
                        self.config.stopped_speed_threshold
                    );
                }
            }
            Some(EstopReason::PeerDead { peer, .. }) => {
                let peer = peer.clone();
                if self.peer_healthy(&peer) {
                    self.clear_estop("peer heartbeat recovered and commands resumed");
                }
            }
            None => {}
        }
    }

    /// Notify that a heartbeat was received from a peer.
    pub fn notify_heartbeat(&mut self, peer: &str) {
        self.peer_heartbeats
            .insert(peer.to_string(), Instant::now());
    }

    /// Update the measured linear velocity from chassis feedback.
    /// Used to seed the deceleration ramp at e-stop entry and to verify
    /// the vehicle is stopped before releasing an explicit e-stop.
    pub fn update_measured_speed(&mut self, speed: f64) {
        self.measured_speed = speed;
    }

    /// Trigger an emergency stop, latching the reason. A latched reason
    /// is only replaced by a stricter one (never weakened).
    pub fn trigger_estop(&mut self, reason: EstopReason) {
        match &self.latched {
            None => {
                error!("EMERGENCY STOP triggered: {:?}", reason);
                // Seed the open-loop deceleration ramp from the last
                // measured speed; the ramp is never overwritten while
                // the e-stop is latched.
                self.ramp_speed = self.measured_speed;
                self.latched = Some(reason);
                self.estop_active.store(true, Ordering::Release);
            }
            Some(current) if reason.severity() > current.severity() => {
                error!("EMERGENCY STOP escalated: {:?} -> {:?}", current, reason);
                self.latched = Some(reason);
            }
            Some(_) => {}
        }
    }

    fn clear_estop(&mut self, why: &str) {
        if self.latched.is_some() {
            info!("Watchdog: clearing e-stop ({})", why);
            self.latched = None;
            self.ramp_speed = 0.0;
            self.estop_active.store(false, Ordering::Release);
        }
    }

    fn peer_healthy(&self, peer: &str) -> bool {
        self.peer_heartbeats
            .get(peer)
            .is_some_and(|t| t.elapsed() <= Duration::from_millis(self.config.heartbeat_dead_ms))
    }

    /// Check all watchdog conditions. Call this at the watchdog rate (10Hz).
    /// Returns Some(EstopReason) if e-stop should be triggered.
    pub fn check(&mut self) -> Option<EstopReason> {
        // Check command freshness
        let cmd_age = self.last_command_time.elapsed();
        let cmd_timeout = Duration::from_millis(self.config.command_timeout_ms);

        if cmd_age > cmd_timeout {
            let reason = EstopReason::CommandTimeout {
                age_ms: cmd_age.as_millis() as u64,
            };
            self.trigger_estop(reason.clone());
            return Some(reason);
        }

        // Check planning heartbeat (critical peer)
        let hb_timeout = Duration::from_millis(self.config.heartbeat_dead_ms);
        if let Some(last_hb) = self.peer_heartbeats.get("planning") {
            if last_hb.elapsed() > hb_timeout {
                let reason = EstopReason::PeerDead {
                    peer: "planning".into(),
                    age_ms: last_hb.elapsed().as_millis() as u64,
                };
                self.trigger_estop(reason.clone());
                return Some(reason);
            }
        }

        // Check sensperc heartbeat (warn but don't e-stop immediately)
        if let Some(last_hb) = self.peer_heartbeats.get("sensperc") {
            if last_hb.elapsed() > hb_timeout {
                warn!(
                    "Watchdog: sensperc heartbeat stale ({:.0}ms)",
                    last_hb.elapsed().as_millis()
                );
                // Don't e-stop for sensperc — planning should handle this
                // by reducing speed or planning a safe stop
            }
        }

        None
    }

    /// Compute the deceleration command for controlled stop.
    /// Runs OPEN-LOOP on the ramp speed captured at e-stop entry —
    /// measured feedback must not overwrite the profile mid-ramp.
    /// Returns the velocity to send during e-stop deceleration.
    pub fn deceleration_velocity(&mut self, dt: f64) -> f64 {
        if self.ramp_speed.abs() < 0.01 {
            self.ramp_speed = 0.0;
            return 0.0;
        }

        let decel = self.config.deceleration_rate * dt;
        if self.ramp_speed > 0.0 {
            self.ramp_speed = (self.ramp_speed - decel).max(0.0);
        } else {
            self.ramp_speed = (self.ramp_speed + decel).min(0.0);
        }

        self.ramp_speed
    }

    /// The currently latched e-stop reason, if any.
    pub fn latched_reason(&self) -> Option<&EstopReason> {
        self.latched.as_ref()
    }

    // Public accessor used by the control loop and StatePublisher to gate output.
    pub fn is_estop_active(&self) -> bool {
        self.estop_active.load(Ordering::Acquire)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_wd(config: WatchdogConfig) -> (Watchdog, Arc<AtomicBool>) {
        let estop = Arc::new(AtomicBool::new(false));
        let wd = Watchdog::new(config, Arc::clone(&estop));
        (wd, estop)
    }

    #[test]
    fn test_command_timeout() {
        let (mut wd, estop) = make_wd(WatchdogConfig {
            command_timeout_ms: 50, // short for testing
            ..Default::default()
        });

        // Fresh command → no e-stop
        wd.notify_command_received(false);
        assert!(wd.check().is_none());
        assert!(!estop.load(Ordering::Acquire));

        // Wait for timeout
        std::thread::sleep(Duration::from_millis(60));
        let reason = wd.check();
        assert!(reason.is_some());
        assert!(estop.load(Ordering::Acquire));
        assert!(wd.latched_reason().unwrap().is_command_timeout());
    }

    #[test]
    fn test_timeout_auto_recovery() {
        let (mut wd, estop) = make_wd(WatchdogConfig {
            command_timeout_ms: 50,
            ..Default::default()
        });

        // Trigger timeout
        std::thread::sleep(Duration::from_millis(60));
        wd.check();
        assert!(estop.load(Ordering::Acquire));

        // Receive fresh command → auto-recover
        wd.notify_command_received(false);
        assert!(!estop.load(Ordering::Acquire));
        assert!(wd.latched_reason().is_none());
    }

    #[test]
    fn test_explicit_estop_held_while_moving() {
        let (mut wd, estop) = make_wd(WatchdogConfig::default());

        wd.update_measured_speed(0.5);
        wd.notify_command_received(true); // explicit e-stop
        assert!(estop.load(Ordering::Acquire));

        // Fresh release command, but vehicle still moving → latch held
        wd.notify_command_received(false);
        assert!(estop.load(Ordering::Acquire));

        // Vehicle stopped → next fresh release clears
        wd.update_measured_speed(0.0);
        wd.notify_command_received(false);
        assert!(!estop.load(Ordering::Acquire));
    }

    #[test]
    fn test_explicit_estop_not_weakened_by_timeout() {
        let (mut wd, estop) = make_wd(WatchdogConfig {
            command_timeout_ms: 30,
            ..Default::default()
        });

        wd.update_measured_speed(0.4);
        wd.notify_command_received(true); // explicit e-stop latched

        // Command timeout fires while explicit e-stop latched — must not
        // downgrade the latch to the auto-clearing CommandTimeout reason.
        std::thread::sleep(Duration::from_millis(40));
        wd.check();
        assert!(matches!(
            wd.latched_reason(),
            Some(EstopReason::ExplicitRequest)
        ));

        // Fresh release while still moving must NOT clear.
        wd.notify_command_received(false);
        assert!(estop.load(Ordering::Acquire));
    }

    #[test]
    fn test_peer_dead_clears_only_after_heartbeat_recovers() {
        let (mut wd, estop) = make_wd(WatchdogConfig {
            heartbeat_dead_ms: 30,
            command_timeout_ms: 10_000, // commands stay "fresh"
            ..Default::default()
        });

        wd.notify_command_received(false);
        std::thread::sleep(Duration::from_millis(40));
        wd.notify_command_received(false); // keep commands fresh
        let reason = wd.check();
        assert!(matches!(reason, Some(EstopReason::PeerDead { .. })));
        assert!(estop.load(Ordering::Acquire));

        // Fresh command alone does not clear — heartbeat still dead.
        wd.notify_command_received(false);
        assert!(estop.load(Ordering::Acquire));

        // Heartbeat recovers + fresh command → clears.
        wd.notify_heartbeat("planning");
        wd.notify_command_received(false);
        assert!(!estop.load(Ordering::Acquire));
    }

    #[test]
    fn test_deceleration_open_loop() {
        let (mut wd, _estop) = make_wd(WatchdogConfig::default());
        wd.update_measured_speed(1.0);
        wd.trigger_estop(EstopReason::CommandTimeout { age_ms: 250 });

        let dt = 0.1;
        let v1 = wd.deceleration_velocity(dt);
        assert!(v1 < 1.0);
        assert!(v1 > 0.0);

        // Feedback updates must NOT clobber the ramp mid-profile.
        wd.update_measured_speed(0.9);
        let v2 = wd.deceleration_velocity(dt);
        assert!(v2 < v1);

        // Keep decelerating until stopped
        for _ in 0..100 {
            wd.deceleration_velocity(dt);
        }
        assert_eq!(wd.deceleration_velocity(dt), 0.0);
    }

    #[test]
    fn test_negative_speed_ramps_up_to_zero() {
        let (mut wd, _estop) = make_wd(WatchdogConfig::default());
        wd.update_measured_speed(-0.8);
        wd.trigger_estop(EstopReason::ExplicitRequest);

        let dt = 0.1;
        let v1 = wd.deceleration_velocity(dt);
        assert!(v1 > -0.8 && v1 < 0.0);
        for _ in 0..100 {
            wd.deceleration_velocity(dt);
        }
        assert_eq!(wd.deceleration_velocity(dt), 0.0);
    }
}
