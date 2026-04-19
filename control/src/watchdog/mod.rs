/// Watchdog and emergency stop handler.
///
/// Monitors:
/// - Freshness of ControlCommand messages from Planning (CH2)
/// - Heartbeats from peer processes (CH0)
///
/// Triggers autonomous emergency stop when:
/// - No valid ControlCommand received for `command_timeout_ms`
/// - A critical peer (planning) is detected as dead
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
}

fn default_command_timeout_ms() -> u64 { 200 }
fn default_deceleration_rate() -> f64 { 0.5 }
fn default_heartbeat_dead_ms() -> u64 { 1000 }

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            command_timeout_ms: default_command_timeout_ms(),
            deceleration_rate: default_deceleration_rate(),
            heartbeat_dead_ms: default_heartbeat_dead_ms(),
        }
    }
}

/// Reason the emergency stop was triggered.
// Variants carry diagnostic fields read via Debug formatting in the e-stop log line.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum EstopReason {
    CommandTimeout { age_ms: u64 },
    PeerDead { peer: String, age_ms: u64 },
    ExplicitRequest,
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

    /// Current linear velocity (for controlled deceleration).
    current_speed: f64,
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
            current_speed: 0.0,
        }
    }

    /// Notify that a valid ControlCommand was received.
    pub fn notify_command_received(&mut self) {
        self.last_command_time = Instant::now();

        // Clear e-stop if it was triggered by timeout (auto-recovery)
        if self.estop_active.load(Ordering::Acquire) {
            info!("Watchdog: command received, clearing e-stop");
            self.estop_active.store(false, Ordering::Release);
        }
    }

    /// Notify that a heartbeat was received from a peer.
    pub fn notify_heartbeat(&mut self, peer: &str) {
        self.peer_heartbeats
            .insert(peer.to_string(), Instant::now());
    }

    /// Update the current speed (for controlled deceleration calculation).
    pub fn update_speed(&mut self, speed: f64) {
        self.current_speed = speed;
    }

    /// Trigger an explicit emergency stop.
    pub fn trigger_estop(&self, reason: EstopReason) {
        if !self.estop_active.load(Ordering::Acquire) {
            error!("EMERGENCY STOP triggered: {:?}", reason);
            self.estop_active.store(true, Ordering::Release);
        }
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
    /// Returns the velocity to send during e-stop deceleration.
    pub fn deceleration_velocity(&mut self, dt: f64) -> f64 {
        if self.current_speed.abs() < 0.01 {
            self.current_speed = 0.0;
            return 0.0;
        }

        let decel = self.config.deceleration_rate * dt;
        if self.current_speed > 0.0 {
            self.current_speed = (self.current_speed - decel).max(0.0);
        } else {
            self.current_speed = (self.current_speed + decel).min(0.0);
        }

        self.current_speed
    }

    // Public accessor used by the control loop and StatePublisher to gate output.
    #[allow(dead_code)]
    pub fn is_estop_active(&self) -> bool {
        self.estop_active.load(Ordering::Acquire)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_command_timeout() {
        let estop = Arc::new(AtomicBool::new(false));
        let config = WatchdogConfig {
            command_timeout_ms: 50, // short for testing
            ..Default::default()
        };
        let mut wd = Watchdog::new(config, Arc::clone(&estop));

        // Fresh command → no e-stop
        wd.notify_command_received();
        assert!(wd.check().is_none());
        assert!(!estop.load(Ordering::Acquire));

        // Wait for timeout
        std::thread::sleep(Duration::from_millis(60));
        let reason = wd.check();
        assert!(reason.is_some());
        assert!(estop.load(Ordering::Acquire));
    }

    #[test]
    fn test_auto_recovery() {
        let estop = Arc::new(AtomicBool::new(false));
        let config = WatchdogConfig {
            command_timeout_ms: 50,
            ..Default::default()
        };
        let mut wd = Watchdog::new(config, Arc::clone(&estop));

        // Trigger timeout
        std::thread::sleep(Duration::from_millis(60));
        wd.check();
        assert!(estop.load(Ordering::Acquire));

        // Receive command → auto-recover
        wd.notify_command_received();
        assert!(!estop.load(Ordering::Acquire));
    }

    #[test]
    fn test_deceleration() {
        let estop = Arc::new(AtomicBool::new(true));
        let mut wd = Watchdog::new(WatchdogConfig::default(), estop);
        wd.current_speed = 1.0;

        let dt = 0.1;
        let v1 = wd.deceleration_velocity(dt);
        assert!(v1 < 1.0);
        assert!(v1 > 0.0);

        // Keep decelerating until stopped
        for _ in 0..100 {
            wd.deceleration_velocity(dt);
        }
        assert_eq!(wd.current_speed, 0.0);
    }
}
