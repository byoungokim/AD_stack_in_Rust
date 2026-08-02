/// Pipeline arbitrator: selects between traditional and E2E outputs,
/// applies safety envelope constraints.
///
/// Always runs as the final gate before publishing ControlCommand on CH2.
/// Ensures no command violates physical limits regardless of source.
use serde::Deserialize;

use crate::local_planner::VelocityCommand;

/// Pipeline mode (matches proto PipelineMode).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // E2E + Shadow wired via config; not yet selected at runtime.
pub enum PipelineMode {
    Traditional,
    E2E,
    Shadow,
}

#[derive(Debug, Clone, Deserialize)]
pub struct ArbitratorConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default)]
    pub safety: SafetyEnvelopeConfig,
    #[serde(default = "default_e2e_confidence_threshold")]
    pub e2e_confidence_threshold: f32,
    /// Minimum confidence required to trust the traditional pipeline as a
    /// fallback from E2E. In E2E mode, if both E2E and traditional are below
    /// threshold, the arbitrator emits an emergency stop. In Traditional and
    /// Shadow modes a confidence below this enters the DEGRADED response
    /// instead: zero-speed commands decelerate through the normal envelope
    /// (controlled stop, not a latched E-stop) and non-zero commands are
    /// speed-capped at `degraded_speed_cap`.
    #[serde(default = "default_fallback_min_confidence")]
    pub fallback_min_confidence: f32,
    /// Linear speed cap (m/s) applied to low-confidence-but-feasible commands
    /// in Traditional/Shadow modes (graduated response, not an E-stop).
    #[serde(default = "default_degraded_speed_cap")]
    pub degraded_speed_cap: f64,
    /// Consecutive cycles with confidence >= `fallback_min_confidence`
    /// required before the degraded speed cap is lifted (hysteresis against
    /// oscillating speed at the confidence boundary).
    #[serde(default = "default_confidence_recovery_cycles")]
    pub confidence_recovery_cycles: u32,
}

fn default_rate_hz() -> u32 {
    10
}
fn default_e2e_confidence_threshold() -> f32 {
    0.7
}
fn default_fallback_min_confidence() -> f32 {
    0.3
}
fn default_degraded_speed_cap() -> f64 {
    0.15
}
fn default_confidence_recovery_cycles() -> u32 {
    5
}

impl Default for ArbitratorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            safety: SafetyEnvelopeConfig::default(),
            e2e_confidence_threshold: default_e2e_confidence_threshold(),
            fallback_min_confidence: default_fallback_min_confidence(),
            degraded_speed_cap: default_degraded_speed_cap(),
            confidence_recovery_cycles: default_confidence_recovery_cycles(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct SafetyEnvelopeConfig {
    #[serde(default = "default_max_speed")]
    pub max_speed: f64, // m/s
    #[serde(default = "default_max_accel")]
    pub max_acceleration: f64, // m/s^2
    #[serde(default = "default_max_decel")]
    pub max_deceleration: f64, // m/s^2
    #[serde(default = "default_max_angular")]
    pub max_angular_speed: f64, // rad/s
    #[serde(default = "default_max_curvature")]
    pub max_curvature: f64, // 1/m
}

// Envelope sized for the >1.5 m/s gauntlet target and kept coherent with the
// DWA limits (which must never exceed it): 2.2 m/s cruise, 2.5/3.0 m/s²
// accel/decel, and max_angular_speed 4.5 — the curvature envelope needs
// max_curvature × max_speed = 2.0 × 2.2 = 4.4 rad/s to be executable at
// full cruise, otherwise the angular clamp would widen verified arcs.
fn default_max_speed() -> f64 {
    2.2
}
fn default_max_accel() -> f64 {
    2.5
}
fn default_max_decel() -> f64 {
    3.0
}
fn default_max_angular() -> f64 {
    4.5
}
fn default_max_curvature() -> f64 {
    2.0
}

impl Default for SafetyEnvelopeConfig {
    fn default() -> Self {
        Self {
            max_speed: default_max_speed(),
            max_acceleration: default_max_accel(),
            max_deceleration: default_max_decel(),
            max_angular_speed: default_max_angular(),
            max_curvature: default_max_curvature(),
        }
    }
}

impl SafetyEnvelopeConfig {
    /// Reject physically nonsensical envelope values. A safety gate must not be
    /// disabled by a YAML typo (max_speed: 0 means "never move", max_accel: -1
    /// means "anti-accelerate"). Fail loudly at startup.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.max_speed > 0.0 && self.max_speed.is_finite()) {
            return Err(format!(
                "safety.max_speed must be > 0, got {}",
                self.max_speed
            ));
        }
        if !(self.max_acceleration > 0.0 && self.max_acceleration.is_finite()) {
            return Err(format!(
                "safety.max_acceleration must be > 0, got {}",
                self.max_acceleration
            ));
        }
        if !(self.max_deceleration > 0.0 && self.max_deceleration.is_finite()) {
            return Err(format!(
                "safety.max_deceleration must be > 0, got {}",
                self.max_deceleration
            ));
        }
        if !(self.max_angular_speed > 0.0 && self.max_angular_speed.is_finite()) {
            return Err(format!(
                "safety.max_angular_speed must be > 0, got {}",
                self.max_angular_speed
            ));
        }
        if !(self.max_curvature > 0.0 && self.max_curvature.is_finite()) {
            return Err(format!(
                "safety.max_curvature must be > 0, got {}",
                self.max_curvature
            ));
        }
        Ok(())
    }
}

impl ArbitratorConfig {
    pub fn validate(&self) -> Result<(), String> {
        self.safety.validate()?;
        if !(0.0..=1.0).contains(&self.e2e_confidence_threshold) {
            return Err(format!(
                "e2e_confidence_threshold must be in [0, 1], got {}",
                self.e2e_confidence_threshold
            ));
        }
        if !(0.0..=1.0).contains(&self.fallback_min_confidence) {
            return Err(format!(
                "fallback_min_confidence must be in [0, 1], got {}",
                self.fallback_min_confidence
            ));
        }
        if !(self.degraded_speed_cap > 0.0 && self.degraded_speed_cap.is_finite()) {
            return Err(format!(
                "degraded_speed_cap must be > 0, got {}",
                self.degraded_speed_cap
            ));
        }
        if self.degraded_speed_cap > self.safety.max_speed {
            return Err(format!(
                "degraded_speed_cap ({}) must not exceed safety.max_speed ({})",
                self.degraded_speed_cap, self.safety.max_speed
            ));
        }
        if self.confidence_recovery_cycles == 0 {
            return Err("confidence_recovery_cycles must be >= 1".into());
        }
        Ok(())
    }
}

/// Output from the arbitrator.
#[derive(Debug, Clone)]
pub struct ArbitratorOutput {
    pub command: VelocityCommand,
    pub source: PipelineMode,
    pub emergency_stop: bool,
    pub safety_clipped: bool,
}

/// Encode an ArbitratorOutput as the wire-format ControlCommand published on CH2.
/// Shadow mode maps to SourceTraditional on the wire — the wire command IS the
/// traditional one; Shadow is an internal mode flag, not a wire source.
pub fn encode_control_command(
    out: &ArbitratorOutput,
    sequence: u32,
    timestamp_ns: u64,
) -> limo_proto::ControlCommand {
    limo_proto::ControlCommand {
        header: Some(limo_proto::Header {
            timestamp_ns,
            sequence,
            frame_id: "".into(),
        }),
        source: match out.source {
            PipelineMode::Traditional => limo_proto::PipelineSource::SourceTraditional as i32,
            PipelineMode::E2E => limo_proto::PipelineSource::SourceE2e as i32,
            PipelineMode::Shadow => limo_proto::PipelineSource::SourceTraditional as i32,
        },
        command: Some(limo_proto::control_command::Command::VelocityCmd(
            limo_proto::Twist2D {
                linear_x: out.command.linear_x,
                linear_y: 0.0,
                angular_z: out.command.angular_z,
            },
        )),
        confidence: out.command.confidence,
        emergency_stop: out.emergency_stop,
    }
}

/// Linear speeds below this (m/s) are treated as "zero command": DWA signals
/// infeasibility with an exact 0.0 command; the small band absorbs float noise.
const NEAR_ZERO_SPEED: f64 = 0.02;

pub struct Arbitrator {
    config: ArbitratorConfig,
    mode: PipelineMode,
    prev_linear: f64,
    /// Graduated-response latch: true while the Traditional/Shadow pipeline is
    /// in the low-confidence degraded regime (speed capped).
    degraded: bool,
    /// Consecutive cycles at/above `fallback_min_confidence` while degraded.
    recovery_streak: u32,
}

impl Arbitrator {
    pub fn new(config: ArbitratorConfig) -> Self {
        Self {
            config,
            mode: PipelineMode::Traditional,
            prev_linear: 0.0,
            degraded: false,
            recovery_streak: 0,
        }
    }

    /// True while the degraded speed cap is active (test/status inspection).
    #[allow(dead_code)]
    pub fn is_degraded(&self) -> bool {
        self.degraded
    }

    /// Update the degraded latch from this cycle's planner confidence and
    /// return whether the cap applies THIS cycle. Entering is immediate; the
    /// cap only lifts after `confidence_recovery_cycles` consecutive cycles
    /// at/above the threshold (the K-th good cycle is still capped, the
    /// K+1-th is free) — hysteresis against speed oscillation at the boundary.
    fn update_degraded(&mut self, confidence: f32) -> bool {
        if confidence < self.config.fallback_min_confidence {
            self.degraded = true;
            self.recovery_streak = 0;
        } else if self.degraded {
            self.recovery_streak += 1;
            if self.recovery_streak >= self.config.confidence_recovery_cycles {
                self.degraded = false;
                self.recovery_streak = 0;
                return false;
            }
        }
        self.degraded
    }

    /// Graduated low-confidence response for Traditional/Shadow: never an
    /// E-stop. A (near-)zero command (DWA infeasibility) passes through the
    /// envelope as a decel-limited controlled stop; a non-zero command is
    /// forwarded with |linear_x| capped at `degraded_speed_cap`.
    fn apply_degraded_cap(&self, cmd: &VelocityCommand) -> VelocityCommand {
        let mut out = cmd.clone();
        if out.linear_x.abs() < NEAR_ZERO_SPEED {
            // Infeasibility signal: controlled stop through the envelope.
            out.linear_x = 0.0;
            out.angular_z = 0.0;
        } else {
            let cap = self.config.degraded_speed_cap;
            out.linear_x = out.linear_x.clamp(-cap, cap);
        }
        out
    }

    #[allow(dead_code)] // Public knob for runtime mode swap; not yet wired to config reload.
    pub fn set_mode(&mut self, mode: PipelineMode) {
        self.mode = mode;
    }

    // Public getter, symmetric with set_mode; used by status publisher.
    #[allow(dead_code)]
    pub fn mode(&self) -> PipelineMode {
        self.mode
    }

    /// Select between traditional and E2E commands, apply safety envelope.
    ///
    /// E2E mode keeps the hard fallback ladder: E2E below
    /// `e2e_confidence_threshold` falls back to traditional; if the fallback
    /// itself is below `fallback_min_confidence` the arbitrator emits an
    /// emergency stop (a neural pipeline with no trustworthy fallback must
    /// not keep driving).
    ///
    /// Traditional/Shadow use the GRADUATED response instead: low confidence
    /// means the classical planner found no (or only a poor) trajectory —
    /// a zero command becomes a decel-limited controlled stop and a non-zero
    /// command is forwarded speed-capped at `degraded_speed_cap`, with
    /// hysteresis on recovery. A permanent E-stop latch here deadlocked the
    /// robot mid-slalom (MPC confidence pinned low near obstacles).
    pub fn arbitrate(
        &mut self,
        traditional: &VelocityCommand,
        e2e: Option<&VelocityCommand>,
        dt: f64,
    ) -> ArbitratorOutput {
        let (raw_cmd, source) = match self.mode {
            PipelineMode::Traditional => {
                let cmd = if self.update_degraded(traditional.confidence) {
                    self.apply_degraded_cap(traditional)
                } else {
                    traditional.clone()
                };
                (cmd, PipelineMode::Traditional)
            }

            PipelineMode::E2E => {
                let e2e_usable =
                    e2e.filter(|c| c.confidence >= self.config.e2e_confidence_threshold);
                match e2e_usable {
                    Some(c) => (c.clone(), PipelineMode::E2E),
                    None => {
                        if traditional.confidence < self.config.fallback_min_confidence {
                            return self.emergency_stop();
                        }
                        (traditional.clone(), PipelineMode::Traditional)
                    }
                }
            }

            PipelineMode::Shadow => {
                // Shadow: traditional controls; E2E ran in parallel for logging.
                // The traditional command drives, so it gets the same graduated
                // low-confidence response as in Traditional mode.
                let cmd = if self.update_degraded(traditional.confidence) {
                    self.apply_degraded_cap(traditional)
                } else {
                    traditional.clone()
                };
                (cmd, PipelineMode::Shadow)
            }
        };

        // Apply safety envelope
        let (safe_cmd, clipped) = self.apply_safety_envelope(&raw_cmd, dt);

        self.prev_linear = safe_cmd.linear_x;

        ArbitratorOutput {
            command: safe_cmd,
            source,
            emergency_stop: false,
            safety_clipped: clipped,
        }
    }

    /// Force an emergency stop output.
    pub fn emergency_stop(&mut self) -> ArbitratorOutput {
        self.prev_linear = 0.0;
        ArbitratorOutput {
            command: VelocityCommand {
                linear_x: 0.0,
                angular_z: 0.0,
                confidence: 1.0,
            },
            source: self.mode,
            emergency_stop: true,
            safety_clipped: false,
        }
    }

    /// Clamp command to safety limits. Returns (clamped_cmd, was_clipped).
    fn apply_safety_envelope(&self, cmd: &VelocityCommand, dt: f64) -> (VelocityCommand, bool) {
        let mut clipped = false;

        // Clamp speed
        let mut v = cmd
            .linear_x
            .clamp(-self.config.safety.max_speed, self.config.safety.max_speed);
        if v != cmd.linear_x {
            clipped = true;
        }

        // Clamp acceleration/deceleration, symmetric in the driving direction:
        // "acceleration" is |v| moving away from zero, "deceleration" is |v|
        // moving toward zero. The recovery behavior commands reverse speeds
        // (linear_x < 0); braking OUT of reverse must be decel-limited (fast),
        // not accel-limited, and speeding up in reverse must be accel-limited.
        let dv = v - self.prev_linear;
        let max_dv_accel = self.config.safety.max_acceleration * dt;
        let max_dv_decel = self.config.safety.max_deceleration * dt;
        let (dv_max, dv_min) = if self.prev_linear > 0.0 {
            // Moving forward: +dv accelerates, -dv brakes (toward/through 0).
            (max_dv_accel, -max_dv_decel)
        } else if self.prev_linear < 0.0 {
            // Moving in reverse: -dv accelerates (faster reverse), +dv brakes.
            (max_dv_decel, -max_dv_accel)
        } else {
            // At rest: either direction is acceleration.
            (max_dv_accel, -max_dv_accel)
        };

        if dv > dv_max {
            v = self.prev_linear + dv_max;
            clipped = true;
        } else if dv < dv_min {
            v = self.prev_linear + dv_min;
            clipped = true;
        }

        // Clamp angular speed
        let mut w = cmd.angular_z.clamp(
            -self.config.safety.max_angular_speed,
            self.config.safety.max_angular_speed,
        );
        if w != cmd.angular_z {
            clipped = true;
        }

        // Clamp curvature (if moving)
        if v.abs() > 0.01 {
            let curvature = (w / v).abs();
            if curvature > self.config.safety.max_curvature {
                w = w.signum() * self.config.safety.max_curvature * v.abs();
                clipped = true;
            }
        }

        let safe_cmd = VelocityCommand {
            linear_x: v,
            angular_z: w,
            confidence: cmd.confidence,
        };

        (safe_cmd, clipped)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_safety_clamp_speed() {
        let cfg = ArbitratorConfig::default();
        let max_speed = cfg.safety.max_speed;
        let mut arb = Arbitrator::new(cfg);
        // Well above the envelope regardless of the configured limit.
        let cmd = VelocityCommand {
            linear_x: max_speed + 5.0,
            angular_z: 0.0,
            confidence: 0.9,
        };

        // Even fully accelerated (many cycles), the speed must clamp at the
        // envelope, never at the commanded value.
        let mut out = arb.arbitrate(&cmd, None, 0.1);
        for _ in 0..100 {
            out = arb.arbitrate(&cmd, None, 0.1);
        }
        assert!(out.command.linear_x <= max_speed + 1e-9);
        assert!(out.safety_clipped);
    }

    #[test]
    fn test_safety_clamp_acceleration() {
        let cfg = ArbitratorConfig::default();
        let max_dv = cfg.safety.max_acceleration * 0.1; // per 10Hz cycle
        let mut arb = Arbitrator::new(cfg);
        arb.prev_linear = 0.0;

        // Try to jump from 0 to max_speed in one 0.1s cycle: must be
        // accel-limited to max_acceleration * dt.
        let cmd = VelocityCommand {
            linear_x: ArbitratorConfig::default().safety.max_speed,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let out = arb.arbitrate(&cmd, None, 0.1);

        assert!(out.command.linear_x <= max_dv + 1e-6);
        assert!(out.safety_clipped);
    }

    #[test]
    fn test_e2e_fallback_low_confidence() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.1,
            confidence: 0.9,
        };
        let e2e = VelocityCommand {
            linear_x: 0.8,
            angular_z: 0.2,
            confidence: 0.3,
        }; // low

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        assert_eq!(out.source, PipelineMode::Traditional); // should fallback
    }

    #[test]
    fn test_e2e_accepted_high_confidence() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand {
            linear_x: 0.3,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let e2e = VelocityCommand {
            linear_x: 0.4,
            angular_z: 0.1,
            confidence: 0.95,
        };

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        assert_eq!(out.source, PipelineMode::E2E);
    }

    #[test]
    fn test_emergency_stop() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        let out = arb.emergency_stop();
        assert!(out.emergency_stop);
        assert_eq!(out.command.linear_x, 0.0);
        assert_eq!(out.command.angular_z, 0.0);
    }

    #[test]
    fn test_e2e_mode_both_low_confidence_emergency_stops() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.1,
            confidence: 0.2,
        };
        let e2e = VelocityCommand {
            linear_x: 0.8,
            angular_z: 0.2,
            confidence: 0.3,
        };

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        assert!(out.emergency_stop);
        assert_eq!(out.command.linear_x, 0.0);
        assert_eq!(out.command.angular_z, 0.0);
    }

    #[test]
    fn test_e2e_mode_no_e2e_low_traditional_emergency_stops() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.1,
            confidence: 0.1,
        };
        let out = arb.arbitrate(&traditional, None, 0.1);
        assert!(out.emergency_stop);
    }

    #[test]
    fn test_traditional_mode_low_confidence_zero_cmd_is_controlled_stop() {
        // DWA reports confidence 0.1 with a zero command when no feasible
        // trajectory exists. Traditional mode must respond with a CONTROLLED
        // stop through the normal envelope decel — never a latched emergency
        // stop (that deadlocked the robot mid-slalom when MPC confidence
        // pinned low every cycle).
        let mut arb = Arbitrator::new(ArbitratorConfig::default());

        // Ramp up to speed first so decel limiting is observable.
        let cruise = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.0,
            confidence: 0.9,
        };
        for _ in 0..20 {
            arb.arbitrate(&cruise, None, 0.1);
        }
        assert!(arb.prev_linear > 0.4);

        let infeasible = VelocityCommand {
            linear_x: 0.0,
            angular_z: 0.0,
            confidence: 0.1,
        };
        let out = arb.arbitrate(&infeasible, None, 0.1);
        assert!(
            !out.emergency_stop,
            "planner infeasibility must not latch an emergency stop"
        );
        // Decel-limited toward zero: one cycle at max_deceleration removes
        // max_deceleration * 0.1 from ~0.5 without reaching zero instantly.
        assert!(out.command.linear_x < 0.45);
        assert!(out.command.linear_x > 0.0);

        // Kept infeasible: the envelope walks the speed to exactly zero.
        for _ in 0..10 {
            arb.arbitrate(&infeasible, None, 0.1);
        }
        let out = arb.arbitrate(&infeasible, None, 0.1);
        assert!(!out.emergency_stop);
        assert_eq!(out.command.linear_x, 0.0);
        assert_eq!(out.command.angular_z, 0.0);
    }

    #[test]
    fn test_shadow_mode_low_confidence_is_graduated_not_estop() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::Shadow);

        // Non-zero but low-confidence traditional command: forwarded with the
        // degraded speed cap, never an E-stop; E2E must not leak through.
        let low_quality = VelocityCommand {
            linear_x: 0.4,
            angular_z: 0.0,
            confidence: 0.1,
        };
        let e2e = VelocityCommand {
            linear_x: 0.8,
            angular_z: 0.5,
            confidence: 0.95,
        };
        let out = arb.arbitrate(&low_quality, Some(&e2e), 0.1);
        assert!(!out.emergency_stop);
        assert_eq!(out.source, PipelineMode::Shadow);
        let cap = ArbitratorConfig::default().degraded_speed_cap;
        assert!(
            out.command.linear_x <= cap + 1e-9,
            "degraded command {} exceeds cap {}",
            out.command.linear_x,
            cap
        );
        assert!(out.command.linear_x > 0.0, "feasible command must move");
    }

    #[test]
    fn test_degraded_cap_applies_to_nonzero_low_confidence_command() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());

        // Warm up above the cap so the cap (not the accel limiter) binds.
        let cruise = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.0,
            confidence: 0.9,
        };
        for _ in 0..20 {
            arb.arbitrate(&cruise, None, 0.1);
        }

        let low_quality = VelocityCommand {
            linear_x: 0.5,
            angular_z: 0.2,
            confidence: 0.2,
        };
        // Decel-limited ramp-down to the cap, then held at the cap.
        let cap = ArbitratorConfig::default().degraded_speed_cap;
        let mut out = arb.arbitrate(&low_quality, None, 0.1);
        for _ in 0..10 {
            out = arb.arbitrate(&low_quality, None, 0.1);
        }
        assert!(!out.emergency_stop);
        assert!((out.command.linear_x - cap).abs() < 1e-9);
        assert!(arb.is_degraded());
    }

    #[test]
    fn test_degraded_cap_recovery_hysteresis() {
        let cfg = ArbitratorConfig::default();
        let k = cfg.confidence_recovery_cycles;
        let mut arb = Arbitrator::new(cfg);

        let low = VelocityCommand {
            linear_x: 0.3,
            angular_z: 0.0,
            confidence: 0.1,
        };
        let good = VelocityCommand {
            linear_x: 0.14, // below cap AND accel-reachable: cap effect is
            angular_z: 0.0, // observed via is_degraded(), not clipping
            confidence: 0.9,
        };

        arb.arbitrate(&low, None, 0.1);
        assert!(arb.is_degraded());

        // K-1 good cycles: still degraded.
        for i in 0..k - 1 {
            arb.arbitrate(&good, None, 0.1);
            assert!(arb.is_degraded(), "cap lifted too early at good cycle {i}");
        }
        // K-th good cycle lifts the latch.
        arb.arbitrate(&good, None, 0.1);
        assert!(!arb.is_degraded(), "cap not lifted after {k} good cycles");

        // A single low-confidence cycle re-enters immediately (asymmetric).
        arb.arbitrate(&low, None, 0.1);
        assert!(arb.is_degraded());

        // ... and an intervening low cycle resets the recovery streak.
        for _ in 0..k - 1 {
            arb.arbitrate(&good, None, 0.1);
        }
        arb.arbitrate(&low, None, 0.1); // reset
        for _ in 0..k - 1 {
            arb.arbitrate(&good, None, 0.1);
            assert!(arb.is_degraded(), "streak must restart after a low cycle");
        }
    }

    #[test]
    fn test_safety_envelope_symmetric_for_reverse() {
        // Recovery commands reverse speeds. The envelope must treat them
        // symmetrically: speeding up in reverse is accel-limited, braking out
        // of reverse is decel-limited (NOT accel-limited — that would keep
        // the robot rolling backward for seconds after a stop command).
        //
        // Pinned to an EXPLICIT envelope where one decel step cannot cross
        // zero from the test speeds (accel 0.5, decel 1.0): with the raised
        // defaults both limits exceed the 0.1 m/s crawl in a single cycle and
        // the accel-vs-decel asymmetry would no longer be observable.
        let cfg = ArbitratorConfig {
            safety: SafetyEnvelopeConfig {
                max_speed: 1.0,
                max_acceleration: 0.5,
                max_deceleration: 1.0,
                ..Default::default()
            },
            ..Default::default()
        };
        let max_dv_accel = cfg.safety.max_acceleration * 0.1; // 0.05
        let max_dv_decel = cfg.safety.max_deceleration * 0.1; // 0.10
        let mut arb = Arbitrator::new(cfg.clone());

        // From rest, command -0.1 m/s: accel-limited into reverse.
        let reverse = VelocityCommand {
            linear_x: -0.1,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let out = arb.arbitrate(&reverse, None, 0.1);
        assert!(out.command.linear_x < 0.0, "reverse must pass the envelope");
        assert!(
            out.command.linear_x >= -max_dv_accel - 1e-9,
            "reverse spin-up {} exceeds accel limit {}",
            out.command.linear_x,
            max_dv_accel
        );

        // Reach steady reverse.
        for _ in 0..5 {
            arb.arbitrate(&reverse, None, 0.1);
        }
        assert!((arb.prev_linear + 0.1).abs() < 1e-9);

        // Command 0 from reverse: braking, decel-limited (one 0.1 step).
        let stop = VelocityCommand {
            linear_x: 0.0,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let out = arb.arbitrate(&stop, None, 0.1);
        assert!(
            out.command.linear_x >= -0.1 + max_dv_decel - 1e-9,
            "braking out of reverse {} slower than decel limit allows",
            out.command.linear_x
        );

        // Reverse speed magnitude is clamped to max_speed.
        let mut arb = Arbitrator::new(cfg.clone());
        arb.prev_linear = -1.0;
        let fast_reverse = VelocityCommand {
            linear_x: -5.0,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let out = arb.arbitrate(&fast_reverse, None, 0.1);
        assert!(out.command.linear_x >= -1.0 - 1e-9);
        assert!(out.safety_clipped);

        // Curvature clamp is symmetric: |w| <= max_curvature * |v| in reverse.
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.prev_linear = -0.1;
        let turning_reverse = VelocityCommand {
            linear_x: -0.1,
            angular_z: 1.0,
            confidence: 0.9,
        };
        let out = arb.arbitrate(&turning_reverse, None, 0.1);
        assert!(
            out.command.angular_z.abs() <= 2.0 * out.command.linear_x.abs() + 1e-9,
            "curvature clamp must use |v| for reverse"
        );
    }

    // ---- encode_control_command ----

    #[test]
    fn encode_estop_sets_emergency_stop_and_zero_velocity() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        let out = arb.emergency_stop();
        let wire = encode_control_command(&out, 7, 1_000);

        assert!(wire.emergency_stop);
        assert_eq!(wire.header.as_ref().unwrap().sequence, 7);
        assert_eq!(wire.header.as_ref().unwrap().timestamp_ns, 1_000);
        match wire.command.unwrap() {
            limo_proto::control_command::Command::VelocityCmd(v) => {
                assert_eq!(v.linear_x, 0.0);
                assert_eq!(v.angular_z, 0.0);
            }
            _ => panic!("expected VelocityCmd"),
        }
    }

    #[test]
    fn encode_shadow_mode_maps_to_traditional_source_on_wire() {
        let out = ArbitratorOutput {
            command: VelocityCommand {
                linear_x: 0.3,
                angular_z: 0.1,
                confidence: 0.9,
            },
            source: PipelineMode::Shadow,
            emergency_stop: false,
            safety_clipped: false,
        };
        let wire = encode_control_command(&out, 0, 0);
        assert_eq!(
            wire.source,
            limo_proto::PipelineSource::SourceTraditional as i32
        );
    }

    #[test]
    fn encode_e2e_mode_maps_to_e2e_source_on_wire() {
        let out = ArbitratorOutput {
            command: VelocityCommand {
                linear_x: 0.4,
                angular_z: 0.0,
                confidence: 0.95,
            },
            source: PipelineMode::E2E,
            emergency_stop: false,
            safety_clipped: false,
        };
        let wire = encode_control_command(&out, 0, 0);
        assert_eq!(wire.source, limo_proto::PipelineSource::SourceE2e as i32);
    }

    // ---- Config validation ----

    #[test]
    fn validate_rejects_nonpositive_max_speed() {
        let cfg = SafetyEnvelopeConfig {
            max_speed: 0.0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
        let cfg = SafetyEnvelopeConfig {
            max_speed: -1.0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_rejects_nonpositive_acceleration() {
        let cfg = SafetyEnvelopeConfig {
            max_acceleration: 0.0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_rejects_nan_curvature() {
        let cfg = SafetyEnvelopeConfig {
            max_curvature: f64::NAN,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_accepts_default() {
        assert!(SafetyEnvelopeConfig::default().validate().is_ok());
        assert!(ArbitratorConfig::default().validate().is_ok());
    }

    #[test]
    fn validate_rejects_confidence_out_of_range() {
        let cfg = ArbitratorConfig {
            e2e_confidence_threshold: 1.5,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
        let cfg = ArbitratorConfig {
            fallback_min_confidence: -0.1,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_rejects_bad_degraded_response_config() {
        let cfg = ArbitratorConfig {
            degraded_speed_cap: 0.0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
        let cfg = ArbitratorConfig {
            degraded_speed_cap: 5.0, // above safety.max_speed
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
        let cfg = ArbitratorConfig {
            confidence_recovery_cycles: 0,
            ..Default::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn encode_preserves_confidence_and_velocity() {
        let out = ArbitratorOutput {
            command: VelocityCommand {
                linear_x: 0.6,
                angular_z: -0.2,
                confidence: 0.8,
            },
            source: PipelineMode::Traditional,
            emergency_stop: false,
            safety_clipped: false,
        };
        let wire = encode_control_command(&out, 42, 999);
        assert!((wire.confidence - 0.8).abs() < 1e-6);
        match wire.command.unwrap() {
            limo_proto::control_command::Command::VelocityCmd(v) => {
                assert!((v.linear_x - 0.6).abs() < 1e-9);
                assert!((v.angular_z - (-0.2)).abs() < 1e-9);
                assert_eq!(v.linear_y, 0.0);
            }
            _ => panic!("expected VelocityCmd"),
        }
    }

    #[test]
    fn test_shadow_mode_uses_traditional_and_tags_source() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::Shadow);

        let traditional = VelocityCommand {
            linear_x: 0.3,
            angular_z: 0.0,
            confidence: 0.9,
        };
        let e2e = VelocityCommand {
            linear_x: 0.8,
            angular_z: 0.5,
            confidence: 0.95,
        };

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        // Safety envelope may clip acceleration; check source + that e2e did not flow through.
        assert_eq!(out.source, PipelineMode::Shadow);
        assert!(!out.emergency_stop);
        assert!(out.command.linear_x <= 0.3 + 1e-6);
        assert_eq!(out.command.angular_z, 0.0);
    }
}
