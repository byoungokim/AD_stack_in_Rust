/// Pipeline arbitrator: selects between traditional and E2E outputs,
/// applies safety envelope constraints.
///
/// Always runs as the final gate before publishing ControlCommand on CH2.
/// Ensures no command violates physical limits regardless of source.
use serde::Deserialize;

use crate::local_planner::VelocityCommand;

/// Pipeline mode (matches proto PipelineMode).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
    /// fallback from E2E. If both E2E and traditional are below threshold,
    /// the arbitrator emits an emergency stop instead of propagating a
    /// low-confidence command.
    #[serde(default = "default_fallback_min_confidence")]
    pub fallback_min_confidence: f32,
}

fn default_rate_hz() -> u32 { 10 }
fn default_e2e_confidence_threshold() -> f32 { 0.7 }
fn default_fallback_min_confidence() -> f32 { 0.3 }

impl Default for ArbitratorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            safety: SafetyEnvelopeConfig::default(),
            e2e_confidence_threshold: default_e2e_confidence_threshold(),
            fallback_min_confidence: default_fallback_min_confidence(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct SafetyEnvelopeConfig {
    #[serde(default = "default_max_speed")]
    pub max_speed: f64,           // m/s
    #[serde(default = "default_max_accel")]
    pub max_acceleration: f64,    // m/s^2
    #[serde(default = "default_max_decel")]
    pub max_deceleration: f64,    // m/s^2
    #[serde(default = "default_max_angular")]
    pub max_angular_speed: f64,   // rad/s
    #[serde(default = "default_max_curvature")]
    pub max_curvature: f64,       // 1/m
}

fn default_max_speed() -> f64 { 1.0 }
fn default_max_accel() -> f64 { 0.5 }
fn default_max_decel() -> f64 { 1.0 }
fn default_max_angular() -> f64 { 1.5 }
fn default_max_curvature() -> f64 { 2.0 }

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
            return Err(format!("safety.max_speed must be > 0, got {}", self.max_speed));
        }
        if !(self.max_acceleration > 0.0 && self.max_acceleration.is_finite()) {
            return Err(format!("safety.max_acceleration must be > 0, got {}", self.max_acceleration));
        }
        if !(self.max_deceleration > 0.0 && self.max_deceleration.is_finite()) {
            return Err(format!("safety.max_deceleration must be > 0, got {}", self.max_deceleration));
        }
        if !(self.max_angular_speed > 0.0 && self.max_angular_speed.is_finite()) {
            return Err(format!("safety.max_angular_speed must be > 0, got {}", self.max_angular_speed));
        }
        if !(self.max_curvature > 0.0 && self.max_curvature.is_finite()) {
            return Err(format!("safety.max_curvature must be > 0, got {}", self.max_curvature));
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

pub struct Arbitrator {
    config: ArbitratorConfig,
    mode: PipelineMode,
    prev_linear: f64,
}

impl Arbitrator {
    pub fn new(config: ArbitratorConfig) -> Self {
        Self {
            config,
            mode: PipelineMode::Traditional,
            prev_linear: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: PipelineMode) {
        self.mode = mode;
    }

    // Public getter, symmetric with set_mode; used by status publisher.
    #[allow(dead_code)]
    pub fn mode(&self) -> PipelineMode {
        self.mode
    }

    /// Select between traditional and E2E commands, apply safety envelope.
    /// In E2E mode, falls back to traditional when E2E confidence is below
    /// `e2e_confidence_threshold`; emits an emergency stop when the fallback
    /// itself is below `fallback_min_confidence`.
    pub fn arbitrate(
        &mut self,
        traditional: &VelocityCommand,
        e2e: Option<&VelocityCommand>,
        dt: f64,
    ) -> ArbitratorOutput {
        let fallback_or_estop = |arb: &mut Self| -> Option<ArbitratorOutput> {
            if traditional.confidence < arb.config.fallback_min_confidence {
                Some(arb.emergency_stop())
            } else {
                None
            }
        };

        let (raw_cmd, source) = match self.mode {
            PipelineMode::Traditional => (traditional.clone(), PipelineMode::Traditional),

            PipelineMode::E2E => {
                let e2e_usable = e2e
                    .filter(|c| c.confidence >= self.config.e2e_confidence_threshold);
                match e2e_usable {
                    Some(c) => (c.clone(), PipelineMode::E2E),
                    None => {
                        if let Some(estop) = fallback_or_estop(self) {
                            return estop;
                        }
                        (traditional.clone(), PipelineMode::Traditional)
                    }
                }
            }

            PipelineMode::Shadow => {
                // Shadow: traditional controls; E2E ran in parallel for logging.
                (traditional.clone(), PipelineMode::Shadow)
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
        let mut v = cmd.linear_x.clamp(-self.config.safety.max_speed, self.config.safety.max_speed);
        if v != cmd.linear_x {
            clipped = true;
        }

        // Clamp acceleration/deceleration
        let dv = v - self.prev_linear;
        let max_dv_accel = self.config.safety.max_acceleration * dt;
        let max_dv_decel = self.config.safety.max_deceleration * dt;

        if dv > max_dv_accel {
            v = self.prev_linear + max_dv_accel;
            clipped = true;
        } else if dv < -max_dv_decel {
            v = self.prev_linear - max_dv_decel;
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
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        let cmd = VelocityCommand { linear_x: 5.0, angular_z: 0.0, confidence: 0.9 };

        let out = arb.arbitrate(&cmd, None, 0.1);
        assert!(out.command.linear_x <= 1.0);
        assert!(out.safety_clipped);
    }

    #[test]
    fn test_safety_clamp_acceleration() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.prev_linear = 0.0;

        // Try to jump from 0 to 1 m/s in 0.1s (needs 10 m/s^2, limit is 0.5)
        let cmd = VelocityCommand { linear_x: 1.0, angular_z: 0.0, confidence: 0.9 };
        let out = arb.arbitrate(&cmd, None, 0.1);

        assert!(out.command.linear_x <= 0.05 + 1e-6); // max_accel * dt = 0.5 * 0.1
        assert!(out.safety_clipped);
    }

    #[test]
    fn test_e2e_fallback_low_confidence() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand { linear_x: 0.5, angular_z: 0.1, confidence: 0.9 };
        let e2e = VelocityCommand { linear_x: 0.8, angular_z: 0.2, confidence: 0.3 }; // low

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        assert_eq!(out.source, PipelineMode::Traditional); // should fallback
    }

    #[test]
    fn test_e2e_accepted_high_confidence() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand { linear_x: 0.3, angular_z: 0.0, confidence: 0.9 };
        let e2e = VelocityCommand { linear_x: 0.4, angular_z: 0.1, confidence: 0.95 };

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

        let traditional = VelocityCommand { linear_x: 0.5, angular_z: 0.1, confidence: 0.2 };
        let e2e = VelocityCommand { linear_x: 0.8, angular_z: 0.2, confidence: 0.3 };

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        assert!(out.emergency_stop);
        assert_eq!(out.command.linear_x, 0.0);
        assert_eq!(out.command.angular_z, 0.0);
    }

    #[test]
    fn test_e2e_mode_no_e2e_low_traditional_emergency_stops() {
        let mut arb = Arbitrator::new(ArbitratorConfig::default());
        arb.set_mode(PipelineMode::E2E);

        let traditional = VelocityCommand { linear_x: 0.5, angular_z: 0.1, confidence: 0.1 };
        let out = arb.arbitrate(&traditional, None, 0.1);
        assert!(out.emergency_stop);
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
            command: VelocityCommand { linear_x: 0.3, angular_z: 0.1, confidence: 0.9 },
            source: PipelineMode::Shadow,
            emergency_stop: false,
            safety_clipped: false,
        };
        let wire = encode_control_command(&out, 0, 0);
        assert_eq!(wire.source, limo_proto::PipelineSource::SourceTraditional as i32);
    }

    #[test]
    fn encode_e2e_mode_maps_to_e2e_source_on_wire() {
        let out = ArbitratorOutput {
            command: VelocityCommand { linear_x: 0.4, angular_z: 0.0, confidence: 0.95 },
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
        let cfg = SafetyEnvelopeConfig { max_speed: 0.0, ..Default::default() };
        assert!(cfg.validate().is_err());
        let cfg = SafetyEnvelopeConfig { max_speed: -1.0, ..Default::default() };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_rejects_nonpositive_acceleration() {
        let cfg = SafetyEnvelopeConfig { max_acceleration: 0.0, ..Default::default() };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_rejects_nan_curvature() {
        let cfg = SafetyEnvelopeConfig { max_curvature: f64::NAN, ..Default::default() };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validate_accepts_default() {
        assert!(SafetyEnvelopeConfig::default().validate().is_ok());
        assert!(ArbitratorConfig::default().validate().is_ok());
    }

    #[test]
    fn validate_rejects_confidence_out_of_range() {
        let cfg = ArbitratorConfig { e2e_confidence_threshold: 1.5, ..Default::default() };
        assert!(cfg.validate().is_err());
        let cfg = ArbitratorConfig { fallback_min_confidence: -0.1, ..Default::default() };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn encode_preserves_confidence_and_velocity() {
        let out = ArbitratorOutput {
            command: VelocityCommand { linear_x: 0.6, angular_z: -0.2, confidence: 0.8 },
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

        let traditional = VelocityCommand { linear_x: 0.3, angular_z: 0.0, confidence: 0.9 };
        let e2e = VelocityCommand { linear_x: 0.8, angular_z: 0.5, confidence: 0.95 };

        let out = arb.arbitrate(&traditional, Some(&e2e), 0.1);
        // Safety envelope may clip acceleration; check source + that e2e did not flow through.
        assert_eq!(out.source, PipelineMode::Shadow);
        assert!(!out.emergency_stop);
        assert!(out.command.linear_x <= 0.3 + 1e-6);
        assert_eq!(out.command.angular_z, 0.0);
    }
}
