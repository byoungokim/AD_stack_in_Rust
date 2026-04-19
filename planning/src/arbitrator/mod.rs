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

/// Output from the arbitrator.
#[derive(Debug, Clone)]
pub struct ArbitratorOutput {
    pub command: VelocityCommand,
    pub source: PipelineMode,
    pub emergency_stop: bool,
    pub safety_clipped: bool,
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
