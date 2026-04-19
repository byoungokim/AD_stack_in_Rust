/// End-to-end autonomous driving inference stub.
///
/// Placeholder for a neural network model (ONNX/TensorRT) that takes
/// raw sensor data and produces control commands directly.
/// Will use tch-rs or ort crate for inference when model is available.
use serde::Deserialize;

use crate::local_planner::VelocityCommand;

// Config fields loaded from YAML; rate_hz/confidence_threshold feed the inference loop once the model lands.
#[allow(dead_code)]
#[derive(Debug, Clone, Deserialize)]
pub struct E2EConfig {
    #[serde(default = "default_enabled")]
    pub enabled: bool,
    #[serde(default = "default_model_path")]
    pub model_path: String,
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default = "default_confidence_threshold")]
    pub confidence_threshold: f32,
}

fn default_enabled() -> bool { false }
fn default_model_path() -> String { "models/e2e_driving.onnx".into() }
fn default_rate_hz() -> u32 { 15 }
fn default_confidence_threshold() -> f32 { 0.7 }

impl Default for E2EConfig {
    fn default() -> Self {
        Self {
            enabled: default_enabled(),
            model_path: default_model_path(),
            rate_hz: default_rate_hz(),
            confidence_threshold: default_confidence_threshold(),
        }
    }
}

/// E2E inference engine (stub).
pub struct E2EInference {
    config: E2EConfig,
}

impl E2EInference {
    pub fn new(config: E2EConfig) -> Self {
        if config.enabled {
            tracing::info!("E2E inference enabled (model: {})", config.model_path);
            // TODO: Load ONNX/TensorRT model via ort or tch-rs
        } else {
            tracing::info!("E2E inference disabled");
        }
        Self { config }
    }

    pub fn is_enabled(&self) -> bool {
        self.config.enabled
    }

    /// Run inference on raw sensor data. Returns None if disabled or model not loaded.
    pub fn infer(&self, _sensor_data: &[u8]) -> Option<VelocityCommand> {
        if !self.config.enabled {
            return None;
        }

        // TODO: Actual model inference
        // For now, return None (no model loaded)
        None
    }
}
