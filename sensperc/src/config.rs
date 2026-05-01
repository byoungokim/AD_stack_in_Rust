/// Configuration for the SensPerc process.
/// Driver configs are now in limo-hal. This only covers the aggregator.
use serde::Deserialize;

use limo_hal::sim_zmq::SimFaultConfig;

#[derive(Debug, Clone, Default, Deserialize)]
pub struct SensPercConfig {
    #[serde(default)]
    pub aggregator: AggregatorConfig,
    #[serde(default)]
    pub sim_faults: SimFaultConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct AggregatorConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default = "default_ch1_endpoint")]
    pub ch1_endpoint: String,
}

fn default_rate_hz() -> u32 {
    10
}
fn default_ch1_endpoint() -> String {
    "tcp://*:5551".into()
}

impl Default for AggregatorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            ch1_endpoint: default_ch1_endpoint(),
        }
    }
}

pub fn load_config(path: &str) -> anyhow::Result<SensPercConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: SensPercConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_yaml_uses_defaults() {
        let cfg: SensPercConfig = serde_yaml::from_str("{}").unwrap();
        assert_eq!(cfg.aggregator.rate_hz, 10);
        assert!(!cfg.sim_faults.is_active());
    }

    #[test]
    fn parses_sim_faults_partial() {
        let yaml = r#"
aggregator:
  rate_hz: 20
sim_faults:
  camera_drop_rate: 0.1
  seed: 42
"#;
        let cfg: SensPercConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(cfg.aggregator.rate_hz, 20);
        assert!((cfg.sim_faults.camera_drop_rate - 0.1).abs() < 1e-6);
        assert_eq!(cfg.sim_faults.seed, 42);
        assert_eq!(cfg.sim_faults.lidar_drop_rate, 0.0);
        assert!(cfg.sim_faults.is_active());
    }
}
