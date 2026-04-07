/// Configuration for the SensPerc process.
/// Driver configs are now in limo-hal. This only covers the aggregator.
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct SensPercConfig {
    #[serde(default)]
    pub aggregator: AggregatorConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct AggregatorConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default = "default_ch1_endpoint")]
    pub ch1_endpoint: String,
}

fn default_rate_hz() -> u32 { 10 }
fn default_ch1_endpoint() -> String { "tcp://*:5551".into() }

impl Default for AggregatorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            ch1_endpoint: default_ch1_endpoint(),
        }
    }
}

impl Default for SensPercConfig {
    fn default() -> Self {
        Self {
            aggregator: AggregatorConfig::default(),
        }
    }
}

pub fn load_config(path: &str) -> anyhow::Result<SensPercConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: SensPercConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}
