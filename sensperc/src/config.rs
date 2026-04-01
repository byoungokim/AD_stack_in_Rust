/// Configuration for the SensPerc process.
use serde::Deserialize;

use crate::drivers::camera::CameraConfig;
use crate::drivers::imu::ImuConfig;
use crate::drivers::lidar::LidarConfig;

#[derive(Debug, Clone, Deserialize)]
pub struct SensPercConfig {
    #[serde(default)]
    pub camera: CameraConfig,
    #[serde(default)]
    pub lidar: LidarConfig,
    #[serde(default)]
    pub imu: ImuConfig,
    #[serde(default)]
    pub aggregator: AggregatorConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct AggregatorConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
    #[serde(default = "default_ch1_endpoint")]
    pub ch1_endpoint: String,
    #[serde(default = "default_ch4_endpoint")]
    pub ch4_endpoint: String,
}

fn default_rate_hz() -> u32 { 10 }
fn default_ch1_endpoint() -> String { "tcp://*:5551".into() }
fn default_ch4_endpoint() -> String { "tcp://*:5554".into() }

impl Default for AggregatorConfig {
    fn default() -> Self {
        Self {
            rate_hz: default_rate_hz(),
            ch1_endpoint: default_ch1_endpoint(),
            ch4_endpoint: default_ch4_endpoint(),
        }
    }
}

impl Default for SensPercConfig {
    fn default() -> Self {
        Self {
            camera: CameraConfig::default(),
            lidar: LidarConfig::default(),
            imu: ImuConfig::default(),
            aggregator: AggregatorConfig::default(),
        }
    }
}

/// Load SensPerc config from YAML file.
pub fn load_config(path: &str) -> anyhow::Result<SensPercConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: SensPercConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}
