/// Configuration for the Control process.
use serde::Deserialize;

use crate::chassis::ChassisConfig;
use crate::kinematics::KinematicsConfig;
use crate::tracker::TrackerConfig;
use crate::watchdog::WatchdogConfig;

#[derive(Debug, Clone, Deserialize)]
pub struct ControlConfig {
    #[serde(default)]
    pub chassis: ChassisConfig,
    #[serde(default)]
    pub tracker: TrackerConfig,
    #[serde(default)]
    pub kinematics: KinematicsConfig,
    #[serde(default)]
    pub watchdog: WatchdogConfig,
    #[serde(default)]
    pub transport: TransportConfig,
    #[serde(default)]
    pub state_publisher: StatePublisherConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct TransportConfig {
    #[serde(default = "default_ch2_endpoint")]
    pub ch2_endpoint: String, // subscribe ControlCommand
    #[serde(default = "default_ch3_endpoint")]
    pub ch3_endpoint: String, // publish VehicleState
}

fn default_ch2_endpoint() -> String { "tcp://localhost:5552".into() }
fn default_ch3_endpoint() -> String { "tcp://*:5553".into() }

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            ch2_endpoint: default_ch2_endpoint(),
            ch3_endpoint: default_ch3_endpoint(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct StatePublisherConfig {
    #[serde(default = "default_rate_hz")]
    pub rate_hz: u32,
}

fn default_rate_hz() -> u32 { 20 }

impl Default for StatePublisherConfig {
    fn default() -> Self {
        Self { rate_hz: default_rate_hz() }
    }
}

impl Default for ControlConfig {
    fn default() -> Self {
        Self {
            chassis: ChassisConfig::default(),
            tracker: TrackerConfig::default(),
            kinematics: KinematicsConfig::default(),
            watchdog: WatchdogConfig::default(),
            transport: TransportConfig::default(),
            state_publisher: StatePublisherConfig::default(),
        }
    }
}

pub fn load_config(path: &str) -> anyhow::Result<ControlConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: ControlConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}
