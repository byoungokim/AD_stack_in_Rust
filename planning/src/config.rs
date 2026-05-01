/// Configuration for the Planning process.
use serde::Deserialize;

use crate::arbitrator::ArbitratorConfig;
use crate::behavior::BehaviorConfig;
use crate::e2e::E2EConfig;
use crate::global_planner::HybridAStarConfig;
use crate::local_planner::LocalPlannerConfig;

#[derive(Debug, Clone, Default, Deserialize)]
pub struct PlanningConfig {
    #[serde(default)]
    pub behavior: BehaviorConfig,
    #[serde(default)]
    pub global_planner: HybridAStarConfig,
    #[serde(default)]
    pub local_planner: LocalPlannerConfig,
    #[serde(default)]
    pub arbitrator: ArbitratorConfig,
    #[serde(default)]
    pub e2e: E2EConfig,
    #[serde(default)]
    pub transport: TransportConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct TransportConfig {
    #[serde(default = "default_ch1_endpoint")]
    pub ch1_endpoint: String, // subscribe WorldState
    #[serde(default = "default_ch2_endpoint")]
    pub ch2_endpoint: String, // publish ControlCommand
    #[serde(default = "default_ch3_endpoint")]
    pub ch3_endpoint: String, // subscribe VehicleState
}

fn default_ch1_endpoint() -> String {
    "tcp://localhost:5551".into()
}
fn default_ch2_endpoint() -> String {
    "tcp://*:5552".into()
}
fn default_ch3_endpoint() -> String {
    "tcp://localhost:5553".into()
}

impl Default for TransportConfig {
    fn default() -> Self {
        Self {
            ch1_endpoint: default_ch1_endpoint(),
            ch2_endpoint: default_ch2_endpoint(),
            ch3_endpoint: default_ch3_endpoint(),
        }
    }
}

pub fn load_config(path: &str) -> anyhow::Result<PlanningConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: PlanningConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}
