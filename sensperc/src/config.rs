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
    #[serde(default)]
    pub lidar_mount: LidarMountConfig,
}

/// Planar mount pose of the lidar on the chassis (base_link → lidar).
///
/// Lidar ranges are measured from the sensor origin, not from base_link
/// center: an unmodeled mount offset displaces EVERY projected obstacle by
/// exactly that offset in the world frame (0.2-0.6 m class errors).
///
/// Defaults are zero because the gz model
/// (simulation/models/limo_pro/model.sdf, `<sensor name="lidar">` pose
/// `0.0 0 0.15`) mounts the lidar at the base center. Set x/y/yaw here for
/// hardware where the scanner sits off-center.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct LidarMountConfig {
    /// Forward offset from base_link center (meters).
    pub x: f64,
    /// Left offset from base_link center (meters).
    pub y: f64,
    /// Mount yaw relative to the chassis forward axis (radians).
    pub yaw: f64,
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
        // Sim default: lidar mounted at base center (matches model.sdf).
        assert_eq!(cfg.lidar_mount.x, 0.0);
        assert_eq!(cfg.lidar_mount.y, 0.0);
        assert_eq!(cfg.lidar_mount.yaw, 0.0);
    }

    #[test]
    fn parses_lidar_mount_partial() {
        let yaml = r#"
lidar_mount:
  x: 0.105
"#;
        let cfg: SensPercConfig = serde_yaml::from_str(yaml).unwrap();
        assert!((cfg.lidar_mount.x - 0.105).abs() < 1e-9);
        assert_eq!(cfg.lidar_mount.y, 0.0);
        assert_eq!(cfg.lidar_mount.yaw, 0.0);
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
