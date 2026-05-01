/// Configuration for the Isaac Sim bridge.
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct SimBridgeConfig {
    // Ports for Isaac Sim to publish to
    #[serde(default = "default_ch5_port")]
    pub ch5_port: u16, // SimSensors bind port
    #[serde(default = "default_ch6_port")]
    pub ch6_port: u16, // SimVehicleState bind port

    // Endpoint for subscribing control commands
    #[serde(default = "default_ch7_connect")]
    pub ch7_endpoint_connect: String,
    // Bind endpoint for when sim bridge acts as publisher (future sim-as-server mode)
    #[allow(dead_code)]
    #[serde(default = "default_ch7_bind")]
    pub ch7_endpoint_bind: String,

    // Dummy sim settings
    #[serde(default)]
    pub dummy: DummySimConfig,
}

fn default_ch5_port() -> u16 {
    5560
}
fn default_ch6_port() -> u16 {
    5561
}
fn default_ch7_connect() -> String {
    "tcp://localhost:5562".into()
}
fn default_ch7_bind() -> String {
    "tcp://*:5562".into()
}

impl Default for SimBridgeConfig {
    fn default() -> Self {
        Self {
            ch5_port: default_ch5_port(),
            ch6_port: default_ch6_port(),
            ch7_endpoint_connect: default_ch7_connect(),
            ch7_endpoint_bind: default_ch7_bind(),
            dummy: DummySimConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct DummySimConfig {
    #[serde(default = "default_sensor_rate")]
    pub sensor_rate_hz: u32,
    #[serde(default = "default_state_rate")]
    pub state_rate_hz: u32,
    #[serde(default = "default_image_width")]
    pub image_width: u32,
    #[serde(default = "default_image_height")]
    pub image_height: u32,
    #[serde(default = "default_lidar_points")]
    pub lidar_num_points: u32,
}

fn default_sensor_rate() -> u32 {
    30
}
fn default_state_rate() -> u32 {
    20
}
fn default_image_width() -> u32 {
    640
}
fn default_image_height() -> u32 {
    480
}
fn default_lidar_points() -> u32 {
    360
}

impl Default for DummySimConfig {
    fn default() -> Self {
        Self {
            sensor_rate_hz: default_sensor_rate(),
            state_rate_hz: default_state_rate(),
            image_width: default_image_width(),
            image_height: default_image_height(),
            lidar_num_points: default_lidar_points(),
        }
    }
}

pub fn load_config(path: &str) -> anyhow::Result<SimBridgeConfig> {
    let contents = std::fs::read_to_string(path)?;
    let config: SimBridgeConfig = serde_yaml::from_str(&contents)?;
    Ok(config)
}
