/// Channel definitions and configuration.
///
/// Encodes the 5-channel architecture from the system design.

/// Logical channel identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    /// CH0: Heartbeat bus (all processes)
    Heartbeat,
    /// CH1: WorldState (SensPerc → Planning)
    WorldState,
    /// CH2: ControlCommand (Planning → Control)
    ControlCommand,
    /// CH3: VehicleState (Control → SensPerc + Planning)
    VehicleState,
    /// CH4: SensorSnapshot (SensPerc → Planning, E2E/shadow only)
    SensorSnapshot,
}

impl Channel {
    /// Default bind endpoint for the publisher side.
    pub fn bind_endpoint(&self) -> &'static str {
        match self {
            Channel::Heartbeat => "tcp://*:5550",
            Channel::WorldState => "tcp://*:5551",
            Channel::ControlCommand => "tcp://*:5552",
            Channel::VehicleState => "tcp://*:5553",
            Channel::SensorSnapshot => "tcp://*:5554",
        }
    }

    /// Default connect endpoint for the subscriber side.
    pub fn connect_endpoint(&self) -> &'static str {
        match self {
            Channel::Heartbeat => "tcp://localhost:5550",
            Channel::WorldState => "tcp://localhost:5551",
            Channel::ControlCommand => "tcp://localhost:5552",
            Channel::VehicleState => "tcp://localhost:5553",
            Channel::SensorSnapshot => "tcp://localhost:5554",
        }
    }

    /// Topic prefix used for ZMQ subscription filtering.
    pub fn topic(&self) -> &'static str {
        match self {
            Channel::Heartbeat => "heartbeat",
            Channel::WorldState => "world_state",
            Channel::ControlCommand => "control_cmd",
            Channel::VehicleState => "vehicle_state",
            Channel::SensorSnapshot => "sensor_snapshot",
        }
    }
}

/// Runtime channel configuration (overrides defaults from YAML config).
#[derive(Debug, Clone)]
pub struct ChannelConfig {
    pub channel: Channel,
    pub endpoint: String,
    pub topic: String,
}

impl ChannelConfig {
    pub fn new(channel: Channel) -> Self {
        Self {
            endpoint: channel.bind_endpoint().to_string(),
            topic: channel.topic().to_string(),
            channel,
        }
    }

    pub fn with_endpoint(mut self, endpoint: &str) -> Self {
        self.endpoint = endpoint.to_string();
        self
    }
}
