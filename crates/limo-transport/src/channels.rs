//! Channel definitions and configuration.
//!
//! Encodes the 8-channel architecture:
//! - CH0-CH4: Core inter-process channels
//! - CH5-CH7: Isaac Sim bridge channels (sim mode only)

/// Logical channel identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    // --- Core channels ---
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

    // --- Isaac Sim bridge channels ---
    /// CH5: SimSensorData (Isaac Sim → SensPerc)
    SimSensors,
    /// CH6: SimVehicleState (Isaac Sim → Control)
    SimVehicleState,
    /// CH7: SimControlCommand (Control → Isaac Sim)
    SimControl,

    // --- Scenario layer channels ---
    /// CH8: ScenarioCommand (Scenario Manager → Planning)
    ScenarioCommand,
    /// CH9: ScenarioStatus (Planning → Scenario Manager)
    ScenarioStatus,

    // --- Visualization channel ---
    /// CH10: PlannedPath (Planning → Visualizer)
    PlannedPath,
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
            Channel::SimSensors => "tcp://*:5560",
            Channel::SimVehicleState => "tcp://*:5561",
            Channel::SimControl => "tcp://*:5562",
            Channel::ScenarioCommand => "tcp://*:5580",
            Channel::ScenarioStatus => "tcp://*:5581",
            Channel::PlannedPath => "tcp://*:5590",
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
            Channel::SimSensors => "tcp://localhost:5560",
            Channel::SimVehicleState => "tcp://localhost:5561",
            Channel::SimControl => "tcp://localhost:5562",
            Channel::ScenarioCommand => "tcp://localhost:5580",
            Channel::ScenarioStatus => "tcp://localhost:5581",
            Channel::PlannedPath => "tcp://localhost:5590",
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
            Channel::SimSensors => "sim_sensors",
            Channel::SimVehicleState => "sim_vehicle_state",
            Channel::SimControl => "sim_control",
            Channel::ScenarioCommand => "scenario_cmd",
            Channel::ScenarioStatus => "scenario_status",
            Channel::PlannedPath => "planned_path",
        }
    }

    /// Whether this channel is only used in simulation mode.
    pub fn is_sim_channel(&self) -> bool {
        matches!(
            self,
            Channel::SimSensors | Channel::SimVehicleState | Channel::SimControl
        )
    }

    /// Whether this channel is part of the scenario layer.
    pub fn is_scenario_channel(&self) -> bool {
        matches!(self, Channel::ScenarioCommand | Channel::ScenarioStatus)
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
