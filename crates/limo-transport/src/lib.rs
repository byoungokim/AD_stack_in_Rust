/// Limo Drive ZMQ transport layer.
///
/// Provides type-safe ZMQ PUB/SUB wrappers that serialize/deserialize
/// Protobuf messages. Used for inter-process communication between
/// the 3 main processes.
///
/// # Channels
/// - CH0 (tcp:5560-5562): Heartbeat bus
/// - CH1 (tcp:5551): WorldState (SensPerc → Planning)
/// - CH2 (tcp:5552): ControlCommand (Planning → Control)
/// - CH3 (tcp:5553): VehicleState (Control → SensPerc, Planning)
/// - CH4 (tcp:5554): SensorSnapshot (SensPerc → Planning, E2E only)
pub mod publisher;
pub mod subscriber;
pub mod channels;

pub use publisher::Publisher;
pub use subscriber::Subscriber;
pub use channels::{Channel, ChannelConfig};
