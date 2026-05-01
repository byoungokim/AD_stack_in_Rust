pub mod channels;
pub mod heartbeat;
/// Limo Drive ZMQ transport layer.
///
/// Provides type-safe ZMQ PUB/SUB wrappers that serialize/deserialize
/// Protobuf messages. Used for inter-process communication between
/// the 3 main processes.
///
/// # Channels
/// - CH0 (tcp:5570-5572): Heartbeat (per-process ports)
/// - CH1 (tcp:5551): WorldState (SensPerc → Planning)
/// - CH2 (tcp:5552): ControlCommand (Planning → Control)
/// - CH3 (tcp:5553): VehicleState (Control → SensPerc, Planning)
/// - CH4 (tcp:5554): SensorSnapshot (SensPerc → Planning, E2E only)
/// - CH5-CH7 (tcp:5560-5562): Isaac Sim bridge
pub mod publisher;
pub mod subscriber;

pub use channels::{Channel, ChannelConfig};
pub use heartbeat::{HeartbeatManager, PeerHealth, PeerStatus};
pub use publisher::Publisher;
pub use subscriber::Subscriber;
