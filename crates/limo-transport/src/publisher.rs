/// ZMQ PUB socket wrapper with Protobuf serialization.
///
/// Binds to an endpoint and publishes serialized Protobuf messages
/// with a topic prefix for subscriber filtering.
use anyhow::{Context, Result};
use prost::Message;
use tracing::{debug, info};

/// A ZMQ publisher that sends Protobuf messages on a topic.
pub struct Publisher {
    socket: zmq::Socket,
    topic: String,
    endpoint: String,
    msg_count: u64,
}

impl Publisher {
    /// Create and bind a new publisher.
    ///
    /// # Arguments
    /// * `ctx` - ZMQ context (share across threads for efficiency)
    /// * `endpoint` - Bind endpoint, e.g., "tcp://*:5553"
    /// * `topic` - Topic prefix for messages, e.g., "vehicle_state"
    pub fn bind(ctx: &zmq::Context, endpoint: &str, topic: &str) -> Result<Self> {
        let socket = ctx
            .socket(zmq::PUB)
            .context("Failed to create PUB socket")?;

        // Set a high-water mark to prevent unbounded memory growth
        // if subscribers are slow. Drop oldest messages when full.
        socket.set_sndhwm(100).context("Failed to set SNDHWM")?;

        // Linger: don't block on close, drop unsent messages immediately
        socket.set_linger(0).context("Failed to set linger")?;

        socket
            .bind(endpoint)
            .context(format!("Failed to bind PUB socket to {}", endpoint))?;

        info!("Publisher bound: {} [topic='{}']", endpoint, topic);

        Ok(Self {
            socket,
            topic: topic.to_string(),
            endpoint: endpoint.to_string(),
            msg_count: 0,
        })
    }

    /// Publish a Protobuf message.
    ///
    /// Wire format: multipart [topic_bytes, serialized_proto_bytes]
    pub fn publish<M: Message>(&mut self, msg: &M) -> Result<()> {
        let data = msg.encode_to_vec();

        self.socket
            .send(&self.topic, zmq::SNDMORE)
            .context("Failed to send topic frame")?;
        self.socket
            .send(&data, 0)
            .context("Failed to send data frame")?;

        self.msg_count += 1;
        if self.msg_count.is_multiple_of(1000) {
            debug!(
                "Publisher [{}] sent {} messages on '{}'",
                self.endpoint, self.msg_count, self.topic
            );
        }

        Ok(())
    }

    /// Number of messages published so far.
    pub fn msg_count(&self) -> u64 {
        self.msg_count
    }

    pub fn topic(&self) -> &str {
        &self.topic
    }
}
