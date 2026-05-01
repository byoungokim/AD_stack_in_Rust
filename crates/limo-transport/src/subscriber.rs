/// ZMQ SUB socket wrapper with Protobuf deserialization.
///
/// Connects to a publisher endpoint and receives Protobuf messages
/// filtered by topic. Supports both blocking and non-blocking receive,
/// as well as a background thread mode with a crossbeam channel.
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};
use crossbeam_channel::{self, Receiver, Sender};
use prost::Message;
use tracing::{debug, info, warn};

/// A ZMQ subscriber that receives Protobuf messages on a topic.
pub struct Subscriber {
    socket: zmq::Socket,
    topic: String,
    endpoint: String,
    msg_count: u64,
}

impl Subscriber {
    /// Create and connect a new subscriber.
    ///
    /// # Arguments
    /// * `ctx` - ZMQ context
    /// * `endpoint` - Connect endpoint, e.g., "tcp://localhost:5553"
    /// * `topic` - Topic filter, e.g., "vehicle_state"
    pub fn connect(ctx: &zmq::Context, endpoint: &str, topic: &str) -> Result<Self> {
        let socket = ctx
            .socket(zmq::SUB)
            .context("Failed to create SUB socket")?;

        socket.set_rcvhwm(100).context("Failed to set RCVHWM")?;

        socket.set_linger(0).context("Failed to set linger")?;

        socket
            .connect(endpoint)
            .context(format!("Failed to connect SUB socket to {}", endpoint))?;

        socket
            .set_subscribe(topic.as_bytes())
            .context(format!("Failed to subscribe to topic '{}'", topic))?;

        info!("Subscriber connected: {} [topic='{}']", endpoint, topic);

        Ok(Self {
            socket,
            topic: topic.to_string(),
            endpoint: endpoint.to_string(),
            msg_count: 0,
        })
    }

    /// Receive a message with a timeout. Returns None on timeout.
    ///
    /// Blocks up to `timeout` waiting for a message. Deserializes the
    /// Protobuf payload into type `M`.
    pub fn recv<M: Message + Default>(&mut self, timeout: Duration) -> Result<Option<M>> {
        self.socket
            .set_rcvtimeo(timeout.as_millis() as i32)
            .context("Failed to set RCVTIMEO")?;

        // Receive topic frame
        let topic_result = self.socket.recv_bytes(0);
        match topic_result {
            Err(zmq::Error::EAGAIN) => return Ok(None), // timeout
            Err(e) => return Err(e).context("Failed to receive topic frame"),
            Ok(_topic_bytes) => {}
        }

        // Receive data frame
        let data = self
            .socket
            .recv_bytes(0)
            .context("Failed to receive data frame")?;

        let msg = M::decode(data.as_slice()).context("Failed to decode Protobuf message")?;

        self.msg_count += 1;
        if self.msg_count.is_multiple_of(1000) {
            debug!(
                "Subscriber [{}] received {} messages on '{}'",
                self.endpoint, self.msg_count, self.topic
            );
        }

        Ok(Some(msg))
    }

    /// Receive a message, blocking indefinitely.
    pub fn recv_blocking<M: Message + Default>(&mut self) -> Result<M> {
        self.socket
            .set_rcvtimeo(-1)
            .context("Failed to set RCVTIMEO")?;

        let _topic = self
            .socket
            .recv_bytes(0)
            .context("Failed to receive topic frame")?;
        let data = self
            .socket
            .recv_bytes(0)
            .context("Failed to receive data frame")?;

        let msg = M::decode(data.as_slice()).context("Failed to decode Protobuf message")?;

        self.msg_count += 1;
        Ok(msg)
    }

    pub fn msg_count(&self) -> u64 {
        self.msg_count
    }

    pub fn topic(&self) -> &str {
        &self.topic
    }
}

/// A background subscriber that receives messages in a dedicated thread
/// and delivers them via a crossbeam channel.
///
/// This is useful for processes that need non-blocking access to the
/// latest messages without managing ZMQ sockets in the main loop.
pub struct BackgroundSubscriber<M: Message + Default + Send + 'static> {
    receiver: Receiver<M>,
    _handle: thread::JoinHandle<()>,
}

impl<M: Message + Default + Send + 'static> BackgroundSubscriber<M> {
    /// Start a background subscriber thread.
    ///
    /// Messages are delivered via the returned receiver. The channel has
    /// a bounded capacity; oldest messages are dropped if the consumer
    /// is too slow.
    pub fn start(ctx: &zmq::Context, endpoint: &str, topic: &str, capacity: usize) -> Result<Self> {
        let (tx, rx): (Sender<M>, Receiver<M>) = crossbeam_channel::bounded(capacity);

        let mut sub = Subscriber::connect(ctx, endpoint, topic)?;

        let handle = thread::Builder::new()
            .name(format!("sub-{}", topic))
            .spawn(move || {
                loop {
                    match sub.recv::<M>(Duration::from_millis(100)) {
                        Ok(Some(msg)) => {
                            // Use try_send to avoid blocking if channel is full
                            if tx.try_send(msg).is_err() {
                                // Channel full or disconnected
                                if tx.is_empty() {
                                    // Disconnected, stop
                                    break;
                                }
                                // Full — message dropped (acceptable for sensor data)
                            }
                        }
                        Ok(None) => {
                            // Timeout — keep polling.
                            continue;
                        }
                        Err(e) => {
                            warn!("Background subscriber error: {:#}", e);
                            break;
                        }
                    }
                }
            })
            .context("Failed to spawn background subscriber thread")?;

        Ok(Self {
            receiver: rx,
            _handle: handle,
        })
    }

    /// Try to receive the latest message (non-blocking).
    /// Drains the channel and returns the most recent message.
    pub fn try_recv_latest(&self) -> Option<M> {
        let mut latest = None;
        while let Ok(msg) = self.receiver.try_recv() {
            latest = Some(msg);
        }
        latest
    }

    /// Try to receive one message (non-blocking).
    pub fn try_recv(&self) -> Option<M> {
        self.receiver.try_recv().ok()
    }

    /// Receive one message, blocking up to timeout.
    pub fn recv_timeout(&self, timeout: Duration) -> Option<M> {
        self.receiver.recv_timeout(timeout).ok()
    }

    /// Get the crossbeam receiver for custom select! usage.
    pub fn receiver(&self) -> &Receiver<M> {
        &self.receiver
    }
}
