/// Heartbeat manager: publishes own heartbeat and monitors peers.
///
/// Each process binds a PUB socket on its own port and subscribes
/// to all peer ports. Heartbeats are Protobuf `Heartbeat` messages
/// published at 10Hz.
///
/// Port assignment:
///   sensperc: 5570
///   planning: 5571
///   control:  5572
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use prost::Message;
use tracing::{debug, info, warn};

/// Peer health status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeerStatus {
    Nominal,
    Warn,
    Degraded,
    Dead,
    Unknown,
}

/// Port assignments for heartbeat channels.
pub fn heartbeat_port(process_name: &str) -> u16 {
    match process_name {
        "sensperc" => 5570,
        "planning" => 5571,
        "control" => 5572,
        _ => 5579, // fallback
    }
}

/// All known process names.
pub fn all_processes() -> &'static [&'static str] {
    &["sensperc", "planning", "control"]
}

/// Peers of a given process (everyone except self).
pub fn peers_of(process_name: &str) -> Vec<&'static str> {
    all_processes()
        .iter()
        .filter(|&&p| p != process_name)
        .copied()
        .collect()
}

/// Shared peer health state, safe to read from any thread.
#[derive(Clone)]
pub struct PeerHealth {
    inner: Arc<Mutex<HashMap<String, Instant>>>,
}

impl PeerHealth {
    fn new(peers: &[&str]) -> Self {
        let mut map = HashMap::new();
        let now = Instant::now();
        for &peer in peers {
            map.insert(peer.to_string(), now);
        }
        Self { inner: Arc::new(Mutex::new(map)) }
    }

    fn update(&self, peer: &str) {
        let mut map = self.inner.lock().unwrap();
        map.insert(peer.to_string(), Instant::now());
    }

    /// Get the status of a peer based on heartbeat age.
    pub fn status(&self, peer: &str) -> PeerStatus {
        let map = self.inner.lock().unwrap();
        match map.get(peer) {
            Some(last) => {
                let age = last.elapsed();
                if age < Duration::from_millis(200) {
                    PeerStatus::Nominal
                } else if age < Duration::from_millis(500) {
                    PeerStatus::Warn
                } else if age < Duration::from_millis(1000) {
                    PeerStatus::Degraded
                } else {
                    PeerStatus::Dead
                }
            }
            None => PeerStatus::Unknown,
        }
    }

    /// Get age of last heartbeat from a peer (seconds).
    pub fn age_secs(&self, peer: &str) -> f64 {
        let map = self.inner.lock().unwrap();
        match map.get(peer) {
            Some(last) => last.elapsed().as_secs_f64(),
            None => f64::INFINITY,
        }
    }

    /// Check if all peers are nominal.
    pub fn all_nominal(&self) -> bool {
        let map = self.inner.lock().unwrap();
        map.keys().all(|peer| {
            map.get(peer)
                .map(|last| last.elapsed() < Duration::from_millis(200))
                .unwrap_or(false)
        })
    }
}

/// Manages heartbeat publishing and peer monitoring.
/// Runs publisher and subscriber threads in the background.
pub struct HeartbeatManager {
    process_name: String,
    peer_health: PeerHealth,
    running: Arc<AtomicBool>,
    threads: Vec<thread::JoinHandle<()>>,
}

impl HeartbeatManager {
    /// Start the heartbeat manager for the given process.
    ///
    /// Spawns:
    /// - 1 publisher thread (10Hz heartbeat on own port)
    /// - 1 subscriber thread per peer (listens on peer's port)
    pub fn start(process_name: &str) -> Result<Self> {
        let peers = peers_of(process_name);
        let peer_health = PeerHealth::new(&peers);
        let running = Arc::new(AtomicBool::new(true));
        let mut threads = Vec::new();

        // --- Publisher thread ---
        let pub_port = heartbeat_port(process_name);
        let pub_name = process_name.to_string();
        let pub_running = Arc::clone(&running);

        let pub_handle = thread::Builder::new()
            .name(format!("hb-pub-{}", process_name))
            .spawn(move || {
                if let Err(e) = heartbeat_publish_loop(&pub_name, pub_port, &pub_running) {
                    warn!("Heartbeat publisher error: {:#}", e);
                }
            })
            .context("Failed to spawn heartbeat publisher")?;
        threads.push(pub_handle);

        // --- Subscriber threads (one per peer) ---
        for &peer in &peers {
            let peer_port = heartbeat_port(peer);
            let peer_name = peer.to_string();
            let health = peer_health.clone();
            let sub_running = Arc::clone(&running);

            let sub_handle = thread::Builder::new()
                .name(format!("hb-sub-{}", peer))
                .spawn(move || {
                    if let Err(e) = heartbeat_subscribe_loop(
                        &peer_name, peer_port, &health, &sub_running,
                    ) {
                        debug!("Heartbeat subscriber for '{}' stopped: {:#}", peer_name, e);
                    }
                })
                .context(format!("Failed to spawn heartbeat subscriber for {}", peer))?;
            threads.push(sub_handle);
        }

        info!(
            "Heartbeat started for '{}' on port {}, monitoring peers: {:?}",
            process_name, pub_port, peers
        );

        Ok(Self {
            process_name: process_name.to_string(),
            peer_health,
            running,
            threads,
        })
    }

    /// Get the shared peer health state.
    pub fn peer_health(&self) -> &PeerHealth {
        &self.peer_health
    }

    /// Stop all heartbeat threads.
    pub fn stop(&mut self) {
        self.running.store(false, Ordering::Release);
        for handle in self.threads.drain(..) {
            let _ = handle.join();
        }
        info!("Heartbeat stopped for '{}'", self.process_name);
    }
}

impl Drop for HeartbeatManager {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Publish heartbeat at 10Hz.
fn heartbeat_publish_loop(
    process_name: &str,
    port: u16,
    running: &AtomicBool,
) -> Result<()> {
    let ctx = zmq::Context::new();
    let socket = ctx.socket(zmq::PUB)?;
    socket.set_sndhwm(10)?;
    socket.set_linger(0)?;
    socket.bind(&format!("tcp://*:{}", port))?;

    let topic = format!("heartbeat/{}", process_name);
    let mut sequence: u32 = 0;

    while running.load(Ordering::Acquire) {
        let hb = limo_proto::Heartbeat {
            process_name: process_name.to_string(),
            timestamp_ns: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64,
            status: limo_proto::ProcessStatus::ProcessNominal as i32,
            sequence,
        };

        let data = hb.encode_to_vec();
        let _ = socket.send(&topic, zmq::SNDMORE);
        let _ = socket.send(&data, 0);

        sequence += 1;
        thread::sleep(Duration::from_millis(100)); // 10Hz
    }

    Ok(())
}

/// Subscribe to a peer's heartbeat and update PeerHealth.
fn heartbeat_subscribe_loop(
    peer_name: &str,
    peer_port: u16,
    health: &PeerHealth,
    running: &AtomicBool,
) -> Result<()> {
    let ctx = zmq::Context::new();
    let socket = ctx.socket(zmq::SUB)?;
    socket.set_rcvtimeo(200)?;
    socket.set_linger(0)?;
    socket.connect(&format!("tcp://localhost:{}", peer_port))?;
    socket.set_subscribe(format!("heartbeat/{}", peer_name).as_bytes())?;

    while running.load(Ordering::Acquire) {
        // Receive topic frame
        match socket.recv_bytes(0) {
            Ok(_topic) => {
                // Receive data frame
                if let Ok(data) = socket.recv_bytes(0) {
                    if let Ok(_hb) = limo_proto::Heartbeat::decode(data.as_slice()) {
                        health.update(peer_name);
                    }
                }
            }
            Err(zmq::Error::EAGAIN) => continue, // timeout
            Err(_) => break,
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_peer_health_status() {
        let health = PeerHealth::new(&["planning"]);
        // Just initialized, should be nominal
        assert_eq!(health.status("planning"), PeerStatus::Nominal);

        // Unknown peer
        assert_eq!(health.status("unknown"), PeerStatus::Unknown);
    }

    #[test]
    fn test_heartbeat_port_assignment() {
        assert_eq!(heartbeat_port("sensperc"), 5570);
        assert_eq!(heartbeat_port("planning"), 5571);
        assert_eq!(heartbeat_port("control"), 5572);
    }

    #[test]
    fn test_peers_of() {
        let peers = peers_of("control");
        assert!(peers.contains(&"sensperc"));
        assert!(peers.contains(&"planning"));
        assert!(!peers.contains(&"control"));
    }

    #[test]
    fn test_heartbeat_roundtrip() {
        // Start a manager, let it run briefly, verify it doesn't crash
        let mut mgr = HeartbeatManager::start("control").unwrap();
        thread::sleep(Duration::from_millis(300));

        // Peers won't be nominal (they aren't running), but manager should be alive
        let health = mgr.peer_health();
        // planning and sensperc should still be within the initial grace period
        // or have degraded — either way, no crash
        let _ = health.status("planning");
        let _ = health.status("sensperc");

        mgr.stop();
    }
}
