"""Base process node with lifecycle management, heartbeat, and watchdog."""

import logging
import signal
import threading
import time
from typing import Dict, Optional

import zmq

from core.transport.zmq_transport import ZmqPublisher, ZmqSubscriber

# Import generated protobuf (will be available after `make proto`)
try:
    from proto import system_pb2
except ImportError:
    system_pb2 = None  # type: ignore

logger = logging.getLogger(__name__)

# Channel definitions from architecture
HEARTBEAT_ENDPOINT = "tcp://localhost:5550"
HEARTBEAT_BIND = "tcp://*:5550"


class BaseNode:
    """Base class for all Limo Drive processes.

    Provides:
    - ZMQ context
    - Heartbeat publishing at 10 Hz
    - Peer heartbeat monitoring with configurable timeouts
    - Graceful shutdown on SIGINT/SIGTERM
    - Lifecycle: init → start → running → stop
    """

    # Heartbeat thresholds (seconds)
    WARN_TIMEOUT = 0.2
    DEGRADED_TIMEOUT = 0.5
    DEAD_TIMEOUT = 1.0

    def __init__(self, name: str, config: Optional[dict] = None) -> None:
        self.name = name
        self.config = config or {}
        self._running = False
        self._zmq_ctx = zmq.Context()

        # Heartbeat state
        self._heartbeat_seq = 0
        self._peer_last_seen: Dict[str, float] = {}
        self._heartbeat_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None

        # Heartbeat pub/sub are set up in start() since port sharing
        # requires coordination (only one process can bind)
        self._hb_pub: Optional[ZmqPublisher] = None
        self._hb_sub: Optional[ZmqSubscriber] = None

        # Register signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    @property
    def zmq_ctx(self) -> zmq.Context:
        return self._zmq_ctx

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> None:
        """Start the node: init heartbeat, call on_start, enter main loop."""
        logger.info("[%s] Starting...", self.name)
        self._running = True

        self._setup_heartbeat()
        self.on_start()

        logger.info("[%s] Running", self.name)
        try:
            self.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self) -> None:
        """Graceful shutdown."""
        if not self._running:
            return
        logger.info("[%s] Stopping...", self.name)
        self._running = False
        self.on_stop()
        if self._hb_sub:
            self._hb_sub.close()
        if self._hb_pub:
            self._hb_pub.close()
        self._zmq_ctx.term()
        logger.info("[%s] Stopped", self.name)

    # --- Lifecycle hooks (override in subclasses) ---

    def on_start(self) -> None:
        """Called after heartbeat is set up, before run(). Override to init."""
        pass

    def run(self) -> None:
        """Main loop. Override in subclass."""
        while self._running:
            time.sleep(0.1)

    def on_stop(self) -> None:
        """Called during shutdown. Override to clean up."""
        pass

    # --- Heartbeat ---

    def _setup_heartbeat(self) -> None:
        """Set up heartbeat publishing and peer monitoring."""
        # Each process connects to a shared heartbeat bus.
        # The supervisor (or first process) binds; others connect.
        # For simplicity, use separate pub/sub ports per process
        # or use PGM/EPGM multicast. Here we use a simple approach:
        # each process publishes on a unique port, subscribes to others.
        #
        # Ports: sensperc=5560, planning=5561, control=5562
        port_map = {"sensperc": 5560, "planning": 5561, "control": 5562}
        my_port = port_map.get(self.name, 5563)
        peer_ports = {k: v for k, v in port_map.items() if k != self.name}

        self._hb_pub = ZmqPublisher(
            self._zmq_ctx, f"tcp://*:{my_port}")

        # Subscribe to all peers
        for peer_name, peer_port in peer_ports.items():
            sub = ZmqSubscriber(
                self._zmq_ctx,
                f"tcp://localhost:{peer_port}",
                "heartbeat/"
            )
            sub.start(self._on_heartbeat_received)
            self._peer_last_seen[peer_name] = time.monotonic()

        # Start heartbeat publisher thread
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True)
        self._heartbeat_thread.start()

        # Start peer monitor thread
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()

    def _heartbeat_loop(self) -> None:
        """Publish heartbeat at 10 Hz."""
        while self._running:
            if system_pb2 and self._hb_pub:
                hb = system_pb2.Heartbeat()
                hb.process_name = self.name
                hb.timestamp_ns = int(time.monotonic() * 1e9)
                hb.status = system_pb2.PROCESS_NOMINAL
                hb.sequence = self._heartbeat_seq
                self._heartbeat_seq += 1
                self._hb_pub.publish(f"heartbeat/{self.name}", hb)
            time.sleep(0.1)  # 10 Hz

    def _on_heartbeat_received(self, topic: str, data: bytes) -> None:
        """Callback when a peer heartbeat is received."""
        if not system_pb2:
            return
        hb = system_pb2.Heartbeat()
        hb.ParseFromString(data)
        self._peer_last_seen[hb.process_name] = time.monotonic()

    def _monitor_loop(self) -> None:
        """Monitor peer heartbeats and report status."""
        while self._running:
            now = time.monotonic()
            for peer, last_seen in self._peer_last_seen.items():
                age = now - last_seen
                if age > self.DEAD_TIMEOUT:
                    self.on_peer_dead(peer, age)
                elif age > self.DEGRADED_TIMEOUT:
                    self.on_peer_degraded(peer, age)
                elif age > self.WARN_TIMEOUT:
                    logger.warning("[%s] Peer '%s' heartbeat stale: %.0fms",
                                   self.name, peer, age * 1000)
            time.sleep(0.1)  # check at 10 Hz

    def on_peer_dead(self, peer: str, age: float) -> None:
        """Called when a peer is considered dead. Override for fault handling."""
        logger.error("[%s] Peer '%s' DEAD (no heartbeat for %.1fs)",
                     self.name, peer, age)

    def on_peer_degraded(self, peer: str, age: float) -> None:
        """Called when a peer is degraded. Override for fault handling."""
        logger.warning("[%s] Peer '%s' DEGRADED (%.0fms)",
                       self.name, peer, age * 1000)

    # --- Signal handling ---

    def _signal_handler(self, signum: int, frame) -> None:
        logger.info("[%s] Received signal %d, shutting down", self.name, signum)
        self._running = False
