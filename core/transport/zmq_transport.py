"""ZMQ PUB/SUB transport wrappers for Python processes."""

import logging
import threading
from typing import Callable, Optional

import zmq
from google.protobuf.message import Message as ProtoMessage

logger = logging.getLogger(__name__)


class ZmqPublisher:
    """Publishes Protobuf messages on a ZMQ PUB socket."""

    def __init__(self, ctx: zmq.Context, endpoint: str) -> None:
        self._socket = ctx.socket(zmq.PUB)
        self._socket.bind(endpoint)
        logger.info("ZmqPublisher bound to %s", endpoint)

    def publish(self, topic: str, msg: ProtoMessage) -> bool:
        """Publish a Protobuf message with a topic prefix."""
        try:
            data = msg.SerializeToString()
            self._socket.send_multipart([topic.encode(), data])
            return True
        except Exception:
            logger.exception("Failed to publish on topic '%s'", topic)
            return False

    def close(self) -> None:
        self._socket.close()


class ZmqSubscriber:
    """Subscribes to ZMQ PUB topics and delivers messages via callback."""

    def __init__(self, ctx: zmq.Context, endpoint: str,
                 topic_filter: str = "") -> None:
        self._socket = ctx.socket(zmq.SUB)
        self._socket.connect(endpoint)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, topic_filter)
        self._socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        self._topic_filter = topic_filter
        self._callback: Optional[Callable[[str, bytes], None]] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        logger.info("ZmqSubscriber connected to %s [filter='%s']",
                    endpoint, topic_filter)

    def start(self, callback: Callable[[str, bytes], None]) -> None:
        """Start receiving in a background thread."""
        if self._running:
            return
        self._callback = callback
        self._running = True
        self._thread = threading.Thread(target=self._receive_loop,
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop the background receive thread."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def close(self) -> None:
        self.stop()
        self._socket.close()

    def _receive_loop(self) -> None:
        while self._running:
            try:
                topic_bytes, data = self._socket.recv_multipart()
                topic = topic_bytes.decode()
                if self._callback:
                    self._callback(topic, data)
            except zmq.Again:
                continue  # timeout, check running flag
            except Exception:
                logger.exception("Error in receive loop")
