#!/usr/bin/env python3
"""Gazebo ↔ ZMQ Bridge for Limo Drive.

Bridges Gazebo Harmonic's transport topics to our ZMQ channels:
  - Gazebo camera/lidar/imu → CH5 (SimSensorData)
  - Gazebo odometry → CH6 (SimVehicleState)
  - CH7 (SimControlCommand) → Gazebo cmd_vel

Uses the `gz.transport` Python bindings (gz-transport-python)
or falls back to subprocess `gz topic` calls.

Usage:
    python gz_zmq_bridge.py [--config config/sim_bridge.yaml]
"""

import argparse
import json
import math
import signal
import struct
import subprocess
import sys
import threading
import time
from typing import Optional

import zmq

# Protobuf imports (generated from our proto files)
sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent.parent))
try:
    from proto.gen_py import common_pb2, sensor_pb2, sim_pb2, control_pb2
    HAS_PROTO = True
except ImportError:
    HAS_PROTO = False
    print("WARNING: Protobuf Python bindings not found. Run 'make proto' first.")

# ZMQ channel config
CH5_BIND = "tcp://*:5560"       # SimSensorData publish
CH6_BIND = "tcp://*:5561"       # SimVehicleState publish
CH7_CONNECT = "tcp://localhost:5562"  # SimControlCommand subscribe

# Gazebo topics
GZ_CMD_VEL = "/limo/cmd_vel"
GZ_ODOM = "/limo/odom"
GZ_CAMERA = "/limo/camera"
GZ_LIDAR = "/limo/lidar"
GZ_IMU = "/limo/imu"

RUNNING = True


def signal_handler(sig, frame):
    global RUNNING
    print("\nShutting down bridge...")
    RUNNING = False


class GzZmqBridge:
    """Bridges Gazebo topics to ZMQ channels."""

    def __init__(self):
        self.ctx = zmq.Context()

        # Publishers (Gazebo → our stack)
        self.ch5_pub = self.ctx.socket(zmq.PUB)
        self.ch5_pub.setsockopt(zmq.SNDHWM, 100)
        self.ch5_pub.setsockopt(zmq.LINGER, 0)
        self.ch5_pub.bind(CH5_BIND)

        self.ch6_pub = self.ctx.socket(zmq.PUB)
        self.ch6_pub.setsockopt(zmq.SNDHWM, 100)
        self.ch6_pub.setsockopt(zmq.LINGER, 0)
        self.ch6_pub.bind(CH6_BIND)

        # Subscriber (our stack → Gazebo)
        self.ch7_sub = self.ctx.socket(zmq.SUB)
        self.ch7_sub.setsockopt(zmq.RCVTIMEO, 50)
        self.ch7_sub.setsockopt(zmq.LINGER, 0)
        self.ch7_sub.connect(CH7_CONNECT)
        self.ch7_sub.subscribe(b"sim_control")

        self.sequence = 0
        print(f"Bridge ready: CH5={CH5_BIND}, CH6={CH6_BIND}, CH7={CH7_CONNECT}")

    def run(self):
        """Main bridge loop."""
        # Try to use gz.transport Python bindings
        try:
            import gz.transport
            print("Using gz.transport Python bindings")
            self._run_with_bindings()
        except ImportError:
            print("gz.transport not available, using subprocess bridge")
            self._run_with_subprocess()

    def _run_with_subprocess(self):
        """Fallback: use `gz topic` subprocess to echo Gazebo topics."""
        # Start odom listener thread
        odom_thread = threading.Thread(target=self._odom_listener, daemon=True)
        odom_thread.start()

        # Main loop: forward CH7 commands to Gazebo
        print("Bridge running (subprocess mode)...")

        while RUNNING:
            # Receive control commands from CH7
            try:
                topic_bytes, data = self.ch7_sub.recv_multipart()

                if HAS_PROTO:
                    cmd = sim_pb2.SimControlCommand()
                    cmd.ParseFromString(data)
                    self._send_cmd_vel(cmd.linear_velocity, cmd.angular_velocity)
                else:
                    # Without proto, just log
                    print(f"CH7: received {len(data)} bytes")

            except zmq.Again:
                pass  # timeout
            except Exception as e:
                if RUNNING:
                    print(f"CH7 error: {e}")

            time.sleep(0.01)

    def _odom_listener(self):
        """Listen to Gazebo odometry via `gz topic -e` and publish on CH6."""
        print(f"Starting odom listener on {GZ_ODOM}...")

        try:
            proc = subprocess.Popen(
                ["gz", "topic", "-e", "-t", GZ_ODOM],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            buffer = ""
            while RUNNING and proc.poll() is None:
                line = proc.stdout.readline()
                if not line:
                    time.sleep(0.01)
                    continue

                buffer += line

                # Gazebo outputs protobuf text format, parse when complete
                if line.strip() == "" and buffer.strip():
                    self._parse_and_publish_odom(buffer.strip())
                    buffer = ""

            proc.terminate()
        except FileNotFoundError:
            print("ERROR: 'gz' command not found. Is Gazebo installed?")
        except Exception as e:
            print(f"Odom listener error: {e}")

    def _parse_and_publish_odom(self, text: str):
        """Parse Gazebo odom text output and publish as SimVehicleState on CH6."""
        # Simple text parsing of Gazebo protobuf text format
        # This is a fallback — proper implementation uses gz.transport bindings
        try:
            # Extract position and velocity from text format
            # Format varies by Gazebo version, this handles common output
            x = self._extract_float(text, "x:", 0)
            y = self._extract_float(text, "y:", 0)
            z = self._extract_float(text, "z:", 0)

            if HAS_PROTO:
                msg = sim_pb2.SimVehicleState()
                msg.header.timestamp_ns = int(time.time() * 1e9)
                msg.header.sequence = self.sequence
                msg.header.frame_id = "gz"
                msg.pose.x = x
                msg.pose.y = y
                msg.pose.theta = 0.0  # TODO: extract yaw
                msg.battery_voltage = 12.6
                msg.drive_mode = 1  # Ackermann

                data = msg.SerializeToString()
                self.ch6_pub.send_multipart([b"sim_vehicle_state", data])

            self.sequence += 1

        except Exception as e:
            pass  # silently skip parse errors

    def _extract_float(self, text: str, key: str, default: float) -> float:
        """Extract a float value after a key in text."""
        try:
            idx = text.index(key)
            rest = text[idx + len(key):].strip()
            num_str = ""
            for c in rest:
                if c in "0123456789.-+e":
                    num_str += c
                else:
                    break
            return float(num_str) if num_str else default
        except (ValueError, IndexError):
            return default

    def _send_cmd_vel(self, linear: float, angular: float):
        """Send velocity command to Gazebo via `gz topic -p`."""
        # Gazebo Harmonic uses gz.msgs.Twist
        msg = f'linear: {{x: {linear}}}, angular: {{z: {angular}}}'
        try:
            subprocess.run(
                ["gz", "topic", "-t", GZ_CMD_VEL, "-m", "gz.msgs.Twist",
                 "-p", msg],
                timeout=1.0,
                capture_output=True,
            )
        except subprocess.TimeoutExpired:
            pass
        except FileNotFoundError:
            pass

    def _run_with_bindings(self):
        """Use gz.transport Python bindings for efficient topic bridging."""
        import gz.transport

        node = gz.transport.Node()

        # Subscribe to Gazebo topics
        node.subscribe(GZ_ODOM, self._on_gz_odom)
        # node.subscribe(GZ_CAMERA, self._on_gz_camera)  # TODO
        # node.subscribe(GZ_LIDAR, self._on_gz_lidar)    # TODO

        print("Bridge running (gz.transport bindings mode)...")

        # Forward CH7 commands to Gazebo
        pub = node.advertise(GZ_CMD_VEL, "gz.msgs.Twist")

        while RUNNING:
            try:
                topic_bytes, data = self.ch7_sub.recv_multipart()

                if HAS_PROTO:
                    cmd = sim_pb2.SimControlCommand()
                    cmd.ParseFromString(data)

                    # Publish to Gazebo
                    twist = gz.msgs.Twist()
                    twist.linear.x = cmd.linear_velocity
                    twist.angular.z = cmd.angular_velocity
                    pub.publish(twist)

            except zmq.Again:
                pass
            except Exception as e:
                if RUNNING:
                    print(f"CH7 error: {e}")

            time.sleep(0.01)

    def _on_gz_odom(self, msg):
        """Callback for Gazebo odometry topic."""
        if HAS_PROTO:
            vs = sim_pb2.SimVehicleState()
            vs.header.timestamp_ns = int(time.time() * 1e9)
            vs.header.sequence = self.sequence
            vs.header.frame_id = "gz"
            # Extract pose from gz.msgs.Odometry
            vs.pose.x = msg.pose.position.x
            vs.pose.y = msg.pose.position.y
            # Convert quaternion to yaw
            q = msg.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            vs.pose.theta = yaw
            vs.velocity.linear_x = msg.twist.linear.x
            vs.velocity.angular_z = msg.twist.angular.z
            vs.battery_voltage = 12.6
            vs.drive_mode = 1

            data = vs.SerializeToString()
            self.ch6_pub.send_multipart([b"sim_vehicle_state", data])
            self.sequence += 1

    def shutdown(self):
        self.ch5_pub.close()
        self.ch6_pub.close()
        self.ch7_sub.close()
        self.ctx.term()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    bridge = GzZmqBridge()
    try:
        bridge.run()
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
