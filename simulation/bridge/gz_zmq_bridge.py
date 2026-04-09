#!/usr/bin/env python3
"""Gazebo Harmonic ↔ ZMQ Bridge for Limo Drive.

Reads sensor data and odometry from Gazebo topics, publishes on ZMQ
channels CH5 (SimSensorData) and CH6 (SimVehicleState).
Subscribes CH7 (SimControlCommand) and forwards to Gazebo cmd_vel.

Uses `gz topic` subprocess calls for Gazebo communication.

Usage:
    python gz_zmq_bridge.py
"""

import json
import math
import signal
import subprocess
import sys
import threading
import time

import zmq

# --- ZMQ Channel Config ---
CH5_BIND = "tcp://*:5560"          # SimSensorData → SensPerc
CH6_BIND = "tcp://*:5561"          # SimVehicleState → Control
CH7_CONNECT = "tcp://localhost:5562"  # SimControlCommand from Control

# --- Gazebo Topics ---
GZ_CMD_VEL = "/limo/cmd_vel"
GZ_ODOM = "/limo/odom"
GZ_IMU = "/limo/imu"
GZ_LIDAR = "/limo/lidar"

RUNNING = True


def signal_handler(sig, frame):
    global RUNNING
    print("\n[bridge] Shutting down...")
    RUNNING = False


class GazeboBridge:
    def __init__(self):
        self.ctx = zmq.Context()

        # CH5 publisher
        self.ch5 = self.ctx.socket(zmq.PUB)
        self.ch5.setsockopt(zmq.SNDHWM, 50)
        self.ch5.setsockopt(zmq.LINGER, 0)
        self.ch5.bind(CH5_BIND)

        # CH6 publisher
        self.ch6 = self.ctx.socket(zmq.PUB)
        self.ch6.setsockopt(zmq.SNDHWM, 50)
        self.ch6.setsockopt(zmq.LINGER, 0)
        self.ch6.bind(CH6_BIND)

        # CH7 subscriber
        self.ch7 = self.ctx.socket(zmq.SUB)
        self.ch7.setsockopt(zmq.RCVTIMEO, 50)
        self.ch7.setsockopt(zmq.LINGER, 0)
        self.ch7.connect(CH7_CONNECT)
        self.ch7.subscribe(b"sim_control")

        self.seq = 0
        print(f"[bridge] CH5={CH5_BIND}, CH6={CH6_BIND}, CH7={CH7_CONNECT}")

    def run(self):
        # Start background listeners for Gazebo topics
        threads = [
            threading.Thread(target=self._odom_listener, daemon=True),
            threading.Thread(target=self._imu_listener, daemon=True),
            threading.Thread(target=self._lidar_listener, daemon=True),
            threading.Thread(target=self._cmd_forwarder, daemon=True),
        ]
        for t in threads:
            t.start()

        print("[bridge] Running. Ctrl+C to stop.")
        while RUNNING:
            time.sleep(0.1)

    def _run_gz_echo(self, topic):
        """Run `gz topic -e -t <topic>` and yield complete messages."""
        proc = subprocess.Popen(
            ["gz", "topic", "-e", "-t", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )

        buffer = []
        brace_depth = 0

        while RUNNING and proc.poll() is None:
            line = proc.stdout.readline()
            if not line:
                time.sleep(0.001)
                continue

            stripped = line.strip()
            if not stripped:
                # Empty line = message boundary in protobuf text format
                if buffer:
                    yield "\n".join(buffer)
                    buffer = []
                continue

            buffer.append(stripped)

        proc.terminate()
        proc.wait()

    def _odom_listener(self):
        """Read /limo/odom and publish on CH6 (SimVehicleState)."""
        print(f"[bridge] Listening: {GZ_ODOM}")

        for msg_text in self._run_gz_echo(GZ_ODOM):
            if not RUNNING:
                break

            try:
                pose_x = self._extract_nested(msg_text, "pose", "position", "x")
                pose_y = self._extract_nested(msg_text, "pose", "position", "y")

                # Extract quaternion and convert to yaw
                qw = self._extract_nested(msg_text, "pose", "orientation", "w") or 1.0
                qx = self._extract_nested(msg_text, "pose", "orientation", "x") or 0.0
                qy = self._extract_nested(msg_text, "pose", "orientation", "y") or 0.0
                qz = self._extract_nested(msg_text, "pose", "orientation", "z") or 0.0
                yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz))

                lin_x = self._extract_nested(msg_text, "twist", "linear", "x") or 0.0
                ang_z = self._extract_nested(msg_text, "twist", "angular", "z") or 0.0

                # Build protobuf-like message manually (no proto dependency)
                # Pack as: topic + simple binary format
                # For simplicity, publish as JSON-encoded bytes on ZMQ
                data = {
                    "timestamp_ns": int(time.time() * 1e9),
                    "sequence": self.seq,
                    "pose": {"x": pose_x or 0.0, "y": pose_y or 0.0, "theta": yaw},
                    "velocity": {"linear_x": lin_x, "angular_z": ang_z},
                    "steering_angle": 0.0,
                    "battery_voltage": 12.6,
                }
                # Since we need protobuf format for our ZMQ channels,
                # we'll use the proto Python bindings
                self._publish_vehicle_state(data)
                self.seq += 1

            except Exception as e:
                pass  # skip parse errors silently

    def _imu_listener(self):
        """Read /limo/imu — for now just log (sensor data goes in CH5)."""
        print(f"[bridge] Listening: {GZ_IMU}")
        # IMU data will be bundled into CH5 SimSensorData
        # For now, we skip detailed IMU bridging
        for msg_text in self._run_gz_echo(GZ_IMU):
            if not RUNNING:
                break
            # TODO: parse and include in CH5 bundle

    def _lidar_listener(self):
        """Read /limo/lidar — for CH5."""
        print(f"[bridge] Listening: {GZ_LIDAR}")
        for msg_text in self._run_gz_echo(GZ_LIDAR):
            if not RUNNING:
                break
            # TODO: parse LaserScan data and include in CH5 bundle

    def _cmd_forwarder(self):
        """Receive CH7 SimControlCommand and send to Gazebo cmd_vel."""
        print(f"[bridge] Forwarding CH7 → {GZ_CMD_VEL}")

        while RUNNING:
            try:
                topic_bytes, data = self.ch7.recv_multipart()

                # Parse the SimControlCommand protobuf
                try:
                    sys.path.insert(0, "proto/gen_py")
                    from sim_pb2 import SimControlCommand
                    cmd = SimControlCommand()
                    cmd.ParseFromString(data)
                    linear = cmd.linear_velocity
                    angular = cmd.angular_velocity
                except Exception:
                    # Fallback: assume simple format
                    linear = 0.0
                    angular = 0.0

                # Send to Gazebo
                msg = f'linear: {{x: {linear}}}, angular: {{z: {angular}}}'
                subprocess.run(
                    ["gz", "topic", "-t", GZ_CMD_VEL,
                     "-m", "gz.msgs.Twist", "-p", msg],
                    timeout=1.0, capture_output=True,
                )

            except zmq.Again:
                continue
            except Exception as e:
                if RUNNING:
                    print(f"[bridge] CH7 error: {e}")

    def _publish_vehicle_state(self, data):
        """Publish SimVehicleState on CH6 using proto Python bindings."""
        try:
            sys.path.insert(0, "proto/gen_py")
            from sim_pb2 import SimVehicleState
            from common_pb2 import Header, Pose2D, Twist2D

            vs = SimVehicleState()
            vs.header.timestamp_ns = data["timestamp_ns"]
            vs.header.sequence = data["sequence"]
            vs.header.frame_id = "gz"
            vs.pose.x = data["pose"]["x"]
            vs.pose.y = data["pose"]["y"]
            vs.pose.theta = data["pose"]["theta"]
            vs.velocity.linear_x = data["velocity"]["linear_x"]
            vs.velocity.angular_z = data["velocity"]["angular_z"]
            vs.steering_angle = data["steering_angle"]
            vs.battery_voltage = data["battery_voltage"]
            vs.drive_mode = 1  # Ackermann

            self.ch6.send_multipart([b"sim_vehicle_state", vs.SerializeToString()])

        except ImportError:
            print("[bridge] WARNING: proto/gen_py not found. Run 'make proto'.")
        except Exception as e:
            print(f"[bridge] CH6 publish error: {e}")

    def _extract_nested(self, text, *keys):
        """Extract a numeric value from protobuf text format given nested keys."""
        # Simple recursive key search in protobuf text format
        lines = text.split("\n")
        depth = 0
        key_idx = 0

        for line in lines:
            stripped = line.strip()

            if key_idx < len(keys) - 1:
                if stripped.startswith(keys[key_idx]) and "{" in stripped:
                    key_idx += 1
                    continue

            if key_idx == len(keys) - 1:
                target = keys[-1]
                if stripped.startswith(f"{target}:"):
                    val_str = stripped.split(":", 1)[1].strip()
                    try:
                        return float(val_str)
                    except ValueError:
                        return None

            if "{" in stripped:
                depth += 1
            if "}" in stripped:
                depth -= 1
                if depth < key_idx:
                    key_idx = depth

        return None

    def shutdown(self):
        self.ch5.close()
        self.ch6.close()
        self.ch7.close()
        self.ctx.term()


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    bridge = GazeboBridge()
    try:
        bridge.run()
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
