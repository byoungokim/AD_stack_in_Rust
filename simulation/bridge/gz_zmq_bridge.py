#!/usr/bin/env python3
"""Gazebo ↔ ZMQ Bridge using native gz.transport13 Python bindings.

Real-time bidirectional bridge:
  Gazebo /limo/odom  → CH6 (SimVehicleState) → Control process
  Gazebo /limo/imu   → (bundled into CH5 SimSensorData)
  Gazebo /limo/lidar → (bundled into CH5 SimSensorData)
  CH7 (SimControlCommand) → Gazebo /limo/cmd_vel

Uses native gz.transport13 callbacks — no subprocess overhead.
"""

import math
import signal
import sys
import threading
import time

# System Python has gz bindings; venv may not, so use system Python
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.laserscan_pb2 import LaserScan

import zmq

# Add proto path
sys.path.insert(0, "proto/gen_py")
try:
    from sim_pb2 import SimVehicleState, SimSensorData, SimControlCommand
    from common_pb2 import Header, Pose2D, Twist2D, Vector3
    from sensor_pb2 import LaserScan as ProtoLaserScan, ImuReading as ProtoImuReading
    HAS_PROTO = True
except ImportError:
    HAS_PROTO = False
    print("[bridge] WARNING: proto/gen_py not found. Run 'make proto'.")

CH5_BIND = "tcp://*:5560"
CH6_BIND = "tcp://*:5561"
CH7_CONNECT = "tcp://localhost:5562"

RUNNING = True


def signal_handler(sig, frame):
    global RUNNING
    RUNNING = False


class NativeGzBridge:
    """Bridges Gazebo topics to ZMQ using native gz.transport13."""

    def __init__(self):
        # ZMQ
        self.ctx = zmq.Context()

        self.ch5 = self.ctx.socket(zmq.PUB)
        self.ch5.setsockopt(zmq.SNDHWM, 50)
        self.ch5.setsockopt(zmq.LINGER, 0)
        self.ch5.bind(CH5_BIND)

        self.ch6 = self.ctx.socket(zmq.PUB)
        self.ch6.setsockopt(zmq.SNDHWM, 50)
        self.ch6.setsockopt(zmq.LINGER, 0)
        self.ch6.bind(CH6_BIND)

        self.ch7 = self.ctx.socket(zmq.SUB)
        self.ch7.setsockopt(zmq.RCVTIMEO, 10)
        self.ch7.setsockopt(zmq.LINGER, 0)
        self.ch7.connect(CH7_CONNECT)
        self.ch7.subscribe(b"sim_control")

        # Gazebo transport
        self.gz_node = Node()

        # Publisher for cmd_vel
        self.cmd_pub = self.gz_node.advertise("/limo/cmd_vel", Twist)

        # State
        self.seq = 0
        self.odom_count = 0
        self.cmd_count = 0
        self.last_imu = None
        self.last_lidar = None
        self.last_lidar_ts = 0.0  # wall-clock arrival time of last_lidar
        self.lock = threading.Lock()

        print(f"[bridge] Native gz.transport13 bridge")
        print(f"[bridge] CH5={CH5_BIND} CH6={CH6_BIND} CH7={CH7_CONNECT}")

    def start(self):
        """Subscribe to Gazebo topics and start forwarding."""
        # Subscribe to Gazebo topics with callbacks
        self.gz_node.subscribe(Odometry, "/limo/odom", self._on_odom)
        self.gz_node.subscribe(IMU, "/limo/imu", self._on_imu)
        self.gz_node.subscribe(LaserScan, "/limo/lidar", self._on_lidar)

        print("[bridge] Subscribed: /limo/odom, /limo/imu, /limo/lidar")
        print("[bridge] Publishing cmd_vel to /limo/cmd_vel")
        print("[bridge] Running...")

        # Main loop: forward CH7 → Gazebo cmd_vel
        while RUNNING:
            self._forward_cmd()

            if self.odom_count > 0 and self.odom_count % 200 == 0:
                print(f"[bridge] odom={self.odom_count} cmd={self.cmd_count}")

    def _on_odom(self, msg):
        """Callback: Gazebo /limo/odom → CH6 SimVehicleState."""
        if not HAS_PROTO:
            return

        # Extract pose
        pos = msg.pose.position
        ori = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (ori.w * ori.z + ori.x * ori.y),
            1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        )

        # Extract velocity
        lin_x = msg.twist.linear.x
        ang_z = msg.twist.angular.z

        # Build SimVehicleState
        vs = SimVehicleState()
        vs.header.timestamp_ns = int(time.time() * 1e9)
        vs.header.sequence = self.seq
        vs.header.frame_id = "gz_odom"
        vs.pose.x = pos.x
        vs.pose.y = pos.y
        vs.pose.theta = yaw
        vs.velocity.linear_x = lin_x
        vs.velocity.angular_z = ang_z
        vs.battery_voltage = 12.6
        vs.drive_mode = 1  # Ackermann

        # Publish on CH6
        try:
            self.ch6.send_multipart([b"sim_vehicle_state", vs.SerializeToString()])
        except Exception:
            pass

        # Also build and publish CH5 SimSensorData with latest IMU/LiDAR
        self._publish_sensor_data(pos.x, pos.y, yaw, lin_x, ang_z)

        self.seq += 1
        self.odom_count += 1

    def _on_imu(self, msg):
        """Callback: Gazebo /limo/imu → store for CH5 bundle."""
        with self.lock:
            self.last_imu = msg

    def _on_lidar(self, msg):
        """Callback: Gazebo /limo/lidar → store for CH5 bundle.

        The arrival time is recorded so the bundle can carry the scan's OWN
        stamp: scans refresh at 10Hz but bundles ship at the 20Hz odom rate,
        so without this the scan inherits a fresh bundle timestamp and
        downstream scan-time pose lookup silently degrades to latest-pose.
        """
        with self.lock:
            self.last_lidar = msg
            self.last_lidar_ts = time.time()

    def _publish_sensor_data(self, x, y, yaw, lin_x, ang_z):
        """Bundle latest sensor data into CH5 SimSensorData."""
        if not HAS_PROTO:
            return

        sd = SimSensorData()
        sd.header.timestamp_ns = int(time.time() * 1e9)
        sd.header.sequence = self.seq
        sd.header.frame_id = "gz"

        # Ground truth pose
        sd.ground_truth_pose.x = x
        sd.ground_truth_pose.y = y
        sd.ground_truth_pose.theta = yaw
        sd.ground_truth_velocity.linear_x = lin_x
        sd.ground_truth_velocity.angular_z = ang_z

        # IMU
        with self.lock:
            if self.last_imu is not None:
                imu = self.last_imu
                sd.imu.linear_acceleration.x = imu.linear_acceleration.x
                sd.imu.linear_acceleration.y = imu.linear_acceleration.y
                sd.imu.linear_acceleration.z = imu.linear_acceleration.z
                sd.imu.angular_velocity.x = imu.angular_velocity.x
                sd.imu.angular_velocity.y = imu.angular_velocity.y
                sd.imu.angular_velocity.z = imu.angular_velocity.z

            # LiDAR
            if self.last_lidar is not None:
                lidar = self.last_lidar
                sd.lidar_scan.header.timestamp_ns = int(self.last_lidar_ts * 1e9)
                sd.lidar_scan.angle_min = lidar.angle_min
                sd.lidar_scan.angle_max = lidar.angle_max
                sd.lidar_scan.angle_increment = lidar.angle_step
                sd.lidar_scan.range_min = lidar.range_min
                sd.lidar_scan.range_max = lidar.range_max
                sd.lidar_scan.ranges.extend(lidar.ranges)

        try:
            self.ch5.send_multipart([b"sim_sensors", sd.SerializeToString()])
        except Exception:
            pass

    def _forward_cmd(self):
        """Receive CH7 SimControlCommand and publish to Gazebo cmd_vel."""
        try:
            topic_bytes, data = self.ch7.recv_multipart()

            if HAS_PROTO:
                cmd = SimControlCommand()
                cmd.ParseFromString(data)

                twist = Twist()
                twist.linear.x = cmd.linear_velocity
                twist.angular.z = cmd.angular_velocity
                self.cmd_pub.publish(twist)
                self.cmd_count += 1

        except zmq.Again:
            pass  # timeout, no message
        except Exception as e:
            if RUNNING:
                pass  # silently skip errors

    def shutdown(self):
        # Send zero velocity
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        self.ch5.close()
        self.ch6.close()
        self.ch7.close()
        self.ctx.term()
        print("[bridge] Stopped")


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    bridge = NativeGzBridge()
    try:
        bridge.start()
    finally:
        bridge.shutdown()


if __name__ == "__main__":
    main()
