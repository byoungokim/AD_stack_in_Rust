#!/usr/bin/env python3
"""Gazebo Path Visualizer for Limo Drive.

Subscribes to CH10 (PlannedPath) and spawns/moves SDF models in Gazebo
to visualize waypoints and the robot's planned path.

Uses `gz service -s /world/limo_test_track/create` to spawn models
and `gz service -s /world/limo_test_track/set_pose` to move them.

Visual elements:
  - Colored spheres at scenario waypoints (red=current, yellow=others)
  - Poles under each waypoint for visibility

Usage:
    python gz_path_visualizer.py
"""

import math
import signal
import subprocess
import sys
import time

import zmq

sys.path.insert(0, "proto/gen_py")
try:
    from visualization_pb2 import PlannedPath
    HAS_PROTO = True
except ImportError:
    print("ERROR: proto/gen_py not found. Run 'make proto'.")
    sys.exit(1)

CH10_CONNECT = "tcp://localhost:5590"
WORLD = "limo_test_track"
RUNNING = True
SPAWNED = set()


def signal_handler(sig, frame):
    global RUNNING
    RUNNING = False


def gz_spawn(name, sdf_xml):
    """Spawn an SDF model in Gazebo."""
    if name in SPAWNED:
        return
    req = f'sdf: "{sdf_xml}", name: "{name}"'
    try:
        subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/create",
             "--reqtype", "gz.msgs.EntityFactory",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "2000", "--req", req],
            timeout=3.0, capture_output=True,
        )
        SPAWNED.add(name)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass


def gz_set_pose(name, x, y, z):
    """Move an entity to a new position."""
    req = f'name: "{name}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}'
    try:
        subprocess.run(
            ["gz", "service", "-s", f"/world/{WORLD}/set_pose",
             "--reqtype", "gz.msgs.Pose",
             "--reptype", "gz.msgs.Boolean",
             "--timeout", "1000", "--req", req],
            timeout=2.0, capture_output=True,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass


def make_sphere_sdf(r, g, b, radius=0.12):
    """Generate inline SDF for a colored sphere."""
    # Escape for embedding in protobuf string field
    sdf = (
        f'<sdf version=\\"1.9\\">'
        f'<model name=\\"sphere\\">'
        f'<static>true</static>'
        f'<link name=\\"link\\">'
        f'<visual name=\\"v\\">'
        f'<geometry><sphere><radius>{radius}</radius></sphere></geometry>'
        f'<material>'
        f'<ambient>{r} {g} {b} 1</ambient>'
        f'<diffuse>{r} {g} {b} 1</diffuse>'
        f'</material>'
        f'</visual>'
        f'</link>'
        f'</model>'
        f'</sdf>'
    )
    return sdf


def make_pole_sdf():
    """Generate inline SDF for a thin pole."""
    sdf = (
        '<sdf version=\\"1.9\\">'
        '<model name=\\"pole\\">'
        '<static>true</static>'
        '<link name=\\"link\\">'
        '<visual name=\\"v\\">'
        '<geometry><cylinder><radius>0.015</radius><length>0.4</length></cylinder></geometry>'
        '<material>'
        '<ambient>0.8 0.8 0.0 1</ambient>'
        '<diffuse>0.9 0.9 0.0 1</diffuse>'
        '</material>'
        '</visual>'
        '</link>'
        '</model>'
        '</sdf>'
    )
    return sdf


def spawn_waypoint_markers(msg):
    """Spawn or update waypoint markers in Gazebo."""
    for i, wp in enumerate(msg.scenario_waypoints):
        is_current = (i == msg.current_waypoint_index)
        sphere_name = f"wp_sphere_{i}"
        pole_name = f"wp_pole_{i}"

        if sphere_name not in SPAWNED:
            # Spawn sphere
            if is_current:
                sdf = make_sphere_sdf(1.0, 0.0, 0.0, 0.15)  # red, larger
            else:
                sdf = make_sphere_sdf(1.0, 1.0, 0.0)  # yellow
            gz_spawn(sphere_name, sdf)
            # Spawn pole
            gz_spawn(pole_name, make_pole_sdf())

        # Position them
        gz_set_pose(sphere_name, wp.x, wp.y, 0.15 if is_current else 0.12)
        gz_set_pose(pole_name, wp.x, wp.y, 0.2)

    # Spawn robot heading indicator
    if msg.robot_pose:
        arrow_name = "robot_heading"
        if arrow_name not in SPAWNED:
            sdf = make_sphere_sdf(0.0, 1.0, 1.0, 0.06)  # cyan small
            gz_spawn(arrow_name, sdf)
        rp = msg.robot_pose
        d = 0.3
        gz_set_pose(arrow_name,
                    rp.x + d * math.cos(rp.theta),
                    rp.y + d * math.sin(rp.theta),
                    0.15)

    # Spawn goal indicator (green sphere at current goal)
    if msg.current_goal:
        goal_name = "goal_indicator"
        if goal_name not in SPAWNED:
            sdf = make_sphere_sdf(0.0, 1.0, 0.0, 0.08)  # green
            gz_spawn(goal_name, sdf)
        gz_set_pose(goal_name, msg.current_goal.x, msg.current_goal.y, 0.3)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.RCVTIMEO, 500)
    sub.setsockopt(zmq.LINGER, 0)
    sub.connect(CH10_CONNECT)
    sub.subscribe(b"planned_path")

    print("[visualizer] Connected to CH10, spawning models in Gazebo")
    print("[visualizer] YELLOW spheres = waypoints, RED = current target")
    print("[visualizer] CYAN = robot heading, GREEN = goal")

    count = 0
    last_viz = 0.0

    while RUNNING:
        try:
            topic, data = sub.recv_multipart()
            msg = PlannedPath()
            msg.ParseFromString(data)
            count += 1

            now = time.time()
            # Spawn/update at ~1Hz (gz service calls are slow)
            if now - last_viz >= 1.0:
                spawn_waypoint_markers(msg)
                last_viz = now

            if count % 50 == 0:
                print(
                    f"[visualizer] {count} msgs | state={msg.behavior_state} "
                    f"path={len(msg.global_path)}pts "
                    f"wp={msg.current_waypoint_index}/{len(msg.scenario_waypoints)} "
                    f"speed={msg.robot_speed:.2f}"
                )

        except zmq.Again:
            continue
        except Exception as e:
            if RUNNING:
                print(f"[visualizer] Error: {e}")

    sub.close()
    ctx.term()
    print("[visualizer] Stopped")


if __name__ == "__main__":
    main()
