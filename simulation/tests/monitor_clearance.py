#!/usr/bin/env python3
"""Ground-truth obstacle-clearance monitor for Gazebo scenario runs.

Grades *avoidance*, not just arrival: tracks the robot's true world pose and
reports its minimum clearance to every static obstacle and pedestrian actor
over the run. Negative static clearance means the robot's (circumscribed)
footprint overlapped an obstacle; negative actor clearance means it passed
through a pedestrian — actors have no collision geometry, so Gazebo physics
never catches that.

Robot pose comes from a PosePublisher plugin on the robot model
(/model/limo/pose, true world pose + sim-time stamps). Do NOT subscribe to
SceneBroadcaster's /world/*/dynamic_pose/info instead — that segfaults
gz-sim 8.11 on macOS. Actor positions are computed analytically from their
scripted trajectories at the same sim time (actors are deterministic).

Usage (system python3 with gz.transport13 bindings):
  python3 simulation/tests/monitor_clearance.py \
      simulation/tests/fixtures/gauntlet_obstacles.json \
      simulation/tests/fixtures/gauntlet_actors.json

Fixture formats:
  obstacles.json: [{"name","type":"box","x","y","yaw","sx","sy"} |
                   {"name","type":"cylinder","x","y","r"}]
  actors.json:    [{"name","waypoints":[[t,x,y],...]}]  (looping script)

Prints a running minimum every 5 s and, on SIGTERM/SIGINT, a final line:
  CLEARANCE_SUMMARY samples=N static_min=<m> obstacle=<name> ... actor_min=<m> ped=<name> ...

Note: clearance uses the robot's circumscribed radius (0.19 m for the
0.32x0.20 Limo), so small negative values (> -0.09 m) can mean a corner-safe
sideways pass rather than physical contact — but always a violated safety
margin, since the planner is configured for 0.2-0.3 m.
"""
import json
import math
import signal
import sys
import time

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose

ROBOT_RADIUS = 0.19  # circumscribed radius of the Limo Pro footprint
ACTOR_RADIUS = 0.15
POSE_TOPIC = "/model/limo/pose"

RUNNING = True


def _stop(sig, frame):
    global RUNNING
    RUNNING = False


def dist_to_box(px, py, ob):
    """Exact distance from a point to a (possibly rotated) rectangle footprint."""
    c, s = math.cos(-ob["yaw"]), math.sin(-ob["yaw"])
    lx = c * (px - ob["x"]) - s * (py - ob["y"])
    ly = s * (px - ob["x"]) + c * (py - ob["y"])
    dx = max(abs(lx) - ob["sx"] / 2, 0.0)
    dy = max(abs(ly) - ob["sy"] / 2, 0.0)
    if dx == 0.0 and dy == 0.0:
        return -min(ob["sx"] / 2 - abs(lx), ob["sy"] / 2 - abs(ly))
    return math.hypot(dx, dy)


def actor_pos(traj, t):
    """Linear interpolation along the scripted loop at sim time t."""
    wps = traj["waypoints"]
    period = wps[-1][0]
    tt = t % period if period > 0 else 0.0
    for i in range(1, len(wps)):
        t0, x0, y0 = wps[i - 1]
        t1, x1, y1 = wps[i]
        if tt <= t1:
            f = 0.0 if t1 == t0 else (tt - t0) / (t1 - t0)
            return (x0 + f * (x1 - x0), y0 + f * (y1 - y0))
    return (wps[-1][1], wps[-1][2])


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    with open(sys.argv[1]) as f:
        obstacles = json.load(f)
    with open(sys.argv[2]) as f:
        actors = json.load(f)

    state = {
        "static_min": (float("inf"), "", 0.0, (0.0, 0.0)),
        "actor_min": (float("inf"), "", 0.0, (0.0, 0.0)),
        "last_print": 0.0,
        "samples": 0,
    }

    def on_pose(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        rx, ry = msg.position.x, msg.position.y
        state["samples"] += 1

        for ob in obstacles:
            if ob["type"] == "cylinder":
                d = math.hypot(rx - ob["x"], ry - ob["y"]) - ob["r"]
            else:
                d = dist_to_box(rx, ry, ob)
            d -= ROBOT_RADIUS
            if d < state["static_min"][0]:
                state["static_min"] = (d, ob["name"], t, (rx, ry))

        for traj in actors:
            ax, ay = actor_pos(traj, t)
            d = math.hypot(rx - ax, ry - ay) - ACTOR_RADIUS - ROBOT_RADIUS
            if d < state["actor_min"][0]:
                state["actor_min"] = (d, traj["name"], t, (rx, ry))

        now = time.monotonic()
        if now - state["last_print"] >= 5.0:
            state["last_print"] = now
            sm, so, _, _ = state["static_min"]
            am, ao, _, _ = state["actor_min"]
            print(
                f"t={t:6.1f}s robot=({rx:5.2f},{ry:5.2f}) "
                f"static_min={sm:.3f} ({so}) actor_min={am:.3f} ({ao})",
                flush=True,
            )

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)
    node = Node()
    if not node.subscribe(Pose, POSE_TOPIC, on_pose):
        print(f"ERROR: failed to subscribe {POSE_TOPIC}", flush=True)
        sys.exit(1)
    print(
        f"[clearance] subscribed {POSE_TOPIC}, "
        f"{len(obstacles)} obstacles, {len(actors)} actors",
        flush=True,
    )
    try:
        while RUNNING:
            time.sleep(0.5)
    finally:
        sm, so, st_, sp = state["static_min"]
        am, ao, at_, ap = state["actor_min"]
        print(
            f"CLEARANCE_SUMMARY samples={state['samples']} "
            f"static_min={sm:.3f} obstacle={so} t={st_:.1f}s pos=({sp[0]:.2f},{sp[1]:.2f}) "
            f"actor_min={am:.3f} ped={ao} t={at_:.1f}s pos=({ap[0]:.2f},{ap[1]:.2f})",
            flush=True,
        )


if __name__ == "__main__":
    main()
