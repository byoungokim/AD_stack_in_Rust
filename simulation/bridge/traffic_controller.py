#!/usr/bin/env python3
"""Erratic traffic controller for the generated city (GTA v2).

Drives the vehicle_* car models emitted by gen_city_world.py along the
right-hand lane network via each model's VelocityControl plugin, with
deliberately IRREGULAR behavior — the point is to threaten the robot:

  - random turn choice at every intersection (straight / left / right)
  - target speed re-sampled every few seconds within the car's band
  - sudden "distracted" full stops (probability scaled by `erratic`)
  - only `attentive` cars (a minority) yield to the nearby robot;
    the rest do NOT — the robot must keep itself safe
  - simple car-following so traffic queues instead of stacking

Closed-loop: each car carries a PosePublisher; control is unicycle-style
(forward velocity + yaw rate in the body frame) against actual poses.

Usage (run_gazebo_full.sh starts this automatically):
    python3 simulation/bridge/traffic_controller.py simulation/worlds/city_blocks_traffic.json
"""

import json
import math
import random
import signal
import sys
import time

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.twist_pb2 import Twist

RATE_HZ = 15.0
TARGET_TOL = 0.55       # m: intersection lane-point reached
YAW_GAIN = 2.2          # yaw-rate P gain
MAX_YAW_RATE = 1.8      # rad/s
FOLLOW_DIST = 1.7       # m: brake if a car ahead within this
ATTENTIVE_DIST = 1.3    # m: attentive cars stop for the robot inside this
TURN_WEIGHTS = {"straight": 0.5, "left": 0.25, "right": 0.25}

RUNNING = True


def _sig(_s, _f):
    global RUNNING
    RUNNING = False


DIRS = {"E": (1, 0), "W": (-1, 0), "N": (0, 1), "S": (0, -1)}
LEFT = {"E": "N", "N": "W", "W": "S", "S": "E"}
RIGHT = {"E": "S", "S": "W", "W": "N", "N": "E"}


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Car:
    def __init__(self, spec, centers, lane, rng):
        self.name = spec["name"]
        self.dir = spec["dir"]
        self.i, self.j = spec["i"], spec["j"]
        self.smin, self.smax = spec["speed_min"], spec["speed_max"]
        self.attentive = spec["attentive"]
        self.erratic = spec["erratic"]
        self.centers = centers
        self.lane = lane
        self.rng = rng
        self.pose = None  # (x, y, yaw)
        self.speed_target = rng.uniform(self.smin, self.smax)
        self.next_speed_change = time.time() + rng.uniform(3.0, 8.0)
        self.stopped_until = 0.0
        self.advance()  # sets self.target from the spawn cell

    def lane_point(self, d, i, j):
        ci, cj = self.centers[i], self.centers[j]
        if d == "E":
            return (ci, cj - self.lane)
        if d == "W":
            return (ci, cj + self.lane)
        if d == "N":
            return (ci + self.lane, cj)
        return (ci - self.lane, cj)

    def _options(self):
        """Legal moves from intersection (i, j) heading self.dir."""
        K = len(self.centers)
        opts = []
        for action, d in (("straight", self.dir), ("left", LEFT[self.dir]), ("right", RIGHT[self.dir])):
            dx, dy = DIRS[d]
            ni, nj = self.i + dx, self.j + dy
            if 0 <= ni < K and 0 <= nj < K:
                opts.append((action, d, ni, nj))
        return opts

    def advance(self):
        opts = self._options()
        if not opts:  # boxed corner (shouldn't happen: turns always exist)
            self.dir = LEFT[self.dir]
            return self.advance()
        weights = [TURN_WEIGHTS[a] for (a, _, _, _) in opts]
        action, d, ni, nj = self.rng.choices(opts, weights=weights)[0]
        self.dir, self.i, self.j = d, ni, nj
        self.target = self.lane_point(d, ni, nj)

    def on_pose(self, msg: Pose):
        self.pose = (msg.position.x, msg.position.y, yaw_from_quat(msg.orientation))

    def command(self, robot_pos, others, now):
        """Return (v, w) body-frame command for this cycle."""
        if self.pose is None:
            return (0.0, 0.0)
        x, y, yaw = self.pose

        # Erratic speed life-cycle
        if now >= self.next_speed_change:
            self.speed_target = self.rng.uniform(self.smin, self.smax)
            self.next_speed_change = now + self.rng.uniform(3.0, 8.0)
            if self.rng.random() < 0.18 * self.erratic:  # distracted full stop
                self.stopped_until = now + self.rng.uniform(1.0, 3.5)
        if now < self.stopped_until:
            return (0.0, 0.0)

        # Attentive minority yields to the robot ahead of them
        if self.attentive and robot_pos is not None:
            rd = math.hypot(robot_pos[0] - x, robot_pos[1] - y)
            if rd < ATTENTIVE_DIST:
                bearing = math.atan2(robot_pos[1] - y, robot_pos[0] - x)
                if abs(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))) < 1.2:
                    return (0.0, 0.0)

        # Car-following: brake for a car ahead in my cone
        for o in others:
            if o is self or o.pose is None:
                continue
            ox, oy, _ = o.pose
            d = math.hypot(ox - x, oy - y)
            if d < FOLLOW_DIST:
                bearing = math.atan2(oy - y, ox - x)
                if abs(math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))) < 0.6:
                    return (0.0, 0.0)

        tx, ty = self.target
        dist = math.hypot(tx - x, ty - y)
        if dist < TARGET_TOL:
            self.advance()
            tx, ty = self.target
            dist = math.hypot(tx - x, ty - y)
        bearing = math.atan2(ty - y, tx - x)
        err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        w = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, YAW_GAIN * err))
        v = self.speed_target * max(0.2, math.cos(err))
        return (v, w)


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <traffic.json>", file=sys.stderr)
        return 2
    with open(sys.argv[1]) as f:
        cfg = json.load(f)
    rng = random.Random(20)
    cars = [Car(spec, cfg["centers"], cfg["lane"], rng) for spec in cfg["cars"]]
    if not cars:
        print("[traffic] no cars configured")
        return 0

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    node = Node()
    robot = {"pos": None}

    def on_robot(msg: Pose):
        robot["pos"] = (msg.position.x, msg.position.y)

    node.subscribe(Pose, "/model/limo/pose", on_robot)
    pubs = {}
    for c in cars:
        node.subscribe(Pose, f"/model/{c.name}/pose", c.on_pose)
        pubs[c.name] = node.advertise(f"/model/{c.name}/cmd_vel", Twist)

    n_att = sum(1 for c in cars if c.attentive)
    print(f"[traffic] {len(cars)} erratic cars ({n_att} attentive, {len(cars)-n_att} will NOT yield)", flush=True)

    period = 1.0 / RATE_HZ
    last_report = time.time()
    while RUNNING:
        now = time.time()
        for c in cars:
            v, w = c.command(robot["pos"], cars, now)
            tw = Twist()
            tw.linear.x = v
            tw.angular.z = w
            pubs[c.name].publish(tw)
        if now - last_report > 30.0:
            live = sum(1 for c in cars if c.pose is not None)
            moving = sum(1 for c in cars if c.pose and now >= c.stopped_until)
            print(f"[traffic] {live}/{len(cars)} tracked, {moving} rolling", flush=True)
            last_report = now
        time.sleep(period)

    for c in cars:
        pubs[c.name].publish(Twist())
    print("[traffic] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
