#!/usr/bin/env python3
"""Reactive pedestrian controller for the generated city world.

Drives the ped_* models emitted by gen_city_world.py along their routes
(<world>_peds.json) via each model's VelocityControl plugin, and — unlike
the old scripted <actor> pedestrians, which were blind — PAUSES a ped when
the robot is close, resuming with hysteresis once it has moved away. Real
pedestrians yield; now the simulated ones do too.

Closed-loop: each ped model carries a PosePublisher (10 Hz), so control is
against actual poses, not dead reckoning. Peds are kinematic cylinders and
never rotate — commanded velocity is world-frame by construction (body
frame stays axis-aligned forever).

Usage (run_gazebo_full.sh starts this automatically when the routes file
exists next to the world):
    python3 simulation/bridge/ped_controller.py simulation/worlds/city_blocks_peds.json
"""

import json
import math
import signal
import sys
import time

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.twist_pb2 import Twist

RATE_HZ = 10.0
PAUSE_DIST = 0.75   # m: robot closer than this -> ped stops
RESUME_DIST = 1.00  # m: robot farther than this -> ped resumes (hysteresis)
WAYPOINT_TOL = 0.15  # m: waypoint reached

RUNNING = True


def _sig(_s, _f):
    global RUNNING
    RUNNING = False


class Ped:
    def __init__(self, spec):
        self.name = spec["name"]
        self.mode = spec["mode"]  # "shuttle" | "loop"
        self.route = [tuple(p) for p in spec["route"]]
        self.speed = float(spec["speed"])
        self.dwell = float(spec.get("dwell", 0.0))
        self.pos = None          # latest (x, y) from PosePublisher
        self.target_idx = 1 if len(self.route) > 1 else 0
        self.direction = 1       # shuttle: +1 forward, -1 backward
        self.dwell_until = 0.0
        self.paused = False

    def on_pose(self, msg: Pose):
        self.pos = (msg.position.x, msg.position.y)

    def advance(self):
        """Move target to the next route index; shuttle reverses at ends."""
        if self.mode == "loop":
            self.target_idx = (self.target_idx + 1) % len(self.route)
            return
        nxt = self.target_idx + self.direction
        if nxt < 0 or nxt >= len(self.route):
            self.direction = -self.direction
            nxt = self.target_idx + self.direction
            if self.dwell > 0.0:
                self.dwell_until = time.time() + self.dwell
        self.target_idx = nxt

    def command(self, robot_pos, now):
        """Return (vx, vy) world-frame velocity for this cycle."""
        if self.pos is None:
            return (0.0, 0.0)
        if now < self.dwell_until:
            return (0.0, 0.0)

        # Yield to the robot (with hysteresis so peds don't stutter).
        if robot_pos is not None:
            d = math.hypot(robot_pos[0] - self.pos[0], robot_pos[1] - self.pos[1])
            if self.paused:
                if d > RESUME_DIST:
                    self.paused = False
            elif d < PAUSE_DIST:
                self.paused = True
            if self.paused:
                return (0.0, 0.0)

        tx, ty = self.route[self.target_idx]
        dx, dy = tx - self.pos[0], ty - self.pos[1]
        dist = math.hypot(dx, dy)
        if dist < WAYPOINT_TOL:
            self.advance()
            tx, ty = self.route[self.target_idx]
            dx, dy = tx - self.pos[0], ty - self.pos[1]
            dist = math.hypot(dx, dy)
            if dist < WAYPOINT_TOL:
                return (0.0, 0.0)
        # Slow into the waypoint so the 10 Hz loop cannot overshoot through it.
        v = min(self.speed, dist * 2.0)
        return (v * dx / dist, v * dy / dist)


def main():
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <peds.json>", file=sys.stderr)
        return 2
    with open(sys.argv[1]) as f:
        peds = [Ped(spec) for spec in json.load(f)["peds"]]
    if not peds:
        print("[peds] no pedestrians in routes file")
        return 0

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    node = Node()
    robot = {"pos": None}

    def on_robot(msg: Pose):
        robot["pos"] = (msg.position.x, msg.position.y)

    node.subscribe(Pose, "/model/limo/pose", on_robot)
    pubs = {}
    for p in peds:
        node.subscribe(Pose, f"/model/{p.name}/pose", p.on_pose)
        pubs[p.name] = node.advertise(f"/model/{p.name}/cmd_vel", Twist)

    print(f"[peds] reactive controller: {len(peds)} pedestrians, "
          f"pause<{PAUSE_DIST}m resume>{RESUME_DIST}m")

    period = 1.0 / RATE_HZ
    last_report = time.time()
    while RUNNING:
        now = time.time()
        n_paused = 0
        for p in peds:
            vx, vy = p.command(robot["pos"], now)
            n_paused += 1 if p.paused else 0
            tw = Twist()
            tw.linear.x = vx
            tw.linear.y = vy
            pubs[p.name].publish(tw)
        if now - last_report > 30.0:
            live = sum(1 for p in peds if p.pos is not None)
            print(f"[peds] {live}/{len(peds)} tracked, {n_paused} yielding")
            last_report = now
        time.sleep(period)

    # Leave everyone stopped on shutdown.
    for p in peds:
        pubs[p.name].publish(Twist())
    print("[peds] stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
