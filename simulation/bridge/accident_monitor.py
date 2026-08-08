#!/usr/bin/env python3
"""Accident monitor: counts collisions and logs the reason for each.

Two detection paths, because the world has two kinds of moving bodies:

1. TRUE CONTACTS — the chassis contact sensor (/limo/contacts) fires on any
   physical touch: pedestrian models, buildings, walls, parked obstacles.
   The contact message names the exact entity, so the culprit is ground
   truth, not a guess.
2. GHOST OVERLAPS — vehicle actors are kinematic visuals with no collision
   body, so a robot-car "crash" is a silent visual overlap. Those are
   detected perceptually from CH1 WorldState: a tracked mover or car-sized
   cluster whose surface penetrates the robot footprint.

Every episode (contacts within EPISODE_GAP_S collapse into one accident) is
classified by who approached whom at impact:
  robot stopped + other party closing   -> "<party> ran into stopped robot"
  robot moving toward the other party   -> "robot drove into <party>"
  otherwise                             -> "side swipe"

Output: console lines + append-only accidents.log in the project root, one
line per accident plus a run header/summary. The log format is stable and
greppable:  ACCIDENT #n | <iso time> | <party> | <classification> | ...

Usage (run_gazebo_full.sh starts this automatically):
    python3 simulation/bridge/accident_monitor.py [--log accidents.log]
"""

import argparse
import datetime
import math
import signal
import sys
import threading
import time

from gz.transport13 import Node
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.pose_pb2 import Pose

import zmq

sys.path.insert(0, "proto/gen_py")
try:
    from world_state_pb2 import WorldState
    HAS_PROTO = True
except ImportError:
    HAS_PROTO = False

CH1_CONNECT = "tcp://localhost:5551"
ROBOT_FOOTPRINT_M = 0.19   # matches planner collision footprint
EPISODE_GAP_S = 3.0        # contacts closer than this = same accident
GHOST_MIN_SPEED = 0.25     # mover threshold for perceptual detection
CAR_MIN_EXTENT = 0.30      # half-extent above this = car-sized

RUNNING = True


def _sig(_s, _f):
    global RUNNING
    RUNNING = False


def classify(robot_speed, approach_deg):
    """Who-into-whom from robot speed and the party's approach angle
    (0 deg = party moving straight at the robot)."""
    if robot_speed < 0.05 and approach_deg < 60.0:
        return "party ran into stopped robot"
    if robot_speed >= 0.05 and approach_deg > 120.0:
        return "robot drove into party"
    return "side swipe"


def party_kind(name):
    if name.startswith("ped_"):
        return "pedestrian"
    if name.startswith("vehicle_"):
        return "vehicle"
    if name.startswith(("building_", "wall_", "parked_")):
        return "static:" + name
    if name in ("ground_plane",):
        return None  # rolling on the ground is not an accident
    return "static:" + name


class AccidentLog:
    def __init__(self, path):
        self.path = path
        self.count = 0
        self.last_at = {}  # party -> monotonic time of last contact
        self.lock = threading.Lock()
        with open(self.path, "a") as f:
            f.write(
                f"\n=== accident monitor started {datetime.datetime.now().isoformat(timespec='seconds')} ===\n"
            )

    def report(self, source, party, robot_speed, party_speed, approach_deg, where):
        now = time.monotonic()
        with self.lock:
            if now - self.last_at.get(party, -1e9) < EPISODE_GAP_S:
                self.last_at[party] = now  # still the same episode
                return
            self.last_at[party] = now
            self.count += 1
            reason = classify(robot_speed, approach_deg)
            line = (
                f"ACCIDENT #{self.count} | {datetime.datetime.now().isoformat(timespec='seconds')} "
                f"| {party} | {reason} | source={source} "
                f"| robot_v={robot_speed:.2f} party_v={party_speed:.2f} "
                f"approach={approach_deg:.0f}deg | at=({where[0]:.2f},{where[1]:.2f})"
            )
            print(f"[accidents] {line}", flush=True)
            with open(self.path, "a") as f:
                f.write(line + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", default="accidents.log")
    args = ap.parse_args()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    log = AccidentLog(args.log)

    state = {"robot_xy": None, "robot_v": (0.0, 0.0), "peds": {}}
    state_lock = threading.Lock()

    gz = Node()

    # Robot world pose (for velocity estimation on the contact path).
    pose_hist = []

    def on_robot_pose(msg: Pose):
        now = time.monotonic()
        with state_lock:
            state["robot_xy"] = (msg.position.x, msg.position.y)
            pose_hist.append((now, msg.position.x, msg.position.y))
            while pose_hist and now - pose_hist[0][0] > 1.0:
                pose_hist.pop(0)
            if len(pose_hist) >= 2:
                (t0, x0, y0), (t1, x1, y1) = pose_hist[0], pose_hist[-1]
                dt = max(t1 - t0, 1e-3)
                state["robot_v"] = ((x1 - x0) / dt, (y1 - y0) / dt)

    gz.subscribe(Pose, "/model/limo/pose", on_robot_pose)

    # Pedestrian poses (world frame) for party-velocity on the contact path.
    def make_ped_cb(name):
        hist = []

        def cb(msg: Pose):
            now = time.monotonic()
            hist.append((now, msg.position.x, msg.position.y))
            while hist and now - hist[0][0] > 1.0:
                hist.pop(0)
            v = (0.0, 0.0)
            if len(hist) >= 2:
                (t0, x0, y0), (t1, x1, y1) = hist[0], hist[-1]
                dt = max(t1 - t0, 1e-3)
                v = ((x1 - x0) / dt, (y1 - y0) / dt)
            with state_lock:
                state["peds"][name] = ((msg.position.x, msg.position.y), v)

        return cb

    for i in range(64):  # subscribe generously; absent topics are harmless
        for kind in ("cross", "loop"):
            name = f"ped_{i}_{kind}"
            gz.subscribe(Pose, f"/model/{name}/pose", make_ped_cb(name))

    # Path 1: ground-truth chassis contacts.
    def on_contacts(msg: Contacts):
        with state_lock:
            robot_xy = state["robot_xy"]
            rvx, rvy = state["robot_v"]
            peds = dict(state["peds"])
        if robot_xy is None:
            return
        rspeed = math.hypot(rvx, rvy)
        for c in msg.contact:
            names = {c.collision1.name, c.collision2.name}
            other = None
            for n in names:
                model = n.split("::")[0]
                if model != "limo":
                    other = model
            if other is None:
                continue
            kind = party_kind(other)
            if kind is None:
                continue
            pspeed, approach = 0.0, 90.0
            if other in peds:
                (px, py), (pvx, pvy) = peds[other]
                pspeed = math.hypot(pvx, pvy)
                if pspeed > 0.05:
                    to_robot = math.atan2(robot_xy[1] - py, robot_xy[0] - px)
                    mv = math.atan2(pvy, pvx)
                    approach = abs(
                        math.degrees(math.atan2(math.sin(mv - to_robot), math.cos(mv - to_robot)))
                    )
                elif rspeed >= 0.05:
                    approach = 180.0  # party effectively still; robot moved in
            elif rspeed >= 0.05:
                approach = 180.0  # static party: the robot did the approaching
            log.report("contact", kind, rspeed, pspeed, approach, robot_xy)

    gz.subscribe(Contacts, "/limo/contacts", on_contacts)

    # Path 2: perceptual ghost-overlap with vehicle actors (CH1 WorldState).
    def ch1_loop():
        if not HAS_PROTO:
            print("[accidents] proto/gen_py missing — ghost-overlap path disabled", flush=True)
            return
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.RCVTIMEO, 500)
        sub.setsockopt(zmq.LINGER, 0)
        sub.connect(CH1_CONNECT)
        sub.subscribe(b"world_state")
        while RUNNING:
            try:
                _topic, payload = sub.recv_multipart()
            except zmq.Again:
                continue
            except Exception:
                break
            ws = WorldState()
            try:
                ws.ParseFromString(payload)
            except Exception:
                continue
            ex, ey = ws.robot_pose.x, ws.robot_pose.y
            rvx = ws.robot_velocity.linear_x * math.cos(ws.robot_pose.theta)
            rvy = ws.robot_velocity.linear_x * math.sin(ws.robot_pose.theta)
            rspeed = abs(ws.robot_velocity.linear_x)
            for d in ws.detections.detections:
                pvx, pvy = d.velocity_world.linear_x, d.velocity_world.linear_y
                pspeed = math.hypot(pvx, pvy)
                hx = max(d.half_extent_x, d.half_extent_y, d.radius)
                if pspeed < GHOST_MIN_SPEED and hx < CAR_MIN_EXTENT:
                    continue  # not a mover, not car-sized: contact path owns it
                px, py = d.position_world.x, d.position_world.y
                surf = math.hypot(px - ex, py - ey) - hx - ROBOT_FOOTPRINT_M
                if surf >= 0.0:
                    continue
                approach = 90.0
                if pspeed > 0.05:
                    to_robot = math.atan2(ey - py, ex - px)
                    mv = math.atan2(pvy, pvx)
                    approach = abs(
                        math.degrees(math.atan2(math.sin(mv - to_robot), math.cos(mv - to_robot)))
                    )
                elif rspeed >= 0.05:
                    approach = 180.0
                party = "vehicle(perceived)" if hx >= CAR_MIN_EXTENT else "mover(perceived)"
                log.report("ghost-overlap", party, rspeed, pspeed, approach, (ex, ey))
        sub.close()
        ctx.term()

    t = threading.Thread(target=ch1_loop, daemon=True)
    t.start()

    print(f"[accidents] monitoring: contact sensor + CH1 ghost-overlap -> {args.log}", flush=True)
    last_summary = time.time()
    while RUNNING:
        time.sleep(0.5)
        if time.time() - last_summary > 120.0:
            print(f"[accidents] running total: {log.count}", flush=True)
            last_summary = time.time()

    with open(args.log, "a") as f:
        f.write(
            f"=== accident monitor stopped {datetime.datetime.now().isoformat(timespec='seconds')} "
            f"— total accidents: {log.count} ===\n"
        )
    print(f"[accidents] stopped — total: {log.count}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
