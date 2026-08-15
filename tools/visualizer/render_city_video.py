#!/usr/bin/env python3
"""Render a shareable bird's-eye MP4 of a city patrol from a .limorec.

Draws the generated city (streets, sidewalk rings, zebra crosswalks,
buildings parsed from the world SDF) and animates the recorded run on top:
robot + trail, planned global path and local trajectory, the active
roadmap route nodes and leg goal, and perceived actors (pedestrians
yellow, car-sized movers red). Same media pipeline as the gauntlet GIFs.

Usage:
    .venv/bin/python tools/visualizer/render_city_video.py \
        city_demo_run7_reactive_peds.limorec docs/media/city_sidewalk_patrol.mp4

Requires ffmpeg on PATH. Geometry constants match gen_city_world.py
defaults (blocks=3, block=7.0, street=3.0, sidewalk=1.5).
"""
import math
import re
import sys

sys.path.insert(0, "proto/gen_py")
sys.path.insert(0, "tools/visualizer")
import importlib.util

spec = importlib.util.spec_from_file_location("rv", "tools/visualizer/replay_view.py")
rv = importlib.util.module_from_spec(spec)
sys.argv = ["rv"]
try:
    spec.loader.exec_module(rv)
except SystemExit:
    pass

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation, patches

REC = sys.argv[1] if len(sys.argv) > 1 else "city_demo_run7_reactive_peds.limorec"
OUT = sys.argv[2] if len(sys.argv) > 2 else "docs/media/city_sidewalk_patrol.mp4"

# --- geometry (matches gen_city_world.py defaults; planner = world + OFF) ---
OFF = 12.75
STREETS = [c + OFF for c in (-15.0, -5.0, 5.0, 15.0)]
BLOCKC = [c + OFF for c in (-10.0, 0.0, 10.0)]
HALF = 16.5
LO, HI = -HALF + OFF, HALF + OFF
RH = 2.75
SW = 1.5

rec = rv.Recording(REC)
print(f"loaded {REC}: {len(rec.world_frames)} frames, {rec.t1-rec.t0:.0f}s")

# Buildings / parked boxes straight from the world SDF (planner frame).
boxes = []
sdf = open("simulation/worlds/city_blocks.sdf").read()
for m in re.finditer(
    r'<model name="((?:building|parked_box)_\d+)">\s*<static>true</static>\s*'
    r"<pose>([-\d.]+) ([-\d.]+) [-\d.]+ 0 0 ([-\d.]+)</pose>.*?<box><size>([-\d.]+) ([-\d.]+)",
    sdf, re.S,
):
    name, x, y, yaw, sx, sy = m.groups()
    boxes.append((name, float(x) + OFF, float(y) + OFF, float(sx), float(sy)))

fig, ax = plt.subplots(figsize=(9, 9), dpi=110)
fig.patch.set_facecolor("#f2f0ec")
ax.set_facecolor("#dfe3da")  # ground
ax.set_xlim(LO - 2.2, HI + 2.2)
ax.set_ylim(LO - 2.2, HI + 2.2)
ax.set_aspect("equal")
ax.axis("off")

# --- static scene ---
for c in STREETS:  # asphalt
    ax.add_patch(patches.Rectangle((c - 1.5, LO - 1.5), 3.0, HI - LO + 3.0, fc="#3a3a3e", ec="none", zorder=1))
    ax.add_patch(patches.Rectangle((LO - 1.5, c - 1.5), HI - LO + 3.0, 3.0, fc="#3a3a3e", ec="none", zorder=1))
for c in STREETS:  # centerlines
    ax.plot([c, c], [LO - 1.5, HI + 1.5], color="#c9b23a", lw=0.7, zorder=2)
    ax.plot([LO - 1.5, HI + 1.5], [c, c], color="#c9b23a", lw=0.7, zorder=2)
for bc in BLOCKC:  # sidewalk rings
    for bc2 in BLOCKC:
        ax.add_patch(patches.Rectangle((bc - 3.5, bc2 - 3.5), 7.0, 7.0, fc="#b9b6ae", ec="none", zorder=2))
        ax.add_patch(patches.Rectangle((bc - 2.0, bc2 - 2.0), 4.0, 4.0, fc="#dfe3da", ec="none", zorder=2))
for name, x, y, sx, sy in boxes:
    fc = "#a08b6f" if name.startswith("building") else "#8b8f94"
    ax.add_patch(patches.Rectangle((x - sx / 2, y - sy / 2), sx, sy, fc=fc, ec="#6f6152", lw=0.5, zorder=3))
# crosswalks: zebra bands across streets at the ring-corner latitudes
for bc in BLOCKC:
    for lat in (bc - RH, bc + RH):
        for c in STREETS[1:-1]:
            for k in range(6):
                x0 = c - 1.5 + k * 0.5 + 0.09
                ax.add_patch(patches.Rectangle((x0, lat - 0.25), 0.32, 0.5, fc="white", ec="none", zorder=2.5, alpha=0.9))
                ax.add_patch(patches.Rectangle((lat - 0.25, x0), 0.5, 0.32, fc="white", ec="none", zorder=2.5, alpha=0.9))
ax.add_patch(patches.Rectangle((LO - 1.6, LO - 1.6), HI - LO + 3.2, HI - LO + 3.2, fc="none", ec="#b0491f", lw=2.5, zorder=4))

# --- dynamic artists ---
(trail_ln,) = ax.plot([], [], color="#e13bd0", lw=1.4, alpha=0.55, zorder=7)
(gpath_ln,) = ax.plot([], [], color="#2a78d6", lw=1.6, alpha=0.9, zorder=6)
(ltraj_ln,) = ax.plot([], [], color="#1baf7a", lw=2.4, zorder=7)
robot_dot = ax.add_patch(patches.Circle((0, 0), 0.30, fc="#e13bd0", ec="#7c1470", lw=1.2, zorder=9))
(heading_ln,) = ax.plot([], [], color="#7c1470", lw=2.0, zorder=9)
stat_sc = ax.scatter([], [], s=4, c="#5c5c60", zorder=5)
ped_sc = ax.scatter([], [], s=52, c="#e8b31a", edgecolors="#8a6a08", linewidths=0.8, zorder=8)
car_sc = ax.scatter([], [], s=150, c="#c33028", marker="s", edgecolors="#701812", linewidths=0.8, zorder=8)
wp_sc = ax.scatter([], [], s=26, facecolors="none", edgecolors="#4a6a96", linewidths=1.2, zorder=6)
(goal_star,) = ax.plot([], [], marker="*", markersize=15, color="#d03b3b", ls="none", zorder=8)
hud = ax.text(0.012, 0.988, "", transform=ax.transAxes, va="top", ha="left",
              fontsize=10, family="monospace", color="#222",
              bbox=dict(fc="#f6f4ef", ec="#999", alpha=0.9, boxstyle="round,pad=0.35"))
ax.text(0.5, 1.015, "Limo Drive — autonomous sidewalk patrol in the generated city (4x speed)",
        transform=ax.transAxes, ha="center", fontsize=12, color="#333")

STEP, FPS = 2, 20
frames = rec.world_frames[::STEP]
MAXF = 1300
frames = frames[:MAXF]
print(f"rendering {len(frames)} frames -> {OUT}")

trail_x, trail_y = [], []

def draw(i):
    t, ws = frames[i]
    ex, ey, th = ws.robot_pose.x, ws.robot_pose.y, ws.robot_pose.theta
    trail_x.append(ex)
    trail_y.append(ey)
    trail_ln.set_data(trail_x, trail_y)
    robot_dot.center = (ex, ey)
    heading_ln.set_data([ex, ex + 0.55 * math.cos(th)], [ey, ey + 0.55 * math.sin(th)])

    sx, sy, px, py, cx, cy = [], [], [], [], [], []
    for d in ws.detections.detections:
        v = math.hypot(d.velocity_world.linear_x, d.velocity_world.linear_y)
        hx = max(d.half_extent_x, d.half_extent_y, d.radius)
        if v > 0.12 or hx >= 0.28:
            if hx >= 0.28:
                cx.append(d.position_world.x); cy.append(d.position_world.y)
            else:
                px.append(d.position_world.x); py.append(d.position_world.y)
        else:
            sx.append(d.position_world.x); sy.append(d.position_world.y)
    stat_sc.set_offsets(list(zip(sx, sy)) or [(-99, -99)])
    ped_sc.set_offsets(list(zip(px, py)) or [(-99, -99)])
    car_sc.set_offsets(list(zip(cx, cy)) or [(-99, -99)])

    _, pp = rec.at(t)
    state, speed = "", 0.0
    if pp is not None:
        gpath_ln.set_data([q.x for q in pp.global_path], [q.y for q in pp.global_path])
        ltraj_ln.set_data([q.x for q in pp.local_trajectory], [q.y for q in pp.local_trajectory])
        wp_sc.set_offsets([(w.x, w.y) for w in pp.scenario_waypoints] or [(-99, -99)])
        if pp.HasField("current_goal"):
            goal_star.set_data([pp.current_goal.x], [pp.current_goal.y])
        state, speed = pp.behavior_state, pp.robot_speed
    hud.set_text(
        f"t = {t - rec.t0:6.1f}s   v = {speed:4.2f} m/s   state: {state or '—'}\n"
        f"yellow: pedestrians (they yield)   red: vehicles   o: route nodes   *: leg goal"
    )

writer = animation.FFMpegWriter(fps=FPS, bitrate=2400)
with writer.saving(fig, OUT, dpi=110):
    for i in range(len(frames)):
        draw(i)
        writer.grab_frame()
        if i % 200 == 0:
            print(f"  frame {i}/{len(frames)}")
print("done:", OUT)
