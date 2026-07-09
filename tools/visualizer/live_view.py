#!/usr/bin/env python3
"""Live top-down visualizer: tracked obstacles + planned trajectory.

Subscribes directly to the stack's ZMQ channels — no Gazebo required, so it
works identically against the Gazebo sim, the dummy sim, and the real robot:

  CH1  (tcp:5551, world_state)  robot pose, detections (tracked obstacles
                                carry velocity, extent radius, track id)
  CH10 (tcp:5590, planned_path) global A* path, local DWA trajectory, goal,
                                scenario waypoints, behavior state

Rendering:
  - static obstacle points: gray dots
  - tracked obstacles: circle of true extent, velocity arrow (1 s lead),
    track id label — amber when moving, slate when static
  - robot: heading wedge + trail; global path (blue), local trajectory
    (green), goal star, scenario waypoints (current one highlighted)
  - HUD: behavior state, speed, localization confidence, tracked count

Usage:
    .venv/bin/pip install matplotlib   # one-time
    .venv/bin/python tools/visualizer/live_view.py            # live window
    .venv/bin/python tools/visualizer/live_view.py \
        --snapshot 5 --out /tmp/view.png                      # headless PNG

Run `make proto` first if proto/gen_py is missing or stale.
"""
import argparse
import math
import sys
import time

sys.path.insert(0, "proto/gen_py")

try:
    import zmq
    from world_state_pb2 import WorldState
    from visualization_pb2 import PlannedPath
except ImportError as e:
    print(f"ERROR: {e}\nRun 'make proto' and use the project venv python.")
    sys.exit(1)

try:
    import matplotlib
except ImportError:
    print("ERROR: matplotlib not installed. Run: .venv/bin/pip install matplotlib")
    sys.exit(1)

CH1_ENDPOINT = "tcp://localhost:5551"
CH10_ENDPOINT = "tcp://localhost:5590"
TRAIL_SECONDS = 60.0
RATE_HZ = 12


class Feed:
    """Non-blocking ZMQ subscriber pair holding the latest messages."""

    def __init__(self):
        ctx = zmq.Context.instance()
        self.ch1 = ctx.socket(zmq.SUB)
        self.ch1.connect(CH1_ENDPOINT)
        self.ch1.setsockopt(zmq.SUBSCRIBE, b"world_state")
        self.ch10 = ctx.socket(zmq.SUB)
        self.ch10.connect(CH10_ENDPOINT)
        self.ch10.setsockopt(zmq.SUBSCRIBE, b"planned_path")
        self.world = None
        self.path = None
        self.trail = []  # (t, x, y)

    def poll(self):
        while True:
            try:
                _, payload = self.ch1.recv_multipart(zmq.NOBLOCK)
                ws = WorldState()
                ws.ParseFromString(payload)
                self.world = ws
                now = time.monotonic()
                self.trail.append((now, ws.robot_pose.x, ws.robot_pose.y))
                cutoff = now - TRAIL_SECONDS
                while self.trail and self.trail[0][0] < cutoff:
                    self.trail.pop(0)
            except zmq.Again:
                break
        while True:
            try:
                _, payload = self.ch10.recv_multipart(zmq.NOBLOCK)
                pp = PlannedPath()
                pp.ParseFromString(payload)
                self.path = pp
            except zmq.Again:
                break


def draw(ax, feed):
    ax.clear()
    ax.set_aspect("equal")
    ax.grid(True, linewidth=0.4, alpha=0.4)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("limo_drive — tracked obstacles & planned trajectory")

    ws, pp = feed.world, feed.path
    if ws is None:
        ax.text(0.5, 0.5, "waiting for WorldState on CH1…", ha="center",
                va="center", transform=ax.transAxes, color="gray")
        return

    rx, ry, rt = ws.robot_pose.x, ws.robot_pose.y, ws.robot_pose.theta

    # trail
    if len(feed.trail) > 1:
        ax.plot([p[1] for p in feed.trail], [p[2] for p in feed.trail],
                color="#9ec5f4", linewidth=1.2, alpha=0.8, zorder=2)

    # detections
    static_x, static_y = [], []
    for det in ws.detections.detections:
        p = det.position_world
        if det.track_id == 0:
            static_x.append(p.x)
            static_y.append(p.y)
        else:
            vx = det.velocity_world.linear_x
            vy = det.velocity_world.linear_y
            moving = math.hypot(vx, vy) > 1e-6
            color = "#eda100" if moving else "#52514e"
            r = max(det.radius, 0.08)
            ax.add_patch(matplotlib.patches.Circle(
                (p.x, p.y), r, fill=True, alpha=0.35, color=color, zorder=4))
            ax.add_patch(matplotlib.patches.Circle(
                (p.x, p.y), r, fill=False, linewidth=1.4, color=color, zorder=4))
            if moving:
                ax.annotate("", xy=(p.x + vx, p.y + vy), xytext=(p.x, p.y),
                            arrowprops=dict(arrowstyle="->", color=color, lw=1.6),
                            zorder=5)
            ax.text(p.x, p.y + r + 0.08, f"#{det.track_id}", fontsize=7,
                    ha="center", color=color, zorder=5)
    if static_x:
        ax.scatter(static_x, static_y, s=6, c="#898781", zorder=3, label="static points")

    if pp is not None:
        if len(pp.global_path) > 1:
            ax.plot([q.x for q in pp.global_path], [q.y for q in pp.global_path],
                    color="#2a78d6", linewidth=1.6, alpha=0.9, zorder=3,
                    label="global path (A*)")
        if len(pp.local_trajectory) > 1:
            ax.plot([q.x for q in pp.local_trajectory],
                    [q.y for q in pp.local_trajectory],
                    color="#1baf7a", linewidth=2.6, zorder=6,
                    label="local trajectory (DWA)")
        for i, wp in enumerate(pp.scenario_waypoints):
            current = i == pp.current_waypoint_index
            ax.plot(wp.x, wp.y, marker="o", markersize=9 if current else 6,
                    markerfacecolor="none",
                    markeredgecolor="#d03b3b" if current else "#898781",
                    markeredgewidth=1.6, zorder=5)
        if pp.HasField("current_goal"):
            g = pp.current_goal
            ax.plot(g.x, g.y, marker="*", markersize=15, color="#d03b3b", zorder=6)
            if pp.goal_label:
                ax.text(g.x, g.y + 0.2, pp.goal_label, fontsize=8,
                        ha="center", color="#d03b3b", zorder=6)

    # robot: heading wedge
    L = 0.28
    pts = [(rx + L * math.cos(rt), ry + L * math.sin(rt)),
           (rx + 0.12 * math.cos(rt + 2.5), ry + 0.12 * math.sin(rt + 2.5)),
           (rx + 0.12 * math.cos(rt - 2.5), ry + 0.12 * math.sin(rt - 2.5))]
    ax.add_patch(matplotlib.patches.Polygon(pts, closed=True, color="#2a78d6", zorder=7))

    # HUD
    n_tracked = sum(1 for d in ws.detections.detections if d.track_id != 0)
    hud = [f"loc conf: {ws.localization_confidence:.2f}",
           f"tracked obstacles: {n_tracked}"]
    if pp is not None:
        hud.insert(0, f"state: {pp.behavior_state}   speed: {pp.robot_speed:.2f} m/s")
    ax.text(0.01, 0.99, "\n".join(hud), transform=ax.transAxes, fontsize=9,
            va="top", family="monospace",
            bbox=dict(facecolor="white", alpha=0.75, edgecolor="none"))

    # view window follows the robot
    ax.set_xlim(rx - 6, rx + 6)
    ax.set_ylim(ry - 4.5, ry + 4.5)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc="lower right", fontsize=8, framealpha=0.75)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--snapshot", type=float, metavar="SECONDS",
                    help="headless: collect for N seconds, save PNG, exit")
    ap.add_argument("--out", default="live_view.png", help="snapshot output path")
    args = ap.parse_args()

    if args.snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    feed = Feed()
    fig, ax = plt.subplots(figsize=(11, 8))

    if args.snapshot:
        deadline = time.monotonic() + args.snapshot
        while time.monotonic() < deadline:
            feed.poll()
            time.sleep(0.05)
        draw(ax, feed)
        fig.savefig(args.out, dpi=110, bbox_inches="tight")
        print(f"saved {args.out} "
              f"(world={'yes' if feed.world else 'NO'}, "
              f"path={'yes' if feed.path else 'NO'})")
        return

    from matplotlib.animation import FuncAnimation

    def tick(_frame):
        feed.poll()
        draw(ax, feed)

    _anim = FuncAnimation(fig, tick, interval=int(1000 / RATE_HZ),
                          cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
