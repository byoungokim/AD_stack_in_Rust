#!/usr/bin/env python3
"""Record & replay visualizer: whole-map playback with parameter inspection.

Complements live_view.py (live-only) with a flight-recorder workflow so a
problematic run can be scrubbed back and forth and discussed offline:

  record   subscribe CH1 (world_state) + CH10 (planned_path) and append
           timestamped frames to a .limorec file. The file header snapshots
           config/planning.yaml, the scenario yaml, and the roadmap yaml so
           the replay shows THE parameters that produced the behavior.
  play     scrub the recording: time slider, space = play/pause, ←/→ = step,
           ↑/↓ = playback speed, p = parameter panel, e = event list panel.
           With --log <run.log>, planner log events (Blocked reasons with
           culprit coordinates, recovery transitions, route changes) are
           aligned on the timeline; the active event is drawn on the map
           (red × on the blocking obstacle) and listed in the HUD.

File format (.limorec):
  line 1: JSON header {"v": 1, "started_at": epoch, "configs": {name: text}}
  then repeated binary frames: <d channel:B length:I> payload
  channel 1 = WorldState, 10 = PlannedPath (raw protobuf bytes)

Usage:
    .venv/bin/python tools/visualizer/replay_view.py record --out run.limorec
    .venv/bin/python tools/visualizer/replay_view.py play run.limorec \
        --log demo_run.log --full
"""
import argparse
import bisect
import json
import math
import os
import re
import struct
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

CH1_ENDPOINT = "tcp://localhost:5551"
CH2_ENDPOINT = "tcp://localhost:5552"
CH3_ENDPOINT = "tcp://localhost:5553"
CH10_ENDPOINT = "tcp://localhost:5590"
FRAME_HDR = struct.Struct("<dBI")  # wall time, channel, payload length

CONFIG_FILES = {
    "planning": "config/planning.yaml",
    "scenario": "config/scenarios/obstacle_gauntlet.yaml",
    "roadmap": "config/maps/obstacle_gauntlet_roadmap.yaml",
    "control": "config/control.yaml",
    "sensperc": "config/sensperc.yaml",
}


# --------------------------------------------------------------------------
# record
# --------------------------------------------------------------------------

def record(out_path, scenario_yaml=None):
    configs = {}
    files = dict(CONFIG_FILES)
    if scenario_yaml:
        files["scenario"] = scenario_yaml
    for name, path in files.items():
        try:
            with open(path) as f:
                configs[name] = f.read()
        except OSError:
            configs[name] = f"<unavailable: {path}>"

    ctx = zmq.Context.instance()
    subs = []
    for ch, endpoint, topic in ((1, CH1_ENDPOINT, b"world_state"),
                                (2, CH2_ENDPOINT, b"control_cmd"),
                                (3, CH3_ENDPOINT, b"vehicle_state"),
                                (10, CH10_ENDPOINT, b"planned_path")):
        s = ctx.socket(zmq.SUB)
        s.connect(endpoint)
        s.setsockopt(zmq.SUBSCRIBE, topic)
        subs.append((ch, s))

    n = 0
    with open(out_path, "wb") as f:
        header = {"v": 1, "started_at": time.time(), "configs": configs}
        f.write((json.dumps(header) + "\n").encode())
        print(f"recording CH1+CH10 -> {out_path} (Ctrl-C to stop)")
        try:
            while True:
                wrote = False
                for ch, sock in subs:
                    while True:
                        try:
                            _, payload = sock.recv_multipart(zmq.NOBLOCK)
                        except zmq.Again:
                            break
                        f.write(FRAME_HDR.pack(time.time(), ch, len(payload)))
                        f.write(payload)
                        n += 1
                        wrote = True
                if wrote:
                    f.flush()
                    if n % 500 == 0:
                        print(f"  {n} frames")
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
    print(f"recorded {n} frames -> {out_path}")


# --------------------------------------------------------------------------
# recording + log loading
# --------------------------------------------------------------------------

class Recording:
    def __init__(self, path):
        self.configs = {}
        self.world_frames = []   # (t, WorldState)
        self.path_frames = []    # (t, PlannedPath)
        self.cmd_frames = []     # (t, raw ControlCommand bytes) — CH2
        self.state_frames = []   # (t, raw VehicleState bytes) — CH3
        with open(path, "rb") as f:
            header = json.loads(f.readline().decode())
            self.configs = header.get("configs", {})
            self.started_at = header.get("started_at", 0.0)
            while True:
                hdr = f.read(FRAME_HDR.size)
                if len(hdr) < FRAME_HDR.size:
                    break
                t, ch, length = FRAME_HDR.unpack(hdr)
                payload = f.read(length)
                if len(payload) < length:
                    break
                try:
                    if ch == 1:
                        ws = WorldState()
                        ws.ParseFromString(payload)
                        self.world_frames.append((t, ws))
                    elif ch == 2:
                        self.cmd_frames.append((t, payload))
                    elif ch == 3:
                        self.state_frames.append((t, payload))
                    elif ch == 10:
                        pp = PlannedPath()
                        pp.ParseFromString(payload)
                        self.path_frames.append((t, pp))
                except Exception:
                    continue  # skip torn frame at EOF
        if not self.world_frames and not self.path_frames:
            raise SystemExit("recording holds no frames")
        ts = [t for t, _ in self.world_frames] + [t for t, _ in self.path_frames]
        self.t0, self.t1 = min(ts), max(ts)
        self._wt = [t for t, _ in self.world_frames]
        self._pt = [t for t, _ in self.path_frames]

    def at(self, t):
        """Latest (WorldState, PlannedPath) at wall time <= t."""
        ws = pp = None
        i = bisect.bisect_right(self._wt, t) - 1
        if i >= 0:
            ws = self.world_frames[i][1]
        j = bisect.bisect_right(self._pt, t) - 1
        if j >= 0:
            pp = self.path_frames[j][1]
        return ws, pp

    def trail_until(self, t):
        i = bisect.bisect_right(self._wt, t)
        return [(w.robot_pose.x, w.robot_pose.y) for _, w in self.world_frames[:i]]


LOG_TS = re.compile(r"^(\d{4}-\d{2}-\d{2})T(\d{2}):(\d{2}):(\d{2})\.(\d+)Z")
LOG_BLOCKED = re.compile(
    r"Blocked\[obs=\(([-\d.]+),([-\d.]+)\) v_obs=([-\d.]+) net=([-\d.]+) "
    r"req=([-\d.]+) at v=([-\d.]+) phase=([\w-]+)\]")
LOG_KEEP = re.compile(
    r"Blocked\[|entering Recovery|recovery FAILED|recovery phase|blocked \(|"
    r"Route \(|WAYPOINT|escape mode|dropping stale path|Global path replaced")


def parse_log(path):
    """[(epoch, line, (ox, oy) or None)] for planner events worth showing."""
    import calendar
    events = []
    with open(path, errors="replace") as f:
        for raw in f:
            line = re.sub(r"\x1b\[[0-9;]*m", "", raw).rstrip()
            if not LOG_KEEP.search(line):
                continue
            m = LOG_TS.match(line)
            if not m:
                continue
            date, hh, mm, ss, frac = m.groups()
            y, mo, d = (int(v) for v in date.split("-"))
            epoch = calendar.timegm((y, mo, d, int(hh), int(mm), int(ss), 0, 0, 0))
            epoch += float(f"0.{frac}")
            b = LOG_BLOCKED.search(line)
            obs = (float(b.group(1)), float(b.group(2))) if b else None
            # keep the message tail only (drop timestamp/level/module noise)
            msg = line.split(": ", 2)[-1]
            events.append((epoch, msg, obs))
    return events


# --------------------------------------------------------------------------
# play
# --------------------------------------------------------------------------

def flatten_yaml(text, prefix=""):
    """Best-effort flatten of a yaml document into 'a.b.c: value' lines."""
    try:
        import yaml
        data = yaml.safe_load(text)
    except Exception:
        return [ln for ln in text.splitlines() if ln.strip()][:80]
    lines = []

    def walk(node, pfx):
        if isinstance(node, dict):
            for k, v in node.items():
                walk(v, f"{pfx}{k}.")
        elif isinstance(node, list):
            if all(not isinstance(v, (dict, list)) for v in node):
                lines.append(f"{pfx[:-1]}: {node}")
            else:
                for i, v in enumerate(node):
                    walk(v, f"{pfx}{i}.")
        else:
            lines.append(f"{pfx[:-1]}: {node}")

    walk(data, prefix)
    return lines


def load_roadmap_overlay(text):
    """(nodes {name: (x,y)}, links [((x1,y1),(x2,y2))]) from roadmap yaml."""
    try:
        import yaml
        data = yaml.safe_load(text)
        nodes = {n["name"]: (float(n["x"]), float(n["y"]))
                 for n in data.get("nodes", [])}
        links = []
        for l in data.get("links", []):
            a, b = l["from"], l["to"]
            if a in nodes and b in nodes:
                links.append((nodes[a], nodes[b]))
        return nodes, links
    except Exception:
        return {}, []


class Player:
    OWNED_KEYS = {" ", "left", "right", "up", "down", "p", "e", "home", "end"}

    def __init__(self, rec, events, full_extent):
        import matplotlib
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
        self.mpl = matplotlib
        self.rec = rec
        self.events = events
        self.ev_t = [e[0] for e in events]
        self.full_extent = full_extent
        self.t = rec.t0
        self.playing = False
        self.speed = 1.0
        self.show_params = False
        self.show_events = False
        self.param_lines = self._build_param_lines()
        self.nodes, self.links = load_roadmap_overlay(
            rec.configs.get("roadmap", ""))

        # strip default keymaps for owned keys (lesson from roadmap_editor:
        # 's' opens savefig dialog, arrows navigate history, etc.)
        for name, keys in list(plt.rcParams.items()):
            if name.startswith("keymap."):
                plt.rcParams[name] = [k for k in keys
                                      if k not in self.OWNED_KEYS]

        self.fig = plt.figure(figsize=(13, 8.5))
        self.ax = self.fig.add_axes([0.05, 0.14, 0.66, 0.82])
        self.side = self.fig.add_axes([0.73, 0.14, 0.26, 0.82])
        self.side.axis("off")
        slider_ax = self.fig.add_axes([0.08, 0.05, 0.60, 0.03])
        self.slider = Slider(slider_ax, "t (s)", 0.0,
                             max(rec.t1 - rec.t0, 0.001), valinit=0.0)
        self.slider.on_changed(self._on_slider)
        # event ticks on the slider rail
        for et, _, obs in events:
            if rec.t0 <= et <= rec.t1:
                slider_ax.axvline(et - rec.t0, color="#d03b3b" if obs
                                  else "#eda100", alpha=0.5, linewidth=0.8)
        self._in_slider_cb = False
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.timer = self.fig.canvas.new_timer(interval=80)
        self.timer.add_callback(self._tick)
        self.timer.start()
        self.draw()
        plt.show()

    def _build_param_lines(self):
        lines = []
        for name in ("planning", "scenario", "roadmap"):
            text = self.rec.configs.get(name)
            if not text:
                continue
            lines.append(f"── {name} " + "─" * max(0, 24 - len(name)))
            lines.extend(flatten_yaml(text))
        return lines

    # ---- controls ----
    def _on_slider(self, val):
        if self._in_slider_cb:
            return
        self.t = self.rec.t0 + val
        self.playing = False
        self.draw()

    def _set_t(self, t):
        self.t = min(max(t, self.rec.t0), self.rec.t1)
        self._in_slider_cb = True
        self.slider.set_val(self.t - self.rec.t0)
        self._in_slider_cb = False
        self.draw()

    def _on_key(self, ev):
        if ev.key == " ":
            self.playing = not self.playing
        elif ev.key == "left":
            self.playing = False
            self._set_t(self.t - 0.5)
        elif ev.key == "right":
            self.playing = False
            self._set_t(self.t + 0.5)
        elif ev.key == "up":
            self.speed = min(self.speed * 2, 16)
            self.draw()
        elif ev.key == "down":
            self.speed = max(self.speed / 2, 0.25)
            self.draw()
        elif ev.key == "p":
            self.show_params = not self.show_params
            self.show_events = False
            self.draw()
        elif ev.key == "e":
            self.show_events = not self.show_events
            self.show_params = False
            self.draw()
        elif ev.key == "home":
            self._set_t(self.rec.t0)
        elif ev.key == "end":
            self._set_t(self.rec.t1)

    def _tick(self):
        if self.playing:
            nt = self.t + 0.08 * self.speed
            if nt >= self.rec.t1:
                nt = self.rec.t1
                self.playing = False
            self._set_t(nt)

    # ---- drawing ----
    def draw(self):
        ax = self.ax
        ax.clear()
        ax.set_aspect("equal")
        ax.grid(True, linewidth=0.4, alpha=0.4)
        rel = self.t - self.rec.t0
        state = "PLAYING" if self.playing else "PAUSED"
        ax.set_title(f"replay {state} t={rel:6.1f}s  x{self.speed:g}   "
                     "(space/←→/↑↓, p=params, e=events)")
        ws, pp = self.rec.at(self.t)

        # roadmap overlay (standing knowledge)
        for (x1, y1), (x2, y2) in self.links:
            ax.plot([x1, x2], [y1, y2], color="#b48ead", linewidth=1.0,
                    linestyle="--", alpha=0.7, zorder=1)
        for name, (nx, ny) in self.nodes.items():
            ax.plot(nx, ny, marker="s", markersize=4, color="#b48ead",
                    alpha=0.8, zorder=1)
            ax.text(nx, ny + 0.12, name, fontsize=6, ha="center",
                    color="#8a6ba1", alpha=0.9, zorder=1)

        # trail up to t
        trail = self.rec.trail_until(self.t)
        if len(trail) > 1:
            ax.plot([p[0] for p in trail], [p[1] for p in trail],
                    color="#9ec5f4", linewidth=1.2, alpha=0.8, zorder=2)

        if ws is not None:
            self._draw_world(ax, ws)
        if pp is not None:
            self._draw_path(ax, pp)

        # active log event: last event at/before t within 3s
        active = None
        i = bisect.bisect_right(self.ev_t, self.t) - 1
        if i >= 0 and self.t - self.events[i][0] <= 3.0:
            active = self.events[i]
        if active and active[2] is not None:
            ox, oy = active[2]
            ax.plot(ox, oy, marker="x", markersize=14, markeredgewidth=3,
                    color="#d03b3b", zorder=9)

        # HUD
        hud = []
        if pp is not None:
            hud.append(f"state: {pp.behavior_state}  "
                       f"speed: {pp.robot_speed:.2f} m/s")
        if ws is not None:
            n_tracked = sum(1 for d in ws.detections.detections
                            if d.track_id != 0)
            hud.append(f"loc conf: {ws.localization_confidence:.2f}  "
                       f"tracked: {n_tracked}")
        if active:
            hud.append(f"event: {active[1][:90]}")
        if hud:
            ax.text(0.01, 0.99, "\n".join(hud), transform=ax.transAxes,
                    fontsize=8, va="top", family="monospace",
                    bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"))

        if self.full_extent:
            x0, y0, x1, y1 = self.full_extent
            ax.set_xlim(x0, x1)
            ax.set_ylim(y0, y1)
        elif ws is not None:
            rx, ry = ws.robot_pose.x, ws.robot_pose.y
            ax.set_xlim(rx - 6, rx + 6)
            ax.set_ylim(ry - 4.5, ry + 4.5)

        self._draw_side()
        self.fig.canvas.draw_idle()

    def _draw_world(self, ax, ws):
        rx, ry, rt = ws.robot_pose.x, ws.robot_pose.y, ws.robot_pose.theta
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
                ax.add_patch(self.mpl.patches.Circle(
                    (p.x, p.y), r, fill=True, alpha=0.35, color=color,
                    zorder=4))
                if moving:
                    ax.annotate("", xy=(p.x + vx, p.y + vy), xytext=(p.x, p.y),
                                arrowprops=dict(arrowstyle="->", color=color,
                                                lw=1.6), zorder=5)
        if static_x:
            ax.scatter(static_x, static_y, s=6, c="#898781", zorder=3)
        L = 0.28
        pts = [(rx + L * math.cos(rt), ry + L * math.sin(rt)),
               (rx + 0.12 * math.cos(rt + 2.5), ry + 0.12 * math.sin(rt + 2.5)),
               (rx + 0.12 * math.cos(rt - 2.5), ry + 0.12 * math.sin(rt - 2.5))]
        ax.add_patch(self.mpl.patches.Polygon(pts, closed=True,
                                              color="#2a78d6", zorder=7))
        # physical footprint + planning radius rings
        ax.add_patch(self.mpl.patches.Circle((rx, ry), 0.19, fill=False,
                     color="#2a78d6", linewidth=0.8, alpha=0.6, zorder=7))
        ax.add_patch(self.mpl.patches.Circle((rx, ry), 0.24, fill=False,
                     color="#2a78d6", linewidth=0.6, alpha=0.35,
                     linestyle=":", zorder=7))

    def _draw_path(self, ax, pp):
        if len(pp.global_path) > 1:
            ax.plot([q.x for q in pp.global_path],
                    [q.y for q in pp.global_path],
                    color="#2a78d6", linewidth=1.6, alpha=0.9, zorder=3)
        if len(pp.local_trajectory) > 1:
            ax.plot([q.x for q in pp.local_trajectory],
                    [q.y for q in pp.local_trajectory],
                    color="#1baf7a", linewidth=2.6, zorder=6)
        for i, wp in enumerate(pp.scenario_waypoints):
            current = i == pp.current_waypoint_index
            ax.plot(wp.x, wp.y, marker="o", markersize=9 if current else 6,
                    markerfacecolor="none",
                    markeredgecolor="#d03b3b" if current else "#898781",
                    markeredgewidth=1.6, zorder=5)
        if pp.HasField("current_goal"):
            g = pp.current_goal
            ax.plot(g.x, g.y, marker="*", markersize=15, color="#d03b3b",
                    zorder=6)

    def _draw_side(self):
        self.side.clear()
        self.side.axis("off")
        if self.show_params:
            lines = self.param_lines[:96]
            self.side.text(0.0, 1.0, "\n".join(lines), fontsize=5.4,
                           va="top", family="monospace",
                           transform=self.side.transAxes)
        elif self.show_events:
            i = bisect.bisect_right(self.ev_t, self.t)
            window = self.events[max(0, i - 28):i]
            lines = [f"{et - self.rec.t0:7.1f}s {msg[:60]}"
                     for et, msg, _ in window]
            self.side.text(0.0, 1.0, "\n".join(lines) or "(no events yet)",
                           fontsize=6, va="top", family="monospace",
                           transform=self.side.transAxes)
        else:
            self.side.text(
                0.0, 1.0,
                "p — parameter panel\n"
                "e — event list (needs --log)\n"
                "space — play/pause\n"
                "←/→ — step 0.5s\n"
                "↑/↓ — speed\n"
                "home/end — jump",
                fontsize=8, va="top", family="monospace",
                transform=self.side.transAxes)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    rp = sub.add_parser("record", help="record CH1+CH10 to a .limorec file")
    rp.add_argument("--out", required=True)
    rp.add_argument("--scenario", help="scenario yaml to snapshot in header")
    pp_ = sub.add_parser("play", help="replay a .limorec file")
    pp_.add_argument("file")
    pp_.add_argument("--log", help="planner run log to align on the timeline")
    pp_.add_argument("--full", nargs="?", const="-1.5,-4.2,20.5,4.2",
                     metavar="X0,Y0,X1,Y1",
                     help="fixed whole-course extent (default fits gauntlet)")
    args = ap.parse_args()

    if args.cmd == "record":
        record(args.out, args.scenario)
        return

    rec = Recording(args.file)
    events = parse_log(args.log) if args.log else []
    extent = None
    if args.full:
        try:
            extent = tuple(float(v) for v in args.full.split(","))
            assert len(extent) == 4
        except (ValueError, AssertionError):
            ap.error(f"--full expects X0,Y0,X1,Y1 — got '{args.full}'")
    print(f"replay: {len(rec.world_frames)} world frames, "
          f"{len(rec.path_frames)} path frames, "
          f"{rec.t1 - rec.t0:.1f}s span, {len(events)} log events")
    global _player_anchor  # GC anchor (matplotlib holds callbacks as weakrefs)
    _player_anchor = Player(rec, events, extent)


if __name__ == "__main__":
    main()
