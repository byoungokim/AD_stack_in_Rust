#!/usr/bin/env python3
"""Interactive editor + route explorer for prior roadmap files.

Loads a node-link roadmap YAML (see config/maps/) over the world's
ground-truth obstacle fixtures, lets you edit the network graphically,
and previews routes with the same semantics as the Rust planner
(planning/src/roadmap.rs): travel-time Dijkstra with mid-link snapping
of both endpoints, oneway-aware.

Usage:
    .venv/bin/python tools/roadmap_editor.py \
        [--map config/maps/obstacle_gauntlet_roadmap.yaml] \
        [--obstacles simulation/tests/fixtures/gauntlet_obstacles.json]

    # headless route query (no GUI): prints the route and exits
    ... --route 0,0 17.9,0.5

    # headless render to PNG (optionally with --route overlay)
    ... --snapshot /tmp/map.png [--route 0,0 17.9,0.5]

Interactive controls:
    drag node      move it (0.05 m snap)
    a              add a node at the cursor
    l              link mode: press over the first node, then the second
    d              delete the node under the cursor (with its links),
                   else the nearest link
    e              edit nearest link's speed/width (terminal prompt)
    n              rename the node under the cursor (terminal prompt)
    r              route mode: click the start point, then the goal —
                   the route is drawn with length / est. time
    c              clear the route overlay
    s              save the YAML (preserves the file's header comments)
    q              quit

Obstacle fixtures are in the world frame; the roadmap is in the planner
frame (world + (9, 0)) — the tool converts for display.
"""
import argparse
import heapq
import json
import math
import sys

import yaml

WORLD_TO_PLANNER_X = 9.0
NODE_PICK_RADIUS = 0.35
GRID_SNAP = 0.05


# ---------------------------------------------------------------- graph model
class Roadmap:
    def __init__(self, path):
        self.path = path
        with open(path) as f:
            raw = f.read()
        # Preserve everything before the 'nodes:' key (header comments).
        idx = raw.find("nodes:")
        self.header = raw[:idx] if idx > 0 else "frame: planner\n"
        data = yaml.safe_load(raw)
        self.nodes = {n["id"]: [float(n["x"]), float(n["y"])] for n in data["nodes"]}
        self.links = [
            {
                "from": l["from"],
                "to": l["to"],
                "width": float(l["width"]),
                "speed": float(l["speed"]),
                "oneway": bool(l.get("oneway", False)),
            }
            for l in data["links"]
        ]

    def save(self):
        out = [self.header.rstrip("\n"), "nodes:"]
        for nid, (x, y) in self.nodes.items():
            out.append(f"  - {{id: {nid}, x: {x:.2f}, y: {y:.2f}}}")
        out.append("links:")
        for l in self.links:
            one = ", oneway: true" if l["oneway"] else ""
            out.append(
                f"  - {{from: {l['from']}, to: {l['to']}, "
                f"width: {l['width']:.2f}, speed: {l['speed']:.2f}{one}}}"
            )
        with open(self.path, "w") as f:
            f.write("\n".join(out) + "\n")

    # ---------------------------------------------------------- route query
    def _seg_project(self, p, a, b):
        ax, ay = a
        bx, by = b
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0:
            return a, 0.0
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        return (ax + t * dx, ay + t * dy), t

    def route(self, start, goal):
        """Travel-time Dijkstra with both endpoints snapped to the nearest
        point on any link segment (mirrors planning/src/roadmap.rs)."""

        def snap(p):
            best = None
            for i, l in enumerate(self.links):
                a, b = self.nodes[l["from"]], self.nodes[l["to"]]
                q, t = self._seg_project(p, a, b)
                d = math.dist(p, q)
                if best is None or d < best[0]:
                    best = (d, i, q, t)
            return best  # (dist, link_idx, point, t)

        if not self.links:
            return None
        s = snap(start)
        g = snap(goal)

        # Vertices: node ids + 'S' + 'G'. Edge list built on the fly.
        edges = {}  # v -> list of (w, cost, via_points)

        def add(u, v, cost, pts):
            edges.setdefault(u, []).append((v, cost, pts))

        for l in self.links:
            a, b = self.nodes[l["from"]], self.nodes[l["to"]]
            cost = math.dist(a, b) / l["speed"]
            add(l["from"], l["to"], cost, [tuple(a), tuple(b)])
            if not l["oneway"]:
                add(l["to"], l["from"], cost, [tuple(b), tuple(a)])

        def attach(tag, snapped, point, is_start):
            d, li, q, _t = snapped
            l = self.links[li]
            a, b = self.nodes[l["from"]], self.nodes[l["to"]]
            if is_start:
                # S -> both endpoint directions along the snapped link
                add(tag, l["to"], math.dist(q, b) / l["speed"], [point, tuple(q), tuple(b)])
                if not l["oneway"]:
                    add(tag, l["from"], math.dist(q, a) / l["speed"], [point, tuple(q), tuple(a)])
            else:
                add(l["from"], tag, math.dist(a, q) / l["speed"], [tuple(a), tuple(q), point])
                if not l["oneway"]:
                    add(l["to"], tag, math.dist(b, q) / l["speed"], [tuple(b), tuple(q), point])

        attach("S", s, tuple(start), True)
        attach("G", g, tuple(goal), False)
        # Same-link shortcut: start and goal on one link.
        if s[1] == g[1]:
            l = self.links[s[1]]
            if not l["oneway"] or s[3] <= g[3]:
                cost = math.dist(s[2], g[2]) / l["speed"]
                add("S", "G", cost, [tuple(start), s[2], g[2], tuple(goal)])

        dist = {"S": 0.0}
        prev = {}
        pq = [(0.0, "S")]
        while pq:
            d, u = heapq.heappop(pq)
            if u == "G":
                break
            if d > dist.get(u, math.inf):
                continue
            for v, c, pts in edges.get(u, []):
                nd = d + c
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = (u, pts)
                    heapq.heappush(pq, (nd, v))
        if "G" not in dist:
            return None
        # Reconstruct polyline
        pts = []
        v = "G"
        chain = []
        while v != "S":
            u, seg = prev[v]
            chain.append(seg)
            v = u
        for seg in reversed(chain):
            for p in seg:
                if not pts or math.dist(pts[-1], p) > 1e-6:
                    pts.append(p)
        length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        return {"points": pts, "time": dist["G"], "length": length}


# ------------------------------------------------------------------ rendering
def load_obstacles(path):
    try:
        with open(path) as f:
            return json.load(f)
    except OSError:
        return []


def draw(ax, rm, obstacles, route, mode_text="", hover=None, selected=None):
    import matplotlib.patches as mpatches
    from matplotlib import cm

    ax.clear()
    for o in obstacles:
        x = o["x"] + WORLD_TO_PLANNER_X
        y = o["y"]
        if o.get("type") == "box":
            sx, sy = o.get("sx", 0.3), o.get("sy", 0.3)
            yaw = math.degrees(o.get("yaw", 0.0))
            ax.add_patch(
                mpatches.Rectangle(
                    (x - sx / 2, y - sy / 2), sx, sy, angle=yaw,
                    rotation_point="center", color="0.75", zorder=1,
                )
            )
        else:
            ax.add_patch(mpatches.Circle((x, y), o.get("r", 0.15), color="0.75", zorder=1))

    for l in rm.links:
        a, b = rm.nodes[l["from"]], rm.nodes[l["to"]]
        color = cm.viridis(min(l["speed"] / 2.2, 1.0))
        ax.plot(
            [a[0], b[0]], [a[1], b[1]],
            color=color, linewidth=1.0 + 3.0 * l["width"], alpha=0.55, zorder=2,
            solid_capstyle="round",
        )
        mx, my = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2
        ax.annotate(f"{l['speed']:.1f}", (mx, my), fontsize=7, color="0.3", zorder=4)
        if l["oneway"]:
            ax.annotate(
                "", xy=b, xytext=a,
                arrowprops=dict(arrowstyle="->", color=color, lw=1.2), zorder=3,
            )

    node_artists = {}
    for nid, (x, y) in rm.nodes.items():
        if nid == selected:
            (art,) = ax.plot(x, y, "o", color="tab:orange", markersize=13,
                             zorder=5, markeredgecolor="darkorange",
                             markeredgewidth=2, pickradius=12)
        elif nid == hover:
            (art,) = ax.plot(x, y, "o", color="tab:blue", markersize=13,
                             zorder=5, markeredgecolor="gold",
                             markeredgewidth=2.5, pickradius=12)
        else:
            (art,) = ax.plot(x, y, "o", color="tab:blue", markersize=9,
                             zorder=5, pickradius=12)
        node_artists[nid] = art
        ax.annotate(nid, (x, y), textcoords="offset points", xytext=(6, 6),
                    fontsize=8, zorder=6,
                    fontweight="bold" if nid in (hover, selected) else "normal")

    title = "roadmap editor — a:add l:link d:del e:edit n:rename r:route c:clear s:save q:quit"
    if route:
        xs = [p[0] for p in route["points"]]
        ys = [p[1] for p in route["points"]]
        ax.plot(xs, ys, color="tab:orange", linewidth=3.5, alpha=0.9, zorder=7)
        ax.plot(xs[0], ys[0], "^", color="tab:green", markersize=12, zorder=8)
        ax.plot(xs[-1], ys[-1], "*", color="tab:red", markersize=16, zorder=8)
        title = f"route: {route['length']:.1f} m, est {route['time']:.1f} s   ({mode_text})" \
            if mode_text else f"route: {route['length']:.1f} m, est {route['time']:.1f} s"
    elif mode_text:
        title = mode_text
    if selected is not None and selected in rm.nodes:
        sx, sy = rm.nodes[selected]
        title += f"   |  selected: {selected} ({sx:.2f}, {sy:.2f}) — arrows nudge, d delete, n rename"
    ax.set_title(title, fontsize=9)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)
    return node_artists


# ---------------------------------------------------------------- interaction
class Editor:
    def __init__(self, rm, obstacles):
        self.rm = rm
        self.obstacles = obstacles
        self.route = None
        self.mode = None          # None | 'link1' | ('link2', first_id) | 'route1' | ('route2', start)
        self.drag = None
        self.hover = None
        self.selected = None
        import matplotlib
        owned = {"a", "l", "d", "e", "n", "r", "c", "s", "q",
                 "up", "down", "left", "right"}
        for param in list(matplotlib.rcParams):
            if param.startswith("keymap."):
                matplotlib.rcParams[param] = [
                    k for k in matplotlib.rcParams[param] if k not in owned]
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(13, 6))
        try:
            self.fig.canvas.manager.set_window_title(
                f"roadmap editor — {self.rm.path}")
        except Exception:
            pass
        # Context: what file this is and how it reaches the simulation.
        self.fig.text(
            0.5, 0.015,
            f"editing {self.rm.path}  —  loaded by limo_planning at startup "
            "(config/planning.yaml: roadmap.file)  —  's' saves IN PLACE, "
            "no rebuild; next sim run uses it",
            ha="center", fontsize=8, color="0.35")
        self.fig._roadmap_editor = self  # second GC anchor via the figure
        self.log_events = True
        self._motion_count = 0
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.redraw()

    def redraw(self, msg=""):
        self.node_artists = draw(
            self.ax, self.rm, self.obstacles, self.route, msg,
            hover=self.hover, selected=self.selected)
        self.fig.canvas.draw_idle()

    def node_at(self, x, y, ev=None):
        """Hit-test via the node artists' own contains(): matplotlib handles
        every backend's coordinate frames (incl. Retina scaling) internally,
        which manual pixel math got wrong. Data-space fallback without an
        event."""
        if ev is not None:
            for nid, art in getattr(self, "node_artists", {}).items():
                try:
                    hit, _ = art.contains(ev)
                except Exception:
                    hit = False
                if hit:
                    return nid
            return None
        best = None
        for nid, (nx, ny) in self.rm.nodes.items():
            d = math.dist((x, y), (nx, ny))
            if d < NODE_PICK_RADIUS and (best is None or d < best[0]):
                best = (d, nid)
        return best[1] if best else None

    def link_at(self, x, y):
        best = None
        for i, l in enumerate(self.rm.links):
            a, b = self.rm.nodes[l["from"]], self.rm.nodes[l["to"]]
            q, _ = self.rm._seg_project((x, y), a, b)
            d = math.dist((x, y), q)
            if best is None or d < best[0]:
                best = (d, i)
        return best[1] if best and best[0] < 0.5 else None

    # ------------------------------------------------------------- handlers
    def on_press(self, ev):
        if self.log_events:
            print(f"[event] press button={ev.button} inaxes={ev.inaxes is self.ax} "
                  f"data=({ev.xdata}, {ev.ydata}) px=({ev.x},{ev.y})", flush=True)
        if ev.inaxes != self.ax or ev.xdata is None:
            return
        # The navigation toolbar's pan/zoom modes swallow drags — ignore
        # editor interactions while one is active so behavior is predictable.
        toolbar = getattr(self.fig.canvas, "toolbar", None)
        if toolbar is not None and getattr(toolbar, "mode", ""):
            return
        x, y = ev.xdata, ev.ydata
        if self.mode == "link1":
            nid = self.node_at(x, y, ev)
            if nid:
                self.mode = ("link2", nid)
                self.redraw(f"link: {nid} -> click second node")
            return
        if isinstance(self.mode, tuple) and self.mode[0] == "link2":
            nid = self.node_at(x, y, ev)
            if nid and nid != self.mode[1]:
                self.rm.links.append(
                    {"from": self.mode[1], "to": nid, "width": 1.0, "speed": 1.0, "oneway": False}
                )
                self.mode = None
                self.redraw(f"linked {self.mode} (width 1.0, speed 1.0 — press e to edit)")
            return
        if self.mode == "route1":
            self.mode = ("route2", (x, y))
            self.redraw("route: click the goal point")
            return
        if isinstance(self.mode, tuple) and self.mode[0] == "route2":
            self.route = self.rm.route(self.mode[1], (x, y))
            self.mode = None
            self.redraw("" if self.route else "no route found")
            return
        nid = self.node_at(x, y, ev)
        if nid:
            self.selected = nid
            self.drag = nid
            self.redraw()
        elif self.selected is not None:
            self.selected = None
            self.redraw()

    def on_motion(self, ev):
        self._motion_count += 1
        if self.log_events and (self._motion_count == 1 or self._motion_count % 200 == 0):
            print(f"[event] motion #{self._motion_count} inaxes={ev.inaxes is self.ax} "
                  f"data=({ev.xdata}, {ev.ydata})", flush=True)
        if not self.drag and ev.inaxes == self.ax and ev.xdata is not None:
            nid = self.node_at(ev.xdata, ev.ydata, ev)
            if nid != self.hover:
                if self.log_events:
                    print(f"[event] hover -> {nid}", flush=True)
                self.hover = nid
                self.redraw()
            return
        if self.drag and ev.inaxes == self.ax and ev.xdata is not None:
            self.rm.nodes[self.drag] = [
                round(ev.xdata / GRID_SNAP) * GRID_SNAP,
                round(ev.ydata / GRID_SNAP) * GRID_SNAP,
            ]
            # Full redraws at mouse-move rate make dragging feel dead on the
            # macOS backend — throttle to ~30 fps; release does a final one.
            import time as _t
            now = _t.monotonic()
            if now - getattr(self, "_last_motion_draw", 0.0) > 0.033:
                self._last_motion_draw = now
                self.redraw()

    def on_release(self, _ev):
        if self.drag:
            self.drag = None
            self.redraw()
        self.drag = None

    def on_key(self, ev):
        if self.log_events:
            print(f"[event] key '{ev.key}'", flush=True)
        x = ev.xdata if ev.inaxes == self.ax else None
        y = ev.ydata if ev.inaxes == self.ax else None
        if ev.key == "a" and x is not None:
            base = "node"
            i = 0
            while f"{base}_{i}" in self.rm.nodes:
                i += 1
            self.rm.nodes[f"{base}_{i}"] = [
                round(x / GRID_SNAP) * GRID_SNAP, round(y / GRID_SNAP) * GRID_SNAP]
            self.redraw(f"added node_{i} (press n over it to rename)")
        elif ev.key == "l":
            self.mode = "link1"
            self.redraw("link: click the first node")
        elif ev.key == "d":
            nid = (self.node_at(x, y, ev) if x is not None else None) or self.selected
            if nid:
                self.rm.links = [
                    l for l in self.rm.links if l["from"] != nid and l["to"] != nid]
                del self.rm.nodes[nid]
                if self.selected == nid:
                    self.selected = None
                if self.hover == nid:
                    self.hover = None
                self.redraw(f"deleted {nid}")
            else:
                li = self.link_at(x, y)
                if li is not None:
                    l = self.rm.links.pop(li)
                    self.redraw(f"deleted link {l['from']}->{l['to']}")
        elif ev.key == "e" and x is not None:
            li = self.link_at(x, y)
            if li is not None:
                l = self.rm.links[li]
                print(f"editing link {l['from']}->{l['to']} "
                      f"(speed {l['speed']}, width {l['width']})")
                try:
                    sp = input("  speed m/s (enter=keep): ").strip()
                    wd = input("  width m  (enter=keep): ").strip()
                    if sp:
                        l["speed"] = float(sp)
                    if wd:
                        l["width"] = float(wd)
                except (ValueError, EOFError):
                    print("  no terminal input available — unchanged "
                          "(launch from a terminal for prompts)")
                self.redraw()
        elif ev.key == "n":
            nid = (self.node_at(x, y, ev) if x is not None else None) or self.selected
            if nid:
                try:
                    new = input(f"rename '{nid}' to: ").strip()
                except EOFError:
                    print("no terminal input available — launch from a terminal")
                    return
                if new and new not in self.rm.nodes:
                    self.rm.nodes[new] = self.rm.nodes.pop(nid)
                    for l in self.rm.links:
                        if l["from"] == nid:
                            l["from"] = new
                        if l["to"] == nid:
                            l["to"] = new
                    self.redraw(f"renamed {nid} -> {new}")
        elif ev.key in ("up", "down", "left", "right") and self.selected:
            dx = {"left": -GRID_SNAP, "right": GRID_SNAP}.get(ev.key, 0.0)
            dy = {"down": -GRID_SNAP, "up": GRID_SNAP}.get(ev.key, 0.0)
            px, py = self.rm.nodes[self.selected]
            self.rm.nodes[self.selected] = [round(px + dx, 4), round(py + dy, 4)]
            self.redraw()
        elif ev.key == "r":
            self.mode = "route1"
            self.redraw("route: click the start point")
        elif ev.key == "c":
            self.route = None
            self.redraw()
        elif ev.key == "s":
            self.rm.save()
            import os
            print(f"[saved] {os.path.abspath(self.rm.path)}", flush=True)
            self.redraw(f"saved {self.rm.path} — next sim launch routes on it")
        elif ev.key == "q":
            self.plt.close(self.fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--map", default="config/maps/obstacle_gauntlet_roadmap.yaml")
    ap.add_argument("--obstacles",
                    default="simulation/tests/fixtures/gauntlet_obstacles.json")
    ap.add_argument("--route", nargs=2, metavar=("X1,Y1", "X2,Y2"),
                    help="headless route query: start and goal as x,y")
    ap.add_argument("--snapshot", metavar="OUT.png",
                    help="headless render to PNG and exit")
    args = ap.parse_args()

    rm = Roadmap(args.map)
    obstacles = load_obstacles(args.obstacles)

    route = None
    if args.route:
        p1 = tuple(float(v) for v in args.route[0].split(","))
        p2 = tuple(float(v) for v in args.route[1].split(","))
        route = rm.route(p1, p2)
        if route is None:
            print("NO ROUTE")
            if not args.snapshot:
                sys.exit(1)
        else:
            pts = " -> ".join(f"({x:.1f},{y:.1f})" for x, y in route["points"])
            print(f"ROUTE length={route['length']:.2f}m "
                  f"est_time={route['time']:.2f}s avg={route['length']/route['time']:.2f}m/s")
            print(pts)
        if not args.snapshot:
            return

    import matplotlib
    if args.snapshot:
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(13, 6))
        draw(ax, rm, obstacles, route)
        fig.savefig(args.snapshot, dpi=110, bbox_inches="tight")
        print(f"saved {args.snapshot}")
        return

    editor = Editor(rm, obstacles)          # keep alive: mpl callbacks are weakrefs
    import matplotlib.pyplot as plt
    plt.show()
    del editor


if __name__ == "__main__":
    main()
