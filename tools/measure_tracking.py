#!/usr/bin/env python3
"""Tracking-loop measurement from a .limorec recording.

Quantifies the three numbers the tracking discussion needs, from data
instead of estimates:

  1. Command->execution latency: cross-correlation lag between the CH2
     commanded twist and the CH3 achieved twist (angular is the sharpest
     signal; linear reported too). This is the number actuation_delay_s
     should be set to.
  2. Execution gain: achieved/commanded amplitude ratio — a gain well below
     1.0 means the bridge/chassis dynamics filter our commands and the
     planner is steering a sluggish plant.
  3. Reference tracking error: cross-track distance from the executed pose
     (CH1) to the CURRENT global path (CH10) over the run — mean / p95 /
     max, plus the same restricted to "cruise" samples (speed > 0.3 m/s,
     path >= 2 waypoints) so recovery holds don't dominate the statistic.

Usage:
    .venv/bin/python tools/measure_tracking.py <run.limorec>
"""
import bisect
import sys

sys.path.insert(0, "proto/gen_py")
sys.path.insert(0, "tools/visualizer")

from control_pb2 import ControlCommand, VehicleState  # noqa: E402
from replay_view import Recording  # noqa: E402


def series_cmd(rec):
    out = []
    for t, payload in rec.cmd_frames:
        try:
            c = ControlCommand()
            c.ParseFromString(payload)
        except Exception:
            continue
        if c.HasField("velocity_cmd"):
            out.append((t, c.velocity_cmd.linear_x, c.velocity_cmd.angular_z))
    return out


def series_state(rec):
    out = []
    for t, payload in rec.state_frames:
        try:
            s = VehicleState()
            s.ParseFromString(payload)
        except Exception:
            continue
        v = s.odometry_velocity
        out.append((t, v.linear_x, v.angular_z))
    return out


def resample(series, t0, t1, dt):
    """Zero-order hold resample of (t, v, w) onto a regular grid."""
    ts = [s[0] for s in series]
    grid_v, grid_w = [], []
    t = t0
    while t <= t1:
        i = bisect.bisect_right(ts, t) - 1
        if i >= 0:
            grid_v.append(series[i][1])
            grid_w.append(series[i][2])
        else:
            grid_v.append(0.0)
            grid_w.append(0.0)
        t += dt
    return grid_v, grid_w


def xcorr_lag(a, b, dt, max_lag_s):
    """Lag (s) maximizing correlation of b against a (b delayed vs a)."""
    n = len(a)
    max_shift = int(max_lag_s / dt)
    mean_a = sum(a) / n
    mean_b = sum(b) / n
    a = [x - mean_a for x in a]
    b = [x - mean_b for x in b]
    best = (0.0, -1e18)
    for shift in range(0, max_shift + 1):
        m = n - shift
        if m < 10:
            break
        c = sum(a[i] * b[i + shift] for i in range(m)) / m
        if c > best[1]:
            best = (shift * dt, c)
    return best[0]


def std(xs):
    n = len(xs)
    if n < 2:
        return 0.0
    m = sum(xs) / n
    return (sum((x - m) ** 2 for x in xs) / (n - 1)) ** 0.5


def cross_track(rec):
    """Per-CH1-frame distance to the concurrent CH10 global path."""
    pts = []
    pt_times = [t for t, _ in rec.path_frames]
    for t, ws in rec.world_frames:
        j = bisect.bisect_right(pt_times, t) - 1
        if j < 0:
            continue
        pp = rec.path_frames[j][1]
        path = pp.global_path
        if len(path) < 2:
            continue
        x, y = ws.robot_pose.x, ws.robot_pose.y
        best = None
        for k in range(len(path) - 1):
            ax, ay, bx, by = path[k].x, path[k].y, path[k + 1].x, path[k + 1].y
            dx, dy = bx - ax, by - ay
            l2 = dx * dx + dy * dy
            u = ((x - ax) * dx + (y - ay) * dy) / l2 if l2 > 1e-12 else 0.0
            u = min(1.0, max(0.0, u))
            px, py = ax + u * dx, ay + u * dy
            d = ((x - px) ** 2 + (y - py) ** 2) ** 0.5
            if best is None or d < best:
                best = d
        pts.append((t, best, pp.robot_speed))
    return pts


def pctl(xs, q):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))]


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    rec = Recording(sys.argv[1])
    cmd = series_cmd(rec)
    ach = series_state(rec)
    print(f"frames: cmd={len(cmd)} state={len(ach)} "
          f"world={len(rec.world_frames)} path={len(rec.path_frames)}")
    if len(cmd) > 50 and len(ach) > 50:
        t0 = max(cmd[0][0], ach[0][0])
        t1 = min(cmd[-1][0], ach[-1][0])
        dt = 0.02
        cv, cw = resample(cmd, t0, t1, dt)
        av, aw = resample(ach, t0, t1, dt)
        lag_w = xcorr_lag(cw, aw, dt, 1.0)
        lag_v = xcorr_lag(cv, av, dt, 1.0)
        print(f"latency  cmd->achieved: angular {lag_w*1000:.0f} ms, "
              f"linear {lag_v*1000:.0f} ms   "
              f"(actuation_delay_s currently guesses 0.2)")
        gw = std(aw) / std(cw) if std(cw) > 1e-6 else float("nan")
        gv = std(av) / std(cv) if std(cv) > 1e-6 else float("nan")
        print(f"gain     achieved/commanded: angular {gw:.2f}, linear {gv:.2f}")
    else:
        print("insufficient CH2/CH3 frames for latency analysis")

    pts = cross_track(rec)
    if pts:
        all_d = [d for _, d, _ in pts]
        cruise = [d for _, d, v in pts if v > 0.3]
        print(f"cross-track ALL:    mean {sum(all_d)/len(all_d):.3f}m  "
              f"p95 {pctl(all_d, 0.95):.3f}m  max {max(all_d):.3f}m  "
              f"(n={len(all_d)})")
        if cruise:
            print(f"cross-track CRUISE: mean {sum(cruise)/len(cruise):.3f}m  "
                  f"p95 {pctl(cruise, 0.95):.3f}m  max {max(cruise):.3f}m  "
                  f"(n={len(cruise)})")
    else:
        print("no cross-track samples (missing CH1/CH10)")


if __name__ == "__main__":
    main()
