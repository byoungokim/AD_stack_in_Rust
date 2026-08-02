//! Path smoothing stage for the Hybrid A* global planner.
//!
//! The raw A* output is a chain of grid motion primitives: jagged and
//! curvature-discontinuous at the joints. The pure-pursuit executor tracks
//! whatever line it is given, so the line itself must be good. Every
//! successful plan passes through here before publication:
//!
//! - **Stage A — shortcutting**: waypoint subchains are replaced by straight
//!   segments where the segment (sampled at ≤5cm against the occupancy grid's
//!   hard inflation) stays collision-free AND does not reduce the replaced
//!   subchain's minimum clearance (measured on the capped chamfer
//!   [`ClearanceField`]). Since the field caps at the soft band's outer edge,
//!   this is exactly "keep at least the soft-band midpoint where the corridor
//!   allows, and never shortcut INTO the band below the original path's
//!   clearance" — open-space zigzags collapse to straight lines, while a
//!   detour A* paid for around an obstacle is never traded for edge-hugging.
//! - **Stage B — curvature-bounded gradient smoothing**: uniform ~0.15m
//!   resampling, then Jacobi iterations of
//!   `x_i += α·(x_{i-1} + x_{i+1} − 2·x_i) + β·(1 − d/decay)·∇d` with fixed
//!   endpoints (the β term is `−β·∇(clearance penalty)`), followed by a
//!   curvature-relaxation pass confined to vertices exceeding the bound.
//! - **Post-check**: the result is rejected — falling back to the raw path —
//!   if any 3-point discrete (Menger) curvature exceeds
//!   `0.9 × dwa.max_curvature` or any ≤5cm sample violates hard inflation.

use super::{
    pose_blocked, ClearanceField, EscapeZone, HybridAStarConfig, OccupancyGrid, PathWaypoint,
    SegmentDir, PATH_SAMPLE_STEP_M,
};

/// Output waypoint spacing (m) of the smoothed path.
pub const RESAMPLE_SPACING_M: f64 = 0.15;

/// The smoothed path must stay below this fraction of the executor's
/// curvature envelope so tracking error never pushes commands outside it.
const CURVATURE_SAFETY_FRACTION: f64 = 0.9;

/// Per-iteration cap (m) on the clearance-gradient push, for stability on the
/// grid-quantized field (a full β·(1 − d/decay) step at d ≈ 0 would be 0.2m).
const MAX_CLEARANCE_PUSH_M: f64 = 0.05;

/// Smooth a raw A* path. Returns the smoothed, uniformly resampled path, or a
/// clone of the raw path when smoothing is disabled, the path is degenerate,
/// or the post-check rejects the smoothed result.
///
/// Direction-aware: the path is split into maximal same-direction subchains
/// at every cusp, and each subchain is shortcut/smoothed/resampled SEPARATELY
/// — never across a cusp. The cusp is an intentional stop-and-switch point;
/// smoothing across it would replace the reversal with a geometrically
/// meaningless average. Cusp points (subchain endpoints) are preserved
/// exactly (both stages pin first/last points).
///
/// `escape` is the optional start-pocket zone (robot wedged inside hard
/// inflation): samples inside it are judged by the true-footprint check
/// throughout — shortcutting, per-iteration move guards, and the final
/// post-check — otherwise the post-check would reject every smoothed escape
/// path on its own initial segment.
pub fn smooth_path(
    raw: &[PathWaypoint],
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    cfg: &HybridAStarConfig,
    executor_max_curvature: f64,
    escape: Option<&EscapeZone>,
) -> Vec<PathWaypoint> {
    if !cfg.smoothing_enabled || raw.len() < 2 {
        return raw.to_vec();
    }
    let kappa_max = CURVATURE_SAFETY_FRACTION * executor_max_curvature;

    let mut out: Vec<PathWaypoint> = Vec::new();
    for run in direction_runs(raw) {
        let dir = raw[run.end].dir;
        let chain = &raw[run.start..=run.end];
        let wps = if chain.len() < 2 {
            chain.to_vec()
        } else {
            smooth_chain(chain, dir, grid, clearance, cfg, kappa_max, escape)
        };
        // Adjacent runs share the cusp waypoint; keep a single copy (the
        // incoming run's, whose `dir` is the arrival direction — matching
        // the A* waypoint contract).
        let skip = usize::from(!out.is_empty());
        out.extend(wps.into_iter().skip(skip));
    }
    out
}

/// Inclusive index range of one same-direction subchain.
struct Run {
    start: usize,
    end: usize,
}

/// Split a path into maximal same-direction runs. Waypoint `dir` is the
/// ARRIVAL direction, and the first waypoint carries the first segment's
/// direction, so a cusp is exactly an index `i >= 1` where `dir` changes;
/// the cusp waypoint terminates the incoming run and anchors the outgoing
/// one (shared point).
fn direction_runs(path: &[PathWaypoint]) -> Vec<Run> {
    let mut runs = Vec::new();
    let mut start = 0usize;
    for i in 1..path.len() {
        if path[i].dir != path[i - 1].dir {
            runs.push(Run { start, end: i - 1 });
            start = i - 1; // cusp point shared with the next run
        }
    }
    runs.push(Run {
        start,
        end: path.len().saturating_sub(1),
    });
    runs
}

/// The original single-direction pipeline: shortcut, resample, gradient
/// smooth, resample, post-check; falls back to the raw subchain when the
/// post-check rejects.
fn smooth_chain(
    raw: &[PathWaypoint],
    dir: SegmentDir,
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    cfg: &HybridAStarConfig,
    kappa_max: f64,
    escape: Option<&EscapeZone>,
) -> Vec<PathWaypoint> {
    let pts: Vec<(f64, f64)> = raw.iter().map(|w| (w.x, w.y)).collect();
    let cut = shortcut(&pts, grid, clearance, escape);
    let mut dense = resample(&cut, RESAMPLE_SPACING_M);
    gradient_smooth(&mut dense, grid, clearance, cfg, kappa_max, escape);
    // Smoothing contracts segments near former corners; restore uniform
    // spacing before the executor sees the path.
    let out = resample(&dense, RESAMPLE_SPACING_M);

    if !post_check(&out, grid, kappa_max, escape) {
        return raw.to_vec();
    }
    to_waypoints(&out, cfg.wheelbase, dir)
}

/// Stage A: greedy farthest-first shortcutting. From each anchor, connect to
/// the farthest waypoint whose straight segment is admissible; endpoints are
/// always preserved.
fn shortcut(
    pts: &[(f64, f64)],
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    escape: Option<&EscapeZone>,
) -> Vec<(f64, f64)> {
    if pts.len() < 3 {
        return pts.to_vec();
    }
    let mut out = vec![pts[0]];
    let mut i = 0;
    while i + 1 < pts.len() {
        // Running minimum clearance of the original subchain i..=k, so the
        // admissibility requirement for a candidate segment is O(1).
        let chain_min: Vec<f64> = clearance
            .map(|c| {
                let mut mins = Vec::with_capacity(pts.len() - i);
                let mut m = f64::INFINITY;
                for p in &pts[i..] {
                    m = m.min(c.distance_at(p.0, p.1));
                    mins.push(m);
                }
                mins
            })
            .unwrap_or_default();

        let mut chosen = i + 1;
        for j in ((i + 2)..pts.len()).rev() {
            let required = clearance.map(|_| chain_min[j - i]);
            if segment_admissible(pts[i], pts[j], grid, clearance, required, escape) {
                chosen = j;
                break;
            }
        }
        out.push(pts[chosen]);
        i = chosen;
    }
    out
}

/// A candidate shortcut segment is admissible when every ≤5cm sample is off
/// occupied cells (hard inflation) and, with the clearance layer on, keeps at
/// least `required` clearance — the replaced subchain's own minimum on the
/// capped chamfer field, so a shortcut never trades away clearance A* paid
/// distance to obtain.
fn segment_admissible(
    a: (f64, f64),
    b: (f64, f64),
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    required: Option<f64>,
    escape: Option<&EscapeZone>,
) -> bool {
    let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    let n = (len / PATH_SAMPLE_STEP_M).ceil().max(1.0) as usize;
    for k in 0..=n {
        let t = k as f64 / n as f64;
        let x = a.0 + (b.0 - a.0) * t;
        let y = a.1 + (b.1 - a.1) * t;
        if pose_blocked(grid, escape, x, y) {
            return false;
        }
        if let (Some(field), Some(req)) = (clearance, required) {
            if field.distance_at(x, y) < req - 1e-9 {
                return false;
            }
        }
    }
    true
}

/// Resample a polyline to (near-)uniform spacing along its arc length. The
/// first and last points are preserved exactly; a stub final interval shorter
/// than half the spacing is merged into its predecessor.
fn resample(pts: &[(f64, f64)], spacing: f64) -> Vec<(f64, f64)> {
    if pts.len() < 2 {
        return pts.to_vec();
    }
    let mut out = vec![pts[0]];
    let mut carried = 0.0; // arc length since the last emitted point
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let seg = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
        if seg < 1e-12 {
            continue;
        }
        let mut along = spacing - carried;
        while along <= seg {
            let t = along / seg;
            out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
            along += spacing;
        }
        carried = seg - (along - spacing);
    }
    let last = *pts.last().unwrap();
    if carried < spacing * 0.5 && out.len() > 1 {
        // Replace the trailing emitted point so the final interval is
        // ~spacing instead of a sub-half stub.
        *out.last_mut().unwrap() = last;
    } else {
        out.push(last);
    }
    out
}

/// Stage B: fixed-endpoint Jacobi gradient smoothing, then a curvature
/// relaxation pass restricted to offending vertices. Every proposed move is
/// rejected if it would land on an occupied cell (hard inflation is never
/// violated mid-iteration).
fn gradient_smooth(
    pts: &mut [(f64, f64)],
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    cfg: &HybridAStarConfig,
    kappa_max: f64,
    escape: Option<&EscapeZone>,
) {
    if pts.len() < 3 {
        return;
    }
    let alpha = cfg.smoothing_alpha;
    let beta = cfg.smoothing_clearance_beta;
    let decay = cfg.clearance_decay_m;

    for _ in 0..cfg.smoothing_iterations {
        smoothing_pass(pts, grid, clearance, alpha, beta, decay, None, escape);
    }

    // Curvature relaxation: extra pure-Laplacian passes applied only where
    // the 3-point curvature still exceeds the bound (dilated by one vertex so
    // the excess can diffuse), within the same iteration budget again.
    for _ in 0..cfg.smoothing_iterations {
        let offending = offending_vertices(pts, kappa_max);
        if offending.is_empty() {
            break;
        }
        smoothing_pass(
            pts,
            grid,
            clearance,
            alpha,
            0.0,
            decay,
            Some(&offending),
            escape,
        );
    }
}

/// One Jacobi smoothing pass. `only` restricts updates to flagged interior
/// vertices (curvature relaxation); `beta == 0.0` disables the clearance push.
#[allow(clippy::too_many_arguments)] // one smoothing-iteration contract, not a config bundle
fn smoothing_pass(
    pts: &mut [(f64, f64)],
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    alpha: f64,
    beta: f64,
    decay: f64,
    only: Option<&[bool]>,
    escape: Option<&EscapeZone>,
) {
    let snapshot: Vec<(f64, f64)> = pts.to_vec();
    for i in 1..snapshot.len() - 1 {
        if let Some(mask) = only {
            if !mask[i] {
                continue;
            }
        }
        let (px, py) = snapshot[i];
        let mut dx = alpha * (snapshot[i - 1].0 + snapshot[i + 1].0 - 2.0 * px);
        let mut dy = alpha * (snapshot[i - 1].1 + snapshot[i + 1].1 - 2.0 * py);
        if beta > 0.0 && decay > 0.0 {
            if let Some(field) = clearance {
                let d = field.distance_at(px, py);
                if d < decay {
                    // −β·∇(½(decay−d)²) ∝ β·(1 − d/decay)·∇d, with ∇d from
                    // central differences at the field's own resolution.
                    let h = grid.resolution;
                    let gx =
                        (field.distance_at(px + h, py) - field.distance_at(px - h, py)) / (2.0 * h);
                    let gy =
                        (field.distance_at(px, py + h) - field.distance_at(px, py - h)) / (2.0 * h);
                    let scale = beta * (1.0 - d / decay);
                    let (mut cx, mut cy) = (scale * gx, scale * gy);
                    let mag = (cx * cx + cy * cy).sqrt();
                    if mag > MAX_CLEARANCE_PUSH_M {
                        cx *= MAX_CLEARANCE_PUSH_M / mag;
                        cy *= MAX_CLEARANCE_PUSH_M / mag;
                    }
                    dx += cx;
                    dy += cy;
                }
            }
        }
        let (nx, ny) = (px + dx, py + dy);
        if !pose_blocked(grid, escape, nx, ny) {
            pts[i] = (nx, ny);
        }
    }
}

/// Interior vertices whose Menger curvature exceeds the bound, dilated by one
/// neighbor on each side.
fn offending_vertices(pts: &[(f64, f64)], kappa_max: f64) -> Vec<bool> {
    let mut mask = vec![false; pts.len()];
    let mut any = false;
    for i in 1..pts.len() - 1 {
        if menger_curvature(pts[i - 1], pts[i], pts[i + 1]).abs() > kappa_max {
            mask[i - 1] = true;
            mask[i] = true;
            mask[i + 1] = true;
            any = true;
        }
    }
    if any {
        mask
    } else {
        Vec::new()
    }
}

/// Signed 3-point discrete (Menger) curvature: 2·cross / (|ab|·|bc|·|ac|).
/// Positive for a left turn.
fn menger_curvature(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    let cross = (b.0 - a.0) * (c.1 - b.1) - (b.1 - a.1) * (c.0 - b.0);
    let ab = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    let bc = ((c.0 - b.0).powi(2) + (c.1 - b.1).powi(2)).sqrt();
    let ac = ((c.0 - a.0).powi(2) + (c.1 - a.1).powi(2)).sqrt();
    let denom = ab * bc * ac;
    if denom < 1e-12 {
        0.0
    } else {
        2.0 * cross / denom
    }
}

/// Final acceptance: every ≤5cm segment sample off occupied cells (relaxed to
/// the true footprint inside the escape zone) AND every 3-point discrete
/// curvature within the bound. On failure the caller falls back to the raw
/// A* path.
fn post_check(
    pts: &[(f64, f64)],
    grid: &OccupancyGrid,
    kappa_max: f64,
    escape: Option<&EscapeZone>,
) -> bool {
    if pts.len() < 2 {
        return false;
    }
    for w in pts.windows(2) {
        if !segment_admissible(w[0], w[1], grid, None, None, escape) {
            return false;
        }
    }
    pts.windows(3)
        .all(|w| menger_curvature(w[0], w[1], w[2]).abs() <= kappa_max + 1e-9)
}

/// Rebuild `PathWaypoint`s from smoothed points: headings from central
/// differences, steering feed-forward from the signed discrete curvature
/// (`δ = atan(κ·wheelbase)`, the inverse of the bicycle model). On a REVERSE
/// subchain the robot's heading is the travel tangent rotated by π (it backs
/// along the curve), and the steering sign flips with the travel direction
/// (`tan δ = wb·dθ/ds_travel`, and ds_travel is negative).
fn to_waypoints(pts: &[(f64, f64)], wheelbase: f64, dir: SegmentDir) -> Vec<PathWaypoint> {
    let n = pts.len();
    let flip = if dir == SegmentDir::Reverse {
        std::f64::consts::PI
    } else {
        0.0
    };
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let (bx, by) = if i == 0 { pts[0] } else { pts[i - 1] };
        let (fx, fy) = if i + 1 == n { pts[n - 1] } else { pts[i + 1] };
        let theta = normalize_angle((fy - by).atan2(fx - bx) + flip);
        let kappa = if i > 0 && i + 1 < n {
            menger_curvature(pts[i - 1], pts[i], pts[i + 1])
        } else {
            0.0
        };
        out.push(PathWaypoint {
            x: pts[i].0,
            y: pts[i].1,
            theta,
            steering: (dir.sign() * kappa * wheelbase).atan(),
            dir,
        });
    }
    out
}

fn normalize_angle(a: f64) -> f64 {
    let mut v = a;
    while v > std::f64::consts::PI {
        v -= 2.0 * std::f64::consts::PI;
    }
    while v < -std::f64::consts::PI {
        v += 2.0 * std::f64::consts::PI;
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Executor curvature envelope (dwa.max_curvature); the smoother bound is
    /// 0.9 × this = 2.16.
    const EXEC_KAPPA: f64 = 2.4;

    fn empty_grid() -> OccupancyGrid {
        // Production-sized rolling map (40m × 40m): out-of-bounds counts as
        // occupied, so undersized test grids would veto smoothing near edges.
        OccupancyGrid::new(400, 400, 0.1, -20.0, -20.0)
    }

    fn cfg() -> HybridAStarConfig {
        HybridAStarConfig::default()
    }

    /// Dense polyline through the given corner points, stepped at ~5cm like a
    /// raw A* primitive chain (theta/steering left zero — smoothing ignores
    /// them).
    fn chain(corners: &[(f64, f64)]) -> Vec<PathWaypoint> {
        let mut pts = Vec::new();
        for w in corners.windows(2) {
            let (a, b) = (w[0], w[1]);
            let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
            let n = (len / 0.05).ceil().max(1.0) as usize;
            for k in 0..n {
                let t = k as f64 / n as f64;
                pts.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t));
            }
        }
        pts.push(*corners.last().unwrap());
        pts.iter()
            .map(|&(x, y)| PathWaypoint {
                x,
                y,
                theta: 0.0,
                steering: 0.0,
                dir: Default::default(),
            })
            .collect()
    }

    fn length(path: &[PathWaypoint]) -> f64 {
        path.windows(2)
            .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt())
            .sum()
    }

    fn max_curvature(path: &[PathWaypoint]) -> f64 {
        path.windows(3)
            .map(|w| menger_curvature((w[0].x, w[0].y), (w[1].x, w[1].y), (w[2].x, w[2].y)).abs())
            .fold(0.0, f64::max)
    }

    #[test]
    fn shortcut_removes_zigzag_in_open_space() {
        // ±0.3m zigzag over 3m of open space: the smoothed path must collapse
        // to (nearly) the straight line, at uniform spacing.
        let grid = empty_grid();
        let raw = chain(&[
            (0.0, 0.0),
            (0.5, 0.3),
            (1.0, -0.3),
            (1.5, 0.3),
            (2.0, -0.3),
            (2.5, 0.3),
            (3.0, 0.0),
        ]);
        let field = ClearanceField::build(&grid, 0.5);
        let smoothed = smooth_path(&raw, &grid, Some(&field), &cfg(), EXEC_KAPPA, None);
        assert!(length(&raw) > 3.5, "raw zigzag must be meaningfully longer");
        assert!(
            length(&smoothed) < 3.05,
            "zigzag not shortcut: smoothed length {:.2}",
            length(&smoothed)
        );
        assert!((smoothed[0].x, smoothed[0].y) == (0.0, 0.0));
        let last = smoothed.last().unwrap();
        assert!((last.x - 3.0).abs() < 1e-9 && last.y.abs() < 1e-9);
    }

    #[test]
    fn shortcut_never_cuts_into_soft_band_below_original_clearance() {
        // A* detoured around a cone at (2.0, 0.15): the original path keeps
        // full band clearance (≥ 0.5), while the straight start→goal line
        // would pass the cone at ~0.15m — deep inside the soft band. The
        // shortcut must NOT take that trade: the smoothed path's minimum
        // clearance may not drop below the original's.
        let mut grid = empty_grid();
        grid.set_occupied(2.0, 0.15);
        let field = ClearanceField::build(&grid, 0.5);
        let raw = chain(&[(0.0, 0.0), (1.2, -0.6), (2.8, -0.6), (4.0, 0.0)]);
        let orig_min = raw
            .iter()
            .map(|p| field.distance_at(p.x, p.y))
            .fold(f64::INFINITY, f64::min);
        assert!(orig_min >= 0.5 - 1e-9, "detour must start band-clear");

        let smoothed = smooth_path(&raw, &grid, Some(&field), &cfg(), EXEC_KAPPA, None);
        let new_min = smoothed
            .iter()
            .map(|p| field.distance_at(p.x, p.y))
            .fold(f64::INFINITY, f64::min);
        assert!(
            new_min >= orig_min - 1e-6,
            "smoothing cut into the soft band: min clearance {:.3} < original {:.3}",
            new_min,
            orig_min
        );

        // Control: with the cone removed the same call straight-lines it.
        let open = empty_grid();
        let open_field = ClearanceField::build(&open, 0.5);
        let straight = smooth_path(&raw, &open, Some(&open_field), &cfg(), EXEC_KAPPA, None);
        assert!(length(&straight) < 4.05);
    }

    #[test]
    fn gradient_smoothing_respects_curvature_bound_on_right_angle() {
        // Stage B in isolation (shortcutting would trivially straight-line an
        // open-space corner): a hard right angle must smooth to ≤ 2.16 1/m.
        let grid = empty_grid();
        let raw = chain(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]);
        let mut pts: Vec<(f64, f64)> = raw.iter().map(|p| (p.x, p.y)).collect();
        pts = resample(&pts, RESAMPLE_SPACING_M);
        gradient_smooth(&mut pts, &grid, None, &cfg(), 0.9 * EXEC_KAPPA, None);
        let final_pts = resample(&pts, RESAMPLE_SPACING_M);
        let worst = final_pts
            .windows(3)
            .map(|w| menger_curvature(w[0], w[1], w[2]).abs())
            .fold(0.0, f64::max);
        assert!(
            worst <= 0.9 * EXEC_KAPPA + 1e-9,
            "right angle not smoothed under the bound: max curvature {:.2}",
            worst
        );
        assert!(post_check(&final_pts, &grid, 0.9 * EXEC_KAPPA, None));
    }

    #[test]
    fn full_pipeline_l_corridor_stays_bounded_and_collision_free() {
        // Right angle forced by an L-corridor (1.2m wide), so the shortcut
        // cannot bypass the corner: the full pipeline must still deliver a
        // curvature-bounded, hard-inflation-clean path (not the raw
        // fallback, whose corner curvature would exceed the bound).
        let mut grid = empty_grid();
        // Outer walls of the L: x from -0.5..2.6 at y = ±0.6 up the first
        // leg, then y from -0.6..2.6 at x = 1.4 and 2.6 for the second leg.
        let mut s = -0.5;
        while s <= 2.6 {
            grid.set_occupied(s, -0.6); // south wall of leg 1
            if s <= 0.85 {
                grid.set_occupied(s, 0.6); // north wall until the turn opening
            }
            s += 0.05;
        }
        let mut s = -0.6;
        while s <= 2.6 {
            grid.set_occupied(2.6, s); // east wall of leg 2
            if s >= 0.6 {
                grid.set_occupied(1.4, s); // west wall above the opening
            }
            s += 0.05;
        }
        let raw = chain(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]);
        let field = ClearanceField::build(&grid, 0.5);
        let smoothed = smooth_path(&raw, &grid, Some(&field), &cfg(), EXEC_KAPPA, None);
        assert!(
            max_curvature(&smoothed) <= 0.9 * EXEC_KAPPA + 1e-9,
            "corner curvature {:.2} exceeds the bound",
            max_curvature(&smoothed)
        );
        for w in smoothed.windows(2) {
            assert!(
                segment_admissible((w[0].x, w[0].y), (w[1].x, w[1].y), &grid, None, None, None),
                "smoothed path violates hard inflation near ({:.2},{:.2})",
                w[0].x,
                w[0].y
            );
        }
        // It genuinely smoothed (did not fall back to the raw right angle).
        assert!(max_curvature(&raw) > 0.9 * EXEC_KAPPA);
    }

    #[test]
    fn smoothed_spacing_is_uniform() {
        let grid = empty_grid();
        let raw = chain(&[(0.0, 0.0), (1.0, 0.4), (2.0, -0.4), (3.5, 0.0)]);
        let smoothed = smooth_path(&raw, &grid, None, &cfg(), EXEC_KAPPA, None);
        assert!(smoothed.len() >= 3);
        let gaps: Vec<f64> = smoothed
            .windows(2)
            .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt())
            .collect();
        // Interior gaps are exact multiples of the resample spacing; the
        // final (merged) gap may stretch to 1.5× spacing.
        for (i, g) in gaps.iter().enumerate() {
            let (lo, hi) = if i + 1 == gaps.len() {
                (0.5 * RESAMPLE_SPACING_M, 1.5 * RESAMPLE_SPACING_M + 1e-9)
            } else {
                (RESAMPLE_SPACING_M - 1e-6, RESAMPLE_SPACING_M + 1e-6)
            };
            assert!(
                (lo..=hi).contains(g),
                "gap {} of {} is {:.3}m, outside [{:.3}, {:.3}]",
                i,
                gaps.len(),
                g,
                lo,
                hi
            );
        }
    }

    #[test]
    fn disabled_flag_reproduces_raw_output() {
        let grid = empty_grid();
        let raw = chain(&[(0.0, 0.0), (0.5, 0.3), (1.0, -0.3), (1.5, 0.0)]);
        let off = HybridAStarConfig {
            smoothing_enabled: false,
            ..HybridAStarConfig::default()
        };
        let out = smooth_path(&raw, &grid, None, &off, EXEC_KAPPA, None);
        assert_eq!(out.len(), raw.len());
        for (a, b) in out.iter().zip(raw.iter()) {
            assert_eq!(
                (a.x, a.y, a.theta, a.steering),
                (b.x, b.y, b.theta, b.steering)
            );
        }
    }

    #[test]
    fn impossible_curvature_bound_falls_back_to_raw() {
        // With an absurd executor bound (κ ≤ 0.009 → turn radius ≥ 111m) no
        // smoothing of a corner inside a corridor can pass the post-check;
        // the raw A* path must come back unmodified rather than a
        // bound-violating "smoothed" one.
        let mut grid = empty_grid();
        let mut s = -0.5;
        while s <= 2.6 {
            grid.set_occupied(s, -0.4);
            if s <= 1.2 {
                grid.set_occupied(s, 0.4);
            }
            s += 0.05;
        }
        let mut s = -0.4;
        while s <= 2.6 {
            grid.set_occupied(2.4, s);
            if s >= 0.4 {
                grid.set_occupied(1.6, s);
            }
            s += 0.05;
        }
        let raw = chain(&[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]);
        let out = smooth_path(&raw, &grid, None, &cfg(), 0.01, None);
        assert_eq!(out.len(), raw.len(), "must fall back to the raw path");
        assert_eq!((out[5].x, out[5].y), (raw[5].x, raw[5].y));
    }

    #[test]
    fn smoother_never_smooths_across_a_cusp() {
        // (d) A forward zigzag leg, a cusp at exactly (2.0, 0.0), then a
        // reverse zigzag leg: each leg must be smoothed (shortcut collapses
        // the zigzags) but never ACROSS the cusp — the cusp point survives
        // exactly, as the single direction transition.
        let grid = empty_grid();
        let mut raw = chain(&[(0.0, 0.0), (0.5, 0.3), (1.0, -0.3), (1.5, 0.3), (2.0, 0.0)]);
        let mut rev = chain(&[
            (2.0, 0.0),
            (1.7, -0.5),
            (1.3, -0.3),
            (1.0, -1.0),
            (0.8, -1.2),
        ]);
        for w in rev.iter_mut() {
            w.dir = SegmentDir::Reverse;
        }
        raw.extend(rev.into_iter().skip(1));

        let field = ClearanceField::build(&grid, 0.5);
        let sm = smooth_path(&raw, &grid, Some(&field), &cfg(), EXEC_KAPPA, None);

        // Exactly one direction transition, and the waypoint preceding it is
        // the cusp point, preserved EXACTLY.
        let cusps: Vec<usize> = (1..sm.len())
            .filter(|&i| sm[i].dir != sm[i - 1].dir)
            .collect();
        assert_eq!(cusps.len(), 1, "exactly one cusp expected, got {cusps:?}");
        let c = cusps[0];
        assert_eq!(
            (sm[c - 1].x, sm[c - 1].y),
            (2.0, 0.0),
            "cusp point must be preserved exactly"
        );

        // Both legs individually smoothed: total length strictly below the
        // raw zigzag total (shortcutting worked on each side).
        let len = |p: &[PathWaypoint]| -> f64 {
            p.windows(2)
                .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt())
                .sum()
        };
        let fwd_raw: Vec<PathWaypoint> = raw
            .iter()
            .filter(|w| w.dir == SegmentDir::Forward)
            .cloned()
            .collect();
        assert!(
            len(&sm[..c]) < len(&fwd_raw) - 0.3,
            "forward leg not smoothed: {:.2} vs raw {:.2}",
            len(&sm[..c]),
            len(&fwd_raw)
        );
        // Endpoints preserved exactly.
        assert_eq!((sm[0].x, sm[0].y), (0.0, 0.0));
        let last = sm.last().unwrap();
        assert!((last.x - 0.8).abs() < 1e-9 && (last.y - (-1.2)).abs() < 1e-9);

        // Reverse-leg headings oppose the travel direction (robot backs
        // along the curve): cos(heading − travel) < 0 for interior points.
        for w in sm[c..].windows(2) {
            let travel = (w[1].y - w[0].y).atan2(w[1].x - w[0].x);
            let d = (w[0].theta - travel).cos();
            assert!(
                d < 0.0,
                "reverse-leg heading {:.2} does not oppose travel {:.2} at ({:.2},{:.2})",
                w[0].theta,
                travel,
                w[0].x,
                w[0].y
            );
        }
    }

    #[test]
    fn smoothing_budget_fits_the_replan_interval() {
        // The smoother runs inside the 4 Hz (250ms) replan slot after A*.
        // Smooth a representative ~12m slalom chain against a populated grid
        // 20 times and require the mean well under the budget. (Report: see
        // stdout with --nocapture.)
        let mut grid = empty_grid();
        for i in 0..6 {
            let x = 1.5 + 1.8 * i as f64;
            let y = if i % 2 == 0 { 0.45 } else { -0.45 };
            let mut dx = -0.15;
            while dx <= 0.15 {
                let mut dy = -0.15;
                while dy <= 0.15 {
                    grid.set_occupied(x + dx, y + dy);
                    dy += 0.05;
                }
                dx += 0.05;
            }
        }
        let raw = chain(&[
            (0.0, 0.0),
            (1.5, -0.5),
            (3.3, 0.5),
            (5.1, -0.5),
            (6.9, 0.5),
            (8.7, -0.5),
            (10.5, 0.5),
            (12.0, 0.0),
        ]);
        let field = ClearanceField::build(&grid, 0.5);
        let t0 = std::time::Instant::now();
        let mut out_len = 0;
        for _ in 0..20 {
            out_len = smooth_path(&raw, &grid, Some(&field), &cfg(), EXEC_KAPPA, None).len();
        }
        let per_call = t0.elapsed() / 20;
        println!(
            "smoothing: {} raw -> {} smoothed waypoints in {:?} per call",
            raw.len(),
            out_len,
            per_call
        );
        assert!(
            per_call < std::time::Duration::from_millis(20),
            "smoothing took {:?} per call — too close to the 250ms replan budget",
            per_call
        );
    }
}
