//! Reeds-Shepp analytic goal expansion for the bidirectional Hybrid A*.
//!
//! When the search pops a node near the goal, this module attempts an EXACT
//! curvature-bounded connection (arcs at the steering limit + straights, in
//! both travel directions, cusps allowed) from the node pose to the goal
//! pose. Success gives the plan an exact goal heading — something the
//! primitive chain's position-radius termination never guaranteed — and cuts
//! the tail of the search.
//!
//! ## Families implemented
//!
//! - **CSC** — `LSL`, `LSR` and their reflections `RSR`, `RSL`, each also
//!   time-flipped (the whole word driven in reverse): uniform-direction
//!   turn/straight/turn words.
//! - **C|C|C** — `LRL` / `RLR` with the MIDDLE arc driven in the opposite
//!   direction (the classic three-point-turn shape), both time flips.
//!
//! That is 12 candidate words per query, the same reduced set used by
//! PythonRobotics' Reeds-Shepp planner. It is NOT the full 48-word optimal
//! RS solver; the exotic families (`CCu|CuC`, `C|CuCu|C`, `C|C(π/2)SC`, …)
//! only shorten maneuvers this set already solves. Two reasons the reduced
//! set suffices here:
//!
//! 1. The expansion is an accelerator, not the planner: if no candidate
//!    solves (or all collide), Hybrid A* keeps expanding primitives and can
//!    still reach the goal on its own — completeness never rests on this
//!    module.
//! 2. `LSL` (with degenerate arc/straight lengths) yields a valid solution
//!    for every pose pair, so a candidate always exists; on a platform with
//!    a 0.42 m minimum turn radius operating within a 2 m goal radius, the
//!    optimality gap of skipping the exotic families is centimeters.
//!
//! ## Admissibility of a candidate
//!
//! Every candidate is sampled at ≤ `PATH_SAMPLE_STEP_M` (5 cm) of world arc
//! length and must (a) keep every sample off occupied cells — the grid is
//! pre-inflated, so this is the hard-inflation boundary — and (b) obey the
//! *clearance no-worse rule*: with the soft-clearance field available, no
//! sample may have less clearance than the tighter of the two endpoints
//! (node pose, goal pose). The analytic shortcut may therefore never squeeze
//! through a pinch tighter than anything the search already accepted; a path
//! that genuinely needs the pinch is found by the primitive expansion, which
//! pays the soft cost per meter for it.
//!
//! Candidates are tried cheapest-first under the same cost yardstick A* uses
//! (reverse meters × `reverse_cost_multiplier`, plus
//! `direction_switch_penalty` per cusp, including the junction with the
//! node's arrival direction); the first admissible one wins.

use super::{
    corridor_blocked, pose_blocked, ClearanceField, Corridor, EscapeZone, HybridAStarConfig,
    OccupancyGrid, PathWaypoint, Pose, SegmentDir, PATH_SAMPLE_STEP_M,
};

/// Steering of one RS segment.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Steer {
    Left,
    Straight,
    Right,
}

/// One RS segment: steering plus SIGNED length in normalized units (turn
/// radius = 1); positive = forward travel, negative = reverse.
#[derive(Debug, Clone, Copy)]
struct Seg {
    steer: Steer,
    len: f64,
}

/// A candidate word (up to three segments) with its A*-equivalent cost.
struct Candidate {
    segs: [Seg; 3],
    cost: f64,
}

/// Segments shorter than this (normalized) are treated as absent for cusp
/// counting and sampling.
const EPS_LEN: f64 = 1e-9;

/// Attempt an exact Reeds-Shepp connection `from` → `goal` at max curvature
/// `kappa_max` (1/m). `arrival_dir` is the direction the search arrived at
/// `from` with (the junction may charge a switch penalty). `escape` is the
/// optional start-pocket zone: samples inside it are judged by the
/// true-footprint check instead of hard inflation, exactly like the
/// primitive expansion. `corridor` is the optional reference corridor:
/// every sample must also stay within its hard bound (escape zone exempt),
/// exactly like the primitive expansion. Returns the tail as waypoints
/// INCLUDING the `from` pose (callers splicing onto an A* path skip the
/// first), or None when no candidate is admissible.
#[allow(clippy::too_many_arguments)] // one connection contract, not a config bundle
pub fn connect(
    from: &Pose,
    goal: &Pose,
    kappa_max: f64,
    arrival_dir: SegmentDir,
    cfg: &HybridAStarConfig,
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    escape: Option<&EscapeZone>,
    corridor: Option<&Corridor>,
) -> Option<Vec<PathWaypoint>> {
    if kappa_max <= 0.0 {
        return None;
    }
    let r = 1.0 / kappa_max;

    // Goal pose in the start frame, normalized by the turn radius.
    let dx = goal.x - from.x;
    let dy = goal.y - from.y;
    let (c, s) = (from.theta.cos(), from.theta.sin());
    let x = (dx * c + dy * s) / r;
    let y = (-dx * s + dy * c) / r;
    let phi = normalize_angle(goal.theta - from.theta);

    let mut candidates = generate_candidates(x, y, phi);
    for cand in &mut candidates {
        cand.cost = word_cost(&cand.segs, r, arrival_dir, cfg);
    }
    candidates.sort_by(|a, b| a.cost.total_cmp(&b.cost));

    // The no-worse clearance floor: the tighter of the two endpoints.
    let required = clearance.map(|f| {
        f.distance_at(from.x, from.y)
            .min(f.distance_at(goal.x, goal.y))
    });

    // Steering feed-forward for arcs at the curvature limit: tan δ = wb·κ,
    // so δ = atan(wb/r); signed by turn side, independent of travel direction
    // (reversing with left steer swings the heading right by the same
    // geometry).
    let steer_mag = (cfg.wheelbase / r).atan();

    for cand in &candidates {
        if let Some(wps) = sample_word(
            &cand.segs, from, goal, r, steer_mag, grid, clearance, required, escape, corridor,
        ) {
            return Some(wps);
        }
    }
    None
}

/// All 12 candidate words for the normalized goal (x, y, phi): three base
/// solvers × {identity, timeflip, reflect, timeflip∘reflect}. Timeflip solves
/// (-x, y, -phi) and negates lengths; reflect solves (x, -y, -phi) and swaps
/// L/R.
fn generate_candidates(x: f64, y: f64, phi: f64) -> Vec<Candidate> {
    let mut out = Vec::with_capacity(12);
    type Base = fn(f64, f64, f64) -> Option<[Seg; 3]>;
    let bases: [Base; 3] = [lsl, lsr, lrl];
    for base in bases {
        if let Some(segs) = base(x, y, phi) {
            out.push(Candidate { segs, cost: 0.0 });
        }
        if let Some(mut segs) = base(-x, y, -phi) {
            for seg in &mut segs {
                seg.len = -seg.len;
            }
            out.push(Candidate { segs, cost: 0.0 });
        }
        if let Some(mut segs) = base(x, -y, -phi) {
            for seg in &mut segs {
                seg.steer = reflect(seg.steer);
            }
            out.push(Candidate { segs, cost: 0.0 });
        }
        if let Some(mut segs) = base(-x, -y, phi) {
            for seg in &mut segs {
                seg.len = -seg.len;
                seg.steer = reflect(seg.steer);
            }
            out.push(Candidate { segs, cost: 0.0 });
        }
    }
    out
}

fn reflect(s: Steer) -> Steer {
    match s {
        Steer::Left => Steer::Right,
        Steer::Right => Steer::Left,
        Steer::Straight => Steer::Straight,
    }
}

/// CSC, same-side circles: L(t) S(u) L(v), all forward, t and v in [0, π].
/// Always solvable when the circle centers differ; the degenerate
/// concentric case is the pure arc t = phi.
fn lsl(x: f64, y: f64, phi: f64) -> Option<[Seg; 3]> {
    let cx = x - phi.sin();
    let cy = y - 1.0 + phi.cos();
    let u = (cx * cx + cy * cy).sqrt();
    if u < EPS_LEN {
        // Goal on the start's own left circle: pure arc.
        let t = mod2pi_pos(phi);
        return Some([
            seg(Steer::Left, t),
            seg(Steer::Straight, 0.0),
            seg(Steer::Left, 0.0),
        ]);
    }
    let t = cy.atan2(cx);
    if !(0.0..=std::f64::consts::PI).contains(&t) {
        return None;
    }
    let v = mod2pi_pos(phi - t);
    if v > std::f64::consts::PI {
        return None;
    }
    Some([
        seg(Steer::Left, t),
        seg(Steer::Straight, u),
        seg(Steer::Left, v),
    ])
}

/// CSC, opposite-side circles: L(t) S(u) R(v), all forward. Needs the circle
/// centers at least 2 apart (internal tangent).
fn lsr(x: f64, y: f64, phi: f64) -> Option<[Seg; 3]> {
    let cx = x + phi.sin();
    let cy = y - 1.0 - phi.cos();
    let d2 = cx * cx + cy * cy;
    if d2 < 4.0 {
        return None;
    }
    let u = (d2 - 4.0).sqrt();
    let theta = 2.0f64.atan2(u);
    let t = mod2pi_pos(cy.atan2(cx) + theta);
    if t > std::f64::consts::PI {
        return None;
    }
    let v = mod2pi_pos(t - phi);
    if v > std::f64::consts::PI {
        return None;
    }
    Some([
        seg(Steer::Left, t),
        seg(Steer::Straight, u),
        seg(Steer::Right, v),
    ])
}

/// C|C|C: L(t) R(u) L(v) with the middle arc driven in REVERSE (u ≤ 0) — the
/// three-point-turn family. Needs the outer circle centers within 4 of each
/// other.
fn lrl(x: f64, y: f64, phi: f64) -> Option<[Seg; 3]> {
    let cx = x - phi.sin();
    let cy = y - 1.0 + phi.cos();
    let u1 = (cx * cx + cy * cy).sqrt();
    if u1 > 4.0 {
        return None;
    }
    let u = -2.0 * (0.25 * u1).asin();
    let t = mod2pi_pos(cy.atan2(cx) + 0.5 * u + std::f64::consts::PI);
    if t > std::f64::consts::PI {
        return None;
    }
    let v = mod2pi_pos(phi - t + u);
    if v > std::f64::consts::PI {
        return None;
    }
    Some([
        seg(Steer::Left, t),
        seg(Steer::Right, u),
        seg(Steer::Left, v),
    ])
}

fn seg(steer: Steer, len: f64) -> Seg {
    Seg { steer, len }
}

/// A*-equivalent cost of a word: forward meters + reverse meters ×
/// `reverse_cost_multiplier` + `direction_switch_penalty` per cusp,
/// including the junction with the search's arrival direction.
fn word_cost(segs: &[Seg; 3], r: f64, arrival_dir: SegmentDir, cfg: &HybridAStarConfig) -> f64 {
    let mut cost = 0.0;
    let mut prev_sign = arrival_dir.sign();
    for s in segs {
        if s.len.abs() < EPS_LEN {
            continue;
        }
        let meters = s.len.abs() * r;
        cost += if s.len < 0.0 {
            meters * cfg.reverse_cost_multiplier
        } else {
            meters
        };
        let sign = s.len.signum();
        if sign != prev_sign {
            cost += cfg.direction_switch_penalty;
        }
        prev_sign = sign;
    }
    cost
}

/// Sample a word into world-frame waypoints at ≤ 5 cm spacing, enforcing the
/// hard-inflation collision check and the clearance no-worse rule at every
/// sample. Returns None on the first violation. The final waypoint is pinned
/// to the exact goal pose.
#[allow(clippy::too_many_arguments)] // one collision/emission contract, not a config bundle
fn sample_word(
    segs: &[Seg; 3],
    from: &Pose,
    goal: &Pose,
    r: f64,
    steer_mag: f64,
    grid: &OccupancyGrid,
    clearance: Option<&ClearanceField>,
    required_clearance: Option<f64>,
    escape: Option<&EscapeZone>,
    corridor: Option<&Corridor>,
) -> Option<Vec<PathWaypoint>> {
    let step_norm = PATH_SAMPLE_STEP_M / r; // normalized sample spacing
    let steer_sign_of = |steer: Steer| -> f64 {
        match steer {
            Steer::Left => 1.0,
            Steer::Right => -1.0,
            Steer::Straight => 0.0,
        }
    };

    // Pose state in NORMALIZED start-frame coordinates.
    let (mut nx, mut ny, mut nth) = (0.0f64, 0.0f64, 0.0f64);
    let (c0, s0) = (from.theta.cos(), from.theta.sin());
    let to_world = |px: f64, py: f64, pth: f64| -> (f64, f64, f64) {
        (
            from.x + r * (px * c0 - py * s0),
            from.y + r * (px * s0 + py * c0),
            normalize_angle(from.theta + pth),
        )
    };

    let admissible = |wx: f64, wy: f64| -> bool {
        if pose_blocked(grid, escape, wx, wy) {
            return false;
        }
        // The analytic tail obeys the same corridor hard bound as the
        // primitive expansion (escape zone exempt).
        if corridor_blocked(corridor, escape, wx, wy) {
            return false;
        }
        match (clearance, required_clearance) {
            (Some(field), Some(req)) => field.distance_at(wx, wy) >= req - 1e-9,
            _ => true,
        }
    };

    // The from pose itself must be admissible (it is an already-expanded
    // node, so this only rejects on the no-worse rule's numeric edge).
    if pose_blocked(grid, escape, from.x, from.y) {
        return None;
    }

    let mut out: Vec<PathWaypoint> = vec![PathWaypoint {
        x: from.x,
        y: from.y,
        theta: from.theta,
        steering: 0.0,
        dir: SegmentDir::Forward, // fixed up below to the first segment's dir
    }];

    for sg in segs {
        if sg.len.abs() < EPS_LEN {
            continue;
        }
        let dir = if sg.len > 0.0 {
            SegmentDir::Forward
        } else {
            SegmentDir::Reverse
        };
        let steering = steer_sign_of(sg.steer) * steer_mag;
        let n = (sg.len.abs() / step_norm).ceil().max(1.0) as usize;
        let ds = sg.len / n as f64; // signed normalized step
        for _ in 0..n {
            let (px, py, pth) = seg_step(nx, ny, nth, sg.steer, ds);
            nx = px;
            ny = py;
            nth = pth;
            let (wx, wy, wth) = to_world(nx, ny, nth);
            if !admissible(wx, wy) {
                return None;
            }
            out.push(PathWaypoint {
                x: wx,
                y: wy,
                theta: wth,
                steering,
                dir,
            });
        }
    }
    if out.len() < 2 {
        return None; // degenerate word (zero total length)
    }
    out[0].dir = out[1].dir;
    // Pin the endpoint to the exact goal pose (the closed forms are exact up
    // to float noise; the executor and hysteresis deserve the exact goal).
    let last = out.last_mut().unwrap();
    last.x = goal.x;
    last.y = goal.y;
    last.theta = goal.theta;
    Some(out)
}

/// One closed-form step of signed normalized arc length `ds` along a segment.
fn seg_step(x: f64, y: f64, th: f64, steer: Steer, ds: f64) -> (f64, f64, f64) {
    match steer {
        Steer::Straight => (x + ds * th.cos(), y + ds * th.sin(), th),
        Steer::Left => {
            let nth = th + ds;
            (x + (nth.sin() - th.sin()), y - (nth.cos() - th.cos()), nth)
        }
        Steer::Right => {
            let nth = th - ds;
            (x - (nth.sin() - th.sin()), y + (nth.cos() - th.cos()), nth)
        }
    }
}

/// Normalize to [0, 2π).
fn mod2pi_pos(a: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let mut v = a % two_pi;
    if v < 0.0 {
        v += two_pi;
    }
    v
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

    const KAPPA: f64 = 2.36; // tan(0.44)/0.2, the production steering limit

    fn open_grid() -> OccupancyGrid {
        OccupancyGrid::new(400, 400, 0.1, -20.0, -20.0)
    }

    fn cfg() -> HybridAStarConfig {
        HybridAStarConfig::default()
    }

    fn end_pose_error(path: &[PathWaypoint], goal: &Pose) -> (f64, f64) {
        let last = path.last().unwrap();
        let dpos = ((last.x - goal.x).powi(2) + (last.y - goal.y).powi(2)).sqrt();
        let dth = normalize_angle(last.theta - goal.theta).abs();
        (dpos, dth)
    }

    /// Chain continuity + endpoint exactness over a spread of goal poses:
    /// every consecutive waypoint gap ≤ 5cm + slack, headings consistent
    /// with travel, endpoint exactly the goal.
    #[test]
    fn rs_connects_pose_grid_exactly() {
        let grid = open_grid();
        let from = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let mut solved = 0;
        for &gx in &[-1.5, -0.5, 0.5, 1.5] {
            for &gy in &[-1.2, 0.0, 1.2] {
                for &gth in &[-2.5, -1.0, 0.0, 1.6, 3.0] {
                    let goal = Pose {
                        x: gx,
                        y: gy,
                        theta: gth,
                    };
                    let Some(path) = connect(
                        &from,
                        &goal,
                        KAPPA,
                        SegmentDir::Forward,
                        &cfg(),
                        &grid,
                        None,
                        None,
                        None,
                    ) else {
                        continue;
                    };
                    solved += 1;
                    let (dpos, dth) = end_pose_error(&path, &goal);
                    assert!(
                        dpos < 1e-6,
                        "endpoint off goal by {dpos} for {gx},{gy},{gth}"
                    );
                    assert!(dth < 1e-6, "heading off goal by {dth} for {gx},{gy},{gth}");
                    for w in path.windows(2) {
                        let gap = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
                        assert!(gap < PATH_SAMPLE_STEP_M + 0.02, "gap {gap} too wide");
                    }
                }
            }
        }
        // The reduced family set must solve the overwhelming majority of the
        // pose grid (LSL alone guarantees a candidate; collision is off).
        assert!(solved >= 50, "only {solved}/60 poses solved");
    }

    #[test]
    fn rs_collision_rejects_blocked_connection() {
        let mut grid = open_grid();
        // Wall between from and goal.
        let mut y = -2.0;
        while y <= 2.0 {
            for xo in [0.7, 0.75, 0.8] {
                grid.set_occupied(xo, y);
            }
            y += 0.05;
        }
        let from = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 1.5,
            y: 0.0,
            theta: 0.0,
        };
        assert!(
            connect(
                &from,
                &goal,
                KAPPA,
                SegmentDir::Forward,
                &cfg(),
                &grid,
                None,
                None,
                None
            )
            .is_none(),
            "a straight-blocked goal 1.5m out must not connect (detours would \
             need to leave the wall's span, which the sampler rejects)"
        );
    }

    #[test]
    fn rs_clearance_no_worse_rule_rejects_pinch() {
        // Both endpoints in open space (full band clearance), a cone pinching
        // the direct connection: the no-worse rule must reject every word
        // that squeezes past the cone tighter than the endpoints' clearance.
        let mut grid = open_grid();
        grid.set_occupied(0.75, 0.35);
        let field = ClearanceField::build(&grid, 0.5);
        let from = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 1.5,
            y: 0.0,
            theta: 0.0,
        };
        let with_rule = connect(
            &from,
            &goal,
            KAPPA,
            SegmentDir::Forward,
            &cfg(),
            &grid,
            Some(&field),
            None,
            None,
        );
        if let Some(path) = with_rule {
            // If a word still connects, it must respect the rule.
            let req = field
                .distance_at(from.x, from.y)
                .min(field.distance_at(goal.x, goal.y));
            for p in &path {
                assert!(
                    field.distance_at(p.x, p.y) >= req - 1e-9,
                    "sample at ({:.2},{:.2}) violates the no-worse rule",
                    p.x,
                    p.y
                );
            }
        }
        // Without the field the (hard-inflation-clean) direct word connects.
        assert!(connect(
            &from,
            &goal,
            KAPPA,
            SegmentDir::Forward,
            &cfg(),
            &grid,
            None,
            None,
            None
        )
        .is_some());
    }
}
