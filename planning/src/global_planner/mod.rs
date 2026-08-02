use std::cmp::Ordering;
/// Hybrid A* global planner for Ackermann vehicles.
///
/// Plans in (x, y, theta) state space using kinematically feasible
/// motion primitives (bicycle model). Produces paths the Limo Pro
/// can actually follow without post-processing.
///
/// Runs at 1Hz — triggered by behavior planner when replanning is needed.
use std::collections::BinaryHeap;

use serde::Deserialize;
use tracing::info;

pub mod reeds_shepp;
pub mod smoother;

#[derive(Debug, Clone, Deserialize)]
pub struct HybridAStarConfig {
    #[serde(default = "default_xy_resolution")]
    pub xy_resolution: f64, // meters per grid cell
    #[serde(default = "default_theta_resolution")]
    pub theta_resolution: f64, // radians per heading bin
    #[serde(default = "default_wheelbase")]
    pub wheelbase: f64, // meters
    #[serde(default = "default_max_steering")]
    pub max_steering_angle: f64, // radians
    #[serde(default = "default_step_size")]
    pub step_size: f64, // meters per expansion step
    #[serde(default = "default_num_steer")]
    pub num_steer_angles: usize, // number of discrete steering samples
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,
    /// Soft-clearance cost weight (dimensionless, per meter of travel at the
    /// hard obstacle edge). Traversal through the decay band beyond occupied
    /// cells adds `weight * step_size * (1 - d/clearance_decay_m)` to the
    /// step cost, so hugging the hard edge for 1m costs `weight` extra
    /// equivalent path-meters — at 2.0 a detour up to ~2m longer per meter of
    /// edge-hugging still wins, which centers paths in gaps whenever the free
    /// width allows. The penalty is always finite: a narrow-but-necessary
    /// passage stays traversable, just more expensive. 0.0 disables the layer
    /// (pure distance + steering cost, the old behavior).
    #[serde(default = "default_clearance_cost_weight")]
    pub clearance_cost_weight: f64,
    /// Width (m) of the soft-cost decay band beyond occupied cells: the
    /// proximity penalty is `clearance_cost_weight` at the hard edge and
    /// decays linearly to 0 at this distance.
    #[serde(default = "default_clearance_decay_m")]
    pub clearance_decay_m: f64,
    /// Post-process every successful A* plan through the smoother (shortcut +
    /// curvature-bounded gradient smoothing + uniform resampling). `false`
    /// publishes the raw motion-primitive chain (the old behavior).
    #[serde(default = "default_smoothing_enabled")]
    pub smoothing_enabled: bool,
    /// Laplacian smoothing gain per iteration: x_i += α·(x_{i-1} + x_{i+1} −
    /// 2·x_i). Must stay ≤ 0.5 for the Jacobi update to be stable.
    #[serde(default = "default_smoothing_alpha")]
    pub smoothing_alpha: f64,
    /// Clearance-penalty gain per iteration: points inside the soft band are
    /// pushed along the clearance gradient with strength β·(1 − d/decay).
    #[serde(default = "default_smoothing_clearance_beta")]
    pub smoothing_clearance_beta: f64,
    /// Iteration cap for the gradient-smoothing stage (the curvature
    /// relaxation pass gets the same budget again).
    #[serde(default = "default_smoothing_iterations")]
    pub smoothing_iterations: usize,
    /// Path hysteresis: a valid current path is only replaced by a new plan
    /// whose cost (smoothed length + soft clearance cost) is at least this
    /// fraction better. Kills topology flapping between 4 Hz replans on a
    /// noisy obstacle snapshot.
    #[serde(default = "default_path_improvement_threshold")]
    pub path_improvement_threshold: f64,
    /// Bidirectional maneuver planning: expand motion primitives in reverse
    /// as well as forward. `false` restores the old forward-only search (a
    /// cornered robot then has no planned escape and relies on scripted
    /// recovery reverses).
    #[serde(default = "default_reverse_enabled")]
    pub reverse_enabled: bool,
    /// Cost of switching travel direction (equivalent path-meters). Charged
    /// once per cusp, so gratuitous forward/reverse shuttling loses to a
    /// clean single-direction path unless the cusp genuinely buys geometry.
    #[serde(default = "default_direction_switch_penalty")]
    pub direction_switch_penalty: f64,
    /// Reverse travel costs this multiple of forward travel per meter
    /// (blind direction, tracker runs mirrored). Must be >= 1.
    #[serde(default = "default_reverse_cost_multiplier")]
    pub reverse_cost_multiplier: f64,
    /// Attempt an exact Reeds-Shepp analytic connection to the goal whenever
    /// an expanded node is within `rs_expansion_radius` of it.
    #[serde(default = "default_rs_expansion_enabled")]
    pub rs_expansion_enabled: bool,
    /// Goal distance (m) below which Reeds-Shepp goal expansion is attempted.
    #[serde(default = "default_rs_expansion_radius")]
    pub rs_expansion_radius: f64,
    /// Start-pocket escape: when the start pose itself lies INSIDE the grid's
    /// hard inflation (robot wedged into an obstacle's inflated zone), the
    /// collision check within this radius (m) of the start is relaxed to the
    /// TRUE-FOOTPRINT check (obstacle radius + physical footprint + a small
    /// safety pad) so the bidirectional planner can plan the trivial escape
    /// ("reverse 0.4m, turn, proceed") instead of dying on an occupied start
    /// cell. Beyond the radius the standard hard check applies unchanged, and
    /// the relaxation never activates when the start would fail even the
    /// true-footprint check (physical overlap). 0 disables the relaxation.
    #[serde(default = "default_start_escape_radius")]
    pub start_escape_radius: f64,
}

fn default_xy_resolution() -> f64 {
    0.1
}
fn default_theta_resolution() -> f64 {
    0.1745
} // ~10 degrees
fn default_wheelbase() -> f64 {
    0.2
}
fn default_max_steering() -> f64 {
    0.48
}
fn default_step_size() -> f64 {
    0.2
}
fn default_num_steer() -> usize {
    5
}
fn default_max_iterations() -> usize {
    // Sized for the BIDIRECTIONAL search: arrival direction doubles the
    // closed-set state space, and adversarial forced detours measured
    // ~2.8-3.6x the forward-only expansion count (wall-detour benchmark:
    // 39k forward-only vs 112k bidirectional+RS / 142k without RS). 300k
    // keeps those solvable with headroom; typical cluttered-corridor plans
    // terminate in a few thousand pops, so the cap only bites when no path
    // exists.
    300_000
}
fn default_clearance_cost_weight() -> f64 {
    2.0
}
fn default_clearance_decay_m() -> f64 {
    0.5
}
fn default_smoothing_enabled() -> bool {
    true
}
fn default_smoothing_alpha() -> f64 {
    0.3
}
fn default_smoothing_clearance_beta() -> f64 {
    0.2
}
fn default_smoothing_iterations() -> usize {
    50
}
fn default_path_improvement_threshold() -> f64 {
    0.15
}
fn default_reverse_enabled() -> bool {
    true
}
fn default_direction_switch_penalty() -> f64 {
    0.6
}
fn default_reverse_cost_multiplier() -> f64 {
    2.0
}
fn default_rs_expansion_enabled() -> bool {
    true
}
fn default_rs_expansion_radius() -> f64 {
    2.0
}
fn default_start_escape_radius() -> f64 {
    0.6
}

impl Default for HybridAStarConfig {
    fn default() -> Self {
        Self {
            xy_resolution: default_xy_resolution(),
            theta_resolution: default_theta_resolution(),
            wheelbase: default_wheelbase(),
            max_steering_angle: default_max_steering(),
            step_size: default_step_size(),
            num_steer_angles: default_num_steer(),
            max_iterations: default_max_iterations(),
            clearance_cost_weight: default_clearance_cost_weight(),
            clearance_decay_m: default_clearance_decay_m(),
            smoothing_enabled: default_smoothing_enabled(),
            smoothing_alpha: default_smoothing_alpha(),
            smoothing_clearance_beta: default_smoothing_clearance_beta(),
            smoothing_iterations: default_smoothing_iterations(),
            path_improvement_threshold: default_path_improvement_threshold(),
            reverse_enabled: default_reverse_enabled(),
            direction_switch_penalty: default_direction_switch_penalty(),
            reverse_cost_multiplier: default_reverse_cost_multiplier(),
            rs_expansion_enabled: default_rs_expansion_enabled(),
            rs_expansion_radius: default_rs_expansion_radius(),
            start_escape_radius: default_start_escape_radius(),
        }
    }
}

impl HybridAStarConfig {
    /// A negative or non-finite clearance weight/decay is a YAML typo that
    /// would silently invert the soft-cost layer (negative weight REWARDS
    /// edge-hugging). Fail loudly at startup.
    pub fn validate(&self) -> Result<(), String> {
        if !self.clearance_cost_weight.is_finite() || self.clearance_cost_weight < 0.0 {
            return Err(format!(
                "global_planner.clearance_cost_weight must be finite and >= 0 (got {})",
                self.clearance_cost_weight
            ));
        }
        if !self.clearance_decay_m.is_finite() || self.clearance_decay_m < 0.0 {
            return Err(format!(
                "global_planner.clearance_decay_m must be finite and >= 0 (got {})",
                self.clearance_decay_m
            ));
        }
        // α > 0.5 makes the Jacobi Laplacian update oscillate/diverge; a
        // negative α inverts smoothing into roughening. Either is a typo.
        if !self.smoothing_alpha.is_finite()
            || self.smoothing_alpha < 0.0
            || self.smoothing_alpha > 0.5
        {
            return Err(format!(
                "global_planner.smoothing_alpha must be in [0, 0.5] (got {})",
                self.smoothing_alpha
            ));
        }
        // A negative β would PULL the smoothed path toward obstacles.
        if !self.smoothing_clearance_beta.is_finite() || self.smoothing_clearance_beta < 0.0 {
            return Err(format!(
                "global_planner.smoothing_clearance_beta must be finite and >= 0 (got {})",
                self.smoothing_clearance_beta
            ));
        }
        if self.smoothing_enabled && self.smoothing_iterations == 0 {
            return Err(
                "global_planner.smoothing_iterations must be >= 1 when smoothing is enabled"
                    .to_string(),
            );
        }
        // A negative switch penalty REWARDS shuttling; a reverse multiplier
        // below 1 makes reverse travel cheaper than forward. Both are typos.
        if !self.direction_switch_penalty.is_finite() || self.direction_switch_penalty < 0.0 {
            return Err(format!(
                "global_planner.direction_switch_penalty must be finite and >= 0 (got {})",
                self.direction_switch_penalty
            ));
        }
        if !self.reverse_cost_multiplier.is_finite() || self.reverse_cost_multiplier < 1.0 {
            return Err(format!(
                "global_planner.reverse_cost_multiplier must be finite and >= 1 (got {})",
                self.reverse_cost_multiplier
            ));
        }
        if !self.rs_expansion_radius.is_finite() || self.rs_expansion_radius < 0.0 {
            return Err(format!(
                "global_planner.rs_expansion_radius must be finite and >= 0 (got {})",
                self.rs_expansion_radius
            ));
        }
        // A negative/NaN escape radius poisons the wedged-start containment
        // test (every point would fall outside the zone — or inside, for
        // NaN's unordered comparisons — nondeterministically).
        if !self.start_escape_radius.is_finite() || self.start_escape_radius < 0.0 {
            return Err(format!(
                "global_planner.start_escape_radius must be finite and >= 0 (got {})",
                self.start_escape_radius
            ));
        }
        // >= 1.0 means "never replace a valid path" — a frozen stale path is
        // never intended; < 0 would replace on WORSE candidates.
        if !self.path_improvement_threshold.is_finite()
            || self.path_improvement_threshold < 0.0
            || self.path_improvement_threshold >= 1.0
        {
            return Err(format!(
                "global_planner.path_improvement_threshold must be in [0, 1) (got {})",
                self.path_improvement_threshold
            ));
        }
        Ok(())
    }
}

/// A pose in the planning space.
#[derive(Debug, Clone)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// Travel direction of a path segment. Attached to each waypoint as the
/// direction the robot travels to ARRIVE at it; a change between consecutive
/// waypoints marks a cusp (direction-switch point) the executor must stop at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SegmentDir {
    #[default]
    Forward,
    Reverse,
}

impl SegmentDir {
    /// +1.0 forward, -1.0 reverse — sign of the commanded linear velocity.
    pub fn sign(self) -> f64 {
        match self {
            SegmentDir::Forward => 1.0,
            SegmentDir::Reverse => -1.0,
        }
    }
}

/// A waypoint in the planned path.
// `steering` is part of the path contract consumed by the tracker for feed-forward.
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub struct PathWaypoint {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub steering: f64, // steering angle used to reach this point
    /// Direction of travel on the segment arriving at this waypoint. The
    /// first waypoint of a path carries the direction of its FIRST segment
    /// (it has no arrival of its own), so consecutive-waypoint comparison
    /// finds every cusp and only cusps.
    pub dir: SegmentDir,
}

/// Simple occupancy grid for collision checking.
pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub resolution: f64, // meters per cell
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<u8>, // 0 = free, 100 = occupied
}

impl OccupancyGrid {
    pub fn new(width: usize, height: usize, resolution: f64, origin_x: f64, origin_y: f64) -> Self {
        Self {
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            data: vec![0; width * height],
        }
    }

    /// Check if a world position is occupied or out of bounds.
    pub fn is_occupied(&self, x: f64, y: f64) -> bool {
        let gx = ((x - self.origin_x) / self.resolution) as isize;
        let gy = ((y - self.origin_y) / self.resolution) as isize;

        if gx < 0 || gy < 0 || gx >= self.width as isize || gy >= self.height as isize {
            return true; // out of bounds = occupied
        }

        self.data[gy as usize * self.width + gx as usize] >= 50
    }

    pub fn set_occupied(&mut self, x: f64, y: f64) {
        let gx = ((x - self.origin_x) / self.resolution) as isize;
        let gy = ((y - self.origin_y) / self.resolution) as isize;

        if gx >= 0 && gy >= 0 && (gx as usize) < self.width && (gy as usize) < self.height {
            self.data[gy as usize * self.width + gx as usize] = 100;
        }
    }

    /// World position falls inside the grid extent. Out-of-bounds positions
    /// count as occupied everywhere in this module — the escape relaxation
    /// must never treat them as free either.
    pub fn in_bounds(&self, x: f64, y: f64) -> bool {
        let gx = ((x - self.origin_x) / self.resolution) as isize;
        let gy = ((y - self.origin_y) / self.resolution) as isize;
        gx >= 0 && gy >= 0 && gx < self.width as isize && gy < self.height as isize
    }
}

// --- Start-pocket escape (wedged-start relaxation) ---

/// Physical obstacle disc (world frame, actual extent — NOT inflated) for the
/// true-footprint escape check. The occupancy grid only carries the inflated
/// cells, so the wedged-start relaxation needs the raw detections the grid
/// was built from.
#[derive(Debug, Clone, Copy)]
pub struct PhysicalObstacle {
    pub x: f64,
    pub y: f64,
    pub radius: f64,
}

/// Safety pad (m) added to the physical footprint in the true-footprint
/// escape check.
pub const TRUE_FOOTPRINT_SAFETY_M: f64 = 0.03;

/// Required center distance beyond an obstacle's own physical radius under
/// the true-footprint check: the robot's circumscribed footprint plus the
/// safety pad. Deliberately BELOW the hard-inflation margin
/// (`dwa.robot_radius`) — the difference is the escapable pocket band the
/// robot may plan through but never past.
pub const TRUE_FOOTPRINT_CLEARANCE_M: f64 =
    crate::local_planner::dwa::ROBOT_FOOTPRINT_RADIUS + TRUE_FOOTPRINT_SAFETY_M;

/// Start-pocket escape zone: within `radius` of the wedged start pose, the
/// hard-inflation grid check is replaced by the true-footprint check against
/// the physical obstacles (`pose_blocked`). Built by `start_escape_zone`
/// only when the start cell is occupied AND the true footprint is not — the
/// robot may plan through its own inflation pocket but never through an
/// actual physical overlap.
#[derive(Debug, Clone, Copy)]
pub struct EscapeZone<'a> {
    cx: f64,
    cy: f64,
    radius: f64,
    obstacles: &'a [PhysicalObstacle],
}

impl EscapeZone<'_> {
    /// (x, y) lies inside the relaxation zone around the wedged start.
    pub fn contains(&self, x: f64, y: f64) -> bool {
        (x - self.cx).powi(2) + (y - self.cy).powi(2) <= self.radius * self.radius
    }

    /// True-footprint collision: within (obstacle radius +
    /// `TRUE_FOOTPRINT_CLEARANCE_M`) of any physical obstacle.
    fn footprint_blocked(&self, x: f64, y: f64) -> bool {
        self.obstacles.iter().any(|o| {
            ((x - o.x).powi(2) + (y - o.y).powi(2)).sqrt() < o.radius + TRUE_FOOTPRINT_CLEARANCE_M
        })
    }
}

/// Collision check with the optional start-pocket relaxation: inside the
/// escape zone the true-footprint check replaces hard inflation (out-of-grid
/// stays blocked); everywhere else the standard `is_occupied` applies
/// unchanged. The soft clearance cost is NOT affected — escape legs still pay
/// the full proximity penalty, so they prefer the least-bad direction.
pub fn pose_blocked(grid: &OccupancyGrid, escape: Option<&EscapeZone>, x: f64, y: f64) -> bool {
    if let Some(zone) = escape {
        if zone.contains(x, y) {
            return !grid.in_bounds(x, y) || zone.footprint_blocked(x, y);
        }
    }
    grid.is_occupied(x, y)
}

/// Minimum interval between "escape mode" INFO logs (the zone is re-detected
/// every 10Hz cycle while the robot sits in the pocket).
const ESCAPE_LOG_PERIOD: std::time::Duration = std::time::Duration::from_secs(2);
static ESCAPE_LOG_AT: std::sync::Mutex<Option<std::time::Instant>> = std::sync::Mutex::new(None);

/// Detect the wedged-start condition and build the escape zone.
///
/// Returns Some exactly when the start cell is occupied under the standard
/// hard-inflation check but the TRUE footprint at the start is collision-free
/// (the robot sits in an inflation pocket, not in physical overlap). On
/// physical overlap — which should not happen — returns None, preserving the
/// pre-escape failure behavior. Logs at INFO (rate-limited) when escape mode
/// activates.
pub fn start_escape_zone<'a>(
    grid: &OccupancyGrid,
    obstacles: &'a [PhysicalObstacle],
    x: f64,
    y: f64,
    radius: f64,
) -> Option<EscapeZone<'a>> {
    if radius <= 0.0 || !grid.is_occupied(x, y) {
        return None;
    }
    let zone = EscapeZone {
        cx: x,
        cy: y,
        radius,
        obstacles,
    };
    if zone.footprint_blocked(x, y) {
        // Physical overlap: the relaxation must never plan from inside an
        // actual obstacle — behave exactly as before the escape existed.
        return None;
    }
    if let Ok(mut last) = ESCAPE_LOG_AT.lock() {
        if last.is_none_or(|t| t.elapsed() >= ESCAPE_LOG_PERIOD) {
            info!("A* start inside inflation — escape mode (r={:.2}m)", radius);
            *last = Some(std::time::Instant::now());
        }
    }
    Some(zone)
}

/// Distance-to-nearest-occupied field over an occupancy grid, capped at a
/// maximum of interest. Built once per `plan()` call (1Hz, 400×400 grid: two
/// chamfer passes over 160k cells, well under a millisecond) and queried per
/// expanded node for the soft-clearance cost.
///
/// Implementation: two-pass chamfer distance transform (8-neighborhood,
/// axial cost = resolution, diagonal cost = resolution·√2). Slightly
/// overestimates true Euclidean distance (≤ ~8%), which only makes the soft
/// band marginally narrower — irrelevant next to the 0.1m cell quantization.
/// Out-of-grid queries return the cap (no penalty): out-of-bounds positions
/// are already hard-blocked by `OccupancyGrid::is_occupied`, and grid borders
/// are not obstacles.
pub struct ClearanceField {
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    cap: f32,
    dist: Vec<f32>,
}

impl ClearanceField {
    pub fn build(grid: &OccupancyGrid, cap_m: f64) -> Self {
        let n = grid.width * grid.height;
        let cap = cap_m as f32;
        let mut dist = vec![f32::INFINITY; n];
        let mut any_occupied = false;
        for (i, &cell) in grid.data.iter().enumerate() {
            if cell >= 50 {
                dist[i] = 0.0;
                any_occupied = true;
            }
        }
        if !any_occupied {
            dist.fill(cap);
        } else {
            let res = grid.resolution as f32;
            let diag = res * std::f32::consts::SQRT_2;
            let (w, h) = (grid.width, grid.height);
            // Forward pass: propagate from the top-left.
            for y in 0..h {
                for x in 0..w {
                    let i = y * w + x;
                    let mut d = dist[i];
                    if x > 0 {
                        d = d.min(dist[i - 1] + res);
                    }
                    if y > 0 {
                        d = d.min(dist[i - w] + res);
                        if x > 0 {
                            d = d.min(dist[i - w - 1] + diag);
                        }
                        if x + 1 < w {
                            d = d.min(dist[i - w + 1] + diag);
                        }
                    }
                    dist[i] = d;
                }
            }
            // Backward pass: propagate from the bottom-right.
            for y in (0..h).rev() {
                for x in (0..w).rev() {
                    let i = y * w + x;
                    let mut d = dist[i];
                    if x + 1 < w {
                        d = d.min(dist[i + 1] + res);
                    }
                    if y + 1 < h {
                        d = d.min(dist[i + w] + res);
                        if x + 1 < w {
                            d = d.min(dist[i + w + 1] + diag);
                        }
                        if x > 0 {
                            d = d.min(dist[i + w - 1] + diag);
                        }
                    }
                    dist[i] = d.min(cap);
                }
            }
            // The backward pass capped as it went; sweep the forward-pass
            // residue (cells the backward pass never lowered).
            for d in &mut dist {
                *d = d.min(cap);
            }
        }
        Self {
            width: grid.width,
            height: grid.height,
            resolution: grid.resolution,
            origin_x: grid.origin_x,
            origin_y: grid.origin_y,
            cap,
            dist,
        }
    }

    /// Distance (m) from the cell containing (x, y) to the nearest occupied
    /// cell, capped at the build cap. Out of bounds → cap (no soft penalty).
    pub fn distance_at(&self, x: f64, y: f64) -> f64 {
        let gx = ((x - self.origin_x) / self.resolution) as isize;
        let gy = ((y - self.origin_y) / self.resolution) as isize;
        if gx < 0 || gy < 0 || gx >= self.width as isize || gy >= self.height as isize {
            return self.cap as f64;
        }
        self.dist[gy as usize * self.width + gx as usize] as f64
    }
}

// --- Internal types for the search ---

#[derive(Clone)]
struct Node {
    x: f64,
    y: f64,
    theta: f64,
    g_cost: f64,
    f_cost: f64,
    steering: f64,
    /// Direction of the primitive that arrived at this node.
    dir: SegmentDir,
    parent_idx: Option<usize>,
}

struct NodeWrapper {
    f_cost: f64,
    idx: usize,
}

impl PartialEq for NodeWrapper {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost
    }
}

impl Eq for NodeWrapper {}

impl PartialOrd for NodeWrapper {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for NodeWrapper {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap: reverse ordering
        other
            .f_cost
            .partial_cmp(&self.f_cost)
            .unwrap_or(Ordering::Equal)
    }
}

pub struct HybridAStar {
    config: HybridAStarConfig,
    steer_angles: Vec<f64>,
}

impl HybridAStar {
    pub fn new(config: HybridAStarConfig) -> Self {
        let n = config.num_steer_angles;
        let steer_angles: Vec<f64> = (0..n)
            .map(|i| {
                if n == 1 {
                    0.0
                } else {
                    -config.max_steering_angle
                        + 2.0 * config.max_steering_angle * (i as f64) / (n as f64 - 1.0)
                }
            })
            .collect();

        Self {
            config,
            steer_angles,
        }
    }

    /// Build the soft-clearance field for `grid` per this planner's config,
    /// or None when the clearance layer is disabled. Exposed so one chamfer
    /// build per replan can be shared between planning, path smoothing, and
    /// path-cost evaluation (hysteresis).
    pub fn build_clearance(&self, grid: &OccupancyGrid) -> Option<ClearanceField> {
        let use_clearance =
            self.config.clearance_cost_weight > 0.0 && self.config.clearance_decay_m > 0.0;
        use_clearance.then(|| ClearanceField::build(grid, self.config.clearance_decay_m))
    }

    /// Plan a path from start to goal on the given occupancy grid.
    /// Returns None if no path is found.
    // Convenience entry building the clearance field itself; the main loop
    // uses `plan_with_clearance` to share one build. Exercised by unit tests.
    #[allow(dead_code)]
    pub fn plan(
        &self,
        start: &Pose,
        goal: &Pose,
        grid: &OccupancyGrid,
    ) -> Option<Vec<PathWaypoint>> {
        let clearance = self.build_clearance(grid);
        self.plan_with_clearance(start, goal, grid, clearance.as_ref())
    }

    /// `plan` with a caller-supplied (prebuilt) clearance field, so the 4 Hz
    /// replan path builds the chamfer transform exactly once per cycle.
    pub fn plan_with_clearance(
        &self,
        start: &Pose,
        goal: &Pose,
        grid: &OccupancyGrid,
        clearance: Option<&ClearanceField>,
    ) -> Option<Vec<PathWaypoint>> {
        self.plan_with_stats(start, goal, grid, clearance).0
    }

    /// `plan_with_clearance` additionally reporting search statistics
    /// (expansion counts, Reeds-Shepp attempts) for budget instrumentation.
    pub fn plan_with_stats(
        &self,
        start: &Pose,
        goal: &Pose,
        grid: &OccupancyGrid,
        clearance: Option<&ClearanceField>,
    ) -> (Option<Vec<PathWaypoint>>, PlanStats) {
        self.plan_with_escape(start, goal, grid, clearance, None)
    }

    /// Full planning entry with the physical obstacle set: detects the
    /// wedged-start condition itself (`start_escape_zone`) and plans with the
    /// relaxation when it applies. Convenience over `plan_with_escape` for
    /// callers that do not need to share the zone.
    #[allow(dead_code)] // planner-level entry exercised by unit tests
    pub fn plan_with_obstacles(
        &self,
        start: &Pose,
        goal: &Pose,
        grid: &OccupancyGrid,
        physical: &[PhysicalObstacle],
    ) -> Option<Vec<PathWaypoint>> {
        let clearance = self.build_clearance(grid);
        let zone = start_escape_zone(
            grid,
            physical,
            start.x,
            start.y,
            self.config.start_escape_radius,
        );
        self.plan_with_escape(start, goal, grid, clearance.as_ref(), zone.as_ref())
            .0
    }

    /// `plan_with_stats` with an optional start-pocket escape zone: within
    /// the zone the hard-inflation collision check is replaced by the
    /// true-footprint check (see `pose_blocked`), applied identically to
    /// forward and reverse primitives and to Reeds-Shepp expansion samples.
    /// With `escape: None` the search is byte-for-byte the standard one.
    pub fn plan_with_escape(
        &self,
        start: &Pose,
        goal: &Pose,
        grid: &OccupancyGrid,
        clearance: Option<&ClearanceField>,
        escape: Option<&EscapeZone>,
    ) -> (Option<Vec<PathWaypoint>>, PlanStats) {
        let mut stats = PlanStats::default();
        let mut open = BinaryHeap::new();
        let mut nodes: Vec<Node> = Vec::new();
        let mut closed = std::collections::HashSet::new();

        // Reverse primitives share the steering set and step size with the
        // forward ones. Reverse travel is charged `reverse_cost_multiplier`
        // per meter and a direction change costs `direction_switch_penalty`
        // once — both additive-only, so the Euclidean heuristic stays
        // admissible.
        let directions: &[SegmentDir] = if self.config.reverse_enabled {
            &[SegmentDir::Forward, SegmentDir::Reverse]
        } else {
            &[SegmentDir::Forward]
        };
        let kappa_max = self.config.max_steering_angle.tan() / self.config.wheelbase;
        let rs_active = self.config.rs_expansion_enabled && self.config.rs_expansion_radius > 0.0;
        // Throttle for analytic goal expansion: attempting RS on every popped
        // node inside the radius would dominate the cycle with collision
        // sampling; every RS_ATTEMPT_STRIDE-th qualifying pop keeps the
        // accelerator cheap while still firing within a handful of pops.
        let mut rs_countdown = 0usize;

        // Soft-clearance layer: precomputed distance-to-occupied field, so a
        // path grazing the hard collision boundary costs more than one
        // centered in the gap. Purely additive and finite — feasibility is
        // unchanged (a tight gate stays reachable, just dispreferred), and
        // the heuristic remains admissible (costs only increase). The START
        // pose is never charged: only successor steps pay proximity, so
        // starting wedged against a cone adds the same bounded offset to
        // every outgoing path and cannot poison the search.

        // The start node "arrives" forward by convention: a robot at rest can
        // start either way, but charging the switch penalty for an initial
        // reverse biases toward forward starts, which is the conservative
        // default (reverse is the blind direction).
        let start_node = Node {
            x: start.x,
            y: start.y,
            theta: normalize_angle(start.theta),
            g_cost: 0.0,
            f_cost: heuristic(start.x, start.y, start.theta, goal),
            steering: 0.0,
            dir: SegmentDir::Forward,
            parent_idx: None,
        };

        nodes.push(start_node);
        open.push(NodeWrapper {
            f_cost: nodes[0].f_cost,
            idx: 0,
        });

        let mut iterations = 0;

        while let Some(current) = open.pop() {
            iterations += 1;
            stats.iterations = iterations;
            if iterations > self.config.max_iterations {
                return (None, stats); // timeout
            }

            let node = &nodes[current.idx];
            let nx = node.x;
            let ny = node.y;
            let ntheta = node.theta;
            let ng_cost = node.g_cost;
            let ndir = node.dir;

            // Check if goal reached
            let dx = nx - goal.x;
            let dy = ny - goal.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist < self.config.xy_resolution * 2.0 {
                return (Some(self.reconstruct_path(&nodes, current.idx)), stats);
            }

            // Discretize for closed set. The ARRIVAL DIRECTION is part of the
            // key: two nodes at the same (x, y, θ) cell reached forward vs
            // reverse are NOT interchangeable — continuing in the arrival
            // direction is switch-free for one and costs the switch penalty
            // for the other, so closing the cell on the first arrival alone
            // could prune exactly the switch state a cusp maneuver needs
            // (e.g. arriving reversed at the spot where the forward leg must
            // begin). Worst case this doubles the closed set; measured on the
            // gauntlet-scale grid the growth is far smaller because most
            // cells are only ever reached in one direction.
            let key = self.discretize(nx, ny, ntheta, ndir);
            if closed.contains(&key) {
                continue;
            }
            closed.insert(key);

            // Reeds-Shepp analytic goal expansion: near the goal, try the
            // exact curvature-bounded connection (both directions, cusps
            // allowed) from this node pose, collision-checked at <= 5cm.
            // First success appends the RS tail and terminates the search —
            // marginally suboptimal (a later node might have offered a
            // cheaper tail) but standard Hybrid A* practice: the RS tail is
            // exact in heading, which the primitive chain's coarse
            // termination radius never was.
            if rs_active && dist < self.config.rs_expansion_radius {
                if rs_countdown == 0 {
                    rs_countdown = RS_ATTEMPT_STRIDE;
                    stats.rs_attempts += 1;
                    let from = Pose {
                        x: nx,
                        y: ny,
                        theta: ntheta,
                    };
                    if let Some(tail) = reeds_shepp::connect(
                        &from,
                        goal,
                        kappa_max,
                        ndir,
                        &self.config,
                        grid,
                        clearance,
                        escape,
                    ) {
                        stats.rs_connected = true;
                        let mut path = self.reconstruct_path(&nodes, current.idx);
                        // Drop the duplicated junction pose (tail starts at
                        // the node pose).
                        path.extend(tail.into_iter().skip(1));
                        return (Some(path), stats);
                    }
                } else {
                    rs_countdown -= 1;
                }
            }

            // Expand with each steering angle in each enabled direction.
            for &dir in directions {
                for &steer in &self.steer_angles {
                    let (new_x, new_y, new_theta) = self.bicycle_step(nx, ny, ntheta, steer, dir);

                    // Collision check: hard inflation, relaxed to the true
                    // footprint inside the start-pocket escape zone — the
                    // same predicate for forward and reverse primitives.
                    if pose_blocked(grid, escape, new_x, new_y) {
                        continue;
                    }

                    let new_key = self.discretize(new_x, new_y, new_theta, dir);
                    if closed.contains(&new_key) {
                        continue;
                    }

                    // Proximity penalty: linear decay from `clearance_cost_weight`
                    // at the hard edge to 0 at `clearance_decay_m`, scaled by
                    // step_size so it composes with the distance term as a
                    // per-meter cost density. Applied identically in both
                    // directions — a reverse graze is no safer than a forward
                    // one.
                    let proximity_cost = clearance.map_or(0.0, |c| {
                        let d = c.distance_at(new_x, new_y);
                        let decay = self.config.clearance_decay_m;
                        if d < decay {
                            self.config.clearance_cost_weight
                                * self.config.step_size
                                * (1.0 - d / decay)
                        } else {
                            0.0
                        }
                    });
                    let travel_cost = match dir {
                        SegmentDir::Forward => self.config.step_size,
                        SegmentDir::Reverse => {
                            self.config.step_size * self.config.reverse_cost_multiplier
                        }
                    };
                    let switch_cost = if dir != ndir {
                        self.config.direction_switch_penalty
                    } else {
                        0.0
                    };
                    let step_cost = travel_cost + 0.5 * steer.abs() + proximity_cost + switch_cost;
                    let new_g = ng_cost + step_cost;
                    let new_f = new_g + heuristic(new_x, new_y, new_theta, goal);

                    let new_idx = nodes.len();
                    nodes.push(Node {
                        x: new_x,
                        y: new_y,
                        theta: normalize_angle(new_theta),
                        g_cost: new_g,
                        f_cost: new_f,
                        steering: steer,
                        dir,
                        parent_idx: Some(current.idx),
                    });

                    open.push(NodeWrapper {
                        f_cost: new_f,
                        idx: new_idx,
                    });
                }
            }
        }

        (None, stats) // no path found
    }

    /// Bicycle model step in the given direction: the displacement flips sign
    /// while the steering geometry stays the same, so reversing with left
    /// steer swings the heading right — the true Ackermann reverse.
    fn bicycle_step(
        &self,
        x: f64,
        y: f64,
        theta: f64,
        steer: f64,
        dir: SegmentDir,
    ) -> (f64, f64, f64) {
        let d = self.config.step_size * dir.sign();
        let wb = self.config.wheelbase;

        // Proper bicycle model: compute position using current theta,
        // then update theta. Use midpoint integration for better accuracy.
        let dtheta = (d / wb) * steer.tan();
        let mid_theta = theta + dtheta * 0.5;
        let new_x = x + d * mid_theta.cos();
        let new_y = y + d * mid_theta.sin();
        let new_theta = theta + dtheta;

        (new_x, new_y, normalize_angle(new_theta))
    }

    /// Discretize (x, y, theta, arrival direction) into a closed-set key.
    /// See the closed-set comment in `plan_with_stats` for why the arrival
    /// direction must be part of it.
    fn discretize(&self, x: f64, y: f64, theta: f64, dir: SegmentDir) -> (i32, i32, i32, u8) {
        let gx = (x / self.config.xy_resolution).round() as i32;
        let gy = (y / self.config.xy_resolution).round() as i32;
        let gt = (normalize_angle(theta) / self.config.theta_resolution).round() as i32;
        (gx, gy, gt, dir as u8)
    }

    /// Reconstruct path by walking parent pointers.
    fn reconstruct_path(&self, nodes: &[Node], goal_idx: usize) -> Vec<PathWaypoint> {
        let mut path = Vec::new();
        let mut idx = Some(goal_idx);

        while let Some(i) = idx {
            let node = &nodes[i];
            path.push(PathWaypoint {
                x: node.x,
                y: node.y,
                theta: node.theta,
                steering: node.steering,
                dir: node.dir,
            });
            idx = node.parent_idx;
        }

        path.reverse();
        // The start waypoint has no arrival of its own; give it the first
        // segment's direction so consecutive-dir comparison marks cusps only
        // where the direction actually changes.
        if path.len() > 1 {
            path[0].dir = path[1].dir;
        }
        path
    }
}

/// Search statistics from one `plan_with_stats` call.
#[derive(Debug, Clone, Copy, Default)]
pub struct PlanStats {
    /// Nodes popped from the open list (expansion count).
    pub iterations: usize,
    /// Reeds-Shepp analytic connections attempted.
    pub rs_attempts: usize,
    /// The returned path ends in an RS analytic tail.
    pub rs_connected: bool,
}

/// Attempt the Reeds-Shepp goal expansion on every N-th qualifying node pop.
const RS_ATTEMPT_STRIDE: usize = 5;

/// Heuristic: Euclidean distance + heading penalty.
fn heuristic(x: f64, y: f64, theta: f64, goal: &Pose) -> f64 {
    let dx = x - goal.x;
    let dy = y - goal.y;
    let dist = (dx * dx + dy * dy).sqrt();
    let dtheta = normalize_angle(theta - goal.theta).abs();
    dist + 0.3 * dtheta
}

/// Sample step (m) along path segments for the per-cycle validity
/// re-verification and the shortcut/post-check collision sampling.
pub const PATH_SAMPLE_STEP_M: f64 = 0.05;

/// True while a (remaining) path is still executable against the LATEST grid
/// at hard inflation: every segment, sampled at ≤ `PATH_SAMPLE_STEP_M` steps
/// (endpoints included), stays off occupied cells. The grid cells are already
/// inflated by robot radius + obstacle extent when populated, so occupancy IS
/// the hard-inflation boundary. Empty paths are not valid (nothing to keep).
// The main loop always calls the escape-aware variant; this strict wrapper is
// the pre-escape contract, kept for tests and future strict-check callers.
#[allow(dead_code)]
pub fn path_remains_valid(path: &[PathWaypoint], grid: &OccupancyGrid) -> bool {
    path_remains_valid_with_escape(path, grid, None)
}

/// `path_remains_valid` under the start-pocket relaxation: samples inside
/// the escape zone (built around the CURRENT robot pose when it is itself in
/// collision) are judged by the true-footprint check instead of hard
/// inflation. Without this, a freshly planned escape leg through the robot's
/// own inflation pocket would be invalidated on the very next 10Hz cycle.
pub fn path_remains_valid_with_escape(
    path: &[PathWaypoint],
    grid: &OccupancyGrid,
    escape: Option<&EscapeZone>,
) -> bool {
    match path {
        [] => false,
        [only] => !pose_blocked(grid, escape, only.x, only.y),
        _ => path
            .windows(2)
            .all(|w| segment_free(grid, escape, (w[0].x, w[0].y), (w[1].x, w[1].y))),
    }
}

/// Every ≤5cm sample of the segment a→b (endpoints included) passes the
/// (possibly escape-relaxed) collision check.
fn segment_free(
    grid: &OccupancyGrid,
    escape: Option<&EscapeZone>,
    a: (f64, f64),
    b: (f64, f64),
) -> bool {
    let len = ((b.0 - a.0).powi(2) + (b.1 - a.1).powi(2)).sqrt();
    let n = (len / PATH_SAMPLE_STEP_M).ceil().max(1.0) as usize;
    (0..=n).all(|k| {
        let t = k as f64 / n as f64;
        !pose_blocked(grid, escape, a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
    })
}

/// Index of the waypoint nearest to (x, y) — the truncation point when the
/// retained path is committed to across replans. 0 for an empty path.
pub fn nearest_waypoint_index(path: &[PathWaypoint], x: f64, y: f64) -> usize {
    let mut best = 0;
    let mut best_d2 = f64::INFINITY;
    for (i, wp) in path.iter().enumerate() {
        let d2 = (wp.x - x).powi(2) + (wp.y - y).powi(2);
        if d2 < best_d2 {
            best_d2 = d2;
            best = i;
        }
    }
    best
}

/// Direction-penalty knobs for `path_cost`, mirroring what A* charges during
/// the search so the hysteresis comparison judges maneuver paths by the same
/// yardstick that produced them.
#[derive(Debug, Clone, Copy)]
pub struct CostPenalties {
    /// Per-meter multiplier on reverse-segment length (>= 1).
    pub reverse_cost_multiplier: f64,
    /// Flat cost per direction switch (cusp).
    pub direction_switch_penalty: f64,
}

impl CostPenalties {
    pub fn from_config(cfg: &HybridAStarConfig) -> Self {
        Self {
            reverse_cost_multiplier: cfg.reverse_cost_multiplier,
            direction_switch_penalty: cfg.direction_switch_penalty,
        }
    }

    /// Neutral penalties (pure length + clearance — the pre-maneuver cost).
    #[allow(dead_code)] // test convenience
    pub fn none() -> Self {
        Self {
            reverse_cost_multiplier: 1.0,
            direction_switch_penalty: 0.0,
        }
    }
}

/// Hysteresis cost of a path: length (reverse meters weighted by
/// `reverse_cost_multiplier`) plus `direction_switch_penalty` per cusp plus
/// the soft clearance cost along it — the same per-meter penalty density A*
/// charges (`weight · (1 − d/decay)` inside the band), evaluated at segment
/// midpoints. Candidate and retained path are scored with the SAME field, so
/// the comparison is fair, and a shuttling maneuver path never beats a clean
/// pure-forward one of equal geometric length.
pub fn path_cost(
    path: &[PathWaypoint],
    clearance: Option<&ClearanceField>,
    weight: f64,
    decay_m: f64,
    penalties: CostPenalties,
) -> f64 {
    let mut cost = 0.0;
    for w in path.windows(2) {
        let ds = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
        cost += if w[1].dir == SegmentDir::Reverse {
            ds * penalties.reverse_cost_multiplier
        } else {
            ds
        };
        if w[1].dir != w[0].dir {
            cost += penalties.direction_switch_penalty;
        }
        if let Some(field) = clearance {
            if weight > 0.0 && decay_m > 0.0 {
                let d = field.distance_at((w[0].x + w[1].x) * 0.5, (w[0].y + w[1].y) * 0.5);
                if d < decay_m {
                    cost += weight * ds * (1.0 - d / decay_m);
                }
            }
        }
    }
    cost
}

fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn empty_grid() -> OccupancyGrid {
        OccupancyGrid::new(100, 100, 0.1, -5.0, -5.0) // 10m x 10m
    }

    #[test]
    fn test_plan_straight_line() {
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 2.0,
            y: 0.0,
            theta: 0.0,
        };

        let path = planner.plan(&start, &goal, &empty_grid());
        assert!(path.is_some(), "Should find a straight-line path");

        let path = path.unwrap();
        assert!(path.len() > 2, "Path should have multiple waypoints");

        // First point near start
        assert!((path[0].x).abs() < 0.2);
        // Last point near goal
        let last = path.last().unwrap();
        assert!(
            (last.x - 2.0).abs() < 0.5,
            "Last waypoint should be near goal"
        );
    }

    #[test]
    fn test_plan_with_obstacle() {
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };

        let mut grid = empty_grid();
        // Place a thick wall at x=1.5, spanning y=-2.0 to y=2.0
        // Use multiple x columns for thickness so the planner can't step through
        for x_offset in &[1.4, 1.45, 1.5, 1.55, 1.6] {
            for y_step in -40..40 {
                grid.set_occupied(*x_offset, y_step as f64 * 0.05);
            }
        }
        // Leave a gap at y > 2.0 for the planner to go around

        let path = planner.plan(&start, &goal, &grid);
        assert!(path.is_some(), "Should find a path around the wall");

        let path = path.unwrap();
        // Path should deviate in y to go around the wall
        let max_y = path.iter().map(|p| p.y.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_y > 0.5,
            "Path should deviate significantly to avoid wall"
        );
    }

    #[test]
    fn test_no_path_blocked() {
        let planner = HybridAStar::new(HybridAStarConfig {
            max_iterations: 5000, // limit search
            ..Default::default()
        });
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };

        let mut grid = empty_grid();
        // Fill everything around the start as occupied, leaving only a tiny pocket
        // The robot has no way out — every direction is blocked
        for gx in 0..100 {
            for gy in 0..100 {
                let wx = -5.0 + gx as f64 * 0.1;
                let wy = -5.0 + gy as f64 * 0.1;
                let dist = (wx * wx + wy * wy).sqrt();
                // Block everything outside 0.15m radius (less than one step)
                if dist > 0.15 {
                    grid.set_occupied(wx, wy);
                }
            }
        }

        let path = planner.plan(&start, &goal, &grid);
        assert!(
            path.is_none(),
            "Should not find a path when completely blocked"
        );
    }

    /// Corridor along +x with walls at y = ±1.0 and a cone (hard extent
    /// ~0.15m of occupied cells) at (2.0, 0.3), offset from the centerline:
    /// the free gap on the -y side (~1.15m) is much wider than on the +y side
    /// (~0.55m). Start (0,0,0), goal (4,0,0).
    fn corridor_with_offset_cone() -> OccupancyGrid {
        let mut grid = empty_grid();
        let mut x = -0.5;
        while x <= 4.5 {
            for wall_y in &[1.0, 1.05, -1.0, -1.05] {
                grid.set_occupied(x, *wall_y);
            }
            x += 0.05;
        }
        // Cone blob: all cells within 0.15m of (2.0, 0.3).
        let mut cx: f64 = -0.2;
        while cx <= 0.2 {
            let mut cy: f64 = -0.2;
            while cy <= 0.2 {
                if (cx * cx + cy * cy).sqrt() <= 0.15 {
                    grid.set_occupied(2.0 + cx, 0.3 + cy);
                }
                cy += 0.05;
            }
            cx += 0.05;
        }
        grid
    }

    /// Minimum distance from any path waypoint to the cone center (2.0, 0.3).
    fn min_dist_to_cone(path: &[PathWaypoint]) -> f64 {
        path.iter()
            .map(|p| ((p.x - 2.0).powi(2) + (p.y - 0.3).powi(2)).sqrt())
            .fold(f64::INFINITY, f64::min)
    }

    #[test]
    fn test_clearance_centers_path_on_wide_side_of_cone() {
        // With the soft-clearance layer on (default weight 2.0, decay 0.5),
        // the path must swing to the WIDE (-y) side of the offset cone and
        // keep at least the hard extent plus ~half the decay band of
        // clearance, instead of riding the cone's collision boundary.
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 4.0,
            y: 0.0,
            theta: 0.0,
        };
        let path = planner
            .plan(&start, &goal, &corridor_with_offset_cone())
            .expect("corridor with offset cone must be traversable");

        let min_dist = min_dist_to_cone(&path);
        assert!(
            min_dist >= 0.15 + 0.2,
            "path hugs the cone: min distance to cone center {:.3} < hard extent + half decay",
            min_dist
        );
        // Wide side: while abreast of the cone the path must sit below the
        // cone center, not squeeze through the narrow +y gap.
        for p in path.iter().filter(|p| p.x > 1.6 && p.x < 2.4) {
            assert!(
                p.y < 0.3,
                "path passed the cone on the narrow side at ({:.2}, {:.2})",
                p.x,
                p.y
            );
        }
    }

    #[test]
    fn test_clearance_off_reproduces_edge_hugging() {
        // Control experiment: weight 0.0 restores the old distance+steering
        // cost, whose optimum grazes the cone's collision boundary. This pins
        // both backward compatibility and that the clearance term (not
        // something else) produces the centering above.
        let planner = HybridAStar::new(HybridAStarConfig {
            clearance_cost_weight: 0.0,
            ..Default::default()
        });
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 4.0,
            y: 0.0,
            theta: 0.0,
        };
        let path = planner
            .plan(&start, &goal, &corridor_with_offset_cone())
            .expect("corridor must be traversable with the cost layer off");
        assert!(
            min_dist_to_cone(&path) < 0.15 + 0.2,
            "with weight 0 the shortest path should pass closer than the soft band"
        );
    }

    #[test]
    fn test_clearance_narrow_gate_stays_traversable_and_centered() {
        // A 1.1m gate between two walls: the soft cost is finite, so the gate
        // must remain traversable, and symmetric penalties from both jambs
        // must center the crossing.
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let mut grid = empty_grid();
        for wall_x in &[1.9, 1.95, 2.0, 2.05, 2.1] {
            let mut y: f64 = -4.9;
            while y <= 4.9 {
                if y.abs() >= 0.55 {
                    grid.set_occupied(*wall_x, y);
                }
                y += 0.05;
            }
        }
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 4.0,
            y: 0.0,
            theta: 0.0,
        };
        let path = planner
            .plan(&start, &goal, &grid)
            .expect("1.1m gate must remain traversable with the clearance layer on");
        let crossing: Vec<&PathWaypoint> = path.iter().filter(|p| p.x > 1.7 && p.x < 2.3).collect();
        assert!(!crossing.is_empty(), "path must actually cross the gate");
        for p in crossing {
            assert!(
                p.y.abs() < 0.25,
                "gate crossing off-center at ({:.2}, {:.2})",
                p.x,
                p.y
            );
        }
    }

    #[test]
    fn test_clearance_field_distances() {
        let mut grid = OccupancyGrid::new(40, 40, 0.1, -2.0, -2.0);
        grid.set_occupied(0.0, 0.0);
        let field = ClearanceField::build(&grid, 0.5);
        assert_eq!(field.distance_at(0.0, 0.0), 0.0);
        // One cell away along an axis: exactly one resolution. Query points
        // sit safely inside their cells (grid indexing truncates, and e.g.
        // (-0.1 + 2.0)/0.1 rounds DOWN a cell in f64).
        assert!((field.distance_at(0.1, 0.0) - 0.1).abs() < 1e-6);
        assert!((field.distance_at(0.02, -0.06) - 0.1).abs() < 1e-6);
        // Chamfer diagonal: resolution * sqrt(2).
        assert!((field.distance_at(0.1, 0.1) - 0.1 * std::f64::consts::SQRT_2).abs() < 1e-6);
        // Far away and out of bounds: capped, no penalty.
        assert_eq!(field.distance_at(1.5, 1.5), 0.5);
        assert_eq!(field.distance_at(10.0, 10.0), 0.5);
        // Empty grid: everything at the cap.
        let empty = OccupancyGrid::new(10, 10, 0.1, 0.0, 0.0);
        let field = ClearanceField::build(&empty, 0.5);
        assert_eq!(field.distance_at(0.5, 0.5), 0.5);
    }

    #[test]
    fn test_clearance_config_validation() {
        assert!(HybridAStarConfig::default().validate().is_ok());
        assert!(HybridAStarConfig {
            clearance_cost_weight: -1.0,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(HybridAStarConfig {
            clearance_decay_m: f64::NAN,
            ..Default::default()
        }
        .validate()
        .is_err());
        // Zero weight (layer off) is a legitimate configuration.
        assert!(HybridAStarConfig {
            clearance_cost_weight: 0.0,
            ..Default::default()
        }
        .validate()
        .is_ok());
        // Smoothing/hysteresis nonsense fails loudly at startup.
        assert!(HybridAStarConfig {
            smoothing_alpha: 0.6, // Jacobi-unstable
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(HybridAStarConfig {
            smoothing_clearance_beta: -0.1, // pulls INTO obstacles
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(HybridAStarConfig {
            smoothing_iterations: 0,
            ..Default::default()
        }
        .validate()
        .is_err());
        // ...but 0 iterations with smoothing off is fine.
        assert!(HybridAStarConfig {
            smoothing_enabled: false,
            smoothing_iterations: 0,
            ..Default::default()
        }
        .validate()
        .is_ok());
        assert!(HybridAStarConfig {
            path_improvement_threshold: 1.0, // never replace = frozen path
            ..Default::default()
        }
        .validate()
        .is_err());
        // Maneuver-planning knobs: a negative switch penalty REWARDS
        // shuttling, a sub-1 multiplier makes reverse cheaper than forward,
        // a NaN radius poisons the RS gate.
        assert!(HybridAStarConfig {
            direction_switch_penalty: -0.1,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(HybridAStarConfig {
            reverse_cost_multiplier: 0.5,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(HybridAStarConfig {
            rs_expansion_radius: f64::NAN,
            ..Default::default()
        }
        .validate()
        .is_err());
        // A negative or NaN escape radius poisons the wedged-start zone.
        assert!(HybridAStarConfig {
            start_escape_radius: -0.1,
            ..Default::default()
        }
        .validate()
        .is_err());
        assert!(HybridAStarConfig {
            start_escape_radius: f64::NAN,
            ..Default::default()
        }
        .validate()
        .is_err());
        // Zero (escape disabled) is a legitimate configuration.
        assert!(HybridAStarConfig {
            start_escape_radius: 0.0,
            ..Default::default()
        }
        .validate()
        .is_ok());
        // Zero switch penalty and multiplier exactly 1 are legitimate.
        assert!(HybridAStarConfig {
            direction_switch_penalty: 0.0,
            reverse_cost_multiplier: 1.0,
            ..Default::default()
        }
        .validate()
        .is_ok());
        assert!(HybridAStarConfig {
            path_improvement_threshold: -0.1, // replace on WORSE candidates
            ..Default::default()
        }
        .validate()
        .is_err());
    }

    fn wp(x: f64, y: f64) -> PathWaypoint {
        PathWaypoint {
            x,
            y,
            theta: 0.0,
            steering: 0.0,
            dir: Default::default(),
        }
    }

    #[test]
    fn test_path_validity_reverification() {
        let mut grid = empty_grid();
        let path = vec![wp(0.0, 0.0), wp(1.0, 0.0), wp(2.0, 0.0)];
        assert!(path_remains_valid(&path, &grid));

        // A newly-perceived obstacle BETWEEN waypoints (mid-segment) must
        // invalidate the path — this is what the ≤5cm sub-stepping is for.
        grid.set_occupied(1.5, 0.0);
        assert!(!path_remains_valid(&path, &grid));

        // An obstacle off the corridor does not.
        let mut grid2 = empty_grid();
        grid2.set_occupied(1.5, 1.0);
        assert!(path_remains_valid(&path, &grid2));

        // Empty path: nothing to keep.
        assert!(!path_remains_valid(&[], &grid2));
    }

    #[test]
    fn test_nearest_waypoint_index() {
        let path = vec![wp(0.0, 0.0), wp(1.0, 0.0), wp(2.0, 0.0)];
        assert_eq!(nearest_waypoint_index(&path, -1.0, 0.0), 0);
        assert_eq!(nearest_waypoint_index(&path, 1.1, 0.3), 1);
        assert_eq!(nearest_waypoint_index(&path, 5.0, 5.0), 2);
        assert_eq!(nearest_waypoint_index(&[], 0.0, 0.0), 0);
    }

    #[test]
    fn test_path_cost_length_plus_clearance_penalty() {
        // Open space: cost is pure length.
        let empty = empty_grid();
        let field = ClearanceField::build(&empty, 0.5);
        let path = vec![wp(0.0, 0.0), wp(2.0, 0.0)];
        let no_pen = CostPenalties::none();
        assert!((path_cost(&path, Some(&field), 3.0, 0.5, no_pen) - 2.0).abs() < 1e-9);
        assert!((path_cost(&path, None, 3.0, 0.5, no_pen) - 2.0).abs() < 1e-9);

        // The same path grazing an obstacle costs strictly more, so the
        // hysteresis comparison prefers the better-cleared corridor.
        let mut grid = empty_grid();
        grid.set_occupied(1.0, 0.2);
        let field = ClearanceField::build(&grid, 0.5);
        let hugging = path_cost(&path, Some(&field), 3.0, 0.5, no_pen);
        assert!(hugging > 2.0);
        let clear = vec![wp(0.0, -1.0), wp(2.0, -1.0)];
        assert!(path_cost(&clear, Some(&field), 3.0, 0.5, no_pen) < hugging);
    }

    #[test]
    fn test_bicycle_step_straight() {
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let (nx, ny, nt) = planner.bicycle_step(0.0, 0.0, 0.0, 0.0, SegmentDir::Forward);
        let step = planner.config.step_size; // 0.2
        assert!((nx - step).abs() < 1e-6); // step_size forward
        assert!(ny.abs() < 1e-6);
        assert!(nt.abs() < 1e-6);
    }
}

#[cfg(test)]
mod maneuver_tests {
    use super::*;

    fn pose(x: f64, y: f64, theta: f64) -> Pose {
        Pose { x, y, theta }
    }

    /// Cornered start: robot facing a wall 0.3m ahead inside a pocket whose
    /// side walls (y = ±0.35, x ∈ [-0.5, 0.4]) leave no lateral room to arc
    /// away forward at the 0.38m minimum turn radius; the only open space is
    /// BEHIND. Goal ahead-left beyond the frontal wall.
    fn cornered_pocket() -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(100, 100, 0.1, -5.0, -5.0);
        // Frontal wall x ∈ [0.3, 0.4], y ∈ [-1.2, 1.2].
        for xo in [0.3, 0.35, 0.4] {
            let mut y = -1.2;
            while y <= 1.2 {
                grid.set_occupied(xo, y);
                y += 0.05;
            }
        }
        // Pocket side walls y = ±0.35, x ∈ [-0.5, 0.4].
        let mut x = -0.5;
        while x <= 0.4 {
            for yo in [0.35, 0.4, -0.35, -0.4] {
                grid.set_occupied(x, yo);
            }
            x += 0.05;
        }
        grid
    }

    #[test]
    fn cornered_start_plans_reverse_then_forward() {
        // (a) Bidirectional: the plan must first back out of the pocket
        // (reverse segment), then drive forward around the wall to the goal.
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = pose(0.0, 0.0, 0.0);
        let goal = pose(2.0, 1.8, 0.0);
        let path = planner
            .plan(&start, &goal, &cornered_pocket())
            .expect("bidirectional search must escape the pocket");

        let first_rev = path.iter().position(|w| w.dir == SegmentDir::Reverse);
        let first_rev = first_rev.expect("plan must contain a reverse segment");
        assert!(
            path[first_rev..]
                .iter()
                .any(|w| w.dir == SegmentDir::Forward),
            "plan must continue forward after the reverse escape"
        );
        let last = path.last().unwrap();
        assert!(
            ((last.x - goal.x).powi(2) + (last.y - goal.y).powi(2)).sqrt() < 0.5,
            "path must end near the goal"
        );
    }

    #[test]
    fn cornered_start_forward_only_control_fails() {
        // (a) control: with reverse disabled the pocket has no exit — the
        // forward-only frontier exhausts and the planner must return None.
        let planner = HybridAStar::new(HybridAStarConfig {
            reverse_enabled: false,
            ..Default::default()
        });
        let start = pose(0.0, 0.0, 0.0);
        let goal = pose(2.0, 1.8, 0.0);
        assert!(
            planner.plan(&start, &goal, &cornered_pocket()).is_none(),
            "forward-only search must fail from the cornered pose"
        );
    }

    #[test]
    fn open_corridor_stays_pure_forward() {
        // (b) The switch penalty and reverse multiplier keep a simple open
        // corridor plan pure forward — no gratuitous shuttling.
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let mut grid = OccupancyGrid::new(100, 100, 0.1, -5.0, -5.0);
        let mut x = -0.5;
        while x <= 4.5 {
            for yo in [1.0, 1.05, -1.0, -1.05] {
                grid.set_occupied(x, yo);
            }
            x += 0.05;
        }
        let path = planner
            .plan(&pose(0.0, 0.0, 0.0), &pose(4.0, 0.0, 0.0), &grid)
            .expect("open corridor must be plannable");
        assert!(
            path.iter().all(|w| w.dir == SegmentDir::Forward),
            "corridor plan must not contain reverse segments"
        );
    }

    #[test]
    fn rs_expansion_connects_heading_flip() {
        // (c) Goal 1.5m away with a >90° heading change: the Reeds-Shepp
        // expansion must produce the direct analytic connection with the
        // goal heading matched within 0.1 rad and curvature within the
        // steering limit.
        let cfg = HybridAStarConfig::default();
        let kappa_max = cfg.max_steering_angle.tan() / cfg.wheelbase; // 2.60
        let planner = HybridAStar::new(cfg);
        let grid = OccupancyGrid::new(200, 200, 0.1, -10.0, -10.0);
        let start = pose(0.0, 0.0, 0.0);
        let goal = pose(1.2, 0.9, 2.0); // 1.5m away, Δθ = 2.0 rad
        let (path, stats) = planner.plan_with_stats(&start, &goal, &grid, None);
        let path = path.expect("open-space RS connection must exist");
        assert!(stats.rs_connected, "plan must end in an RS analytic tail");

        let last = path.last().unwrap();
        assert!(
            ((last.x - goal.x).powi(2) + (last.y - goal.y).powi(2)).sqrt() < 1e-6,
            "RS tail must end exactly at the goal position"
        );
        let dth = normalize_angle(last.theta - goal.theta).abs();
        assert!(dth < 0.1, "goal heading missed by {dth:.3} rad");

        // Discrete curvature within the steering limit, per same-direction
        // subchain (a 3-point window across a cusp measures the reversal,
        // not a driven arc).
        for w in path.windows(3) {
            if w[1].dir != w[0].dir || w[2].dir != w[1].dir {
                continue;
            }
            let ab = ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt();
            let bc = ((w[2].x - w[1].x).powi(2) + (w[2].y - w[1].y).powi(2)).sqrt();
            let ac = ((w[2].x - w[0].x).powi(2) + (w[2].y - w[0].y).powi(2)).sqrt();
            let cross =
                (w[1].x - w[0].x) * (w[2].y - w[1].y) - (w[1].y - w[0].y) * (w[2].x - w[1].x);
            let denom = ab * bc * ac;
            if denom < 1e-12 {
                continue;
            }
            let kappa = (2.0 * cross / denom).abs();
            assert!(
                kappa <= kappa_max * 1.05 + 1e-9,
                "path curvature {kappa:.2} exceeds the steering limit {kappa_max:.2}"
            );
        }
    }

    fn wp_dir(x: f64, y: f64, dir: SegmentDir) -> PathWaypoint {
        PathWaypoint {
            x,
            y,
            theta: 0.0,
            steering: 0.0,
            dir,
        }
    }

    #[test]
    fn path_cost_charges_reverse_and_switch_penalties() {
        // (f) A shuttling path is charged reverse meters × multiplier plus
        // the switch penalty per cusp; the neutral penalties reproduce pure
        // length.
        let shuttle = vec![
            wp_dir(0.0, 0.0, SegmentDir::Forward),
            wp_dir(1.0, 0.0, SegmentDir::Forward),
            wp_dir(0.5, 0.0, SegmentDir::Reverse),
            wp_dir(1.5, 0.0, SegmentDir::Forward),
        ];
        let pen = CostPenalties {
            reverse_cost_multiplier: 2.0,
            direction_switch_penalty: 0.6,
        };
        // 1.0 (F) + 0.5·2 (R) + 0.6 + 1.0 (F) + 0.6 = 4.2
        let cost = path_cost(&shuttle, None, 0.0, 0.0, pen);
        assert!((cost - 4.2).abs() < 1e-9, "shuttle cost {cost} != 4.2");
        let neutral = path_cost(&shuttle, None, 0.0, 0.0, CostPenalties::none());
        assert!(
            (neutral - 2.5).abs() < 1e-9,
            "neutral cost {neutral} != 2.5"
        );
    }

    /// Search-cost instrumentation on the production-scale grid (400×400,
    /// 40m × 40m) with gauntlet-style clutter: measures node-expansion growth
    /// of the bidirectional primitive set and the RS accelerator against the
    /// forward-only baseline. Budget-asserted loosely (debug profile); the
    /// printed numbers are the report. Run with --nocapture for details.
    #[test]
    fn bidirectional_search_budget_gauntlet_scale() {
        let mut grid = OccupancyGrid::new(400, 400, 0.1, -20.0, -20.0);
        // Corridor walls y = ±1.2 plus a 6-cone slalom (0.15m blobs).
        let mut x = -0.5;
        while x <= 12.5 {
            for yo in [1.2, 1.25, -1.2, -1.25] {
                grid.set_occupied(x, yo);
            }
            x += 0.05;
        }
        for i in 0..6 {
            let cx = 1.5 + 1.8 * i as f64;
            let cy = if i % 2 == 0 { 0.45 } else { -0.45 };
            let mut dx = -0.15;
            while dx <= 0.15 {
                let mut dy = -0.15;
                while dy <= 0.15 {
                    grid.set_occupied(cx + dx, cy + dy);
                    dy += 0.05;
                }
                dx += 0.05;
            }
        }
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 12.0,
            y: 0.0,
            theta: 0.0,
        };
        let mut results = Vec::new();
        for (label, rev, rs) in [
            ("forward-only", false, false),
            ("bidirectional", true, false),
            ("bidirectional+RS", true, true),
        ] {
            let planner = HybridAStar::new(HybridAStarConfig {
                reverse_enabled: rev,
                rs_expansion_enabled: rs,
                ..Default::default()
            });
            let clearance = planner.build_clearance(&grid);
            let t0 = std::time::Instant::now();
            let (path, stats) = planner.plan_with_stats(&start, &goal, &grid, clearance.as_ref());
            let elapsed = t0.elapsed();
            println!(
                "{label}: found={} iters={} rs_attempts={} rs_connected={} time={:?}",
                path.is_some(),
                stats.iterations,
                stats.rs_attempts,
                stats.rs_connected,
                elapsed
            );
            assert!(path.is_some(), "{label} must solve the gauntlet corridor");
            results.push((label, stats.iterations, elapsed));
        }
        // The slalom has a clear route: even bidirectional must stay far from
        // the iteration cap and inside the 250ms replan slot in DEBUG builds.
        for (label, iters, elapsed) in &results {
            assert!(
                *iters < 100_000,
                "{label} used {iters} expansions on a clear route"
            );
            assert!(
                *elapsed < std::time::Duration::from_millis(250),
                "{label} took {elapsed:?} — exceeds the replan slot even in debug"
            );
        }
    }
}

#[cfg(test)]
mod escape_tests {
    use super::*;

    /// Tracked cone (physical extent 0.15m) dead ahead of the origin-facing
    /// robot: center distance 0.38 puts the robot inside the hard-inflation
    /// band (0.39 = dwa.robot_radius 0.24 + extent) while staying outside
    /// the true-footprint threshold (0.37 = 0.15 + 0.19 + 0.03).
    const CONE: PhysicalObstacle = PhysicalObstacle {
        x: 0.38,
        y: 0.0,
        radius: 0.15,
    };

    /// Wall point samples (untracked, radius 0) flanking the robot at
    /// y = ±0.235, x ∈ [-0.2, 0.4]: center distance 0.235 is inside their
    /// 0.24 hard inflation and outside the 0.22 true-footprint threshold.
    fn wall_points() -> Vec<PhysicalObstacle> {
        let mut pts = Vec::new();
        let mut x = -0.2;
        while x <= 0.4 + 1e-9 {
            for y in [0.235, -0.235] {
                pts.push(PhysicalObstacle { x, y, radius: 0.0 });
            }
            x += 0.05;
        }
        pts
    }

    /// The live wedge, reconstructed: grid painted with HARD inflation
    /// (robot_radius 0.24 + obstacle extent) around the cone and the wall
    /// samples — exactly what main.rs paints from detections. The origin
    /// cell AND every 0.2m first-step successor land in inflated cells
    /// (forward steps inside the cone's 0.39 blob, reverse steps inside the
    /// wall points' overlapping 0.24 blobs), so the un-relaxed search dies
    /// immediately. The TRUE footprint is free at the origin and straight
    /// behind: the physical pocket is escapable, only its inflation is not.
    fn pocket() -> (OccupancyGrid, Vec<PhysicalObstacle>) {
        let mut physical = vec![CONE];
        physical.extend(wall_points());
        let mut grid = OccupancyGrid::new(100, 100, 0.1, -5.0, -5.0);
        for obs in &physical {
            let blob = 0.24 + obs.radius;
            let mut dx = -(blob + 0.05);
            while dx <= blob + 0.05 {
                let mut dy = -(blob + 0.05);
                while dy <= blob + 0.05 {
                    if (dx * dx + dy * dy).sqrt() <= blob {
                        grid.set_occupied(obs.x + dx, obs.y + dy);
                    }
                    dy += 0.05;
                }
                dx += 0.05;
            }
        }
        (grid, physical)
    }

    fn dist_to_cone(wp: &PathWaypoint) -> f64 {
        ((wp.x - CONE.x).powi(2) + (wp.y - CONE.y).powi(2)).sqrt()
    }

    /// First same-direction run of a path (the leg the executor runs first).
    fn first_run(path: &[PathWaypoint]) -> &[PathWaypoint] {
        let dir = path.get(1).map_or(SegmentDir::Forward, |w| w.dir);
        let end = path.iter().position(|w| w.dir != dir).unwrap_or(path.len());
        &path[..end.max(1)]
    }

    #[test]
    fn wedged_start_escape_mode_produces_reverse_escape_plan() {
        // (a) + (c): start wedged in the inflation pocket, rear physically
        // open (no wall behind). Escape mode must activate, a plan must
        // exist, its FIRST segment must be reverse, and the first leg must
        // monotonically increase clearance to the wedging cone.
        let (grid, physical) = pocket();
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };

        assert!(grid.is_occupied(start.x, start.y), "start must be wedged");
        let zone = start_escape_zone(&grid, &physical, start.x, start.y, 0.6);
        assert!(
            zone.is_some(),
            "escape mode must activate: start inside inflation, footprint free"
        );

        // Control (the live failure): without the relaxation the search dies
        // immediately — every first-step successor is hard-blocked.
        assert!(
            planner.plan(&start, &goal, &grid).is_none(),
            "un-relaxed search must fail from the pocket (pre-fix behavior)"
        );

        let path = planner
            .plan_with_obstacles(&start, &goal, &grid, &physical)
            .expect("escape mode must produce a plan out of the pocket");
        assert!(path.len() > 2);
        assert_eq!(
            path[1].dir,
            SegmentDir::Reverse,
            "with the cone dead ahead and the rear open, the first segment must be reverse"
        );
        // The first leg increases clearance to the wedging cone: never comes
        // closer than the start already is, and ends strictly farther away.
        let run = first_run(&path);
        let start_dist = dist_to_cone(&run[0]);
        for w in run {
            assert!(
                dist_to_cone(w) >= start_dist - 1e-9,
                "escape leg approached the wedging cone at ({:.2},{:.2})",
                w.x,
                w.y
            );
        }
        assert!(
            dist_to_cone(run.last().unwrap()) > start_dist + 0.1,
            "escape leg must end clearly farther from the cone"
        );
        let last = path.last().unwrap();
        assert!(
            ((last.x - goal.x).powi(2) + (last.y - goal.y).powi(2)).sqrt() < 0.5,
            "escape plan must still end near the goal"
        );
    }

    #[test]
    fn free_start_is_unchanged_node_for_node() {
        // (a) control: with the start OUTSIDE the pocket the escape zone is
        // None and the obstacle-aware entry must reproduce the standard
        // search exactly — waypoint for waypoint.
        let (grid, physical) = pocket();
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: -1.5,
            y: 0.5,
            theta: 0.0,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };
        assert!(!grid.is_occupied(start.x, start.y));
        assert!(start_escape_zone(&grid, &physical, start.x, start.y, 0.6).is_none());

        let standard = planner
            .plan(&start, &goal, &grid)
            .expect("free start must plan");
        let with_obstacles = planner
            .plan_with_obstacles(&start, &goal, &grid, &physical)
            .expect("obstacle-aware entry must plan identically");
        assert_eq!(standard.len(), with_obstacles.len());
        for (a, b) in standard.iter().zip(&with_obstacles) {
            assert_eq!(
                (a.x, a.y, a.theta, a.steering, a.dir),
                (b.x, b.y, b.theta, b.steering, b.dir),
                "free-start plan must be unchanged node-for-node"
            );
        }
    }

    #[test]
    fn physical_overlap_keeps_pre_escape_failure_behavior() {
        // (b) Start physically overlapping the cone (0.28m < the 0.37
        // true-footprint threshold): the guard rail refuses escape mode and
        // behavior is exactly as before — the boxed-in search fails.
        let (grid, physical) = pocket();
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: 0.1,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };
        assert!(
            start_escape_zone(&grid, &physical, start.x, start.y, 0.6).is_none(),
            "physical overlap must never activate escape mode"
        );
        assert!(planner.plan(&start, &goal, &grid).is_none());
        assert!(
            planner
                .plan_with_obstacles(&start, &goal, &grid, &physical)
                .is_none(),
            "overlapping start must fail exactly like the pre-escape planner"
        );
    }

    #[test]
    fn escape_path_survives_validity_reverification_and_smoothing() {
        // (e) The freshly planned escape must NOT be judged invalid by the
        // per-cycle re-verification while the robot is still in the pocket:
        // the relaxed rule accepts the initial leg, the strict rule (the
        // pre-fix check) rejects it — proving the relaxation is what keeps
        // the plan alive. The smoother must survive the same pocket.
        let (grid, physical) = pocket();
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        };
        let goal = Pose {
            x: 3.0,
            y: 0.0,
            theta: 0.0,
        };
        let zone = start_escape_zone(&grid, &physical, start.x, start.y, 0.6)
            .expect("escape mode must activate");
        let clearance = planner.build_clearance(&grid);
        let raw = planner
            .plan_with_escape(&start, &goal, &grid, clearance.as_ref(), Some(&zone))
            .0
            .expect("escape plan must exist");

        let escape_leg = first_run(&raw);
        assert!(
            path_remains_valid_with_escape(escape_leg, &grid, Some(&zone)),
            "relaxed re-verification must keep the escape leg valid"
        );
        assert!(
            !path_remains_valid(escape_leg, &grid),
            "the strict (pre-fix) check must reject the same leg — the \
             relaxation is load-bearing"
        );

        // The smoothed candidate (what main.rs actually publishes) keeps the
        // wedged start point and stays escape-valid on its first leg.
        let cfg = HybridAStarConfig::default();
        let smoothed =
            smoother::smooth_path(&raw, &grid, clearance.as_ref(), &cfg, 2.4, Some(&zone));
        assert!(smoothed.len() >= 2);
        assert!(
            (smoothed[0].x - start.x).abs() < 1e-9 && (smoothed[0].y - start.y).abs() < 1e-9,
            "smoothing must preserve the wedged start point"
        );
        assert!(
            path_remains_valid_with_escape(first_run(&smoothed), &grid, Some(&zone)),
            "smoothed escape leg must pass the relaxed re-verification"
        );
    }

    #[test]
    fn pose_blocked_relaxes_only_inside_the_zone() {
        // The relaxation is spatially bounded: inside the zone the pocket
        // band is free (true footprint), outside it the very same hard
        // inflation blocks — and out-of-grid positions stay blocked even
        // inside the zone.
        let (grid, physical) = pocket();
        let zone = start_escape_zone(&grid, &physical, 0.0, 0.0, 0.6).expect("zone must build");
        // In-zone pocket band point: hard-blocked but footprint-free.
        assert!(grid.is_occupied(-0.2, 0.0));
        assert!(!pose_blocked(&grid, Some(&zone), -0.2, 0.0));
        // In-zone physical overlap: still blocked.
        assert!(pose_blocked(&grid, Some(&zone), 0.3, 0.0));
        // Outside the zone, hard inflation applies unchanged.
        assert!(pose_blocked(&grid, Some(&zone), 0.62, 0.23));
        // No zone: plain hard inflation.
        assert!(pose_blocked(&grid, None, -0.2, 0.0));
        // Out of grid bounds: blocked regardless (zone centered at a corner
        // cannot open the map edge).
        let corner_zone = EscapeZone {
            cx: -4.95,
            cy: -4.95,
            radius: 0.6,
            obstacles: &[],
        };
        assert!(pose_blocked(&grid, Some(&corner_zone), -5.2, -4.95));
    }
}
