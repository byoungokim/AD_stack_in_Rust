/// Hybrid A* global planner for Ackermann vehicles.
///
/// Plans in (x, y, theta) state space using kinematically feasible
/// motion primitives (bicycle model). Produces paths the Limo Pro
/// can actually follow without post-processing.
///
/// Runs at 1Hz — triggered by behavior planner when replanning is needed.
use std::collections::BinaryHeap;
use std::cmp::Ordering;

use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct HybridAStarConfig {
    #[serde(default = "default_xy_resolution")]
    pub xy_resolution: f64,      // meters per grid cell
    #[serde(default = "default_theta_resolution")]
    pub theta_resolution: f64,   // radians per heading bin
    #[serde(default = "default_wheelbase")]
    pub wheelbase: f64,          // meters
    #[serde(default = "default_max_steering")]
    pub max_steering_angle: f64, // radians
    #[serde(default = "default_step_size")]
    pub step_size: f64,          // meters per expansion step
    #[serde(default = "default_num_steer")]
    pub num_steer_angles: usize, // number of discrete steering samples
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,
}

fn default_xy_resolution() -> f64 { 0.1 }
fn default_theta_resolution() -> f64 { 0.1745 } // ~10 degrees
fn default_wheelbase() -> f64 { 0.2 }
fn default_max_steering() -> f64 { 0.48 }
fn default_step_size() -> f64 { 0.2 }
fn default_num_steer() -> usize { 5 }
fn default_max_iterations() -> usize { 100_000 }

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
        }
    }
}

/// A pose in the planning space.
#[derive(Debug, Clone)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

/// A waypoint in the planned path.
#[derive(Debug, Clone)]
pub struct PathWaypoint {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
    pub steering: f64, // steering angle used to reach this point
}

/// Simple occupancy grid for collision checking.
pub struct OccupancyGrid {
    pub width: usize,
    pub height: usize,
    pub resolution: f64,  // meters per cell
    pub origin_x: f64,
    pub origin_y: f64,
    pub data: Vec<u8>,    // 0 = free, 100 = occupied
}

impl OccupancyGrid {
    pub fn new(width: usize, height: usize, resolution: f64, origin_x: f64, origin_y: f64) -> Self {
        Self {
            width, height, resolution, origin_x, origin_y,
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
        other.f_cost.partial_cmp(&self.f_cost).unwrap_or(Ordering::Equal)
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

        Self { config, steer_angles }
    }

    /// Plan a path from start to goal on the given occupancy grid.
    /// Returns None if no path is found.
    pub fn plan(
        &self,
        start: &Pose,
        goal: &Pose,
        grid: &OccupancyGrid,
    ) -> Option<Vec<PathWaypoint>> {
        let mut open = BinaryHeap::new();
        let mut nodes: Vec<Node> = Vec::new();
        let mut closed = std::collections::HashSet::new();

        let start_node = Node {
            x: start.x,
            y: start.y,
            theta: normalize_angle(start.theta),
            g_cost: 0.0,
            f_cost: heuristic(start.x, start.y, start.theta, goal),
            steering: 0.0,
            parent_idx: None,
        };

        nodes.push(start_node);
        open.push(NodeWrapper { f_cost: nodes[0].f_cost, idx: 0 });

        let mut iterations = 0;

        while let Some(current) = open.pop() {
            iterations += 1;
            if iterations > self.config.max_iterations {
                return None; // timeout
            }

            let node = &nodes[current.idx];
            let nx = node.x;
            let ny = node.y;
            let ntheta = node.theta;
            let ng_cost = node.g_cost;

            // Check if goal reached
            let dx = nx - goal.x;
            let dy = ny - goal.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist < self.config.xy_resolution * 2.0 {
                return Some(self.reconstruct_path(&nodes, current.idx));
            }

            // Discretize for closed set
            let key = self.discretize(nx, ny, ntheta);
            if closed.contains(&key) {
                continue;
            }
            closed.insert(key);

            // Expand with each steering angle (forward only for now)
            for &steer in &self.steer_angles {
                let (new_x, new_y, new_theta) =
                    self.bicycle_step(nx, ny, ntheta, steer);

                // Collision check
                if grid.is_occupied(new_x, new_y) {
                    continue;
                }

                let new_key = self.discretize(new_x, new_y, new_theta);
                if closed.contains(&new_key) {
                    continue;
                }

                let step_cost = self.config.step_size
                    + 0.5 * steer.abs(); // penalize steering
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
                    parent_idx: Some(current.idx),
                });

                open.push(NodeWrapper { f_cost: new_f, idx: new_idx });
            }
        }

        None // no path found
    }

    /// Bicycle model forward step.
    fn bicycle_step(&self, x: f64, y: f64, theta: f64, steer: f64) -> (f64, f64, f64) {
        let d = self.config.step_size;
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

    /// Discretize (x, y, theta) into a grid key for the closed set.
    fn discretize(&self, x: f64, y: f64, theta: f64) -> (i32, i32, i32) {
        let gx = (x / self.config.xy_resolution).round() as i32;
        let gy = (y / self.config.xy_resolution).round() as i32;
        let gt = (normalize_angle(theta) / self.config.theta_resolution).round() as i32;
        (gx, gy, gt)
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
            });
            idx = node.parent_idx;
        }

        path.reverse();
        path
    }
}

/// Heuristic: Euclidean distance + heading penalty.
fn heuristic(x: f64, y: f64, theta: f64, goal: &Pose) -> f64 {
    let dx = x - goal.x;
    let dy = y - goal.y;
    let dist = (dx * dx + dy * dy).sqrt();
    let dtheta = normalize_angle(theta - goal.theta).abs();
    dist + 0.3 * dtheta
}

fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI { a -= 2.0 * std::f64::consts::PI; }
    while a < -std::f64::consts::PI { a += 2.0 * std::f64::consts::PI; }
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
        let start = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let goal = Pose { x: 2.0, y: 0.0, theta: 0.0 };

        let path = planner.plan(&start, &goal, &empty_grid());
        assert!(path.is_some(), "Should find a straight-line path");

        let path = path.unwrap();
        assert!(path.len() > 2, "Path should have multiple waypoints");

        // First point near start
        assert!((path[0].x).abs() < 0.2);
        // Last point near goal
        let last = path.last().unwrap();
        assert!((last.x - 2.0).abs() < 0.5, "Last waypoint should be near goal");
    }

    #[test]
    fn test_plan_with_obstacle() {
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let start = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let goal = Pose { x: 3.0, y: 0.0, theta: 0.0 };

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
        assert!(max_y > 0.5, "Path should deviate significantly to avoid wall");
    }

    #[test]
    fn test_no_path_blocked() {
        let planner = HybridAStar::new(HybridAStarConfig {
            max_iterations: 5000, // limit search
            ..Default::default()
        });
        let start = Pose { x: 0.0, y: 0.0, theta: 0.0 };
        let goal = Pose { x: 3.0, y: 0.0, theta: 0.0 };

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
        assert!(path.is_none(), "Should not find a path when completely blocked");
    }

    #[test]
    fn test_bicycle_step_straight() {
        let planner = HybridAStar::new(HybridAStarConfig::default());
        let (nx, ny, nt) = planner.bicycle_step(0.0, 0.0, 0.0, 0.0);
        let step = planner.config.step_size; // 0.2
        assert!((nx - step).abs() < 1e-6); // step_size forward
        assert!(ny.abs() < 1e-6);
        assert!(nt.abs() < 1e-6);
    }
}
