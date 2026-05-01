/// Occupancy grid builder via raycasting.
///
/// Builds a local 2D occupancy grid from LiDAR scans and the
/// estimated robot pose. Uses log-odds update for probabilistic
/// occupancy estimation.
use crate::store::types::SlamOccupancyGrid;

const GRID_WIDTH: usize = 200;
const GRID_HEIGHT: usize = 200;
const GRID_RESOLUTION: f64 = 0.1; // meters per cell

// Log-odds parameters
const LOG_ODD_OCC: i16 = 3; // increment for occupied
const LOG_ODD_FREE: i16 = -1; // decrement for free
const LOG_ODD_MIN: i16 = -10;
const LOG_ODD_MAX: i16 = 50;
const LOG_ODD_THRESHOLD: i16 = 0; // above = occupied

pub struct GridBuilder {
    log_odds: Vec<i16>, // internal log-odds representation
}

impl GridBuilder {
    pub fn new() -> Self {
        Self {
            log_odds: vec![0i16; GRID_WIDTH * GRID_HEIGHT],
        }
    }

    /// Update the occupancy grid with a LiDAR scan from the given pose.
    ///
    /// For each ray:
    /// - Walk cells from robot pose along ray direction
    /// - Mark traversed cells as free (decrement log-odds)
    /// - Mark endpoint cell as occupied (increment log-odds)
    #[allow(clippy::too_many_arguments)] // Mirrors LiDAR scan parameter set; bundling would just rename the noise.
    pub fn update(
        &mut self,
        robot_x: f64,
        robot_y: f64,
        robot_theta: f64,
        ranges: &[f32],
        angle_min: f32,
        angle_increment: f32,
        range_min: f32,
        range_max: f32,
    ) {
        // Grid origin: centered on robot
        let origin_x = robot_x - (GRID_WIDTH as f64 * GRID_RESOLUTION) / 2.0;
        let origin_y = robot_y - (GRID_HEIGHT as f64 * GRID_RESOLUTION) / 2.0;

        // Clear grid (reset to zero) — rolling window, rebuild each time
        self.log_odds.fill(0);

        for (i, &range) in ranges.iter().enumerate() {
            if range < range_min || range > range_max {
                continue;
            }

            let angle = robot_theta + (angle_min + i as f32 * angle_increment) as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            // Raycasting: walk along the ray
            let step = GRID_RESOLUTION * 0.5; // half-cell steps for accuracy
            let num_steps = (range as f64 / step) as usize;

            for s in 0..num_steps {
                let d = s as f64 * step;
                let wx = robot_x + d * cos_a;
                let wy = robot_y + d * sin_a;

                let gx = ((wx - origin_x) / GRID_RESOLUTION) as isize;
                let gy = ((wy - origin_y) / GRID_RESOLUTION) as isize;

                if gx >= 0 && gy >= 0 && (gx as usize) < GRID_WIDTH && (gy as usize) < GRID_HEIGHT {
                    let idx = gy as usize * GRID_WIDTH + gx as usize;
                    // Mark as free
                    self.log_odds[idx] = (self.log_odds[idx] + LOG_ODD_FREE).max(LOG_ODD_MIN);
                }
            }

            // Mark endpoint as occupied
            let ox = robot_x + range as f64 * cos_a;
            let oy = robot_y + range as f64 * sin_a;
            let gx = ((ox - origin_x) / GRID_RESOLUTION) as isize;
            let gy = ((oy - origin_y) / GRID_RESOLUTION) as isize;

            if gx >= 0 && gy >= 0 && (gx as usize) < GRID_WIDTH && (gy as usize) < GRID_HEIGHT {
                let idx = gy as usize * GRID_WIDTH + gx as usize;
                self.log_odds[idx] = (self.log_odds[idx] + LOG_ODD_OCC).min(LOG_ODD_MAX);
            }
        }
    }

    /// Convert log-odds grid to binary occupancy grid output.
    pub fn to_occupancy_grid(&self, robot_x: f64, robot_y: f64) -> SlamOccupancyGrid {
        let origin_x = robot_x - (GRID_WIDTH as f64 * GRID_RESOLUTION) / 2.0;
        let origin_y = robot_y - (GRID_HEIGHT as f64 * GRID_RESOLUTION) / 2.0;

        let data: Vec<u8> = self
            .log_odds
            .iter()
            .map(|&lo| if lo > LOG_ODD_THRESHOLD { 100 } else { 0 })
            .collect();

        SlamOccupancyGrid {
            width: GRID_WIDTH,
            height: GRID_HEIGHT,
            resolution: GRID_RESOLUTION,
            origin_x,
            origin_y,
            data,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_builder_single_wall() {
        let mut builder = GridBuilder::new();

        // Simulate a wall at 3m directly ahead (0 degrees)
        let ranges: Vec<f32> = (0..360)
            .map(|i| {
                let angle = i as f32 * std::f32::consts::TAU / 360.0;
                if angle.abs() < 0.05 {
                    3.0
                } else {
                    10.0
                } // wall at 3m, 0 deg
            })
            .collect();

        builder.update(
            0.0,
            0.0,
            0.0,
            &ranges,
            0.0,
            std::f32::consts::TAU / 360.0,
            0.1,
            12.0,
        );

        let grid = builder.to_occupancy_grid(0.0, 0.0);
        assert_eq!(grid.width, GRID_WIDTH);
        assert_eq!(grid.height, GRID_HEIGHT);

        // Check that the cell at ~3m ahead is occupied
        let gx = ((3.0 - grid.origin_x) / grid.resolution) as usize;
        let gy = ((0.0 - grid.origin_y) / grid.resolution) as usize;
        if gx < grid.width && gy < grid.height {
            let idx = gy * grid.width + gx;
            assert_eq!(grid.data[idx], 100, "Cell at wall should be occupied");
        }

        // Check that cells between robot and wall are free
        let mid_gx = ((1.5 - grid.origin_x) / grid.resolution) as usize;
        let mid_gy = ((0.0 - grid.origin_y) / grid.resolution) as usize;
        if mid_gx < grid.width && mid_gy < grid.height {
            let idx = mid_gy * grid.width + mid_gx;
            assert_eq!(
                grid.data[idx], 0,
                "Cell between robot and wall should be free"
            );
        }
    }

    #[test]
    fn test_grid_builder_circular_room() {
        let mut builder = GridBuilder::new();

        // Circular room: all ranges at 4m
        let n = 360;
        let ranges = vec![4.0f32; n];

        builder.update(
            0.0,
            0.0,
            0.0,
            &ranges,
            0.0,
            std::f32::consts::TAU / n as f32,
            0.1,
            12.0,
        );

        let grid = builder.to_occupancy_grid(0.0, 0.0);

        // Count occupied cells — should form a rough circle
        let occ_count = grid.data.iter().filter(|&&v| v == 100).count();
        assert!(
            occ_count > 100,
            "Should have many occupied cells forming circle walls"
        );
        assert!(occ_count < 2000, "Not too many — just the ring");
    }
}
