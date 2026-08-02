/// Occupancy grid builder via raycasting.
///
/// Builds a local 2D occupancy grid from LiDAR scans and the
/// estimated robot pose. Log-odds evidence is ACCUMULATED across
/// scans (review finding #8): the grid is world-anchored and scrolls
/// under the robot in whole-cell steps, so cell values are stable
/// scan-to-scan instead of being a single-scan binary snapshot.
use crate::store::types::SlamOccupancyGrid;

const GRID_WIDTH: usize = 200;
const GRID_HEIGHT: usize = 200;
const GRID_RESOLUTION: f64 = 0.1; // meters per cell

// Log-odds parameters (integer log-odds, updated at 10Hz).
//
// Tuned together so that:
// - Single-dropout immunity (the original motivation): 3 consecutive hits
//   (3 * OCC = 9) survive one contradicting free observation
//   (9 + FREE = 7 > threshold), so a wall cell missed by one noisy scan
//   does not flicker out of the published map.
// - Fast fade of moving objects: FREE (|−2|) is strong relative to OCC
//   (asymmetric evidence), and MAX caps how much belief a cell can bank,
//   so even a fully saturated cell observed free clears in
//   MAX / |FREE| = 6 updates (0.6 s at 10 Hz). A 0.75 m/s walker dwells
//   ~0.13 s per 0.1 m cell (1–2 hits), so its trail clears in 1–3 updates
//   once rays pass through the vacated cells.
// - Static persistence: cells re-hit every scan gain OCC per scan and pin
//   at MAX. There is deliberately NO global decay — belief changes only on
//   actual observations. (Multiplicative decay truncates poorly on integer
//   log-odds and would erode map areas the sensor currently cannot see.)
// - MIN bounds free-space confidence so a genuinely new obstacle inside
//   well-observed free space flips occupied within 2 hits
//   (MIN + 2 * OCC = 2 > threshold, i.e. 0.2 s), while a single spurious
//   hit (MIN + OCC = −1) does not.
const LOG_ODD_OCC: i16 = 3; // increment for an endpoint hit
const LOG_ODD_FREE: i16 = -2; // decrement for a traversed (free) cell
const LOG_ODD_MIN: i16 = -4;
const LOG_ODD_MAX: i16 = 12;
const LOG_ODD_THRESHOLD: i16 = 0; // above = occupied

pub struct GridBuilder {
    log_odds: Vec<i16>, // internal log-odds representation, row-major
    /// World-frame cell index of grid column 0 / row 0. The origin is
    /// snapped to whole cells so recentering is a pure translation of the
    /// retained map (origin_x = origin_cell_x * GRID_RESOLUTION).
    origin_cell_x: i64,
    origin_cell_y: i64,
    initialized: bool,
}

impl GridBuilder {
    pub fn new() -> Self {
        Self {
            log_odds: vec![0i16; GRID_WIDTH * GRID_HEIGHT],
            origin_cell_x: 0,
            origin_cell_y: 0,
            initialized: false,
        }
    }

    /// World-frame origin (meters) of the grid's (0, 0) cell corner.
    fn origin_meters(&self) -> (f64, f64) {
        (
            self.origin_cell_x as f64 * GRID_RESOLUTION,
            self.origin_cell_y as f64 * GRID_RESOLUTION,
        )
    }

    /// Map a world point to a grid buffer index, or None if outside.
    fn world_to_cell(&self, wx: f64, wy: f64) -> Option<usize> {
        let (origin_x, origin_y) = self.origin_meters();
        let gx = ((wx - origin_x) / GRID_RESOLUTION).floor() as i64;
        let gy = ((wy - origin_y) / GRID_RESOLUTION).floor() as i64;
        if gx < 0 || gy < 0 || gx >= GRID_WIDTH as i64 || gy >= GRID_HEIGHT as i64 {
            return None;
        }
        Some(gy as usize * GRID_WIDTH + gx as usize)
    }

    /// Recenter the grid on the robot by translating the retained cells.
    ///
    /// The target origin is snapped to whole cells; the buffer is shifted
    /// by the whole-cell delta so every retained cell keeps its WORLD
    /// position. Cells that scroll out are dropped; newly exposed cells
    /// start at 0 (unknown).
    fn recenter(&mut self, robot_x: f64, robot_y: f64) {
        let target_cx = (robot_x / GRID_RESOLUTION).floor() as i64 - GRID_WIDTH as i64 / 2;
        let target_cy = (robot_y / GRID_RESOLUTION).floor() as i64 - GRID_HEIGHT as i64 / 2;

        if !self.initialized {
            self.origin_cell_x = target_cx;
            self.origin_cell_y = target_cy;
            self.initialized = true;
            return;
        }

        let dx = target_cx - self.origin_cell_x;
        let dy = target_cy - self.origin_cell_y;
        if dx != 0 || dy != 0 {
            self.scroll(dx, dy);
            self.origin_cell_x = target_cx;
            self.origin_cell_y = target_cy;
        }
    }

    /// Shift the buffer so new[x][y] = old[x + dx][y + dy].
    fn scroll(&mut self, dx: i64, dy: i64) {
        if dx.abs() >= GRID_WIDTH as i64 || dy.abs() >= GRID_HEIGHT as i64 {
            // Jumped further than the whole grid — nothing overlaps.
            self.log_odds.fill(0);
            return;
        }

        let mut new_grid = vec![0i16; GRID_WIDTH * GRID_HEIGHT];
        let len = GRID_WIDTH - dx.unsigned_abs() as usize;
        let (dst_off, src_off) = if dx >= 0 {
            (0usize, dx as usize)
        } else {
            ((-dx) as usize, 0usize)
        };

        for ny in 0..GRID_HEIGHT {
            let oy = ny as i64 + dy;
            if oy < 0 || oy >= GRID_HEIGHT as i64 {
                continue;
            }
            let src_row = oy as usize * GRID_WIDTH;
            let dst_row = ny * GRID_WIDTH;
            new_grid[dst_row + dst_off..dst_row + dst_off + len]
                .copy_from_slice(&self.log_odds[src_row + src_off..src_row + src_off + len]);
        }
        self.log_odds = new_grid;
    }

    /// Update the occupancy grid with a LiDAR scan from the given pose.
    ///
    /// Evidence accumulates across calls. For each ray:
    /// - Walk cells from robot pose along ray direction
    /// - Mark traversed cells as free (decrement log-odds, once per cell per ray)
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
        // Keep the robot centered by SCROLLING the retained cells, never by
        // re-mapping coordinates over stale contents.
        self.recenter(robot_x, robot_y);

        for (i, &range) in ranges.iter().enumerate() {
            if range < range_min || range > range_max {
                continue;
            }

            let angle = robot_theta + (angle_min + i as f32 * angle_increment) as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();

            // Endpoint cell, computed first so the free-space walk can
            // avoid eroding the very cell this ray declares occupied.
            let end_cell = self.world_to_cell(
                robot_x + range as f64 * cos_a,
                robot_y + range as f64 * sin_a,
            );

            // Raycasting: walk along the ray
            let step = GRID_RESOLUTION * 0.5; // half-cell steps for accuracy
            let num_steps = (range as f64 / step) as usize;
            let mut last_cell: Option<usize> = None;

            for s in 0..num_steps {
                let d = s as f64 * step;
                let wx = robot_x + d * cos_a;
                let wy = robot_y + d * sin_a;

                let Some(idx) = self.world_to_cell(wx, wy) else {
                    continue;
                };
                if Some(idx) == end_cell {
                    // Reached the hit cell; the segment inside a convex cell
                    // is contiguous, so no free cells remain past this point.
                    break;
                }
                if Some(idx) == last_cell {
                    continue; // one free observation per cell per ray
                }
                last_cell = Some(idx);
                self.log_odds[idx] = (self.log_odds[idx] + LOG_ODD_FREE).max(LOG_ODD_MIN);
            }

            // Mark endpoint as occupied
            if let Some(idx) = end_cell {
                self.log_odds[idx] = (self.log_odds[idx] + LOG_ODD_OCC).min(LOG_ODD_MAX);
            }
        }
    }

    /// Convert log-odds grid to binary occupancy grid output.
    ///
    /// Wire format unchanged: 100 = occupied (log-odds above threshold),
    /// 0 otherwise. Unknown (log-odds exactly 0, never observed) is not
    /// representable in the current format and publishes as free.
    pub fn to_occupancy_grid(&self) -> SlamOccupancyGrid {
        let (origin_x, origin_y) = self.origin_meters();

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

    const ANGLE_INC: f32 = std::f32::consts::TAU / 360.0; // 1 degree

    /// Look up the published value at a world coordinate.
    fn cell_value(grid: &SlamOccupancyGrid, wx: f64, wy: f64) -> u8 {
        let gx = ((wx - grid.origin_x) / grid.resolution).floor() as isize;
        let gy = ((wy - grid.origin_y) / grid.resolution).floor() as isize;
        assert!(
            gx >= 0 && gy >= 0 && (gx as usize) < grid.width && (gy as usize) < grid.height,
            "query ({wx}, {wy}) outside grid"
        );
        grid.data[gy as usize * grid.width + gx as usize]
    }

    /// Forward-sector scan (90 beams over 0..90 deg): beams 0..3 return
    /// `front`, the rest return 10 m. Avoids grazing beams from below the
    /// x-axis so the beam-0 endpoint cell is touched only by beam 0.
    fn sector_ranges(front: f32) -> Vec<f32> {
        (0..90).map(|i| if i < 3 { front } else { 10.0 }).collect()
    }

    fn update_at(builder: &mut GridBuilder, x: f64, y: f64, ranges: &[f32]) {
        builder.update(x, y, 0.0, ranges, 0.0, ANGLE_INC, 0.1, 12.0);
    }

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

        builder.update(0.0, 0.0, 0.0, &ranges, 0.0, ANGLE_INC, 0.1, 12.0);

        let grid = builder.to_occupancy_grid();
        assert_eq!(grid.width, GRID_WIDTH);
        assert_eq!(grid.height, GRID_HEIGHT);

        // Check that the cell at ~3m ahead is occupied
        assert_eq!(
            cell_value(&grid, 3.0, 0.0),
            100,
            "Cell at wall should be occupied"
        );

        // Check that cells between robot and wall are free
        assert_eq!(
            cell_value(&grid, 1.5, 0.0),
            0,
            "Cell between robot and wall should be free"
        );
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

        let grid = builder.to_occupancy_grid();

        // Count occupied cells — should form a rough circle
        let occ_count = grid.data.iter().filter(|&&v| v == 100).count();
        assert!(
            occ_count > 100,
            "Should have many occupied cells forming circle walls"
        );
        assert!(occ_count < 2000, "Not too many — just the ring");
    }

    #[test]
    fn test_accumulation_survives_single_dropout() {
        let mut builder = GridBuilder::new();
        let wall = sector_ranges(3.0);

        // Cell hit in 3 consecutive scans...
        for _ in 0..3 {
            update_at(&mut builder, 0.0, 0.0, &wall);
        }
        let grid = builder.to_occupancy_grid();
        assert_eq!(cell_value(&grid, 3.0, 0.0), 100, "wall cell occupied");

        // ...stays occupied when one scan drops it (ray passes through).
        let dropout = sector_ranges(10.0);
        update_at(&mut builder, 0.0, 0.0, &dropout);
        let grid = builder.to_occupancy_grid();
        assert_eq!(
            cell_value(&grid, 3.0, 0.0),
            100,
            "single dropout must not flicker the cell out of the map"
        );
    }

    #[test]
    fn test_moving_object_footprint_fades() {
        let mut builder = GridBuilder::new();
        let open = sector_ranges(10.0);
        let occupied = sector_ranges(3.05);

        // Establish known-free space.
        for _ in 0..3 {
            update_at(&mut builder, 0.0, 0.0, &open);
        }
        assert_eq!(cell_value(&builder.to_occupancy_grid(), 3.05, 0.0), 0);

        // Transient object dwells for a few scans.
        for _ in 0..3 {
            update_at(&mut builder, 0.0, 0.0, &occupied);
        }
        assert_eq!(
            cell_value(&builder.to_occupancy_grid(), 3.05, 0.0),
            100,
            "object present must show occupied"
        );

        // Object leaves; the space is observed free again. The footprint
        // must clear within a handful of updates (<= 0.5 s at 10 Hz).
        let mut cleared_after = None;
        for k in 1..=5 {
            update_at(&mut builder, 0.0, 0.0, &open);
            if cell_value(&builder.to_occupancy_grid(), 3.05, 0.0) == 0 {
                cleared_after = Some(k);
                break;
            }
        }
        assert!(
            cleared_after.is_some(),
            "transient footprint must fade within 5 updates"
        );
    }

    #[test]
    fn test_scroll_preserves_world_positions() {
        let mut builder = GridBuilder::new();
        let wall = sector_ranges(3.05);

        for _ in 0..3 {
            update_at(&mut builder, 0.0, 0.0, &wall);
        }
        let grid = builder.to_occupancy_grid();
        assert_eq!(cell_value(&grid, 3.05, 0.0), 100);
        let old_origin_x = grid.origin_x;

        // Robot moves 1.5 cells forward; update with no returns so the only
        // effect is the recenter scroll.
        update_at(&mut builder, 0.15, 0.0, &[]);
        let grid = builder.to_occupancy_grid();
        assert!(
            grid.origin_x > old_origin_x,
            "grid origin must follow the robot"
        );
        assert_eq!(
            cell_value(&grid, 3.05, 0.0),
            100,
            "occupied WORLD position must survive the scroll"
        );
        assert_eq!(
            cell_value(&grid, 1.5, 0.0),
            0,
            "free world position stays free after scroll"
        );
    }

    #[test]
    fn test_static_wall_pins_at_max_and_never_fades() {
        let mut builder = GridBuilder::new();
        let wall = sector_ranges(3.0);

        // Warm-up: saturate the wall cell.
        for _ in 0..5 {
            update_at(&mut builder, 0.0, 0.0, &wall);
        }
        let idx = builder
            .world_to_cell(3.0, 0.0)
            .expect("wall cell inside grid");
        assert_eq!(builder.log_odds[idx], LOG_ODD_MAX, "wall pins at max");

        // Hit every scan for a long stretch: stays pinned, never fades.
        for _ in 0..50 {
            update_at(&mut builder, 0.0, 0.0, &wall);
            assert_eq!(builder.log_odds[idx], LOG_ODD_MAX);
            assert_eq!(cell_value(&builder.to_occupancy_grid(), 3.0, 0.0), 100);
        }
    }
}
