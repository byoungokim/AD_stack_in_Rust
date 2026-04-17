/// SLAM module: Feature-based 2D LiDAR SLAM.
///
/// Extracts line/corner features from LiDAR scans, matches between
/// consecutive scans to estimate motion, accumulates into global pose,
/// and builds an occupancy grid via raycasting.
///
/// Runs as a single thread at 10Hz (LiDAR rate).
pub mod features;
pub mod scan_matcher;
pub mod pose_tracker;
pub mod grid_builder;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use tracing::{debug, info};

use crate::store::SensorStore;
use features::{scan_to_points, extract_lines, extract_corners, LineSegment};
use scan_matcher::match_scans;
use pose_tracker::PoseTracker;
use grid_builder::GridBuilder;

/// Run the SLAM processing loop.
///
/// Pops LiDAR scans from the sensor store, extracts features,
/// matches against previous scan, updates pose, builds occupancy grid.
pub fn slam_loop(store: &Arc<SensorStore>, shutdown: &AtomicBool) {
    info!("SLAM thread started");

    let mut tracker = PoseTracker::new();
    let mut grid_builder = GridBuilder::new();
    let mut prev_lines: Vec<LineSegment> = Vec::new();
    let mut cycle: u64 = 0;

    let interval = Duration::from_millis(100); // 10Hz

    while !shutdown.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // Get latest LiDAR scan
        let scan = match store.lidar_buffer.pop_latest() {
            Some(s) => s,
            None => {
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }
        };

        // 1. Convert scan to Cartesian points
        let points = scan_to_points(
            &scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
        );

        if points.len() < 10 {
            std::thread::sleep(Duration::from_millis(10));
            continue;
        }

        // 2. Extract line segments and corners
        let lines = extract_lines(&points, 0.05); // 5cm split threshold
        let _corners = extract_corners(&lines, 0.5); // ~30 degree minimum

        // 3. Match against previous scan
        if !prev_lines.is_empty() && !lines.is_empty() {
            let match_result = match_scans(
                &prev_lines, &lines,
                0.3,  // max angle diff for line matching (radians)
                2.0,  // max distance for matching (meters)
            );

            if match_result.confidence > 0.2 && match_result.num_matches >= 2 {
                // Update pose from scan matching
                tracker.update(match_result.dx, match_result.dy, match_result.dtheta);

                // Optionally fuse with IMU
                if let Some(imu) = store.imu_buffer.pop_latest() {
                    let imu_yaw = imu.orientation_euler.z;
                    tracker.fuse_imu_heading(imu_yaw, 0.1); // light IMU correction
                }

                // Write SLAM pose to store (confidence=0.8, higher than odometry)
                // Only overwrite if no higher-priority source is fresh
                if store.latest_pose.age_secs() > 0.05 || store.localization_confidence.load().unwrap_or(0.0) < 0.8 {
                    store.latest_pose.store(crate::store::types::Pose2D {
                        x: tracker.x,
                        y: tracker.y,
                        theta: tracker.theta,
                    });
                    store.localization_confidence.store(0.8);
                }
            }
        }

        // 4. Build occupancy grid from current scan + estimated pose
        grid_builder.update(
            tracker.x, tracker.y, tracker.theta,
            &scan.ranges,
            scan.angle_min, scan.angle_increment,
            scan.range_min, scan.range_max,
        );

        let occupancy_grid = grid_builder.to_occupancy_grid(tracker.x, tracker.y);
        store.slam_local_map.store(occupancy_grid);

        // Save current lines for next iteration
        prev_lines = lines;

        cycle += 1;
        if cycle % 50 == 0 {
            debug!(
                "SLAM cycle {}: pose=({:.2}, {:.2}, {:.1}°) lines={} prev_lines={}",
                cycle, tracker.x, tracker.y, tracker.theta.to_degrees(),
                prev_lines.len(), prev_lines.len(),
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            std::thread::sleep(interval - elapsed);
        }
    }

    info!("SLAM thread stopped");
}
