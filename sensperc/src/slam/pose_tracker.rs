//! Pose tracker: accumulates scan matching results into global pose.
//!
//! Maintains the robot's estimated position in the world frame.
//! Optionally fuses with IMU data for heading correction.

/// Global pose tracker.
pub struct PoseTracker {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl PoseTracker {
    pub fn new() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            theta: 0.0,
        }
    }

    /// Update pose with a relative motion estimate from scan matching.
    pub fn update(&mut self, dx: f64, dy: f64, dtheta: f64) {
        // Transform delta from robot frame to world frame
        let cos_t = self.theta.cos();
        let sin_t = self.theta.sin();
        self.x += dx * cos_t - dy * sin_t;
        self.y += dx * sin_t + dy * cos_t;
        self.theta += dtheta;
        self.normalize_theta();
    }

    /// Fuse with IMU heading if available (simple complementary filter).
    pub fn fuse_imu_heading(&mut self, imu_yaw: f64, alpha: f64) {
        // alpha: weight of IMU (0.0 = ignore IMU, 1.0 = trust IMU fully)
        let diff = normalize_angle(imu_yaw - self.theta);
        self.theta += alpha * diff;
        self.normalize_theta();
    }

    fn normalize_theta(&mut self) {
        while self.theta > std::f64::consts::PI {
            self.theta -= 2.0 * std::f64::consts::PI;
        }
        while self.theta < -std::f64::consts::PI {
            self.theta += 2.0 * std::f64::consts::PI;
        }
    }
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

    #[test]
    fn test_straight_motion() {
        let mut tracker = PoseTracker::new();
        // Move 1m forward (dx=1, dy=0, dtheta=0)
        tracker.update(1.0, 0.0, 0.0);
        assert!((tracker.x - 1.0).abs() < 1e-6);
        assert!(tracker.y.abs() < 1e-6);
    }

    #[test]
    fn test_turn_then_move() {
        let mut tracker = PoseTracker::new();
        // Turn 90 degrees
        tracker.update(0.0, 0.0, std::f64::consts::FRAC_PI_2);
        // Move 1m forward (now in y direction)
        tracker.update(1.0, 0.0, 0.0);
        assert!(tracker.x.abs() < 1e-6);
        assert!((tracker.y - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_imu_fusion() {
        let mut tracker = PoseTracker::new();
        tracker.theta = 0.1; // slight drift
        tracker.fuse_imu_heading(0.0, 0.5); // IMU says 0, alpha=0.5
        assert!((tracker.theta - 0.05).abs() < 1e-6); // should be halfway
    }
}
