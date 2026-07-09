/// Sensor store: intra-process shared state for the SensPerc process.
///
/// All driver threads write raw data into ring buffers.
/// Processing threads read from ring buffers and write results to atomic slots.
/// The aggregator reads all atomic slots to compose WorldState.
pub mod atomic_slot;
pub mod ring_buffer;
pub mod types;

use atomic_slot::AtomicSlot;
use ring_buffer::RingBuffer;
use types::*;

use std::collections::VecDeque;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

/// Pose history depth: 64 entries covers >3s at the sim's 20Hz pose rate.
const POSE_HISTORY_LEN: usize = 64;

fn normalize_angle(a: f64) -> f64 {
    let mut a = a;
    while a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    while a < -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    }
    a
}

/// Central shared state for the SensPerc process.
///
/// Shared across all threads via Arc (cloned from SensorStore).
pub struct SensorStore {
    // --- Ring buffers for raw sensor data (drivers → processors) ---
    pub camera_buffer: RingBuffer<CameraFrame>,
    pub lidar_buffer: RingBuffer<LidarScan>,
    pub imu_buffer: RingBuffer<ImuReading>,

    // --- Atomic slots for processed results (processors → aggregator) ---
    // Written by the SensorFusion thread once EKF lands; aggregator already references the slot field.
    #[allow(dead_code)]
    pub latest_fused_state: AtomicSlot<FusedState>,

    // --- Localization: pose and velocity from any source ---
    // Updated by: sim ground truth (CH5), odometry (CH3), or SLAM
    // Latest lidar scan, non-destructive (the ring buffer is SLAM's work
    // queue; popping it from another thread steals scans — the aggregator
    // and SLAM used to race on it, leaving the aggregator scanless on ~half
    // its cycles and publishing obstacle-free WorldStates).
    pub latest_scan: AtomicSlot<LidarScan>,

    pub latest_pose: AtomicSlot<Pose2D>,
    pub latest_velocity: AtomicSlot<Twist2D>,
    pub localization_confidence: AtomicSlot<f32>,

    // --- Timestamped pose history (for scan-time pose lookup) ---
    // Obstacle projection must use the pose the robot had AT SCAN TIME:
    // pairing a scan with the latest pose displaces projected obstacles by
    // ~omega * skew * range during fast yaw (decimeters), which measured as
    // mid-turn obstacle "swim" and clipping in the gauntlet runs.
    pose_history: Mutex<VecDeque<(u64, Pose2D)>>,

    // --- Perception output ---
    pub latest_detections: AtomicSlot<Vec<crate::perception::detector::CameraDetection>>,

    // --- SLAM output ---
    pub slam_local_map: AtomicSlot<SlamOccupancyGrid>,

    // --- Counters for monitoring ---
    pub frame_count: AtomicU64,
    pub scan_count: AtomicU64,
    pub imu_count: AtomicU64,
}

impl SensorStore {
    pub fn new() -> Self {
        Self {
            camera_buffer: RingBuffer::new(4), // ~133ms of frames at 30Hz
            lidar_buffer: RingBuffer::new(8),  // ~800ms of scans at 10Hz
            imu_buffer: RingBuffer::new(128),  // ~1.28s of readings at 100Hz
            latest_fused_state: AtomicSlot::new(),
            latest_scan: AtomicSlot::new(),
            latest_pose: AtomicSlot::new(),
            latest_velocity: AtomicSlot::new(),
            localization_confidence: AtomicSlot::new(),
            pose_history: Mutex::new(VecDeque::with_capacity(POSE_HISTORY_LEN)),
            latest_detections: AtomicSlot::new(),
            slam_local_map: AtomicSlot::new(),
            frame_count: AtomicU64::new(0),
            scan_count: AtomicU64::new(0),
            imu_count: AtomicU64::new(0),
        }
    }

    /// Record a timestamped pose into the history (and refresh the latest
    /// slot). Zero timestamps (unstamped sources) skip the history.
    pub fn push_stamped_pose(&self, timestamp_ns: u64, pose: Pose2D) {
        self.latest_pose.store(pose.clone());
        if timestamp_ns == 0 {
            return;
        }
        let mut hist = self.pose_history.lock().unwrap();
        // Keep the history monotonic; drop out-of-order stamps.
        if hist.back().is_some_and(|(t, _)| *t >= timestamp_ns) {
            return;
        }
        if hist.len() == POSE_HISTORY_LEN {
            hist.pop_front();
        }
        hist.push_back((timestamp_ns, pose));
    }

    /// Pose interpolated at `timestamp_ns` from the history. Clamps to the
    /// nearest end outside the recorded range; None if the history is empty.
    pub fn pose_at(&self, timestamp_ns: u64) -> Option<Pose2D> {
        let hist = self.pose_history.lock().unwrap();
        let (first, last) = (hist.front()?, hist.back()?);
        if timestamp_ns <= first.0 {
            return Some(first.1.clone());
        }
        if timestamp_ns >= last.0 {
            return Some(last.1.clone());
        }
        let idx = hist.partition_point(|(t, _)| *t < timestamp_ns);
        let (t0, p0) = &hist[idx - 1];
        let (t1, p1) = &hist[idx];
        let f = (timestamp_ns - t0) as f64 / (t1 - t0) as f64;
        let dtheta = normalize_angle(p1.theta - p0.theta);
        Some(Pose2D {
            x: p0.x + f * (p1.x - p0.x),
            y: p0.y + f * (p1.y - p0.y),
            theta: normalize_angle(p0.theta + f * dtheta),
        })
    }

    /// Push a camera frame (called by CameraDriver thread).
    pub fn push_camera_frame(&self, frame: CameraFrame) {
        self.camera_buffer.push_overwrite(frame);
        self.frame_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Push a LiDAR scan (called by LidarDriver thread).
    pub fn push_lidar_scan(&self, scan: LidarScan) {
        self.lidar_buffer.push_overwrite(scan);
        self.scan_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Push an IMU reading (called by ImuDriver thread).
    pub fn push_imu_reading(&self, reading: ImuReading) {
        self.imu_buffer.push_overwrite(reading);
        self.imu_count.fetch_add(1, Ordering::Relaxed);
    }

    /// Get throughput stats.
    pub fn stats(&self) -> SensorStats {
        SensorStats {
            camera_frames: self.frame_count.load(Ordering::Relaxed),
            lidar_scans: self.scan_count.load(Ordering::Relaxed),
            imu_readings: self.imu_count.load(Ordering::Relaxed),
        }
    }
}

impl Default for SensorStore {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug)]
pub struct SensorStats {
    pub camera_frames: u64,
    pub lidar_scans: u64,
    pub imu_readings: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(x: f64, theta: f64) -> Pose2D {
        Pose2D { x, y: 0.0, theta }
    }

    #[test]
    fn pose_at_interpolates_between_stamps() {
        let store = SensorStore::new();
        store.push_stamped_pose(1_000, pose(0.0, 0.0));
        store.push_stamped_pose(2_000, pose(1.0, 0.4));
        let p = store.pose_at(1_500).unwrap();
        assert!((p.x - 0.5).abs() < 1e-9);
        assert!((p.theta - 0.2).abs() < 1e-9);
    }

    #[test]
    fn pose_at_clamps_outside_range() {
        let store = SensorStore::new();
        store.push_stamped_pose(1_000, pose(0.0, 0.0));
        store.push_stamped_pose(2_000, pose(1.0, 0.0));
        assert!((store.pose_at(500).unwrap().x - 0.0).abs() < 1e-9);
        assert!((store.pose_at(9_000).unwrap().x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn pose_at_interpolates_theta_across_pi_wrap() {
        // 3.0 rad -> -3.0 rad is a 0.28 rad step through pi, not a 6 rad
        // swing back through zero. Naive lerp would put the midpoint near 0
        // and displace a 3m-away projected obstacle by ~6m.
        let store = SensorStore::new();
        store.push_stamped_pose(1_000, pose(0.0, 3.0));
        store.push_stamped_pose(2_000, pose(0.0, -3.0));
        let p = store.pose_at(1_500).unwrap();
        assert!(
            (p.theta.abs() - std::f64::consts::PI).abs() < 0.02,
            "midpoint should be near +-pi, got {}",
            p.theta
        );
    }

    #[test]
    fn pose_at_empty_history_is_none() {
        let store = SensorStore::new();
        assert!(store.pose_at(1_000).is_none());
    }

    #[test]
    fn unstamped_and_out_of_order_poses_skip_history() {
        let store = SensorStore::new();
        store.push_stamped_pose(0, pose(9.0, 0.0)); // unstamped: latest only
        assert!(store.pose_at(1).is_none());
        store.push_stamped_pose(2_000, pose(1.0, 0.0));
        store.push_stamped_pose(1_500, pose(5.0, 0.0)); // out of order: dropped
        assert!((store.pose_at(2_000).unwrap().x - 1.0).abs() < 1e-9);
    }

    #[test]
    fn history_is_bounded() {
        let store = SensorStore::new();
        for i in 0..(POSE_HISTORY_LEN as u64 + 20) {
            store.push_stamped_pose(1_000 + i, pose(i as f64, 0.0));
        }
        let hist = store.pose_history.lock().unwrap();
        assert_eq!(hist.len(), POSE_HISTORY_LEN);
    }
}
