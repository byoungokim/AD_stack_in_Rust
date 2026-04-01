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

use std::sync::atomic::{AtomicU64, Ordering};

/// Central shared state for the SensPerc process.
///
/// Shared across all threads via Arc (cloned from SensorStore).
pub struct SensorStore {
    // --- Ring buffers for raw sensor data (drivers → processors) ---
    pub camera_buffer: RingBuffer<CameraFrame>,
    pub lidar_buffer: RingBuffer<LidarScan>,
    pub imu_buffer: RingBuffer<ImuReading>,

    // --- Atomic slots for processed results (processors → aggregator) ---
    pub latest_fused_state: AtomicSlot<FusedState>,

    // --- Counters for monitoring ---
    pub frame_count: AtomicU64,
    pub scan_count: AtomicU64,
    pub imu_count: AtomicU64,
}

impl SensorStore {
    pub fn new() -> Self {
        Self {
            camera_buffer: RingBuffer::new(4),   // ~133ms of frames at 30Hz
            lidar_buffer: RingBuffer::new(8),    // ~800ms of scans at 10Hz
            imu_buffer: RingBuffer::new(128),    // ~1.28s of readings at 100Hz
            latest_fused_state: AtomicSlot::new(),
            frame_count: AtomicU64::new(0),
            scan_count: AtomicU64::new(0),
            imu_count: AtomicU64::new(0),
        }
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
