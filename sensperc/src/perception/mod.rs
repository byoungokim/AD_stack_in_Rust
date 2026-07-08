/// Camera perception pipeline.
///
/// Runs YOLO object detection on camera frames via ONNX Runtime (ort crate).
/// Produces bounding box detections with class labels and confidence scores.
///
/// When no ONNX model is available, falls back to a no-op detector
/// that passes through without errors.
pub mod detector;
pub mod postprocessing;
pub mod preprocessing;
pub mod tracker;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tracing::{debug, info, warn};

use crate::store::SensorStore;
use detector::ObjectDetector;

/// Run the perception processing loop.
///
/// Pops camera frames from the sensor store, runs object detection,
/// and writes results to the detection slot.
pub fn perception_loop(store: &Arc<SensorStore>, shutdown: &AtomicBool, model_path: &str) {
    info!("Perception thread started");

    let mut detector = match ObjectDetector::new(model_path) {
        Ok(d) => {
            info!("ONNX model loaded: {}", model_path);
            d
        }
        Err(e) => {
            warn!(
                "Failed to load ONNX model '{}': {}. Using fallback detector.",
                model_path, e
            );
            ObjectDetector::fallback()
        }
    };

    let interval = Duration::from_millis(67); // ~15Hz
    let mut cycle: u64 = 0;

    while !shutdown.load(Ordering::Acquire) {
        let cycle_start = Instant::now();

        // Get latest camera frame
        let frame = match store.camera_buffer.pop_latest() {
            Some(f) => f,
            None => {
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }
        };

        // Run detection
        let detections = detector.detect(&frame.data, frame.width, frame.height);

        // Store detections for the aggregator
        store.latest_detections.store(detections);

        cycle += 1;
        if cycle.is_multiple_of(75) {
            // every ~5 seconds
            debug!(
                "Perception cycle {}: {} detections",
                cycle,
                store.latest_detections.load().map_or(0, |d| d.len())
            );
        }

        let elapsed = cycle_start.elapsed();
        if elapsed < interval {
            std::thread::sleep(interval - elapsed);
        }
    }

    info!("Perception thread stopped");
}
