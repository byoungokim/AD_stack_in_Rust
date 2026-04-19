/// YOLO Object Detector using ONNX Runtime.
///
/// Loads a YOLO .onnx model and runs inference on camera frames.
/// Falls back to a no-op detector if model is unavailable.
use anyhow::Result;


const DEFAULT_INPUT_SIZE: u32 = 640;
const DEFAULT_CONFIDENCE: f32 = 0.5;
const DEFAULT_NMS_THRESHOLD: f32 = 0.45;

/// Object detector wrapping ONNX Runtime inference.
// Inference fields are only read in the `onnx` feature path; kept for fallback parity.
#[allow(dead_code)]
pub struct ObjectDetector {
    mode: DetectorMode,
    input_size: u32,
    confidence_threshold: f32,
    nms_threshold: f32,
    num_classes: usize,
}

enum DetectorMode {
    /// ONNX Runtime session (when ort crate + model available)
    #[cfg(feature = "onnx")]
    Onnx(ort::Session),
    /// Fallback: no model, returns empty detections
    Fallback,
}

impl ObjectDetector {
    /// Try to load an ONNX model. Returns error if file doesn't exist.
    pub fn new(model_path: &str) -> Result<Self> {
        // Check if model file exists
        if !std::path::Path::new(model_path).exists() {
            anyhow::bail!("Model file not found: {}", model_path);
        }

        #[cfg(feature = "onnx")]
        {
            let session = ort::Session::builder()?
                .with_optimization_level(ort::GraphOptimizationLevel::Level3)?
                .commit_from_file(model_path)
                .context("Failed to load ONNX model")?;

            info!("ONNX model loaded: {}", model_path);
            return Ok(Self {
                mode: DetectorMode::Onnx(session),
                input_size: DEFAULT_INPUT_SIZE,
                confidence_threshold: DEFAULT_CONFIDENCE,
                nms_threshold: DEFAULT_NMS_THRESHOLD,
                num_classes: 80, // COCO classes
            });
        }

        #[cfg(not(feature = "onnx"))]
        {
            anyhow::bail!("ONNX feature not enabled. Build with --features onnx");
        }
    }

    /// Create a fallback detector that returns empty results.
    pub fn fallback() -> Self {
        Self {
            mode: DetectorMode::Fallback,
            input_size: DEFAULT_INPUT_SIZE,
            confidence_threshold: DEFAULT_CONFIDENCE,
            nms_threshold: DEFAULT_NMS_THRESHOLD,
            num_classes: 80,
        }
    }

    /// Run detection on a camera frame.
    ///
    /// Input: raw BGR8 image bytes, width, height
    /// Output: list of detections
    pub fn detect(&mut self, _image_data: &[u8], _width: u32, _height: u32) -> Vec<CameraDetection> {
        match &self.mode {
            #[cfg(feature = "onnx")]
            DetectorMode::Onnx(session) => {
                self.detect_onnx(session, image_data, width, height)
            }
            DetectorMode::Fallback => {
                // No model — return empty
                vec![]
            }
        }
    }

    /// Run ONNX inference.
    #[cfg(feature = "onnx")]
    fn detect_onnx(
        &self,
        session: &ort::Session,
        image_data: &[u8],
        width: u32,
        height: u32,
    ) -> Vec<CameraDetection> {
        use ndarray::Array4;

        // Preprocess
        let tensor_data = preprocess_image(image_data, width, height, self.input_size);
        let input = Array4::from_shape_vec(
            (1, 3, self.input_size as usize, self.input_size as usize),
            tensor_data,
        );

        let input = match input {
            Ok(arr) => arr,
            Err(_) => return vec![],
        };

        // Run inference
        let outputs = match session.run(ort::inputs![input].unwrap()) {
            Ok(o) => o,
            Err(e) => {
                debug!("ONNX inference error: {}", e);
                return vec![];
            }
        };

        // Parse output
        let output_tensor = match outputs[0].try_extract_tensor::<f32>() {
            Ok(t) => t,
            Err(_) => return vec![],
        };

        let output_data = output_tensor.view();
        let shape = output_data.shape();
        let num_boxes = if shape.len() >= 2 { shape[1] } else { 0 };

        let (scale_x, scale_y) = compute_scale(width, height, self.input_size);

        let raw = decode_yolo_output(
            output_data.as_slice().unwrap_or(&[]),
            num_boxes,
            self.num_classes,
            self.confidence_threshold,
            scale_x,
            scale_y,
        );

        let filtered = nms(&raw, self.nms_threshold);

        // Convert to CameraDetection
        filtered.into_iter().map(|d| CameraDetection {
            x1: d.x1,
            y1: d.y1,
            x2: d.x2,
            y2: d.y2,
            class_id: coco_to_object_class(d.class_id),
            confidence: d.confidence,
        }).collect()
    }
}

/// Detection result from camera perception.
#[derive(Clone, Debug)]
pub struct CameraDetection {
    pub x1: f32,          // bounding box left
    pub y1: f32,          // bounding box top
    pub x2: f32,          // bounding box right
    pub y2: f32,          // bounding box bottom
    pub class_id: usize,  // proto ObjectClass value
    pub confidence: f32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fallback_detector() {
        let mut det = ObjectDetector::fallback();
        let data = vec![128u8; 640 * 480 * 3];
        let results = det.detect(&data, 640, 480);
        assert!(results.is_empty()); // fallback returns nothing
    }

    #[test]
    fn test_model_not_found() {
        let result = ObjectDetector::new("nonexistent.onnx");
        assert!(result.is_err());
    }
}
