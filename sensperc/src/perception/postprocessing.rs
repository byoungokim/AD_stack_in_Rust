/// YOLO output postprocessing: bounding box decoding and NMS.
///
/// Converts raw YOLO model output (tensor of [batch, num_boxes, 4+num_classes])
/// into structured detections with Non-Maximum Suppression.

/// A raw detection from the YOLO model.
#[derive(Clone, Debug)]
pub struct RawDetection {
    pub x_center: f32,
    pub y_center: f32,
    pub width: f32,
    pub height: f32,
    pub class_id: usize,
    pub confidence: f32,
}

/// A final detection after NMS.
#[derive(Clone, Debug)]
pub struct Detection {
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
    pub class_id: usize,
    pub confidence: f32,
}

/// Object class names matching our proto ObjectClass enum.
// Used for human-readable logging once the ONNX detector path is enabled.
#[allow(dead_code)]
pub const CLASS_NAMES: &[&str] = &[
    "person",       // 0 → OBJECT_PERSON (1)
    "vehicle",      // 1 → OBJECT_VEHICLE (2)
    "bicycle",      // 2 → OBJECT_BICYCLE (3)
    "cone",         // 3 → OBJECT_CONE (4)
    "sign",         // 4 → OBJECT_SIGN (5)
    "obstacle",     // 5 → OBJECT_OBSTACLE (6)
    "stop_sign",    // 6 → OBJECT_STOP_SIGN (7)
    "yield_sign",   // 7 → OBJECT_YIELD_SIGN (8)
    "speed_limit",  // 8 → OBJECT_SPEED_LIMIT (9)
];

/// Map YOLO class index to proto ObjectClass value.
/// COCO pretrained models use different indices; this mapping
/// converts common COCO classes to our classes.
pub fn coco_to_object_class(coco_id: usize) -> usize {
    match coco_id {
        0 => 1,   // person → OBJECT_PERSON
        1 => 2,   // bicycle → OBJECT_BICYCLE (COCO) — we map to our bicycle
        2 => 2,   // car → OBJECT_VEHICLE
        3 => 2,   // motorcycle → OBJECT_VEHICLE
        5 => 2,   // bus → OBJECT_VEHICLE
        7 => 2,   // truck → OBJECT_VEHICLE
        11 => 7,  // stop sign → OBJECT_STOP_SIGN
        _ => 6,   // everything else → OBJECT_OBSTACLE
    }
}

/// Decode YOLO output tensor into raw detections.
///
/// YOLO v8 output format: [1, num_classes+4, num_boxes]
/// Transposed: [1, num_boxes, num_classes+4]
/// Each box: [x_center, y_center, width, height, class_scores...]
// Only invoked from the `onnx` feature gated detector path.
#[allow(dead_code)]
pub fn decode_yolo_output(
    output: &[f32],
    num_boxes: usize,
    num_classes: usize,
    confidence_threshold: f32,
    scale_x: f32,
    scale_y: f32,
) -> Vec<RawDetection> {
    let stride = 4 + num_classes;
    let mut detections = Vec::new();

    for i in 0..num_boxes {
        let offset = i * stride;
        if offset + stride > output.len() {
            break;
        }

        let x_center = output[offset] * scale_x;
        let y_center = output[offset + 1] * scale_y;
        let width = output[offset + 2] * scale_x;
        let height = output[offset + 3] * scale_y;

        // Find max class score
        let mut max_score = 0.0f32;
        let mut max_class = 0;
        for c in 0..num_classes {
            let score = output[offset + 4 + c];
            if score > max_score {
                max_score = score;
                max_class = c;
            }
        }

        if max_score >= confidence_threshold {
            detections.push(RawDetection {
                x_center, y_center, width, height,
                class_id: max_class,
                confidence: max_score,
            });
        }
    }

    detections
}

/// Apply Non-Maximum Suppression to remove overlapping detections.
pub fn nms(detections: &[RawDetection], iou_threshold: f32) -> Vec<Detection> {
    if detections.is_empty() {
        return vec![];
    }

    // Convert to corner format and sort by confidence (descending)
    let mut boxes: Vec<Detection> = detections.iter().map(|d| {
        let x1 = d.x_center - d.width / 2.0;
        let y1 = d.y_center - d.height / 2.0;
        let x2 = d.x_center + d.width / 2.0;
        let y2 = d.y_center + d.height / 2.0;
        Detection {
            x1, y1, x2, y2,
            class_id: d.class_id,
            confidence: d.confidence,
        }
    }).collect();

    boxes.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());

    let mut keep = Vec::new();
    let mut suppressed = vec![false; boxes.len()];

    for i in 0..boxes.len() {
        if suppressed[i] {
            continue;
        }
        keep.push(boxes[i].clone());

        for j in (i + 1)..boxes.len() {
            if suppressed[j] {
                continue;
            }
            if boxes[i].class_id == boxes[j].class_id {
                let iou = compute_iou(&boxes[i], &boxes[j]);
                if iou > iou_threshold {
                    suppressed[j] = true;
                }
            }
        }
    }

    keep
}

fn compute_iou(a: &Detection, b: &Detection) -> f32 {
    let x1 = a.x1.max(b.x1);
    let y1 = a.y1.max(b.y1);
    let x2 = a.x2.min(b.x2);
    let y2 = a.y2.min(b.y2);

    let intersection = (x2 - x1).max(0.0) * (y2 - y1).max(0.0);
    let area_a = (a.x2 - a.x1) * (a.y2 - a.y1);
    let area_b = (b.x2 - b.x1) * (b.y2 - b.y1);
    let union = area_a + area_b - intersection;

    if union > 0.0 { intersection / union } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nms_removes_overlapping() {
        let dets = vec![
            RawDetection { x_center: 100.0, y_center: 100.0, width: 50.0, height: 50.0, class_id: 0, confidence: 0.9 },
            RawDetection { x_center: 105.0, y_center: 105.0, width: 50.0, height: 50.0, class_id: 0, confidence: 0.8 },
            RawDetection { x_center: 300.0, y_center: 300.0, width: 50.0, height: 50.0, class_id: 0, confidence: 0.7 },
        ];

        let result = nms(&dets, 0.5);
        assert_eq!(result.len(), 2); // first two overlap, third is separate
        assert!((result[0].confidence - 0.9).abs() < 0.01); // highest confidence kept
    }

    #[test]
    fn test_nms_different_classes() {
        let dets = vec![
            RawDetection { x_center: 100.0, y_center: 100.0, width: 50.0, height: 50.0, class_id: 0, confidence: 0.9 },
            RawDetection { x_center: 105.0, y_center: 105.0, width: 50.0, height: 50.0, class_id: 1, confidence: 0.8 },
        ];

        let result = nms(&dets, 0.5);
        assert_eq!(result.len(), 2); // different classes, both kept
    }

    #[test]
    fn test_iou_identical() {
        let a = Detection { x1: 0.0, y1: 0.0, x2: 10.0, y2: 10.0, class_id: 0, confidence: 1.0 };
        assert!((compute_iou(&a, &a) - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_iou_no_overlap() {
        let a = Detection { x1: 0.0, y1: 0.0, x2: 10.0, y2: 10.0, class_id: 0, confidence: 1.0 };
        let b = Detection { x1: 20.0, y1: 20.0, x2: 30.0, y2: 30.0, class_id: 0, confidence: 1.0 };
        assert!((compute_iou(&a, &b)).abs() < 0.01);
    }

    #[test]
    fn test_coco_mapping() {
        assert_eq!(coco_to_object_class(0), 1);  // person
        assert_eq!(coco_to_object_class(2), 2);  // car → vehicle
        assert_eq!(coco_to_object_class(11), 7); // stop sign
        assert_eq!(coco_to_object_class(99), 6); // unknown → obstacle
    }
}
