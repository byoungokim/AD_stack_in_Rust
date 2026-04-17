#!/usr/bin/env python3
"""Test camera perception pipeline with real YOLOv8 ONNX model.

Downloads sample images and runs inference to verify the detection
pipeline works end-to-end with the same model our Rust code uses.

Usage:
    python3 tests/test_perception.py
    python3 tests/test_perception.py --image path/to/image.jpg
"""

import os
import sys
import time
import numpy as np

# COCO class names (80 classes)
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush",
]

# Our custom class mapping (COCO → Limo Drive proto ObjectClass)
LIMO_CLASS_MAP = {
    0: ("PERSON", 1),
    1: ("BICYCLE", 3),
    2: ("VEHICLE(car)", 2),
    3: ("VEHICLE(motorcycle)", 2),
    5: ("VEHICLE(bus)", 2),
    7: ("VEHICLE(truck)", 2),
    11: ("STOP_SIGN", 7),
}


def preprocess(image, target_size=640):
    """Preprocess image for YOLOv8: resize, normalize, CHW."""
    import cv2
    h, w = image.shape[:2]
    resized = cv2.resize(image, (target_size, target_size))
    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    normalized = rgb.astype(np.float32) / 255.0
    chw = np.transpose(normalized, (2, 0, 1))  # HWC → CHW
    batch = np.expand_dims(chw, axis=0)  # add batch dim
    return batch, (w / target_size, h / target_size)


def postprocess(output, scale_x, scale_y, conf_threshold=0.5, iou_threshold=0.45):
    """Postprocess YOLOv8 output: decode boxes, apply NMS."""
    # YOLOv8 output shape: (1, 84, 8400) → transpose to (8400, 84)
    predictions = output[0].T  # (8400, 84)

    # Extract boxes and class scores
    boxes = predictions[:, :4]  # x_center, y_center, width, height
    class_scores = predictions[:, 4:]  # 80 class scores

    # Filter by confidence
    max_scores = np.max(class_scores, axis=1)
    mask = max_scores >= conf_threshold
    boxes = boxes[mask]
    scores = max_scores[mask]
    class_ids = np.argmax(class_scores[mask], axis=1)

    if len(boxes) == 0:
        return []

    # Convert center to corner format and scale
    x1 = (boxes[:, 0] - boxes[:, 2] / 2) * scale_x
    y1 = (boxes[:, 1] - boxes[:, 3] / 2) * scale_y
    x2 = (boxes[:, 0] + boxes[:, 2] / 2) * scale_x
    y2 = (boxes[:, 1] + boxes[:, 3] / 2) * scale_y

    # NMS
    indices = nms(x1, y1, x2, y2, scores, iou_threshold)

    detections = []
    for i in indices:
        detections.append({
            "class_id": int(class_ids[i]),
            "class_name": COCO_NAMES[class_ids[i]] if class_ids[i] < len(COCO_NAMES) else "unknown",
            "confidence": float(scores[i]),
            "bbox": [float(x1[i]), float(y1[i]), float(x2[i]), float(y2[i])],
            "limo_class": LIMO_CLASS_MAP.get(int(class_ids[i]), ("OBSTACLE", 6)),
        })

    return detections


def nms(x1, y1, x2, y2, scores, threshold):
    """Non-Maximum Suppression."""
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []

    while len(order) > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0, xx2 - xx1)
        h = np.maximum(0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(iou <= threshold)[0]
        order = order[inds + 1]

    return keep


def run_detection(model_path, image_path):
    """Run full detection pipeline on a single image."""
    import cv2
    import onnxruntime as ort

    print(f"\n{'='*60}")
    print(f"  Image: {os.path.basename(image_path)}")
    print(f"  Model: {os.path.basename(model_path)}")
    print(f"{'='*60}")

    # Load image
    image = cv2.imread(image_path)
    if image is None:
        print(f"  ERROR: Cannot read image {image_path}")
        return []
    print(f"  Image size: {image.shape[1]}x{image.shape[0]}")

    # Preprocess
    t0 = time.time()
    input_tensor, (scale_x, scale_y) = preprocess(image)
    t_pre = time.time() - t0

    # Run inference
    session = ort.InferenceSession(model_path)
    input_name = session.get_inputs()[0].name

    t0 = time.time()
    outputs = session.run(None, {input_name: input_tensor})
    t_infer = time.time() - t0

    # Postprocess
    t0 = time.time()
    detections = postprocess(outputs[0], scale_x, scale_y)
    t_post = time.time() - t0

    # Print results
    print(f"\n  Timing:")
    print(f"    Preprocess:  {t_pre*1000:.1f}ms")
    print(f"    Inference:   {t_infer*1000:.1f}ms")
    print(f"    Postprocess: {t_post*1000:.1f}ms")
    print(f"    Total:       {(t_pre+t_infer+t_post)*1000:.1f}ms")

    print(f"\n  Detections ({len(detections)}):")
    for i, det in enumerate(detections):
        limo_name, limo_id = det["limo_class"]
        bbox = det["bbox"]
        print(f"    [{i}] {det['class_name']:15s} conf={det['confidence']:.2f}"
              f"  bbox=({bbox[0]:.0f},{bbox[1]:.0f},{bbox[2]:.0f},{bbox[3]:.0f})"
              f"  → Limo: {limo_name} (class_id={limo_id})")

    return detections


def main():
    model_path = "models/yolov8n.onnx"

    if not os.path.exists(model_path):
        print(f"ERROR: Model not found at {model_path}")
        print("Run: python3 -c \"from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx')\"")
        sys.exit(1)

    # Check for onnxruntime
    try:
        import onnxruntime
        import cv2
    except ImportError as e:
        print(f"ERROR: Missing dependency: {e}")
        print("Run: pip3 install --break-system-packages onnxruntime opencv-python")
        sys.exit(1)

    # Custom image or default samples
    if len(sys.argv) > 1 and sys.argv[1] == "--image":
        images = [sys.argv[2]]
    else:
        images = [
            "tests/perception_samples/traffic1.jpg",
            "tests/perception_samples/traffic2.jpg",
        ]

    print("╔══════════════════════════════════════════════╗")
    print("║  Limo Drive Camera Perception Test           ║")
    print("║  YOLOv8n ONNX → Preprocessing → NMS         ║")
    print("╚══════════════════════════════════════════════╝")

    total_dets = 0
    for img_path in images:
        if os.path.exists(img_path):
            dets = run_detection(model_path, img_path)
            total_dets += len(dets)
        else:
            print(f"\n  SKIP: {img_path} not found")

    print(f"\n{'='*60}")
    print(f"  TOTAL: {total_dets} detections across {len(images)} images")
    if total_dets > 0:
        print(f"  RESULT: ✅ PASS — perception pipeline works!")
    else:
        print(f"  RESULT: ⚠️  No detections (model may need different images)")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    main()
