#!/usr/bin/env python3
"""
ROS 2 Node: Vision Object Detection (AI YOLOv8 + Robotics Color Block Tracking)
Subscribes to `/camera/image_raw` and provides clean, collision-free real-time detection:
  - Mode 'all'   : Hybrid detection (manipulable items + yellow/red/green/blue cubes)
  - Mode 'block' : Pure high-precision color cube tracking for myCobot pick & place
  - Mode 'ai'    : 80 COCO classes via YOLOv8 (cups, bottles, phones, scissors, etc.)

Features solidity and strict HSV filtering to eliminate false skin detections on hands/fingers,
plus background filtering to remove visual clutter from persons and monitors.

Topics Published:
  - /camera/detected_objects             (sensor_msgs/msg/Image)
  - /camera/detected_objects/compressed  (sensor_msgs/msg/CompressedImage)
  - /camera/detections_json              (std_msgs/msg/String)
"""

import json
import os
import time
import urllib.request
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header, String

# 80 COCO Classes
COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
    'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
    'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
]

# Background classes to filter out for clean robotics manipulation
BACKGROUND_CLASSES = {
    'person', 'tv', 'laptop', 'chair', 'couch', 'bed',
    'dining table', 'toilet', 'refrigerator', 'potted plant',
    'sink', 'bench', 'traffic light', 'stop sign'
}

# Calibrated HSV ranges for myCobot wooden colored cubes
# Uses high saturation lower bound and RGB ratio validation to prevent skin tones from ever triggering "red cube"
COLOR_BLOCK_RANGES = {
    'yellow cube': {
        'lower': np.array([16, 90, 80], dtype=np.uint8),
        'upper': np.array([35, 255, 255], dtype=np.uint8),
        'bgr': (0, 235, 255)   # Bright Yellow
    },
    'red cube': {
        # Strict S >= 165 threshold: Human skin has S < 140, painted red cubes have S >= 170
        'lower1': np.array([0, 165, 80], dtype=np.uint8),
        'upper1': np.array([7, 255, 255], dtype=np.uint8),
        'lower2': np.array([173, 165, 80], dtype=np.uint8),
        'upper2': np.array([180, 255, 255], dtype=np.uint8),
        'bgr': (0, 0, 255)     # Bright Red
    },
    'green cube': {
        'lower': np.array([40, 95, 50], dtype=np.uint8),
        'upper': np.array([85, 255, 255], dtype=np.uint8),
        'bgr': (0, 255, 0)     # Bright Green
    },
    'blue cube': {
        'lower': np.array([96, 95, 50], dtype=np.uint8),
        'upper': np.array([130, 255, 255], dtype=np.uint8),
        'bgr': (255, 100, 0)   # Bright Blue
    },
    'wood cube': {
        'lower': np.array([10, 65, 65], dtype=np.uint8),
        'upper': np.array([18, 145, 215], dtype=np.uint8),
        'bgr': (50, 140, 210)   # Warm Wood Tan/Brown
    }
}

# Distinct palette for AI classes
np.random.seed(42)
AI_COLORS = np.random.randint(50, 255, size=(len(COCO_CLASSES), 3), dtype=np.uint8).tolist()


def calculate_iou(boxA, boxB):
    xA = max(boxA[0], boxB[0])
    yA = max(boxA[1], boxB[1])
    xB = min(boxA[2], boxB[2])
    yB = min(boxA[3], boxB[3])
    interW = max(0, xB - xA)
    interH = max(0, yB - yA)
    interArea = interW * interH
    if interArea == 0:
        return 0.0
    boxAArea = (boxA[2] - boxA[0]) * (boxA[3] - boxA[1])
    boxBArea = (boxB[2] - boxB[0]) * (boxB[3] - boxB[1])
    return interArea / float(boxAArea + boxBArea - interArea)


class DetectObjectsNode(Node):
    def __init__(self):
        super().__init__('detect_objects')

        self.declare_parameter('mode', 'ai')  # 'ai' (original YOLOv8), 'block' (color cubes), or 'all' (hybrid)
        self.declare_parameter('conf_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('min_block_area', 600)
        self.declare_parameter('filter_background', False)
        self.declare_parameter('model_path', '/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/models/yolov8n.onnx')

        self.mode = str(self.get_parameter('mode').value).lower()
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.min_block_area = int(self.get_parameter('min_block_area').value)
        self.filter_background = bool(self.get_parameter('filter_background').value)
        self.model_path = str(self.get_parameter('model_path').value)

        self.get_logger().info(f"Initializing Object Detection Node [MODE: {self.mode.upper()}]...")

        # 1. Initialize YOLO Model if needed
        self.session = None
        if self.mode in ('ai', 'all'):
            self.ensure_model()
            import onnxruntime as ort
            opts = ort.SessionOptions()
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            opts.intra_op_num_threads = 4
            self.session = ort.InferenceSession(self.model_path, opts, providers=['CPUExecutionProvider'])
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info(f"[+] YOLOv8 Model loaded successfully from {self.model_path}!")

        # 2. Subscribers and Publishers
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        self.image_pub = self.create_publisher(Image, '/camera/detected_objects', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, '/camera/detected_objects/compressed', 10)
        self.json_pub = self.create_publisher(String, '/camera/detections_json', 10)

        self.busy = False
        self.fps = 0.0

    def ensure_model(self):
        if not os.path.exists(self.model_path):
            os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
            url = "https://github.com/ultralytics/assets/releases/download/v8.4.0/yolov8n.onnx"
            self.get_logger().info(f"[*] Model not found locally. Downloading YOLOv8n ONNX (~12MB) from {url}...")
            urllib.request.urlretrieve(url, self.model_path)
            self.get_logger().info("[+] Download complete!")

    def detect_color_blocks(self, frame, hsv):
        """
        Detect colored toy cubes (Yellow, Red, Green, Blue) using strict HSV and solidity geometry.
        Fingers and skin are filtered out by high saturation and solidity requirements.
        """
        raw_boxes = []
        raw_confs = []
        raw_data = []
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        for color_name, params in COLOR_BLOCK_RANGES.items():
            if 'lower1' in params:
                m1 = cv2.inRange(hsv, params['lower1'], params['upper1'])
                m2 = cv2.inRange(hsv, params['lower2'], params['upper2'])
                mask = cv2.bitwise_or(m1, m2)
            else:
                mask = cv2.inRange(hsv, params['lower'], params['upper'])

            # Clean speckle noise
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_block_area:
                    continue

                x, y, bw, bh = cv2.boundingRect(c)
                aspect_ratio = float(bw) / float(bh)

                # Cubes/blocks must be roughly square (aspect ratio between 0.58 and 1.68)
                # Fingers/hands are long and narrow and get rejected here
                if aspect_ratio < 0.58 or aspect_ratio > 1.68:
                    continue

                # Solidity check: a real solid block fills its bounding box (area / (bw * bh) >= 0.55)
                # Curved finger silhouettes and gaps have much lower solidity (< 0.45)
                solidity = float(area) / float(bw * bh)
                if solidity < 0.55:
                    continue

                # Triple Skin-Protection check for 'red cube':
                # Painted red wooden blocks have vibrant red (R > 130) with very low G and B.
                # Human skin always has high Green undertones (G >= 70, R - G < 65).
                if color_name == 'red cube':
                    roi = frame[y:y+bh, x:x+bw]
                    mean_b = float(np.mean(roi[:, :, 0]))
                    mean_g = float(np.mean(roi[:, :, 1]))
                    mean_r = float(np.mean(roi[:, :, 2]))
                    if (mean_r - mean_g < 65) or (mean_r - mean_b < 65) or (mean_g > 115):
                        continue

                cx = int(x + bw / 2)
                cy = int(y + bh / 2)
                conf = min(0.99, round(0.85 + (area / 15000.0) * 0.14, 2))

                raw_boxes.append([x, y, bw, bh])
                raw_confs.append(conf)
                raw_data.append({
                    "label": color_name,
                    "confidence": conf,
                    "box": [x, y, x + bw, y + bh],
                    "center": [cx, cy],
                    "color_bgr": params['bgr'],
                    "is_block": True
                })

        # Non-Maximum Suppression to collapse duplicate/overlapping block detections
        block_detections = []
        if raw_boxes:
            nms_idx = cv2.dnn.NMSBoxes(raw_boxes, raw_confs, 0.4, 0.35)
            if len(nms_idx) > 0:
                for idx in np.array(nms_idx).flatten():
                    block_detections.append(raw_data[idx])

        return block_detections

    def detect_ai_objects(self, frame, h, w):
        """Run YOLOv8 inference for everyday manipulable objects."""
        if self.session is None:
            return []

        input_size = 640
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (input_size, input_size), swapRB=True)
        outputs = self.session.run(None, {self.input_name: blob})[0]
        preds = np.squeeze(outputs).T

        boxes, confidences, class_ids = [], [], []
        x_scale = w / float(input_size)
        y_scale = h / float(input_size)

        for row in preds:
            scores = row[4:]
            class_id = int(np.argmax(scores))
            score = float(scores[class_id])
            if score >= self.conf_threshold:
                cx, cy, bw, bh = row[:4]
                x1 = int((cx - bw / 2) * x_scale)
                y1 = int((cy - bh / 2) * y_scale)
                bw = int(bw * x_scale)
                bh = int(bh * y_scale)
                boxes.append([x1, y1, bw, bh])
                confidences.append(score)
                class_ids.append(class_id)

        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.iou_threshold)
        ai_detections = []

        if len(indices) > 0:
            indices = np.array(indices).flatten()
            for i in indices:
                x, y, bw, bh = boxes[i]
                cls_id = class_ids[i]
                conf = confidences[i]
                label = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else f"id_{cls_id}"

                # In hybrid 'all' mode, optionally filter background if requested
                if self.mode == 'all' and self.filter_background and label in BACKGROUND_CLASSES:
                    continue

                x1 = max(0, x)
                y1 = max(0, y)
                x2 = min(w, x + bw)
                y2 = min(h, y + bh)
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                ai_detections.append({
                    "label": label,
                    "confidence": round(conf, 2),
                    "box": [x1, y1, x2, y2],
                    "center": [cx, cy],
                    "color_bgr": AI_COLORS[cls_id % len(AI_COLORS)],
                    "is_block": False
                })

        return ai_detections

    def image_callback(self, msg: Image):
        if self.busy or not rclpy.ok():
            return
        now = time.time()
        # Cap detection at ~15 FPS to reduce CPU load
        if now - getattr(self, "last_detect_time", 0.0) < 0.066:
            return
        self.last_detect_time = now
        self.busy = True

        try:
            t_start = time.time()
            h, w = msg.height, msg.width
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3)).copy()

            all_detections = []

            # 1. Color Block Detection (Yellow, Red, Green, Blue)
            if self.mode in ('block', 'all'):
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                blocks = self.detect_color_blocks(frame, hsv)
                all_detections.extend(blocks)

            # 2. AI YOLO Detection (Everyday manipulable objects)
            if self.mode in ('ai', 'all'):
                ai_objs = self.detect_ai_objects(frame, h, w)

                # Suppress any AI detections that overlap with detected blocks (e.g. if YOLO confuses a block with a phone, book, etc.)
                if self.mode == 'all':
                    filtered_ai = []
                    for ai_obj in ai_objs:
                        suppress = False
                        for block in all_detections:
                            iou = calculate_iou(ai_obj['box'], block['box'])
                            if iou > 0.15:
                                suppress = True
                                break
                        if not suppress:
                            filtered_ai.append(ai_obj)
                    all_detections.extend(filtered_ai)
                else:
                    all_detections.extend(ai_objs)

            # Global Non-Maximum Suppression to eliminate colliding/overlapping boxes
            if len(all_detections) > 1:
                nms_boxes = []
                nms_confs = []
                for d in all_detections:
                    bx1, by1, bx2, by2 = d['box']
                    nms_boxes.append([bx1, by1, max(1, bx2 - bx1), max(1, by2 - by1)])
                    nms_confs.append(float(d['confidence']))
                nms_indices = cv2.dnn.NMSBoxes(nms_boxes, nms_confs, 0.35, 0.35)
                if len(nms_indices) > 0:
                    keep_idx = set(np.array(nms_indices).flatten())
                    all_detections = [all_detections[i] for i in range(len(all_detections)) if i in keep_idx]

            # 3. Draw Detections
            json_detections = []
            for det in all_detections:
                x1, y1, x2, y2 = det['box']
                cx, cy = det['center']
                color = det['color_bgr']
                label = det['label']
                conf = det['confidence']

                json_detections.append({
                    "label": label,
                    "confidence": conf,
                    "box": [x1, y1, x2, y2],
                    "center": [cx, cy]
                })

                thickness = 3 if det.get('is_block', False) else 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

                # Centroid Crosshair
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1)
                cv2.drawMarker(frame, (cx, cy), (0, 0, 0), cv2.MARKER_CROSS, 12, 1)

                # Text Banner
                text = f"{label.upper()} {int(conf * 100)}%"
                (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
                cv2.rectangle(frame, (x1, max(0, y1 - th - 8)), (x1 + tw + 6, y1), color, -1)
                text_color = (0, 0, 0) if det.get('is_block', False) and 'yellow' in label else (255, 255, 255)
                cv2.putText(frame, text, (x1 + 3, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, text_color, 1, cv2.LINE_AA)

            # Performance stats
            dt = time.time() - t_start
            fps_val = 1.0 / dt if dt > 0 else 0.0
            self.fps = 0.85 * self.fps + 0.15 * fps_val

            # Top Header Overlay
            header_text = f"Vision [{self.mode.upper()}] | {self.fps:.1f} FPS ({int(dt*1000)}ms) | Detections: {len(all_detections)}"
            cv2.rectangle(frame, (0, 0), (w, 28), (15, 23, 42), -1)
            cv2.putText(frame, header_text, (10, 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (56, 189, 248), 1, cv2.LINE_AA)

            now_stamp = self.get_clock().now().to_msg()

            # 4. Publish Annotated Image
            out_img = Image()
            out_img.header = Header()
            out_img.header.stamp = now_stamp
            out_img.header.frame_id = msg.header.frame_id
            out_img.height = h
            out_img.width = w
            out_img.encoding = 'bgr8'
            out_img.is_bigendian = 0
            out_img.step = w * 3
            out_img.data = frame.tobytes()
            self.image_pub.publish(out_img)

            # 5. Publish Compressed Image only if subscribers are listening
            if self.compressed_pub.get_subscription_count() > 0:
                ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ret:
                    comp_msg = CompressedImage()
                    comp_msg.header = Header()
                    comp_msg.header.stamp = now_stamp
                    comp_msg.header.frame_id = msg.header.frame_id
                    comp_msg.format = 'jpeg'
                    comp_msg.data = jpeg.tobytes()
                    self.compressed_pub.publish(comp_msg)

            # 6. Publish JSON Metadata
            json_msg = String()
            json_msg.data = json.dumps({"timestamp": time.time(), "detections": json_detections})
            self.json_pub.publish(json_msg)

        except Exception as e:
            self.get_logger().error(f"Error in vision callback: {e}")
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = DetectObjectsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
