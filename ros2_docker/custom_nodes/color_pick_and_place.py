#!/usr/bin/env python3
"""
ROS 2 Node: Color Cube Pick & Place with Interactive Teaching & Custom Points
For myCobot 280 M5 with Camera and 7th Joint Claw (Adaptive Gripper).

Features:
  - 1-CLICK CAMERA CALIBRATION ([k]):
    Jog the arm above any cube on your table and press [k].
    Instantly aligns camera pixels to physical robot coordinates!
  - TEACH CUSTOM PLACE POINTS ([1], [2], [3], [4]):
    Move the robot to your custom container/spot for RED, GREEN, BLUE, or YELLOW,
    press the key, and it permanently saves that custom drop position!
  - FIXED PICK SPOT MODE ([s] & [f]):
    If you place cubes at a fixed spot on the table, save it with [s] and toggle
    Fixed Mode with [f]. The camera detects the color, and the arm picks from your spot!
  - ACTIVE SELF-COLLISION GUARD:
    Prevents the claw from ever crossing into arm joints or folding backwards.
  - PERSISTENT CONFIGURATION:
    All custom positions and calibration are auto-saved to 'custom_positions.json'.
  - PS5 DUALSENSE INTEGRATION:
    Square (⏹) triggers autonomous pick & place.
"""

import json
import math
import os
import select
import sys
import threading
import time
import cv2
import numpy as np

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Point
from mycobot_interfaces.srv import (
    GetAngles,
    GetCoords,
    GripperStatus,
    SetAngles,
    SetCoords,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, ColorRGBA, Header, String
from visualization_msgs.msg import Marker

CONFIG_FILE_PATH = "/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/custom_positions.json"
HOST_BACKUP_PATH = "/tmp/custom_positions.json"

# ---------------------------------------------------------
# Default Safe Poses & Coordinates
# ---------------------------------------------------------
# High observation / standby pose (claw elevated, clear of camera view)
DEFAULT_OBSERVE_ANGLES = [-7.11, -6.94, -55.01, -24.16, 0.0, -15.0]

# Pre-grab intermediate pose: guarantees the arm enters an elbow-up forward reach branch
PRE_GRAB_ANGLES = [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]

# Default verified drop bin angles (clearance > 175mm to all links)
DEFAULT_BIN_ANGLES = {
    'red': [-24.87, -2.98, -92.46, 5.88, -3.07, -8.34],     # Custom Place Point 1
    'green': [-13.71, -52.11, -25.4, -4.57, -3.86, -7.73],   # Custom Place Point 2
    'blue': [74.0, -18.1, -64.24, -9.84, -0.79, -9.49],     # Custom Place Point 3
    'yellow': [112.93, 3.16, -96.32, 0.87, 0.26, -9.75],    # Custom Place Point 4
}

# Strict Physical Safe Workspace Envelope for myCobot 280 (Prevents singularity over-extension)
SAFE_WORKSPACE = {
    'x_min': 80.0,       # mm: minimum forward reach (clears robot base)
    'x_max': 250.0,      # mm: maximum forward reach (keeps downward wrist reachable)
    'y_min': -145.0,     # mm: right limit
    'y_max': 145.0,      # mm: left limit
    'max_radius': 260.0, # mm: maximum reach radius sqrt(x^2 + y^2)
    'min_radius': 80.0,  # mm: minimum radius around base
    'z_min': 85.0,       # mm: absolute floor limit to protect table surface
    'z_hover': 185.0,    # mm: approach / transit height
    'z_lift': 215.0,     # mm: lift height
}

COLOR_RGBA = {
    'red': ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.9),
    'green': ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9),
    'blue': ColorRGBA(r=0.1, g=0.4, b=1.0, a=0.9),
    'yellow': ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.9),
}

COLOR_HSV_RANGES = {
    'yellow': {
        'lower': np.array([16, 90, 80], dtype=np.uint8),
        'upper': np.array([35, 255, 255], dtype=np.uint8),
    },
    'red': {
        'lower1': np.array([0, 165, 80], dtype=np.uint8),
        'upper1': np.array([7, 255, 255], dtype=np.uint8),
        'lower2': np.array([173, 165, 80], dtype=np.uint8),
        'upper2': np.array([180, 255, 255], dtype=np.uint8),
    },
    'green': {
        'lower': np.array([40, 95, 50], dtype=np.uint8),
        'upper': np.array([85, 255, 255], dtype=np.uint8),
    },
    'blue': {
        'lower': np.array([96, 95, 50], dtype=np.uint8),
        'upper': np.array([130, 255, 255], dtype=np.uint8),
    },
}


class ColorPickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('color_pick_and_place')
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('auto_mode', False)
        self.declare_parameter('target_color', 'all')   # 'all', 'red', 'green', 'blue', 'yellow'
        self.declare_parameter('speed', 30)             # Speed 1-80%
        self.declare_parameter('grip_z', 95.0)          # Grasp height in mm
        self.declare_parameter('use_fixed_pick', False) # Use taught fixed spot instead of camera math

        self.auto_mode = bool(self.get_parameter('auto_mode').value)
        self.target_color = str(self.get_parameter('target_color').value).lower()
        self.speed = int(self.get_parameter('speed').value)
        self.grip_z = float(self.get_parameter('grip_z').value)
        self.use_fixed_pick = bool(self.get_parameter('use_fixed_pick').value)

        # Custom positions & calibration dictionary
        self.calib = {
            "ref_pixel": [320.0, 240.0],
            "ref_robot": [170.0, 0.0],
            "scale_x": 0.25,
            "scale_y": -0.25,
            "fixed_pick_coords": [170.0, 0.0, 95.0],
            "drop_bins": dict(DEFAULT_BIN_ANGLES)
        }
        self.load_custom_positions()

        # Service Clients
        self.set_coords_cli = self.create_client(SetCoords, 'set_coords', callback_group=self.cb_group)
        self.set_angles_cli = self.create_client(SetAngles, 'set_angles', callback_group=self.cb_group)
        self.get_coords_cli = self.create_client(GetCoords, 'get_coords', callback_group=self.cb_group)
        self.get_angles_cli = self.create_client(GetAngles, 'get_angles', callback_group=self.cb_group)
        self.set_gripper_cli = self.create_client(GripperStatus, 'set_gripper', callback_group=self.cb_group)

        # Publishers
        self.status_pub = self.create_publisher(String, '/color_sort/status', 10)
        self.busy_pub = self.create_publisher(Bool, '/color_sort/busy', 10)
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)
        self.safe_area_pub = self.create_publisher(Marker, '/safe_area_marker', 10)

        # Subscribers
        self.json_sub = self.create_subscription(
            String, '/camera/detections_json', self.detections_callback, 10, callback_group=self.cb_group
        )
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 1, callback_group=self.cb_group
        )
        self.trigger_sub = self.create_subscription(
            String, '/color_sort/trigger', self.trigger_callback, 10, callback_group=self.cb_group
        )

        # State Variables
        self.busy = False
        self.latest_detections = []
        self.last_detection_time = 0.0
        self.current_action = "IDLE / OBSERVING"
        self.sorted_count = 0

        # Periodic timer (2Hz) to keep safe area visualization active in RViz
        self.vis_timer = self.create_timer(0.5, self.publish_safe_area_marker, callback_group=self.cb_group)

        self.get_logger().info("=" * 65)
        self.get_logger().info("  myCobot 280 Safe Color Pick & Place (Interactive Teaching Enabled)")
        self.get_logger().info(f"  Target: {self.target_color.upper()} | Grip Z: {self.grip_z}mm")
        self.get_logger().info(f"  Pick Mode: {'FIXED SPOT' if self.use_fixed_pick else 'DYNAMIC CAMERA'}")
        self.get_logger().info("=" * 65)

        self.publish_status("READY")

        # Start CLI & Auto-worker Threads
        self.running = True
        self.worker_thread = threading.Thread(target=self.auto_worker_loop, daemon=True)
        self.worker_thread.start()

        self.cli_thread = threading.Thread(target=self.interactive_cli_loop, daemon=True)
        self.cli_thread.start()

    # ---------------------------------------------------------
    # Persistent Configuration (Load & Save)
    # ---------------------------------------------------------
    def load_custom_positions(self):
        # 1. Load official Elephant Robotics EyesInHand_matrix.json
        for mat_path in [
            "/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/EyesInHand_matrix.json",
            "/tmp/EyesInHand_matrix.json"
        ]:
            if os.path.exists(mat_path):
                try:
                    with open(mat_path, 'r') as f:
                        m_data = json.load(f)
                    self.calib.update(m_data)
                    self.get_logger().info(f"[+] Loaded official Elephant Robotics Hand-Eye Matrix from {mat_path}")
                    break
                except Exception as e:
                    self.get_logger().warn(f"Failed to read EyesInHand_matrix: {e}")

        # 2. Load custom place points and preferences
        path = CONFIG_FILE_PATH if os.path.exists(os.path.dirname(CONFIG_FILE_PATH)) else HOST_BACKUP_PATH
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    data = json.load(f)
                self.calib.update(data)
                self.get_logger().info(f"[+] Loaded custom calibration & place points from {path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to read custom positions: {e}")

    def save_custom_positions(self):
        path = CONFIG_FILE_PATH if os.path.exists(os.path.dirname(CONFIG_FILE_PATH)) else HOST_BACKUP_PATH
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                json.dump(self.calib, f, indent=2)
            # Also save EyesInHand_matrix.json if present
            if "EyesInHand_matrix" in self.calib:
                mat_path = os.path.join(os.path.dirname(path), "EyesInHand_matrix.json")
                with open(mat_path, 'w') as f:
                    json.dump(self.calib, f, indent=2)
                with open("/tmp/EyesInHand_matrix.json", 'w') as f:
                    json.dump(self.calib, f, indent=2)
            self.get_logger().info(f"[✓] Saved custom positions & calibration to {path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to save custom positions: {e}")
            return False

    # ---------------------------------------------------------
    # Safety Validation & Software Interlocks
    # ---------------------------------------------------------
    def is_in_safe_workspace(self, x: float, y: float) -> bool:
        radius = math.hypot(x, y)
        if radius > SAFE_WORKSPACE['max_radius'] or radius < SAFE_WORKSPACE['min_radius']:
            return False
        if not (SAFE_WORKSPACE['x_min'] <= x <= SAFE_WORKSPACE['x_max']):
            return False
        if not (SAFE_WORKSPACE['y_min'] <= y <= SAFE_WORKSPACE['y_max']):
            return False
        return True

    def is_pose_safe(self, angles: list) -> bool:
        if len(angles) < 6:
            return False
        j1, j2, j3, j4, j5, j6 = [float(a) for a in angles[:6]]
        if j3 < -100.0 or j3 > 15.0:
            return False
        if j2 < -35.0 or j2 > 65.0:
            return False
        if (j2 - j3) > 105.0:
            return False
        return True

    # ---------------------------------------------------------
    # Safe Synchronous Service Calls
    # ---------------------------------------------------------
    def call_service_sync(self, client, req, timeout=5.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.5):
                self.get_logger().warn(f"Service {client.srv_name} not available!")
                return None
        future = client.call_async(req)
        t0 = time.time()
        while not future.done():
            if time.time() - t0 > timeout:
                self.get_logger().error(f"Service call {client.srv_name} timed out after {timeout}s!")
                return None
            time.sleep(0.015)
        return future.result()

    def set_gripper(self, open_state: bool):
        req = GripperStatus.Request()
        req.status = open_state
        res = self.call_service_sync(self.set_gripper_cli, req, timeout=3.0)
        time.sleep(0.65)
        return res is not None and res.flag

    def send_coords(self, coords, speed=None, model=0):
        if speed is None:
            speed = self.speed
        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
        if not self.is_in_safe_workspace(x, y):
            self.get_logger().error(f"[!] Target [X:{x:.1f}, Y:{y:.1f}] is OUTSIDE safe workspace! Blocked.")
            return False
        z = max(SAFE_WORKSPACE['z_min'], min(SAFE_WORKSPACE['z_lift'] + 20.0, z))

        req = SetCoords.Request()
        req.x, req.y, req.z = x, y, z
        req.rx, req.ry, req.rz = float(coords[3]), float(coords[4]), float(coords[5])
        req.speed = int(speed)
        req.model = int(model)
        res = self.call_service_sync(self.set_coords_cli, req, timeout=5.0)
        return res is not None and res.flag

    def send_angles(self, angles, speed=None):
        if speed is None:
            speed = self.speed
        if not self.is_pose_safe(angles):
            self.get_logger().error(f"[!] Angles {angles[:4]} would cause self-collision! Rejected.")
            return False
        req = SetAngles.Request()
        req.joint_1, req.joint_2, req.joint_3 = float(angles[0]), float(angles[1]), float(angles[2])
        req.joint_4, req.joint_5, req.joint_6 = float(angles[3]), float(angles[4]), float(angles[5])
        req.speed = int(speed)
        res = self.call_service_sync(self.set_angles_cli, req, timeout=5.0)
        return res is not None and res.flag

    def get_current_angles(self):
        req = GetAngles.Request()
        res = self.call_service_sync(self.get_angles_cli, req, timeout=2.0)
        if res and [res.joint_1, res.joint_2, res.joint_3] != [-1.0, -1.0, -1.0]:
            return [res.joint_1, res.joint_2, res.joint_3, res.joint_4, res.joint_5, res.joint_6]
        return None

    def get_current_coords(self):
        req = GetCoords.Request()
        res = self.call_service_sync(self.get_coords_cli, req, timeout=2.0)
        if res and [res.x, res.y, res.z] != [-1.0, -1.0, -1.0]:
            return [res.x, res.y, res.z, res.rx, res.ry, res.rz]
        return None

    def move_to_observe(self):
        self.set_gripper(True)
        self.send_angles(DEFAULT_OBSERVE_ANGLES, speed=35)
        time.sleep(1.2)

    # ---------------------------------------------------------
    # Coordinate Mapping: Pixel (u, v) -> Workspace (X, Y)
    # Using Interactive Calibration Reference
    # ---------------------------------------------------------
    def pixel_to_workspace(self, cx, cy):
        ref_px = self.calib.get("ref_pixel", [320.0, 240.0])
        ref_robot = self.calib.get("ref_robot", [170.0, 0.0])
        scale_x = self.calib.get("scale_x", 0.25)
        scale_y = self.calib.get("scale_y", -0.25)

        dx_px = float(cx) - ref_px[0]
        dy_px = float(cy) - ref_px[1]

        robot_x = ref_robot[0] + dy_px * scale_x
        robot_y = ref_robot[1] + dx_px * scale_y
        return robot_x, robot_y

    # ---------------------------------------------------------
    # Teaching & Calibration Functions
    # ---------------------------------------------------------
    def calibrate_camera_reference(self):
        """
        1-CLICK CALIBRATION:
        Matches current physical arm position with the detected cube's pixel (cx, cy).
        """
        coords = self.get_current_coords()
        if not coords:
            print("\n[-] Error: Could not read current robot coordinates!")
            print("    Cause: The robot service 'get_coords' is not running.")
            print("    👉 Fix: Please launch the full system with:")
            print("       ros2 launch mycobot_280 color_pick_and_place.launch.py port:=/dev/ttyUSB0 baud:=115200\n")
            return False

        if not self.latest_detections:
            print("\n[-] Error: No cube currently detected in camera view!")
            print("    1. Ensure the camera bridge is streaming on Mac (./start_ros2_dualsense.sh).")
            print("    2. Place a cube (Red, Green, Blue, or Yellow) in view of the camera.")
            print("    3. Move the claw above it and press [k] again.\n")
            return False

        target_det = self.latest_detections[0]
        cx, cy = target_det['center']
        rx, ry = coords[0], coords[1]

        self.calib["ref_pixel"] = [float(cx), float(cy)]
        self.calib["ref_robot"] = [float(rx), float(ry)]
        self.save_custom_positions()

        print("\n" + "=" * 60)
        print("  [✓] CAMERA REFERENCE CALIBRATION COMPLETE!")
        print(f"      Matched Camera Pixel:   ({cx}, {cy})")
        print(f"      To Physical Robot X,Y: ({rx:.1f}, {ry:.1f}) mm")
        print("=" * 60 + "\n")
        return True

    def teach_place_point(self, color_name: str):
        """Teaches custom drop/place point for a specific color."""
        angles = self.get_current_angles()
        if not angles:
            print(f"\n[-] Error: Could not read current arm joint angles!")
            print("    Cause: The robot service 'get_angles' is not running.")
            print("    👉 Fix: Launch with: ros2 launch mycobot_280 color_pick_and_place.launch.py port:=/dev/ttyUSB0 baud:=115200\n")
            return False

        if not self.is_pose_safe(angles):
            print(f"\n[-] Warning: Current arm pose is unsafe (claw near joint). Please adjust and try again.")
            return False

        self.calib["drop_bins"][color_name] = [round(float(a), 2) for a in angles]
        self.save_custom_positions()
        print(f"\n[✓] SAVED Custom Place Point for '{color_name.upper()}': {self.calib['drop_bins'][color_name]}")
        return True

    def teach_fixed_pick_spot(self):
        """Teaches a fixed tabletop pickup position."""
        coords = self.get_current_coords()
        if not coords:
            print("\n[-] Error: Could not read current robot coordinates!")
            return False

        rx, ry, rz = coords[0], coords[1], coords[2]
        if not self.is_in_safe_workspace(rx, ry):
            print(f"\n[-] Warning: Position ({rx:.1f}, {ry:.1f}) is outside the safe workspace bounds!")
            return False

        self.calib["fixed_pick_coords"] = [round(float(rx), 1), round(float(ry), 1), round(float(rz), 1)]
        self.save_custom_positions()
        print(f"\n[✓] SAVED Fixed Tabletop Pick Spot: X:{rx:.1f}mm, Y:{ry:.1f}mm, Z:{rz:.1f}mm")
        return True

    # ---------------------------------------------------------
    # Detection Callbacks
    # ---------------------------------------------------------
    def detections_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_detections = data.get("detections", [])
            self.last_detection_time = time.time()
            self.publish_rviz_markers()
        except Exception:
            pass

    def image_callback(self, msg: Image):
        if time.time() - self.last_detection_time < 1.5 or self.busy:
            return
        try:
            h, w = msg.height, msg.width
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            fallback_detections = []
            for c_name, params in COLOR_HSV_RANGES.items():
                if 'lower1' in params:
                    m1 = cv2.inRange(hsv, params['lower1'], params['upper1'])
                    m2 = cv2.inRange(hsv, params['lower2'], params['upper2'])
                    mask = cv2.bitwise_or(m1, m2)
                else:
                    mask = cv2.inRange(hsv, params['lower'], params['upper'])

                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in contours:
                    area = cv2.contourArea(c)
                    if area < 600:
                        continue
                    x, y, bw, bh = cv2.boundingRect(c)
                    ratio = float(bw) / float(bh)
                    if ratio < 0.6 or ratio > 1.65:
                        continue
                    cx = int(x + bw / 2)
                    cy = int(y + bh / 2)
                    fallback_detections.append({
                        "label": f"{c_name} cube",
                        "confidence": 0.90,
                        "box": [x, y, x + bw, y + bh],
                        "center": [cx, cy]
                    })
            if fallback_detections:
                self.latest_detections = fallback_detections
                self.last_detection_time = time.time()
                self.publish_rviz_markers()
        except Exception:
            pass

    def trigger_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"[!] Trigger Received: '{cmd}'")
        if cmd in ('cancel', 'stop'):
            self.auto_mode = False
            self.publish_status("STOPPED")
            return
        if cmd in ('home', 'observe'):
            threading.Thread(target=self.move_to_observe, daemon=True).start()
            return

        # 1-Click Calibration via Topic or Gamepad
        if cmd in ('k', 'calib', 'calibrate'):
            self.get_logger().info("[🎯] Calibration triggered!")
            threading.Thread(target=self.calibrate_camera_reference, daemon=True).start()
            return

        # Save Fixed Spot / Mode Toggle via Topic or Gamepad
        if cmd in ('s', 'save', 'fixed'):
            self.get_logger().info("[📌] Fixed Pick Spot teach triggered!")
            threading.Thread(target=self.teach_fixed_pick_spot, daemon=True).start()
            return
        if cmd in ('f', 'toggle_mode'):
            self.use_fixed_pick = not self.use_fixed_pick
            self.get_logger().info(f"[+] Switched Pick Mode: {'FIXED SPOT' if self.use_fixed_pick else 'DYNAMIC CAMERA'}")
            return
        if cmd in ('t', 'test_hover'):
            self.get_logger().info("[🧪] Test Hover Alignment triggered!")
            threading.Thread(target=self.test_hover_alignment, daemon=True).start()
            return

        # Place Point Teaching
        for color_name in ('red', 'green', 'blue', 'yellow'):
            if cmd == f"teach_{color_name}":
                self.get_logger().info(f"[📌] Teaching place point for {color_name.upper()}!")
                threading.Thread(target=lambda c=color_name: self.teach_place_point(c), daemon=True).start()
                return

        if cmd in ('auto', 'all'):
            target = 'all'
        elif any(c in cmd for c in ('red', 'green', 'blue', 'yellow')):
            target = [c for c in ('red', 'green', 'blue', 'yellow') if c in cmd][0]
        else:
            target = 'all'

        threading.Thread(target=lambda: self.execute_single_sort(target), daemon=True).start()

    # ---------------------------------------------------------
    # RViz Safe Area & Object Visualization
    # ---------------------------------------------------------
    def publish_safe_area_marker(self):
        now = self.get_clock().now().to_msg()

        # 3D Safe Bounding Volume
        box = Marker()
        box.header.frame_id = "g_base"
        box.header.stamp = now
        box.ns = "safe_workspace"
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD

        x_mid = (SAFE_WORKSPACE['x_min'] + SAFE_WORKSPACE['x_max']) / 2.0 / 1000.0
        y_mid = (SAFE_WORKSPACE['y_min'] + SAFE_WORKSPACE['y_max']) / 2.0 / 1000.0
        # Visually ground the safe box from the tabletop (Z=0) up to hover reach height
        reach_height = SAFE_WORKSPACE['z_hover'] / 1000.0
        z_mid = reach_height / 2.0

        box.pose.position.x = x_mid
        box.pose.position.y = y_mid
        box.pose.position.z = z_mid
        box.pose.orientation.w = 1.0

        box.scale.x = (SAFE_WORKSPACE['x_max'] - SAFE_WORKSPACE['x_min']) / 1000.0
        box.scale.y = (SAFE_WORKSPACE['y_max'] - SAFE_WORKSPACE['y_min']) / 1000.0
        box.scale.z = reach_height

        box.color = ColorRGBA(r=0.0, g=0.9, b=0.7, a=0.18)
        self.safe_area_pub.publish(box)

        txt = Marker()
        txt.header.frame_id = "g_base"
        txt.header.stamp = now
        txt.ns = "safe_workspace"
        txt.id = 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = x_mid
        txt.pose.position.y = y_mid
        txt.pose.position.z = reach_height + 0.035
        txt.scale.z = 0.024
        txt.color = ColorRGBA(r=0.1, g=1.0, b=0.8, a=1.0)
        mode_str = "[FIXED PICK SPOT]" if self.use_fixed_pick else "[CAMERA TRACKING]"
        txt.text = f"SAFE AREA {mode_str}"
        self.safe_area_pub.publish(txt)

    def publish_rviz_markers(self):
        now = self.get_clock().now().to_msg()
        for idx, det in enumerate(self.latest_detections):
            label = det.get('label', '').lower()
            cx, cy = det.get('center', [320, 240])
            rx, ry = self.pixel_to_workspace(cx, cy)

            matched_color = None
            for c in ('red', 'green', 'blue', 'yellow'):
                if c in label:
                    matched_color = c
                    break
            if not matched_color:
                continue

            in_safe_zone = self.is_in_safe_workspace(rx, ry)

            m = Marker()
            m.header.frame_id = "g_base"
            m.header.stamp = now
            m.ns = "colored_cubes"
            m.id = idx
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = rx / 1000.0
            m.pose.position.y = ry / 1000.0
            # Tabletop cube resting on table (Z=0): 25mm cube center is at Z=12.5mm
            m.pose.position.z = 0.0125
            m.pose.orientation.w = 1.0
            m.scale.x = 0.025
            m.scale.y = 0.025
            m.scale.z = 0.025

            if in_safe_zone or self.use_fixed_pick:
                m.color = COLOR_RGBA.get(matched_color, ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9))
            else:
                m.color = ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.45)
            self.marker_pub.publish(m)

            txt = Marker()
            txt.header.frame_id = "g_base"
            txt.header.stamp = now
            txt.ns = "cube_labels"
            txt.id = idx + 100
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = rx / 1000.0
            txt.pose.position.y = ry / 1000.0
            # Label floats slightly above the cube
            txt.pose.position.z = 0.045
            txt.scale.z = 0.018

            if in_safe_zone or self.use_fixed_pick:
                txt.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
                txt.text = f"{matched_color.upper()} [SAFE] ({int(rx)}, {int(ry)})"
            else:
                txt.color = ColorRGBA(r=1.0, g=0.3, b=0.3, a=1.0)
                txt.text = f"{matched_color.upper()} [OUT OF BOUNDS] ({int(rx)}, {int(ry)})"
            self.marker_pub.publish(txt)

    def publish_status(self, text: str):
        self.current_action = text
        s = String()
        s.data = text
        self.status_pub.publish(s)

        b = Bool()
        b.data = self.busy
        self.busy_pub.publish(b)

    # ---------------------------------------------------------
    # Core Pick & Place Routine (with Elbow-Up Guard)
    # ---------------------------------------------------------
    def execute_single_sort(self, color_filter='all'):
        if self.busy:
            self.get_logger().warn("[*] Robot is already executing an action. Please wait...")
            return False

        self.busy = True
        self.publish_status("SCANNING FOR TARGETS")

        try:
            target_color = None
            target_rx = None
            target_ry = None

            # 1. Acquire target
            t_search = time.time()
            while time.time() - t_search < 3.0:
                dets = list(self.latest_detections)
                valid = []
                for d in dets:
                    lbl = d.get('label', '').lower()
                    for c in ('red', 'green', 'blue', 'yellow'):
                        if c in lbl:
                            if color_filter in ('all', 'auto') or color_filter == c:
                                if self.use_fixed_pick:
                                    valid.append((c, d, self.calib["fixed_pick_coords"][0], self.calib["fixed_pick_coords"][1]))
                                else:
                                    cx, cy = d['center']
                                    rx, ry = self.pixel_to_workspace(cx, cy)
                                    if self.is_in_safe_workspace(rx, ry):
                                        valid.append((c, d, rx, ry))
                                    else:
                                        self.get_logger().warn(
                                            f"[-] Rejected {c.upper()} at ({rx:.0f}, {ry:.0f}): Outside Safe Area!",
                                            throttle_duration_sec=2.0
                                        )
                            break
                if valid:
                    valid.sort(key=lambda item: item[1].get('confidence', 0.0), reverse=True)
                    target_color, _, target_rx, target_ry = valid[0]
                    break
                time.sleep(0.1)

            if not target_color:
                self.get_logger().warn(f"[-] No '{color_filter.upper()}' cube found in the safe zone.")
                self.publish_status("NO SAFE TARGET FOUND")
                return False

            mode_str = "FIXED SPOT" if self.use_fixed_pick else "CAMERA TARGET"
            self.get_logger().info(f"[+] {mode_str}: Picking {target_color.upper()} Cube at [X:{target_rx:.1f}, Y:{target_ry:.1f}] mm")

            # Calibrated wrist posture (rx, ry, rz)
            posture_rx = 179.87
            posture_ry = -3.78
            posture_rz = -62.75

            # Phase 1: Pre-grab intermediate pose (elbow-up guarantee)
            self.publish_status("ALIGNING (PRE-GRAB POSE)")
            self.set_gripper(True)
            self.send_angles(PRE_GRAB_ANGLES, speed=30)
            time.sleep(1.2)

            # Phase 2: Hover above target
            self.publish_status(f"HOVERING -> {target_color.upper()}")
            hover_coords = [target_rx, target_ry, SAFE_WORKSPACE['z_hover'], posture_rx, posture_ry, posture_rz]
            self.send_coords(hover_coords, speed=self.speed, model=1)
            time.sleep(1.4)

            # Phase 3: Descend straight down to grip height
            self.publish_status(f"DESCENDING -> {target_color.upper()}")
            pick_z = self.calib["fixed_pick_coords"][2] if self.use_fixed_pick else self.grip_z
            grip_coords = [target_rx, target_ry, pick_z, posture_rx, posture_ry, posture_rz]
            self.send_coords(grip_coords, speed=20, model=1)
            time.sleep(1.2)

            # Phase 4: Close claw firmly on cube
            self.publish_status(f"GRASPING -> {target_color.upper()}")
            self.set_gripper(False)
            time.sleep(0.9)

            # Phase 5: Safe Vertical Lift using Joint 2 Angle
            self.publish_status("SAFE LIFTING")
            curr_angles = self.get_current_angles()
            if curr_angles:
                lift_angles = [curr_angles[0], -0.71, -54.49, -23.02, -0.79, curr_angles[5]]
                self.send_angles(lift_angles, speed=25)
            else:
                lift_coords = [target_rx, target_ry, SAFE_WORKSPACE['z_lift'], posture_rx, posture_ry, posture_rz]
                self.send_coords(lift_coords, speed=25, model=1)
            time.sleep(1.3)

            # Phase 6: Rotate to Custom Place Point
            self.publish_status(f"TRANSPORTING -> CUSTOM {target_color.upper()} PLACE POINT")
            drop_angles = self.calib["drop_bins"].get(target_color, DEFAULT_OBSERVE_ANGLES)
            self.send_angles(drop_angles, speed=30)
            time.sleep(1.8)

            # Phase 7: Release cube
            self.publish_status(f"RELEASING -> {target_color.upper()}")
            self.set_gripper(True)
            time.sleep(0.7)

            # Phase 8: Return to Observation Pose
            self.publish_status("RETURNING TO OBSERVE")
            self.send_angles(DEFAULT_OBSERVE_ANGLES, speed=35)
            time.sleep(1.5)

            self.sorted_count += 1
            self.get_logger().info(f"[✓] Successfully sorted {target_color.upper()} Cube to Custom Place Point! (Total: {self.sorted_count})")
            self.publish_status(f"SORTED: {target_color.upper()}")
            return True

        except Exception as e:
            self.get_logger().error(f"Error during pick & place execution: {e}")
            self.publish_status(f"ERROR: {e}")
            try:
                self.send_angles(DEFAULT_OBSERVE_ANGLES, speed=25)
            except Exception:
                pass
            return False
        finally:
            self.busy = False
            b = Bool()
            b.data = False
            self.busy_pub.publish(b)

    # ---------------------------------------------------------
    # Autonomous Sorting Worker Loop
    # ---------------------------------------------------------
    def auto_worker_loop(self):
        time.sleep(2.0)
        while self.running and rclpy.ok():
            if self.auto_mode and not self.busy:
                dets = list(self.latest_detections)
                has_safe_cube = False
                for d in dets:
                    lbl = d.get('label', '').lower()
                    if any(c in lbl for c in ('red', 'green', 'blue', 'yellow')):
                        if self.use_fixed_pick:
                            has_safe_cube = True
                            break
                        else:
                            cx, cy = d['center']
                            rx, ry = self.pixel_to_workspace(cx, cy)
                            if self.is_in_safe_workspace(rx, ry):
                                has_safe_cube = True
                                break

                if has_safe_cube:
                    self.execute_single_sort(self.target_color)
                    time.sleep(1.0)
                else:
                    time.sleep(0.5)
            else:
                time.sleep(0.2)

    # ---------------------------------------------------------
    # Interactive Teaching & Control CLI
    # ---------------------------------------------------------
    def interactive_cli_loop(self):
        time.sleep(1.0)
        self.print_menu()

        while self.running and rclpy.ok():
            try:
                if select.select([sys.stdin], [], [], 0.5)[0]:
                    key = sys.stdin.readline().strip().lower()
                    if not key:
                        continue
                    if key == 'q':
                        self.get_logger().info("Exiting CLI...")
                        self.running = False
                        break
                    elif key == '?':
                        self.print_menu()
                    elif key == 'p':
                        self.print_status_info()
                    elif key == 'k':
                        self.calibrate_camera_reference()
                    elif key == 's':
                        self.teach_fixed_pick_spot()
                    elif key == 'f':
                        self.use_fixed_pick = not self.use_fixed_pick
                        state = "FIXED PICK SPOT" if self.use_fixed_pick else "DYNAMIC CAMERA TRACKING"
                        print(f"\n[*] Pick Mode Toggled: {state}")
                    elif key == '1':
                        self.teach_place_point('red')
                    elif key == '2':
                        self.teach_place_point('green')
                    elif key == '3':
                        self.teach_place_point('blue')
                    elif key == '4':
                        self.teach_place_point('yellow')
                    elif key == 'a':
                        self.auto_mode = not self.auto_mode
                        state_str = "ENABLED (Continuous Sorting)" if self.auto_mode else "DISABLED"
                        print(f"\n[*] Auto-Sort Mode: {state_str}")
                    elif key == 'r':
                        print("\n[*] Picking RED Cube...")
                        threading.Thread(target=lambda: self.execute_single_sort('red'), daemon=True).start()
                    elif key == 'g':
                        print("\n[*] Picking GREEN Cube...")
                        threading.Thread(target=lambda: self.execute_single_sort('green'), daemon=True).start()
                    elif key == 'b':
                        print("\n[*] Picking BLUE Cube...")
                        threading.Thread(target=lambda: self.execute_single_sort('blue'), daemon=True).start()
                    elif key == 'y':
                        print("\n[*] Picking YELLOW Cube...")
                        threading.Thread(target=lambda: self.execute_single_sort('yellow'), daemon=True).start()
                    elif key == 'h':
                        print("\n[*] Moving to Observation Pose...")
                        threading.Thread(target=self.move_to_observe, daemon=True).start()
                    elif key == 'o':
                        print("\n[*] Opening Claw...")
                        threading.Thread(target=lambda: self.set_gripper(True), daemon=True).start()
                    elif key == 'c':
                        print("\n[*] Closing Claw...")
                        threading.Thread(target=lambda: self.set_gripper(False), daemon=True).start()
                    elif key in ('+', '='):
                        self.grip_z += 2.0
                        print(f"\n[+] Grip Z-Height increased to: {self.grip_z:.1f} mm")
                    elif key in ('-', '_'):
                        self.grip_z = max(SAFE_WORKSPACE['z_min'], self.grip_z - 2.0)
                        print(f"\n[-] Grip Z-Height decreased to: {self.grip_z:.1f} mm")
                    elif key == 't':
                        print("\n[*] Testing Hover Alignment...")
                        threading.Thread(target=self.test_hover_alignment, daemon=True).start()
            except Exception:
                break

    def test_hover_alignment(self):
        if self.use_fixed_pick:
            fx, fy, _ = self.calib["fixed_pick_coords"]
            self.get_logger().info(f"Hovering at Fixed Pick Spot X:{fx:.1f}mm, Y:{fy:.1f}mm")
            self.set_gripper(True)
            self.send_angles(PRE_GRAB_ANGLES, speed=30)
            time.sleep(1.2)
            self.send_coords([fx, fy, SAFE_WORKSPACE['z_hover'], 179.87, -3.78, -62.75], speed=25, model=1)
            return

        for d in self.latest_detections:
            cx, cy = d['center']
            rx, ry = self.pixel_to_workspace(cx, cy)
            if self.is_in_safe_workspace(rx, ry):
                self.get_logger().info(f"Hovering at Detected Target X:{rx:.1f}mm, Y:{ry:.1f}mm")
                self.set_gripper(True)
                self.send_angles(PRE_GRAB_ANGLES, speed=30)
                time.sleep(1.2)
                self.send_coords([rx, ry, SAFE_WORKSPACE['z_hover'], 179.87, -3.78, -62.75], speed=25, model=1)
                return
        self.get_logger().warn("[-] No valid target currently detected inside the Safe Area.")

    def print_menu(self):
        print("\n" + "=" * 65)
        print("   myCobot 280 - Interactive Calibration & Custom Place Points")
        print("=" * 65)
        if not self.get_coords_cli.service_is_ready():
            print("  ⚠️  WARNING: Robot driver ('listen_real_service') is NOT detected!")
            print("     To launch all required services together, please run:")
            print("     ros2 launch mycobot_280 color_pick_and_place.launch.py port:=/dev/ttyUSB0 baud:=115200")
            print("-" * 65)
        print("  TEACHING & CALIBRATION COMMANDS:")
        print("    [k] CALIBRATE CAMERA REFERENCE (Place cube under claw & press [k])")
        print("    [s] SAVE CURRENT POSE as Custom Fixed Pick Spot")
        print("    [f] TOGGLE PICK MODE: Fixed Spot vs Dynamic Camera Tracking")
        print("    [1] SAVE CURRENT POSE as Custom RED Place Point")
        print("    [2] SAVE CURRENT POSE as Custom GREEN Place Point")
        print("    [3] SAVE CURRENT POSE as Custom BLUE Place Point")
        print("    [4] SAVE CURRENT POSE as Custom YELLOW Place Point")
        print("    [p] PRINT CURRENT ROBOT COORDINATES & SAVED CUSTOM POINTS")
        print("-" * 65)
        print("  EXECUTION & TESTING:")
        print("    [a] Toggle Autonomous Auto-Sort (Continuously sort cubes)")
        print("    [r] Pick & Sort RED Cube")
        print("    [g] Pick & Sort GREEN Cube")
        print("    [b] Pick & Sort BLUE Cube")
        print("    [y] Pick & Sort YELLOW Cube")
        print("    [h] Return to Observation / Home Pose")
        print("    [t] Test Hover Alignment (Hover without picking)")
        print("    [o] / [c] Open / Close Claw")
        print("    [+] / [-] Adjust Grasp Z-Height (+/- 2mm)")
        print("    [?] Show this Help Menu | [q] Quit")
        print("=" * 65 + "\n")

    def print_status_info(self):
        angles = self.get_current_angles()
        coords = self.get_current_coords()
        print("\n" + "-" * 60)
        print("  CURRENT PHYSICAL ROBOT STATE:")
        if coords:
            print(f"    Coords: [X:{coords[0]:.1f}, Y:{coords[1]:.1f}, Z:{coords[2]:.1f}] mm | [Rx:{coords[3]:.1f}, Ry:{coords[4]:.1f}, Rz:{coords[5]:.1f}]")
        if angles:
            print(f"    Angles: {[round(a, 1) for a in angles]}")
        print("\n  SAVED CUSTOM CONFIGURATION:")
        print(f"    Pick Mode: {'FIXED PICK SPOT' if self.use_fixed_pick else 'DYNAMIC CAMERA'}")
        print(f"    Fixed Pick Spot:      {self.calib['fixed_pick_coords']}")
        print(f"    Camera Reference Px:  {self.calib['ref_pixel']} -> Robot: {self.calib['ref_robot']}")
        print("    Custom Drop Points:")
        for color, j_angles in self.calib['drop_bins'].items():
            print(f"      {color.upper():6s} Drop Point: {j_angles}")
        print("-" * 60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = ColorPickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.running = False
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
