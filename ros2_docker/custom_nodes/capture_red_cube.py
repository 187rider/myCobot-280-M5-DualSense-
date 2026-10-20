#!/usr/bin/env python3
"""
myCobot 280 M5 - Real-Time Dynamic Visual Tracking & Capture Node
(Cobra / Dobot Magician Style Dynamic Visual Servoing & Grasp Control)

Features:
  - 🔗 Continuous Linked Following:
      The robot hand (claw) is dynamically linked to the RED CUBE in real-time!
      As you move the red cube across the desk with your hand, the robot hand
      actively shadows and follows its (X, Y) coordinates directly above it.
  - 🎯 Instant Capture ([c], [Space], or DualSense Square ⏹):
      Descends straight down from hover height (180mm) to grasp height (95mm),
      clamps the red cube firmly with the gripper, and lifts it into the air (215mm).
  - ⬇️  Release & Resume ([r] or DualSense Circle ◯):
      Gently lowers the captured red cube back onto the table, opens the gripper,
      lifts back to hover height, and immediately resumes dynamic following!
  - ⚡ Auto-Grasp Mode ([a]):
      Optionally auto-captures the red cube whenever it stops moving on the table for ~1.2s.
  - 🛡️ Hard Physical Protection:
      Table floor limit (Z >= 85mm), workspace boundary clamp, elbow-up IK branch.
  - 🚫 No bins / no sorting: Pure Cobra Magician visual tracking & capture!
"""

import sys
import os
import json
import time
import math
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import String, Header, ColorRGBA, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from mycobot_interfaces.srv import (
    SetAngles,
    GetAngles,
    SetCoords,
    GetCoords,
    GripperStatus
)

CONFIG_FILE_PATH = "/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/custom_positions.json"
EYES_IN_HAND_PATH = "/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config/EyesInHand_matrix.json"
HOST_BACKUP_PATH = "/tmp/custom_positions.json"

# Standby observation pose
DEFAULT_OBSERVE_ANGLES = [-7.11, -6.94, -55.01, -24.16, 0.0, -15.0]
# Pre-grab intermediate posture (guarantees forward elbow-up branch)
PRE_GRAB_ANGLES = [18.8, -7.91, -54.49, -23.02, -0.79, -14.76]

# Calibrated flange wrist orientation (wrist perpendicular pointing straight down at table)
FLANGE_POSTURE = [179.87, -3.78, -62.75]

HAND_RADIUS = 60.0  # mm (gripper tool extension / finger reach)

# Safe physical workspace bounds (increased by hand radius for generous reach)
SAFE_WORKSPACE = {
    'x_min': 50.0,       # mm (expanded inward toward base)
    'x_max': 305.0,      # mm (expanded forward: 255 + 50mm hand radius)
    'y_min': -210.0,     # mm (expanded right span: -150 - 60mm hand radius)
    'y_max': 210.0,      # mm (expanded left span: +150 + 60mm hand radius)
    'max_radius': 320.0, # mm (expanded maximum reach radius with hand: 265 + 55mm)
    'min_radius': 50.0,  # mm
    'z_min': 85.0,       # mm hard table floor limit
    'z_hover': 180.0,    # mm hover tracking height
    'z_lift': 215.0,     # mm holding height after grasp
}


class CaptureRedCubeNode(Node):
    def __init__(self):
        super().__init__('capture_red_cube')
        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('speed', 35)
        self.declare_parameter('grip_z', 95.0)
        self.declare_parameter('auto_capture', False)
        self.declare_parameter('invert_y', True)
        self.declare_parameter('hand_radius', HAND_RADIUS)

        self.speed = int(self.get_parameter('speed').value)
        self.grip_z = float(self.get_parameter('grip_z').value)
        self.auto_capture_enabled = bool(self.get_parameter('auto_capture').value)
        self.invert_y = bool(self.get_parameter('invert_y').value)
        self.hand_radius = float(self.get_parameter('hand_radius').value)

        self.z_hover = SAFE_WORKSPACE['z_hover']
        self.z_lift = SAFE_WORKSPACE['z_lift']
        self.z_min = SAFE_WORKSPACE['z_min']
        self.x_min = SAFE_WORKSPACE['x_min']
        self.x_max = SAFE_WORKSPACE['x_max']
        self.y_min = SAFE_WORKSPACE['y_min']
        self.y_max = SAFE_WORKSPACE['y_max']
        self.max_radius = SAFE_WORKSPACE['max_radius']
        self.min_radius = SAFE_WORKSPACE['min_radius']

        # Camera & Table Calibration Reference
        self.calib = {
            "ref_pixel": [320.0, 240.0],
            "ref_robot": [170.0, 0.0],
            "scale_x": 0.25,
            "scale_y": -0.25
        }
        self.load_calibration()

        # Service Clients
        self.set_coords_cli = self.create_client(SetCoords, 'set_coords', callback_group=self.cb_group)
        self.set_angles_cli = self.create_client(SetAngles, 'set_angles', callback_group=self.cb_group)
        self.get_coords_cli = self.create_client(GetCoords, 'get_coords', callback_group=self.cb_group)
        self.get_angles_cli = self.create_client(GetAngles, 'get_angles', callback_group=self.cb_group)
        self.set_gripper_cli = self.create_client(GripperStatus, 'set_gripper', callback_group=self.cb_group)

        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)
        self.safe_area_pub = self.create_publisher(Marker, '/safe_area_marker', 10)
        self.status_pub = self.create_publisher(String, '/color_sort/status', 10)

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
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10, callback_group=self.cb_group
        )

        # Vision State
        self.latest_frame = None
        self.latest_red_cube = None
        self.last_detection_time = 0.0

        # Cobra Magician Dynamic Tracking State
        self.following_enabled = True       # Dynamic follow active
        self.tracking_active = False        # Arm currently in hover tracking posture
        self.busy = False                   # Atomic grasp or release running
        self.has_cube_grasped = False       # True when red cube is held in claw

        # Filtered target positions (starts at center of workspace)
        self.tracked_x = 170.0
        self.tracked_y = 0.0
        self.still_counter = 0
        self.last_joy_time = 0.0

        self.get_logger().info("=" * 65)
        self.get_logger().info("  🔴 Cobra Magician Dynamic Red Cube Tracker Initialized")
        self.get_logger().info("=" * 65)

        # Timers and Threads
        self.viz_timer = self.create_timer(0.2, self.publish_rviz_markers)

        # Continuous High-Frequency Follow Loop (~10 Hz)
        self.follow_thread = threading.Thread(target=self.follow_worker_loop, daemon=True)
        self.follow_thread.start()

        # Interactive Terminal Loop
        self.cli_thread = threading.Thread(target=self.run_cli_loop, daemon=True)
        self.cli_thread.start()

    def load_calibration(self):
        """Loads Hand-Eye calibration matrix or custom anchor positions."""
        for p in [EYES_IN_HAND_PATH, "/tmp/EyesInHand_matrix.json"]:
            if os.path.exists(p):
                try:
                    with open(p, 'r') as f:
                        self.calib.update(json.load(f))
                    self.get_logger().info(f"[+] Loaded Hand-Eye Matrix from {p}")
                    break
                except Exception:
                    pass

        for p in [CONFIG_FILE_PATH, HOST_BACKUP_PATH]:
            if os.path.exists(p):
                try:
                    with open(p, 'r') as f:
                        self.calib.update(json.load(f))
                    self.get_logger().info(f"[+] Loaded custom config from {p}")
                    break
                except Exception:
                    pass

    # ---------------------------------------------------------
    # Synchronous Driver Service Calls
    # ---------------------------------------------------------
    def call_service_sync(self, client, request, timeout=3.5):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=timeout):
                return None
        future = client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            if future.done():
                return future.result()
            time.sleep(0.015)
        return None

    def send_angles(self, angles, speed=35):
        req = SetAngles.Request()
        req.joint_1 = float(angles[0])
        req.joint_2 = float(angles[1])
        req.joint_3 = float(angles[2])
        req.joint_4 = float(angles[3])
        req.joint_5 = float(angles[4])
        req.joint_6 = float(angles[5])
        req.speed = int(speed)
        return self.call_service_sync(self.set_angles_cli, req)

    def send_coords(self, coords, speed=35, model=1):
        x, y, z = coords[0], coords[1], coords[2]
        # Anti-crash table clamp
        z = max(SAFE_WORKSPACE['z_min'], min(SAFE_WORKSPACE['z_lift'] + 20.0, z))
        req = SetCoords.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.rx = float(coords[3])
        req.ry = float(coords[4])
        req.rz = float(coords[5])
        req.speed = int(speed)
        req.model = int(model)
        return self.call_service_sync(self.set_coords_cli, req)

    def set_gripper(self, open_state: bool):
        req = GripperStatus.Request()
        req.status = open_state
        return self.call_service_sync(self.set_gripper_cli, req)

    def get_current_coords(self):
        req = GetCoords.Request()
        res = self.call_service_sync(self.get_coords_cli, req, timeout=2.0)
        if res and [res.x, res.y, res.z] != [-1.0, -1.0, -1.0]:
            return [res.x, res.y, res.z, res.rx, res.ry, res.rz]
        return None

    # ---------------------------------------------------------
    # Spatial Coordinate Transform
    # ---------------------------------------------------------
    def pixel_to_workspace(self, cx, cy):
        ref_px = self.calib.get("ref_pixel", [320.0, 240.0])
        ref_robot = self.calib.get("ref_robot", [170.0, 0.0])
        scale_x = float(self.calib.get("scale_x", 0.25))
        scale_y = float(self.calib.get("scale_y", -0.25))

        dx_px = float(cx) - ref_px[0]
        dy_px = float(cy) - ref_px[1]

        # Invert lateral Y tracking direction when invert_y is enabled
        y_sign = -1.0 if self.invert_y else 1.0

        robot_x = ref_robot[0] + dy_px * scale_x
        robot_y = ref_robot[1] + (dx_px * scale_y * y_sign)
        return robot_x, robot_y

    def is_in_safe_workspace(self, x: float, y: float) -> bool:
        radius = math.hypot(x, y)
        if radius > self.max_radius or radius < self.min_radius:
            return False
        if not (self.x_min <= x <= self.x_max):
            return False
        if not (self.y_min <= y <= self.y_max):
            return False
        return True

    # ---------------------------------------------------------
    # Visual Detections & Callbacks
    # ---------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            self.latest_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            # Fallback red detection directly from frame if detections topic is quiet
            if time.time() - self.last_detection_time > 0.8:
                self.detect_red_from_frame(self.latest_frame)
        except Exception:
            pass

    def detections_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            dets = data.get("detections", [])
            for d in dets:
                label = d.get('label', '').lower()
                if 'red' in label:
                    cx, cy = d.get('center', [320, 240])
                    rx, ry = self.pixel_to_workspace(cx, cy)
                    self.latest_red_cube = {
                        'pixel': (float(cx), float(cy)),
                        'robot_x': float(rx),
                        'robot_y': float(ry),
                        'confidence': float(d.get('confidence', 0.95)),
                        'box': d.get('box', [cx - 15, cy - 15, cx + 15, cy + 15]),
                        'in_safe_zone': self.is_in_safe_workspace(rx, ry),
                        'time': time.time()
                    }
                    self.last_detection_time = time.time()
                    break
        except Exception:
            pass

    def detect_red_from_frame(self, frame):
        try:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Dual-band red filter (ignoring skin tones S < 155)
            m1 = cv2.inRange(hsv, np.array([0, 160, 80]), np.array([8, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([172, 160, 80]), np.array([180, 255, 255]))
            mask = cv2.bitwise_or(m1, m2)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in contours:
                if cv2.contourArea(c) > 500:
                    x, y, w, h = cv2.boundingRect(c)
                    cx = float(x + w / 2)
                    cy = float(y + h / 2)
                    rx, ry = self.pixel_to_workspace(cx, cy)
                    self.latest_red_cube = {
                        'pixel': (cx, cy),
                        'robot_x': float(rx),
                        'robot_y': float(ry),
                        'confidence': 0.92,
                        'box': [x, y, x + w, y + h],
                        'in_safe_zone': self.is_in_safe_workspace(rx, ry),
                        'time': time.time()
                    }
                    self.last_detection_time = time.time()
                    break
        except Exception:
            pass

    def trigger_callback(self, msg: String):
        cmd = msg.data.lower()
        if 'capture' in cmd or 'red' in cmd or 'pick' in cmd:
            threading.Thread(target=self.execute_capture_red_cube, daemon=True).start()
        elif 'release' in cmd:
            threading.Thread(target=self.execute_release_to_table, daemon=True).start()
        elif 'follow' in cmd or 'track' in cmd:
            self.following_enabled = not self.following_enabled
            self.get_logger().info(f"[!] Follow Mode: {self.following_enabled}")

    def joy_callback(self, msg: Joy):
        """DualSense Gamepad triggers: Square (⏹) captures, Circle (◯) releases."""
        now = time.time()
        if now - self.last_joy_time < 0.5:
            return

        # Square button (button 2 or 3)
        square_pressed = False
        if len(msg.buttons) > 3:
            if msg.buttons[2] == 1 or msg.buttons[3] == 1:
                square_pressed = True

        # Circle button (button 1)
        circle_pressed = False
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            circle_pressed = True

        if square_pressed:
            self.last_joy_time = now
            if self.has_cube_grasped:
                self.get_logger().info("[🎮 DualSense] Square/Circle: Releasing Red Cube to table...")
                threading.Thread(target=self.execute_release_to_table, daemon=True).start()
            else:
                self.get_logger().info("[🎮 DualSense] Square: Capturing Red Cube!")
                threading.Thread(target=self.execute_capture_red_cube, daemon=True).start()

        elif circle_pressed and self.has_cube_grasped:
            self.last_joy_time = now
            self.get_logger().info("[🎮 DualSense] Circle: Releasing Red Cube to table...")
            threading.Thread(target=self.execute_release_to_table, daemon=True).start()

    # ---------------------------------------------------------
    # Cobra Magician Style Dynamic Visual Servoing
    # ---------------------------------------------------------
    def init_hover_posture(self):
        """Safely positions claw above table in hover follow posture."""
        self.busy = True
        try:
            self.get_logger().info("[*] Moving hand into hover tracking posture...")
            self.set_gripper(True)  # Open claw wide
            # 1. Elbow-up pre-grab branch
            self.send_angles(PRE_GRAB_ANGLES, speed=32)
            time.sleep(1.2)
            # 2. Hover coordinates
            hover_coords = [self.tracked_x, self.tracked_y, self.z_hover,
                            FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(hover_coords, speed=30, model=1)
            time.sleep(1.0)
            self.tracking_active = True
            print("\n" + "=" * 65)
            print("  🟢 COBRA MAGICIAN DYNAMIC TRACKING READY!")
            print("  👉 Move the Red Cube with your hand — the robot hand will follow it!")
            print("  👉 Press [c] or [Space] or DualSense Square (⏹) to CAPTURE!")
            print("=" * 65 + "\n")
        finally:
            self.busy = False

    def follow_worker_loop(self):
        """
        High-frequency visual servoing loop (10 Hz).
        Continuously links the robot hand to the red cube on the desk.
        """
        time.sleep(2.0)

        if self.following_enabled:
            self.init_hover_posture()

        loop_rate_sec = 0.1  # 10 Hz

        while rclpy.ok():
            time.sleep(loop_rate_sec)

            if not self.following_enabled or self.busy or self.has_cube_grasped:
                continue

            now = time.time()
            if not self.latest_red_cube or (now - self.latest_red_cube['time'] > 0.75):
                continue

            target_rx = self.latest_red_cube['robot_x']
            target_ry = self.latest_red_cube['robot_y']

            if not self.is_in_safe_workspace(target_rx, target_ry):
                continue

            # Displacement from current hand position
            delta = math.hypot(target_rx - self.tracked_x, target_ry - self.tracked_y)

            # Deadband threshold (6.5mm) filters out camera sensor noise
            if delta > 6.5:
                # Exponential low-pass filter (alpha = 0.65) for fluid tracking
                alpha = 0.65
                self.tracked_x = alpha * target_rx + (1.0 - alpha) * self.tracked_x
                self.tracked_y = alpha * target_ry + (1.0 - alpha) * self.tracked_y

                # Stream coordinate update to driver (async non-blocking UART)
                cmd_coords = [
                    self.tracked_x,
                    self.tracked_y,
                    self.z_hover,
                    FLANGE_POSTURE[0],
                    FLANGE_POSTURE[1],
                    FLANGE_POSTURE[2]
                ]
                self.send_coords(cmd_coords, speed=self.speed, model=1)
                self.still_counter = 0
                self.tracking_active = True
            else:
                # Red cube is resting still
                self.still_counter += 1
                if self.auto_capture_enabled and self.still_counter >= 12:
                    self.get_logger().info("[⚡ Auto-Grasp] Cube stabilized! Capturing now...")
                    self.still_counter = 0
                    threading.Thread(target=self.execute_capture_red_cube, daemon=True).start()

    # ---------------------------------------------------------
    # Core Actions: Capture, Release & Hover
    # ---------------------------------------------------------
    def execute_capture_red_cube(self):
        """
        Capture Action:
        Locks the red cube's position, descends straight down from hover (180mm)
        to grasp height (95mm), clamps gripper closed, and lifts into the air.
        """
        if self.busy:
            print("\n[!] Arm is currently busy. Please wait...")
            return False

        self.busy = True
        try:
            print("\n" + "=" * 60)
            print("  🔴 CAPTURING RED CUBE")
            print("=" * 60)

            # 1. Determine target lock coordinates
            now = time.time()
            if self.latest_red_cube and (now - self.latest_red_cube['time'] < 0.9):
                target_x = self.latest_red_cube['robot_x']
                target_y = self.latest_red_cube['robot_y']
            else:
                target_x = self.tracked_x
                target_y = self.tracked_y

            if not self.is_in_safe_workspace(target_x, target_y):
                print(f"[-] Cannot capture: Coordinates ({target_x:.1f}, {target_y:.1f}) out of safe workspace!")
                return False

            print(f"[1/4] Locked on Red Cube: X={target_x:.1f} mm, Y={target_y:.1f} mm")
            self.set_gripper(True)  # Open claw

            # 2. Center directly above cube at hover height
            print(f"[2/4] Centering directly over cube at Z={self.z_hover} mm...")
            hover_coords = [target_x, target_y, self.z_hover,
                            FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(hover_coords, speed=30, model=1)
            time.sleep(0.4)

            # 3. Descend straight down to grasp height
            print(f"[3/4] Descending to grasp height Z={self.grip_z} mm...")
            grip_coords = [target_x, target_y, self.grip_z,
                           FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(grip_coords, speed=22, model=1)
            time.sleep(1.1)

            # 4. Clamp & Lift
            print("[4/4] Clamping gripper closed and lifting Red Cube...")
            self.set_gripper(False)  # Close claw firmly
            time.sleep(0.8)

            lift_coords = [target_x, target_y, self.z_lift,
                           FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(lift_coords, speed=25, model=1)
            time.sleep(1.1)

            self.has_cube_grasped = True
            print("\n" + "=" * 60)
            print("  🎉 [✓] RED CUBE CAPTURED!")
            print("=" * 60)
            print("  The robot hand is holding the Red Cube in the air.")
            print("  👉 Press [r] or [Space] to RELEASE back down to table (and resume following)")
            print("=" * 60 + "\n")
            return True

        finally:
            self.busy = False

    def execute_release_to_table(self):
        """Gently lowers the red cube back to the desk, opens claw, and resumes tracking."""
        if self.busy:
            print("[!] Arm is busy. Please wait...")
            return

        self.busy = True
        try:
            print("\n[*] Gently releasing Red Cube back down to desk...")
            coords = self.get_current_coords()
            rx = coords[0] if coords else self.tracked_x
            ry = coords[1] if coords else self.tracked_y

            # Descend to table
            put_coords = [rx, ry, self.grip_z,
                          FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(put_coords, speed=20, model=1)
            time.sleep(1.2)

            # Open claw to release
            self.set_gripper(True)
            time.sleep(0.7)

            # Elevate back to hover
            up_coords = [rx, ry, self.z_hover,
                         FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(up_coords, speed=25, model=1)
            time.sleep(1.0)

            self.has_cube_grasped = False
            print("[✓] Red Cube released to desk! Resuming dynamic visual tracking...\n")
        finally:
            self.busy = False

    def execute_hover_test(self):
        """Aligns claw directly above the red cube at hover height."""
        if self.busy:
            print("[!] Arm is busy. Please wait...")
            return

        self.busy = True
        try:
            print("\n[*] Hover Alignment: Centering claw over Red Cube...")
            self.set_gripper(True)
            self.send_angles(PRE_GRAB_ANGLES, speed=30)
            time.sleep(1.0)

            target_x = self.latest_red_cube['robot_x'] if self.latest_red_cube else self.tracked_x
            target_y = self.latest_red_cube['robot_y'] if self.latest_red_cube else self.tracked_y
            hover = [target_x, target_y, self.z_hover,
                     FLANGE_POSTURE[0], FLANGE_POSTURE[1], FLANGE_POSTURE[2]]
            self.send_coords(hover, speed=25, model=1)
            print(f"[✓] Hovering over Red Cube at [X:{target_x:.1f}, Y:{target_y:.1f}, Z:{self.z_hover:.1f}] mm\n")
        finally:
            self.busy = False

    # ---------------------------------------------------------
    # RViz 3D Markers & Real-Time Visual Link
    # ---------------------------------------------------------
    def publish_rviz_markers(self):
        now = self.get_clock().now().to_msg()

        # 1. Grounded Safe Workspace Box
        reach_h = self.z_hover / 1000.0
        x_mid = (self.x_min + self.x_max) / 2000.0
        y_mid = (self.y_min + self.y_max) / 2000.0

        box = Marker()
        box.header.frame_id = "g_base"
        box.header.stamp = now
        box.ns = "safe_workspace"
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = x_mid
        box.pose.position.y = y_mid
        box.pose.position.z = reach_h / 2.0
        box.pose.orientation.w = 1.0
        box.scale.x = (self.x_max - self.x_min) / 1000.0
        box.scale.y = (self.y_max - self.y_min) / 1000.0
        box.scale.z = reach_h
        box.color = ColorRGBA(r=0.0, g=0.9, b=0.7, a=0.15)
        self.safe_area_pub.publish(box)

        # 2. Red Cube Marker & Dynamic Tracking Link
        if self.latest_red_cube and (time.time() - self.latest_red_cube['time'] < 2.0):
            rx = self.latest_red_cube['robot_x']
            ry = self.latest_red_cube['robot_y']
            safe = self.latest_red_cube['in_safe_zone']

            # Red Cube on desk
            m = Marker()
            m.header.frame_id = "g_base"
            m.header.stamp = now
            m.ns = "red_cube_target"
            m.id = 1
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = rx / 1000.0
            m.pose.position.y = ry / 1000.0
            m.pose.position.z = 0.0125  # Sitting flat on desk
            m.pose.orientation.w = 1.0
            m.scale.x = 0.025
            m.scale.y = 0.025
            m.scale.z = 0.025
            m.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.95) if safe else ColorRGBA(r=0.8, g=0.2, b=0.2, a=0.45)
            self.marker_pub.publish(m)

            # Dynamic Visual Link Line connecting robot hand hover to red cube
            link = Marker()
            link.header.frame_id = "g_base"
            link.header.stamp = now
            link.ns = "tracking_link"
            link.id = 3
            link.type = Marker.LINE_STRIP
            link.action = Marker.ADD
            link.scale.x = 0.004
            link.color = ColorRGBA(r=0.0, g=1.0, b=0.8, a=0.85) if safe else ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)

            p_cube = Point(x=rx / 1000.0, y=ry / 1000.0, z=0.025)
            p_hand = Point(x=self.tracked_x / 1000.0, y=self.tracked_y / 1000.0, z=self.z_hover / 1000.0)
            link.points = [p_cube, p_hand]
            self.marker_pub.publish(link)

            # 3D Text Label
            txt = Marker()
            txt.header.frame_id = "g_base"
            txt.header.stamp = now
            txt.ns = "red_cube_label"
            txt.id = 2
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = rx / 1000.0
            txt.pose.position.y = ry / 1000.0
            txt.pose.position.z = (self.z_hover / 1000.0) + 0.040
            txt.scale.z = 0.022
            txt.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=1.0)
            status_tag = "🔗 FOLLOWING" if (self.following_enabled and not self.has_cube_grasped) else "[LOCKED]"
            txt.text = f"🔴 RED CUBE {status_tag} ({int(rx)}, {int(ry)})"
            self.marker_pub.publish(txt)

    def print_diagnostics(self):
        """Prints live tracking and camera diagnostics."""
        print("\n" + "=" * 60)
        print("  🔍 LIVE TRACKING & VISION DIAGNOSTICS")
        print("=" * 60)
        if self.latest_frame is None:
            print("  Camera Frame:       [-] Not received yet on /camera/image_raw")
        else:
            h, w = self.latest_frame.shape[:2]
            print(f"  Camera Stream:      [✓] Active ({w}x{h})")

        if self.latest_red_cube and (time.time() - self.latest_red_cube['time'] < 2.0):
            c = self.latest_red_cube
            print(f"  Red Cube Detected:  [✓] YES (Confidence: {c['confidence'] * 100:.1f}%)")
            print(f"  Pixel Center:       ({c['pixel'][0]:.1f}, {c['pixel'][1]:.1f})")
            print(f"  Robot Coordinates:  X={c['robot_x']:.1f} mm, Y={c['robot_y']:.1f} mm")
            safe_str = f"YES (Within [{self.x_min:.0f}..{self.x_max:.0f}, {self.y_min:.0f}..{self.y_max:.0f}] mm)" if c['in_safe_zone'] else "NO (Out of bounds)"
            print(f"  Safe To Track:      {safe_str}")
        else:
            print("  Red Cube Detected:  [-] NOT currently detected on desk.")

        print(f"  Safe Bounds:        X=[{self.x_min:.0f}, {self.x_max:.0f}], Y=[{self.y_min:.0f}, {self.y_max:.0f}], R_max={self.max_radius:.0f} mm (+{self.hand_radius:.0f}mm hand radius)")
        print(f"  Hand Hover Pose:    X={self.tracked_x:.1f} mm, Y={self.tracked_y:.1f} mm, Z={self.z_hover:.1f} mm")
        print(f"  Follow Mode:        {'[ACTIVE]' if self.following_enabled else '[PAUSED]'}")
        print(f"  Auto-Grasp:         {'[ON]' if self.auto_capture_enabled else '[OFF]'}")
        print(f"  Claw Status:        {'[GRASPING RED CUBE]' if self.has_cube_grasped else '[OPEN / TRACKING]'}")
        print("=" * 60 + "\n")

    # ---------------------------------------------------------
    # Interactive CLI Loop
    # ---------------------------------------------------------
    def run_cli_loop(self):
        time.sleep(2.5)
        print("\n" + "=" * 65)
        print("      🔴 MYCOBOT 280 - COBRA MAGICIAN DYNAMIC RED TRACKER")
        print("=" * 65)
        print(" Controls:")
        print("   [c] or [Space] : 🎯 CAPTURE RED CUBE (Descend, grasp & lift)")
        print("   [r]            : ⬇️  RELEASE TO TABLE (Set down & resume tracking)")
        print("   [f]            : 🔄 TOGGLE DYNAMIC FOLLOW (Active / Paused)")
        print("   [a]            : ⚡ TOGGLE AUTO-GRASP (Grasp when stationary)")
        print("   [h]            : 📍 HOVER TEST (Re-center over cube)")
        print("   [i]            : ↔️  TOGGLE Y-AXIS REVERSE (Invert Lateral Polarity)")
        print("   [s]            : 🔍 LIVE VISION & TRACKING STATUS")
        print("   [o]            : 👁️  RETURN TO OBSERVATION STANDBY POSE")
        print("   [q]            : 🚪 QUIT PROGRAM")
        print("=" * 65)

        while rclpy.ok():
            try:
                cmd = input("\n[Cobra Tracker] Command [c/r/f/a/i/h/s/o/q]: ").strip().lower()
                if cmd in ('c', ' ', 'capture'):
                    if self.has_cube_grasped:
                        self.execute_release_to_table()
                    else:
                        self.execute_capture_red_cube()
                elif cmd in ('r', 'release'):
                    self.execute_release_to_table()
                elif cmd in ('f', 'follow'):
                    self.following_enabled = not self.following_enabled
                    state = "ACTIVE (Hand will follow Red Cube)" if self.following_enabled else "PAUSED"
                    print(f"[*] Dynamic Follow Mode: {state}")
                elif cmd in ('a', 'auto'):
                    self.auto_capture_enabled = not self.auto_capture_enabled
                    state = "ENABLED (Auto-grasps when cube is still)" if self.auto_capture_enabled else "DISABLED"
                    print(f"[*] Auto-Grasp: {state}")
                elif cmd in ('i', 'invert'):
                    self.invert_y = not self.invert_y
                    print(f"[*] Y-Axis Inverted: {self.invert_y}")
                elif cmd in ('h', 'hover'):
                    self.execute_hover_test()
                elif cmd in ('s', 'status', 'diag'):
                    self.print_diagnostics()
                elif cmd in ('o', 'observe'):
                    print("[*] Moving to observation pose...")
                    self.send_angles(DEFAULT_OBSERVE_ANGLES, speed=35)
                elif cmd in ('q', 'quit', 'exit'):
                    print("[*] Exiting Dynamic Tracking Program...")
                    os._exit(0)
            except (EOFError, KeyboardInterrupt):
                break


def main(args=None):
    rclpy.init(args=args)
    node = CaptureRedCubeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
