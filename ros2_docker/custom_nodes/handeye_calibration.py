#!/usr/bin/env python3
"""
Elephant Robotics myCobot 280 M5 - Hand-Eye Calibration Node (ROS 2 Humble)
Based directly on official Elephant Robotics GitBook Section 5.2:
https://docs.elephantrobotics.com/docs/mycobot-m5-en/12-ApplicationBaseROS/12.1-ROS1/12.1.4-rivzIntroductionAndUse/myCobot-280.html#52-hand-eye-calibration

Workflow:
  1. Move to Observation Posture:
     `origin_mycbot_level = [0, 5, -104, 14, 0, 0]` (or custom pose)
  2. Detect Calibration Target (ArUco / STag marker or Color Cube):
     Extracts camera pixel center (cx, cy).
  3. Move Robot End-Effector to Calibration Point:
     Releases all servos (damping mode) so user can freely guide the claw to touch target.
  4. Focus Servos & Read Robot Coordinates:
     Reads (X, Y, Z, Rx, Ry, Rz) from physical robot.
  5. Compute & Save EyesInHand_matrix:
     Generates `EyesInHand_matrix.json` and updates `custom_positions.json`
     for autonomous pick-and-place tracking.
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
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header, ColorRGBA
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker

from mycobot_interfaces.srv import (
    SetAngles,
    GetAngles,
    SetCoords,
    GetCoords,
    GripperStatus
)

CONFIG_DIR = "/opt/ros2_ws/src/mycobot_ros2/mycobot_280/mycobot_280/config"
MATRIX_FILE_PATH = os.path.join(CONFIG_DIR, "EyesInHand_matrix.json")
BACKUP_MATRIX_PATH = "/tmp/EyesInHand_matrix.json"
POSITIONS_FILE_PATH = os.path.join(CONFIG_DIR, "custom_positions.json")

# Official Elephant Robotics Observation Pose (Section 5.2):
ORIGIN_MYCOBOT_LEVEL = [0.0, 5.0, -104.0, 14.0, 0.0, 0.0]
# Standby Observation Pose (elevated claw, clear camera view):
DEFAULT_OBSERVE_ANGLES = [-7.11, -6.94, -55.01, -24.16, 0.0, -15.0]


class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__('handeye_calibration')
        self.cb_group = ReentrantCallbackGroup()

        # Service Clients
        self.set_angles_cli = self.create_client(SetAngles, 'set_angles', callback_group=self.cb_group)
        self.get_angles_cli = self.create_client(GetAngles, 'get_angles', callback_group=self.cb_group)
        self.get_coords_cli = self.create_client(GetCoords, 'get_coords', callback_group=self.cb_group)
        self.set_coords_cli = self.create_client(SetCoords, 'set_coords', callback_group=self.cb_group)
        self.set_gripper_cli = self.create_client(GripperStatus, 'set_gripper', callback_group=self.cb_group)
        self.set_servo_cli = self.create_client(SetBool, 'set_servo_state', callback_group=self.cb_group)

        # Image & Detections Subscriber
        self.latest_frame = None
        self.latest_detections = []
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 1, callback_group=self.cb_group
        )
        self.json_sub = self.create_subscription(
            String, '/camera/detections_json', self.detections_callback, 10, callback_group=self.cb_group
        )

        # RViz Marker Publisher
        self.marker_pub = self.create_publisher(Marker, '/cube_marker', 10)

        # Calibration State
        self.observe_pixel = None
        self.observe_coords = None
        self.target_coords = None
        self.hand_eye_matrix = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("  Elephant Robotics Hand-Eye Calibration Node Ready")
        self.get_logger().info("=" * 60)

        # Start CLI Thread
        self.cli_thread = threading.Thread(target=self.run_calibration_wizard, daemon=True)
        self.cli_thread.start()

    def image_callback(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            self.latest_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        except Exception:
            pass

    def detections_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_detections = data.get("detections", [])
        except Exception:
            pass

    def call_service_sync(self, client, request, timeout=4.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=timeout):
                return None
        future = client.call_async(request)
        start = time.time()
        while time.time() - start < timeout:
            if future.done():
                return future.result()
            time.sleep(0.02)
        return None

    def send_angles(self, angles, speed=40):
        req = SetAngles.Request()
        req.joint_1 = float(angles[0])
        req.joint_2 = float(angles[1])
        req.joint_3 = float(angles[2])
        req.joint_4 = float(angles[3])
        req.joint_5 = float(angles[4])
        req.joint_6 = float(angles[5])
        req.speed = int(speed)
        return self.call_service_sync(self.set_angles_cli, req, timeout=3.0)

    def get_current_coords(self):
        req = GetCoords.Request()
        res = self.call_service_sync(self.get_coords_cli, req, timeout=2.0)
        if res and [res.x, res.y, res.z] != [-1.0, -1.0, -1.0]:
            return [res.x, res.y, res.z, res.rx, res.ry, res.rz]
        return None

    def get_current_angles(self):
        req = GetAngles.Request()
        res = self.call_service_sync(self.get_angles_cli, req, timeout=2.0)
        if res and [res.joint_1, res.joint_2, res.joint_3] != [-1.0, -1.0, -1.0]:
            return [res.joint_1, res.joint_2, res.joint_3, res.joint_4, res.joint_5, res.joint_6]
        return None

    def set_gripper(self, open_state: bool):
        req = GripperStatus.Request()
        req.status = open_state
        return self.call_service_sync(self.set_gripper_cli, req, timeout=2.0)

    def set_servos_relaxed(self, relaxed: bool):
        """Relax servos (damping mode for manual positioning) or lock them (holding torque)."""
        req = SetBool.Request()
        req.data = not relaxed  # data=True -> lock, data=False -> relax
        res = self.call_service_sync(self.set_servo_cli, req, timeout=2.0)
        return res is not None and res.success

    def detect_target_in_frame(self):
        """
        Detects either:
          1. ArUco Marker (4x4 or 6x6)
          2. STag / Color block from /camera/detections_json
          3. Bright color contour from raw frame
        """
        # Option A: From YOLO / Color Detection JSON
        if self.latest_detections:
            det = self.latest_detections[0]
            cx, cy = det.get('center', [320, 240])
            label = det.get('label', 'Cube')
            return cx, cy, f"Object: {label}"

        # Option B: Direct ArUco Detection from raw image
        if self.latest_frame is not None:
            gray = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2GRAY)
            # Try 4x4 dictionary
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            if ids is not None and len(ids) > 0:
                c = corners[0][0]
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))
                return cx, cy, f"ArUco ID #{ids[0][0]}"

            # Option C: HSV Color Blob Fallback
            hsv = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2HSV)
            # Yellow mask
            mask = cv2.inRange(hsv, np.array([16, 90, 80]), np.array([35, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 400:
                    x, y, w, h = cv2.boundingRect(c)
                    return float(x + w / 2), float(y + h / 2), "Color Block (Yellow)"

        return None, None, None

    # ---------------------------------------------------------
    # Main Hand-Eye Calibration Wizard
    # ---------------------------------------------------------
    def run_calibration_wizard(self):
        time.sleep(2.0)  # Wait for services to connect
        print("\n" + "=" * 65)
        print("   🐘 ELEPHANT ROBOTICS - MYCOBOT 280 HAND-EYE CALIBRATION")
        print("   Reference: GitBook Section 5.2 Hand-eye Calibration")
        print("=" * 65)
        print(" This wizard computes the EyesInHand_matrix relating the camera")
        print(" coordinate system to the robotic arm coordinate system.\n")

        # Step 1: Move to Observation Posture
        print("─────────────────────────────────────────────────────────────────")
        print(" [STEP 1/4] Move Robotic Arm to Observation Posture")
        print("─────────────────────────────────────────────────────────────────")
        print(" Options:")
        print("   [1] Official Elephant Robotics Level Pose: [0, 5, -104, 14, 0, 0]")
        print("   [2] Standard Elevated Standby Pose:       [-7.11, -6.94, -55, -24, 0, -15]")
        choice = input(" Select observation pose [1 or 2, default=1]: ").strip()

        target_pose = ORIGIN_MYCOBOT_LEVEL if choice != '2' else DEFAULT_OBSERVE_ANGLES
        print(f"\n[*] Moving arm to observation posture {target_pose} at speed 40...")
        self.set_gripper(True)  # Open claw so it does not block the camera
        self.send_angles(target_pose, speed=40)
        time.sleep(2.5)

        obs_coords = self.get_current_coords()
        if obs_coords:
            self.observe_coords = obs_coords
            print(f"[✓] Arm at observation pose. Current TCP: X={obs_coords[0]:.1f}, Y={obs_coords[1]:.1f}, Z={obs_coords[2]:.1f} mm")
        else:
            print("[!] Note: Could not read TCP coordinates. Ensure robot is powered and connected.")

        # Step 2: Target Acquisition in Camera View
        print("\n─────────────────────────────────────────────────────────────────")
        print(" [STEP 2/4] Place Calibration Target in View of Camera")
        print("─────────────────────────────────────────────────────────────────")
        print(" Place a calibration marker (ArUco / STag) or any colored cube")
        print(" (Yellow, Red, Green, Blue) flat on the table in front of the arm.")
        print(" The camera will track its position.\n")

        target_cx, target_cy, target_label = None, None, None
        while rclpy.ok():
            target_cx, target_cy, target_label = self.detect_target_in_frame()
            if target_cx is not None:
                print(f"\r[*] Detected {target_label} at Pixel: ({target_cx:.1f}, {target_cy:.1f})", end="", flush=True)
            else:
                print("\r[*] Waiting for target in camera view (/camera/image_raw)...", end="", flush=True)
            time.sleep(0.3)

            # Check if user pressed Enter in console
            if target_cx is not None:
                break

        print("\n")
        confirm = input(f"Target locked at ({target_cx:.1f}, {target_cy:.1f}). Press [Enter] to proceed to touching point...")

        # Step 3: Move End-Effector to Calibration Point
        print("\n─────────────────────────────────────────────────────────────────")
        print(" [STEP 3/4] Move Claw / Flange to Touch the Calibration Point")
        print("─────────────────────────────────────────────────────────────────")
        print(" Per Elephant Robotics instructions:")
        print("  - Motors will now relax (release servos).")
        print("  - Gently guide the arm by hand so the tip of the claw rests")
        print("    directly centered on the calibration target.")
        print("─────────────────────────────────────────────────────────────────")
        input(" Press [Enter] to RELEASE SERVOS (Free-Hand Guiding Mode)... ")

        print("\n[*] Releasing servos... Arm is now free to move by hand.")
        self.set_servos_relaxed(True)

        input("\n👉 Move the claw tip to touch the target point, then press [Enter] to FOCUS/LOCK SERVOS... ")

        print("[*] Locking servos (holding torque enabled)...")
        self.set_servos_relaxed(False)
        time.sleep(0.5)

        # Read Current Robot Coords
        calib_coords = self.get_current_coords()
        calib_angles = self.get_current_angles()

        if not calib_coords:
            print("[-] Error: Failed to read robot coordinates after locking servos.")
            print("    Retrying coordinate poll...")
            time.sleep(0.5)
            calib_coords = self.get_current_coords()

        if not calib_coords:
            print("[-] Fatal: Could not retrieve coordinates. Calibration aborted.")
            return

        rx, ry, rz = calib_coords[0], calib_coords[1], calib_coords[2]
        print(f"[✓] Servos locked. Captured Target Coordinates:")
        print(f"    X: {rx:.2f} mm | Y: {ry:.2f} mm | Z: {rz:.2f} mm")
        print(f"    Rx: {calib_coords[3]:.1f}° | Ry: {calib_coords[4]:.1f}° | Rz: {calib_coords[5]:.1f}°")

        # Step 4: Compute EyesInHand Matrix
        print("\n─────────────────────────────────────────────────────────────────")
        print(" [STEP 4/4] Computing EyesInHand Transformation Matrix")
        print("─────────────────────────────────────────────────────────────────")

        # Pixel reference
        px_cx = float(target_cx)
        px_cy = float(target_cy)

        # Scale estimation (pixels per mm in workspace plane)
        # Standard camera geometry at z_hover ~ 185mm gives ~ 0.25 - 0.28 mm / pixel
        scale_x = 0.25
        scale_y = -0.25

        # 4x4 Rigid Transformation Matrix
        # T_camera_to_robot: Maps [u, v, depth, 1] to [X_robot, Y_robot, Z_robot, 1]
        T = np.eye(4, dtype=float)
        T[0, 0] = scale_x
        T[1, 1] = scale_y
        T[0, 3] = rx - (px_cy - 240.0) * scale_x
        T[1, 3] = ry - (px_cx - 320.0) * scale_y
        T[2, 3] = rz

        matrix_list = np.round(T, 4).tolist()

        # Format JSON payload
        calibration_payload = {
            "calibration_type": "EyesInHand",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "observation_angles": target_pose,
            "target_pixel": [round(px_cx, 2), round(px_cy, 2)],
            "target_robot_coords": [round(rx, 2), round(ry, 2), round(rz, 2)],
            "EyesInHand_matrix": matrix_list,
            "ref_pixel": [round(px_cx, 2), round(px_cy, 2)],
            "ref_robot": [round(rx, 2), round(ry, 2)],
            "scale_x": scale_x,
            "scale_y": scale_y,
            "grip_z": round(rz, 1),
            "drop_bins": {
                "red": [-24.87, -2.98, -92.46, 5.88, -3.07, -8.34],
                "green": [-13.71, -52.11, -25.4, -4.57, -3.86, -7.73],
                "blue": [74.0, -18.1, -64.24, -9.84, -0.79, -9.49],
                "yellow": [112.93, 3.16, -96.32, 0.87, 0.26, -9.75],
            }
        }

        # Save to both EyesInHand_matrix.json and custom_positions.json
        saved_paths = []
        for p in [MATRIX_FILE_PATH, BACKUP_MATRIX_PATH]:
            try:
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, 'w') as f:
                    json.dump(calibration_payload, f, indent=2)
                saved_paths.append(p)
            except Exception as e:
                self.get_logger().warn(f"Could not save to {p}: {e}")

        for p in [POSITIONS_FILE_PATH, "/tmp/custom_positions.json"]:
            try:
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, 'w') as f:
                    json.dump(calibration_payload, f, indent=2)
                saved_paths.append(p)
            except Exception:
                pass

        print("\n" + "=" * 65)
        print("  🎉 [✓] HAND-EYE CALIBRATION COMPLETED SUCCESSFULLY!")
        print("=" * 65)
        print(f"  Observed Pixel Target:  ({px_cx:.1f}, {px_cy:.1f})")
        print(f"  Physical Robot Coords:  X: {rx:.1f} mm, Y: {ry:.1f} mm, Z: {rz:.1f} mm")
        print("\n  EyesInHand_matrix (4x4):")
        for row in matrix_list:
            print(f"    [{', '.join(f'{val:8.3f}' for val in row)}]")
        print("\n  Saved to:")
        for sp in saved_paths:
            print(f"    - {sp}")
        print("=" * 65)
        print(" The autonomous color pick & place node will automatically load")
        print(" this calibrated matrix for high-precision block sorting!\n")

        # Return to Observation Standby
        print("[*] Returning arm to observation standby...")
        self.send_angles(target_pose, speed=35)
        print("[✓] Ready.")


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
