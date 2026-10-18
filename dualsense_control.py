#!/usr/bin/env python3
"""
myCobot 280 M5 - PS5 DualSense Real-time Teleoperation
Controls the robot arm using a Sony PlayStation 5 DualSense Wireless Controller.
"""

import glob
import os
import sys
import time

# Allow pygame to receive background gamepad events even if window loses focus
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
import pygame
from pymycobot import MyCobot280


def find_cobot_port():
    patterns = [
        "/dev/cu.usbserial*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.SLAB_USBtoUART*",
    ]
    for p in patterns:
        matches = glob.glob(p)
        if matches:
            return matches[0]
    return None


class DualSenseCobotController:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        # 1. Connect Controller
        if pygame.joystick.get_count() == 0:
            print("[-] No Gamepad detected! Please connect your PS5 DualSense via Bluetooth or USB.")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        print(f"[+] Connected to Gamepad: {self.joystick.get_name()}")

        # 2. Connect Robot
        print("[*] Looking for myCobot 280 on USB serial...")
        port = find_cobot_port()
        while not port:
            print("    Waiting for myCobot to be plugged in and powered on... (Press Ctrl+C to cancel)", end="\r")
            time.sleep(1)
            port = find_cobot_port()

        print(f"\n[+] Found myCobot at: {port}")
        self.mc = MyCobot280(port, 115200)
        time.sleep(1)

        # Set fresh mode (1 = new commands immediately overwrite old ones for smooth teleop)
        try:
            self.mc.set_fresh_mode(1)
        except Exception:
            pass

        # State variables
        self.mode = "JOINT"  # "JOINT" or "COORDS"
        self.speed = 25      # Speed percentage (10 - 80)
        self.gripper_state = None
        self.servos_locked = True
        self.deadzone = 0.12

        # Cache angles and coords
        self.current_angles = self.mc.get_angles()
        if not isinstance(self.current_angles, list) or len(self.current_angles) != 6:
            self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.current_coords = self.mc.get_coords()
        if not isinstance(self.current_coords, list) or len(self.current_coords) != 6:
            self.current_coords = [150.0, 0.0, 200.0, -180.0, 0.0, 0.0]

        # Initial LED: Blue for JOINT mode
        self.update_led()
        self.print_instructions()

    def update_led(self):
        try:
            if self.mode == "JOINT":
                self.mc.set_color(0, 100, 255)  # Blue
            else:
                self.mc.set_color(0, 255, 100)  # Green
        except Exception:
            pass

    def print_instructions(self):
        print("\n" + "=" * 65)
        print("          myCobot 280 M5 - PS5 DualSense Controls")
        print("=" * 65)
        print(" [Options Button]       : Switch Mode (JOINT <-> CARTESIAN XYZ)")
        print(" [Triangle ▲]           : Return to Origin (Home [0,0,0,0,0,0])")
        print(" [Cross ✖ / Circle 🔴]  : Open / Close Gripper")
        print(" [Square ◼]             : Stop Motion / Lock Servos")
        print(" [D-Pad Up / Down]      : Increase / Decrease Speed")
        print(" [Touchpad / Share]     : Relax Servos (Free hand motion)")
        print("-" * 65)
        print(" JOINT MODE (Blue LED):")
        print("   Left Stick X / Y     : Joint 1 (Base) / Joint 2 (Shoulder)")
        print("   Right Stick Y / X    : Joint 3 (Elbow) / Joint 4 (Wrist Pitch)")
        print("   L2 / R2 Triggers     : Joint 5 (Wrist Roll)")
        print("   L1 / R1 Bumpers      : Joint 6 (Flange Roll)")
        print("-" * 65)
        print(" CARTESIAN MODE (Green LED):")
        print("   Left Stick X / Y     : X axis (Forward / Back) / Y axis (Left / Right)")
        print("   Right Stick Y / X    : Z axis (Up / Down) / Rz (Yaw)")
        print("   L2 / R2 Triggers     : Rx (Roll)")
        print("   L1 / R1 Bumpers      : Ry (Pitch)")
        print("=" * 65 + "\n")

    def apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self.deadzone) / (1.0 - self.deadzone)

    def run(self):
        clock = pygame.time.Clock()
        last_sync_time = time.time()
        last_send_time = time.time()

        try:
            while True:
                clock.tick(30)  # 30 Hz control loop
                pygame.event.pump()

                # Process Button Events
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        btn = event.button
                        # 0: Cross, 1: Circle, 2: Square, 3: Triangle
                        if btn == 3:  # Triangle -> Go Home
                            print("\n[*] Moving to Home Posture...")
                            self.mc.send_angles([0, 0, 0, 0, 0, 0], 25)
                            time.sleep(1.5)
                            self.current_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                        elif btn == 0:  # Cross -> Open Gripper
                            print("\n[*] Opening Gripper...")
                            try:
                                self.mc.set_gripper_state(0, 70)
                            except Exception:
                                pass

                        elif btn == 1:  # Circle -> Close Gripper
                            print("\n[*] Closing Gripper...")
                            try:
                                self.mc.set_gripper_state(1, 70)
                            except Exception:
                                pass

                        elif btn == 2:  # Square -> Stop
                            print("\n[!] Emergency Stop.")
                            self.mc.stop()
                            self.mc.focus_all_servos()

                        elif btn == 6:  # Options -> Toggle Mode
                            self.mode = "COORDS" if self.mode == "JOINT" else "JOINT"
                            print(f"\n[+] Switched to {self.mode} MODE")
                            self.update_led()
                            # Refresh state
                            if self.mode == "JOINT":
                                angles = self.mc.get_angles()
                                if isinstance(angles, list) and len(angles) == 6:
                                    self.current_angles = angles
                            else:
                                coords = self.mc.get_coords()
                                if isinstance(coords, list) and len(coords) == 6:
                                    self.current_coords = coords

                        elif btn in (4, 15):  # Share or Touchpad -> Relax / Lock servos
                            self.servos_locked = not self.servos_locked
                            if not self.servos_locked:
                                print("\n[*] Servos RELAXED (manual damping mode)")
                                self.mc.release_all_servos()
                            else:
                                print("\n[*] Servos LOCKED")
                                self.mc.focus_all_servos()

                        elif btn == 11:  # D-Pad Up -> Speed +
                            self.speed = min(80, self.speed + 5)
                            print(f"\n[+] Speed: {self.speed}%")

                        elif btn == 12:  # D-Pad Down -> Speed -
                            self.speed = max(5, self.speed - 5)
                            print(f"\n[-] Speed: {self.speed}%")

                if not self.servos_locked:
                    continue

                # Read DualSense Analog Axes
                lx = self.apply_deadzone(self.joystick.get_axis(0))
                ly = self.apply_deadzone(self.joystick.get_axis(1))
                rx = self.apply_deadzone(self.joystick.get_axis(2))
                ry = self.apply_deadzone(self.joystick.get_axis(3))

                # Triggers L2 / R2 (axes 4 and 5 on macOS: range -1.0 to 1.0)
                l2_raw = self.joystick.get_axis(4)
                r2_raw = self.joystick.get_axis(5)
                l2 = max(0.0, (l2_raw + 1.0) / 2.0)
                r2 = max(0.0, (r2_raw + 1.0) / 2.0)
                trigger_diff = r2 - l2
                if abs(trigger_diff) < 0.1:
                    trigger_diff = 0.0

                # Bumpers L1 / R1
                l1 = self.joystick.get_button(9) if self.joystick.get_numbuttons() > 9 else 0
                r1 = self.joystick.get_button(10) if self.joystick.get_numbuttons() > 10 else 0
                bumper_diff = (1.0 if r1 else 0.0) - (1.0 if l1 else 0.0)

                # Send commands at max 20 Hz rate
                now = time.time()
                if now - last_send_time >= 0.05:
                    last_send_time = now

                    if self.mode == "JOINT":
                        # Multiplier for degrees per update
                        step = (self.speed / 25.0) * 2.5
                        moved = False

                        # J1: Left Stick X
                        if lx != 0:
                            self.current_angles[0] = max(-165.0, min(165.0, self.current_angles[0] - lx * step))
                            moved = True
                        # J2: Left Stick Y
                        if ly != 0:
                            self.current_angles[1] = max(-135.0, min(135.0, self.current_angles[1] - ly * step))
                            moved = True
                        # J3: Right Stick Y
                        if ry != 0:
                            self.current_angles[2] = max(-145.0, min(145.0, self.current_angles[2] - ry * step))
                            moved = True
                        # J4: Right Stick X
                        if rx != 0:
                            self.current_angles[3] = max(-145.0, min(145.0, self.current_angles[3] + rx * step))
                            moved = True
                        # J5: L2 / R2
                        if trigger_diff != 0:
                            self.current_angles[4] = max(-160.0, min(160.0, self.current_angles[4] + trigger_diff * step))
                            moved = True
                        # J6: L1 / R1
                        if bumper_diff != 0:
                            self.current_angles[5] = max(-175.0, min(175.0, self.current_angles[5] + bumper_diff * step))
                            moved = True

                        if moved:
                            self.mc.send_angles(self.current_angles, self.speed)
                            angles_str = ", ".join(f"{a:5.1f}°" for a in self.current_angles)
                            print(f"\r[JOINT] {angles_str} | Spd: {self.speed}%  ", end="", flush=True)

                    else:  # CARTESIAN XYZ MODE
                        step = (self.speed / 25.0) * 3.5
                        moved = False

                        # X: Left Stick Y (forward/back)
                        if ly != 0:
                            self.current_coords[0] += -ly * step
                            moved = True
                        # Y: Left Stick X (left/right)
                        if lx != 0:
                            self.current_coords[1] += lx * step
                            moved = True
                        # Z: Right Stick Y (up/down)
                        if ry != 0:
                            self.current_coords[2] += -ry * step
                            moved = True
                        # Rz (Yaw): Right Stick X
                        if rx != 0:
                            self.current_coords[5] += rx * step
                            moved = True
                        # Rx (Roll): L2 / R2
                        if trigger_diff != 0:
                            self.current_coords[3] += trigger_diff * step
                            moved = True
                        # Ry (Pitch): L1 / R1
                        if bumper_diff != 0:
                            self.current_coords[4] += bumper_diff * step
                            moved = True

                        if moved:
                            self.mc.send_coords(self.current_coords, self.speed, 0)
                            c_str = f"X:{self.current_coords[0]:5.1f} Y:{self.current_coords[1]:5.1f} Z:{self.current_coords[2]:5.1f}"
                            print(f"\r[XYZ]   {c_str} | Spd: {self.speed}%  ", end="", flush=True)

                # Periodic state resync every 3 seconds if idle
                if now - last_sync_time > 3.0:
                    last_sync_time = now
                    try:
                        if self.mode == "JOINT":
                            live_angles = self.mc.get_angles()
                            if isinstance(live_angles, list) and len(live_angles) == 6:
                                self.current_angles = live_angles
                        else:
                            live_coords = self.mc.get_coords()
                            if isinstance(live_coords, list) and len(live_coords) == 6:
                                self.current_coords = live_coords
                    except Exception:
                        pass

        except KeyboardInterrupt:
            print("\n\n[+] Exiting DualSense Teleoperation.")
        finally:
            try:
                self.mc.set_color(0, 0, 0)  # Turn off LED
            except Exception:
                pass
            pygame.quit()


if __name__ == "__main__":
    controller = DualSenseCobotController()
    controller.run()
