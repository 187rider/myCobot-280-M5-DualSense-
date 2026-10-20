#!/usr/bin/env python3
"""
Mac Host: PS5 DualSense Controller Streamer to ROS 2 (Docker)
Reads DualSense gamepad inputs via Bluetooth on macOS and streams
high-rate TCP packets to the myCobot ROS2 node inside Docker.
"""

import json
import os
import socket
import sys
import time

os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
import pygame


def main():
    bind_ip = "0.0.0.0"
    port = 9998

    pygame.init()
    pygame.joystick.init()

    while pygame.joystick.get_count() == 0:
        print("[*] Waiting for PS5 DualSense controller to connect via Bluetooth or USB...")
        time.sleep(2)
        pygame.joystick.quit()
        pygame.joystick.init()

    joy = pygame.joystick.Joystick(0)
    print(f"[+] Found Controller: {joy.get_name()}")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, 'SO_REUSEPORT'):
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
    server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    server.bind((bind_ip, port))
    server.listen(1)
    print(f"[+] TCP server listening on {bind_ip}:{port} for ROS 2 Docker node...")

    clock = pygame.time.Clock()
    deadzone = 0.10

    def filter_axis(val):
        if abs(val) < deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - deadzone) / (1.0 - deadzone)

    print("\n" + "=" * 60)
    print("      PS5 DualSense ➔ ROS 2 (myCobot 280) Teleoperation")
    print("=" * 60)
    print("  Left Stick       : J1 (Base) / J2 (Shoulder)  [or X/Y in Coords]")
    print("  Right Stick      : J3 (Elbow) / J4 (Wrist)    [or Z/Yaw in Coords]")
    print("  L2 / R2          : J5 (Wrist Roll)            [or Roll]")
    print("  L1 / R1          : J6 (Flange Roll)           [or Pitch]")
    print("  Options          : Switch JOINT <-> CARTESIAN Mode")
    print("  Triangle (▲)     : Go Home [0, 0, 0, 0, 0, 0]")
    print("  Cross (✖)        : Open Gripper")
    print("  Circle (🔴)      : Close Gripper")
    print("  L3 Click / Create: 🎯 1-CLICK CAMERA CALIBRATION ([k])")
    print("  R3 Click         : 📌 Save Fixed Tabletop Pick Spot ([s])")
    print("  Square (■)       : 🚀 Trigger Color Pick & Place")
    print("  D-Pad Up / Down  : Speed +/-")
    print("=" * 60 + "\n")

    while True:
        try:
            print("[*] Waiting for ROS 2 node in Docker to connect...")
            client, addr = server.accept()
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[+] ROS 2 Node connected from {addr}! Streaming active.")

            while True:
                clock.tick(30)  # 30 Hz
                pygame.event.pump()

                # Read Axes
                lx = filter_axis(joy.get_axis(0))
                ly = filter_axis(joy.get_axis(1))
                rx = filter_axis(joy.get_axis(2))
                ry = filter_axis(joy.get_axis(3))

                # Triggers L2 / R2 (range: -1.0 to 1.0 -> 0.0 to 1.0)
                l2_raw = joy.get_axis(4) if joy.get_numaxes() > 4 else -1.0
                r2_raw = joy.get_axis(5) if joy.get_numaxes() > 5 else -1.0
                l2 = max(0.0, (l2_raw + 1.0) / 2.0)
                r2 = max(0.0, (r2_raw + 1.0) / 2.0)

                # Read Buttons
                num_buttons = joy.get_numbuttons()
                buttons = {str(i): bool(joy.get_button(i)) for i in range(num_buttons)}

                # Bumpers L1 (btn 9), R1 (btn 10)
                l1 = joy.get_button(9) if num_buttons > 9 else 0
                r1 = joy.get_button(10) if num_buttons > 10 else 0

                # D-pad
                dpad = [
                    bool(joy.get_button(11)) if num_buttons > 11 else False,  # up
                    bool(joy.get_button(12)) if num_buttons > 12 else False,  # down
                    bool(joy.get_button(13)) if num_buttons > 13 else False,  # left
                    bool(joy.get_button(14)) if num_buttons > 14 else False,  # right
                ]
                if joy.get_numhats() > 0:
                    hat = joy.get_hat(0)
                    if hat[1] == 1:
                        dpad[0] = True
                    elif hat[1] == -1:
                        dpad[1] = True
                    if hat[0] == -1:
                        dpad[2] = True
                    elif hat[0] == 1:
                        dpad[3] = True

                payload = {
                    "axes": [round(lx, 3), round(ly, 3), round(rx, 3), round(ry, 3), round(l2, 3), round(r2, 3)],
                    "buttons": buttons,
                    "bumpers": [int(l1), int(r1)],
                    "dpad": [int(x) for x in dpad],
                }

                line = json.dumps(payload) + "\n"
                try:
                    client.sendall(line.encode("utf-8"))
                except (BrokenPipeError, ConnectionResetError):
                    print("\n[-] ROS 2 node disconnected.")
                    break

                # Terminal Live Status with Robot USB Detection
                now_check = time.time()
                if not hasattr(main, "last_port_check") or now_check - main.last_port_check > 1.0:
                    main.last_port_check = now_check
                    import glob
                    found_ports = glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wch*") + glob.glob("/dev/cu.SLAB*")
                    main.detected_port = found_ports[0].replace("/dev/cu.", "") if found_ports else None

                if main.detected_port:
                    status_str = f"✅ Arm: {main.detected_port}"
                else:
                    status_str = "⚠️  Arm: DISCONNECTED (Check USB-C & 12V Power!)"

                print(f"\r[Streaming] LX:{lx:+.2f} LY:{ly:+.2f} | RX:{rx:+.2f} RY:{ry:+.2f} | {status_str}  ", end="", flush=True)

            client.close()

        except KeyboardInterrupt:
            print("\n\n[+] DualSense Streamer stopped.")
            break
        except Exception as e:
            print(f"[-] Error: {e}")
            time.sleep(1)

    server.close()
    pygame.quit()


if __name__ == "__main__":
    main()
