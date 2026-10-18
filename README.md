# myCobot 280 M5 - DualSense Teleoperation & ROS 2 Digital Twin

Comprehensive real-time teleoperation system for the **Elephant Robotics myCobot 280 M5** collaborative robot arm on **Apple Silicon (macOS arm64)**. Features low-latency wireless control using a **Sony PS5 DualSense controller** via Bluetooth, sub-5ms serial communication, continuous velocity motion, and a real-time **ROS 2 Humble + MoveIt 2 + RViz 2** 3D digital twin running inside an optimized Docker environment.

---

## 🌟 Key Features

- **🎮 DualSense Teleoperation**: Wireless Bluetooth control with analog stick velocity integration, dual triggers for wrist roll, and bumpers for flange rotation.
- **⚡ Ultra Low-Latency Pipeline**: Asynchronous serial command pipeline (`_async=True`) with `TCP_NODELAY` and lock contention fixes eliminating lag (<10ms end-to-end response).
- **🌊 Fluid, Continuous Motion**: Time-delta velocity integration (`dt`) prevents stuttering and discrete "stepping" stops between setpoints.
- **🖥️ 3D Digital Twin in RViz 2**: Continuous 30 Hz `/joint_states` transform broadcasting inside a web-accessible Ubuntu desktop (`http://localhost:6080`).
- **🔄 Auto-Reconnecting Serial Bridge**: Automatic hot-plug USB detection on macOS, bridging the hardware UART to Docker's `/dev/ttyUSB0` via background daemons (`supervisor` + `socat`).
- **🔀 Dual Operating Modes**:
  1. **Joint Mode**: Direct 6-DOF joint angle control.
  2. **Cartesian Mode**: Intuitive XYZ tool-tip translation and RPY orientation.

---

## 🏗️ System Architecture

```
[ PS5 DualSense Controller ]
             │  (Bluetooth HID)
             ▼
[ Mac Host: dualsense_to_ros2.py ] ──(TCP :9998)──▶ [ Docker: teleop_dualsense.py (ROS 2) ]
                                                                      │
                                                ┌─────────────────────┴─────────────────────┐
                                                ▼                                           ▼
                                        [ RViz 2 3D Model ]                     [ listen_real_service ]
                                        (/joint_states 30Hz)                                │
                                                                                            ▼
                                                                                    (/dev/ttyUSB0 pty)
                                                                                            │
[ myCobot 280 M5 Arm ] ◀──(USB UART)── [ Mac Host: bridge_serial.py ] ◀──(TCP :5005)── [ socat ]
```

---

## ⚡ Hardware Checklist (Before Starting)

1. **12V DC Power**: Connect the 12V / 5A DC power brick to the round barrel jack on the robot's base. Turn the power switch **ON**. *(The 6 servo motors require 12V to generate torque; 5V USB alone only powers the screen).*
2. **USB-C Data Cable**: Connect the USB-C cable from your Mac to the **M5Stack Basic (base)**, **NOT** the head/Atom end-effector.
3. **M5Stack Basic Mode**: On the base screen, select **`Transponder`** mode (verify it shows `Atom: OK` and `115200`).
4. **DualSense Controller**: Pair your PS5 controller to your Mac via Bluetooth (`Settings > Bluetooth`).

---

## 🚀 Quick Start Guide

### Mode A: Full ROS 2 + RViz 3D Twin (Recommended)

#### 1. Start the Host Services (Mac Terminal)
```bash
cd ros2_docker
./start_ros2_dualsense.sh
```
This automatically:
- Boots the ROS 2 Docker container.
- Connects the USB serial bridge to the robot (`/dev/cu.usbserial-*` ➔ TCP 5005 ➔ `/dev/ttyUSB0`).
- Connects to your DualSense controller and starts streaming inputs over TCP 9998.

*Live terminal output displays real-time stick inputs and robot connection status:*
```text
[Streaming] LX:+0.00 LY:+0.00 | RX:+0.00 RY:+0.00 | ✅ Arm: usbserial-XXXXXXXX
```

#### 2. Launch ROS 2 in Web Desktop
1. Open your browser to **[http://localhost:6080](http://localhost:6080)** (password: `ubuntu`).
2. Open the terminal inside the Web Desktop and run:
```bash
source /opt/ros2_ws/install/setup.bash
ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200
```
*RViz 2 will launch showing the 3D myCobot 280 digital twin animating simultaneously with the physical arm!*

---

### Mode B: Direct Host-Only Python Control (No Docker / No ROS)

For quick testing or lightweight headless scripting directly on macOS:
```bash
./run_dualsense.sh
```

---

## 🎮 Controller Button Mapping

| Button / Control | Joint Mode | Cartesian (XYZ) Mode |
| :--- | :--- | :--- |
| **Left Stick X** | **J1** (Base Rotation) | **Y** (Left / Right) |
| **Left Stick Y** | **J2** (Shoulder Pitch) | **X** (Forward / Backward) |
| **Right Stick Y** | **J3** (Elbow Pitch) | **Z** (Up / Down) |
| **Right Stick X** | **J4** (Wrist Pitch) | **Rz** (Yaw Rotation) |
| **L2 / R2 Triggers** | **J5** (Wrist Roll) | **Rx** (Roll Tilt) |
| **L1 / R1 Bumpers** | **J6** (Flange Tool Roll) | **Ry** (Pitch Tilt) |
| **Options Button** | Switch to Cartesian Mode | Switch to Joint Mode |
| **Triangle (▲)** | Move to Home `[0, 0, 0, 0, 0, 0]` | Move to Home |
| **Cross (✖)** | Open Gripper | Open Gripper |
| **Circle (🔴)** | Close Gripper | Close Gripper |
| **D-Pad Up / Down** | Increase / Decrease Speed | Increase / Decrease Speed |

---

## 📁 Repository Structure

```text
cobot/
├── README.md                           # Main repository documentation
├── .gitignore                          # Git ignore rules (virtualenvs, binaries, caches)
├── dualsense_control.py                # Direct macOS host teleoperation (no Docker)
├── run_dualsense.sh                    # Direct teleoperation launcher
└── ros2_docker/                        # ROS 2 Humble containerized environment
    ├── README.md                       # ROS 2 specific guide
    ├── Dockerfile                      # Ubuntu 22.04 + ROS 2 Humble + MoveIt 2 + noVNC
    ├── docker-compose.yml              # Container runtime configuration
    ├── cobot.conf                      # Supervisor configuration for automatic socat bridge
    ├── bridge_serial.py                # Host USB UART <-> TCP serial forwarder
    ├── dualsense_to_ros2.py            # Host DualSense Bluetooth <-> TCP streamer
    ├── start_ros2_cobot.sh             # Generic ROS 2 stack launcher
    ├── start_ros2_dualsense.sh         # One-click DualSense + ROS 2 launcher
    └── custom_nodes/
        ├── teleop_dualsense.py         # ROS 2 Python Node (continuous velocity & RViz TF)
        └── teleop_dualsense.launch.py  # ROS 2 Launch file (loads URDF, robot_state_publisher, RViz2)
```

---

## 🛠️ Diagnostics & Troubleshooting

| Issue | Cause | Fix |
| :--- | :--- | :--- |
| **RViz moves but physical servos don't** | 12V power supply disconnected or USB cable in top Atom port | Plug 12V 5A power into round DC jack at base and switch ON; plug USB-C into base M5Stack. |
| **`⚠️ Arm: DISCONNECTED` on terminal** | USB cable loose or M5Stack not in Transponder mode | Re-seat USB-C cable; select `Transponder` on screen (`Atom: OK`). |
| **Motion feels stepped / jerky** | Blocking serial replies | Ensured `_async=True` and continuous `dt` velocity integration in `teleop_dualsense.py`. |
| **High latency / ping** | Nagle's algorithm & serial bus polling collisions | Enabled `TCP_NODELAY` and disabled redundant 50Hz serial polling loop in `listen_real_service.py`. |
| **Container shutdown** | Ending session | Run `docker compose down` inside `ros2_docker/`. |

---

## 📜 License
MIT License. Built for use with Elephant Robotics myCobot 280 M5.
