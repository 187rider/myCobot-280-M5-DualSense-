# myCobot 280 M5 - DualSense Teleoperation & ROS 2 Digital Twin

Comprehensive real-time teleoperation system for the **Elephant Robotics myCobot 280 M5** collaborative robot arm on **Apple Silicon (macOS arm64)**. Features low-latency wireless control using a **Sony PS5 DualSense controller** via Bluetooth, sub-5ms serial communication, continuous velocity motion, live Mac webcam streaming into ROS 2 (`/camera/image_raw`), and a real-time **ROS 2 Humble + MoveIt 2 + RViz 2** 3D digital twin running inside an optimized Docker environment.

---

## 🌟 Key Features

- **🎮 DualSense Wireless Teleoperation**: Bluetooth control with analog stick velocity integration, dual triggers for wrist roll, and bumpers for flange rotation. Automatically waits and reconnects if the controller connects or disconnects.
- **📹 Live Mac Webcam in ROS 2**: Streams your Mac's FaceTime HD Camera, iPhone Continuity Camera, or USB webcam directly into ROS 2 topics (`/camera/image_raw` and `/camera/image_raw/compressed`) over a low-latency binary TCP bridge (<2ms).
- **👁️ AI & High-Speed Vision Object Detection**:
  - **YOLOv8 AI Mode (`mode:=ai`)**: Real-time deep learning inference detecting 80 COCO classes (`person`, `cup`, `bottle`, `cell phone`, `scissors`, `tools`, etc.) via native aarch64 ONNX Runtime (~20 FPS).
  - **Robotics Cube Sorting Mode (`mode:=block`)**: Ultra-fast (>500 FPS, 1ms latency) detection for all 5 sorting blocks: `YELLOW CUBE`, `RED CUBE`, `GREEN CUBE`, `BLUE CUBE`, and natural unpainted `WOOD CUBE` with triple-layer anti-hand protection.
- **🤖 Autonomous Color Cube Sorting & Pick-and-Place**: Complete vision-guided picking of `red`, `green`, `blue`, and `yellow` blocks into color-matched drop points or custom bins. Supports one-touch trigger via DualSense **Square (⏹)** button.
- **🔴 Cobra Magician Dynamic Red Cube Tracker**: Continuous 10 Hz visual servoing that dynamically tracks a moving red cube across the table in real-time. Features an active 3D RViz tether line, inverted Y-axis lateral tracking, one-key grasp & lift (`[c]`), set-down release (`[r]`), and interactive CLI controls.
- **🛡️ Active Safety Workspace & Anti-Collision Shield**: Translucent 3D green bounding box in RViz showing reachable workspace expanded by the 60mm hand radius ($X \in [50, 305]\,\text{mm}$, $Y \in [-210, 210]\,\text{mm}$, $R \le 320\,\text{mm}$, $Z \ge 85\,\text{mm}$). Strict out-of-bounds rejection, pre-grab elbow-up transition posture, and joint angle interlocks prevent self-collisions.
- **🦾 7th Joint Adaptive Claw / Gripper**: Direct control of the electric gripper mounted on the end-effector with synchronized grasping, opening, and closing motions.
- **🎯 1-Point Camera Tabletop Calibration & Custom Drop Teaching**: 1-click calibration (`[k]`) binds camera pixel space to robot Cartesian space; all other cube coordinates recalculate automatically. Custom place points (`[1]`, `[2]`, `[3]`, `[4]`) saved persistently to JSON.
- **⚡ Ultra Low-Latency Serial Pipeline**: Asynchronous serial command pipeline (`_async=True`) with `TCP_NODELAY` and lock contention fixes eliminating lag (<10ms end-to-end response).
- **🌊 Fluid, Continuous Motion**: Time-delta velocity integration (`dt`) prevents stuttering and discrete "stepping" stops between setpoints.
- **🖥️ 3D Digital Twin & Dual Video in RViz 2**: Pre-configured RViz 2 setup displaying the 30 Hz `/joint_states` 3D robot model alongside live **Mac Webcam**, **AI Detected Objects**, and **3D Safe Area Marker** panels, accessible via any browser (`http://localhost:6080`).
- **🔄 Auto-Reconnecting Serial Bridge**: Automatic hot-plug USB detection on macOS, bridging the hardware UART to Docker's `/dev/ttyUSB0` via background daemons (`supervisor` + `socat`).
- **🔀 Dual Operating Modes**:
  1. **Joint Mode**: Direct 6-DOF joint angle control.
  2. **Cartesian Mode**: Intuitive XYZ tool-tip translation and RPY orientation.

---

## 🏗️ System Architecture

```text
[ PS5 DualSense Controller ]
             │  (Bluetooth HID)
             ▼
[ Mac Host: dualsense_to_ros2.py ] ────(TCP :9998)────▶ [ Docker: teleop_dualsense.py (ROS 2) ]
                                                                      │ (Square Button / CLI Trigger)
                                                                      ▼
                                                       [ color_pick_and_place.py ]
                                                       [ capture_red_cube.py     ] (Cobra Red Tracker)
                                                                      │ (Commands & Markers)
                                                ┌─────────────────────┼─────────────────────┐
                                                ▼                     ▼                     ▼
                                        [ RViz 2 3D Model ]   [/safe_area_marker]   [ listen_real_service ]
                                        (/joint_states 30Hz)   [/cube_marker]               │
                                                               [/tracking_link]             ▼
                                                                                     (/dev/ttyUSB0 pty)
                                                                                            │
[ myCobot 280 M5 Arm ] ◀──(USB UART)── [ Mac Host: bridge_serial.py ] ◀──(TCP :5005)── [ socat ]

[ Mac Webcam / iPhone Camera ]
             │
             ▼
[ Mac Host: stream_camera.py ] ────────(TCP :5007)────▶ [ Docker: camera_publisher.py (ROS 2) ]
             │                                                        │
             ▼ (HTTP :5006)                                           ▼ (/camera/image_raw)
    [ Web Browser Preview ]                               [ Docker: detect_objects.py ]
                                                                      │ (YOLOv8 & Block Tracker)
                                                                      ▼
                                                       ┌──────────────────────────────┐
                                                       ▼                              ▼
                                           [/camera/detected_objects]    [/camera/detections_json]
                                           (Annotated Stream in RViz 2)   (Centers & Metadata)
                                                                                      │
                                                                                      ▼
                                                                        [ color_pick_and_place.py ]
```

---

## ⚡ Hardware Checklist (Before Starting)

1. **12V DC Power**: Connect the 12V / 5A DC power brick to the round barrel jack on the robot's base. Turn the power switch **ON**. *(The 6 servo motors require 12V to generate torque; 5V USB alone only powers the screen).*
2. **USB-C Data Cable**: Connect the USB-C cable from your Mac to the **M5Stack Basic (base)**, **NOT** the head/Atom end-effector.
3. **M5Stack Basic Mode**: On the base screen, select **`Transponder`** mode (verify it shows `Atom: OK` and `115200`).
4. **DualSense Controller**: Pair your PS5 controller to your Mac via Bluetooth (`System Settings > Bluetooth`).

---

## 🚀 Quick Start Guide

### Step 1: Start All Host Services (Mac Terminal)

Run the all-in-one launcher:
```bash
cd ros2_docker
./start_ros2_dualsense.sh
```

This single command automatically:
- Boots the ROS 2 Docker container (if not already running).
- Cleans up any stale port listeners.
- Starts the **USB Serial Bridge** (Mac USB ➔ TCP 5005 ➔ Docker `/dev/ttyUSB0`).
- Starts the **Mac Camera Streamer** (Mac Webcam ➔ TCP 5007 & HTTP 5006).
- Starts the **DualSense Gamepad Streamer** (Bluetooth ➔ TCP 9998), waiting seamlessly for your controller to connect.

*To stop all services at any time, simply press `Ctrl + C` in this terminal.*

---

### Step 2: Open Web Desktop & Launch ROS 2

1. Open your browser to **[http://localhost:6080](http://localhost:6080)** (password: `ubuntu`).
2. Open the terminal inside the Web Desktop and run:
```bash
source /opt/ros2_ws/install/setup.bash
ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200
```

*RViz 2 will launch showing:*
- The **3D myCobot 280 digital twin** animating smoothly in sync with your controller inputs.
- The **Mac Webcam feed** embedded directly in RViz 2 under `/camera/image_raw`.

*(Optional: You can also preview the camera in your Mac browser at **[http://localhost:5006](http://localhost:5006)**).*

---

### Step 3 (Optional): Launch with Vision & AI Object Detection

To enable real-time object detection with bounding boxes and centroid coordinates overlaid in RViz 2:

#### Option A: Original YOLOv8 AI Detection (Default with `detect:=true`)
Runs deep learning inference for all 80 COCO classes (`person`, `cup`, `bottle`, `cell phone`, `scissors`, `chair`, `book`, etc.) via native ONNX Runtime (~20 FPS):
```bash
ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200 detect:=true mode:=ai
```

#### Option B: High-Precision Cube Sorting Mode (>500 FPS)
Ultra-fast tracking for all 5 official sorting blocks (`YELLOW CUBE`, `RED CUBE`, `GREEN CUBE`, `BLUE CUBE`, and natural unpainted `WOOD CUBE`) with triple-layer anti-hand skin protection:
```bash
ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200 detect:=true mode:=block
```

#### Option C: Hybrid Mode
Detects color cubes plus everyday tabletop items with anti-collision NMS:
```bash
ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200 detect:=true mode:=all
```

---

### Step 4: Red Cube Dynamic Visual Tracking & Capture (Cobra / Dobot Magician Style)

The **Red Cube Dynamic Tracker** operates like a Cobra / Dobot Magician visual servoing demo:
- **🔗 Real-Time Dynamic Following**: The robot hand (claw) continuously links its motion directly to the **Red Cube**. As you move the red cube across the desk with your hand, the robot hand actively glides and shadows it in $(X, Y)$ coordinates at safe hover height ($180\,\text{mm}$).
- **🎯 Instant Capture**: Press **`[c]`**, **`[Space]`**, or PS5 Gamepad **Square (⏹)** — the claw descends straight down, clamps the red cube firmly, and lifts it into mid-air.
- **⬇️ Release & Resume Following**: Press **`[r]`** or Gamepad **Circle (◯)** — gently places the red cube back down on the table, opens the claw, lifts back to hover height, and immediately resumes dynamically following the cube!
- **🚫 Pure Visual Servoing**: No bins or drop sorting — pure linked tracking and grasping control!

#### Option A: All-in-One Launch (Starts Robot Driver + Camera + RViz + Dynamic Tracker)
```bash
ros2 launch mycobot_280 capture_red_cube.launch.py port:=/dev/ttyUSB0 baud:=115200
```

#### Option B: Standalone Node (If `teleop_dualsense.launch.py` is already running)
```bash
ros2 run mycobot_280 capture_red_cube
```

#### 🎮 Interactive Controls:
| Key / Button | Action | Description |
| :--- | :--- | :--- |
| **`[c]`** or **`[Space]`** | **Capture Red Cube** | Descends directly down, grips the red cube, and lifts to hold. |
| **`[r]`** | **Release to Table** | Gently lowers cube back down to table, opens claw, and resumes following. |
| **`[f]`** | **Toggle Follow Mode** | Pauses or resumes real-time dynamic hand following. |
| **`[a]`** | **Toggle Auto-Grasp** | Automatically grasps the red cube when it is stationary for ~1.2s. |
| **`[k]`** | **1-Click Calibrate Anchor** | Locks the exact pixel position of the physical claw as Tool Center anchor. |
| **`[t]`** | **Tool-Camera Offset** | Cycles the 60mm physical camera offset direction (V-, V+, U+, U-). |
| **`[+]` / `[-]`** | **Fine-Tune Offset** | Adjusts the camera-to-tool center distance by $\pm 2\,\text{mm}$. |
| **`[i]`** | **Invert Y Polarity** | Toggles side-to-side (left/right) tracking polarity in real time. |
| **`[x]`** | **Invert X Polarity** | Toggles front-to-back (forward/reverse) tracking polarity in real time. |
| **`[s]`** | **Vision Diagnostics** | Live coordinates, tracking speed, and camera FPS. |
| **`[o]`** | **Observe Posture** | Moves arm to standby observation posture. |
| **`[q]`** | **Quit** | Exits the tracking program safely. |
| **DualSense ⏹ (Square)** | **Capture / Release** | Instant one-button capture from PS5 controller. |
| **DualSense ◯ (Circle)** | **Release Cube** | Releases held cube back onto desk and resumes following. |

---

### Step 5: Full Autonomous Vision Pick-and-Place Sorting

To run autonomous color sorting with the 7th joint claw, 3D safe area bounding box in RViz, and 1-click calibration:
```bash
ros2 launch mycobot_280 color_pick_and_place.launch.py port:=/dev/ttyUSB0 baud:=115200
```
- Press **`[a]`** to start continuous autonomous sorting.
- Or press **Square (⏹)** on your DualSense controller at any time to pick the detected cube!

---

### Step 5 (Optional): Launch Without Camera

If you want to run teleoperation without camera topics:
```bash
ros2 launch mycobot_280 teleop_dualsense.launch.py camera:=false
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
| **Cross (✖)** | Open Gripper / 7th Joint Claw | Open Gripper / 7th Joint Claw |
| **Circle (🔴)** | Close Gripper / 7th Joint Claw | Close Gripper / 7th Joint Claw |
| **Square (⏹)** | 🎯 Trigger Autonomous Pick & Place | 🎯 Trigger Autonomous Pick & Place |
| **D-Pad Up / Down** | Increase / Decrease Speed | Increase / Decrease Speed |

---

## 🤖 Autonomous Color Pick & Place (Vision-Guided Sorting)

A robust, vision-guided pick-and-place application that detects colored cubes (`red`, `green`, `blue`, `yellow`), approaches with the 7th joint claw open, descends to grasp height, closes the claw, and transports each cube to its designated color-coded drop bin or your custom place points.

### 🦾 7th Joint Adaptive Claw (Gripper) Setup & Control
- **Hardware Connection**:
  1. Mount the adaptive claw to the end-effector mounting flange.
  2. Plug the 4-pin Grove cable from the claw into the **Grove port on the M5 Atom head** (top of Joint 6).
  3. Verify the robot's base M5Stack is in **`Transponder`** mode (Atom: OK).
- **Control Interface**:
  - In ROS 2, the claw is controlled via `/mycobot/gripper_status` and the serial driver service (`set_gripper_state(0, 70)` for Open, `set_gripper_state(1, 70)` for Close).
  - **Manual DualSense Teleop**: Press **Cross (✖)** to open claw, **Circle (🔴)** to close claw.
  - **Autonomous Sorting**: The node automatically manages the open-hover-descend-grasp-lift cycle with calibrated grasp delays.

### 🛡️ Active Safety Workspace & Self-Collision Shield
- **3D Visual Safe Area in RViz**: A translucent emerald green 3D bounding box is rendered directly on your desk in RViz (`SAFE PICK & PLACE AREA`). Any cube detected outside this envelope is marked `[OUT OF BOUNDS]` in red, and the arm **strictly refuses** to command any motion towards it.
- **Physical Reach Envelope**: Clamped to $X \in [50, 305]\,\text{mm}$, $Y \in [-210, 210]\,\text{mm}$, and $R = \sqrt{X^2+Y^2} \le 320\,\text{mm}$ (increased by 60mm hand radius to maximize pick reach across the desk).
- **Hard Table Floor Protection**: $Z$ is strictly clamped to $\ge 85\,\text{mm}$ to prevent driving the claw into the table.
- **Elbow-Up Pre-Grab Transition**: Before descending, the robot always moves through a pre-grab posture (`[18.8, -7.91, -54.49, -23.02, -0.79, -14.76]`). This forces the inverse kinematics solver into the forward elbow-up branch, completely eliminating self-collision where the claw crosses into Joint 2 or Joint 3.
- **Calibrated Flange Posture**: Uses locked end-effector orientation ($Rx=179.87^\circ, Ry=-3.78^\circ, Rz=-62.75^\circ$) so the wrist is always perpendicular to the table.
- **Self-Collision Software Interlock**: Continuously checks angles; blocks motion if $J_3 < -100^\circ$ or $(J_2 - J_3) > 105^\circ$.
- **Kinematic Shoulder Lift**: After grasping, the arm elevates Joint 2 (`-0.71°`) before Cartesian transit, preventing singular branch flips while carrying a payload.

---

## 🔴 Cobra Magician Dynamic Red Cube Tracker (`capture_red_cube`)

High-frequency (~10 Hz) closed-loop visual servoing system inspired by Dobot / Cobra Magician industrial sorters. The myCobot arm dynamically positions its open claw directly above a red cube as it slides or moves across the tabletop desk in real-time.

```text
[ Red Cube Slides on Desk ] ──▶ [ Mac Camera (host:5007) ] ──▶ [ detect_objects (Block Mode) ]
                                                                       │ (Pixel cx, cy)
                                                                       ▼
                                                          [ capture_red_cube.py ]
                                                                       │ (Exponential Low-Pass Filter)
                                                                       ▼
                                                          [ listen_real_service ]
                                                                       │
                                                                       ▼
                                                          [ Arm Follows Smoothly @ 10Hz ]
```

### 🌟 Key Tracking Features:
- **Real-Time Visual Tether (RViz 2)**: A dynamic cyan/orange line connects the robot's hover tool-tip directly to the red cube on the desk.
- **Inverted Lateral Tracking ($Y$-Axis)**: Lateral direction is reversed by default (`invert_y:=true`), matching natural physical perspective. Can be toggled on-the-fly via key `[i]`.
- **Generous Hand-Radius Workspace**: Safe area is expanded by the $+60\,\text{mm}$ hand reach ($X \in [50, 305]\,\text{mm}$, $Y \in [-210, 210]\,\text{mm}$, $R \le 320\,\text{mm}$), preventing out-of-bounds dropouts.
- **One-Touch Grasp & Release**: Instantly grab the tracked cube with **`[Space]`** or **`[c]`**, lift it, and set it down with **`[r]`**.

### 🚀 How to Launch:
Inside your terminal or Web Desktop ([http://localhost:6080](http://localhost:6080)):
```bash
ros2 launch mycobot_280 capture_red_cube.launch.py port:=/dev/ttyUSB0 baud:=115200
```

#### Launch Arguments:
| Argument | Default | Description |
| :--- | :---: | :--- |
| `invert_y` | `true` | Inverts lateral $Y$-axis direction to match physical movement. |
| `hand_radius` | `60.0` | Hand/gripper reach expansion (mm) for the safe tracking envelope. |
| `grip_z` | `95.0` | Tabletop grasp height (mm) above base for picking up cubes. |
| `auto` | `false` | When `true`, automatically grasps the cube whenever it stays still for 1.2s. |

### ⌨️ Interactive Tracker Hotkeys:
| Key | Command | Description |
| :---: | :--- | :--- |
| **`[c]`** or **`[Space]`** | **Capture Red Cube** | Descends directly to table, grasps the red cube, and lifts to holding height. |
| **`[r]`** | **Release to Table** | Lowers the grasped cube back to the desk, opens claw, and resumes tracking. |
| **`[f]`** | **Toggle Follow Mode** | Pauses or resumes real-time hand following (`[ACTIVE]` / `[PAUSED]`). |
| **`[a]`** | **Toggle Auto-Grasp** | Enables/disables automatic grasping when the cube stops moving. |
| **`[i]`** | **Toggle $Y$-Axis Invert** | Inverts the lateral left/right tracking direction on the fly. |
| **`[h]`** | **Hover Test** | Re-centers the hover point directly above the current cube position. |
| **`[s]`** | **Live Diagnostics** | Prints live tracking FPS, pixel center, coordinates, and safe bounds. |
| **`[o]`** | **Observation Pose** | Returns arm to wide observation posture `[-7.11, -6.94, -55.01, -24.16, 0.0, -15.0]`. |
| **`[q]`** | **Quit Tracker** | Safely exits the tracker node. |

---

## 🎯 Calibration Guide: Hand-Eye Calibration & Vision Tracking

We provide two calibration workflows:
1. **Elephant Robotics Official Hand-Eye Calibration (`handeye_calibration`)**:
   Direct implementation of [GitBook Section 5.2 Hand-eye Calibration](https://docs.elephantrobotics.com/docs/mycobot-m5-en/12-ApplicationBaseROS/12.1-ROS1/12.1.4-rivzIntroductionAndUse/myCobot-280.html#52-hand-eye-calibration). Moves to observation pose, relaxes servos for free-hand positioning, and generates `EyesInHand_matrix.json`.
2. **1-Click Gamepad Anchor Method (`[k]` / L3 Click)**:
   Instant 5-second anchor pairing with the PS5 controller or CLI.

---

### 🐘 Method 1: Elephant Robotics Official Hand-Eye Calibration

Open a terminal inside the Web Desktop (**[http://localhost:6080](http://localhost:6080)**):

```bash
ros2 run mycobot_280 handeye_calibration
```

#### Step-by-Step Flow:
1. **Step 1 (Observation Pose)**:
   - The arm moves to the official observation posture:
     `origin_mycbot_level = [0, 5, -104, 14, 0, 0]`
2. **Step 2 (Target Detection)**:
   - Place a calibration target (ArUco marker, STag code, or any colored block) on the table in front of the robot.
   - The camera detects the target center pixel `(cx, cy)` in real-time. Press **Enter** once locked.
3. **Step 3 (Touch Calibration Point)**:
   - Servos automatically **release (free-hand damping mode)**!
   - Gently guide the claw tip by hand until it touches the center of the calibration target.
   - Press **Enter** to re-engage and lock the servos.
4. **Step 4 (Save Matrix)**:
   - The node computes the $4 \times 4$ rigid transform matrix and saves it to:
     `EyesInHand_matrix.json`
   - The autonomous pick-and-place node automatically loads and uses this matrix!

---

### 🎮 Method 2: Rapid 1-Click Gamepad Calibration (`[k]` / L3)

---

### 🚀 2. The 3-Step Calibration Walkthrough

```text
  [ 1. Place Cube ]            [ 2. Jog Claw Above Cube ]         [ 3. Trigger Calibration ]
      ┌─────┐                         ┌───┐                          🎮 DualSense: Click L3 / Create
      │ RED │                         │   │ (Claw Hovering)          ⌨️ Terminal: Type [k] + Enter
      └─────┘                     ┌───┴───┴───┐                      ✅ Calibrated!
   On Tabletop Desk               │    RED    │                   Pixel (320, 240)
                                  └───────────┘                   -> Robot (175, 0)
```

1. **Place any Cube on the Table**:
   - Put a colored cube (Red, Green, Blue, or Yellow) anywhere within the camera's view on your tabletop desk.
2. **Jog the Claw Above the Cube**:
   - Using your **PS5 DualSense Controller** (Left Stick for X/Y, Right Stick for Z) or keyboard teleop, move the robot arm so the open claw is centered directly **above the cube** (approx. 180–190 mm hover height).
3. **Trigger Calibration (Choose Any Method)**:
   - 🎮 **Easiest (No Keyboard Required)**: Press **L3 (Click Left Stick down)** or the **Create / Share** button on your DualSense gamepad!
   - ⌨️ **Terminal Topic Command**:
     ```bash
     ros2 topic pub --once /color_sort/trigger std_msgs/msg/String "{data: 'calibrate'}"
     ```
   - 💻 **Option B Interactive Terminal**: In an interactive terminal running `ros2 run mycobot_280 color_pick_and_place`, press **`k`** + **`Enter`**.

   - The system immediately reads the cube's 2D pixel center $(cx_0, cy_0)$ from `/camera/detections_json` and pairs it with the physical tool coordinates $(X_0, Y_0)$:
     ```text
     [✓] CAMERA REFERENCE CALIBRATION COMPLETE!
         Matched Camera Pixel:   (320, 240)
         To Physical Robot X,Y: (175.0, 0.0) mm
     ```
   - **Calibration is done!** Calibration data is saved persistently in `custom_positions.json` and automatically reloaded on every launch.

---

### 🔍 How to Verify Calibration (`[t]` Test Hover)

1. Pick up the cube and move it to any other spot on your table.
2. In the terminal, press **`[t]`** (Test Hover).
3. The robot arm will automatically transition through the safe elbow-up posture and smoothly fly to hover directly above the cube at its new position!
4. If the claw centers right above the cube, your calibration is 100% accurate.

---

### 📐 How It Works (The 1-Point Recalculation Principle)

Because standard webcams are stationary relative to your desk:
1. **Linear Perspective Recalculation**:
   The anchor pair $(cx_0, cy_0) \longleftrightarrow (X_0, Y_0)$ establishes the desk's center reference. For any other cube detected at pixel $(cx, cy)$, physical Cartesian coordinates are calculated automatically:
   $$X_{\text{target}} = X_0 + (cy - cy_0) \times s_x$$
   $$Y_{\text{target}} = Y_0 + (cx - cx_0) \times s_y$$
   where $s_x$ and $s_y$ are the calibrated millimeter-per-pixel scale factors.
   **All other cubes recalculate automatically wherever they are placed on the table!**

2. **Tabletop Surface Depth ($Z$ Plane)**:
   Standard webcams lack 3D depth sensors, but all cubes rest on a flat table surface. The system maintains two fixed vertical planes:
   - **Hover / Transit Height ($Z_{\text{hover}} = 185\,\text{mm}$)**: Safe horizontal travel altitude above obstacles.
   - **Grasp Height ($Z_{\text{grip}} = 95\,\text{mm}$)**: Exact elevation where the claw fingers grip the cube.
   - **Live $Z$ Height Adjustment**: If your tabletop or cubes have slightly different heights:
     - Press **`[+]`** to raise the grasp surface by $+2\,\text{mm}$.
     - Press **`[-]`** to lower the grasp surface by $-2\,\text{mm}$.

---

### 📦 Teaching Custom Drop Points (`[1]`, `[2]`, `[3]`, `[4]`)

You can place your sorting bins or boxes anywhere within the arm's reach:
1. Jog the arm so the claw is directly above your **Red** container/bin $\to$ Press **`[1]`**.
2. Jog the arm above your **Green** container/bin $\to$ Press **`[2]`**.
3. Jog the arm above your **Blue** container/bin $\to$ Press **`[3]`**.
4. Jog the arm above your **Yellow** container/bin $\to$ Press **`[4]`**.
5. Press **`[p]`** at any time to print all saved drop coordinates.
All taught points are permanently saved in `custom_positions.json`.

---

### ⌨️ Complete Interactive Hotkeys Reference

| Key | Action | Description |
| :---: | :--- | :--- |
| **`[k]`** | **Calibrate Camera Anchor** | Binds current claw position to detected cube's pixel center. |
| **`[t]`** | **Test Hover** | Flies arm to hover above currently detected cube to test calibration. |
| **`[1]`** | **Save RED Place Point** | Saves current claw position as the Red drop destination. |
| **`[2]`** | **Save GREEN Place Point** | Saves current claw position as the Green drop destination. |
| **`[3]`** | **Save BLUE Place Point** | Saves current claw position as the Blue drop destination. |
| **`[4]`** | **Save YELLOW Place Point** | Saves current claw position as the Yellow drop destination. |
| **`[s]`** | **Save Fixed Pick Spot** | Saves current claw position as fixed pickup jig coordinate. |
| **`[f]`** | **Toggle Pick Mode** | Switches between Dynamic Vision Tracking and Fixed Spot Pickup. |
| **`[p]`** | **Print Status** | Prints current joint angles, coordinates, and all saved drop points. |
| **`[a]`** | **Toggle Auto Mode** | Continuously detects, picks, and sorts all visible cubes. |
| **`[r]`** | **Sort Red Cube** | Picks and sorts nearest detected Red cube. |
| **`[g]`** | **Sort Green Cube** | Picks and sorts nearest detected Green cube. |
| **`[b]`** | **Sort Blue Cube** | Picks and sorts nearest detected Blue cube. |
| **`[y]`** | **Sort Yellow Cube** | Picks and sorts nearest detected Yellow cube. |
| **`[h]`** | **Home Pose** | Moves arm to high observation/ready pose `[0, 0, 0, 0, 0, 0]`. |
| **`[o]`** | **Open Claw** | Opens the 7th joint adaptive gripper. |
| **`[c]`** | **Close Claw** | Closes the 7th joint adaptive gripper. |
| **`[+]`** | **Raise Table Surface** | Increases grasp height $Z_{\text{grip}}$ by $+2\,\text{mm}$. |
| **`[-]`** | **Lower Table Surface** | Decreases grasp height $Z_{\text{grip}}$ by $-2\,\text{mm}$. |

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
    ├── bridge_serial.py                # Host USB UART <-> TCP serial forwarder (:5005)
    ├── dualsense_to_ros2.py            # Host DualSense Bluetooth <-> TCP streamer (:9998)
    ├── stream_camera.py                # Host Mac camera streamer (:5007 TCP & :5006 HTTP)
    ├── start_ros2_cobot.sh             # Generic ROS 2 stack launcher
    ├── start_ros2_dualsense.sh         # All-in-one launcher (Serial + Camera + DualSense)
    ├── sync_and_build.sh               # Hot-reload sync & fast rebuild inside Docker container
    └── custom_nodes/
        ├── setup.py                    # Package build & console scripts definition
        ├── teleop_dualsense.py         # ROS 2 Python Node (continuous velocity & RViz TF)
        ├── camera_publisher.py         # ROS 2 Python Node (publishes /camera/image_raw)
        ├── detect_objects.py           # ROS 2 Vision Node (YOLOv8 + Color/Wood Block Tracker)
        ├── listen_real_service.py      # ROS 2 Hardware Driver Service for myCobot 280
        ├── capture_red_cube.py         # Cobra Magician Dynamic Red Cube Tracker & Visual Servoing
        ├── capture_red_cube.launch.py  # Launch file for dynamic red tracker + RViz + camera
        ├── color_pick_and_place.py     # Autonomous Vision Sorting State Machine & Safety Guard
        ├── color_pick_and_place.launch.py # All-in-one vision sorting launch file
        ├── handeye_calibration.py      # Official Elephant Robotics Hand-Eye Calibration Node
        ├── teleop_dualsense.launch.py  # ROS 2 Launch file (loads URDF, robot state, camera, RViz)
        ├── custom_positions.json       # Calibrated camera anchor & saved custom place drop points
        └── mycobot.rviz                # Pre-configured RViz setup with robot model, dual cameras & safe area
```

---

## 🛠️ Diagnostics & Troubleshooting

| Issue | Cause | Fix |
| :--- | :--- | :--- |
| **RViz moves but physical servos don't** | 12V power supply disconnected or USB cable in top Atom port | Plug 12V 5A power into round DC jack at base and switch ON; plug USB-C into base M5Stack. |
| **`⚠️ Arm: DISCONNECTED` on terminal** | USB cable loose or M5Stack not in Transponder mode | Re-seat USB-C cable; select `Transponder` on screen (`Atom: OK`). |
| **Camera shows "Permission Pending"** | macOS privacy settings blocking camera | Open **System Settings ➔ Privacy & Security ➔ Camera** and toggle Terminal/IDE **ON**, or run `tccutil reset Camera`. |
| **Stop camera stream** | Wanting to turn off webcam | Press `Ctrl + C` in launcher terminal, or run `pkill -f stream_camera.py`. |
| **Motion feels stepped / jerky** | Discrete commands without velocity integration | Handled via continuous `dt` velocity integration in `teleop_dualsense.py`. |
| **Address already in use (5005/5007/9998)** | Stale processes from earlier runs | Run `lsof -ti :5005 -ti :5006 -ti :5007 -ti :9998 \| xargs kill -9 2>/dev/null`. |
| **Container shutdown** | Ending session | Run `docker compose down` inside `ros2_docker/`. |

---

## 📜 License
MIT License. Built for use with Elephant Robotics myCobot 280 M5.
