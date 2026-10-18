# myCobot 280 M5 - ROS 2 Humble Docker Environment (macOS Apple Silicon)

This environment runs **Ubuntu 22.04 + ROS 2 Humble + MoveIt 2 + mycobot_ros2** inside a container with a full Web-based GUI desktop.

---

## 1. Quick Start

### Step 1: Plug in and power the Robot
1. Plug the **12V power adapter** into the base of the myCobot and turn on the power switch.
2. Connect the **USB-C cable** from your Mac to the **side of the M5Stack Basic (base)**.
3. On the M5 Basic screen: select **`Transponder`** -> press **Button A** -> confirm it displays **`Atom: OK`**.

### Step 2: Start the Environment & Serial Bridge
Open a terminal on your Mac and run:
```bash
cd ros2_docker
./start_ros2_cobot.sh
```
This will:
- Start the Docker container (if not already running).
- Automatically detect the robot's USB port (e.g. `/dev/cu.usbserial-*`) and bridge it to Docker port 5005.

### Step 3: Open the Web Desktop
Open your browser (Safari or Chrome) to:
👉 **[http://localhost:6080](http://localhost:6080)**

- Password (if prompted): `ubuntu`

### Step 4: Serial Port is Automatically Connected
Inside the Docker container, a background supervisor daemon automatically links `host.docker.internal:5005` to `/dev/ttyUSB0`. No manual setup is needed!

---

## 2. Running ROS 2 Commands (Inside the Web Desktop)

Open a terminal (Terminator icon) in the web desktop:

### 1. Slider Control (Move Robot via RViz2 Sliders)
```bash
ros2 launch mycobot_280 slider_control.launch.py port:=/dev/ttyUSB0 baud:=115200
```
Drag the joint sliders in the GUI to move the physical robot.

### 2. Keyboard Teleoperation
```bash
ros2 launch mycobot_280 teleop_keyboard.launch.py port:=/dev/ttyUSB0 baud:=115200
```
Use keys `w/a/s/d/z/x` to control coordinates, and `g/h` to open/close the gripper.

### 3. Model Following (Real-to-Sim)
```bash
ros2 launch mycobot_280 follow_display.launch.py port:=/dev/ttyUSB0 baud:=115200
```
Move the physical arm by hand; the 3D model in RViz tracks its motion in real time.

### 4. MoveIt 2 (Motion Planning & Collision Avoidance)
```bash
ros2 launch mycobot_280_moveit2 demo.launch.py
```
To execute the planned path on the physical arm, open another terminal:
```bash
ros2 run mycobot_280_moveit2_control sync_plan
```

---

## 4. PS5 DualSense Controller (ROS 2 Teleoperation)

Control the robot in real-time with your DualSense while watching the 3D model move in RViz2:

### Step 1: Start the DualSense Launcher (on Mac)
```bash
cd ros2_docker
./start_ros2_dualsense.sh
```

### Step 2: Open RViz2 (in Web Desktop)
Go to **http://localhost:6080** and:
- Double-click the **DualSense Teleop (RViz2)** icon on the desktop.
- Or run in terminal:
  ```bash
  ros2 launch mycobot_280 teleop_dualsense.launch.py port:=/dev/ttyUSB0 baud:=115200
  ```

### DualSense Controls:
- **Left Stick**: Joint 1 & 2 (or X / Y in Cartesian mode)
- **Right Stick**: Joint 3 & 4 (or Z / Yaw in Cartesian mode)
- **L2 / R2**: Joint 5 (or Roll)
- **L1 / R1**: Joint 6 (or Pitch)
- **Options Button**: Toggle between Joint Mode and Cartesian Mode
- **Triangle (▲)**: Home Position `[0, 0, 0, 0, 0, 0]`
- **Cross (✖) / Circle (🔴)**: Open / Close Gripper
- **D-Pad Up / Down**: Increase / Decrease Speed

---

## 5. Stopping the Container
When you are finished:
```bash
cd ros2_docker
docker compose down
```
