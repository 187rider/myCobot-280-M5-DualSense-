import json
import math
import socket
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from mycobot_interfaces.srv import (
    GetAngles,
    GetCoords,
    SetAngles,
    SetCoords,
    GripperStatus,
    PumpStatus
)

COORD_LIMITS = {
    'x': [-280.0, 280.0],
    'y': [-280.0, 280.0],
    'z': [-50.0, 380.0],
    'rx': [-180.0, 180.0],
    'ry': [-180.0, 180.0],
    'rz': [-180.0, 180.0]
}

JOINT_LIMITS = [
    [-165.0, 165.0],
    [-135.0, 135.0],
    [-145.0, 145.0],
    [-145.0, 145.0],
    [-160.0, 160.0],
    [-175.0, 175.0]
]

class DualSenseTeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_dualsense")
        self.get_logger().info("Initializing myCobot DualSense ROS2 Teleop Node...")

        # Publisher for RViz joint state transforms
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)

        # Service clients provided by listen_real_service
        self.set_angles_client = self.create_client(SetAngles, "set_angles")
        self.get_angles_client = self.create_client(GetAngles, "get_angles")
        self.set_coords_client = self.create_client(SetCoords, "set_coords")
        self.get_coords_client = self.create_client(GetCoords, "get_coords")
        self.set_gripper_client = self.create_client(GripperStatus, "set_gripper")
        self.set_pump_client = self.create_client(PumpStatus, "set_pump_status")

        # State
        self.mode = "JOINT"  # "JOINT" or "CARTESIAN"
        self.speed = 25
        self.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.coords = [150.0, 0.0, 200.0, -180.0, 0.0, 0.0]
        self.prev_buttons = {}

        # TCP Connection to Mac host
        self.sock = None
        self.tcp_buffer = ""
        self.last_connect_try = 0.0

        # 30Hz Timer for polling TCP, commanding arm, and publishing RViz TFs
        self.timer = self.create_timer(0.033, self.control_loop)
        self.last_send_time = time.time()
        self.synced = False

    def get_socket(self):
        if self.sock is not None:
            return self.sock
        now = time.time()
        if now - self.last_connect_try < 1.0:
            return None
        self.last_connect_try = now
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(0.2)
            s.connect(("host.docker.internal", 9998))
            s.setblocking(False)
            self.sock = s
            self.tcp_buffer = ""
            self.get_logger().info("[+] Connected to DualSense Streamer on host.docker.internal:9998")
            return self.sock
        except Exception:
            return None

    def publish_joint_states(self):
        if not rclpy.ok():
            return
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6"
        ]
        js.position = [math.radians(float(a)) for a in self.angles]
        try:
            self.joint_state_pub.publish(js)
        except Exception:
            pass

    def sync_initial_state(self):
        if self.get_angles_client.service_is_ready():
            try:
                req = GetAngles.Request()
                future = self.get_angles_client.call_async(req)
                def on_angles_done(fut):
                    res = fut.result()
                    if res and [res.joint_1, res.joint_2, res.joint_3] != [-1, -1, -1]:
                        self.angles = [res.joint_1, res.joint_2, res.joint_3, res.joint_4, res.joint_5, res.joint_6]
                        self.get_logger().info(f"[+] Synced Physical Arm Angles: {self.angles}")
                future.add_done_callback(on_angles_done)
            except Exception:
                pass

        if self.get_coords_client.service_is_ready():
            try:
                req_c = GetCoords.Request()
                future_c = self.get_coords_client.call_async(req_c)
                def on_coords_done(fut):
                    res = fut.result()
                    if res and [res.x, res.y, res.z] != [-1, -1, -1]:
                        self.coords = [res.x, res.y, res.z, res.rx, res.ry, res.rz]
                future_c.add_done_callback(on_coords_done)
            except Exception:
                pass

    def send_angles(self):
        if getattr(self, "sending_angles", False):
            return
        if self.set_angles_client.service_is_ready():
            self.sending_angles = True
            req = SetAngles.Request()
            req.joint_1 = float(self.angles[0])
            req.joint_2 = float(self.angles[1])
            req.joint_3 = float(self.angles[2])
            req.joint_4 = float(self.angles[3])
            req.joint_5 = float(self.angles[4])
            req.joint_6 = float(self.angles[5])
            req.speed = min(95, max(int(self.speed * 2), 65))
            future = self.set_angles_client.call_async(req)
            def on_done(fut):
                self.sending_angles = False
            future.add_done_callback(on_done)

    def send_coords(self):
        if getattr(self, "sending_coords", False):
            return
        for i, axis in enumerate(["x", "y", "z", "rx", "ry", "rz"]):
            min_l, max_l = COORD_LIMITS[axis]
            self.coords[i] = max(min_l, min(max_l, self.coords[i]))

        if self.set_coords_client.service_is_ready():
            self.sending_coords = True
            req = SetCoords.Request()
            req.x = float(self.coords[0])
            req.y = float(self.coords[1])
            req.z = float(self.coords[2])
            req.rx = float(self.coords[3])
            req.ry = float(self.coords[4])
            req.rz = float(self.coords[5])
            req.speed = min(95, max(int(self.speed * 2), 65))
            req.model = 0
            future = self.set_coords_client.call_async(req)
            def on_done(fut):
                self.sending_coords = False
            future.add_done_callback(on_done)

    def set_gripper(self, status):
        if self.set_gripper_client.service_is_ready():
            req = GripperStatus.Request()
            req.status = status
            self.set_gripper_client.call_async(req)
            grip_str = "OPEN" if status else "CLOSE"
            self.get_logger().info(f"Gripper: {grip_str}")

    def control_loop(self):
        # Always publish joint states so RViz transforms are continuous and 100% OK
        self.publish_joint_states()

        if not self.synced:
            self.sync_initial_state()
            self.synced = True

        sock = self.get_socket()
        if not sock:
            return

        try:
            data = sock.recv(8192)
            if not data:
                self.sock = None
                return
            self.tcp_buffer += data.decode("utf-8")
        except BlockingIOError:
            pass
        except Exception:
            self.sock = None
            return

        lines = self.tcp_buffer.split("\n")
        self.tcp_buffer = lines[-1]
        valid_lines = [l.strip() for l in lines[:-1] if l.strip()]
        if not valid_lines:
            return

        # Always take the newest/freshest packet for ZERO latency
        try:
            msg = json.loads(valid_lines[-1])
        except Exception:
            return

        axes = msg.get("axes", [0.0]*6)
        buttons = msg.get("buttons", {})
        bumpers = msg.get("bumpers", [0, 0])
        dpad = msg.get("dpad", [0, 0, 0, 0])

        # Button Edge Detection
        def just_pressed(btn_id):
            return buttons.get(str(btn_id), False) and not self.prev_buttons.get(str(btn_id), False)

        # 3: Triangle -> Go Home
        if just_pressed(3):
            self.get_logger().info("[*] DualSense: Moving Home [0,0,0,0,0,0]")
            self.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.send_angles()

        # 0: Cross -> Open Gripper
        if just_pressed(0):
            self.set_gripper(True)

        # 1: Circle -> Close Gripper
        if just_pressed(1):
            self.set_gripper(False)

        # 6: Options -> Switch Mode
        if just_pressed(6):
            self.mode = "CARTESIAN" if self.mode == "JOINT" else "JOINT"
            self.get_logger().info(f"[+] Switched to {self.mode} MODE")

        # D-pad Up / Down -> Speed +/-
        if dpad[0] and not self.prev_buttons.get("dup", False):
            self.speed = min(80, self.speed + 5)
            self.get_logger().info(f"Speed: {self.speed}%")
        if dpad[1] and not self.prev_buttons.get("ddown", False):
            self.speed = max(5, self.speed - 5)
            self.get_logger().info(f"Speed: {self.speed}%")

        self.prev_buttons = {**buttons, "dup": dpad[0], "ddown": dpad[1]}

        # Analog Stick Motions (Smooth Velocity Integration at ~30Hz)
        now = time.time()
        dt = now - self.last_send_time
        if dt < 0.030:
            return
        self.last_send_time = now

        lx, ly, rx, ry = axes[0], axes[1], axes[2], axes[3]
        l2, r2 = axes[4], axes[5]
        trigger_diff = r2 - l2
        bumper_diff = (1.0 if bumpers[1] else 0.0) - (1.0 if bumpers[0] else 0.0)

        if self.mode == "JOINT":
            # Degrees per second scale based on current speed setting
            deg_per_sec = (self.speed / 25.0) * 45.0
            step = deg_per_sec * dt
            moved = False
            # J1: Left Stick X
            if abs(lx) > 0.05:
                self.angles[0] = max(JOINT_LIMITS[0][0], min(JOINT_LIMITS[0][1], self.angles[0] - lx * step))
                moved = True
            # J2: Left Stick Y
            if abs(ly) > 0.05:
                self.angles[1] = max(JOINT_LIMITS[1][0], min(JOINT_LIMITS[1][1], self.angles[1] - ly * step))
                moved = True
            # J3: Right Stick Y
            if abs(ry) > 0.05:
                self.angles[2] = max(JOINT_LIMITS[2][0], min(JOINT_LIMITS[2][1], self.angles[2] - ry * step))
                moved = True
            # J4: Right Stick X
            if abs(rx) > 0.05:
                self.angles[3] = max(JOINT_LIMITS[3][0], min(JOINT_LIMITS[3][1], self.angles[3] + rx * step))
                moved = True
            # J5: L2 / R2
            if abs(trigger_diff) > 0.1:
                self.angles[4] = max(JOINT_LIMITS[4][0], min(JOINT_LIMITS[4][1], self.angles[4] + trigger_diff * step))
                moved = True
            # J6: L1 / R1
            if bumper_diff != 0:
                self.angles[5] = max(JOINT_LIMITS[5][0], min(JOINT_LIMITS[5][1], self.angles[5] + bumper_diff * step))
                moved = True

            if moved:
                self.send_angles()

        else: # CARTESIAN MODE
            # mm per second scale based on current speed setting
            mm_per_sec = (self.speed / 25.0) * 60.0
            step = mm_per_sec * dt
            moved = False
            # X: Left Stick Y
            if abs(ly) > 0.05:
                self.coords[0] += -ly * step
                moved = True
            # Y: Left Stick X
            if abs(lx) > 0.05:
                self.coords[1] += lx * step
                moved = True
            # Z: Right Stick Y
            if abs(ry) > 0.05:
                self.coords[2] += -ry * step
                moved = True
            # Rz (Yaw): Right Stick X
            if abs(rx) > 0.05:
                self.coords[5] += rx * step
                moved = True
            # Rx (Roll): L2 / R2
            if abs(trigger_diff) > 0.1:
                self.coords[3] += trigger_diff * step
                moved = True
            # Ry (Pitch): L1 / R1
            if bumper_diff != 0:
                self.coords[4] += bumper_diff * step
                moved = True

            if moved:
                self.send_coords()

def main(args=None):
    rclpy.init(args=args)
    node = DualSenseTeleopNode()
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

if __name__ == "__main__":
    main()
