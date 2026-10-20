#!/usr/bin/env python3
import math
import time
import os
import fcntl
import rclpy
import traceback
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from mycobot_interfaces.srv import (
    SetAngles,
    SetCoords,
    GetCoords,
    GripperStatus,
    GetAngles,
    PumpStatus
)
from std_srvs.srv import SetBool
import pymycobot
from packaging import version

# Minimum pymycobot version requirement
MIN_REQUIRE_VERSION = '3.6.1'
current_version = pymycobot.__version__
if version.parse(current_version) < version.parse(MIN_REQUIRE_VERSION):
    raise RuntimeError(
        f'The version of pymycobot library must be greater than {MIN_REQUIRE_VERSION}. '
        f'The current version is {current_version}.'
    )
from pymycobot import MyCobot280


def acquire(lock_file):
    """
    Non-blocking / micro-sleep file lock.
    Uses 2ms sleep on contention instead of stock 1000ms sleep.
    """
    try:
        fd = os.open(lock_file, os.O_RDWR | os.O_CREAT | os.O_TRUNC)
    except OSError:
        return None
    timeout = 2.0
    start_time = time.time()
    while time.time() < start_time + timeout:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return fd
        except (BlockingIOError, OSError):
            time.sleep(0.002)
    try:
        os.close(fd)
    except Exception:
        pass
    return None


def release(fd):
    """Safely release lock file descriptor."""
    if fd is not None:
        try:
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
        except Exception:
            pass


class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f"Connecting to myCobot on {port} @ {baud} baud...")
        self.mc = MyCobot280(port, str(baud))

        # Always enable fresh mode (1 = execute latest command, don't queue)
        try:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)
            self.get_logger().info("myCobot fresh mode enabled (real-time stream).")
        except Exception as e:
            self.get_logger().warn(f"Failed to set fresh mode: {e}")

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        # State cache for fast non-blocking streaming
        self.cached_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_cmd_time = 0.0
        self.last_serial_query_time = 0.0

        # Gentle 10 Hz timer (instead of stock 50 Hz serial flood)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.srv_angles = self.create_service(
            SetAngles, 'set_angles', self.set_angles_callback)
        self.srv_coords = self.create_service(
            SetCoords, 'set_coords', self.set_coords_callback)
        self.srv_get_coords = self.create_service(
            GetCoords, 'get_coords', self.get_coords_callback)
        self.srv_get_angles = self.create_service(
            GetAngles, 'get_angles', self.get_angles_callback)
        self.srv_gripper = self.create_service(
            GripperStatus, 'set_gripper', self.set_gripper_callback)
        self.srv_pump = self.create_service(
            PumpStatus, 'set_pump_status', self.set_pump_callback)
        self.srv_servo_state = self.create_service(
            SetBool, 'set_servo_state', self.set_servo_state_callback)

        self.get_logger().info("myCobot driver services ready.")

    def publish_joint_states(self):
        try:
            now = time.time()
            # If commands were sent within the last 0.5s, DO NOT poll get_angles() over UART.
            # Polling get_angles() at 115200 baud collides with real-time stream commands.
            if now - self.last_cmd_time < 0.5:
                angles = self.cached_angles
            else:
                # Arm is idle: gently poll hardware at 5 Hz to stay synchronized
                if now - self.last_serial_query_time < 0.2:
                    angles = self.cached_angles
                else:
                    self.last_serial_query_time = now
                    lock = acquire('/tmp/mycobot_lock')
                    try:
                        hw_angles = self.mc.get_angles()
                        if (
                            hw_angles
                            and isinstance(hw_angles, list)
                            and len(hw_angles) == 6
                            and hw_angles[0:3] != [0.0, 0.0, 0.0]
                        ):
                            self.cached_angles = hw_angles
                    finally:
                        release(lock)
                    angles = self.cached_angles

            if not angles or not isinstance(angles, list) or len(angles) != 6:
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
            js.position = [math.radians(float(a)) for a in angles]
            self.pub.publish(js)
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Joint state publish error: {e}")

    def set_angles_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            angles = [
                request.joint_1,
                request.joint_2,
                request.joint_3,
                request.joint_4,
                request.joint_5,
                request.joint_6,
            ]
            speed = int(request.speed)
            self.cached_angles = angles
            self.last_cmd_time = time.time()
            # _async=True sends UART command immediately without blocking for reply
            self.mc.send_angles(angles, speed, _async=True)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetJointAngles service error: {e}")
            response.flag = False
        finally:
            release(lock)
        return response

    def set_coords_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            coords = [
                request.x,
                request.y,
                request.z,
                request.rx,
                request.ry,
                request.rz,
            ]
            speed = int(request.speed)
            model = int(request.model)
            self.last_cmd_time = time.time()
            # _async=True sends UART command immediately without blocking for reply
            self.mc.send_coords(coords, speed, model, _async=True)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"Set coords failed: {e}")
            response.flag = False
        finally:
            release(lock)
        return response

    def get_coords_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            coords = self.mc.get_coords()
            if coords and len(coords) == 6 and all(c != -1 for c in coords):
                response.x = float(coords[0])
                response.y = float(coords[1])
                response.z = float(coords[2])
                response.rx = float(coords[3])
                response.ry = float(coords[4])
                response.rz = float(coords[5])
            else:
                self.get_logger().error("Failed to get coordinates.")
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"GetCoords service error: {e}")
        finally:
            release(lock)
        return response

    def get_angles_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6 and all(a != -1 for a in angles):
                self.cached_angles = angles
                response.joint_1 = float(angles[0])
                response.joint_2 = float(angles[1])
                response.joint_3 = float(angles[2])
                response.joint_4 = float(angles[3])
                response.joint_5 = float(angles[4])
                response.joint_6 = float(angles[5])
            else:
                self.get_logger().error("Failed to get angles.")
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"GetAngles service error: {e}")
        finally:
            release(lock)
        return response

    def set_gripper_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            speed = 80
            if request.status:
                self.mc.set_gripper_state(0, speed, 1)  # open gripper
            else:
                self.mc.set_gripper_state(1, speed, 1)  # close gripper
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetGripper service error: {e}")
            response.flag = False
        finally:
            release(lock)
        return response

    def set_pump_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            pin1 = request.pin1
            pin2 = request.pin2
            if request.status:
                self.mc.set_basic_output(pin2, 0)
                time.sleep(0.05)
            else:
                self.mc.set_basic_output(pin2, 1)
                time.sleep(0.05)
                self.mc.set_basic_output(pin1, 0)
                time.sleep(0.05)
                self.mc.set_basic_output(pin1, 1)
                time.sleep(0.05)
            response.flag = True
        except Exception as e:
            e = traceback.format_exc()
            self.get_logger().error(f"SetPump service error: {e}")
            response.flag = False
        finally:
            release(lock)
        return response

    def set_servo_state_callback(self, request, response):
        lock = acquire('/tmp/mycobot_lock')
        try:
            if request.data:
                # True: Focus / lock servos (holding torque on)
                self.mc.focus_all_servos()
                time.sleep(0.15)
                angles = self.mc.get_angles()
                if angles and len(angles) == 6 and all(a != -1 for a in angles):
                    self.cached_angles = angles
                response.success = True
                response.message = "Servos focused (locked)"
                self.get_logger().info("[*] Servos focused (holding torque on).")
            else:
                # False: Release / relax servos (free hand motion)
                self.mc.release_all_servos()
                response.success = True
                response.message = "Servos relaxed (free-motion enabled)"
                self.get_logger().info("[*] Servos relaxed (damping mode).")
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"SetServoState service error: {e}")
        finally:
            release(lock)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotDriver()
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


if __name__ == "__main__":
    main()
