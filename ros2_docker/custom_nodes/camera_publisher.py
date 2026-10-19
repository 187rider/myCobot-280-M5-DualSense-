#!/usr/bin/env python3
"""
ROS 2 Node: Mac Webcam Publisher
Receives the live webcam stream from the Mac host (http://host.docker.internal:5006/video)
and publishes it as standard ROS 2 Image and CameraInfo topics.

Topics published:
  - /camera/image_raw  (sensor_msgs/msg/Image)
  - /camera/camera_info (sensor_msgs/msg/CameraInfo)
"""

import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header


class MacCameraPublisher(Node):
    def __init__(self):
        super().__init__('mac_camera_publisher')

        self.declare_parameter('url', 'http://host.docker.internal:5006/video')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('fps', 30.0)

        self.stream_url = self.get_parameter('url').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.get_logger().info(f"[+] Mac Camera Publisher connecting to {self.stream_url}...")

        self.cap = None
        self.last_connect_attempt = 0.0

        timer_period = 1.0 / max(1.0, fps)
        self.timer = self.create_timer(timer_period, self.capture_and_publish)

    def get_capture(self):
        if self.cap is not None and self.cap.isOpened():
            return self.cap

        now = time.time()
        if now - self.last_connect_attempt < 2.0:
            return None
        self.last_connect_attempt = now

        try:
            self.cap = cv2.VideoCapture(self.stream_url)
            if self.cap.isOpened():
                self.get_logger().info("[+] Connected to Mac camera stream!")
                return self.cap
            else:
                self.cap = None
                return None
        except Exception as e:
            self.cap = None
            return None

    def capture_and_publish(self):
        cap = self.get_capture()
        if cap is None:
            return

        ret, frame = cap.read()
        if not ret or frame is None:
            self.get_logger().warn("[-] Stream interrupted. Reconnecting...")
            if self.cap is not None:
                self.cap.release()
            self.cap = None
            return

        now_stamp = self.get_clock().now().to_msg()

        # Build Image message
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = now_stamp
        img_msg.header.frame_id = self.frame_id
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()

        # Build basic CameraInfo message
        info_msg = CameraInfo()
        info_msg.header = Header()
        info_msg.header.stamp = now_stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.height = frame.shape[0]
        info_msg.width = frame.shape[1]

        self.image_pub.publish(img_msg)
        self.info_pub.publish(info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MacCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
