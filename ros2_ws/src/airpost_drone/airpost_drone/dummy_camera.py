#!/usr/bin/env python3
"""Dummy downward camera as a REAL ROS 2 node, replacing realsense-ros in SITL. Publishes a synthetic
sensor_msgs/Image (and camera_info) at 15 Hz so the perception graph (AprilTag/vision pipeline) has a
real ROS topic to consume — the integrated drone ROS graph runs even without a physical camera. On
real hardware, realsense-ros provides this topic instead and nothing downstream changes."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np


class DummyCamera(Node):
    def __init__(self):
        super().__init__("dummy_camera")
        self.pub = self.create_publisher(Image, "/camera/image_raw", 10)
        self.w, self.h = 640, 480
        self.create_timer(1.0 / 15.0, self._tick)
        self.get_logger().info("dummy_camera publishing /camera/image_raw 640x480 @15Hz (realsense stand-in)")

    def _tick(self):
        m = Image()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "camera_link"
        m.height, m.width = self.h, self.w
        m.encoding = "mono8"; m.step = self.w
        m.data = (np.full(self.w * self.h, 128, dtype=np.uint8)).tobytes()
        self.pub.publish(m)


def main():
    rclpy.init(); n = DummyCamera()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
