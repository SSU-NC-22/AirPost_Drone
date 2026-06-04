#!/usr/bin/env python3
"""Dummy downward camera as a REAL ROS 2 node, standing in for realsense-ros in SITL. Publishes a
synthetic sensor_msgs/Image at 15 Hz on the SAME topic realsense-ros uses, so the perception graph
(AprilTag/vision pipeline) has a real ROS topic to consume and the integrated drone ROS graph runs
without a physical camera.

Swapping to a real RealSense is config-only — no downstream change:
    don't launch this node; run realsense-ros on the same topic instead, e.g.
        ros2 launch realsense2_camera rs_launch.py   # publishes /camera/color/image_raw
    and set CAMERA_TOPIC to match. Subscribers key off CAMERA_TOPIC, so nothing else changes."""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

# Match realsense-ros's topic so the real camera is a drop-in replacement (override per deployment).
CAMERA_TOPIC = os.environ.get("CAMERA_TOPIC", "/camera/color/image_raw")
CAMERA_W = int(os.environ.get("CAMERA_W", "640"))
CAMERA_H = int(os.environ.get("CAMERA_H", "480"))
CAMERA_HZ = float(os.environ.get("CAMERA_HZ", "15"))


class DummyCamera(Node):
    def __init__(self):
        super().__init__("dummy_camera")
        self.pub = self.create_publisher(Image, CAMERA_TOPIC, 10)
        self.w, self.h = CAMERA_W, CAMERA_H
        self.create_timer(1.0 / CAMERA_HZ, self._tick)
        self.get_logger().info(f"dummy_camera publishing {CAMERA_TOPIC} {self.w}x{self.h} "
                               f"@{CAMERA_HZ:g}Hz (realsense-ros stand-in)")

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
