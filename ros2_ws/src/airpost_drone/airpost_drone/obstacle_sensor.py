#!/usr/bin/env python3
"""Onboard obstacle sensor as a REAL ROS 2 node — the drone's local perception for collision
avoidance. It models a 360deg horizontal range sensor (lidar/depth ring) and publishes the standard
px4_msgs/ObstacleDistance the avoidance layer consumes.

WHY this lives on the companion (not PX4): PX4's built-in Collision Prevention (CP_DIST) is "only used
in Position mode" and is NOT wired into AUTO/OFFBOARD — so during the companion-driven OFFBOARD
delivery flight, avoiding obstacles is the companion's job (exactly what PX4 expects for offboard).
drone_node subscribes to this node's ObstacleDistance and bends its setpoints around obstacles.

In SIM the obstacles are read from OBSTACLES (local-NED circles relative to the drone's takeoff point,
"N,E,R;N,E,R"); on real hardware this node is replaced by the RealSense/lidar driver publishing the
same ObstacleDistance topic, and nothing downstream changes.

Env: OBSTACLES (default empty), PX4_NS (namespace), SENSOR_RANGE m (default 20), OBSTACLE_TOPIC."""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import ObstacleDistance, VehicleLocalPosition

PX4_NS = os.environ.get("PX4_NS", "").strip("/")
RANGE_M = float(os.environ.get("SENSOR_RANGE", "20"))
N_BINS = 72                       # 5deg per bin, index 0 = local North (ObstacleDistance convention)
INCREMENT = 360.0 / N_BINS
OBSTACLE_TOPIC = os.environ.get("OBSTACLE_TOPIC", "/airpost/obstacle_distance")
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=10)


def _parse_obstacles(spec):
    out = []
    for part in spec.split(";"):
        part = part.strip()
        if not part:
            continue
        try:
            n, e, r = (float(x) for x in part.split(","))
            out.append((n, e, r))
        except ValueError:
            pass
    return out


def fmu(topic):
    return f"/{PX4_NS}/fmu/{topic}" if PX4_NS else f"/fmu/{topic}"


class ObstacleSensor(Node):
    def __init__(self):
        super().__init__("obstacle_sensor")
        self.obstacles = _parse_obstacles(os.environ.get("OBSTACLES", ""))
        self.x = self.y = 0.0
        self.create_subscription(VehicleLocalPosition, fmu("out/vehicle_local_position_v1"), self._lp, PX4_QOS)
        self.pub = self.create_publisher(ObstacleDistance, OBSTACLE_TOPIC, 10)
        self.create_timer(0.1, self._tick)    # 10 Hz range scan
        self.get_logger().info(f"obstacle_sensor: {len(self.obstacles)} obstacle(s), range {RANGE_M:g} m "
                               f"-> {OBSTACLE_TOPIC} (range-sensor stand-in for the real depth/lidar driver)")

    def _lp(self, m):
        self.x, self.y = m.x, m.y          # local-NED north, east

    def _tick(self):
        rmax = int(RANGE_M * 100)          # cm
        dist = [rmax + 1] * N_BINS          # "no obstacle" sentinel
        for on, oe, orad in self.obstacles:
            dn, de = on - self.x, oe - self.y
            d = math.hypot(dn, de) - orad   # surface distance
            if d > RANGE_M:
                continue
            bearing = math.degrees(math.atan2(de, dn)) % 360.0   # 0 = North, CW positive
            # mark the angular span the obstacle subtends (half-angle from its radius)
            half = math.degrees(math.atan2(orad, max(0.5, math.hypot(dn, de)))) + INCREMENT
            d_cm = max(0, int(d * 100))
            span = max(1, int(half / INCREMENT))
            c = int(bearing / INCREMENT)
            for k in range(c - span, c + span + 1):
                b = k % N_BINS
                dist[b] = min(dist[b], d_cm)
        msg = ObstacleDistance()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.frame = ObstacleDistance.MAV_FRAME_LOCAL_NED
        msg.sensor_type = ObstacleDistance.MAV_DISTANCE_SENSOR_LASER
        msg.distances = dist
        msg.increment = INCREMENT
        msg.min_distance = 20
        msg.max_distance = rmax
        msg.angle_offset = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init(); n = ObstacleSensor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
