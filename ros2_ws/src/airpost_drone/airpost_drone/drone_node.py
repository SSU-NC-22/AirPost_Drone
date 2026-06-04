#!/usr/bin/env python3
"""AirPost drone controller — ROS 2 (Humble) port for PX4 v1.17 via uXRCE-DDS (px4_msgs).

This is the v1.17-compatible migration of the ROS1/MAVROS drone_controller (preserved on the `noetic`
branch). It talks to PX4 over the native DDS bridge (no MAVROS), using the real px4_msgs topics that
PX4 v1.17 exports (dds_topics.yaml):

  Subscribes (telemetry — the drone's OWN autopilot state, sent over DDS, not gz ground truth):
    /fmu/out/vehicle_global_position  (VehicleGlobalPosition: lat/lon/alt)
    /fmu/out/battery_status           (BatteryStatus)
    /fmu/out/vehicle_local_position   (VehicleLocalPosition: NED, for offboard control)
    /fmu/out/vehicle_status           (arming/nav state)
  Publishes (commands):
    /fmu/in/offboard_control_mode + /fmu/in/trajectory_setpoint  (offboard position flight)
    /fmu/in/vehicle_command           (arm, set OFFBOARD, land)

It keeps the SAME MQTT contract as the hardware drone: streams telemetry on data/<DRONE_ID>, and on a
command/downlink/ActuatorReq/<DRONE_ID> order flies a delivery (takeoff -> drop point -> winch the
parcel down -> land), driving the winch through the Gazebo winch flags. So the real ROS 2 graph
commands the gz/PX4 drone; on real hardware the same node runs against the real Pixhawk's DDS client.
"""
import json
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus, BatteryStatus)
import paho.mqtt.client as mqtt

DRONE_ID = os.environ.get("DRONE_ID", "DRO51")
MQTT_HOST = os.environ.get("MQTT_BROKER_HOST", "127.0.0.1")
WINCH_INST = os.environ.get("DRONE_INSTANCE", "0")
# Multi-drone: every PX4 instance shares one agent on :8888, so each is booted with a distinct
# uXRCE-DDS namespace (PX4_UXRCE_DDS_NS=px4_<key>, key=instance+1) and its topics live under
# /px4_<key>/fmu/... . PX4_NS selects which drone this node drives; empty = the single-drone
# default (/fmu/...). MAV_SYS_ID = instance+1 targets VehicleCommands at the right autopilot.
PX4_NS = os.environ.get("PX4_NS", "").strip("/")
MAV_SYS_ID = int(WINCH_INST) + 1


def fmu(topic):
    """Build a /fmu topic, namespaced per drone when PX4_NS is set (e.g. /px4_1/fmu/out/...)."""
    return f"/{PX4_NS}/fmu/{topic}" if PX4_NS else f"/fmu/{topic}"


# PX4 publishes /fmu/out/* with BEST_EFFORT + KEEP_LAST + TRANSIENT_LOCAL; subscribers must match.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=10)


class DroneNode(Node):
    def __init__(self):
        super().__init__("airpost_drone")
        self.gp = None; self.lp = None; self.batt = None; self.status = None
        # PX4 v1.17 exports message-versioned topics with a _vN suffix (vehicle_global_position is
        # currently unversioned). Names confirmed from `ros2 topic list -t` against the live bridge.
        self.create_subscription(VehicleGlobalPosition, fmu("out/vehicle_global_position"), self._gp, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, fmu("out/vehicle_local_position_v1"), self._lp, PX4_QOS)
        self.create_subscription(BatteryStatus, fmu("out/battery_status_v1"), self._batt, PX4_QOS)
        self.create_subscription(VehicleStatus, fmu("out/vehicle_status_v1"), self._status, PX4_QOS)
        self.pub_ocm = self.create_publisher(OffboardControlMode, fmu("in/offboard_control_mode"), 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, fmu("in/trajectory_setpoint"), 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, fmu("in/vehicle_command"), 10)
        self.flight_status = None              # mirrors the ROS1 controller's mission status
        self._sp = None                        # active offboard position setpoint [n,e,d]
        # MQTT: same contract as the hardware drone
        self.mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, DRONE_ID)
        self.mq.on_connect = lambda c, u, f, rc: (c.subscribe(f"command/downlink/ActuatorReq/{DRONE_ID}"),
                                                   print(f"[{DRONE_ID}] MQTT connected", flush=True))
        self.mq.message_callback_add(f"command/downlink/ActuatorReq/{DRONE_ID}", self._on_order)
        self.mq.connect(MQTT_HOST, 1883); self.mq.loop_start()
        self.create_timer(0.1, self._offboard_heartbeat)   # 10 Hz offboard stream (required by PX4)
        self.create_timer(1.0, self._publish_telemetry)    # 1 Hz telemetry -> data/<id>

    # ---- telemetry callbacks ----
    def _gp(self, m): self.gp = m
    def _lp(self, m): self.lp = m
    def _batt(self, m): self.batt = m
    def _status(self, m): self.status = m

    def _publish_telemetry(self):
        gp, batt = self.gp, self.batt
        msg = {"node_id": DRONE_ID, "values": [
            getattr(gp, "lat", None), getattr(gp, "lon", None), getattr(gp, "alt", None),
            None,
            (batt.remaining * 100.0) if batt else None,
            self.flight_status,
        ], "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")}
        self.mq.publish(f"data/{DRONE_ID}", json.dumps(msg))

    # ---- offboard control ----
    def _now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _offboard_heartbeat(self):
        # PX4 requires OffboardControlMode at >2 Hz to stay in / enter OFFBOARD.
        if self._sp is None:
            return
        ocm = OffboardControlMode(); ocm.timestamp = self._now_us()
        ocm.position = True
        self.pub_ocm.publish(ocm)
        sp = TrajectorySetpoint(); sp.timestamp = self._now_us()
        sp.position = [float(self._sp[0]), float(self._sp[1]), float(self._sp[2])]
        sp.yaw = float("nan")
        self.pub_sp.publish(sp)

    def _cmd(self, command, **params):
        c = VehicleCommand(); c.timestamp = self._now_us()
        c.command = command
        for i in range(1, 8):
            setattr(c, f"param{i}", float(params.get(f"p{i}", 0.0)))
        c.target_system = MAV_SYS_ID; c.target_component = 1; c.source_system = MAV_SYS_ID; c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def arm(self):    self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
    def disarm(self): self._cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=0.0)
    def set_offboard(self): self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
    def land(self):   self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def _winch_drop(self):
        try:
            open(f"/tmp/airpost_winch_ground_{WINCH_INST}", "w").write("0.25")
            open(f"/tmp/airpost_winch_go_{WINCH_INST}", "w").write("go")
        except OSError:
            pass

    # ---- mission: triggered by an MQTT order, flown in OFFBOARD over local-NED setpoints ----
    def _on_order(self, c, u, msg):
        try:
            order = json.loads(msg.payload.decode())
        except Exception:
            return
        threading.Thread(target=self._fly, args=(order,), daemon=True).start()

    def _fly(self, order):
        # order carries NED waypoints relative to takeoff: [[n,e,down,cmd]...] + tagidx (drop index)
        wpl = order.get("values", [])
        tagidx = int(order.get("tagidx", max(0, len(wpl) - 2)))
        print(f"[{DRONE_ID}] order: {len(wpl)} waypoints, drop at {tagidx}", flush=True)
        # 1) start streaming a setpoint (current/takeoff), then arm + OFFBOARD
        self._sp = [0.0, 0.0, -float(order.get("takeoff_alt", 5.0))]
        for _ in range(20):
            time.sleep(0.1)
        self.set_offboard(); self.arm()
        # 2) fly each waypoint; winch-drop at tagidx
        for i, wp in enumerate(wpl):
            self._sp = [float(wp[0]), float(wp[1]), -abs(float(wp[2]))]
            self.flight_status = 1 if i >= tagidx else 0
            self._settle(self._sp)
            if i == tagidx:
                print(f"[{DRONE_ID}] ***winch drop***", flush=True); self._winch_drop(); time.sleep(8)
        # 3) land
        self.land()
        print(f"[{DRONE_ID}] landing", flush=True)

    def _settle(self, target, tol=1.0, secs=60):
        t0 = time.time()
        while time.time() - t0 < secs:
            lp = self.lp
            if lp is not None:
                if math.dist((lp.x, lp.y, lp.z), target) < tol:
                    return
            time.sleep(0.2)


def main():
    rclpy.init()
    node = DroneNode()
    print(f"[{DRONE_ID}] ROS2 drone node up; bridged to PX4 v1.17 over uXRCE-DDS", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
