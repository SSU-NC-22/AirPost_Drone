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
command/downlink/ActuatorReq/<DRONE_ID> order flies the AirPost delivery mission — mirroring
simulation/fleet_service.py: take off to the cruise band, fly to the drop pad and lower the parcel by
winch at ~10 m (hovering, NOT landing), climb back, return to the landing station, and AUTO.PRECLAND on
its AprilTag pad. So the real ROS 2 graph commands the gz/PX4 drone; on real hardware the same node
runs against the real Pixhawk's DDS client.
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
                          VehicleGlobalPosition, VehicleLocalPosition, VehicleStatus, BatteryStatus,
                          ObstacleDistance)
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
# Onboard local obstacle avoidance: drone_node consumes its obstacle sensor (obstacle_sensor node, or
# the real depth/lidar driver) and bends its OFFBOARD setpoints around obstacles — PX4's built-in
# Collision Prevention is Position-mode-only, so in offboard the companion owns avoidance.
OBSTACLE_TOPIC = os.environ.get("OBSTACLE_TOPIC", "/airpost/obstacle_distance")
AVOID_DIST = float(os.environ.get("AVOID_DIST", "8.0"))   # start steering away within this many metres
AVOID_GAIN = float(os.environ.get("AVOID_GAIN", "1.6"))   # repulsion strength
AVOID_MAX = 18.0                                          # cap the lateral diversion (m)


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
        self.obs = None    # latest ObstacleDistance from the onboard sensor (local-NED ring)
        self.create_subscription(ObstacleDistance, OBSTACLE_TOPIC, self._obs, 10)
        self.pub_ocm = self.create_publisher(OffboardControlMode, fmu("in/offboard_control_mode"), 10)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, fmu("in/trajectory_setpoint"), 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, fmu("in/vehicle_command"), 10)
        self.flight_status = None              # mirrors the ROS1 controller's mission status
        self._sp = None                        # active offboard position setpoint [n,e,d]
        self._held = False                     # backend control-tower HOLD (fleet deconfliction)
        # MQTT: same contract as the hardware drone
        self.mq = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, DRONE_ID)
        self.mq.on_connect = lambda c, u, f, rc: (c.subscribe(f"command/downlink/ActuatorReq/{DRONE_ID}"),
                                                   c.subscribe(f"command/downlink/Hold/{DRONE_ID}"),
                                                   print(f"[{DRONE_ID}] MQTT connected", flush=True))
        self.mq.message_callback_add(f"command/downlink/ActuatorReq/{DRONE_ID}", self._on_order)
        self.mq.message_callback_add(f"command/downlink/Hold/{DRONE_ID}", self._on_hold)
        self.mq.connect(MQTT_HOST, 1883); self.mq.loop_start()
        self.create_timer(0.1, self._offboard_heartbeat)   # 10 Hz offboard stream (required by PX4)
        self.create_timer(1.0, self._publish_telemetry)    # 1 Hz telemetry -> data/<id>

    # ---- telemetry callbacks ----
    def _gp(self, m): self.gp = m
    def _lp(self, m): self.lp = m
    def _batt(self, m): self.batt = m
    def _status(self, m): self.status = m
    def _obs(self, m): self.obs = m

    def _avoid_offset(self):
        """Repulsion (north, east) pushing the setpoint away from any obstacle within AVOID_DIST.
        Decodes the local-NED ObstacleDistance ring (index 0 = North, `increment` deg per bin)."""
        m = self.obs
        if m is None:
            return 0.0, 0.0
        rn = re = 0.0
        for i, d_cm in enumerate(m.distances):
            d = d_cm / 100.0
            if d <= 0.0 or d > AVOID_DIST or d_cm >= m.max_distance:   # no obstacle / out of range
                continue
            brg = math.radians(i * m.increment + m.angle_offset)       # obstacle bearing from North
            mag = AVOID_GAIN * (AVOID_DIST - d)                        # stronger the closer it is
            rn -= mag * math.cos(brg)                                  # push AWAY from the obstacle
            re -= mag * math.sin(brg)
        n = math.hypot(rn, re)
        if n > AVOID_MAX:
            rn, re = rn / n * AVOID_MAX, re / n * AVOID_MAX
        return rn, re

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
    def set_offboard(self):  self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)        # OFFBOARD
    def set_precland(self):  self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=4.0, p3=9.0)  # AUTO.PRECLAND
    def land(self):          self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # ---- mission, mirroring simulation/fleet_service.py fly() over the uXRCE-DDS link ----------
    # takeoff -> cruise to the drop pad -> descend to WINCH height -> lower the parcel by winch (hover,
    # NOT land) -> climb back -> cruise to the landing station -> AUTO.PRECLAND on its AprilTag pad.
    def _on_order(self, c, u, msg):
        try:
            order = json.loads(msg.payload.decode())
        except Exception:
            return
        threading.Thread(target=self._fly, args=(order,), daemon=True).start()

    def _on_hold(self, c, u, msg):
        # Backend control-tower deconfliction: hold in place / resume. Fleet-level separation is the
        # backend's job; local obstacle dodging is still the drone's own (_avoid_offset).
        try:
            held = bool(json.loads(msg.payload.decode()).get("hold"))
        except Exception:
            return
        if held != self._held:
            print(f"[{DRONE_ID}] backend {'HOLD' if held else 'CLEAR'}", flush=True)
        self._held = held

    def _fly(self, order):
        cruise   = float(order.get("cruise", 15.0))
        winch_h  = float(order.get("winch_height", 10.0))      # metres above the pad while lowering
        pre_alt  = float(order.get("precland_alt", 3.0))       # align 3 m over the tag, then vision precland
        deliver  = order.get("deliver_ned", [0.0, 12.0])       # NED offset from takeoff to the drop pad
        landing  = order.get("landing_ned", [0.0, 0.0])        # NED offset to the landing station (def: home)
        dworld   = order.get("deliver_world")                  # [E, N, rest_z] gz drop-pad centre (winch slide)
        lworld   = order.get("landing_world")                  # [N, E, station_id, marker_z] (precland target)
        print(f"[{DRONE_ID}] order: deliver@{deliver} winch {winch_h:.0f}m -> land@{landing} (precland)", flush=True)

        # 1) takeoff: stream a climb setpoint, switch to OFFBOARD + arm, wait until at cruise.
        self.flight_status = 1
        self._sp = [0.0, 0.0, -cruise]
        for _ in range(20):
            time.sleep(0.1)
        self.set_offboard(); self.arm()
        self._wait_alt(cruise)

        # 2) delivery leg: cruise to the pad, descend to winch height, lower the parcel, climb back.
        self._deliver(deliver, dworld, cruise, winch_h)

        # 3) return to the landing station and precision-land on its pad.
        self._land_at(landing, lworld, pre_alt)
        self.flight_status = 0

    def _deliver(self, deliver_ned, deliver_world, cruise, winch_h):
        dn, de = float(deliver_ned[0]), float(deliver_ned[1])
        self._goto(dn, de, cruise)                       # arrive over the drop at the cruise band
        self._goto(dn, de, winch_h, z_tol=0.8, avoid=False)  # vertical descent over the pad (no diversion)
        time.sleep(2)
        go_f, done_f, ground_f = (f"/tmp/airpost_winch_{s}_{WINCH_INST}" for s in ("go", "done", "ground"))
        try:
            os.remove(done_f)
        except OSError:
            pass
        try:
            with open(ground_f, "w") as f:
                # "rest_z E N" lets parcel_fleet slide the parcel onto the box centre as it lowers.
                f.write(f"{deliver_world[2]} {deliver_world[0]} {deliver_world[1]}" if deliver_world else "0.29")
            open(go_f, "w").write("go")
        except OSError:
            pass
        print(f"[{DRONE_ID}] lowering parcel by winch at {winch_h:.0f} m", flush=True)
        t0 = time.time(); lowered = False
        while time.time() - t0 < 60.0:        # match fleet_service WINCH_TIMEOUT; hover until delivered
            if os.path.exists(done_f):
                lowered = True
                break
            time.sleep(0.3)
        try:
            os.remove(go_f)
        except OSError:
            pass
        print(f"[{DRONE_ID}] parcel delivered" if lowered else f"[{DRONE_ID}] winch lower timed out", flush=True)
        self._goto(dn, de, cruise)                       # climb back, clearing the drop column

    def _land_at(self, landing_ned, landing_world, pre_alt):
        ln, le = float(landing_ned[0]), float(landing_ned[1])
        if landing_world:                                # tell this drone's detector which tag to seek
            try:
                tmp = f"/tmp/airpost_land_target_{WINCH_INST}.tmp"
                with open(tmp, "w") as f:
                    f.write(" ".join(str(x) for x in landing_world))  # "N E station_id marker_z"
                os.replace(tmp, f"/tmp/airpost_land_target_{WINCH_INST}")
            except OSError:
                pass
        open(f"/tmp/airpost_landing_active_{WINCH_INST}", "w").write("1")  # gate the detector to NOW
        self._goto(ln, le, pre_alt, xy_tol=1.0, z_tol=0.8, avoid=False)  # precise align over the tag
        print(f"[{DRONE_ID}] precision-landing on station pad", flush=True)
        self.set_precland()                              # PX4 AUTO.PRECLAND centres on the LANDING_TARGET
        self._sp = None                                  # stop the offboard stream so PRECLAND owns the drone
        t0 = time.time()
        while time.time() - t0 < 120:
            if self.status is not None and self.status.arming_state == 1:   # DISARMED -> touched down
                print(f"[{DRONE_ID}] landed + disarmed on the pad", flush=True)
                break
            if int(time.time() - t0) % 8 == 0:
                self.set_precland()                      # re-issue in case an ack was dropped
            time.sleep(0.5)
        try:
            os.remove(f"/tmp/airpost_landing_active_{WINCH_INST}")
        except OSError:
            pass

    def _goto(self, n, e, alt, xy_tol=2.0, z_tol=1.5, secs=200, avoid=True):
        """Steer the live OFFBOARD setpoint to (n, e, alt-above-takeoff) and wait until arrival,
        bending the commanded position away from obstacles (onboard local avoidance) en route."""
        t0 = time.time(); warned = False
        while time.time() - t0 < secs:
            lp = self.lp
            # Backend HOLD: hover at the current spot (or the target alt) until cleared — don't make
            # progress toward the target, and don't time out while legitimately holding.
            if self._held:
                if lp is not None:
                    self._sp = [lp.x, lp.y, -abs(float(alt))]
                t0 = time.time()
                time.sleep(0.2)
                continue
            rn, re = self._avoid_offset() if avoid else (0.0, 0.0)
            if (rn or re) and not warned:
                print(f"[{DRONE_ID}] obstacle ahead — steering around", flush=True); warned = True
            self._sp = [float(n) + rn, float(e) + re, -abs(float(alt))]
            if lp is not None and math.hypot(lp.x - n, lp.y - e) < xy_tol and abs(-lp.z - abs(alt)) < z_tol:
                return
            time.sleep(0.2)

    def _wait_alt(self, alt, secs=60):
        t0 = time.time()
        while time.time() - t0 < secs:
            lp = self.lp
            if lp is not None and -lp.z >= alt - 1.0:
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
