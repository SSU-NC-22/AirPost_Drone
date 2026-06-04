"""Simulated rospy + MAVROS bridge so the REAL drone_controller (MQTT.py / DroneController.py) runs
in SITL with no ROS install. It bridges the MAVROS topics/services the drone code uses to a PX4 SITL
instance over MAVLink (pymavlink), translating the (ArduPilot-style) calls to PX4:

  Subscribers  -> a background thread reads PX4 MAVLink and delivers the messages the code expects
                  (/mavros/global_position/global, /battery, /rel_alt, /mission/reached, gps_vel).
  ServiceProxy -> /mavros/set_mode, /cmd/arming, /cmd/takeoff, /mission/push, /mission/clear become
                  the matching MAVLink commands / mission upload to PX4.

When a real drone exists, the genuine rospy + MAVROS run on the Jetson and this shim is off the path.
PX4 instance i is reached on its GCS MAVLink port 18570+i (DRONE_INSTANCE env)."""
import os
import threading
import time

from pymavlink import mavutil

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import WaypointReached


class ServiceException(Exception):
    pass


# ---- one shared MAVLink link to this drone's PX4 instance -------------------------------------
class _Bridge:
    def __init__(self):
        self.inst = int(os.environ.get("DRONE_INSTANCE", "0"))
        # Bind PX4's onboard link (px4-rc.mavlink: `-m onboard -o 14540+i`): PX4 *pushes* its telemetry
        # stream to this fixed port, and we reply to that same peer to send commands — reliable, unlike
        # the GCS port 18570 which locks to its first partner (e.g. QGroundControl) and won't stream back.
        port = 14540 + self.inst
        self.m = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}", source_system=255, source_component=190)
        self.subs = []          # (topic, msg_type, callback)
        self._last_seq = -1
        # request the streams the drone code needs, then pump messages to subscribers
        threading.Thread(target=self._rx_loop, daemon=True).start()

    def add_sub(self, topic, msg_type, cb):
        self.subs.append((topic, msg_type, cb))

    def _request_streams(self):
        # heartbeat so PX4 records us as a GCS peer, then ask for the messages we translate
        try:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            for mid, hz in ((33, 5), (24, 5), (1, 2), (147, 5), (46, 5)):  # GLOBAL_POSITION_INT, GPS_RAW, SYS_STATUS, BATTERY_STATUS, MISSION_ITEM_REACHED
                self.m.mav.command_long_send(self.inst + 1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                             0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)
        except Exception:
            pass

    def _rx_loop(self):
        next_hb = 0.0
        while True:
            now = time.time()
            if now >= next_hb:
                self._request_streams(); next_hb = now + 1.0
            msg = self.m.recv_match(blocking=True, timeout=0.3)
            if msg is None:
                continue
            t = msg.get_type()
            for topic, _mt, cb in self.subs:
                try:
                    if t == "GLOBAL_POSITION_INT" and topic.endswith("global_position/global"):
                        n = NavSatFix(); n.latitude = msg.lat / 1e7; n.longitude = msg.lon / 1e7; n.altitude = msg.alt / 1e3
                        cb(n)
                    elif t == "GLOBAL_POSITION_INT" and topic.endswith("rel_alt"):
                        f = Float64(); f.data = msg.relative_alt / 1e3; cb(f)
                    elif t in ("BATTERY_STATUS", "SYS_STATUS") and topic.endswith("battery"):
                        b = BatteryState()
                        b.voltage = (getattr(msg, "voltages", [0])[0] or 0) / 1000.0 if t == "BATTERY_STATUS" else getattr(msg, "voltage_battery", 0) / 1000.0
                        rem = getattr(msg, "battery_remaining", -1)
                        b.percentage = (rem / 100.0) if rem >= 0 else 1.0
                        cb(b)
                    elif t == "MISSION_ITEM_REACHED" and topic.endswith("mission/reached"):
                        cb(WaypointReached(wp_seq=msg.seq))
                except Exception:
                    pass


_bridge = None


def _get_bridge():
    global _bridge
    if _bridge is None:
        _bridge = _Bridge()
    return _bridge


# ---- rospy API surface used by the drone code ------------------------------------------------
def init_node(name, anonymous=False):
    _get_bridge()
    print(f"[sim_ros] node '{name}' up; bridged to PX4 instance {_bridge.inst} (udp {14540 + _bridge.inst})", flush=True)


class Rate:
    def __init__(self, hz): self.dt = 1.0 / max(1e-6, hz)
    def sleep(self): time.sleep(self.dt)


def is_shutdown():
    return False


class Subscriber:
    def __init__(self, topic, msg_type, callback, *a, **k):
        _get_bridge().add_sub(topic, msg_type, callback)


class Publisher:
    def __init__(self, topic, msg_type, queue_size=1): self.topic = topic
    def publish(self, *a, **k): pass   # /mavros/rc/override is unused in the autonomous flow


class ServiceProxy:
    """Routes a MAVROS service call to the matching PX4 MAVLink command/mission op (by service name)."""
    def __init__(self, name, srv_type, persistent=False):
        self.name = name.lstrip("/")
        self.b = _get_bridge()

    def __call__(self, *args, **kwargs):
        return self.call(*args, **kwargs)

    def call(self, *args, **kwargs):
        b, m, tgt = self.b, self.b.m, (self.b.inst + 1)
        if self.name.endswith("set_mode"):
            mode = str(kwargs.get("custom_mode", args[0] if args else "9"))
            # map ArduPilot-style intent -> PX4 custom main/sub mode (base mode = CUSTOM_MODE_ENABLED=1)
            px4 = {"3": (4, 4), "4": (4, 0), "9": (6, 0)}.get(mode, (4, 4))  # AUTO.MISSION / AUTO(takeoff) / AUTO.LAND
            m.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, px4[0], px4[1], 0, 0, 0, 0)
            return type("R", (), {"mode_sent": True, "success": True})()
        if self.name.endswith("cmd/arming"):
            val = kwargs.get("value", args[0] if args else True)
            m.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if val else 0, 0, 0, 0, 0, 0, 0)
            return type("R", (), {"success": True})()
        if self.name.endswith("cmd/takeoff"):
            alt = kwargs.get("altitude", 2.0)
            m.mav.command_long_send(tgt, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, float(alt))
            return type("R", (), {"success": True})()
        if self.name.endswith("mission/clear"):
            try: m.mav.mission_clear_all_send(tgt, 1)
            except Exception: pass
            return type("R", (), {"success": True})()
        if self.name.endswith("mission/push"):
            wps = kwargs.get("waypoints", [])
            _upload_mission(m, tgt, wps)
            return type("R", (), {"success": True})()
        return type("R", (), {"success": True})()


def _upload_mission(m, tgt, wps):
    """Upload a MAVLink mission from the drone code's Waypoint list (frame 3 = GLOBAL_RELATIVE_ALT)."""
    try:
        m.mav.mission_count_send(tgt, 1, len(wps))
        for i, wp in enumerate(wps):
            m.mav.mission_item_int_send(
                tgt, 1, i, getattr(wp, "frame", 3), getattr(wp, "command", 16),
                0, int(getattr(wp, "autocontinue", True)),
                float(getattr(wp, "param1", 0)), float(getattr(wp, "param2", 0)),
                float(getattr(wp, "param3", 0)), float(getattr(wp, "param4", 0)),
                int(getattr(wp, "x_lat", 0) * 1e7), int(getattr(wp, "y_long", 0) * 1e7),
                float(getattr(wp, "z_alt", 0)), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            time.sleep(0.05)
        print(f"[sim_ros] uploaded {len(wps)} mission items to PX4", flush=True)
    except Exception as e:
        print(f"[sim_ros] mission upload failed: {e}", flush=True)
