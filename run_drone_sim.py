#!/usr/bin/env python3
"""Run the REAL drone_controller code (MQTT.py + DroneController.py + DCmotor.py) in SIMULATION,
with no ROS/MAVROS/Jetson hardware — bridged to a running PX4 SITL instance.

It injects sim_ros/ (simulated rospy + MAVROS bridge over pymavlink, and Jetson.GPIO winch shim) so
the unchanged drone code runs anywhere: it subscribes to PX4 telemetry as if MAVROS were publishing
it, issues set_mode/arm/takeoff/mission-push as MAVLink to PX4, drives the winch via the Gazebo
winch flags, and publishes the SAME MQTT data/<DRONE_ID> a real drone would (-> Sink -> Kafka).
When a real Jetson/Pixhawk drone exists, run drone_controller under genuine ROS+MAVROS instead and
nothing in MQTT.py / DroneController.py changes.

Usage:
    python3 run_drone_sim.py <instance>     # instance i -> PX4 on udp 18570+i, id DRO<51+i>
Env: MQTT_BROKER_HOST (default 127.0.0.1)
Requires a PX4 SITL instance already running (e.g. from simulation/run_airpost_fleet.sh)."""
import os
import runpy
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    inst = sys.argv[1] if len(sys.argv) > 1 else "0"
    os.environ["DRONE_INSTANCE"] = str(inst)
    os.environ.setdefault("MQTT_BROKER_HOST", "127.0.0.1")
    os.environ.setdefault("DRONE_ID", f"DRO{51 + int(inst)}")   # matches the backend drone-id scheme

    sys.path.insert(0, os.path.join(HERE, "sim_ros"))           # rospy/mavros/Jetson shims first
    scripts = os.path.join(HERE, "catkin_ws", "src", "drone_controller", "scripts")
    sys.path.insert(0, scripts)                                  # so MQTT.py's local imports resolve
    os.chdir(scripts)
    print(f"AirPost_Drone SITL: running real drone_controller as {os.environ['DRONE_ID']} "
          f"on PX4 instance {inst} -> MQTT data/{os.environ['DRONE_ID']}", flush=True)
    runpy.run_path(os.path.join(scripts, "MQTT.py"), run_name="__main__")


if __name__ == "__main__":
    main()
