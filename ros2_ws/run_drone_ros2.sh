#!/usr/bin/env bash
# Bring up the REAL ROS 2 drone graph against a running PX4 v1.17 SITL instance (no MAVROS, no
# hardware). It (1) runs the Micro-XRCE-DDS Agent so PX4's uxrce_dds_client bridges its topics to
# ROS 2, then (2) launches the airpost_drone graph (controller + dummy camera). Runs inside the
# `airpost-ros2` Humble container (host network -> reaches the host's PX4 SITL + mosquitto).
#
#   ./run_drone_ros2.sh [instance]      # default 0  (PX4 instance i, DRONE_ID DRO<51+i>)
set -uo pipefail
INST="${1:-0}"
DID="DRO$((51 + INST))"
C=airpost-ros2

# 1) Micro-XRCE-DDS Agent on :8888 (PX4 SITL's uxrce_dds_client connects to it). Idempotent.
docker exec "$C" bash -lc "pgrep -f MicroXRCEAgent >/dev/null || (MicroXRCEAgent udp4 -p 8888 >/tmp/agent_run.log 2>&1 &)"
sleep 3
# 2) integrated drone graph: real ROS2 nodes, telemetry from PX4 over DDS, MQTT data/<id> out.
docker exec -e DRONE_ID="$DID" -e DRONE_INSTANCE="$INST" -e MQTT_BROKER_HOST=127.0.0.1 "$C" \
  bash -lc ". /opt/ros/humble/setup.bash && . /ros2_ws/install/setup.bash && \
            ros2 launch airpost_drone all_drone_control.launch.py"
