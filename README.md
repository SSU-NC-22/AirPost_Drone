# AirPost_Drone — the flying courier (on-drone ROS 2 software + hardware)

This repository is the **drone itself** in the [AirPost](https://github.com/jsoone24/NC_AirPost)
autonomous parcel-delivery system: the physical aircraft, plus the software that runs **on board** to
fly missions, deliver the parcel, and camera-land precisely on a station.

The same code is exercised **end-to-end in simulation** (PX4 v1.17 SITL + Gazebo) before it ever
touches hardware — and it transfers to the real Pixhawk+Jetson with no logic changes, because in both
cases it speaks to PX4 over the **native uXRCE-DDS bridge** (ROS 2), not a desktop-only shim.

## What it is, in plain terms

A delivery drone is really two computers bolted together:

1. **A flight controller (Pixhawk)** — the real-time autopilot that keeps the aircraft stable and
   executes low-level flight (arm, take off, go to a point, land). It runs **PX4 v1.17**.
2. **A companion computer (NVIDIA Jetson)** — a small Linux box that does the "thinking" the autopilot
   can't: runs the **ROS 2** delivery logic + camera/AprilTag vision for precision landing, talks to
   the rest of AirPost over LTE/MQTT, and commands the Pixhawk.

The companion talks to PX4 over **uXRCE-DDS**: PX4's `uxrce_dds_client` exposes the flight stack as
ROS 2 topics (`/fmu/out/*` telemetry, `/fmu/in/*` commands) through a Micro-XRCE-DDS Agent. Our ROS 2
node subscribes to the drone's **own** state and flies it in OFFBOARD mode — no MAVROS, no MAVLink
translation layer.

A downward camera (Intel RealSense) feeds the AprilTag vision for precision landing, and a winch motor
lowers the parcel without landing.

## Where it sits in AirPost

```
AirPost backend ──MQTT(flight order)──► AirPost_Drone companion (ROS 2)
                                          │  uXRCE-DDS  (/fmu/in/* commands)
                                          ▼
                                        PX4 v1.17 autopilot ──► motors, GPS, sensors
                                          │  uXRCE-DDS  (/fmu/out/* telemetry)
                                          ▼
                  camera ──► AprilTag ──► precision-land on the station marker
AirPost_Drone ──MQTT(GPS, status, battery)──► Sink → Kafka → backend (live tracking + storage)
```

- **Receives** a flight order over MQTT from the backend (drop point, landing station).
- **Flies it** by commanding PX4 over DDS (arm → take off → cruise → winch the parcel down → land).
- **Reports** GPS / status / battery — read from PX4's *own* telemetry over DDS — back over MQTT so the
  operator sees it live and the data is archived.

---

## Branches — pick the autopilot/ROS pairing you run

| Branch    | ROS            | Autopilot link                | Use it for |
|-----------|----------------|-------------------------------|------------|
| **`main`**   | **ROS 2 Humble** | **PX4 v1.17 · uXRCE-DDS** | Current. The migrated, working on-drone stack (this README). |
| `humble`  | ROS 2 Humble   | PX4 v1.17 · uXRCE-DDS         | Same ROS 2 migration, named for the ROS distro. |
| `noetic`  | ROS 1 Noetic   | MAVROS / MAVLink              | The legacy ROS 1 controller (`catkin_ws/drone_controller`), preserved for reference. |

The jump from `noetic` to `main` is the migration from **ROS 1 + MAVROS** to **ROS 2 + native DDS**,
matching PX4 v1.17's message-versioned uXRCE-DDS interface.

## The ROS 2 package: `ros2_ws/src/airpost_drone`

Two real ROS 2 nodes — the entire on-drone graph:

- **`drone_node`** (`airpost_drone/drone_node.py`) — the controller.
  - *Subscribes* (the drone's **own** autopilot state over DDS): `/fmu/out/vehicle_global_position`,
    `vehicle_local_position_v1`, `battery_status_v1`, `vehicle_status_v1`.
  - *Publishes* (commands): `/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command`
    (arm, set OFFBOARD, land).
  - On an MQTT order (`command/downlink/ActuatorReq/<id>`) it flies a delivery in OFFBOARD —
    takeoff → waypoints → winch-drop → land — and streams telemetry on `data/<id>` (→ Sink → Kafka).
  - **Multi-drone:** set `PX4_NS=px4_<key>` and it drives `/px4_<key>/fmu/...`, targeting
    `VehicleCommand`s at `MAV_SYS_ID = instance+1`, so many drones share one agent without collision.
- **`dummy_camera`** (`airpost_drone/dummy_camera.py`) — a **real ROS 2 node** that stands in for
  realsense-ros in simulation, publishing `sensor_msgs/Image` on the RealSense topic so the perception
  graph runs without a physical camera.

### Swapping the dummy camera for a real RealSense — config only

`dummy_camera` publishes on **`$CAMERA_TOPIC`** (default `/camera/color/image_raw`, the realsense-ros
topic). To use a real camera, **don't launch `dummy_camera`** — run realsense-ros on the same topic:

```bash
ros2 launch realsense2_camera rs_launch.py     # publishes /camera/color/image_raw
```

Nothing downstream changes: every subscriber keys off `CAMERA_TOPIC`. The dummy node exists only so the
integrated graph is testable without hardware.

---

## Try it in simulation (no hardware)

The umbrella repo's [`simulation/`](https://github.com/jsoone24/NC_AirPost/tree/main/simulation) runs
this exact ROS 2 stack against PX4 v1.17 SITL + Gazebo:

```bash
cd NC_AirPost/simulation
./run_ros2_fleet.sh 1          # 1 drone: Agent + PX4/gz + drone_node + dummy_camera + GCS heartbeat
# then send a delivery order (the backend dispatcher does this in production):
#   mosquitto_pub -t command/downlink/ActuatorReq/DRO51 -m \
#     '{"takeoff_alt":6.0,"tagidx":1,"values":[[0,0,6,16],[0,12,6,16]]}'
```

`run_ros2_fleet.sh N` launches N drones (each on DDS namespace `px4_<key>`). Telemetry appears on
`data/DRO51..` exactly as from the real aircraft. See the simulation README for the dependency setup
(ros-humble env, the Micro-XRCE-DDS Agent, px4_msgs).

> **Why a GCS heartbeat?** PX4 treats the DDS link as the onboard control link, not a ground station.
> With no MAVLink GCS it boots into a datalink-loss state and won't arm. `gcs_link.py` provides the
> heartbeat a real telemetry radio/ground station would; the airframe also disables the RC/datalink
> failsafes for autonomous offboard flight (`NAV_DLL_ACT=0`, `COM_RCL_EXCEPT=4`).

---

## H/W Drone Parts

1. [Nvidia] Jetson Nano Dev. kit
2. [Waveshare] SIM7600G-H LTE Module for Jetson Nano
3. Motor Driver HG7881
4. Step down circuit XL4015
5. Li-po battery 5800mah
6. [HOLYBRO] Pixhawk 4 Power Module (PM07)
7. [Intel] RealSense Tracking/Depth Camera
8. [HOLYBRO] Pixhawk4 500mW Telemetry 433Mhz
9. [TMOTOR] AIR2213 KV920 BLDC Motor, T9545 Prop
10. [TMOTOR] AIR20A ESC
11. [Motorbank] GM24-KTX Dc motor (winch)
12. [HOLYBRO] Pixhawk 4
13. [HOLYBRO] Pixhawk 4 GPS Module
14. [RadioLink] T8FB BT Transmitter, Receiver set
15. dji f450 drone frame

## On-hardware bring-up (ROS 2 / PX4 v1.17)

The companion runs the **same `airpost_drone` package** as in simulation; only the DDS endpoints point
at the real Pixhawk instead of SITL.

1. **Pixhawk:** flash PX4 v1.17, enable the DDS client (`UXRCE_DDS_CFG` → the companion serial/UDP
   link), and load the autonomous-offboard params (`COM_RC_IN_MODE=4`, `COM_RCL_EXCEPT=4`,
   `NAV_RCL_ACT=0`, `NAV_DLL_ACT=0`).
2. **Jetson:** Ubuntu + ROS 2 Humble + `px4_msgs` (release/1.17). Build:
   ```bash
   colcon build --packages-select px4_msgs airpost_drone
   ```
3. **Micro-XRCE-DDS Agent** on the companion, bridging the Pixhawk link to DDS:
   ```bash
   MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600     # or  udp4 -p 8888
   ```
4. **Camera:** `ros2 launch realsense2_camera rs_launch.py` (replaces `dummy_camera`).
5. **Run the stack:**
   ```bash
   ros2 run airpost_drone drone_node       # + your perception/precision-landing nodes
   ```
6. **Boot autostart / connectivity:** the LTE module + reverse-SSH tunnel (`scripts/`) bring the drone
   online; the backend then sends flight orders over MQTT.

## CI

`.github/workflows/ros2-build.yml` builds `airpost_drone` + `px4_msgs` in a clean `ros:humble`
container and smoke-tests that both executables install and the nodes import — the deployment image,
proven on every push.

## Troubleshooting

- **`ros2 topic list` shows no `/fmu/*`:** the DDS session isn't up. Confirm the Agent is running and
  PX4's `uxrce_dds_client` is connected (`uxrce_dds_client status` in the PX4 shell). On macOS, pin DDS
  discovery to loopback with the provided `fastdds_localhost.xml` (multicast doesn't loop reliably).
- **Drone won't arm (offboard):** PX4 needs a GCS heartbeat and the RC/datalink failsafes off — see the
  GCS-heartbeat note above. The airframe sets these by default for the airpost delivery model.
- **`ros2 run` can't find the executable:** rebuild — `setup.cfg` installs the console scripts under
  `lib/airpost_drone/` where `ros2 run/launch` looks.

## Reference

1. [PX4 uXRCE-DDS (ROS 2) interface](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
2. [px4_msgs](https://github.com/PX4/px4_msgs) · [px4_ros_com offboard example](https://github.com/PX4/px4_ros_com)
3. [Micro-XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
4. [Precision landing with vision + AprilTag](https://docs.px4.io/main/en/advanced_features/precland.html)
5. [Holybro X500 + Pixhawk assembly](https://docs.px4.io/main/en/frames_multicopter/holybro_x500_pixhawk4.html)
6. [RealSense ROS 2 (realsense-ros)](https://github.com/IntelRealSense/realsense-ros)
7. [lte module wiki](https://www.waveshare.com/wiki/SIM7600G-H_4G_for_Jetson_Nano) · [reverse ssh](https://system-monitoring.readthedocs.io/en/latest/ssh.html)
