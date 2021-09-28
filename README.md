# AirPost_Drone
Code and instruction for configuring drone.

## H/W Drone Parts

1. [Nvidia] Jetson Nano Dev. kit
2. [Waveshare] SIM7600G-H LTE Module for Jetson Nano
3. Motor Driver HG7881
4. Step down circuit XL4015
5. Li-po battery 5800mah
6. [HOLYBRO] Pixhawk 4 Power Module (PM07)
7. [Intel] RealSense TrackingCamera T265
8. [HOLYBRO] Pixhawk4 500mW Telemetry 433Mhz
9. [TMOTOR] AIR2213 KV920 BLDC Motor, T9545 Prop
10. [TMOTOR] AIR20A ESC
11. [Motorbank] GM24-KTX Dc motor
12. [HOLYBRO] Pixhawk 4
13. [HOLYBRO] Pixhawk 4 GPS Module
14. [RadioLink] T8FB BT Transmitter, Receiver set
15. dji f450 drone frame

## S/W on Jetson nano

1. Jetpack 4.2
2. opencv 3.2
3. librealsense v2.25.0
4. ROS melodic
   1. mavros
   2. apriltag-ros
   3. Realsense_ros v2.2.8
   4. vision_to_mavros
   5. rosbags
5. apriltag

## Installation Guide

1. Jetpack 4.2
   1. format sd card in windows computer with default settings
   2. format sdcard with sd card formatter in the link below
   3. download jetpack image and flash
   4. https://developer.nvidia.com/embedded/jetpack
2. useful packages
   1. ```sudo apt update```
   2. ```sudo apt upgrade```
   3. ```sudo apt install build-essential cmake git vim tmux terminator```
   4. [install jtop](https://github.com/rbonghi/jetson_stats)
   5. [vim setting](https://hyoje420.tistory.com/51)
3. [opencv 3.2](https://github.com/jetsonhacks/buildOpenCVXavier)
4. [JetsonHacksNano librealsense  v2.25.0](https://github.com/JetsonHacksNano/installLibrealsense/tree/vL4T32.2.1)
   1. [librealsense v2.25.0 source](https://github.com/IntelRealSense/librealsense/releases/tag/v2.25.0)
5. [apriltag](https://github.com/AprilRobotics/apriltag)
6. [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
   1. with catkin_build in catkin_ws_build directory
      1. [mavros](https://ardupilot.org/dev/docs/ros-install.html#ros-install)
      2. [apriltag-ros](https://github.com/AprilRobotics/apriltag_ros)
      3. [rosbags](https://github.com/ros/ros_comm/tree/melodic-devel)
   2. with catkin_make in catkin_ws directory
      1. [vision_to_mavros](https://github.com/thien94/vision_to_mavros)
      2. [JetsonHacksNano realsense_ros_v2.2.8](https://github.com/JetsonHacksNano/installRealSenseROS/releases/tag/vL4T32.2.1)
         1. [realsense_ros_v2.2.8 source](https://github.com/IntelRealSense/realsense-ros/tree/2.2.8)
7. Python packages
   1. ```pip install cython```
   2. ```pip install pymavros```
   3. ```pip install pyrealsense```
8. Set Auto reverse ssh tunnel on boot
   1. copy scripts/reverse_ssh_continuous.sh to /scripts/ folder
      - ```mkdir /scripts && sudo cp scripts/reverse_ssh_continuous.sh /scripts```
   2. run crontab edit with ```crontab -e``` and add following commands below
      ```
      * * * * *  /scripts/reverse_ssh_continuous.sh
      * * * * * sleep 20; /scripts/reverse_ssh_continuous.sh
      * * * * * sleep 40; /scripts/reverse_ssh_continuous.sh
      ```
## Running the code
1. git clone this repository
2. move catkin_ws, catkin_ws_build folder to your home directory. 
3. build ROS packages
   1. ```catkin_make``` with catkin_ws folder
   2. ```catkin build``` with catkin_ws_build folder
4. run the whole code with command
   - ```roslaunch drone_controller all_drone_control.launch```

## Trouble shootGuide
1. Jetson nano is not reading values properly from pixhawk through ROS.
   - Try ```rosservice call /mavros/set stream rate 0 10 1```
2. Cannot launch file at catkin_ws_build after compiling catkin_make at catkin_ws folder
   - [this reference](https://answers.ros.org/question/214923/roslaunch-cant-find-launch-files-with-catkin-build/) would be helpful
3. Realsense Camera tf error
   - It's problem with camera. you should power off and power on the camera
   - [reference link](https://discuss.ardupilot.org/t/precision-landing-with-ros-realsense-t265-camera-and-apriltag-3-part-2-2/51493/5)

## Reference

1. [lte module wiki](https://www.waveshare.com/wiki/SIM7600G-H_4G_for_Jetson_Nano)
2. [Precision Landing with ROS](https://discuss.ardupilot.org/t/precision-landing-with-ros-realsense-t265-camera-and-apriltag-3-part-2-2/51493)
3. [ROS and VIO tracking camera for non-GPS Navigation](https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html)
4. [Drone Assembly Guide video 1. Falcon shop](https://www.youtube.com/watch?v=IdpUYPuSYhE&ab_channel=KoreaFalcon)
5. [pixhawk 4 official guide](https://docs.px4.io/master/ko/getting_started/px4_basic_concepts.html)
6. [pm07 setting](http://www.holybro.com/product/pixhawk-4-power-module-pm07/) 
7. [Holybro X500 + Pixhawk4 assembly](https://docs.px4.io/master/ko/frames_multicopter/holybro_x500_pixhawk4.html)
8. [pixhawk 4 airframe](https://docs.px4.io/master/ko/airframes/airframe_reference.html)
9. [pixhawk 4 wiring](https://docs.px4.io/master/ko/assembly/quick_start_pixhawk4.html#power)
10. [xcopter esc](https://www.youtube.com/watch?v=nJnbSJwq4Uw&t=199s&ab_channel=Dronejang)
11. [reverse ssh configuration](https://system-monitoring.readthedocs.io/en/latest/ssh.html)
12. Rospy subscribe multiple topics [first_link](https://robotics.stackexchange.com/questions/6652/how-to-get-a-python-node-in-ros-subscribe-to-multiple-topics) [second_link](https://answers.ros.org/question/225008/python-class-for-subscribing-data-for-several-topics/)
13. [set waypoint to pixhawk](https://github.com/mavlink/mavros/issues/837)
14. [mavros wiki](http://wiki.ros.org/mavros#gcs_bridge)
15. [rospy tutorial](https://github.com/greattoe/ros_tutorial_kr/blob/master/ros1_tutorial/rospy/rospy_1_How2UsePythonWithCatkin_1.md)
16. [rospkg module not found](https://answers.ros.org/question/245967/importerror-no-module-named-rospkg-python3-solved/)
17. [Ardupilot Mission commands overview](https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-nav-takeoff)
18. [Ardupilot Set Flight mode](https://ardupilot.org/dev/docs/apmcopter-adding-a-new-flight-mode.html)
19. [pixhawk gps led warning](https://ardupilot.org/copter/docs/common-leds-pixhawk.html)
20. [SITL Guide](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
