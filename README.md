# AirPost_Drone

Code and instruction for configuring drone.

[TOC]

## H/W Drone Parts



## H/W Companion Computer Settings

1. Jetson nano
2. Simcom 7600G lte module
3. Intel realsense t265 camera

## S/W Settings

1. Jetpack 4.2
2. opencv 3.2
3. librealsense
4. ROS melodic
   1. mavros
   2. apriltag-ros
   3. Realsense_ros
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
8. Auto reverse ssh tunnel on boot
   ```
   * * * * *  /scripts/reverse_ssh_continuous.sh
   * * * * * sleep 20; /scripts/reverse_ssh_continuous.sh
   * * * * * sleep 40; /scripts/reverse_ssh_continuous.sh
   ```
## Guide

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

