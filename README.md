# AirPost_Drone

Code and instruction for configuring drone.

## H/W Settings

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
   4. [install jtop][https://github.com/rbonghi/jetson_stats]
   5. [vim setting][https://hyoje420.tistory.com/51]
3. [opencv 3.2][https://github.com/jetsonhacks/buildOpenCVXavier]
4. librealsense
5. [ROS melodic][http://wiki.ros.org/melodic/Installation/Ubuntu]
   1. [mavros][https://ardupilot.org/dev/docs/ros-install.html#ros-install]
   2. [Vision_to_mavros][https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html]
   3. apriltag, apriltag-ros
   4. [Realsense_ros][https://github.com/jetsonhacks/installRealSenseSDK]
   5. [rosbags][https://github.com/ros/ros_comm]
6. Python packages
   1. ```pip install cython```
   2. ```pip install pymavros```
   3. ```pip install pyrealsense```

## Reference

1. [lte module wiki][https://www.waveshare.com/wiki/SIM7600G-H_4G_for_Jetson_Nano]
2. Precision landing through apriltag
3. [ROS and VIO tracking camera for non-GPS Navigation][https://ardupilot.org/dev/docs/ros-vio-tracking-camera.html]

[https://github.com/rbonghi/jetson_stats ]:

[https://github.com/rbonghi/jetson_stats ]:

[https://github.com/rbonghi/jetson_stats ]:

[https://hyoje420.tistory.com/51]:

[https://www.github.com/rbonghi/jetson_stats ]: 

[https://github.com/rbonghi/jetson_stats]: