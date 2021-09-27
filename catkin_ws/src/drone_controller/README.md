# drone_controller
drone_controller is a ROS package which controls drone through mavros.
## Prerequisite S/W
1. ROS package MAVROS
2. Python Package Jetson.GPIO, paho.mqtt, 

## Prerequisite H/W
1. Pixhawk 4 connected with usb
2. Dcmotor connected with GPIO

## Interface
1. clone repository to your catkin_ws/src folder
2. make scripts/MQTT.py executable with command ```chmod +x MQTT.py```
3. build node with catkin_make command
4. run with command ```rosurn drone_controller MQTT.py```
