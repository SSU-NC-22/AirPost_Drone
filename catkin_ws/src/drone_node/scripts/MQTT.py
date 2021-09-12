#!/usr/bin/env python3

from AirPost_Drone.catkin_ws.src.drone_node.scripts.DroneController import DroneController
from paho import mqtt
from paho.mqtt import client as mqtt_client
from datetime import datetime
import time
import json
 import rospy
 from std_msgs.msg import String
 from sensor_msgs.msg import NavSatFix
 from sensor_msgs.msg import BatteryState
 from geometry_msgs.msg import TwistStamped
 from mavros_msgs.msg import *
 from mavros_msgs.srv import *
from DroneController import DroneController
from DCmotor import DCMotor

broker = '58.230.119.87'
port = 9708
client_id = 'DRO0'

class MQTT():
	def __init__(self, broker, port, client_id):
		self.broker = broker
		self.port = port
		self.client_id = client_id
		self.client = mqtt_client.Client(self.client_id)
	
	def connect_mqtt(self):
		def on_connect(client, userdata, flags, rc):
			if rc == 0:
				print("Connected to MQTT Broker!")
			else:
				print("Failed to connect, return code %d\n", rc)

		self.client.on_connect = on_connect
		self.client.connect(self.broker, self.port)
		self.client.loop_start()

	def publish(self, topic, msg): #input msg type : string
		self.pub_topic = topic
		result = self.client.publish(self.pub_topic, msg)

		# result: [0, 1]
		status = result[0]
		if status == 0:
			print(f"Send msg to topic `{self.pub_topic}`")
		else:
			print(f"Failed to send message to topic {self.pub_topic}")


	def subscribe(self, topic):
		self.sub_topic = topic
		self.client.on_message = mqtt_callback_function
		self.client.subscribe(self.sub_topic)


def mqtt_callback_function(client, userdata, msg, ):
	# 1. get waypoint, send to pixhawk
	msg = msg.payload
	msg = json.loads(msg)
	dronecontroller.create_waypoint(msg['values']['wp0'])
	# 2. , 3. arm the vehicle and take off 
	dronecontroller.takeoff(1)
	# 4. , 5. set mode to auto. fly to tag
	# 6. 7. hold still above tag drop the object
	while mqtt_ros_bridge.reachedWaypoint != len(msg['values']['wp0']):
		pass
	# 8. , 9. set new waypoint
	dronecontroller.create_waypoint(msg['values']['wp1'])
	# 10. when reached. land.
	dronecontroller.land()



class MQTT_ROS_bridge():
	def __init__(self) -> None:
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.globalValCallback)
		rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.velocityCallback)
		rospy.Subscriber("/mavros/battery", BatteryState, self.batteryCallback)
		rospy.Subscriber("/mission/reached", WaypointReached, waypointCallback)

		self.lat = None
		self.lon = None
		self.alt = None
		self.battery = None
		self.velcity = None
		self.reachedWaypoint = None
	
	def globalValCallback(self, data):
		self.lat = data.latitude
		self.long = data.longitude
		self.alt = data.altitude
	
	def velocityCallback(self, data):
		self.velcity = (data.twist.linear.x ** 2 + data.twist.linear.y ** 2 + data.twist.linear.z ** 2 ) ** 1/2
		
	def batteryCallback(self, data):
		self.battery = data.percentage

	def waypointCallback(self. data):
		self.reachedWaypoint = data.wp_seq

if __name__=='__main__':	
	# ros, ros subscribe setting
	rospy.init_node('mqtt', anonymous = True)
	rate = rospy.Rate(1) #10hz
	mqtt_ros_bridge = MQTT_ROS_bridge()
	
	# mqtt, mqtt sub, pub setting
	mqtt = MQTT(broker, port, client_id)
	mqtt.connect_mqtt()
	mqtt.subscribe("command/downlink/ActuatorReq/"+client_id)

	#drone_controller init
	dronecontroller = DroneController()

	while not rospy.is_shutdown():
		# sensor value sending code
		msgs = {
			"node_id": client_id,
			"values": [
				mqtt_ros_bridge.lat,
				mqtt_ros_bridge.lon,
				mqtt_ros_bridge.alt,
				mqtt_ros_bridge.velcity,
				mqtt_ros_bridge.battery
			],
			"timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		}

		msg = json.dumps(msgs)
		mqtt.publish("data/"+client_id, msg)
		print("publish: ", type(msg), msg, "\n")
		
		rate.sleep()