#!/usr/bin/env python3

from paho import mqtt
from paho.mqtt import client as mqtt_client
from datetime import datetime
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
import time

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
			print("Send msg to topic",self.pub_topic)
		else:
			print("Failed to send message to topic",self.pub_topic)


	def subscribe(self, topic):
		self.sub_topic = topic
		self.client.on_message = self.mqtt_on_message
		self.client.subscribe(self.sub_topic)
	
	def mqtt_on_message(self, client, userdata, msg): # save waypoint to pixhawk
		print("got it\n\n")
		# 1. get waypoint, send to pixhawk
		global tagidx
		global endidx
		msg = json.loads(msg.payload.decode())
		endidx = len(msg['values']) -1
		tagidx = msg['tagidx']
		dronecontroller.create_waypoint(msg['values'], tagidx)
		# 2. , 3. arm the vehicle and take off 
		dronecontroller.takeoff(1)
		rospy.sleep(3)
		# 4., 5.,8., 9.,10. set mode to auto. will fly to the tag, drop the package, and do remain missions
		dronecontroller.setmode(3)
		# continued to SensorSub.waypointCallback
		#dronecontroller.disarm()

class SensorSub():
	def __init__(self):
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.globalValCallback)
		rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.velocityCallback)
		rospy.Subscriber("/mavros/battery", BatteryState, self.batteryCallback)
		rospy.Subscriber("/mission/reached", WaypointReached, self.waypointCallback)

		self.lat = None
		self.lon = None
		self.alt = None
		self.battery = None
		self.velcity = None
		self.reachedWaypoint = None
		self.status = None
		self.dcmotor = DCMotor()
	
	def globalValCallback(self, data):
		self.lat = data.latitude
		self.long = data.longitude
		self.alt = data.altitude
	
	def velocityCallback(self, data):
		self.velcity = (data.twist.linear.x ** 2 + data.twist.linear.y ** 2 + data.twist.linear.z ** 2 ) ** 1/2
		
	def batteryCallback(self, data):
		self.battery = data.percentage

	def waypointCallback(self, data):
		self.reachedWaypoint = data.wp_seq
		# 6. 7. hold still above tag drop the object
		if self.reachedWaypoint == tagidx+1:
			self.dcmotor.counterclockwise()
			time.sleep(0.2)
			self.dcmotor.stop()
			time.sleep(0.2)
			self.dcmotor.clockwise()
			time.sleep(1)
			self.dcmotor.stop()
			time.sleep(0.2)
	
	def statusCallback(self, data):
		if self.reachedWaypoint ==  endidx:
			self.status = "done"
		else : 
			self.status = "on going"
			

if __name__ == "__main__" : 
	# ros, ros subscribe setting
	rospy.init_node('mqtt', anonymous = True)
	rate = rospy.Rate(1) #10hz
	sensorSub = SensorSub()

	#drone_controller init
	dronecontroller = DroneController()

	# mqtt, mqtt sub, pub setting
	mqtt = MQTT(broker, port, client_id)
	mqtt.connect_mqtt()
	mqtt.subscribe("command/downlink/ActuatorReq/"+client_id)


	while not rospy.is_shutdown():
		# sensor value sending code
		msgs = {
			"node_id": client_id,
			"values": [
				sensorSub.lat,
				sensorSub.lon,
				sensorSub.alt,
				sensorSub.velcity,
				sensorSub.battery
			],
			"status" : sensorSub.status,
			"timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		}

		msg = json.dumps(msgs)
		mqtt.publish("data/"+client_id, msg)
		print("publish: ", type(msg), msg, "\n")
		
		rate.sleep()