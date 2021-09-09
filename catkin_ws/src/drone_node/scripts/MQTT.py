#!/usr/bin/env python3
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
		
		def on_message(client, userdata, msg): #return msg.payload
			print(f"Received msg from `{msg.topic}` topic")

		self.client.on_message = on_message
		self.client.subscribe(self.sub_topic)

class DroneNode():
	def __init__(self) -> None:
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.globalValCallback)
		rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.velocityCallback)
		rospy.Subscriber("/mavros/battery", BatteryState, self.batteryCallback)

		self.lat = None
		self.lon = None
		self.alt = None
		self.battery = None
		self.velcity = None
	
	def globalValCallback(self, data):
		self.lat = data.latitude
		self.long = data.longitude
		self.alt = data.altitude
	
	def velocityCallback(self, data):
		self.velcity = data.twist.linear.x
		
	def batteryCallback(self, data):
		self.battery = data.percentage

#mqtt.subscribe("command/downlink/ActuatorReq/0", handler)

if __name__=='__main__':
	rospy.init_node('DroneNode', anonymous = True)
	rate = rospy.Rate(1) #10hz
	droneNode = DroneNode()
	
	mqtt = MQTT(broker, port, client_id)
	mqtt.connect_mqtt()
	mqtt.subscribe("data/"+client_id)
	
	while not rospy.is_shutdown():
		#pub = rospy.Publisher('chatter',String,queue_size=10)
		#droneNode.pub.publish()
		rospy.spin()

		msgs = {
			"node_id": client_id,
			"values": [
				droneNode.lat,
				droneNode.lon,
				droneNode.alt,
				droneNode.battery,
				droneNode.velcity
			],
			"timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		}

		msg = json.dumps(msgs)
		mqtt.publish("data/"+client_id, msg)
		print("publish: ", type(msg), msg, "\n")
		
		rate.sleep()

	
	#mqtt.subscribe("command/downlink/ActuatorReq/0", handler)