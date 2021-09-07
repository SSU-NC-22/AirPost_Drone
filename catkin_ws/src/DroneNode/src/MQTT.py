#!/usr/bin/env python

from paho.mqtt import client as mqtt_client
import datetime
import time
import json
import rospy
from std_msgs.msg import String

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


	def subscribe(self, topic, handler):
		self.sub_topic = topic
		self.handler = handler
		
		def on_message(client, userdata, msg): #return msg.payload
			print(f"Received msg from `{msg.topic}` topic")
			self.handler.run(msg.payload)

		self.client.on_message = on_message
		self.client.subscribe(self.sub_topic)

class DroneNode():
	def __init__(self) -> None:
		rospy.Subscriber("mavros/global_position/global", String, self.globalValCallback)
		rospy.Subscriber("mavros/global_position/gp_vel", String, self.batteryCallback)
		rospy.Subscriber("mavros/battery", String, self.velocityCallback)

		self.lat = None
		self.lon = None
		self.alt = None
		self.battery = None
		self.velcity = None
	
	def globalValCallback(self, data):
		self.lat = None
		self.long = None
		self.alt = None

		rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	
	def batteryCallback(self, data):
		self.battery = None
	
	def velocityCallback(self, data):
		self.velcity = None

#mqtt.subscribe("command/downlink/ActuatorReq/0", handler)

if __name__=='__main__':
	rospy.init_node('DroneNode', anonymous = True)
	rate = rospy.Rate(10) #10hz
	droneNode = DroneNode()
	
	mqtt = MQTT(broker, port, client_id)
	mqtt.connect_mqtt()
	mqtt.subscribe("data/"+client_id, handler)
	
	while not rospy.is_shutdown():
		pub = rospy.Publisher('chatter',String,queue_size=10)
		droneNode.pub.publish()
		rospy.spinonce()

		msgs = {
			"node_id": client_id,
			"values": {
				droneNode.lat,
				droneNode.lon,
				droneNode.alt,
				droneNode.battery,
				droneNode.velcity
			},
			"timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
		}

		msg = json.dumps(msgs)
		mqtt.publish("data/"+client_id, msg)
		print("publish: ", type(msg), msg, "\n")
		
		rate.sleep()

	
	#mqtt.subscribe("command/downlink/ActuatorReq/0", handler)