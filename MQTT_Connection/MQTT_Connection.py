from paho.mqtt import client as mqtt_client

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