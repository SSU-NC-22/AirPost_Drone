from MQTT_Connection.MQTT_Connection import MQTT
from Actuator.Handler import Handler
import datetime
import time
import json

broker = '58.230.119.87'
port = 9708
client_id = 'DRO00000'

handler = Handler()
mqtt = MQTT(broker, port, client_id)
mqtt.connect_mqtt()
mqtt.subscribe("data/"+client_id, handler)
#mqtt.subscribe("command/downlink/ActuatorReq/0", handler)


while True:	
	msgs = {
		"drone_id": client_id,
		"values": {
			"lat": None,
			"lon": None,
			"alt": None,
			"motor1" : None,
			"motor2" : None,
			"motor3" : None,
			"motor4" : None,

			"battery" : None,
			"velocity" : None,
			"status" : None,
		},
		"timestamp": str(datetime.datetime.now())
	}
	
	msg = json.dumps(msgs)

	mqtt.publish("data/"+client_id, msg)
	print("publish: ", type(msg), msg, "\n")

	time.sleep(1)