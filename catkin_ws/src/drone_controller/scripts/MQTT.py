#!/usr/bin/env python3

from paho import mqtt
from paho.mqtt import client as mqtt_client
from datetime import datetime
import json
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from DroneController import DroneController
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

    def publish(self, topic, msg):  # input msg type : string
        self.pub_topic = topic
        result = self.client.publish(self.pub_topic, msg)

        # result: [0, 1]
        status = result[0]
        if status == 0:
            print("Send msg to topic", self.pub_topic)
        else:
            print("Failed to send message to topic", self.pub_topic)

    def subscribe(self, topic):
        self.sub_topic = topic
        self.client.on_message = self.mqtt_on_message
        self.client.subscribe(self.sub_topic)

    def mqtt_on_message(self, client, userdata, msg):  # save waypoint to pixhawk
        print("Got Actutator message from server")
        # 1. get waypoint, send to pixhawk
        msg = msg.payload.decode()
        msg = json.loads(msg)

        endidx = len(msg['values'])
        tagidx = msg['tagidx']
        wpl = msg['values']

        # dronecontroller will take over the mssion
        dronecontroller.mqtt_call_back(wpl, tagidx, endidx)
        


class SensorSub():
    def __init__(self):
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.globalValCallback)
        rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, self.velocityCallback)
        rospy.Subscriber("/mavros/battery", BatteryState, self.batteryCallback)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.rel_altCallback )

        self.lat = None
        self.long = None
        self.alt = None
        self.batteryPer = None
        self.batteryVol = None
        self.velcity = None

    def globalValCallback(self, data):
        self.lat = data.latitude
        self.long = data.longitude
        #self.alt = data.altitude

    def velocityCallback(self, data):
        self.velcity = (data.twist.linear.x ** 2 + data.twist.linear.y ** 2 + data.twist.linear.z ** 2) ** 1/2

    def batteryCallback(self, data):
        self.batteryPer = data.percentage * 100
        self.batteryVol = data.voltage

    def rel_altCallback(self, data):
        self.alt = data.data

if __name__ == "__main__":
    # ros, ros subscribe setting
    rospy.init_node('mqtt', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    sensorSub = SensorSub()

    # drone_controller init
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
                sensorSub.long,
                sensorSub.alt,
                sensorSub.velcity,
                sensorSub.batteryPer
            ],
            "status": dronecontroller.status,
            "timestamp": datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }

        msg = json.dumps(msgs)
        mqtt.publish("data/"+client_id, msg)
        print("publish: ", type(msg), msg, "\n")

        rate.sleep()
