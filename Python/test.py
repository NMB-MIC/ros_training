import tkinter as tk
import paho.mqtt.client as mqtt

broker_ip = "192.168.1.95"
broker_port = 1883
publish_topic = "/ros_mqtt"
subscribe_topic = "/agv_feedback"
connected = False
message_received = ""
global move_base,mqtt_client
mqtt_client = mqtt.Client()

mqtt_client.connect(broker_ip, broker_port)
print("connected")
# def connect():
#     mqtt_client.connect(broker_ip, broker_port, 120)
#     mqtt_client.loop_start()

# connect()