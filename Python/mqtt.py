import tkinter as tk
import paho.mqtt.client as mqtt

broker_ip = "192.168.1.95"
broker_port = 1883
publish_topic = "/ros_mqtt"
subscribe_topic = "/agv_feedback"
connected = False
message_received = ""

def on_connect(client, userdata, flags, rc):
    global connected
    connected = True
    client.subscribe(subscribe_topic)

def on_message(client, userdata, msg):
    global message_received
    message_received = msg.payload.decode()

def connect():
    mqtt_client.connect(broker_ip, broker_port, 120)
    mqtt_client.loop_start()

def disconnect():
    mqtt_client.disconnect()
    global connected
    connected = False
    update_connection_status("Disconnected from MQTT Broker.")
    home_button.config(state=tk.DISABLED)
    a_button.config(state=tk.DISABLED)
    b_button.config(state=tk.DISABLED)
    connect_button.config(text="Connect MQTT", command=toggle_connection)

def toggle_connection():
    global connected
    if not connected:
        try:
            connect()
            update_connection_status("Connected to MQTT Broker.")
            home_button.config(state=tk.NORMAL)
            a_button.config(state=tk.NORMAL)
            b_button.config(state=tk.NORMAL)
            connect_button.config(text="Disconnect MQTT", command=disconnect)
            subscribe_status()
        except Exception as e:
            update_connection_status(f"Error: {e}")
    else:
        disconnect()

def publish_message(message):
    if not connected:
        raise Exception("Not connected to MQTT Broker.")
    mqtt_client.publish(publish_topic, message)

def publish_home():
    publish_message("home")

def publish_a():
    publish_message("A")

def publish_b():
    publish_message("B")

def subscribe_status():
    root.after(1000, update_status_label)

def update_status_label():
    global message_received
    status_label.config(text=f"AGV Status: {message_received}")
    root.after(1000, update_status_label)

def update_connection_status(message):
    connection_status_label.config(text=f"Connection Status: {message}")

def update_publish_status(message):
    publish_status_label.config(text=f"Publish Status: {message}")

# MQTT settings
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message

# GUI components
root = tk.Tk()
root.title("MQTT Publisher")
root.geometry("300x300")  # Adjusted window size

connect_button = tk.Button(root, text="Connect MQTT", command=toggle_connection)
connect_button.pack(pady=10)

home_button = tk.Button(root, text="Position: Home", command=publish_home, state=tk.DISABLED)
home_button.pack(pady=10)

a_button = tk.Button(root, text="Position: A", command=publish_a, state=tk.DISABLED)
a_button.pack(pady=10)

b_button = tk.Button(root, text="Position: B", command=publish_b, state=tk.DISABLED)
b_button.pack(pady=10)

connection_status_label = tk.Label(root, text="Connection Status: Not Connected", anchor="w", justify="left")
connection_status_label.pack(pady=10)

publish_status_label = tk.Label(root, text="Publish Status: ", anchor="w", justify="left")
publish_status_label.pack(pady=5)

status_label = tk.Label(root, text="Status: ", anchor="w", justify="left")
status_label.pack(pady=5)

root.mainloop()
