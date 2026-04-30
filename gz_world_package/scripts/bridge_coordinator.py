import paho.mqtt.client as mqtt
import json

BROKER = "localhost"

ur_ready = False
kuka_ready = False

def on_message(client, userdata, msg):
    global ur_ready, kuka_ready

    topic = msg.topic
    data = json.loads(msg.payload.decode())

    if topic == "/robot/ur3e/state":
        ur_ready = (data["state"] == "idle")

    if topic == "/robot/kuka/state":
        kuka_ready = (data["state"] == "idle")

    if ur_ready and kuka_ready:
        print("Both robots ready → start task")
        client.publish("/cell/commands", "START_SORTING")


client = mqtt.Client()
client.connect(BROKER, 1883, 60)

client.subscribe("/robot/ur3e/state")
client.subscribe("/robot/kuka/state")

client.on_message = on_message

client.loop_forever()