import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import paho.mqtt.client as mqtt

BROKER = "localhost"

class ROS2MQTTBridge(Node):
    def __init__(self):
        super().__init__('ros2_mqtt_bridge')

        # MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(BROKER, 1883, 60)

        # ROS2 subscriber (robot state)
        self.subscription = self.create_subscription(
            String,
            '/ur3e/state',
            self.ros_callback,
            10
        )

        # MQTT subscriber (commands)
        self.mqtt_client.subscribe("/cell/commands")
        self.mqtt_client.on_message = self.mqtt_callback
        self.mqtt_client.loop_start()

        self.publisher = self.create_publisher(String, '/ur3e/commands', 10)

    def ros_callback(self, msg):
        data = {
            "robot": "ur3e",
            "state": msg.data,
            "timestamp": time.time(),
            "error": False,
            "message": ""
        }
        self.mqtt_client.publish("/robot/ur3e/state", json.dumps(data))

    def mqtt_callback(self, client, userdata, msg):
        command = msg.payload.decode()
        ros_msg = String()
        ros_msg.data = command
        self.publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROS2MQTTBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()