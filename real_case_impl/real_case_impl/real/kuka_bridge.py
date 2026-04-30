import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from real_case_impl.common.constants import (
    COMMAND_STOP,
    DEFAULT_KUKA_ROBOT_NAME,
    KUKA_BACKEND_CMD_MQTT_TOPIC,
    KUKA_BACKEND_EVENT_MQTT_TOPIC,
    KUKA_BACKEND_STATUS_MQTT_TOPIC,
    robot_task_cmd_topic,
    robot_task_event_topic,
    robot_task_status_topic,
)
from real_case_impl.common.contracts import loads


class KukaRealBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("kuka_real_bridge_node")

        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("robot_name", DEFAULT_KUKA_ROBOT_NAME)
        self.declare_parameter("backend_cmd_topic", KUKA_BACKEND_CMD_MQTT_TOPIC)
        self.declare_parameter("backend_status_topic", KUKA_BACKEND_STATUS_MQTT_TOPIC)
        self.declare_parameter("backend_event_topic", KUKA_BACKEND_EVENT_MQTT_TOPIC)

        mqtt_host = self.get_parameter("mqtt_host").get_parameter_value().string_value
        mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.robot_name = robot_name
        self.backend_cmd_topic = self.get_parameter("backend_cmd_topic").get_parameter_value().string_value
        self.backend_status_topic = self.get_parameter("backend_status_topic").get_parameter_value().string_value
        self.backend_event_topic = self.get_parameter("backend_event_topic").get_parameter_value().string_value

        self.task_status_pub = self.create_publisher(String, robot_task_status_topic(robot_name), 10)
        self.task_event_pub = self.create_publisher(String, robot_task_event_topic(robot_name), 10)
        self.task_cmd_sub = self.create_subscription(
            String,
            robot_task_cmd_topic(robot_name),
            self.task_cmd_callback,
            10,
        )

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        self.mqtt_client.on_message = self.on_mqtt_message

        try:
            self.mqtt_client.connect(mqtt_host, int(mqtt_port), 60)
            self.mqtt_client.loop_start()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to connect to MQTT broker at {mqtt_host}:{mqtt_port}: {exc}"
            )

    def task_cmd_callback(self, msg: String) -> None:
        try:
            loads(msg.data)
        except Exception:
            self.get_logger().warn(f"Invalid JSON on KUKA task_cmd: {msg.data}")
            return

        try:
            self.mqtt_client.publish(self.backend_cmd_topic, msg.data, qos=1, retain=False)
        except Exception as exc:
            self.get_logger().error(f"Failed to forward command to KUKA backend: {exc}")
            if COMMAND_STOP not in msg.data:
                self._publish_bridge_event(
                    "TASK_FAILED",
                    message=f"KUKA backend command forwarding failed: {exc}",
                )

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.backend_status_topic, qos=1)
            client.subscribe(self.backend_event_topic, qos=1)
            self.get_logger().info(
                f"Connected to backend MQTT topics: {self.backend_status_topic}, {self.backend_event_topic}"
            )
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, rc={rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"Disconnected from MQTT broker, rc={rc}")

    def on_mqtt_message(self, client, userdata, msg):
        payload_raw = msg.payload.decode("utf-8", errors="ignore")
        try:
            loads(payload_raw)
        except Exception:
            self.get_logger().warn(f"Invalid JSON on MQTT topic {msg.topic}: {payload_raw}")
            return

        ros_msg = String()
        ros_msg.data = payload_raw

        if msg.topic == self.backend_status_topic:
            self.task_status_pub.publish(ros_msg)
        elif msg.topic == self.backend_event_topic:
            self.task_event_pub.publish(ros_msg)

    def _publish_bridge_event(self, event: str, message: str = "") -> None:
        ros_msg = String()
        ros_msg.data = json.dumps(
            {
                "robot": self.robot_name,
                "event": event,
                "task": "none",
                "job_id": -1,
                "message": message,
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
            }
        )
        self.task_event_pub.publish(ros_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KukaRealBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KUKA real bridge interrupted by user")
    finally:
        try:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
