import time
from dataclasses import dataclass, field

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from real_case_impl.common.constants import (
    CELL_SAFETY_ALARM_MQTT_TOPIC,
    CELL_SAFETY_RESET_MQTT_TOPIC,
    COMMAND_HOME,
    COMMAND_RESET,
    COMMAND_STOP,
    DEFAULT_KUKA_ROBOT_NAME,
    DEFAULT_UR_ROBOT_NAME,
    robot_adapter_cmd_topic,
    robot_cmd_mqtt_topic,
    robot_event_mqtt_topic,
    robot_state_mqtt_topic,
    robot_task_cmd_topic,
    robot_task_event_topic,
    robot_task_status_topic,
)
from real_case_impl.common.contracts import dumps, loads, task_command_payload, task_event_payload, task_status_payload


STATE_PUBLISH_PERIOD = 0.2
STATUS_TIMEOUT = 1.0


@dataclass
class AdapterContext:
    robot_name: str
    state: str = "offline"
    task: str = "none"
    ready: bool = False
    homed: bool = False
    motion: str = "stopped"
    error: bool = False
    error_message: str = ""
    emergency_active: bool = False
    task_active: bool = False
    current_job_id: int = -1
    last_event: str = ""
    last_status_time: float = 0.0
    last_state_publish_time: float = field(default_factory=time.time)


class RobotAdapterNode(Node):
    def __init__(
        self,
        node_name: str,
        default_robot_name: str,
        start_commands: dict[str, str],
    ) -> None:
        super().__init__(node_name)

        self.start_commands = start_commands
        self.supported_commands = {COMMAND_HOME, COMMAND_STOP, COMMAND_RESET, *start_commands.keys()}

        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("robot_name", default_robot_name)
        self.declare_parameter("state_publish_period", STATE_PUBLISH_PERIOD)
        self.declare_parameter("status_timeout", STATUS_TIMEOUT)

        mqtt_host = self.get_parameter("mqtt_host").get_parameter_value().string_value
        mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.state_publish_period = self.get_parameter("state_publish_period").get_parameter_value().double_value
        self.status_timeout = self.get_parameter("status_timeout").get_parameter_value().double_value

        self.ctx = AdapterContext(robot_name=robot_name)

        self.task_cmd_pub = self.create_publisher(String, robot_task_cmd_topic(robot_name), 10)
        self.task_status_sub = self.create_subscription(
            String,
            robot_task_status_topic(robot_name),
            self.task_status_callback,
            10,
        )
        self.task_event_sub = self.create_subscription(
            String,
            robot_task_event_topic(robot_name),
            self.task_event_callback,
            10,
        )
        self.local_operator_cmd_sub = self.create_subscription(
            String,
            robot_adapter_cmd_topic(robot_name),
            self.local_operator_cmd_callback,
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

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"{node_name} started")
        self.get_logger().info(f"Robot name: {robot_name}")
        self.get_logger().info(f"MQTT broker: {mqtt_host}:{mqtt_port}")

    def now(self) -> float:
        return time.time()

    def publish_mqtt_json(self, topic: str, payload: dict, qos: int = 1, retain: bool = False) -> None:
        try:
            self.mqtt_client.publish(topic, dumps(payload), qos=qos, retain=retain)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish MQTT to {topic}: {exc}")

    def publish_state(self) -> None:
        payload = task_status_payload(
            robot=self.ctx.robot_name,
            state=self.ctx.state,
            task=self.ctx.task,
            ready=self.ctx.ready,
            homed=self.ctx.homed,
            motion=self.ctx.motion,
            error=self.ctx.error,
            message=self.ctx.error_message,
            job_id=self.ctx.current_job_id,
            timestamp=self.now(),
        )
        self.publish_mqtt_json(robot_state_mqtt_topic(self.ctx.robot_name), payload, qos=1, retain=False)

    def publish_event(self, event_name: str, message: str = "", job_id: int | None = None) -> None:
        payload = task_event_payload(
            robot=self.ctx.robot_name,
            event=event_name,
            task=self.ctx.task,
            job_id=self.ctx.current_job_id if job_id is None else job_id,
            message=message,
            timestamp=self.now(),
        )
        self.publish_mqtt_json(robot_event_mqtt_topic(self.ctx.robot_name), payload, qos=1, retain=False)
        self.get_logger().info(
            f"Published MQTT event: robot={self.ctx.robot_name}, event={event_name}, "
            f"job_id={payload['job_id']}, message={message}"
        )

    def publish_local_task_command(self, command: str, job_id: int, source: str = "") -> None:
        ros_msg = String()
        ros_msg.data = dumps(task_command_payload(command, job_id, self.now(), source=source))
        self.task_cmd_pub.publish(ros_msg)
        self.get_logger().info(
            f"Published local ROS task command: command={command}, job_id={job_id}"
        )

    def command_allowed(self, command: str) -> tuple[bool, str]:
        if command not in self.supported_commands:
            return False, f"Unsupported command: {command}"

        if self.ctx.state == "offline":
            return False, "Robot is offline"

        if self.ctx.emergency_active and command != COMMAND_RESET:
            return False, "Emergency is active"

        if self.ctx.error and command not in {COMMAND_RESET, COMMAND_STOP}:
            return False, "Robot is in error state"

        if command in self.start_commands:
            if self.ctx.task_active:
                return False, "Task already active"
            if not self.ctx.ready:
                return False, "Robot is not ready"
            if self.ctx.state not in {"idle", "ready"}:
                return False, f"Robot state does not allow task start: {self.ctx.state}"

        if command == COMMAND_HOME:
            if self.ctx.task_active:
                return False, "Cannot home while task is active"
            if self.ctx.state not in {"idle", "ready"}:
                return False, f"Robot state does not allow homing: {self.ctx.state}"

        if command == COMMAND_STOP and not self.ctx.task_active:
            return False, "No active task to stop"

        return True, ""

    def accept_command(self, command: str, job_id: int) -> None:
        if command in self.start_commands:
            self.ctx.task_active = True
            self.ctx.current_job_id = job_id
            self.ctx.task = self.start_commands[command]
            self.ctx.state = "busy"
            self.ctx.motion = "running"
        elif command == COMMAND_HOME:
            self.ctx.task_active = True
            self.ctx.current_job_id = job_id
            self.ctx.task = "home"
            self.ctx.state = "busy"
            self.ctx.motion = "running"
        elif command == COMMAND_STOP:
            self.ctx.motion = "stopping"
        elif command == COMMAND_RESET:
            self.ctx.error = False
            self.ctx.error_message = ""
            self.ctx.emergency_active = False
            self.ctx.task_active = False
            self.ctx.current_job_id = -1
            self.ctx.task = "none"
            self.ctx.state = "idle" if self.ctx.ready else "initializing"
            self.ctx.motion = "stopped"

        self.publish_local_task_command(command, job_id)

        if command in self.start_commands or command == COMMAND_HOME:
            self.publish_event("TASK_ACCEPTED", job_id=job_id)
        elif command == COMMAND_STOP:
            self.publish_event("STOP_REQUESTED", job_id=job_id)
        elif command == COMMAND_RESET:
            self.publish_event("RESET_REQUESTED", job_id=job_id)

    def reject_command(self, command: str, reason: str, job_id: int) -> None:
        self.get_logger().warn(
            f"Rejected command: command={command}, job_id={job_id}, reason={reason}"
        )
        self.publish_event("COMMAND_REJECTED", message=reason, job_id=job_id)

    def task_status_callback(self, msg: String) -> None:
        try:
            payload = loads(msg.data)
        except Exception:
            self.get_logger().warn(f"Invalid JSON on {robot_task_status_topic(self.ctx.robot_name)}: {msg.data}")
            return

        self.ctx.state = str(payload.get("state", self.ctx.state))
        self.ctx.task = str(payload.get("task", self.ctx.task))
        self.ctx.ready = bool(payload.get("ready", self.ctx.ready))
        self.ctx.homed = bool(payload.get("homed", self.ctx.homed))
        self.ctx.motion = str(payload.get("motion", self.ctx.motion))
        self.ctx.error = bool(payload.get("error", self.ctx.error))
        self.ctx.error_message = str(payload.get("message", self.ctx.error_message))
        self.ctx.last_status_time = self.now()

        if self.ctx.state != "offline" and not self.ctx.task_active and not self.ctx.error and not self.ctx.emergency_active:
            if self.ctx.ready:
                self.ctx.state = "idle"

    def task_event_callback(self, msg: String) -> None:
        try:
            payload = loads(msg.data)
        except Exception:
            self.get_logger().warn(f"Invalid JSON on {robot_task_event_topic(self.ctx.robot_name)}: {msg.data}")
            return

        event_name = str(payload.get("event", ""))
        job_id = int(payload.get("job_id", self.ctx.current_job_id))
        task_name = str(payload.get("task", self.ctx.task))
        message = str(payload.get("message", ""))

        self.ctx.last_event = event_name
        self.ctx.last_status_time = self.now()

        if event_name == "TASK_STARTED":
            self.ctx.task_active = True
            self.ctx.current_job_id = job_id
            self.ctx.task = task_name
            self.ctx.state = "busy"
            self.ctx.motion = "running"
        elif event_name == "TASK_DONE":
            self.ctx.task_active = False
            self.ctx.current_job_id = -1
            self.ctx.task = "none"
            self.ctx.state = "idle"
            self.ctx.motion = "stopped"
            self.ctx.error = False
            self.ctx.error_message = ""
        elif event_name in {"TASK_FAILED", "FAULT", "PROTECTIVE_STOP"}:
            self.ctx.task_active = False
            self.ctx.state = "fault" if event_name != "PROTECTIVE_STOP" else "protective_stop"
            self.ctx.motion = "stopped"
            self.ctx.error = True
            self.ctx.error_message = message if message else event_name
        elif event_name == "EMERGENCY_STOP":
            self.ctx.task_active = False
            self.ctx.state = "emergency_stop"
            self.ctx.motion = "stopped"
            self.ctx.emergency_active = True
        elif event_name == "RESET_DONE":
            self.ctx.error = False
            self.ctx.error_message = ""
            self.ctx.task_active = False
            self.ctx.current_job_id = -1
            self.ctx.task = "none"
            self.ctx.motion = "stopped"
            self.ctx.state = "idle" if self.ctx.ready else "initializing"
        elif event_name == "HOMED":
            self.ctx.homed = True

        self.publish_event(event_name, message=message, job_id=job_id)

    def local_operator_cmd_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()
        job_id = int(self.now())
        allowed, reason = self.command_allowed(command)
        if allowed:
            self.accept_command(command, job_id)
        else:
            self.reject_command(command, reason, job_id)

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            topics = [
                (robot_cmd_mqtt_topic(self.ctx.robot_name), 1),
                (CELL_SAFETY_ALARM_MQTT_TOPIC, 1),
                (CELL_SAFETY_RESET_MQTT_TOPIC, 1),
            ]
            for topic, qos in topics:
                client.subscribe(topic, qos=qos)
                self.get_logger().info(f"Subscribed to MQTT topic: {topic}")
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, rc={rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"Disconnected from MQTT broker, rc={rc}")

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8", errors="ignore")

        try:
            payload = loads(payload_raw)
        except Exception:
            self.get_logger().warn(f"Invalid JSON on MQTT topic {topic}: {payload_raw}")
            return

        if topic == robot_cmd_mqtt_topic(self.ctx.robot_name):
            self.handle_robot_command(payload)
        elif topic == CELL_SAFETY_ALARM_MQTT_TOPIC:
            self.handle_safety_alarm(payload)
        elif topic == CELL_SAFETY_RESET_MQTT_TOPIC:
            self.handle_safety_reset(payload)

    def handle_robot_command(self, payload: dict) -> None:
        target = str(payload.get("target", ""))
        command = str(payload.get("command", "")).strip().upper()
        job_id = int(payload.get("job_id", -1))

        if target and target != self.ctx.robot_name:
            return

        allowed, reason = self.command_allowed(command)
        if allowed:
            self.accept_command(command, job_id)
        else:
            self.reject_command(command, reason, job_id)

    def handle_safety_alarm(self, payload: dict) -> None:
        active = bool(payload.get("active", False))
        alarm_type = str(payload.get("type", "UNKNOWN"))
        source = str(payload.get("source", "unknown"))

        if not active:
            return

        self.get_logger().warn(f"Safety alarm active: type={alarm_type}, source={source}")
        self.ctx.emergency_active = True
        self.ctx.task_active = False
        self.ctx.state = "emergency_stop"
        self.ctx.motion = "stopped"
        self.ctx.error = False
        self.ctx.error_message = ""

        stop_job_id = self.ctx.current_job_id if self.ctx.current_job_id >= 0 else -1
        self.publish_local_task_command(COMMAND_STOP, stop_job_id, source="safety_alarm")
        self.publish_event(
            "EMERGENCY_STOP",
            message=f"Safety alarm: {alarm_type} from {source}",
            job_id=stop_job_id,
        )

    def handle_safety_reset(self, payload: dict) -> None:
        if not bool(payload.get("reset", False)):
            return

        self.get_logger().info("Received safety reset")
        self.ctx.emergency_active = False

        if not self.ctx.error:
            self.ctx.state = "idle" if self.ctx.ready else "initializing"
            self.ctx.motion = "stopped"

        self.publish_event("SAFETY_RESET")

    def timer_callback(self) -> None:
        now = self.now()

        if self.ctx.last_status_time > 0.0:
            dt = now - self.ctx.last_status_time
            if dt > self.status_timeout:
                if self.ctx.state != "offline":
                    self.get_logger().warn(
                        f"No recent task status from local executor for {dt:.2f}s. Marking offline."
                    )
                self.ctx.state = "offline"
                self.ctx.ready = False
                self.ctx.motion = "stopped"
                self.ctx.task_active = False

        if (now - self.ctx.last_state_publish_time) >= self.state_publish_period:
            self.publish_state()
            self.ctx.last_state_publish_time = now


class URAdapterNode(RobotAdapterNode):
    def __init__(self) -> None:
        super().__init__(
            node_name="ur_adapter_node",
            default_robot_name=DEFAULT_UR_ROBOT_NAME,
            start_commands={"START_PICK_AND_PLACE_TO_HANDOVER": "pick_and_place_to_handover"},
        )


class KukaAdapterNode(RobotAdapterNode):
    def __init__(self) -> None:
        super().__init__(
            node_name="kuka_adapter_node",
            default_robot_name=DEFAULT_KUKA_ROBOT_NAME,
            start_commands={"START_PICK_AND_SORT": "pick_and_sort"},
        )


def run_node(node_cls, interrupted_message: str, args=None) -> None:
    rclpy.init(args=args)
    node = node_cls()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(interrupted_message)
    finally:
        try:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()

