import json
import time
from dataclasses import dataclass, field
from typing import Optional

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from real_case_impl.common.constants import (
    CELL_OPERATOR_CMD_MQTT_TOPIC,
    CELL_OPERATOR_CMD_TOPIC,
    CELL_SAFETY_ALARM_MQTT_TOPIC,
    CELL_SAFETY_RESET_MQTT_TOPIC,
    CELL_SUPERVISOR_STATE_MQTT_TOPIC,
    CELL_SUPERVISOR_STATUS_TOPIC,
    COMMAND_HOME,
    COMMAND_KUKA_TASK,
    COMMAND_RESET,
    COMMAND_STOP,
    COMMAND_UR_TASK,
    DEFAULT_KUKA_ROBOT_NAME,
    DEFAULT_UR_ROBOT_NAME,
    robot_event_mqtt_topic,
    robot_state_mqtt_topic,
)


HEARTBEAT_TIMEOUT = 1.0
UR_TASK_TIMEOUT = 30.0
KUKA_TASK_TIMEOUT = 30.0
READY_TIMEOUT = 10.0
INIT_TIMEOUT = 15.0
HOMING_TIMEOUT = 30.0
RESET_TIMEOUT = 10.0
STOP_TIMEOUT = 10.0


@dataclass
class RobotStatus:
    name: str
    state: str = "offline"
    task: str = "none"
    error: bool = False
    ready: bool = False
    homed: bool = False
    motion: str = "stopped"
    heartbeat_time: float = 0.0
    last_event: str = ""
    last_job_id: int = -1
    error_message: str = ""


@dataclass
class SupervisorContext:
    current_state: str = "INIT"
    previous_state: str = "INIT"
    state_enter_time: float = field(default_factory=time.time)
    cycle_id: int = 0
    auto_mode: bool = True
    emergency_active: bool = False
    last_alarm_source: str = ""
    active_reset_job_id: int = -1
    active_home_job_id: int = -1
    ur: RobotStatus = field(default_factory=lambda: RobotStatus(name=DEFAULT_UR_ROBOT_NAME))
    kuka: RobotStatus = field(default_factory=lambda: RobotStatus(name=DEFAULT_KUKA_ROBOT_NAME))


class SupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__("cell_supervisor_node")

        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("loop_period_sec", 0.1)
        self.declare_parameter("auto_mode", True)
        self.declare_parameter("ur_robot_name", DEFAULT_UR_ROBOT_NAME)
        self.declare_parameter("kuka_robot_name", DEFAULT_KUKA_ROBOT_NAME)

        mqtt_host = self.get_parameter("mqtt_host").get_parameter_value().string_value
        mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        loop_period_sec = self.get_parameter("loop_period_sec").get_parameter_value().double_value
        auto_mode = self.get_parameter("auto_mode").get_parameter_value().bool_value
        ur_robot_name = self.get_parameter("ur_robot_name").get_parameter_value().string_value
        kuka_robot_name = self.get_parameter("kuka_robot_name").get_parameter_value().string_value

        self.ctx = SupervisorContext(
            auto_mode=auto_mode,
            ur=RobotStatus(name=ur_robot_name),
            kuka=RobotStatus(name=kuka_robot_name),
        )

        self.start_requested = False
        self.pause_requested = False
        self.resume_requested = False
        self.reset_requested = False
        self.stop_requested = False
        self.last_state_publish_time = 0.0
        self.state_publish_period = 0.2
        self.next_job_id = 0

        self.operator_cmd_sub = self.create_subscription(
            String,
            CELL_OPERATOR_CMD_TOPIC,
            self.operator_cmd_callback,
            10,
        )
        self.supervisor_status_pub = self.create_publisher(String, CELL_SUPERVISOR_STATUS_TOPIC, 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        try:
            self.mqtt_client.connect(mqtt_host, int(mqtt_port), 60)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to connect to MQTT broker at {mqtt_host}:{mqtt_port}: {exc}"
            )

        self.mqtt_client.loop_start()
        self.timer = self.create_timer(loop_period_sec, self.timer_callback)

    def now(self) -> float:
        return time.time()

    def time_in_state(self) -> float:
        return self.now() - self.ctx.state_enter_time

    def allocate_job_id(self) -> int:
        self.next_job_id += 1
        return self.next_job_id

    def transition(self, new_state: str) -> None:
        if self.ctx.current_state == new_state:
            return
        self.get_logger().info(f"FSM transition: {self.ctx.current_state} -> {new_state}")
        self.ctx.previous_state = self.ctx.current_state
        self.ctx.current_state = new_state
        self.ctx.state_enter_time = self.now()

    def is_alive(self, robot: RobotStatus) -> bool:
        return (self.now() - robot.heartbeat_time) <= HEARTBEAT_TIMEOUT

    def both_alive(self) -> bool:
        return self.is_alive(self.ctx.ur) and self.is_alive(self.ctx.kuka)

    def both_ready(self) -> bool:
        return (
            self.ctx.ur.ready
            and self.ctx.kuka.ready
            and self.ctx.ur.state == "idle"
            and self.ctx.kuka.state == "idle"
            and self.ctx.ur.homed
            and self.ctx.kuka.homed
            and not self.ctx.ur.error
            and not self.ctx.kuka.error
        )

    def robot_by_name(self, robot_name: str) -> Optional[RobotStatus]:
        if robot_name == self.ctx.ur.name:
            return self.ctx.ur
        if robot_name == self.ctx.kuka.name:
            return self.ctx.kuka
        return None

    def publish_mqtt_json(self, topic: str, payload: dict, qos: int = 1, retain: bool = False) -> None:
        try:
            self.mqtt_client.publish(topic, json.dumps(payload), qos=qos, retain=retain)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish MQTT on {topic}: {exc}")

    def publish_command(self, target: str, command: str, job_id: int) -> None:
        topic = f"robot/{target}/cmd"
        payload = {
            "target": target,
            "command": command,
            "job_id": job_id,
            "timestamp": self.now(),
        }
        self.publish_mqtt_json(topic, payload, qos=1, retain=False)
        self.get_logger().info(f"Published command to {target}: {command}, job_id={job_id}")

    def publish_stop_if_active(self, robot: RobotStatus) -> bool:
        if robot.state not in {"busy", "emergency_stop"} and robot.task == "none":
            return False
        job_id = robot.last_job_id if robot.last_job_id >= 0 else self.ctx.cycle_id
        self.publish_command(robot.name, COMMAND_STOP, job_id)
        return True

    def publish_supervisor_state(self) -> None:
        payload = {
            "state": self.ctx.current_state,
            "previous_state": self.ctx.previous_state,
            "cycle_id": self.ctx.cycle_id,
            "auto_mode": self.ctx.auto_mode,
            "emergency_active": self.ctx.emergency_active,
            "alarm_source": self.ctx.last_alarm_source,
            "robots": {
                self.ctx.ur.name: {
                    "state": self.ctx.ur.state,
                    "task": self.ctx.ur.task,
                    "ready": self.ctx.ur.ready,
                    "homed": self.ctx.ur.homed,
                    "error": self.ctx.ur.error,
                    "alive": self.is_alive(self.ctx.ur),
                    "last_event": self.ctx.ur.last_event,
                    "last_job_id": self.ctx.ur.last_job_id,
                },
                self.ctx.kuka.name: {
                    "state": self.ctx.kuka.state,
                    "task": self.ctx.kuka.task,
                    "ready": self.ctx.kuka.ready,
                    "homed": self.ctx.kuka.homed,
                    "error": self.ctx.kuka.error,
                    "alive": self.is_alive(self.ctx.kuka),
                    "last_event": self.ctx.kuka.last_event,
                    "last_job_id": self.ctx.kuka.last_job_id,
                },
            },
            "timestamp": self.now(),
        }

        self.publish_mqtt_json(CELL_SUPERVISOR_STATE_MQTT_TOPIC, payload, qos=1, retain=False)
        ros_msg = String()
        ros_msg.data = json.dumps(payload)
        self.supervisor_status_pub.publish(ros_msg)

    def latch_error(self, message: str) -> None:
        self.get_logger().error(message)
        self.stop_requested = False
        self.publish_stop_if_active(self.ctx.ur)
        self.publish_stop_if_active(self.ctx.kuka)
        self.transition("ERROR")

    def latch_emergency(self, message: str, source: str = "") -> None:
        self.get_logger().error(f"EMERGENCY: {message}")
        self.stop_requested = False
        self.ctx.emergency_active = True
        self.ctx.last_alarm_source = source
        self.publish_stop_if_active(self.ctx.ur)
        self.publish_stop_if_active(self.ctx.kuka)
        self.transition("EMERGENCY_STOP")

    def clear_robot_events(self) -> None:
        self.ctx.ur.last_event = ""
        self.ctx.kuka.last_event = ""
        self.ctx.ur.last_job_id = -1
        self.ctx.kuka.last_job_id = -1

    def robot_motion_cleared(self, robot: RobotStatus) -> bool:
        return robot.state != "busy" and robot.motion not in {"running", "stopping"}

    def begin_operator_stop(self) -> None:
        self.start_requested = False
        self.pause_requested = False
        self.resume_requested = False

        ur_stop_sent = self.publish_stop_if_active(self.ctx.ur)
        kuka_stop_sent = self.publish_stop_if_active(self.ctx.kuka)
        self.stop_requested = False

        if ur_stop_sent or kuka_stop_sent:
            self.transition("STOPPING")
        else:
            self.transition("STOPPED")

    def begin_home_sequence(self, next_state: str) -> None:
        self.ctx.active_home_job_id = self.allocate_job_id()
        self.ctx.ur.homed = False
        self.ctx.kuka.homed = False
        self.clear_robot_events()
        self.publish_command(self.ctx.ur.name, COMMAND_HOME, self.ctx.active_home_job_id)
        self.publish_command(self.ctx.kuka.name, COMMAND_HOME, self.ctx.active_home_job_id)
        self.transition(next_state)

    def begin_reset_sequence(self) -> None:
        self.stop_requested = False
        self.ctx.active_reset_job_id = self.allocate_job_id()
        self.clear_robot_events()
        self.publish_command(self.ctx.ur.name, COMMAND_RESET, self.ctx.active_reset_job_id)
        self.publish_command(self.ctx.kuka.name, COMMAND_RESET, self.ctx.active_reset_job_id)
        self.transition("WAIT_RESET_DONE")

    def home_sequence_done(self) -> bool:
        return (
            self.ctx.ur.homed
            and self.ctx.kuka.homed
            and self.ctx.ur.last_job_id == self.ctx.active_home_job_id
            and self.ctx.kuka.last_job_id == self.ctx.active_home_job_id
            and self.ctx.ur.state == "idle"
            and self.ctx.kuka.state == "idle"
        )

    def reset_sequence_done(self) -> bool:
        return (
            self.ctx.ur.last_event == "RESET_DONE"
            and self.ctx.kuka.last_event == "RESET_DONE"
            and self.ctx.ur.last_job_id == self.ctx.active_reset_job_id
            and self.ctx.kuka.last_job_id == self.ctx.active_reset_job_id
        )

    def operator_cmd_callback(self, msg: String) -> None:
        cmd = msg.data.strip().upper()

        if cmd == "START":
            self.start_requested = True
        elif cmd == "STOP":
            self.stop_requested = True
        elif cmd == "PAUSE":
            self.pause_requested = True
        elif cmd == "RESUME":
            self.resume_requested = True
        elif cmd == "RESET":
            self.reset_requested = True
        elif cmd == "AUTO_ON":
            self.ctx.auto_mode = True
        elif cmd == "AUTO_OFF":
            self.ctx.auto_mode = False
        else:
            self.get_logger().warn(f"Unknown operator command: {cmd}")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            subscribe_topics = [
                (robot_state_mqtt_topic(self.ctx.ur.name), 1),
                (robot_event_mqtt_topic(self.ctx.ur.name), 1),
                (robot_state_mqtt_topic(self.ctx.kuka.name), 1),
                (robot_event_mqtt_topic(self.ctx.kuka.name), 1),
                (CELL_SAFETY_ALARM_MQTT_TOPIC, 1),
                (CELL_SAFETY_RESET_MQTT_TOPIC, 1),
                (CELL_OPERATOR_CMD_MQTT_TOPIC, 1),
            ]
            for topic, qos in subscribe_topics:
                client.subscribe(topic, qos=qos)
        else:
            self.get_logger().error(f"Failed to connect to MQTT broker, rc={rc}")

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"Disconnected from MQTT broker, rc={rc}")

    def on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        payload_raw = msg.payload.decode("utf-8", errors="ignore")

        try:
            payload = json.loads(payload_raw)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON on topic {topic}: {payload_raw}")
            return

        if topic == robot_state_mqtt_topic(self.ctx.ur.name):
            self.handle_robot_state(self.ctx.ur.name, payload)
        elif topic == robot_state_mqtt_topic(self.ctx.kuka.name):
            self.handle_robot_state(self.ctx.kuka.name, payload)
        elif topic == robot_event_mqtt_topic(self.ctx.ur.name):
            self.handle_robot_event(self.ctx.ur.name, payload)
        elif topic == robot_event_mqtt_topic(self.ctx.kuka.name):
            self.handle_robot_event(self.ctx.kuka.name, payload)
        elif topic == CELL_SAFETY_ALARM_MQTT_TOPIC:
            self.handle_safety_alarm(payload)
        elif topic == CELL_SAFETY_RESET_MQTT_TOPIC:
            self.handle_safety_reset(payload)
        elif topic == CELL_OPERATOR_CMD_MQTT_TOPIC:
            self.handle_operator_mqtt_command(payload)

    def handle_robot_state(self, robot_name: str, payload: dict) -> None:
        robot = self.robot_by_name(robot_name)
        if robot is None:
            return

        robot.state = payload.get("state", robot.state)
        robot.task = payload.get("task", robot.task)
        robot.error = bool(payload.get("error", robot.error))
        robot.ready = bool(payload.get("ready", robot.ready))
        robot.homed = bool(payload.get("homed", robot.homed))
        robot.motion = payload.get("motion", robot.motion)
        robot.error_message = payload.get("message", "")
        robot.heartbeat_time = self.now()

    def handle_robot_event(self, robot_name: str, payload: dict) -> None:
        robot = self.robot_by_name(robot_name)
        if robot is None:
            return

        robot.last_event = payload.get("event", "")
        robot.last_job_id = int(payload.get("job_id", -1))
        robot.heartbeat_time = self.now()

        if robot.last_event == "HOMED":
            robot.homed = True
        elif robot.last_event in {"TASK_FAILED", "FAULT", "PROTECTIVE_STOP"}:
            robot.error = True
        elif robot.last_event == "RESET_DONE":
            robot.error = False
            robot.ready = False
        elif robot.last_event == "EMERGENCY_STOP":
            self.ctx.emergency_active = True

    def handle_safety_alarm(self, payload: dict) -> None:
        active = bool(payload.get("active", False))
        source = payload.get("source", "unknown")
        alarm_type = payload.get("type", "UNKNOWN")
        if active:
            self.latch_emergency(f"Safety alarm active: {alarm_type}", source=source)

    def handle_safety_reset(self, payload: dict) -> None:
        if bool(payload.get("reset", False)):
            self.ctx.emergency_active = False

    def handle_operator_mqtt_command(self, payload: dict) -> None:
        cmd = str(payload.get("command", "")).strip().upper()
        if cmd == "START":
            self.start_requested = True
        elif cmd == "STOP":
            self.stop_requested = True
        elif cmd == "PAUSE":
            self.pause_requested = True
        elif cmd == "RESUME":
            self.resume_requested = True
        elif cmd == "RESET":
            self.reset_requested = True
        elif cmd == "AUTO_ON":
            self.ctx.auto_mode = True
        elif cmd == "AUTO_OFF":
            self.ctx.auto_mode = False

    def timer_callback(self) -> None:
        self.step_fsm()
        if (self.now() - self.last_state_publish_time) >= self.state_publish_period:
            self.publish_supervisor_state()
            self.last_state_publish_time = self.now()

    def step_fsm(self) -> None:
        if (
            self.ctx.current_state != "EMERGENCY_STOP"
            and (
                self.ctx.emergency_active
                or self.ctx.ur.state == "emergency_stop"
                or self.ctx.kuka.state == "emergency_stop"
            )
        ):
            source = self.ctx.last_alarm_source or "robot_emergency"
            self.latch_emergency("Emergency active", source)
            return

        if self.stop_requested and self.ctx.current_state in {"STOPPING", "STOPPED", "EMERGENCY_STOP"}:
            self.stop_requested = False

        if self.stop_requested and self.ctx.current_state not in {"STOPPING", "STOPPED", "EMERGENCY_STOP"}:
            self.begin_operator_stop()
            return

        if self.ctx.current_state not in ("INIT", "EMERGENCY_STOP") and not self.both_alive():
            self.latch_error("Heartbeat lost from one or both robots")
            return

        if self.ctx.current_state not in ("ERROR", "EMERGENCY_STOP") and (self.ctx.ur.error or self.ctx.kuka.error):
            self.latch_error("Robot error detected")
            return

        if self.pause_requested and self.ctx.current_state not in ("PAUSED", "ERROR", "EMERGENCY_STOP"):
            self.pause_requested = False
            self.transition("PAUSED")
            return

        state = self.ctx.current_state

        if state == "INIT":
            if self.both_alive():
                self.begin_home_sequence("WAIT_STARTUP_HOME")
            elif self.time_in_state() > INIT_TIMEOUT:
                self.latch_error("Initialization timeout: robots are not alive")

        elif state == "WAIT_STARTUP_HOME":
            if self.home_sequence_done():
                self.transition("IDLE")
            elif self.time_in_state() > HOMING_TIMEOUT:
                self.latch_error("Startup homing timeout")

        elif state == "IDLE":
            if self.start_requested:
                self.start_requested = False
                self.clear_robot_events()
                self.transition("WAIT_BOTH_READY")

        elif state == "WAIT_BOTH_READY":
            if self.both_ready():
                self.transition("START_UR_TASK")
            elif self.time_in_state() > READY_TIMEOUT:
                self.latch_error("Robots not ready in time")

        elif state == "START_UR_TASK":
            self.ctx.cycle_id = self.allocate_job_id()
            self.publish_command(self.ctx.ur.name, COMMAND_UR_TASK, self.ctx.cycle_id)
            self.transition("WAIT_UR_DONE")

        elif state == "WAIT_UR_DONE":
            if self.ctx.ur.last_event == "TASK_DONE" and self.ctx.ur.last_job_id == self.ctx.cycle_id:
                self.ctx.ur.last_event = ""
                self.transition("START_KUKA_TASK")
            elif self.time_in_state() > UR_TASK_TIMEOUT:
                self.latch_error("UR task timeout")

        elif state == "START_KUKA_TASK":
            self.publish_command(self.ctx.kuka.name, COMMAND_KUKA_TASK, self.ctx.cycle_id)
            self.transition("WAIT_KUKA_DONE")

        elif state == "WAIT_KUKA_DONE":
            if self.ctx.kuka.last_event == "TASK_DONE" and self.ctx.kuka.last_job_id == self.ctx.cycle_id:
                self.ctx.kuka.last_event = ""
                self.transition("CYCLE_COMPLETE")
            elif self.time_in_state() > KUKA_TASK_TIMEOUT:
                self.latch_error("KUKA task timeout")

        elif state == "CYCLE_COMPLETE":
            if self.ctx.auto_mode:
                self.transition("WAIT_BOTH_READY")
            else:
                self.transition("IDLE")

        elif state == "STOPPING":
            if self.robot_motion_cleared(self.ctx.ur) and self.robot_motion_cleared(self.ctx.kuka):
                self.transition("STOPPED")
            elif self.time_in_state() > STOP_TIMEOUT:
                self.latch_error("Timed out while waiting for robots to stop")

        elif state == "STOPPED":
            if self.reset_requested:
                self.reset_requested = False
                self.begin_reset_sequence()

        elif state == "PAUSED":
            if self.resume_requested:
                self.resume_requested = False
                self.transition("WAIT_BOTH_READY")

        elif state == "ERROR":
            if self.reset_requested:
                self.reset_requested = False
                self.begin_reset_sequence()

        elif state == "EMERGENCY_STOP":
            if not self.ctx.emergency_active and self.reset_requested:
                self.reset_requested = False
                self.begin_reset_sequence()

        elif state == "WAIT_RESET_DONE":
            if self.reset_sequence_done():
                self.begin_home_sequence("WAIT_RECOVERY_HOME")
            elif self.time_in_state() > RESET_TIMEOUT:
                self.latch_error("Reset timeout")

        elif state == "WAIT_RECOVERY_HOME":
            if self.home_sequence_done():
                self.transition("IDLE")
            elif self.time_in_state() > HOMING_TIMEOUT:
                self.latch_error("Recovery homing timeout")

        else:
            self.latch_error(f"Unknown FSM state: {state}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupervisorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Supervisor interrupted by user")
    finally:
        try:
            node.mqtt_client.loop_stop()
            node.mqtt_client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

