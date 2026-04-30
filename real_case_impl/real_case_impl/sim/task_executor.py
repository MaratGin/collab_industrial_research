import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from real_case_impl.common.constants import COMMAND_HOME


STATUS_PUBLISH_PERIOD = 0.2
JOINT_STATE_TIMEOUT = 1.0
SERVER_CHECK_PERIOD = 1.0
ACTION_TIMEOUT = 20.0
CANCEL_TIMEOUT = 5.0


@dataclass
class ActionBinding:
    name: str
    action_name: str
    action_type: str
    joint_names: list[str]
    goal_tolerance: float
    max_effort: float
    client: ActionClient


@dataclass
class ExecutionStep:
    name: str
    controller: str
    duration: float
    positions: list[float]


@dataclass
class TaskDefinition:
    command_name: str
    task_name: str
    steps: list[ExecutionStep]


@dataclass
class ExecutorContext:
    robot_name: str
    state: str = "initializing"
    task: str = "none"
    ready: bool = False
    homed: bool = False
    motion: str = "stopped"
    error: bool = False
    message: str = ""
    task_active: bool = False
    current_job_id: int = -1
    current_command: str = ""
    task_phase: str = "none"
    emergency_latched: bool = False
    last_joint_state_time: float = 0.0
    last_status_publish_time: float = field(default_factory=time.time)


class ConfigDrivenTaskExecutor(Node):
    def __init__(self, node_name: str, default_robot_name: str) -> None:
        super().__init__(node_name)

        self.declare_parameter("robot_name", default_robot_name)
        self.declare_parameter("status_publish_period", STATUS_PUBLISH_PERIOD)
        self.declare_parameter("joint_state_timeout_sec", JOINT_STATE_TIMEOUT)
        self.declare_parameter("server_check_period_sec", SERVER_CHECK_PERIOD)
        self.declare_parameter("action_timeout_sec", ACTION_TIMEOUT)
        self.declare_parameter("cancel_timeout_sec", CANCEL_TIMEOUT)
        self.declare_parameter("task_config_path", "")
        self.declare_parameter("arm_controller_action", "")
        self.declare_parameter("gripper_controller_action", "")
        self.declare_parameter("joint_state_topic", "")

        robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.status_publish_period = (
            self.get_parameter("status_publish_period").get_parameter_value().double_value
        )
        self.joint_state_timeout_sec = (
            self.get_parameter("joint_state_timeout_sec").get_parameter_value().double_value
        )
        self.server_check_period_sec = (
            self.get_parameter("server_check_period_sec").get_parameter_value().double_value
        )
        self.action_timeout_sec = (
            self.get_parameter("action_timeout_sec").get_parameter_value().double_value
        )
        self.cancel_timeout_sec = (
            self.get_parameter("cancel_timeout_sec").get_parameter_value().double_value
        )
        self.task_config_path = self.get_parameter("task_config_path").get_parameter_value().string_value
        self.arm_controller_override = (
            self.get_parameter("arm_controller_action").get_parameter_value().string_value
        )
        self.gripper_controller_override = (
            self.get_parameter("gripper_controller_action").get_parameter_value().string_value
        )
        self.joint_state_topic_override = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )

        self.ctx = ExecutorContext(robot_name=robot_name)
        self.current_task: Optional[TaskDefinition] = None
        self.current_step_index = -1
        self.current_step_started_at = 0.0
        self.stop_requested = False
        self.stop_is_emergency = False
        self.stop_request_job_id = -1
        self.active_goal_handle = None
        self.active_result_future = None
        self.send_goal_future = None
        self.cancel_future = None
        self.last_server_check_time = 0.0
        self.controllers_ready = False

        config = self._load_robot_config(robot_name)
        controller_config = config["controllers"]
        self.tasks = self._parse_tasks(config["tasks"])

        joint_state_topic = self.joint_state_topic_override or config.get(
            "joint_state_topic", f"/{robot_name}/joint_states"
        )
        if self.arm_controller_override:
            controller_config["arm"]["action_name"] = self.arm_controller_override
        if self.gripper_controller_override:
            controller_config["gripper"]["action_name"] = self.gripper_controller_override

        self.action_bindings = self._create_action_bindings(controller_config)

        self.task_cmd_sub = self.create_subscription(
            String,
            f"/{robot_name}/task_cmd",
            self.task_cmd_callback,
            10,
        )
        self.task_status_pub = self.create_publisher(String, f"/{robot_name}/task_status", 10)
        self.task_event_pub = self.create_publisher(String, f"/{robot_name}/task_event", 10)
        self.joint_state_sub = self.create_subscription(
            JointState,
            joint_state_topic,
            self.joint_state_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f"{node_name} started for robot '{robot_name}'")
        self.get_logger().info(f"Task config: {self._resolve_config_path()}")
        self.get_logger().info(f"Joint state topic: {joint_state_topic}")

    def now(self) -> float:
        return time.time()

    def _resolve_config_path(self) -> str:
        if self.task_config_path:
            return self.task_config_path
        share_dir = get_package_share_directory("real_case_impl")
        return str(Path(share_dir) / "config" / "sim_robot_task_sequences.yaml")

    def _load_robot_config(self, robot_name: str) -> dict[str, Any]:
        config_path = Path(self._resolve_config_path())
        if not config_path.exists():
            raise FileNotFoundError(f"Task config file does not exist: {config_path}")

        with config_path.open("r", encoding="utf-8") as handle:
            raw_config = yaml.safe_load(handle) or {}

        robots = raw_config.get("robots", {})
        if robot_name not in robots:
            raise KeyError(f"Robot '{robot_name}' is missing from {config_path}")

        robot_config = robots[robot_name]
        if "controllers" not in robot_config or "tasks" not in robot_config:
            raise KeyError(f"Robot '{robot_name}' config must define controllers and tasks")
        return robot_config

    def _parse_tasks(self, raw_tasks: dict[str, Any]) -> dict[str, TaskDefinition]:
        tasks: dict[str, TaskDefinition] = {}
        for command_name, raw_task in raw_tasks.items():
            steps: list[ExecutionStep] = []
            for raw_step in raw_task.get("steps", []):
                positions = raw_step.get("positions", [])
                if isinstance(positions, (float, int)):
                    positions = [float(positions)]
                steps.append(
                    ExecutionStep(
                        name=str(raw_step.get("name", raw_step.get("controller", "step"))),
                        controller=str(raw_step["controller"]),
                        duration=float(raw_step.get("duration", 2.0)),
                        positions=[float(value) for value in positions],
                    )
                )
            tasks[command_name] = TaskDefinition(
                command_name=command_name,
                task_name=str(raw_task.get("task_name", command_name.lower())),
                steps=steps,
            )
        return tasks

    def _create_action_bindings(self, raw_bindings: dict[str, Any]) -> dict[str, ActionBinding]:
        bindings: dict[str, ActionBinding] = {}
        for name, raw_binding in raw_bindings.items():
            action_type = str(raw_binding.get("action_type", "follow_joint_trajectory"))
            action_name = str(raw_binding["action_name"])
            joint_names = [str(value) for value in raw_binding.get("joint_names", [])]
            goal_tolerance = float(raw_binding.get("goal_tolerance_sec", 1.0))
            max_effort = float(raw_binding.get("max_effort", 40.0))

            if action_type == "gripper_command":
                client = ActionClient(self, GripperCommand, action_name)
            elif action_type == "follow_joint_trajectory":
                client = ActionClient(self, FollowJointTrajectory, action_name)
            else:
                raise ValueError(f"Unsupported action type '{action_type}' for controller '{name}'")

            bindings[name] = ActionBinding(
                name=name,
                action_name=action_name,
                action_type=action_type,
                joint_names=joint_names,
                goal_tolerance=goal_tolerance,
                max_effort=max_effort,
                client=client,
            )
        return bindings

    def publish_status(self) -> None:
        payload = {
            "robot": self.ctx.robot_name,
            "state": self.ctx.state,
            "task": self.ctx.task,
            "ready": self.ctx.ready,
            "homed": self.ctx.homed,
            "motion": self.ctx.motion,
            "error": self.ctx.error,
            "message": self.ctx.message,
            "timestamp": self.now(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.task_status_pub.publish(msg)

    def publish_event(
        self,
        event_name: str,
        message: str = "",
        task_name: Optional[str] = None,
        job_id: Optional[int] = None,
    ) -> None:
        payload = {
            "robot": self.ctx.robot_name,
            "event": event_name,
            "task": self.ctx.task if task_name is None else task_name,
            "job_id": self.ctx.current_job_id if job_id is None else job_id,
            "message": message,
            "timestamp": self.now(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.task_event_pub.publish(msg)
        self.get_logger().info(
            f"Published event: event={event_name}, task={payload['task']}, "
            f"job_id={payload['job_id']}, message={message}"
        )

    def joint_state_callback(self, _msg: JointState) -> None:
        self.ctx.last_joint_state_time = self.now()

    def task_cmd_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Invalid JSON on task_cmd: {msg.data}")
            return

        command = str(payload.get("command", "")).strip().upper()
        job_id = int(payload.get("job_id", -1))
        source = str(payload.get("source", "")).strip().lower()

        self.get_logger().info(f"Received command: {command}, job_id={job_id}, source={source}")

        if command == "STOP":
            self.handle_stop(job_id, source == "safety_alarm")
        elif command == "RESET":
            self.handle_reset(job_id)
        else:
            self.handle_task_start(command, job_id)

    def handle_task_start(self, command: str, job_id: int) -> None:
        if command not in self.tasks:
            self.publish_event("TASK_FAILED", message=f"Unsupported task command: {command}", job_id=job_id)
            return

        if self.ctx.task_active:
            self.publish_event("TASK_FAILED", message="Another task is already active", job_id=job_id)
            return

        if self.ctx.error:
            self.publish_event("TASK_FAILED", message="Executor is in fault state", job_id=job_id)
            return

        if self.ctx.emergency_latched:
            self.publish_event("TASK_FAILED", message="Emergency reset is required", job_id=job_id)
            return

        if self.ctx.state == "stopped":
            self.publish_event("TASK_FAILED", message="Reset is required before starting a new task", job_id=job_id)
            return

        if not self.controllers_ready:
            self.fail_task("Required action server is unavailable", job_id=job_id)
            return

        self.current_task = self.tasks[command]
        self.current_step_index = -1
        self.current_step_started_at = 0.0
        self.ctx.task_active = True
        self.ctx.current_job_id = job_id
        self.ctx.current_command = command
        self.ctx.task = self.current_task.task_name
        self.ctx.state = "busy"
        self.ctx.motion = "running"
        self.ctx.ready = False
        self.ctx.error = False
        self.ctx.message = ""
        self.ctx.task_phase = "starting"
        self.stop_requested = False
        self.stop_is_emergency = False
        self.stop_request_job_id = -1

        if command == COMMAND_HOME:
            self.ctx.homed = False

        self.publish_event("TASK_STARTED", job_id=job_id)
        self._start_next_step()

    def handle_stop(self, job_id: int, emergency: bool) -> None:
        if not self.ctx.task_active:
            self.publish_event("TASK_FAILED", message="No active task to stop", job_id=job_id)
            return

        self.stop_requested = True
        self.stop_is_emergency = emergency
        self.stop_request_job_id = job_id
        self.ctx.motion = "stopping"
        self.ctx.ready = False
        self.ctx.message = "Emergency stop requested" if emergency else "Stop requested"
        self.ctx.task_phase = "stopping"

        if self.active_goal_handle is not None and self.cancel_future is None:
            self.cancel_future = self.active_goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self._handle_cancel_response)
        elif self.send_goal_future is None:
            self._finish_interrupted_task()

    def handle_reset(self, job_id: int) -> None:
        if self.ctx.task_active:
            self.publish_event("TASK_FAILED", message="Cannot RESET while a task is active", job_id=job_id)
            return

        self.stop_requested = False
        self.stop_is_emergency = False
        self.stop_request_job_id = -1
        self.ctx.current_job_id = -1
        self.ctx.current_command = ""
        self.ctx.task = "none"
        self.ctx.motion = "stopped"
        self.ctx.error = False
        self.ctx.message = ""
        self.ctx.task_phase = "none"
        self.ctx.emergency_latched = False
        self.current_task = None
        self.current_step_index = -1
        self.active_goal_handle = None
        self.active_result_future = None
        self.send_goal_future = None
        self.cancel_future = None

        if self._joint_states_alive() and self.controllers_ready:
            self.ctx.state = "idle"
            self.ctx.ready = True
        else:
            self.ctx.state = "initializing"
            self.ctx.ready = False

        self.publish_event("RESET_DONE", job_id=job_id)

    def fail_task(self, reason: str, job_id: Optional[int] = None) -> None:
        failed_job_id = self.ctx.current_job_id if job_id is None else job_id
        failed_task = self.ctx.task if self.ctx.task != "none" else (
            self.current_task.task_name if self.current_task is not None else "none"
        )
        self.ctx.task_active = False
        self.ctx.current_job_id = -1
        self.ctx.current_command = ""
        self.ctx.state = "fault"
        self.ctx.motion = "stopped"
        self.ctx.error = True
        self.ctx.ready = False
        self.ctx.message = reason
        self.ctx.task_phase = "fault"
        self.current_task = None
        self.current_step_index = -1
        self.active_goal_handle = None
        self.active_result_future = None
        self.send_goal_future = None
        self.cancel_future = None
        self.stop_requested = False
        self.stop_is_emergency = False
        self.publish_event("TASK_FAILED", message=reason, task_name=failed_task, job_id=failed_job_id)

    def complete_task(self) -> None:
        finished_task = self.ctx.task
        finished_job_id = self.ctx.current_job_id
        was_home = self.ctx.current_command == COMMAND_HOME

        self.ctx.task_active = False
        self.ctx.current_job_id = -1
        self.ctx.current_command = ""
        self.ctx.task = "none"
        self.ctx.state = "idle"
        self.ctx.motion = "stopped"
        self.ctx.error = False
        self.ctx.ready = True
        self.ctx.message = ""
        self.ctx.task_phase = "none"
        self.current_task = None
        self.current_step_index = -1
        self.active_goal_handle = None
        self.active_result_future = None
        self.send_goal_future = None
        self.cancel_future = None
        self.stop_requested = False
        self.stop_is_emergency = False

        self.publish_event("TASK_DONE", task_name=finished_task, job_id=finished_job_id)

        if was_home:
            self.ctx.homed = True
            self.publish_event("HOMED", task_name=finished_task, job_id=finished_job_id)

    def _start_next_step(self) -> None:
        if not self.ctx.task_active or self.current_task is None:
            return

        self.current_step_index += 1
        if self.current_step_index >= len(self.current_task.steps):
            self.complete_task()
            return

        step = self.current_task.steps[self.current_step_index]
        self.ctx.task_phase = step.name
        self.ctx.message = (
            f"Executing {step.name} ({self.current_step_index + 1}/{len(self.current_task.steps)})"
        )
        self.current_step_started_at = self.now()
        self._dispatch_step(step)

    def _dispatch_step(self, step: ExecutionStep) -> None:
        if step.controller not in self.action_bindings:
            self.fail_task(f"Unknown controller binding: {step.controller}")
            return

        binding = self.action_bindings[step.controller]
        if not binding.client.wait_for_server(timeout_sec=0.0):
            self.fail_task(f"Action server is unavailable: {binding.action_name}")
            return

        goal = self._build_goal(binding, step)
        self.send_goal_future = binding.client.send_goal_async(goal)
        self.send_goal_future.add_done_callback(self._handle_goal_response)

    def _build_goal(self, binding: ActionBinding, step: ExecutionStep):
        if binding.action_type == "follow_joint_trajectory":
            goal = FollowJointTrajectory.Goal()
            trajectory = JointTrajectory()
            trajectory.joint_names = binding.joint_names
            point = JointTrajectoryPoint()
            point.positions = step.positions
            point.time_from_start = Duration(seconds=step.duration).to_msg()
            trajectory.points = [point]
            goal.trajectory = trajectory
            goal.goal_time_tolerance = Duration(seconds=binding.goal_tolerance).to_msg()
            return goal

        goal = GripperCommand.Goal()
        goal.command.position = step.positions[0]
        goal.command.max_effort = binding.max_effort
        return goal

    def _handle_goal_response(self, future) -> None:
        self.send_goal_future = None

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.fail_task(f"Failed to send action goal: {exc}")
            return

        if not goal_handle.accepted:
            self.fail_task("Action goal was rejected by the controller")
            return

        self.active_goal_handle = goal_handle
        if self.stop_requested and self.cancel_future is None:
            self.cancel_future = goal_handle.cancel_goal_async()
            self.cancel_future.add_done_callback(self._handle_cancel_response)

        self.active_result_future = goal_handle.get_result_async()
        self.active_result_future.add_done_callback(self._handle_goal_result)

    def _handle_cancel_response(self, future) -> None:
        self.cancel_future = None
        try:
            future.result()
        except Exception as exc:
            self.fail_task(f"Failed to cancel active goal: {exc}")

    def _handle_goal_result(self, future) -> None:
        self.active_result_future = None
        self.active_goal_handle = None

        try:
            result = future.result()
        except Exception as exc:
            self.fail_task(f"Failed to receive action result: {exc}")
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.stop_requested:
                self._finish_interrupted_task()
            else:
                self._start_next_step()
            return

        if status == GoalStatus.STATUS_CANCELED and self.stop_requested:
            self._finish_interrupted_task()
            return

        self.fail_task(f"Action finished with non-success status {status}")

    def _finish_interrupted_task(self) -> None:
        interrupted_task = self.ctx.task
        interrupted_job_id = self.ctx.current_job_id if self.ctx.current_job_id >= 0 else self.stop_request_job_id

        self.ctx.task_active = False
        self.ctx.current_job_id = -1
        self.ctx.current_command = ""
        self.ctx.task = "none"
        self.ctx.motion = "stopped"
        self.ctx.ready = False
        self.ctx.error = False
        self.ctx.task_phase = "none"
        self.current_task = None
        self.current_step_index = -1
        self.active_goal_handle = None
        self.active_result_future = None
        self.send_goal_future = None
        self.cancel_future = None

        if self.stop_is_emergency:
            self.ctx.state = "emergency_stop"
            self.ctx.emergency_latched = True
            self.ctx.message = "Emergency stop active"
            self.publish_event("EMERGENCY_STOP", task_name=interrupted_task, job_id=interrupted_job_id)
        else:
            self.ctx.state = "stopped"
            self.ctx.message = "Task stopped; RESET required"
            self.publish_event("STOPPED", message="Stop requested by adapter/safety", task_name=interrupted_task, job_id=interrupted_job_id)

        self.stop_requested = False
        self.stop_is_emergency = False
        self.stop_request_job_id = -1

    def _joint_states_alive(self) -> bool:
        if self.ctx.last_joint_state_time <= 0.0:
            return False
        return (self.now() - self.ctx.last_joint_state_time) <= self.joint_state_timeout_sec

    def refresh_connectivity(self) -> None:
        now = self.now()
        if (now - self.last_server_check_time) < self.server_check_period_sec:
            return

        self.last_server_check_time = now
        self.controllers_ready = all(
            binding.client.wait_for_server(timeout_sec=0.0)
            for binding in self.action_bindings.values()
        )

        if self.ctx.task_active:
            return

        joint_states_alive = self._joint_states_alive()
        if self.ctx.error or self.ctx.emergency_latched or self.ctx.state == "stopped":
            return

        if joint_states_alive and self.controllers_ready:
            self.ctx.state = "idle"
            self.ctx.ready = True
            if not self.ctx.message:
                self.ctx.message = ""
        elif joint_states_alive:
            self.ctx.state = "initializing"
            self.ctx.ready = False
            self.ctx.message = "Waiting for action servers"
        else:
            self.ctx.state = "offline"
            self.ctx.ready = False
            self.ctx.message = "Waiting for joint states"

    def check_timeouts(self) -> None:
        if not self.ctx.task_active:
            return

        if self.current_step_started_at <= 0.0:
            return

        elapsed = self.now() - self.current_step_started_at
        timeout_limit = self.action_timeout_sec
        if self.current_task is not None and 0 <= self.current_step_index < len(self.current_task.steps):
            timeout_limit = max(timeout_limit, self.current_task.steps[self.current_step_index].duration + self.cancel_timeout_sec)

        if elapsed > timeout_limit:
            self.fail_task(
                f"Task step timeout in phase '{self.ctx.task_phase}' after {elapsed:.1f}s"
            )

    def timer_callback(self) -> None:
        self.refresh_connectivity()
        self.check_timeouts()

        now = self.now()
        if (now - self.ctx.last_status_publish_time) >= self.status_publish_period:
            self.publish_status()
            self.ctx.last_status_publish_time = now


def run_executor(node_cls, interrupted_message: str, args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = node_cls()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(interrupted_message)
    finally:
        node.destroy_node()
        rclpy.shutdown()

