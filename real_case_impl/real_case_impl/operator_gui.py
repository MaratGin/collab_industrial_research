import json
import threading
import time
from datetime import datetime

import cv2
import paho.mqtt.client as mqtt
import rclpy
from cv_bridge import CvBridge
from PyQt5.QtCore import QThread, QTimer, Qt, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


BANNER_STYLES = {
    "IDLE": "background:#1d6f42;color:white;",
    "WAIT_BOTH_READY": "background:#1d6f42;color:white;",
    "START_UR_TASK": "background:#215ea6;color:white;",
    "WAIT_UR_DONE": "background:#215ea6;color:white;",
    "START_KUKA_TASK": "background:#215ea6;color:white;",
    "WAIT_KUKA_DONE": "background:#215ea6;color:white;",
    "CYCLE_COMPLETE": "background:#1d6f42;color:white;",
    "PAUSED": "background:#745f00;color:white;",
    "STOPPING": "background:#d97706;color:white;",
    "STOPPED": "background:#b45309;color:white;",
    "ERROR": "background:#9b1c1c;color:white;",
    "EMERGENCY_STOP": "background:#7f1d1d;color:white;",
    "WAIT_RESET_DONE": "background:#6b7280;color:white;",
    "WAIT_RECOVERY_HOME": "background:#6b7280;color:white;",
    "INIT": "background:#374151;color:white;",
}

ROBOT_CARD_STYLES = {
    "idle": "background:#ecfdf5;border:2px solid #10b981;",
    "busy": "background:#eff6ff;border:2px solid #3b82f6;",
    "fault": "background:#fef2f2;border:2px solid #ef4444;",
    "emergency_stop": "background:#fef2f2;border:2px solid #991b1b;",
    "stopped": "background:#fff7ed;border:2px solid #f97316;",
    "offline": "background:#f3f4f6;border:2px solid #9ca3af;",
    "initializing": "background:#fffbeb;border:2px solid #f59e0b;",
}


class GuiRosNode(Node):
    def __init__(self, backend):
        super().__init__("operator_gui_node")
        self.backend = backend
        self.bridge = CvBridge()

        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("operator_cmd_topic", "/cell/operator_cmd")
        self.declare_parameter("supervisor_status_topic", "/cell/supervisor_status")
        self.declare_parameter("ur_status_topic", "/ur5e/task_status")
        self.declare_parameter("kuka_status_topic", "/kuka/task_status")
        self.declare_parameter("visual_safety_status_topic", "/cell/visual_safety_status")
        self.declare_parameter("visual_safety_preview_topic", "/cell/visual_safety_mosaic")
        self.declare_parameter("safety_alarm_topic", "cell/safety/alarm")
        self.declare_parameter("safety_reset_topic", "cell/safety/reset")
        self.declare_parameter("ur_robot_name", "ur5e")
        self.declare_parameter("kuka_robot_name", "kuka")

        self.mqtt_host = self.get_parameter("mqtt_host").get_parameter_value().string_value
        self.mqtt_port = self.get_parameter("mqtt_port").get_parameter_value().integer_value
        self.operator_cmd_topic = (
            self.get_parameter("operator_cmd_topic").get_parameter_value().string_value
        )
        self.supervisor_status_topic = (
            self.get_parameter("supervisor_status_topic").get_parameter_value().string_value
        )
        self.ur_status_topic = self.get_parameter("ur_status_topic").get_parameter_value().string_value
        self.kuka_status_topic = (
            self.get_parameter("kuka_status_topic").get_parameter_value().string_value
        )
        self.visual_safety_status_topic = (
            self.get_parameter("visual_safety_status_topic").get_parameter_value().string_value
        )
        self.visual_safety_preview_topic = (
            self.get_parameter("visual_safety_preview_topic").get_parameter_value().string_value
        )
        self.safety_alarm_topic = (
            self.get_parameter("safety_alarm_topic").get_parameter_value().string_value
        )
        self.safety_reset_topic = (
            self.get_parameter("safety_reset_topic").get_parameter_value().string_value
        )
        self.ur_robot_name = self.get_parameter("ur_robot_name").get_parameter_value().string_value
        self.kuka_robot_name = (
            self.get_parameter("kuka_robot_name").get_parameter_value().string_value
        )

        self.operator_cmd_pub = self.create_publisher(String, self.operator_cmd_topic, 10)
        self.create_subscription(
            String, self.supervisor_status_topic, self._supervisor_status_callback, 10
        )
        self.create_subscription(String, self.ur_status_topic, self._ur_status_callback, 10)
        self.create_subscription(String, self.kuka_status_topic, self._kuka_status_callback, 10)
        self.create_subscription(
            String, self.visual_safety_status_topic, self._visual_safety_status_callback, 10
        )
        self.create_subscription(
            Image, self.visual_safety_preview_topic, self._visual_safety_preview_callback, 10
        )

    def send_operator_command(self, command: str) -> None:
        msg = String()
        msg.data = command
        self.operator_cmd_pub.publish(msg)

    def _decode_json(self, raw: str):
        try:
            return json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Failed to decode JSON payload: {exc}")
            return None

    def _supervisor_status_callback(self, msg: String) -> None:
        payload = self._decode_json(msg.data)
        if payload is not None:
            self.backend.handle_supervisor_status(payload)

    def _ur_status_callback(self, msg: String) -> None:
        payload = self._decode_json(msg.data)
        if payload is not None:
            self.backend.handle_robot_status(self.ur_robot_name, payload)

    def _kuka_status_callback(self, msg: String) -> None:
        payload = self._decode_json(msg.data)
        if payload is not None:
            self.backend.handle_robot_status(self.kuka_robot_name, payload)

    def _visual_safety_status_callback(self, msg: String) -> None:
        payload = self._decode_json(msg.data)
        if payload is not None:
            self.backend.handle_visual_safety_status(payload)

    def _visual_safety_preview_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Failed to convert preview image: {exc}")
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb_frame.shape
        bytes_per_line = channels * width
        image = QImage(
            rgb_frame.data,
            width,
            height,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()
        self.backend.preview_updated.emit(image)


class RosSpinThread(QThread):
    def __init__(self, executor: SingleThreadedExecutor):
        super().__init__()
        self.executor = executor
        self.running = True

    def run(self):
        while self.running:
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self):
        self.running = False


class OperatorBackend(QWidget):
    supervisor_updated = pyqtSignal(dict)
    robot_updated = pyqtSignal(str, dict)
    visual_safety_updated = pyqtSignal(dict)
    preview_updated = pyqtSignal(QImage)
    ros_connection_changed = pyqtSignal(bool)
    mqtt_connection_changed = pyqtSignal(bool)
    event_logged = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.executor = None
        self.ros_thread = None
        self.mqtt_client = None
        self.mqtt_connected = False
        self._started = False

    def start(self):
        if self._started:
            return

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = GuiRosNode(self)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.ros_thread = RosSpinThread(self.executor)
        self.ros_thread.start()
        self.ros_connection_changed.emit(True)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        try:
            self.mqtt_client.connect(self.node.mqtt_host, int(self.node.mqtt_port), 60)
            self.mqtt_client.loop_start()
        except Exception as exc:
            self.event_logged.emit(
                f"{self._now_string()} MQTT connection failed: {self.node.mqtt_host}:{self.node.mqtt_port} ({exc})"
            )
            self.mqtt_connection_changed.emit(False)

        self._started = True

    def shutdown(self):
        if not self._started:
            return

        if self.mqtt_client is not None:
            try:
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except Exception:
                pass

        if self.ros_thread is not None:
            self.ros_thread.stop()
            self.ros_thread.wait(2000)

        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
            self.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        self.ros_connection_changed.emit(False)
        self.mqtt_connection_changed.emit(False)
        self._started = False

    def send_operator_command(self, command: str) -> None:
        if self.node is None:
            return
        self.node.send_operator_command(command)
        self.event_logged.emit(f"{self._now_string()} Operator command sent: {command}")

    def send_emergency_alarm(self) -> None:
        if self.mqtt_client is None or self.node is None:
            return
        payload = {
            "active": True,
            "type": "GUI_EMERGENCY_STOP",
            "source": "operator_gui",
        }
        self.mqtt_client.publish(self.node.safety_alarm_topic, json.dumps(payload), qos=1, retain=False)
        self.event_logged.emit(f"{self._now_string()} Emergency alarm published from GUI")

    def send_safety_reset(self) -> None:
        if self.mqtt_client is None or self.node is None:
            return
        payload = {"reset": True}
        self.mqtt_client.publish(self.node.safety_reset_topic, json.dumps(payload), qos=1, retain=False)
        self.event_logged.emit(f"{self._now_string()} Safety reset published from GUI")

    def handle_supervisor_status(self, payload: dict) -> None:
        self.supervisor_updated.emit(payload)

    def handle_robot_status(self, robot_name: str, payload: dict) -> None:
        self.robot_updated.emit(robot_name, payload)

    def handle_visual_safety_status(self, payload: dict) -> None:
        self.visual_safety_updated.emit(payload)

    def _on_mqtt_connect(self, _client, _userdata, _flags, rc):
        self.mqtt_connected = rc == 0
        self.mqtt_connection_changed.emit(self.mqtt_connected)
        if rc == 0:
            self.event_logged.emit(f"{self._now_string()} MQTT connected")
        else:
            self.event_logged.emit(f"{self._now_string()} MQTT connection failed with code {rc}")

    def _on_mqtt_disconnect(self, _client, _userdata, rc):
        self.mqtt_connected = False
        self.mqtt_connection_changed.emit(False)
        self.event_logged.emit(f"{self._now_string()} MQTT disconnected (rc={rc})")

    @staticmethod
    def _now_string() -> str:
        return datetime.now().strftime("%H:%M:%S")


class RobotStatusCard(QFrame):
    def __init__(self, robot_name: str):
        super().__init__()
        self.robot_name = robot_name
        self.last_update_time = 0.0
        self.fields = {}

        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(
            "QFrame {border-radius: 12px;padding: 8px;background:#f3f4f6;border:2px solid #d1d5db;}"
            "QLabel {color:#111827;}"
        )

        layout = QVBoxLayout(self)
        title = QLabel(robot_name.upper())
        title.setStyleSheet("font-size:20px;font-weight:700;")
        layout.addWidget(title)

        grid = QGridLayout()
        for row, field_name in enumerate(
            ["state", "task", "ready", "homed", "motion", "error", "message", "last_ping"]
        ):
            label = QLabel(f"{field_name}:")
            label.setStyleSheet("font-weight:700;")
            value = QLabel("-")
            value.setWordWrap(True)
            grid.addWidget(label, row, 0)
            grid.addWidget(value, row, 1)
            self.fields[field_name] = value
        layout.addLayout(grid)

    def update_payload(self, payload: dict):
        self.last_update_time = time.time()
        for key in ["state", "task", "motion", "message"]:
            self.fields[key].setText(str(payload.get(key, "-")))
        for key in ["ready", "homed", "error"]:
            self.fields[key].setText("YES" if payload.get(key, False) else "NO")

        state = str(payload.get("state", "offline"))
        style = ROBOT_CARD_STYLES.get(state, "background:#f3f4f6;border:2px solid #d1d5db;")
        self.setStyleSheet(
            f"QFrame {{{style} border-radius: 12px;padding: 8px;}} QLabel {{color:#111827;}}"
        )

    def update_last_ping(self):
        if self.last_update_time <= 0.0:
            self.fields["last_ping"].setText("never")
            return
        age = time.time() - self.last_update_time
        self.fields["last_ping"].setText(f"{age:.1f}s ago")


class PreviewDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Visual Safety Cameras")
        self.resize(1200, 500)
        layout = QVBoxLayout(self)
        self.preview_label = QLabel("Waiting for safety preview...")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(1000, 300)
        self.preview_label.setStyleSheet("background:#111827;color:white;border-radius:8px;")
        layout.addWidget(self.preview_label)

    def update_preview(self, image: QImage):
        pixmap = QPixmap.fromImage(image)
        self.preview_label.setPixmap(
            pixmap.scaled(
                self.preview_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )


class OperatorMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Cell Operator Console")
        self.resize(1560, 980)

        self.backend = OperatorBackend()
        self.preview_dialog = PreviewDialog(self)
        self.last_supervisor_state = None
        self.last_popup_key = None
        self.last_visual_stop_sent = False
        self.supervisor_last_update_time = 0.0
        self.safety_last_update_time = 0.0

        self.supervisor_payload = {}
        self.robot_payloads = {}
        self.visual_safety_payload = {}

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(16, 16, 16, 16)
        root_layout.setSpacing(12)

        self.banner_label = QLabel("System state: INIT")
        self.banner_label.setAlignment(Qt.AlignCenter)
        self.banner_label.setStyleSheet(
            "font-size:24px;font-weight:700;padding:14px;border-radius:12px;"
            + BANNER_STYLES["INIT"]
        )
        root_layout.addWidget(self.banner_label)

        status_row = QHBoxLayout()
        self.ros_status_label = self._make_status_chip("ROS", "DISCONNECTED", "#9b1c1c")
        self.mqtt_status_label = self._make_status_chip("MQTT", "DISCONNECTED", "#9b1c1c")
        self.mode_status_label = self._make_status_chip("MODE", "AUTO ?", "#374151")
        self.emergency_status_label = self._make_status_chip("SAFETY", "NORMAL", "#1d6f42")
        status_row.addWidget(self.ros_status_label)
        status_row.addWidget(self.mqtt_status_label)
        status_row.addWidget(self.mode_status_label)
        status_row.addWidget(self.emergency_status_label)
        status_row.addStretch(1)
        root_layout.addLayout(status_row)

        middle_row = QHBoxLayout()
        middle_row.setSpacing(12)
        root_layout.addLayout(middle_row, stretch=1)

        middle_row.addWidget(self._build_controls_panel(), stretch=1)
        middle_row.addWidget(self._build_robot_panel(), stretch=2)
        middle_row.addWidget(self._build_safety_panel(), stretch=2)

        self.event_log = QPlainTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setMaximumBlockCount(400)
        self.event_log.setStyleSheet(
            "background:#0f172a;color:#e5e7eb;border-radius:12px;padding:8px;font-family:monospace;"
        )
        root_layout.addWidget(self.event_log, stretch=1)

        self.backend.supervisor_updated.connect(self.on_supervisor_update)
        self.backend.robot_updated.connect(self.on_robot_update)
        self.backend.visual_safety_updated.connect(self.on_visual_safety_update)
        self.backend.preview_updated.connect(self.on_preview_update)
        self.backend.ros_connection_changed.connect(self.on_ros_connection_changed)
        self.backend.mqtt_connection_changed.connect(self.on_mqtt_connection_changed)
        self.backend.event_logged.connect(self.append_log)

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_last_ping)
        self.refresh_timer.start(500)

        self.backend.start()
        self.update_button_states()

    def _build_controls_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame {background:#f8fafc;border-radius:12px;border:1px solid #d1d5db;}")
        layout = QVBoxLayout(frame)
        title = QLabel("Operator Controls")
        title.setStyleSheet("font-size:18px;font-weight:700;")
        layout.addWidget(title)

        self.command_buttons = {}
        for command in ["START", "STOP", "PAUSE", "RESUME", "RESET", "AUTO ON", "AUTO OFF"]:
            button = QPushButton(command)
            button.setMinimumHeight(46)
            button.clicked.connect(lambda _checked=False, cmd=command: self.handle_command_button(cmd))
            button.setStyleSheet(
                "QPushButton {background:#e5e7eb;border:none;border-radius:10px;font-weight:600;padding:10px;}"
                "QPushButton:hover {background:#d1d5db;}"
                "QPushButton:disabled {background:#f3f4f6;color:#9ca3af;}"
            )
            layout.addWidget(button)
            self.command_buttons[command] = button

        self.emergency_button = QPushButton("EMERGENCY STOP")
        self.emergency_button.setMinimumHeight(60)
        self.emergency_button.clicked.connect(self.handle_emergency_stop)
        self.emergency_button.setStyleSheet(
            "QPushButton {background:#b91c1c;color:white;border:none;border-radius:12px;font-size:18px;font-weight:800;padding:12px;}"
            "QPushButton:hover {background:#991b1b;}"
        )
        layout.addSpacing(12)
        layout.addWidget(self.emergency_button)

        self.safety_reset_button = QPushButton("SAFETY RESET")
        self.safety_reset_button.setMinimumHeight(48)
        self.safety_reset_button.clicked.connect(self.handle_safety_reset)
        self.safety_reset_button.setStyleSheet(
            "QPushButton {background:#7c3aed;color:white;border:none;border-radius:10px;font-weight:700;padding:10px;}"
            "QPushButton:hover {background:#6d28d9;}"
            "QPushButton:disabled {background:#ddd6fe;color:#6b7280;}"
        )
        layout.addWidget(self.safety_reset_button)
        layout.addStretch(1)
        return frame

    def _build_robot_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame {background:#f8fafc;border-radius:12px;border:1px solid #d1d5db;}")
        layout = QVBoxLayout(frame)
        title = QLabel("Robot Status")
        title.setStyleSheet("font-size:22px;font-weight:700;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        cards_row = QHBoxLayout()
        self.ur_card = RobotStatusCard("ur5e")
        self.kuka_card = RobotStatusCard("kuka")
        cards_row.addWidget(self.ur_card)
        cards_row.addWidget(self.kuka_card)
        layout.addLayout(cards_row)
        return frame

    def _build_safety_panel(self):
        frame = QFrame()
        frame.setStyleSheet("QFrame {background:#f8fafc;border-radius:12px;border:1px solid #d1d5db;}")
        layout = QVBoxLayout(frame)
        title = QLabel("Visual Safety")
        title.setStyleSheet("font-size:20px;font-weight:700;")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.visual_status_label = QLabel("Status: waiting")
        self.visual_status_label.setStyleSheet("font-size:16px;font-weight:700;")
        self.visual_probability_label = QLabel("Fused danger probability: -")
        self.visual_probability_label.setStyleSheet("font-size:16px;font-weight:600;")
        self.visual_stop_sent_label = QLabel("Stop sent: NO")
        self.visual_last_ping_label = QLabel("Last update: never")
        layout.addWidget(self.visual_status_label)
        layout.addWidget(self.visual_probability_label)
        layout.addWidget(self.visual_stop_sent_label)
        layout.addWidget(self.visual_last_ping_label)

        self.preview_label = QLabel("Waiting for safety preview...")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(520, 240)
        self.preview_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.preview_label.setStyleSheet(
            "background:#111827;color:white;border-radius:12px;border:1px solid #374151;"
        )
        layout.addWidget(self.preview_label, stretch=1)

        self.watch_cameras_button = QPushButton("Watch Cameras")
        self.watch_cameras_button.setMinimumHeight(42)
        self.watch_cameras_button.clicked.connect(self.preview_dialog.show)
        self.watch_cameras_button.setStyleSheet(
            "QPushButton {background:#1f2937;color:white;border:none;border-radius:10px;font-weight:700;padding:10px;}"
            "QPushButton:hover {background:#111827;}"
        )
        layout.addWidget(self.watch_cameras_button)
        return frame

    def _make_status_chip(self, title: str, value: str, color: str):
        label = QLabel(f"{title}: {value}")
        label.setStyleSheet(
            f"background:{color};color:white;border-radius:999px;padding:8px 14px;font-weight:700;"
        )
        return label

    def handle_command_button(self, command: str):
        if command == "RESET":
            answer = QMessageBox.question(
                self,
                "Confirm Reset",
                "Reset the system and start recovery/homing?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if answer != QMessageBox.Yes:
                return

        command_payload = command.replace(" ", "_")
        self.backend.send_operator_command(command_payload)

    def handle_emergency_stop(self):
        self.backend.send_emergency_alarm()

    def handle_safety_reset(self):
        answer = QMessageBox.question(
            self,
            "Confirm Safety Reset",
            "Clear the emergency safety latch? Use only after the danger is gone.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.backend.send_safety_reset()

    def on_ros_connection_changed(self, connected: bool):
        text = "CONNECTED" if connected else "DISCONNECTED"
        color = "#1d6f42" if connected else "#9b1c1c"
        self.ros_status_label.setText(f"ROS: {text}")
        self.ros_status_label.setStyleSheet(
            f"background:{color};color:white;border-radius:999px;padding:8px 14px;font-weight:700;"
        )

    def on_mqtt_connection_changed(self, connected: bool):
        text = "CONNECTED" if connected else "DISCONNECTED"
        color = "#1d6f42" if connected else "#9b1c1c"
        self.mqtt_status_label.setText(f"MQTT: {text}")
        self.mqtt_status_label.setStyleSheet(
            f"background:{color};color:white;border-radius:999px;padding:8px 14px;font-weight:700;"
        )

    def on_supervisor_update(self, payload: dict):
        self.supervisor_payload = payload
        self.supervisor_last_update_time = time.time()
        state = str(payload.get("state", "INIT"))
        auto_mode = bool(payload.get("auto_mode", False))
        emergency_active = bool(payload.get("emergency_active", False))

        self.banner_label.setText(f"System state: {state}")
        self.banner_label.setStyleSheet(
            "font-size:24px;font-weight:700;padding:14px;border-radius:12px;"
            + BANNER_STYLES.get(state, BANNER_STYLES["INIT"])
        )

        self.mode_status_label.setText(f"MODE: {'AUTO' if auto_mode else 'MANUAL'}")
        self.emergency_status_label.setText(
            f"SAFETY: {'EMERGENCY' if emergency_active else 'NORMAL'}"
        )
        self.emergency_status_label.setStyleSheet(
            f"background:{"#a01111" if emergency_active else '#1d6f42'};"
            "color:white;border-radius:999px;padding:8px 14px;font-weight:700;"
        )

        if state != self.last_supervisor_state:
            self.append_log(f"{self._now_string()} Supervisor state -> {state}")
            if state == "EMERGENCY_STOP":
                self.show_non_blocking_alert(
                    "Emergency Stop",
                    "The system entered EMERGENCY_STOP.",
                )
            elif state in {"STOPPING", "STOPPED"}:
                self.show_non_blocking_alert(
                    "Controlled Stop",
                    f"The system entered {state}. Use RESET to recover.",
                )
            self.last_supervisor_state = state

        self.update_button_states()

    def on_robot_update(self, robot_name: str, payload: dict):
        self.robot_payloads[robot_name] = payload
        if robot_name.lower() == "ur5e":
            self.ur_card.update_payload(payload)
        else:
            self.kuka_card.update_payload(payload)

        state = str(payload.get("state", ""))
        error = bool(payload.get("error", False))
        if error or state in {"fault", "emergency_stop"}:
            self.append_log(
                f"{self._now_string()} {robot_name} state={state} message={payload.get('message', '')}"
            )

    def on_visual_safety_update(self, payload: dict):
        self.visual_safety_payload = payload
        self.safety_last_update_time = time.time()
        status = str(payload.get("status", "UNKNOWN"))
        fused_p_danger = float(payload.get("fused_p_danger", 0.0))
        stop_sent = bool(payload.get("stop_sent", False))
        danger_active = bool(payload.get("danger_active", False))

        self.visual_status_label.setText(f"Status: {status}")
        self.visual_probability_label.setText(f"Fused danger probability: {fused_p_danger:.2f}")
        self.visual_stop_sent_label.setText(f"Stop sent: {'YES' if stop_sent else 'NO'}")

        if status == "DANGER":
            self.visual_status_label.setStyleSheet("font-size:16px;font-weight:700;color:#b91c1c;")
            self.preview_label.setStyleSheet(
                "background:#1f2937;color:white;border-radius:12px;border:3px solid #b91c1c;"
            )
            if stop_sent and not self.last_visual_stop_sent:
                self.append_log(
                    f"{self._now_string()} Visual safety danger active, automatic STOP sent"
                )
        elif status == "SAFE":
            self.visual_status_label.setStyleSheet("font-size:16px;font-weight:700;color:#1d6f42;")
            self.preview_label.setStyleSheet(
                "background:#111827;color:white;border-radius:12px;border:1px solid #374151;"
            )
        else:
            self.visual_status_label.setStyleSheet("font-size:16px;font-weight:700;color:#92400e;")

        if danger_active and stop_sent:
            self.show_non_blocking_alert(
                "Visual Safety Stop",
                "The visual safety system detected danger and triggered STOP.",
                alert_key="visual_safety_stop",
            )
        else:
            self.last_popup_key = None

        self.last_visual_stop_sent = stop_sent

    def on_preview_update(self, image: QImage):
        pixmap = QPixmap.fromImage(image)
        self.preview_label.setPixmap(
            pixmap.scaled(
                self.preview_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )
        self.preview_dialog.update_preview(image)

    def refresh_last_ping(self):
        self.ur_card.update_last_ping()
        self.kuka_card.update_last_ping()

        if self.safety_last_update_time <= 0.0:
            self.visual_last_ping_label.setText("Last update: never")
        else:
            age = time.time() - self.safety_last_update_time
            self.visual_last_ping_label.setText(f"Last update: {age:.1f}s ago")

    def update_button_states(self):
        state = str(self.supervisor_payload.get("state", "INIT"))
        emergency_active = bool(self.supervisor_payload.get("emergency_active", False))

        self.command_buttons["START"].setEnabled(state == "IDLE")
        self.command_buttons["STOP"].setEnabled(state not in {"INIT", "STOPPING", "STOPPED", "EMERGENCY_STOP"})
        self.command_buttons["PAUSE"].setEnabled(state not in {"PAUSED", "EMERGENCY_STOP", "STOPPING", "STOPPED"})
        self.command_buttons["RESUME"].setEnabled(state == "PAUSED")
        self.command_buttons["RESET"].setEnabled(state in {"STOPPED", "ERROR", "EMERGENCY_STOP"})
        self.command_buttons["AUTO ON"].setEnabled(True)
        self.command_buttons["AUTO OFF"].setEnabled(True)
        self.safety_reset_button.setEnabled(emergency_active)

    def append_log(self, message: str):
        self.event_log.appendPlainText(message)

    def show_non_blocking_alert(self, title: str, message: str, alert_key: str = None):
        key = alert_key or title
        if self.last_popup_key == key:
            return
        self.last_popup_key = key

        box = QMessageBox(self)
        box.setWindowTitle(title)
        box.setText(message)
        box.setIcon(QMessageBox.Warning)
        box.setModal(False)
        box.setAttribute(Qt.WA_DeleteOnClose, True)
        box.show()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        current_pixmap = self.preview_label.pixmap()
        if current_pixmap is not None:
            self.preview_label.setPixmap(
                current_pixmap.scaled(
                    self.preview_label.size(),
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
            )

    def closeEvent(self, event):
        self.backend.shutdown()
        super().closeEvent(event)

    @staticmethod
    def _now_string() -> str:
        return datetime.now().strftime("%H:%M:%S")


def main():
    app = QApplication([])
    app.setStyleSheet(
        "QMainWindow {background:#e5e7eb;}"
        "QLabel {font-size:14px;}"
        "QPushButton {font-size:14px;}"
    )
    window = OperatorMainWindow()
    window.show()
    return app.exec_()
