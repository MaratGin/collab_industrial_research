#!/usr/bin/env python3
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO


DEFAULT_TOP_MODEL = (
    "/home/marat/exchange_project_ws/src/collab_industrial_research/"
    "gz_world_package/config/camera_models/final_top_best.pt"
)
DEFAULT_LEFT_MODEL = (
    "/home/marat/exchange_project_ws/src/collab_industrial_research/"
    "gz_world_package/config/camera_models/final_left_best.pt"
)
DEFAULT_RIGHT_MODEL = (
    "/home/marat/exchange_project_ws/src/collab_industrial_research/"
    "gz_world_package/config/camera_models/final_right_best.pt"
)


class SafetyClassifier3CamsFusion(Node):
    """
    Visual safety monitor for the robotic cell.

    Behavior:
    - reads three camera streams
    - runs one YOLO classifier per camera
    - fuses danger probability across cameras
    - publishes human-readable safety status
    - sends a one-shot supervisor STOP command when danger becomes active
    """

    def __init__(self):
        super().__init__("safety_classifier_3cams_fusion")
        self.bridge = CvBridge()

        self.declare_parameter("top_topic", "/top/kinect/image_raw")
        self.declare_parameter("left_topic", "/left/kinect/image_raw")
        self.declare_parameter("right_topic", "/right/kinect/image_raw")
        self.declare_parameter("top_model_path", DEFAULT_TOP_MODEL)
        self.declare_parameter("left_model_path", DEFAULT_LEFT_MODEL)
        self.declare_parameter("right_model_path", DEFAULT_RIGHT_MODEL)
        self.declare_parameter("danger_threshold", 0.7)
        self.declare_parameter("safe_confidence_threshold", 0.87)
        self.declare_parameter("infer_period_sec", 0.2)
        self.declare_parameter("display_window", True)
        self.declare_parameter("operator_cmd_topic", "/cell/operator_cmd")
        self.declare_parameter("status_topic", "/cell/visual_safety_status")
        self.declare_parameter("preview_topic", "/cell/visual_safety_mosaic")

        top_topic = self.get_parameter("top_topic").get_parameter_value().string_value
        left_topic = self.get_parameter("left_topic").get_parameter_value().string_value
        right_topic = self.get_parameter("right_topic").get_parameter_value().string_value
        top_model_path = self.get_parameter("top_model_path").get_parameter_value().string_value
        left_model_path = self.get_parameter("left_model_path").get_parameter_value().string_value
        right_model_path = self.get_parameter("right_model_path").get_parameter_value().string_value
        self.danger_threshold = (
            self.get_parameter("danger_threshold").get_parameter_value().double_value
        )
        self.safe_confidence_threshold = (
            self.get_parameter("safe_confidence_threshold").get_parameter_value().double_value
        )
        self.infer_period_s = (
            self.get_parameter("infer_period_sec").get_parameter_value().double_value
        )
        self.display_window = (
            self.get_parameter("display_window").get_parameter_value().bool_value
        )
        operator_cmd_topic = (
            self.get_parameter("operator_cmd_topic").get_parameter_value().string_value
        )
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        preview_topic = self.get_parameter("preview_topic").get_parameter_value().string_value

        self.cams = {
            "top": self._build_camera_config(top_topic, top_model_path),
            "left": self._build_camera_config(left_topic, left_model_path),
            "right": self._build_camera_config(right_topic, right_model_path),
        }

        # These indices reflect the current trained models.
        self.safe_id = 1
        self.danger_id = 0

        self.operator_cmd_pub = self.create_publisher(String, operator_cmd_topic, 10)
        self.safety_status_pub = self.create_publisher(String, status_topic, 10)
        self.preview_pub = self.create_publisher(Image, preview_topic, 10)

        self.window_name = "Safety Monitor (3 cams, fused)"
        self.danger_latched = False
        self.stop_sent_for_current_danger = False

        for cam_name, cfg in self.cams.items():
            model_path = Path(cfg["model_path"])
            if not model_path.exists():
                raise FileNotFoundError(f"Model file for {cam_name} camera does not exist: {model_path}")

            cfg["model"] = YOLO(str(model_path))
            self.get_logger().info(f"Loaded model for {cam_name}: {model_path}")

        for cam_name, cfg in self.cams.items():
            self.create_subscription(
                Image,
                cfg["topic"],
                lambda msg, cn=cam_name: self.image_cb(msg, cn),
                10,
            )
            self.get_logger().info(f"Subscribed: {cam_name} -> {cfg['topic']}")

        self.timer = self.create_timer(self.infer_period_s, self.infer_and_draw)
        self.get_logger().info("SafetyClassifier3CamsFusion started.")

    def _build_camera_config(self, topic: str, model_path: str) -> dict:
        return {
            "topic": topic,
            "model_path": model_path,
            "model": None,
            "latest_frame": None,
            "latest_stamp": None,
            "last_pdanger": 0.0,
            "last_psafe": 0.0,
        }

    def image_cb(self, msg: Image, cam_name: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.cams[cam_name]["latest_frame"] = frame
            self.cams[cam_name]["latest_stamp"] = (
                int(msg.header.stamp.sec),
                int(msg.header.stamp.nanosec),
            )
        except Exception as exc:
            self.get_logger().error(f"[{cam_name}] cv_bridge error: {exc}")

    @staticmethod
    def _format_ros_time(stamp_tuple):
        if stamp_tuple is None:
            return "rostime: N/A"
        sec, nsec = stamp_tuple
        return f"rostime: {sec}.{nsec:09d}"

    def classify_pdanger_psafe(self, model: YOLO, frame):
        results = model(frame, verbose=False)
        probs = results[0].probs
        if probs is None or probs.data is None:
            return 0.0, 0.0

        try:
            p_danger = float(probs.data[self.danger_id])
            p_safe = float(probs.data[self.safe_id])
        except Exception:
            top = int(probs.top1)
            conf = float(probs.top1conf)
            if top == self.safe_id:
                p_safe, p_danger = conf, 1.0 - conf
            else:
                p_danger, p_safe = conf, 1.0 - conf

        p_safe = max(0.0, min(1.0, p_safe))
        p_danger = max(0.0, min(1.0, p_danger))
        return p_safe, p_danger

    @staticmethod
    def fuse_3cams(p_top, p_left, p_right):
        return 1.0 - (1.0 - p_top) * (1.0 - p_left) * (1.0 - p_right)

    def publish_supervisor_stop(self, reason: str, p_danger: float) -> None:
        if self.stop_sent_for_current_danger:
            return

        msg = String()
        msg.data = "STOP"
        self.operator_cmd_pub.publish(msg)
        self.stop_sent_for_current_danger = True
        self.get_logger().warn(
            f"Visual safety triggered supervisor STOP: reason={reason}, fused_p_danger={p_danger:.2f}"
        )

    def publish_status(self, overall_status: str, p_safe: float, p_danger: float) -> None:
        payload = {
            "source": "visual_safety_system",
            "status": overall_status,
            "danger_active": self.danger_latched,
            "stop_sent": self.stop_sent_for_current_danger,
            "fused_p_safe": p_safe,
            "fused_p_danger": p_danger,
            "danger_threshold": self.danger_threshold,
            "cameras": {
                cam_name: {
                    "p_safe": cfg["last_psafe"],
                    "p_danger": cfg["last_pdanger"],
                    "has_image": cfg["latest_frame"] is not None,
                    "stamp": cfg["latest_stamp"],
                }
                for cam_name, cfg in self.cams.items()
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.safety_status_pub.publish(msg)

    def infer_and_draw(self):
        panels = []
        p_danger = {"top": 0.0, "left": 0.0, "right": 0.0}
        p_safe = {"top": 0.0, "left": 0.0, "right": 0.0}
        all_images_ready = True

        for cam_name in ["top", "left", "right"]:
            cfg = self.cams[cam_name]
            frame = cfg["latest_frame"]

            if frame is None:
                all_images_ready = False
                blank = np.full((360, 640, 3), 255, dtype=np.uint8)
                cv2.putText(
                    blank,
                    f"{cam_name.upper()}: NO IMAGE",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.2,
                    (0, 0, 255),
                    3,
                    cv2.LINE_AA,
                )
                rt = self._format_ros_time(cfg["latest_stamp"])
                cv2.putText(
                    blank,
                    rt,
                    (20, 340),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.9,
                    (50, 50, 50),
                    2,
                    cv2.LINE_AA,
                )
                panels.append(blank)
                continue

            psafe, pdanger = self.classify_pdanger_psafe(cfg["model"], frame)
            cfg["last_pdanger"] = pdanger
            cfg["last_psafe"] = psafe
            p_safe[cam_name] = psafe
            p_danger[cam_name] = pdanger

            vis = cv2.resize(frame.copy(), (640, 360))
            cam_status = "DANGER" if pdanger >= self.danger_threshold else "SAFE"
            cam_color = (0, 0, 255) if cam_status == "DANGER" else (0, 255, 0)
            header = f"{cam_name.upper()} | p_danger={pdanger:.2f} -> {cam_status}"
            cv2.putText(
                vis,
                header,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                cam_color,
                3,
                cv2.LINE_AA,
            )
            rt = self._format_ros_time(cfg["latest_stamp"])
            cv2.putText(
                vis,
                rt,
                (20, 340),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                vis,
                rt,
                (20, 340),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            panels.append(vis)

        fused_p_safe = self.fuse_3cams(p_safe["top"], p_safe["left"], p_safe["right"])
        fused_p_danger = self.fuse_3cams((p_danger["top"]), (max(p_danger["left"] + 0.3, 1.0)), (max(p_danger["right"] + 0.3, 1.0)))
        if not all_images_ready:
            overall_status = "INITIALIZING"
            overall_color = (0, 165, 255)
            self.danger_latched = False
            self.stop_sent_for_current_danger = False
        else:
            if fused_p_danger >= fused_p_safe:
                overall_status = "DANGER"
            else:
                overall_status = (
                    "SAFE" if fused_p_safe >= self.safe_confidence_threshold else "DANGER"
                )

            overall_color = (0, 0, 255) if overall_status == "DANGER" else (0, 255, 0)
            self.danger_latched = overall_status == "DANGER"
            if not self.danger_latched:
                self.stop_sent_for_current_danger = False

            if self.danger_latched:
                self.publish_supervisor_stop("visual danger threshold exceeded", fused_p_danger)

        self.publish_status(overall_status, fused_p_safe, fused_p_danger)

        mosaic = cv2.hconcat(panels)
        overall_text = f"FUSED: P_danger={fused_p_danger:.2f} -> {overall_status}"
        cv2.putText(
            mosaic,
            overall_text,
            (20, 120),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            overall_color,
            4,
            cv2.LINE_AA,
        )

        preview_msg = self.bridge.cv2_to_imgmsg(mosaic, encoding="bgr8")
        preview_msg.header.stamp = self.get_clock().now().to_msg()
        preview_msg.header.frame_id = "visual_safety_mosaic"
        self.preview_pub.publish(preview_msg)

        if self.display_window:
            cv2.imshow(self.window_name, mosaic)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyClassifier3CamsFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
