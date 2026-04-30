#!/usr/bin/env python3
import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class SafetyClassifier3CamsFusion(Node):
    """
    3 камеры + 3 модели классификации safe/danger.
    - Подписка на 3 image_raw
    - Инференс по таймеру
    - Одно OpenCV окно: три изображения + оверлеи + итоговая fused-оценка
    - Итоговая оценка по формуле:
        P = 1 - (1 - p_top)*(1 - p_left)*(1 - p_right)
      где p_cam = P(danger) для каждой камеры
    - ROS time в левом нижнем углу каждого изображения
    """

    def __init__(self):
        super().__init__('safety_classifier_3cams_fusion')
        self.bridge = CvBridge()

        # --- Настройки: топики + пути к моделям ---
        # !!! Поменяй model_path на свои реальные файлы best.pt для каждой камеры
        self.cams = {
            "top": {
                "topic": "/top/kinect/image_raw",
                "model_path": "/home/marat/exchange_project_ws/src/collab_industrial_research/gz_world_package/config/camera_models/final_top_best.pt",
                "model": None,
                "latest_frame": None,
                "latest_stamp": None,  # (sec, nanosec)
                "last_pdanger": 0.0,
                "last_psafe": 0.0,
            },
            "left": {
                "topic": "/left/kinect/image_raw",
                "model_path": "/home/marat/exchange_project_ws/src/collab_industrial_research/gz_world_package/config/camera_models/final_left_best.pt",
                "model": None,
                "latest_frame": None,
                "latest_stamp": None,
                "last_pdanger": 0.0,
                "last_psafe": 0.0,
            },
            "right": {
                "topic": "/right/kinect/image_raw",
                "model_path": "/home/marat/exchange_project_ws/src/collab_industrial_research/gz_world_package/config/camera_models/final_right_best.pt",
                "model": None,
                "latest_frame": None,
                "latest_stamp": None,
                "last_pdanger": 0.0,
                "last_psafe": 0.0,
            },
        }

        # --- Индексы классов (ВАЖНО) ---
        # Предполагаем: 0 = safe, 1 = danger
        self.safe_id = 1
        self.danger_id = 0
        self.T1 = 0.87

        # Порог для отображения итогового статуса (не влияет на формулу, только на "SAFE/DANGER" текст)
        self.danger_threshold = 0.7

        # --- Загружаем модели ---
        for cam_name, cfg in self.cams.items():
            cfg["model"] = YOLO(cfg["model_path"])
            self.get_logger().info(f"Loaded model for {cam_name}: {cfg['model_path']}")

        # --- Подписки ---
        for cam_name, cfg in self.cams.items():
            self.create_subscription(
                Image,
                cfg["topic"],
                lambda msg, cn=cam_name: self.image_cb(msg, cn),
                10
            )
            self.get_logger().info(f"Subscribed: {cam_name} -> {cfg['topic']}")

        # --- Таймер инференса ---
        self.infer_period_s = 0.2  # 5 Гц; если тяжело на CPU — поставь 0.3-0.5
        self.timer = self.create_timer(self.infer_period_s, self.infer_and_draw)

        self.window_name = "Safety Monitor (3 cams, fused)"
        self.get_logger().info("SafetyClassifier3CamsFusion started.")

    def image_cb(self, msg: Image, cam_name: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.cams[cam_name]["latest_frame"] = frame
            self.cams[cam_name]["latest_stamp"] = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        except Exception as e:
            self.get_logger().error(f"[{cam_name}] cv_bridge error: {e}")

    @staticmethod
    def _format_ros_time(stamp_tuple):
        if stamp_tuple is None:
            return "rostime: N/A"
        sec, nsec = stamp_tuple
        return f"rostime: {sec}.{nsec:09d}"

    def classify_pdanger_psafe(self, model: YOLO, frame):
        """
        Возвращает p_danger = P(class=danger) из классификатора Ultralytics.
        """
        results = model(frame, verbose=False)
        probs = results[0].probs
        if probs is None or probs.data is None:
            return 0.0, 0.0

        # probs.data обычно torch.Tensor вида [p_safe, p_danger]
        try:
            p_danger = float(probs.data[self.danger_id])
            p_safe = float(probs.data[self.safe_id])
        except Exception:
            # на всякий случай fallback
            top = int(probs.top1)
            conf = float(probs.top1conf)
            if top == self.safe_id:
                p_safe, p_danger = conf, 1 - conf
            else:
                p_danger, p_safe = conf, 1 - conf

        # ограничим на [0..1]
        p_safe = max(0.0, min(1.0, p_safe))
        p_danger = max(0.0, min(1.0, p_danger))

        return p_safe, p_danger

    @staticmethod
    def fuse_3cams(p_top, p_left, p_right):
        """
        P(H_i)=1-(1-p_top)(1-p_left)(1-p_right)
        """
        return 1.0 - (1.0 - p_top) * (1.0 - p_left) * (1.0 - p_right)


    def infer_and_draw(self):
        panels = []

        # Будем хранить p_danger и p_safe для формулы
        p_danger = {"top": 0.0, "left": 0.0, "right": 0.0}
        p_safe = {"top": 0.0, "left": 0.0, "right": 0.0}

        for cam_name in ["top", "left", "right"]:
            cfg = self.cams[cam_name]
            frame = cfg["latest_frame"]

            # если нет кадра — рисуем заглушку
            if frame is None:
                blank = np.full((360, 640, 3), 255, dtype=np.uint8)
                cv2.putText(blank, f"{cam_name.upper()}: NO IMAGE", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3, cv2.LINE_AA)
                # rostime
                rt = self._format_ros_time(cfg["latest_stamp"])
                cv2.putText(blank, rt, (20, 340),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (50, 50, 50), 2, cv2.LINE_AA)
                panels.append(blank)
                p_safe[cam_name] = 0.0
                p_danger[cam_name] = 0.0

                continue

            # инференс
            # if cam_name == "top":
            #     pdanger = 0.22
            # elif cam_name == "left":
            #     pdanger = 0.13
            # elif cam_name == "right":
            #     pdanger = 0.11

            psafe, pdanger = self.classify_pdanger_psafe(cfg["model"], frame)
            cfg["last_pdanger"] = pdanger
            cfg["last_psafe"] = psafe
            p_safe[cam_name] = psafe
            p_danger[cam_name] = pdanger

            # Визуализация
            vis = frame.copy()
            vis = cv2.resize(vis, (640, 360))

            # локальный статус для этой камеры
            cam_status = "DANGER" if pdanger >= self.danger_threshold else "SAFE"
            cam_color = (0, 0, 255) if cam_status == "DANGER" else (0, 255, 0)

            # header_first = f"{cam_name.upper()} | p_safe={psafe:.2f} p_danger={pdanger:.2f} -> {cam_status}"
            # header_second = f"{cam_status}"
            header_first = f"{cam_name.upper()} | p_danger={pdanger:.2f} -> {cam_status}"
            cv2.putText(vis, header_first, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, cam_color, 3, cv2.LINE_AA)
            # cv2.putText(vis, header_second, (20, 80),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.7, cam_color, 3, cv2.LINE_AA)
            # rostime в левом нижнем углу
            rt = self._format_ros_time(cfg["latest_stamp"])
            cv2.putText(vis, rt, (20, 340),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, rt, (20, 340),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)

            panels.append(vis)

        # --- Fusion по твоей формуле ---
        P_safe = self.fuse_3cams(p_safe["top"], p_safe["left"], p_safe["right"] )
        P_danger = self.fuse_3cams(p_danger["top"], p_danger["left"], p_danger["right"] )
        if P_danger >= P_safe:
            overall_status = "DANGER"
            P_final = P_danger
        else:
            overall_status = "SAFE" if (P_safe >= self.T1) else "DANGER"
            P_final = P_safe
        overall_color = (0, 0, 255) if overall_status == "DANGER" else (0, 255, 0)

        # Склеиваем в одну строку: top | left | right
        mosaic = cv2.hconcat(panels)

        # Итоговый текст
        ""
        overall_text_first = (
            # f"FUSED: P_safe={P_safe:.2f}"
            f"FUSED: P_danger={P_danger:.2f} -> {overall_status}"
        )
        overall_text_second = (
            f"FUSED: P_danger={P_danger:.2f}"
        )
        overall_text_third = (
            f"FINAL STATUS -> {overall_status} "
        )
        cv2.putText(mosaic, overall_text_first, (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, overall_color, 4, cv2.LINE_AA)
        # cv2.putText(mosaic, overall_text_second, (20, 135),
        #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, overall_color, 4, cv2.LINE_AA)
        # cv2.putText(mosaic, overall_text_third, (20, 170),
        #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, overall_color, 4, cv2.LINE_AA)

        # Отдельно выводим p каждой камеры
        # detail = f"p_top={p['top']:.2f}   p_left={p['left']:.2f}   p_right={p['right']:.2f}"
        # cv2.putText(mosaic, detail, (20, 130),
        #             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (30, 30, 30), 3, cv2.LINE_AA)

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
