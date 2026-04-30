#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch

class SafetyClassifier(Node):
    def __init__(self):
        super().__init__('safety_classifier')

        # Загружаем YOLOv11 классификатор
        # Укажи путь к своей модели:
        self.model = YOLO("/home/marat/exchange_project_ws/src/collab_industrial_research/gz_world_package/config/camera_models/final_left_best.pt")

        self.bridge = CvBridge()

        # Подписка на камеру
        self.sub = self.create_subscription(
            Image,
            "/left/kinect/image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("Safety classifier node started.")

    def image_callback(self, msg):
        # Преобразуем ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # --- Инференс YOLO ---
        results = self.model(frame, verbose=False)
        probs = results[0].probs

        # Индекс класса: 0=safe, 1=danger
        # class_id = int(torch.argmax(probs))
        # confidence = float(probs[class_id])

        class_id = int(probs.top1)          # индекс класса
        confidence = float(probs.top1conf)  # вероятность

        label = "SAFE" if class_id == 1 else "DANGER"
        color = (0, 255, 0) if class_id == 1 else (0, 0, 255)

        # --- Отрисовка поверх изображения ---
        text = f"{label}  {confidence:.2f}"
        cv2.putText(frame, text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3, cv2.LINE_AA)

        # Показать окно
        cv2.imshow("Safety Monitor", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SafetyClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
