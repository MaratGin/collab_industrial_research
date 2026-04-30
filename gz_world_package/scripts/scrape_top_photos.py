#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSaver2Cams(Node):
    def __init__(self):
        super().__init__('image_saver_2cams')

        self.bridge = CvBridge()

        # --- Топики камер ---
        self.top_topic = "/top/kinect/image_raw"

        # --- Папки сохранения ---
        self.base_dir = os.path.expanduser("~/camera_images")
        self.top_dir = os.path.join(self.base_dir, "top_camera")

        os.makedirs(self.top_dir, exist_ok=True)

        # --- Последние кадры ---
        self.top_frame = None
        self.top_stamp = None

        # --- Подписки ---
        self.create_subscription(Image, self.top_topic, self.top_cb, 10)

        # --- Таймер: 1 Гц ---
        self.timer = self.create_timer(2, self.save_images)

        self.get_logger().info("ImageSaver2Cams started. Saving images every 1 second.")

    def top_cb(self, msg: Image):
        self.top_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.top_stamp = msg.header.stamp



    def save_images(self):
        # --- top ---
        if self.top_frame is not None and self.top_stamp is not None:
            ts = f"{self.top_stamp.sec}_{self.top_stamp.nanosec:09d}"
            filename = os.path.join(self.top_dir, f"NEW1_human_top_{ts}.png")
            cv2.imwrite(filename, self.top_frame)
            self.get_logger().info(f"Saved top image: {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver2Cams()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()