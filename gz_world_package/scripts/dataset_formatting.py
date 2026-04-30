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
        self.left_topic = "/left/kinect/image_raw"
        self.right_topic = "/right/kinect/image_raw"

        # --- Папки сохранения ---
        self.base_dir = os.path.expanduser("~/camera_images")
        self.left_dir = os.path.join(self.base_dir, "left_camera")
        self.right_dir = os.path.join(self.base_dir, "right_camera")

        os.makedirs(self.left_dir, exist_ok=True)
        os.makedirs(self.right_dir, exist_ok=True)

        # --- Последние кадры ---
        self.left_frame = None
        self.right_frame = None
        self.left_stamp = None
        self.right_stamp = None

        # --- Подписки ---
        self.create_subscription(Image, self.left_topic, self.left_cb, 10)
        self.create_subscription(Image, self.right_topic, self.right_cb, 10)

        # --- Таймер: 1 Гц ---
        self.timer = self.create_timer(1.0, self.save_images)

        self.get_logger().info("ImageSaver2Cams started. Saving images every 1 second.")

    def left_cb(self, msg: Image):
        self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.left_stamp = msg.header.stamp

    def right_cb(self, msg: Image):
        self.right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.right_stamp = msg.header.stamp

    def save_images(self):
        # --- LEFT ---
        if self.left_frame is not None and self.left_stamp is not None:
            ts = f"{self.left_stamp.sec}_{self.left_stamp.nanosec:09d}"
            filename = os.path.join(self.left_dir, f"left_{ts}.png")
            cv2.imwrite(filename, self.left_frame)
            self.get_logger().info(f"Saved LEFT image: {filename}")

        # --- RIGHT ---
        if self.right_frame is not None and self.right_stamp is not None:
            ts = f"{self.right_stamp.sec}_{self.right_stamp.nanosec:09d}"
            filename = os.path.join(self.right_dir, f"right_{ts}.png")
            cv2.imwrite(filename, self.right_frame)
            self.get_logger().info(f"Saved RIGHT image: {filename}")


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
