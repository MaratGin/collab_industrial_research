#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

class DualBaslerViewer(Node):
    def __init__(self):
        super().__init__("dual_basler_viewer")

        self.bridge = CvBridge()

        self.ur_sub = self.create_subscription(
            Image,
            "/ur_cam_id/ur_cam_node/image_raw",
            self.ur_image_callback,
            10
        )

        self.kuka_sub = self.create_subscription(
            Image,
            "/second_cam_id/second_cam_node/image_raw",
            self.kuka_image_callback,
            10
        )

        self.get_logger().info("Dual Basler viewer started")
        self.get_logger().info("Listening to /ur_camera/image_raw")
        self.get_logger().info("Listening to /kuka_camera/image_raw")

    def ur_image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("UR Basler Camera", frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"UR camera conversion error: {e}")

    def kuka_image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("KUKA Basler Camera", frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error(f"KUKA camera conversion error: {e}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = DualBaslerViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping dual camera viewer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()