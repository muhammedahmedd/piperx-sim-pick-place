#!/usr/bin/env python3

import cv2
import rclpy
import cv2.aruco as aruco
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ArucoSimDetector(Node):
    def __init__(self):
        super().__init__("aruco_sim_detector")

        self.bridge = CvBridge()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

        self.subscription = self.create_subscription(
            Image,
            "/isaac/color_image_raw",
            self.image_callback,
            10
        )

        self.get_logger().info("Aruco sim detector started.")
        self.get_logger().info("Subscribing to /isaac/color_image_raw")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        debug_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.imwrite("/workspace/piperx_ws/debug_frame.png", debug_frame)

        corners, ids, rejected = aruco.detectMarkers(
            frame,
            self.aruco_dict
        )

        if ids is not None:
            detected_ids = ids.flatten().tolist()
            self.get_logger().info(f"Detected ArUco marker IDs: {detected_ids}")


def main(args=None):
    rclpy.init(args=args)

    node = ArucoSimDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()