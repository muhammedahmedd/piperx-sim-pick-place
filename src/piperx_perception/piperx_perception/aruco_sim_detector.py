#!/usr/bin/env python3

import cv2
import rclpy
import tf2_ros
import numpy as np
import cv2.aruco as aruco

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from rclpy.duration import Duration
from tf_transformations import quaternion_matrix, quaternion_from_matrix


class ArucoSimDetector(Node):
    def __init__(self):
        super().__init__("aruco_sim_detector")

        self.bridge = CvBridge()

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Physical marker side length in meters
        self.marker_size = 0.04

        self.camera_matrix = None
        self.dist_coeffs = None
        self.printed_camera_info = False
    
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/isaac/camera_info",
            self.camera_info_callback,
            10
        )

        self.image_sub = self.create_subscription(
            Image,
            "/isaac/rgb_raw",
            self.image_callback,
            1
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/aruco/marker_pose",
            10
        )

        self.pose_base_pub = self.create_publisher(
            PoseStamped,
            "/aruco/marker_pose_base",
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        self.get_logger().info("Aruco sim detector started.")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)

        if not self.printed_camera_info:
            self.get_logger().info(
                f"Received camera intrinsics:\n{self.camera_matrix}"
            )
        self.printed_camera_info = True

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for /isaac/camera_info...")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        # debug_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # cv2.imwrite("/workspace/piperx_ws/debug_frame.png", debug_frame)

        corners, ids, rejected = aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None:
            detected_ids = ids.flatten().tolist()
            self.get_logger().info(f"Detected ArUco marker IDs: {detected_ids}")
        else:
            return
        
        
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_size,
            self.camera_matrix,
            self.dist_coeffs
        )

        marker_id = detected_ids[0]
        rvec = rvecs[0][0]
        tvec = tvecs[0][0]

        self.publish_marker_pose(marker_id, rvec, tvec, msg)
        self.publish_marker_pose_base(marker_id, rvec, tvec, msg)

    def publish_marker_pose(self, marker_id, rvec, tvec, image_msg):
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = Rotation.from_matrix(rotation_matrix).as_quat()

        pose_msg = PoseStamped()

        pose_msg.header.stamp = image_msg.header.stamp
        pose_msg.header.frame_id = image_msg.header.frame_id

        pose_msg.pose.position.x = tvec[0]
        pose_msg.pose.position.y = tvec[1]
        pose_msg.pose.position.z = tvec[2]

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_pub.publish(pose_msg)

        self.get_logger().info(
            f"Published marker {marker_id} pose to /aruco/marker_pose"
        )

    def publish_marker_pose_base(self, marker_id, rvec, tvec, image_msg):
        try:
            image_time = Time.from_msg(image_msg.header.stamp)

            tf_base_camera = self.tf_buffer.lookup_transform(
                "base_link",
                "Camera",
                image_time,
                timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not transform base_link to Camera: {e}")
            return
        
        # T_base_camera
        q_base_camera = tf_base_camera.transform.rotation
        t_base_camera = tf_base_camera.transform.translation

        T_base_camera = quaternion_matrix([
            q_base_camera.x,
            q_base_camera.y,
            q_base_camera.z,
            q_base_camera.w
        ])

        T_base_camera[0, 3] = t_base_camera.x
        T_base_camera[1, 3] = t_base_camera.y
        T_base_camera[2, 3] = t_base_camera.z

        # T_optical_marker
        R_optical_marker, _ = cv2.Rodrigues(rvec)
        
        T_optical_marker = np.eye(4)
        T_optical_marker[0:3, 0:3] = R_optical_marker
        T_optical_marker[0, 3] = tvec[0]
        T_optical_marker[1, 3] = tvec[1]
        T_optical_marker[2, 3] = tvec[2]

        # mapping the marker pose from the camera frame into the base_link frame
        T_base_marker = T_base_camera @ T_optical_marker

        quat_base_marker = quaternion_from_matrix(T_base_marker)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = image_msg.header.stamp
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = T_base_marker[0, 3]
        pose_msg.pose.position.y = T_base_marker[1, 3]
        pose_msg.pose.position.z = T_base_marker[2, 3]

        pose_msg.pose.orientation.x = quat_base_marker[0]
        pose_msg.pose.orientation.y = quat_base_marker[1]
        pose_msg.pose.orientation.z = quat_base_marker[2]
        pose_msg.pose.orientation.w = quat_base_marker[3]

        self.pose_base_pub.publish(pose_msg)

        self.get_logger().info(
            f"Published marker {marker_id} pose to /aruco/marker_pose_base"
        )


def main(args=None):
    rclpy.init(args=args)

    node = ArucoSimDetector()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()