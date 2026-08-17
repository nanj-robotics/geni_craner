#!/usr/bin/env python3
"""
棋盘格标定板检测节点
- 订阅 /camera/color/image_raw 和 /camera/color/camera_info
- 检测 9x7 内角点棋盘格（方格 15mm）
- 发布 camera_color_optical_frame -> calibration_board 的 TF
- 发布可视化图像 /chessboard_detection/image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import cv2
import numpy as np

class ChessboardDetector(Node):
    def __init__(self):
        super().__init__('chessboard_detector')

        # 棋盘格参数：9x7 内角点，方格 15mm = 0.015m
        self.pattern_size = (9, 7)
        self.square_size = 0.015

        # 生成棋盘格 3D 点（单位：米）
        self.object_points = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.object_points[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        self.object_points *= self.square_size

        # 相机内参（从 camera_info 获取）
        self.camera_matrix = None
        self.dist_coeffs = None

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)

        # 发布可视化图像
        self.image_pub = self.create_publisher(Image, '/chessboard_detection/image', 10)

        self.get_logger().info('棋盘格检测节点已启动')
        self.get_logger().info(f'棋盘格：{self.pattern_size[0]}x{self.pattern_size[1]} 内角点，方格 {self.square_size*1000}mm')

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('已获取相机内参')

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 检测棋盘格角点
        found, corners = cv2.findChessboardCorners(
            gray, self.pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )

        if found:
            # 亚像素精化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # solvePnP 求位姿
            success, rvec, tvec = cv2.solvePnP(
                self.object_points, corners, self.camera_matrix, self.dist_coeffs
            )

            if success:
                # 旋转向量转旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # 构建 4x4 变换矩阵
                transform = np.eye(4)
                transform[:3, :3] = rotation_matrix
                transform[:3, 3] = tvec.flatten()

                # 发布 TF：camera_color_optical_frame -> calibration_board
                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'camera_color_optical_frame'
                t.child_frame_id = 'calibration_board'

                t.transform.translation.x = float(transform[0, 3])
                t.transform.translation.y = float(transform[1, 3])
                t.transform.translation.z = float(transform[2, 3])

                # 旋转矩阵转四元数
                quat = self.rot_matrix_to_quat(rotation_matrix)
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.tf_broadcaster.sendTransform(t)

                # 绘制检测结果
                cv2.drawChessboardCorners(cv_image, self.pattern_size, corners, found)
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

        # 发布可视化图像
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))

    def rot_matrix_to_quat(self, R):
        """旋转矩阵转四元数 [x, y, z, w]"""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return [x, y, z, w]


def main():
    rclpy.init()
    node = ChessboardDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

