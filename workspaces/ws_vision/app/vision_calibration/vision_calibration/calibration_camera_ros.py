"""
@Descripttion: 内参标定的ROS接口
@version: 1.0
@Author: 崔译文
@Date: 2024-03-21 14:01:53
@LastEditors: 崔译文
@LastEditTime: 2024-04-02 10:15:21
"""

import rclpy
from rclpy.node import Node
from .calibration_camera import CameraCalibration


class CalibrationCameraInterface(Node):
    def __init__(self):
        super().__init__("calibration_camera_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("board_config_file", rclpy.Parameter.Type.STRING),
                ("image_folder", rclpy.Parameter.Type.STRING),
                ("visibilization", rclpy.Parameter.Type.BOOL),
                ("output_dir", rclpy.Parameter.Type.STRING),
            ],
        )
        self.board_config_file = self.get_parameter("board_config_file").value
        self.image_folder = self.get_parameter("image_folder").value
        self.visibilization = self.get_parameter("visibilization").value
        self.output_dir = self.get_parameter("output_dir").value
        self.get_logger().info(f"board_config_file: {self.board_config_file}")
        self.get_logger().info(f"image_folder: {self.image_folder}")
        self.get_logger().info(f"visibilization: {self.visibilization}")
        self.get_logger().info(f"output_dir: {self.output_dir}")
        self.camera_calibrator = CameraCalibration(self.board_config_file, self.image_folder, self.output_dir)

    def calibration(self):
        self.camera_calibrator.calibrate(self.visibilization)


def main(args=None):
    rclpy.init(args=args)
    cali = CalibrationCameraInterface()
    cali.calibration()
    cali.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
