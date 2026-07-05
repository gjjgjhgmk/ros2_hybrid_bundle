"""
@Descripttion: 手眼标定的ROS接口
@version: 1.0
@Author: 崔译文
@Date: 2024-03-21 14:01:53
@LastEditors: 崔译文
@LastEditTime: 2024-04-02 10:50:13
"""

import json
import rclpy
from rclpy.node import Node
from .calibration_handeye import CalibrationHandEye
import numpy as np
import transforms3d as tfs


class CalibrationHandeyeInterface(Node):
    def __init__(self):
        super().__init__("calibration_handeye_node")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("pose_file", rclpy.Parameter.Type.STRING),
                ("calibration_method", rclpy.Parameter.Type.STRING),
                ("calibration_type", rclpy.Parameter.Type.STRING),
            ],
        )
        self.pose_file = self.get_parameter("pose_file").value
        self.calibration_method = self.get_parameter("calibration_method").value
        self.calibration_type = self.get_parameter("calibration_type").value
        self.handeye_calibrator = CalibrationHandEye()

    def calibration(self):
        f = open(self.pose_file)
        data = json.load(f)
        pose_gripperinbase = data["gripperinbase"]
        pose_boardincamera = data["boardincamera"]
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []
        for pose in pose_gripperinbase:
            x, y, z, qx, qy, qz, qw = pose[:7]
            t_gripper2base.append(np.array([[x], [y], [z]]))
            R_gripper2base.append(tfs.quaternions.quat2mat([qw, qx, qy, qz]))
        for pose in pose_boardincamera:
            x, y, z, qx, qy, qz, qw = pose[:7]
            t_target2cam.append(np.array([[x], [y], [z]]))
            R_target2cam.append(tfs.quaternions.quat2mat([qw, qx, qy, qz]))
        R, t = self.handeye_calibrator.calibration(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            cali_type=self.calibration_type,
            method=self.calibration_method,
        )
        x, y, z = t.flatten()[:3]
        qw, qx, qy, qz = tfs.quaternions.mat2quat(R)[:4]
        res = [x, y, z, qx, qy, qz, qw]
        print("设定方法 {} 的手眼标定结果为".format(self.calibration_method), res)
        print("\n其他方法的标定结果如下")
        for key in self.handeye_calibrator.AVAILABLE_ALGORITHMS.keys():
            if key != self.calibration_method:
                R, t = self.handeye_calibrator.calibration(
                    R_gripper2base=R_gripper2base,
                    t_gripper2base=t_gripper2base,
                    R_target2cam=R_target2cam,
                    t_target2cam=t_target2cam,
                    cali_type=self.calibration_type,
                    method=key,
                )
                x, y, z = t.flatten()[:3]
                qw, qx, qy, qz = tfs.quaternions.mat2quat(R)[:4]
                print("\t", key, [x, y, z, qx, qy, qz, qw])
        return res


def main(args=None):
    rclpy.init(args=args)
    cali = CalibrationHandeyeInterface()
    cali.calibration()
    cali.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
