#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定位姿生成与控制程序

该程序用于生成标定所需的相机位姿，并控制机械臂移动到对应位置。
流程：
1. 在指定位置范围和RPY范围内生成相机位姿（基于robot_base）
2. 根据相机位姿计算end_effector位姿（基于robot_base）
3. 使用MoveRobotClient控制机械臂移动到end_effector位姿
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation as R
from .zmq_ur_move_client import UrMoveClient as MoveRobotClient

# from .move_robot_client import MoveRobotClient as MoveRobotClient
from .ros2_tf_subscriber import ROS2TFSubscriber


class CalibrationSampler:
    """标定采样器"""

    def __init__(
        self,
        use_sim_time: bool = True,
        service_name: str = "example_move_robot",
        tf_subscriber: ROS2TFSubscriber = None,
        server_address: str = "tcp://localhost:5605",
        timeout_ms: int = 60000,
        ###################################
        left_arm_executor_host: str = None, 
        right_arm_executor_host: str = "192.168.56.122",
    ):
        """
        初始化标定位姿控制器

        Args:
            use_sim_time: 是否使用仿真时间
            service_name: MoveRobot服务名称
            tf_subscriber: TF订阅器
            server_address: ur_move服务器地址
            timeout_ms: 请求超时时间(毫秒)
        """
        # 检查 rclpy 是否已初始化，避免重复初始化
        if not rclpy.ok():
            rclpy.init()

        # 初始化MoveRobotClient
        ###########################################
        self.move_client = MoveRobotClient(server_address=server_address,
            timeout_ms=timeout_ms,
            left_arm_executor_host=left_arm_executor_host,
            right_arm_executor_host=right_arm_executor_host)
        # self.move_client = MoveRobotClient(service_name=service_name)

        if tf_subscriber is None:
            # 初始化TF订阅器（用于查询变换）
            self.tf_subscriber = ROS2TFSubscriber(use_sim_time=use_sim_time)
        else:
            self.tf_subscriber = tf_subscriber

        # 相机到end_effector的固定变换（如果已知，否则通过TF查询）
        self.camera_to_end_effector_transform = None

        print(f"标定位姿控制器初始化完成")

    def generate_random_poses(
        self,
        base_pose: list[float],
        position_range: list[list[float]],
        rpy_range: list[list[float]],
        num_poses: int = 10,
    ):
        """
        在指定范围内生成随机位姿

        Args:
            base_pose: 基础位姿
            position_range: 位置范围 [[x_min, x_max], [y_min, y_max], [z_min, z_max]]
            rpy_range: RPY角度范围 [[roll_min, roll_max], [pitch_min, pitch_max], [yaw_min, yaw_max]]（弧度）
            num_poses: 生成的随机位姿数量

        Returns:
            list: 生成的位姿列表
        """
        poses = []

        # 将基础位姿转换为numpy数组
        base_pos = np.array([base_pose[0], base_pose[1], base_pose[2]])
        base_rpy = np.array([0, 0, 0])
        if len(base_pose) == 6:
            base_rpy = np.array([base_pose[3], base_pose[4], base_pose[5]])
        elif len(base_pose) == 7:
            base_rpy = R.from_quat(np.array([base_pose[3], base_pose[4], base_pose[5], base_pose[6]])).as_euler("xyz")

        # 生成随机位姿
        np.random.seed(int(time.time()))

        for i in range(num_poses):
            # 生成随机位置偏移
            x_offset = np.random.uniform(position_range[0][0], position_range[0][1])
            y_offset = np.random.uniform(position_range[1][0], position_range[1][1])
            z_offset = np.random.uniform(position_range[2][0], position_range[2][1])

            # 生成随机RPY偏移
            roll_offset = np.random.uniform(rpy_range[0][0], rpy_range[0][1])
            pitch_offset = np.random.uniform(rpy_range[1][0], rpy_range[1][1])
            yaw_offset = np.random.uniform(rpy_range[2][0], rpy_range[2][1])

            # 计算新位姿
            new_pos = base_pos + np.array([x_offset, y_offset, z_offset])
            new_rpy = base_rpy + np.array([roll_offset, pitch_offset, yaw_offset])
            new_rot = R.from_euler("xyz", new_rpy)
            new_quat = new_rot.as_quat()  # [x, y, z, w]

            # 创建Pose消息
            pose = [
                float(new_pos[0]),
                float(new_pos[1]),
                float(new_pos[2]),
                float(new_quat[0]),
                float(new_quat[1]),
                float(new_quat[2]),
                float(new_quat[3]),
            ]
            poses.append(pose)

        return poses

    def apply_transform(self, pose: list[float], transform: list[float]) -> list[float]:
        """
        应用变换，求取新位姿

        计算 pose 经过 transform 变换后的新位姿。
        使用齐次变换矩阵：T_new = T_transform @ T_pose

        Args:
            pose: 原始位姿字典，包含 position 和 orientation
            transform: 变换字典，包含 translation 和 rotation

        Returns:
            dict: 变换后的新位姿字典
        """
        # 将 pose 转换为齐次变换矩阵
        pose_pos = np.array([pose[0], pose[1], pose[2]])
        pose_quat = np.array([pose[3], pose[4], pose[5], pose[6]])
        pose_rot = R.from_quat(pose_quat)
        pose_R = pose_rot.as_matrix()

        # 构建 pose 的齐次变换矩阵
        T_pose = np.eye(4)
        T_pose[:3, :3] = pose_R
        T_pose[:3, 3] = pose_pos

        # 将 transform 转换为齐次变换矩阵
        transform_pos = np.array([transform[0], transform[1], transform[2]])
        transform_quat = np.array([transform[3], transform[4], transform[5], transform[6]])
        transform_rot = R.from_quat(transform_quat)
        transform_R = transform_rot.as_matrix()

        # 构建 transform 的齐次变换矩阵
        T_transform = np.eye(4)
        T_transform[:3, :3] = transform_R
        T_transform[:3, 3] = transform_pos

        # 计算新位姿：T_new = T_transform @ T_pose
        T_new = T_transform @ T_pose

        # 从齐次变换矩阵中提取新的位置和旋转
        new_pos = T_new[:3, 3]
        new_R = T_new[:3, :3]
        new_rot = R.from_matrix(new_R)
        new_quat = new_rot.as_quat()  # [x, y, z, w]

        return [
            float(new_pos[0]),
            float(new_pos[1]),
            float(new_pos[2]),
            float(new_quat[0]),
            float(new_quat[1]),
            float(new_quat[2]),
            float(new_quat[3]),
        ]

    def move_to_pose(
        self,
        pose: list[float],
        group_name: str,
        execute: bool = True,
        timeout_sec: float = 120.0,
        current_frame: str = "",
        target_frame: str = "",
    ):
        """
        控制机械臂移动到指定的相机位姿

        Args:
            pose: 目标位姿（基于 child_frame）
            execute: 是否执行运动
            timeout_sec: 服务调用超时时间（秒）
            current_frame: 当前坐标系
            target_frame: 目标坐标系
        Returns:
            bool: 是否成功
        """
        target_pose = Pose()
        target_pose.position.x = pose[0]
        target_pose.position.y = pose[1]
        target_pose.position.z = pose[2]
        target_pose.orientation.x = pose[3]
        target_pose.orientation.y = pose[4]
        target_pose.orientation.z = pose[5]
        target_pose.orientation.w = pose[6]
        if current_frame != "" and target_frame != "":
            target_pose = self.tf_subscriber.transform_pose(
                target_pose, child_frame=current_frame, parent_frame=target_frame
            )


        response = self.move_client.call_service(
            group_name=group_name,
            planning_mode=1,  # 目标规划模式
            execute=execute,
            target_pose=target_pose,  # 直接使用robot_base中的位姿
            timeout_sec=timeout_sec,
        )



        if response is None:
            print("错误: 服务调用失败")
            return False

        if response.planned:
            if execute:
                if response.success:
                    print("✓ 规划成功，执行成功")
                    return True
                else:
                    print("✗ 规划成功，但执行失败")
                    return False
            else:
                print("✓ 规划成功（未执行）")
                return True
        else:
            print("✗ 规划失败")
            return False

    def plan_and_execute_via_joint_values(
        self,
        group: str,
        planner: str,
        type: str,
        joint_names: list[str] = None,
        joint_values: list[float] = None,
        max_velocity_scaling_factor: float = 0.3,
        max_acceleration_scaling_factor: float = 0.3,
        description: str = "关节空间路径点",
    ) -> list[bool]:
        """
        规划并执行轨迹

        Args:
            group: 机械臂组名称
            planner: 规划器名称
            type: 路径点类型
            joint_names: 关节名称列表
            joint_values: 关节值列表
            max_velocity_scaling_factor: 最大速度缩放因子 (0.0-1.0)
            max_acceleration_scaling_factor: 最大加速度缩放因子 (0.0-1.0)
            description: 描述
        Returns:
            list: 执行结果列表
        """
        waypoints = {
            "关节点位": {
                "group": group,
                "planner": planner,
                "type": type,
                "joint_names": joint_names,
                "joint_values": joint_values,
                "max_velocity_scaling_factor": max_velocity_scaling_factor,
                "max_acceleration_scaling_factor": max_acceleration_scaling_factor,
                "description": description,
            }
        }
        return self.move_client.plan_and_execute_remote(waypoints)

    def plan_and_execute_via_pose(
        self,
        group: str,
        planner: str,
        type: str,
        ik_frame: str,
        frame_id: str,
        position: list[float] = None,
        orientation: list[float] = None,
        max_velocity_scaling_factor: float = 0.3,
        max_acceleration_scaling_factor: float = 0.3,
        description: str = "笛卡尔空间路径点",
    ) -> list[bool]:
        """
        规划并执行笛卡尔空间路径点

        Args:
            group: 机械臂组名称
            planner: 规划器名称
            type: 路径点类型
            ik_frame: IK坐标系
            frame_id: 目标坐标系
            position: 位置列表
            orientation: 方向列表
            max_velocity_scaling_factor: 最大速度缩放因子
            max_acceleration_scaling_factor: 最大加速度缩放因子
            description: 描述

        Returns:
            list: 执行结果列表
        """
        waypoints = {
            "笛卡尔点位": {
                "group": group,
                "planner": planner,
                "type": type,
                "ik_frame": ik_frame,
                "frame_id": frame_id,
                "position": position,
                "orientation": orientation,
                "max_velocity_scaling_factor": max_velocity_scaling_factor,
                "max_acceleration_scaling_factor": max_acceleration_scaling_factor,
                "description": description,
            }
        }
        return self.move_client.plan_and_execute_remote(waypoints)
