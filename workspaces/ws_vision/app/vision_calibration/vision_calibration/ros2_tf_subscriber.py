#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 TF订阅器模块

提供ROS2 TF转换订阅功能，用于获取机器人位姿数据
使用独立线程执行spin，无需外部手动调用spin
"""

import time
import copy
import threading
from typing import Optional, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as SciPyRotation

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Pose, PoseStamped
import tf2_geometry_msgs  # 注册 Pose/PoseStamped 转换支持


class ROS2TFSubscriber(Node):
    """ROS2 TF订阅器 - 支持订阅任意TF转换，使用独立线程执行spin"""

    def __init__(self, use_sim_time: bool = False):
        """
        初始化TF订阅器

        Args:
            use_sim_time: 是否使用仿真时间
        """
        super().__init__("calibration_tf_listener")

        # 允许外部通过 use_sim_time 切换实时时钟/仿真时钟，兼容 real/sim
        # 避免重复声明导致 Parameter already declared
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", use_sim_time)
        # 若外部未设定，与期望值不一致时，按代码入参强制覆盖，确保行为确定
        if self.get_parameter("use_sim_time").get_parameter_value().bool_value != use_sim_time:
            self.set_parameters(
                [
                    Parameter(
                        name="use_sim_time",
                        value=use_sim_time,
                        type_=Parameter.Type.BOOL,
                    )
                ]
            )
        if use_sim_time:
            self.get_logger().info("使用仿真时间 (use_sim_time=true)")

        # 将 Buffer 绑定节点时钟，确保 TF 查询使用与节点一致的 clock
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 创建独立的 Executor 和 spin 线程
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = None
        self._spin_running = False
        self._start_spin_thread()

        self.get_logger().info("TF订阅器初始化（支持任意TF转换，兼容实时时间与仿真时间）")

    def _start_spin_thread(self):
        """启动独立的spin线程"""
        if self._spin_thread is None or not self._spin_thread.is_alive():
            self._spin_running = True
            self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
            self._spin_thread.start()
            self.get_logger().info("Spin线程已启动")

    def _spin_loop(self):
        """Spin循环，在独立线程中运行，使用独立的 Executor"""
        try:
            # 使用 Executor 的 spin 方法，避免 wait set 索引问题
            while self._spin_running and rclpy.ok():
                try:
                    # 使用 Executor 的 spin_once，超时时间较短
                    self._executor.spin_once(timeout_sec=0.1)
                except (rclpy.exceptions.ROSInterruptException, KeyboardInterrupt):
                    # ROS2 中断，正常退出
                    self.get_logger().info("Spin循环收到中断信号，退出")
                    break
                except RuntimeError as e:
                    # 捕获 RuntimeError，特别是 "wait set index too big" 错误
                    error_msg = str(e).lower()
                    if "wait set" in error_msg or "index" in error_msg:
                        self.get_logger().warning(f"Spin循环检测到 Executor 状态异常，退出: {e}")
                        break
                    else:
                        # 其他 RuntimeError，记录但继续
                        self.get_logger().warning(f"Spin循环 RuntimeError: {e}")
                        time.sleep(0.1)
        except Exception as e:
            error_msg = str(e).lower()
            if "destroyed" in error_msg or "shutdown" in error_msg:
                self.get_logger().info(f"Executor 已关闭，退出spin循环")
            else:
                self.get_logger().error(f"Spin循环发生未预期的错误: {e}")

    def shutdown(self):
        """停止spin线程并清理资源"""
        if not self._spin_running:
            return  # 已经关闭

        self._spin_running = False

        # 取消 Executor
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=0.5)
            except Exception as e:
                self.get_logger().warning(f"关闭 Executor 时出错: {e}")

        # 等待spin线程结束
        if self._spin_thread is not None and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():
                self.get_logger().warning("Spin线程未能在超时时间内结束")
            else:
                self.get_logger().info("Spin线程已停止")

    def get_transform(self, parent_frame: str, child_frame: str, timeout: float = 1.0) -> Optional[TransformStamped]:
        """
        获取TF转换

        Args:
            parent_frame: 父坐标系（必须指定）
            child_frame: 子坐标系（必须指定）
            timeout: 超时时间（秒）

        Returns:
            TransformStamped对象，如果失败则返回None
        """
        if not parent_frame or not child_frame:
            self.get_logger().error("必须指定parent_frame和child_frame")
            return None

        try:
            # 等待TF转换可用
            transform = self.tf_buffer.lookup_transform(
                target_frame=parent_frame,
                source_frame=child_frame,
                # 使用当前时钟类型的零时刻，兼容实时时钟和仿真时钟
                time=rclpy.time.Time(clock_type=self.get_clock().clock_type),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
            return transform
        except Exception as e:
            self.get_logger().warning(f"获取TF转换失败: {parent_frame} <- {child_frame}: {e}")
            return None

    def get_transform_as_dict(self, parent_frame: str, child_frame: str, timeout: float = 1.0) -> Optional[dict]:
        """
        获取TF转换字典格式
        """
        transform = self.get_transform(parent_frame=parent_frame, child_frame=child_frame, timeout=timeout)
        if transform is None:
            return None
        try:
            return {
                "rotation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w,
                },
                "translation": {
                    "x": transform.transform.translation.x,
                    "y": transform.transform.translation.y,
                    "z": transform.transform.translation.z,
                },
            }
        except Exception as e:
            self.get_logger().error(f"转换TF转换为字典时出错: {e}")
            return None

    def get_transform_as_list(self, parent_frame: str, child_frame: str, timeout: float = 1.0) -> Optional[list[float]]:
        """
        获取TF转换列表格式
        """
        transform = self.get_transform(parent_frame=parent_frame, child_frame=child_frame, timeout=timeout)
        if transform is None:
            return None
        try:
            return [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        except Exception as e:
            self.get_logger().error(f"转换TF转换为列表时出错: {e}")
            return None

    def get_transform_as_matrix(
        self, parent_frame: str, child_frame: str, timeout: float = 1.0
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        获取TF转换矩阵（旋转矩阵和位移向量）

        Args:
            parent_frame: 父坐标系（必须指定）
            child_frame: 子坐标系（必须指定）
            timeout: 超时时间（秒）

        Returns:
            (rotation_matrix, translation_vector) 元组，如果失败则返回None，rotation_matrix为3x3旋转矩阵，translation_vector为3x1位移向量
        """
        transform = self.get_transform(parent_frame=parent_frame, child_frame=child_frame, timeout=timeout)
        if transform is None:
            return None

        try:
            # 提取旋转四元数
            q = transform.transform.rotation
            # 转换为旋转矩阵
            rotation_matrix = self._quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)

            # 提取位移向量
            t = transform.transform.translation
            translation_vector = np.array([t.x, t.y, t.z])

            return rotation_matrix, translation_vector
        except Exception as e:
            self.get_logger().error(f"转换TF转换为矩阵时出错: {e}")
            return None

    def _quaternion_to_rotation_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """
        将四元数转换为旋转矩阵

        Args:
            x, y, z, w: 四元数分量

        Returns:
            3x3旋转矩阵
        """
        # 使用scipy库函数转换四元数到旋转矩阵
        rotation = SciPyRotation.from_quat([x, y, z, w])
        return rotation.as_matrix()

    def get_pose_as_dict(self, parent_frame: str, child_frame: str, timeout: float = 5.0) -> Optional[dict]:
        """
        获取位姿字典格式

        Args:
            parent_frame: 父坐标系（必须指定）
            child_frame: 子坐标系（必须指定）
            timeout: 超时时间（秒）

        Returns:
            包含位姿信息的字典，如果失败则返回None
        """
        result = self.get_transform_as_list(parent_frame, child_frame, timeout)
        if result is None:
            self.get_logger().error(f"获取位姿失败: {parent_frame} <- {child_frame}")
            return None

        self.get_logger().info(f"成功获取位姿: {parent_frame} <- {child_frame}")

        pose = {
            "position": {
                "x": result[0],
                "y": result[1],
                "z": result[2],
            },
            "orientation": {
                "qx": result[3],
                "qy": result[4],
                "qz": result[5],
                "qw": result[6],
            },
        }
        return pose

    def get_pose_as_list(self, parent_frame: str, child_frame: str, timeout: float = 5.0) -> Optional[list[float]]:
        """
        获取位姿列表格式
        """
        result = self.get_transform_as_list(parent_frame, child_frame, timeout)
        if result is None:
            return None
        return result

    def transform_pose(
        self,
        pose: Pose,
        child_frame: str,
        parent_frame: str,
        timeout_sec: float = 5.0,
    ) -> Optional[Pose]:
        """
        将位姿从child_frame坐标系转换到parent_frame坐标系

        Args:
            pose: 在child_frame坐标系中的位姿 (geometry_msgs/Pose)
            child_frame: 子坐标系名称（源坐标系）
            parent_frame: 父坐标系名称（目标坐标系）
            timeout_sec: 等待变换的超时时间（秒）

        Returns:
            geometry_msgs/Pose: 转换后在parent_frame坐标系中的位姿，失败返回None
        """
        try:
            # 创建PoseStamped消息（TF转换需要带时间戳和frame_id）
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = child_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = pose

            # 转换位姿：从child_frame到parent_frame
            pose_transformed_stamped = self.tf_buffer.transform(
                pose_stamped,
                target_frame=parent_frame,
                timeout=Duration(seconds=timeout_sec),
            )

            self.get_logger().info(
                f"位姿转换成功: {child_frame} -> {parent_frame}, "
                f"位置=({pose_transformed_stamped.pose.position.x:.4f}, "
                f"{pose_transformed_stamped.pose.position.y:.4f}, "
                f"{pose_transformed_stamped.pose.position.z:.4f})"
            )

            return pose_transformed_stamped.pose

        except Exception as e:
            self.get_logger().error(
                f"位姿转换失败: {str(e)}\n" f"请确保TF树中存在从 '{child_frame}' 到 '{parent_frame}' 的变换"
            )
            return None
