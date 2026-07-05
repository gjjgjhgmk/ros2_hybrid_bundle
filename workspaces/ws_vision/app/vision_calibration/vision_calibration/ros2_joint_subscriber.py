#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2关节角度订阅器模块

提供ROS2关节状态订阅功能，用于获取机器人关节角度数据
"""

import time
import copy
from typing import Optional, Dict, List

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

    # 创建虚拟类以避免导入错误
    class Node:
        pass

    class JointState:
        pass


class ROS2JointSubscriber(Node):
    """ROS2关节角度订阅器"""

    def __init__(self, topic_name: str = "/joint_states", joint_names: Optional[List[str]] = None):
        super().__init__("calibration_joint_subscriber")
        self.topic_name = topic_name
        self.joint_names = joint_names or []
        self.latest_joint_state = None
        self.joint_state_received = False

        # 创建订阅器
        self.subscription = self.create_subscription(JointState, topic_name, self.joint_callback, 10)
        self.get_logger().info(f"订阅 {topic_name}")

    def joint_callback(self, msg):
        """关节状态回调函数"""
        try:
            self.latest_joint_state = msg
            self.joint_state_received = True
            self.get_logger().info(f"成功获取关节状态")
        except Exception as e:
            self.get_logger().error(f"处理关节状态时出错: {e}")

    def get_latest_joint_state(
        self, joint_names: Optional[List[str]] = None, timeout: float = 5.0, force_new: bool = False
    ) -> Optional[Dict[str, float]]:
        """
        获取关节位置（关节角度）

        Args:
            joint_names: 关节名称列表，如果为None则返回所有关节的位置
            timeout: 超时时间（秒）
            force_new: 是否强制获取新数据（重置received标志）

        Returns:
            关节名称到位置的字典，如果失败则返回None
            当joint_names为None时，返回所有关节的位置
        """
        if force_new:
            self.joint_state_received = False
            self.latest_joint_state = None

        # 如果已经有数据且不需要强制获取新数据，尝试直接处理
        if self.joint_state_received and not force_new:
            return self._extract_positions(joint_names)

        # 尝试获取新的关节状态数据
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                # 处理一次回调
                rclpy.spin_once(self, timeout_sec=0.1)

                # 检查是否收到新数据
                if self.joint_state_received:
                    return self._extract_positions(joint_names)

            except Exception as e:
                self.get_logger().error(f"处理关节状态回调时出错: {e}")
                break

        # 超时或出错
        if not self.joint_state_received:
            self.get_logger().warning(f"在 {timeout} 秒内未能获取到关节状态")

        return self._extract_positions(joint_names) if self.joint_state_received else None

    def _extract_positions(self, joint_names: Optional[List[str]] = None) -> Optional[Dict[str, float]]:
        """
        从最新关节状态中提取位置信息

        Args:
            joint_names: 关节名称列表，如果为None则获取所有关节

        Returns:
            关节名称到位置的字典，如果失败则返回None
        """
        if not self.joint_state_received or self.latest_joint_state is None:
            return None

        # 使用深拷贝避免原始数据被意外修改
        joint_state = copy.deepcopy(self.latest_joint_state)

        try:
            positions = {}

            # 如果没有指定关节名称，返回所有关节
            if joint_names is None:
                for i, name in enumerate(joint_state.name):
                    if i < len(joint_state.position):
                        positions[name] = joint_state.position[i]
            else:
                # 获取指定关节的位置
                for joint_name in joint_names:
                    if joint_name in joint_state.name:
                        idx = joint_state.name.index(joint_name)
                        if idx < len(joint_state.position):
                            positions[joint_name] = joint_state.position[idx]
            if positions:
                self.get_logger().info(f"成功提取关节位置[extract_positions]，包含 {len(positions)} 个关节")
                return positions
            else:
                self.get_logger().warning("未提取到任何关节位置")
                return None

        except Exception as e:
            self.get_logger().error(f"提取关节位置时出错: {e}")
            return None
