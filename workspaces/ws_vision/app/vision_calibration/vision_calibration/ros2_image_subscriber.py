#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2图像订阅器模块

提供ROS2图像订阅功能，用于从ROS2 topic获取图像数据
"""

import time
import copy
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

    # 创建虚拟类以避免导入错误
    class Node:
        pass

    class Image:
        pass

    class CvBridge:
        pass


class ROS2ImageSubscriber(Node):
    """ROS2图像订阅器"""

    def __init__(self, topic_name: str):
        super().__init__("calibration_image_subscriber")
        self.topic_name = topic_name
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False

        # 创建订阅器
        self.subscription = self.create_subscription(Image, topic_name, self.image_callback, 2)
        self.get_logger().info(f"Subscribed to {topic_name}")

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_received = True
            self.get_logger().info("成功获取新图像[image_callback]")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def get_latest_image(self, timeout: float = 5.0, force_new: bool = False):
        """
        获取最新图像

        Args:
            timeout: 超时时间（秒）
            force_new: 是否强制获取新图像（重置image_received标志）

        Returns:
            最新图像，如果失败则返回None
        """
        if force_new:
            self.image_received = False
            self.latest_image = None

        # 如果已经有图像且不需要强制获取新图像，返回深拷贝
        if self.image_received and not force_new:
            return copy.deepcopy(self.latest_image) if self.latest_image is not None else None

        # 尝试获取新图像
        start_time = time.time()
        while time.time() - start_time < timeout:
            # 处理一次回调
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info("spin_once")

                # 检查是否收到新图像
                if self.image_received:
                    self.get_logger().info("成功获取新图像[get_latest_image]")
                    return copy.deepcopy(self.latest_image) if self.latest_image is not None else None

            except Exception as e:
                self.get_logger().error(f"处理图像回调时出错: {e}")
                break

        # 超时或出错
        if not self.image_received:
            self.get_logger().warning(f"在 {timeout} 秒内未能获取到图像")

        return copy.deepcopy(self.latest_image) if (self.image_received and self.latest_image is not None) else None
