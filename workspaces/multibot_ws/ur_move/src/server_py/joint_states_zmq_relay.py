#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Joint States ZMQ Relay
用于规划端（第三台主机）：
通过 ZMQ SUB 接收左右臂的 joint_states（支持单臂或双臂模式），合并后发布到本地 ROS 2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import zmq
import json
import logging
import threading
from typing import Dict, Optional
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class JointStatesZMQRelay(Node):
    """Joint States ZMQ Relay 节点"""
    
    def __init__(self, 
                 left_arm_host: Optional[str] = None, 
                 right_arm_host: Optional[str] = None,
                 left_arm_port: int = 5650, 
                 right_arm_port: int = 5651):
        """
        初始化
        
        Args:
            left_arm_host: 左臂主机地址（可选，None表示不连接左臂）
            right_arm_host: 右臂主机地址（可选，None表示不连接右臂）
            left_arm_port: 左臂 ZMQ 端口
            right_arm_port: 右臂 ZMQ 端口
        """
        super().__init__('joint_states_zmq_relay')
        
        # 验证至少提供一个主机（允许空字符串作为None处理）
        # 注意：空字符串、None、或只包含空白字符的字符串都视为未设置
        left_arm_host_effective = left_arm_host.strip() if left_arm_host and left_arm_host.strip() else None
        right_arm_host_effective = right_arm_host.strip() if right_arm_host and right_arm_host.strip() else None
        
        if left_arm_host_effective is None and right_arm_host_effective is None:
            raise ValueError("至少需要提供一个主机地址（left_arm_host 或 right_arm_host）")
        
        # 使用有效的主机地址（将空字符串转换为None）
        left_arm_host = left_arm_host_effective
        right_arm_host = right_arm_host_effective
        
        self.left_arm_host = left_arm_host
        self.right_arm_host = right_arm_host
        self.left_arm_port = left_arm_port
        self.right_arm_port = right_arm_port
        
        # 存储最新的 joint_states（用于合并）
        self.left_joint_states: Optional[Dict] = None
        self.right_joint_states: Optional[Dict] = None
        
        # 发布合并后的 joint_states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # ZMQ 设置
        self.zmq_context = zmq.Context()
        self.left_socket = None
        self.right_socket = None
        self.running = False
        
        logger.info(f'Joint States ZMQ Relay 初始化完成')
        if self.left_arm_host:
            logger.info(f'左臂: {self.left_arm_host}:{self.left_arm_port}')
        if self.right_arm_host:
            logger.info(f'右臂: {self.right_arm_host}:{self.right_arm_port}')
    
    def start_zmq(self):
        """启动 ZMQ Subscribers"""
        try:
            # 连接左臂（如果提供）
            if self.left_arm_host:
                self.left_socket = self.zmq_context.socket(zmq.SUB)
                self.left_socket.setsockopt(zmq.SUBSCRIBE, b'')  # 订阅所有消息
                self.left_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1秒超时
                self.left_socket.connect(f'tcp://{self.left_arm_host}:{self.left_arm_port}')
                logger.info(f'左臂 ZMQ SUB 已连接: tcp://{self.left_arm_host}:{self.left_arm_port}')
            
            # 连接右臂（如果提供）
            if self.right_arm_host:
                self.right_socket = self.zmq_context.socket(zmq.SUB)
                self.right_socket.setsockopt(zmq.SUBSCRIBE, b'')  # 订阅所有消息
                self.right_socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1秒超时
                self.right_socket.connect(f'tcp://{self.right_arm_host}:{self.right_arm_port}')
                logger.info(f'右臂 ZMQ SUB 已连接: tcp://{self.right_arm_host}:{self.right_arm_port}')
            
            # 启动接收线程
            self.running = True
            
            if self.left_arm_host:
                self.left_thread = threading.Thread(target=self._receive_left, daemon=True)
                self.left_thread.start()
            
            if self.right_arm_host:
                self.right_thread = threading.Thread(target=self._receive_right, daemon=True)
                self.right_thread.start()
            
            # 启动合并和发布线程
            self.merge_thread = threading.Thread(target=self._merge_and_publish, daemon=True)
            self.merge_thread.start()
            
        except Exception as e:
            logger.error(f'ZMQ 启动失败: {e}')
            raise
    
    def _receive_left(self):
        """接收左臂 joint_states"""
        while self.running and rclpy.ok():
            try:
                message = self.left_socket.recv_string(zmq.NOBLOCK)
                joint_state_dict = json.loads(message)
                self.left_joint_states = joint_state_dict
            except zmq.Again:
                continue
            except Exception as e:
                logger.error(f'接收左臂 joint_states 失败: {e}')
                time.sleep(0.1)
    
    def _receive_right(self):
        """接收右臂 joint_states"""
        while self.running and rclpy.ok():
            try:
                message = self.right_socket.recv_string(zmq.NOBLOCK)
                joint_state_dict = json.loads(message)
                self.right_joint_states = joint_state_dict
            except zmq.Again:
                continue
            except Exception as e:
                logger.error(f'接收右臂 joint_states 失败: {e}')
                time.sleep(0.1)
    
    def _get_default_joint_states(self, joint_names: list) -> Dict:
        """
        为指定的关节名称生成默认的关节状态
        关节1设置为90度（π/2），关节2设置为-90度（-π/2），其他关节为0
        
        Args:
            joint_names: 关节名称列表
            
        Returns:
            Dict: 包含name, position, velocity, effort的字典
        """
        import math
        
        num_joints = len(joint_names)
        positions = []
        
        # 根据关节名称设置默认值
        for joint_name in joint_names:
            # 检查是否是关节1（shoulder_pan_joint 或 joint1）
            if 'shoulder_pan_joint' in joint_name or joint_name.endswith('joint1') or '_joint1' in joint_name:
                positions.append(math.pi / 2.0)  # 90度
            # 检查是否是关节2（shoulder_lift_joint 或 joint2）
            elif 'shoulder_lift_joint' in joint_name or joint_name.endswith('joint2') or '_joint2' in joint_name:
                positions.append(-math.pi / 2.0)  # -90度
            else:
                positions.append(0.0)
        
        return {
            'name': list(joint_names),  # 确保是列表副本
            'position': positions,
            'velocity': [0.0] * num_joints,
            'effort': [0.0] * num_joints
        }
    
    def _infer_missing_arm_joints(self, received_joint_names: list, target_prefix: str) -> list:
        """
        根据已接收的关节名称推断另一个手臂的关节名称
        
        Args:
            received_joint_names: 已接收的关节名称列表
            target_prefix: 目标手臂的前缀（'left_' 或 'right_'）
            
        Returns:
            list: 推断出的关节名称列表
        """
        inferred_joints = []
        
        # 确定源前缀（另一个手臂的前缀）
        source_prefix = 'right_' if target_prefix == 'left_' else 'left_'
        
        # 遍历已接收的关节名称，查找匹配源前缀的关节
        for joint_name in received_joint_names:
            if joint_name.startswith(source_prefix):
                # 替换前缀，得到目标手臂的关节名称
                target_joint_name = joint_name.replace(source_prefix, target_prefix, 1)
                inferred_joints.append(target_joint_name)
        
        return inferred_joints
    
    def _merge_and_publish(self):
        """合并并发布 joint_states"""
        while self.running and rclpy.ok():
            try:
                # 检查是否有数据
                if self.left_joint_states is None and self.right_joint_states is None:
                    time.sleep(0.01)
                    continue
                
                # 获取参考数据（用于确定关节名称）
                reference_states = self.left_joint_states or self.right_joint_states
                if reference_states is None:
                    time.sleep(0.01)
                    continue
                
                # 确定完整的关节列表
                # 如果只有一个手臂的数据，从该数据中提取关节名称模式
                # 假设关节名称包含"left_"或"right_"前缀
                reference_joint_names = reference_states.get('name', [])
                
                # 为缺失的手臂生成默认关节状态
                left_states_to_use = self.left_joint_states
                right_states_to_use = self.right_joint_states
                
                # 如果左臂数据缺失，需要生成默认值（无论是否配置了左臂主机）
                if self.left_joint_states is None:
                    # 尝试从参考数据中提取左臂关节名称
                    left_joint_names = [name for name in reference_joint_names if name.startswith('left_')]
                    
                    # 如果参考数据中没有left_前缀的关节，可能是右臂数据，需要推断
                    if not left_joint_names:
                        left_joint_names = self._infer_missing_arm_joints(reference_joint_names, 'left_')
                    
                    if left_joint_names:
                        left_states_to_use = self._get_default_joint_states(left_joint_names)
                        logger.debug(f'左臂数据缺失，使用默认值（{len(left_joint_names)}个关节，关节1=90度，关节2=-90度，其他=0）')
                
                # 如果右臂数据缺失，需要生成默认值（无论是否配置了右臂主机）
                if self.right_joint_states is None:
                    # 尝试从参考数据中提取右臂关节名称
                    right_joint_names = [name for name in reference_joint_names if name.startswith('right_')]
                    
                    # 如果参考数据中没有right_前缀的关节，可能是左臂数据，需要推断
                    if not right_joint_names:
                        right_joint_names = self._infer_missing_arm_joints(reference_joint_names, 'right_')
                    
                    if right_joint_names:
                        right_states_to_use = self._get_default_joint_states(right_joint_names)
                        logger.debug(f'右臂数据缺失，使用默认值（{len(right_joint_names)}个关节，关节1=90度，关节2=-90度，其他=0）')
                
                # 合并 joint_states
                merged_msg = JointState()
                
                # 设置 header
                merged_msg.header = Header()
                merged_msg.header.frame_id = ''  # joint_states 通常没有 frame_id
                
                # 使用当前时间
                now = self.get_clock().now()
                merged_msg.header.stamp = now.to_msg()
                
                # 合并关节名称、位置、速度、力矩
                merged_msg.name = []
                merged_msg.position = []
                merged_msg.velocity = []
                merged_msg.effort = []
                
                if left_states_to_use:
                    merged_msg.name.extend(left_states_to_use.get('name', []))
                    merged_msg.position.extend(left_states_to_use.get('position', []))
                    merged_msg.velocity.extend(left_states_to_use.get('velocity', []))
                    merged_msg.effort.extend(left_states_to_use.get('effort', []))
                
                if right_states_to_use:
                    merged_msg.name.extend(right_states_to_use.get('name', []))
                    merged_msg.position.extend(right_states_to_use.get('position', []))
                    merged_msg.velocity.extend(right_states_to_use.get('velocity', []))
                    merged_msg.effort.extend(right_states_to_use.get('effort', []))
                
                # 确保数组长度一致
                max_len = len(merged_msg.name)
                if len(merged_msg.position) < max_len:
                    merged_msg.position.extend([0.0] * (max_len - len(merged_msg.position)))
                if len(merged_msg.velocity) < max_len:
                    merged_msg.velocity.extend([0.0] * (max_len - len(merged_msg.velocity)))
                if len(merged_msg.effort) < max_len:
                    merged_msg.effort.extend([0.0] * (max_len - len(merged_msg.effort)))
                
                # 发布合并后的消息
                self.publisher.publish(merged_msg)
                
                # 以 50Hz 频率发布（20ms间隔）
                time.sleep(0.02)
                
            except Exception as e:
                logger.error(f'合并和发布 joint_states 失败: {e}')
                time.sleep(0.1)
    
    def destroy_node(self):
        """清理资源"""
        self.running = False
        
        if self.left_socket:
            self.left_socket.close()
        if self.right_socket:
            self.right_socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        
        super().destroy_node()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Joint States ZMQ Relay')
    parser.add_argument('--left-arm-host', type=str, default=None,
                       help='左臂主机地址（可选，如: 192.168.1.101）')
    parser.add_argument('--right-arm-host', type=str, default=None,
                       help='右臂主机地址（可选，如: 192.168.1.102）')
    parser.add_argument('--left-arm-port', type=int, default=5650,
                       help='左臂 ZMQ 端口（默认: 5650）')
    parser.add_argument('--right-arm-port', type=int, default=5651,
                       help='右臂 ZMQ 端口（默认: 5651）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = JointStatesZMQRelay(
            left_arm_host=args.left_arm_host,
            right_arm_host=args.right_arm_host,
            left_arm_port=args.left_arm_port,
            right_arm_port=args.right_arm_port
        )
        
        node.start_zmq()
        
        logger.info('Joint States ZMQ Relay 已启动')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        logger.info('收到中断信号')
    except Exception as e:
        logger.error(f'启动失败: {e}')
        raise
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

