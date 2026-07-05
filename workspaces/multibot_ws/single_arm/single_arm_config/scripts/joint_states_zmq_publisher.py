#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Joint States ZMQ Publisher
用于驱动端（左右臂主机）：
订阅本地 /joint_states，通过 ZMQ PUB 发送到规划主机
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import zmq
import json
import logging
import argparse
from typing import Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class JointStatesZMQPublisher(Node):
    """Joint States ZMQ Publisher 节点"""
    
    def __init__(self, zmq_port: int, arm_name: str):
        """
        初始化
        
        Args:
            zmq_port: ZMQ PUB 端口（本地绑定）
            arm_name: 手臂名称 ('left_arm' 或 'right_arm')
        """
        super().__init__('joint_states_zmq_publisher')
        
        if arm_name not in ['left_arm', 'right_arm']:
            raise ValueError(f"arm_name 必须是 'left_arm' 或 'right_arm'，当前值: {arm_name}")
        
        self.zmq_port = zmq_port
        self.arm_name = arm_name
        if arm_name == "left_arm":
            self.joint_prefix = "left_"
        elif arm_name == "right_arm":
            self.joint_prefix = "right_"
        else:
            self.joint_prefix = f"{arm_name}_"  # 兼容性处理
        
        # ZMQ 设置
        self.zmq_context = zmq.Context()
        self.socket = None
        
        # 订阅本地 joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        logger.info(f'Joint States ZMQ Publisher 初始化完成')
        logger.info(f'手臂名称: {arm_name}')
        logger.info(f'关节名前缀: {self.joint_prefix}')
        logger.info(f'订阅本地话题: /joint_states')
        logger.info(f'ZMQ PUB 端口: {zmq_port}')
    
    def start_zmq(self):
        """启动 ZMQ Publisher"""
        try:
            self.socket = self.zmq_context.socket(zmq.PUB)
            # 绑定本地端口
            address = f'tcp://*:{self.zmq_port}'
            self.socket.bind(address)
            logger.info(f'ZMQ PUB 已绑定: {address}')
                
        except Exception as e:
            logger.error(f'ZMQ socket 启动失败: {e}')
            raise
    
    def joint_states_callback(self, msg: JointState):
        """处理 joint_states 消息"""
        if self.socket is None:
            return
        
        try:
            # 为关节名添加前缀（ROS订阅到的关节名不带前缀，需要加上前缀后发送给客户端）
            prefixed_names = []
            prefixed_positions = []
            prefixed_velocities = []
            prefixed_efforts = []
            
            # 夹爪关节的基础名称（不带任何前缀）
            gripper_base_names = {
                "robotiq_85_left_knuckle_joint": "main",  # 主关节
                "robotiq_85_right_knuckle_joint": -1,
                "robotiq_85_left_inner_knuckle_joint": 1,
                "robotiq_85_right_inner_knuckle_joint": -1,
                "robotiq_85_left_finger_tip_joint": -1,
                "robotiq_85_right_finger_tip_joint": 1,
            }
            
            # 查找主关节（robotiq_85_left_knuckle_joint）的值
            main_joint_position = None
            main_joint_velocity = None
            main_joint_effort = None
            main_joint_found = False
            
            # 记录已存在的夹爪关节（使用完整名称，包含手臂前缀和gripper_前缀）
            existing_gripper_joints = set()
            
            # 处理原始消息中的关节
            for i, name in enumerate(msg.name):
                # 判断是否是夹爪关节
                is_gripper_joint = False
                
                # 检查是否是夹爪关节（精确匹配基础名称，或匹配 {prefix}_{base_name} 格式）
                for base_name, multiplier in gripper_base_names.items():
                    # 精确匹配基础名称，或者匹配 {prefix}_{base_name} 格式
                    if (name == base_name or 
                        name == f"{self.joint_prefix}{base_name}" or
                        name == f"{self.joint_prefix}gripper_{base_name}" or
                        name.endswith(f"_{base_name}")):
                        is_gripper_joint = True
                        if multiplier == "main":
                            # 这是主关节
                            main_joint_found = True
                            main_joint_position = msg.position[i] if i < len(msg.position) else 0.0
                            main_joint_velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0
                            main_joint_effort = msg.effort[i] if i < len(msg.effort) else 0.0
                        break
                
                # 为关节名添加前缀
                if is_gripper_joint:
                    # 夹爪关节：添加 {手臂前缀}gripper_ 前缀
                    # 例如：robotiq_85_left_knuckle_joint -> left_gripper_robotiq_85_left_knuckle_joint
                    if name.startswith(self.joint_prefix):
                        # 如果已经有手臂前缀，检查是否有gripper_前缀
                        if name.startswith(f"{self.joint_prefix}gripper_"):
                            prefixed_name = name
                        else:
                            # 有手臂前缀但没有gripper_前缀，添加gripper_
                            prefixed_name = name.replace(self.joint_prefix, f"{self.joint_prefix}gripper_", 1)
                    else:
                        # 没有前缀，添加 {手臂前缀}gripper_
                        prefixed_name = f"{self.joint_prefix}gripper_{name}"
                    
                    existing_gripper_joints.add(prefixed_name)
                else:
                    # 非夹爪关节：只添加手臂前缀
                    if name.startswith(self.joint_prefix):
                        prefixed_name = name
                    else:
                        prefixed_name = f"{self.joint_prefix}{name}"
                
                # 添加关节到列表
                prefixed_names.append(prefixed_name)
                prefixed_positions.append(msg.position[i] if i < len(msg.position) else 0.0)
                prefixed_velocities.append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                prefixed_efforts.append(msg.effort[i] if i < len(msg.effort) else 0.0)
            
            # 如果找到了主关节，补充缺失的夹爪关节
            if main_joint_found and main_joint_position is not None:
                # 定义需要补充的夹爪关节（使用 {手臂前缀}gripper_ 格式）
                gripper_joints_to_add = {
                    f"{self.joint_prefix}gripper_robotiq_85_right_knuckle_joint": -1,
                    f"{self.joint_prefix}gripper_robotiq_85_left_inner_knuckle_joint": 1,
                    f"{self.joint_prefix}gripper_robotiq_85_right_inner_knuckle_joint": -1,
                    f"{self.joint_prefix}gripper_robotiq_85_left_finger_tip_joint": -1,
                    f"{self.joint_prefix}gripper_robotiq_85_right_finger_tip_joint": 1,
                }
                
                for joint_name, multiplier in gripper_joints_to_add.items():
                    if joint_name not in existing_gripper_joints:
                        # 补充缺失的关节
                        prefixed_names.append(joint_name)
                        prefixed_positions.append(main_joint_position * multiplier)
                        prefixed_velocities.append(main_joint_velocity * multiplier if main_joint_velocity is not None else 0.0)
                        prefixed_efforts.append(main_joint_effort * multiplier if main_joint_effort is not None else 0.0)
            
            # 将 JointState 消息转换为字典
            joint_state_dict = {
                'header': {
                    'stamp': {
                        'sec': msg.header.stamp.sec,
                        'nanosec': msg.header.stamp.nanosec
                    },
                    'frame_id': msg.header.frame_id
                },
                'name': prefixed_names,
                'position': prefixed_positions,
                'velocity': prefixed_velocities,
                'effort': prefixed_efforts
            }
            
            # 通过 ZMQ 发送（JSON 格式）
            message = json.dumps(joint_state_dict)
            self.socket.send_string(message)
            
        except Exception as e:
            logger.error(f'发送 joint_states 失败: {e}')
    
    def destroy_node(self):
        """清理资源"""
        if self.socket:
            self.socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Joint States ZMQ Publisher')
    parser.add_argument('--zmq-port', type=int, required=True,
                       help='ZMQ PUB 端口（必需，左臂使用5650，右臂使用5651）')
    parser.add_argument('--arm-name', type=str, required=True,
                       choices=['left_arm', 'right_arm'],
                       help='手臂名称（必需）：left_arm 或 right_arm')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    node = JointStatesZMQPublisher(zmq_port=args.zmq_port, arm_name=args.arm_name)
    
    try:
        node.start_zmq()
        
        logger.info('Joint States ZMQ Publisher 已启动')
        logger.info('等待 joint_states 消息...')
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        logger.info('收到中断信号')
    except Exception as e:
        logger.error(f'运行错误: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

