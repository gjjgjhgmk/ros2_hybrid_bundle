#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹执行服务器
用于驱动PC（左右臂主机）：
通过ZMQ REP接收轨迹执行请求，在本地通过ROS 2 Action执行轨迹
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import zmq
import json
import logging
import argparse
import time
from typing import Dict, Any, Optional

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TrajectoryExecutorServer(Node):
    """轨迹执行服务器节点"""
    
    def __init__(self, zmq_port: int, arm_name: str):
        """
        初始化
        
        Args:
            zmq_port: ZMQ REP 端口（本地绑定）
            arm_name: 手臂名称 ('left_arm' 或 'right_arm')
        """
        super().__init__('trajectory_executor_server')
        
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
        
        # 固定的Action名称
        self.action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
        
        # 固定的关节名称（无前缀，用于ROS）
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ]
        
        # ROS 2 Action客户端
        self._action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        self._action_server_ready = False
        
        # ZMQ设置
        self.zmq_context = zmq.Context()
        self.socket = None
        
        logger.info(f'轨迹执行服务器初始化完成')
        logger.info(f'手臂名称: {arm_name}')
        logger.info(f'关节名前缀: {self.joint_prefix}')
        logger.info(f'ZMQ REP 端口: {zmq_port}')
        logger.info(f'Action名称: {self.action_name}')
    
    def wait_for_action_server(self, timeout: float = 30.0) -> bool:
        """
        等待 action server 可用
        
        Args:
            timeout: 最大等待时间（秒）
            
        Returns:
            bool: 如果 action server 可用返回 True，否则返回 False
        """
        if self._action_server_ready:
            return True
        
        logger.info(f'等待 action server: {self.action_name}')
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self._action_client.wait_for_server(timeout_sec=1.0):
                self._action_server_ready = True
                logger.info(f'成功连接到 action server: {self.action_name}')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                logger.info(f'仍在等待 action server... ({elapsed:.1f}s / {timeout}s)')
        
        logger.error(f'无法连接到 action server: {self.action_name} (超时: {timeout}s)')
        logger.error('请确保:')
        logger.error('1. ros2_control_node 正在运行')
        logger.error('2. 机械臂控制器已启动')
        logger.error('3. 使用正确的手臂名称')
        return False
    
    def start_zmq(self):
        """启动ZMQ REP服务器"""
        try:
            self.socket = self.zmq_context.socket(zmq.REP)
            address = f'tcp://*:{self.zmq_port}'
            self.socket.bind(address)
            logger.info(f'ZMQ REP 已绑定: {address}')
        except Exception as e:
            logger.error(f'ZMQ socket 启动失败: {e}')
            raise
    
    def json_to_joint_trajectory(self, traj_json: Dict[str, Any]) -> JointTrajectory:
        """
        将JSON轨迹数据转换为ROS 2 JointTrajectory消息
        
        Args:
            traj_json: 轨迹JSON数据（客户端发送的关节名带前缀）
            
        Returns:
            JointTrajectory: ROS 2轨迹消息（关节名不带前缀）
        """
        trajectory = JointTrajectory()
        
        # 设置关节名称（去掉前缀）
        if "joint_names" in traj_json:
            # 客户端发送的关节名带前缀（如 left_arm_shoulder_pan_joint）
            # 需要去掉前缀后发送给ROS（如 shoulder_pan_joint）
            prefixed_names = traj_json["joint_names"]
            trajectory.joint_names = []
            for name in prefixed_names:
                if name.startswith(self.joint_prefix):
                    # 去掉前缀
                    trajectory.joint_names.append(name[len(self.joint_prefix):])
                else:
                    # 如果没有前缀，直接使用（兼容性处理）
                    logger.warning(f"关节名 '{name}' 不包含预期前缀 '{self.joint_prefix}'，直接使用")
                    trajectory.joint_names.append(name)
        else:
            trajectory.joint_names = self.joint_names
        
        # 转换轨迹点
        if "points" in traj_json:
            for point_json in traj_json["points"]:
                point = JointTrajectoryPoint()
                
                if "positions" in point_json:
                    point.positions = [float(p) for p in point_json["positions"]]
                
                if "velocities" in point_json:
                    point.velocities = [float(v) for v in point_json["velocities"]]
                
                if "accelerations" in point_json:
                    point.accelerations = [float(a) for a in point_json["accelerations"]]
                
                if "time_from_start" in point_json:
                    time_json = point_json["time_from_start"]
                    point.time_from_start.sec = int(time_json.get("sec", 0))
                    point.time_from_start.nanosec = int(time_json.get("nanosec", 0))
                
                trajectory.points.append(point)
        
        return trajectory
    
    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """
        执行轨迹
        
        Args:
            trajectory: ROS 2轨迹消息
            
        Returns:
            bool: 执行是否成功
        """
        # 确保 action server 可用
        if not self._action_server_ready:
            if not self.wait_for_action_server():
                return False
        
        if not trajectory.points:
            logger.error("轨迹为空")
            return False
        
        # 创建goal消息
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        logger.info(f'发送轨迹执行请求，包含 {len(trajectory.points)} 个轨迹点')
        
        # 发送goal并等待结果
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # 等待goal被接受
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            logger.error("发送goal超时")
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            logger.error("Goal被拒绝")
            return False
        
        logger.info("Goal已接受，执行中...")
        
        # 等待执行完成
        get_result_future = goal_handle.get_result_async()
        
        # 使用固定的大超时值（600秒=10分钟），以应对示教器速度设置很慢的情况
        timeout = 600.0  # 固定10分钟超时
        
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=timeout)
        
        if not get_result_future.done():
            logger.error("等待执行结果超时")
            return False
        
        result = get_result_future.result().result
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            logger.info("轨迹执行成功")
            return True
        else:
            logger.error(f"轨迹执行失败: {result.error_string} (错误码: {result.error_code})")
            return False
    
    def handle_request(self, request_data: str) -> str:
        """
        处理执行请求
        
        Args:
            request_data: JSON格式的请求字符串
            
        Returns:
            str: JSON格式的响应字符串
        """
        try:
            request = json.loads(request_data)
            
            # 验证请求格式
            if request.get("action") != "execute":
                return json.dumps({
                    "success": False,
                    "message": f"不支持的操作: {request.get('action')}"
                })
            
            # 获取轨迹数据
            if "trajectory" not in request:
                return json.dumps({
                    "success": False,
                    "message": "请求中缺少trajectory字段"
                })
            
            trajectory_json = request["trajectory"]
            
            # 转换为ROS消息
            trajectory = self.json_to_joint_trajectory(trajectory_json)
            
            # 执行轨迹
            success = self.execute_trajectory(trajectory)
            
            if success:
                return json.dumps({
                    "success": True,
                    "message": "轨迹执行成功"
                })
            else:
                return json.dumps({
                    "success": False,
                    "message": "轨迹执行失败"
                })
                
        except json.JSONDecodeError as e:
            logger.error(f"JSON解析失败: {e}")
            return json.dumps({
                "success": False,
                "message": f"JSON解析失败: {e}"
            })
        except Exception as e:
            logger.error(f"处理请求失败: {e}")
            return json.dumps({
                "success": False,
                "message": f"处理请求失败: {e}"
            })
    
    def server_loop(self):
        """服务器主循环"""
        logger.info("轨迹执行服务器已启动，等待执行请求...")
        
        # 在后台等待 action server（不阻塞服务器启动）
        import threading
        def wait_for_server_thread():
            self.wait_for_action_server(timeout=30.0)
        
        wait_thread = threading.Thread(target=wait_for_server_thread, daemon=True)
        wait_thread.start()
        
        while rclpy.ok():
            try:
                # 设置接收超时，避免阻塞ROS 2循环
                self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1秒超时
                
                # 接收请求
                request_data = self.socket.recv_string()
                logger.info(f"收到执行请求")
                
                # 如果 action server 还没准备好，先等待
                if not self._action_server_ready:
                    logger.warning("Action server 尚未就绪，等待中...")
                    if not self.wait_for_action_server(timeout=10.0):
                        error_response = json.dumps({
                            "success": False,
                            "message": "Action server 不可用，请稍后重试"
                        })
                        self.socket.send_string(error_response)
                        continue
                
                # 处理请求（这会调用ROS 2 Action，需要spin）
                response_data = self.handle_request(request_data)
                
                # 发送响应
                self.socket.send_string(response_data)
                logger.info("已发送响应")
                
            except zmq.Again:
                # 超时，继续循环（允许ROS 2处理）
                continue
            except KeyboardInterrupt:
                logger.info("收到中断信号")
                break
            except Exception as e:
                logger.error(f"服务器循环错误: {e}")
                # 发送错误响应
                try:
                    error_response = json.dumps({
                        "success": False,
                        "message": f"服务器错误: {e}"
                    })
                    self.socket.send_string(error_response)
                except:
                    pass
    
    def destroy_node(self):
        """清理资源"""
        if self.socket:
            self.socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='轨迹执行服务器')
    parser.add_argument('--zmq-port', type=int, required=True,
                       help='ZMQ REP 端口（必需）')
    parser.add_argument('--arm-name', type=str, required=True,
                       choices=['left_arm', 'right_arm'],
                       help='手臂名称（必需）：left_arm 或 right_arm')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = TrajectoryExecutorServer(zmq_port=args.zmq_port, arm_name=args.arm_name)
        node.start_zmq()
        
        # 启动服务器循环（在单独的线程中处理ZMQ，主线程处理ROS）
        import threading
        
        def zmq_loop():
            node.server_loop()
        
        zmq_thread = threading.Thread(target=zmq_loop, daemon=True)
        zmq_thread.start()
        
        # ROS 2主循环（处理Action客户端回调）
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            logger.info("收到中断信号，正在关闭...")
        
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    except Exception as e:
        logger.error(f"运行错误: {e}")
        raise
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

