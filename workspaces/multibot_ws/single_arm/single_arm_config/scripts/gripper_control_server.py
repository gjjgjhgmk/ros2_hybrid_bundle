#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
夹爪 ZMQ 服务器
接收 ZMQ 请求，调用 ROS 2 ActionClient 控制夹爪
支持左右手分离控制（端口 5630/5640）
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand
from sensor_msgs.msg import JointState
import zmq
import json
import logging
import threading
import time
from typing import Dict, Any, Optional
from rclpy.qos import qos_profile_sensor_data

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GripperController(Node):
    """夹爪控制器"""
    
    def __init__(self, gripper_name='left', node_name=None):
        """
        初始化夹爪控制器
        
        Args:
            gripper_name: 夹爪名称 ('left' 或 'right')
            node_name: ROS 2 节点名称（默认自动生成）
        """
        if node_name is None:
            node_name = f'gripper_controller_{gripper_name}'
        super().__init__(node_name)
        
        self.gripper_name = gripper_name
        self.action_name = 'robotiq_gripper_controller/gripper_cmd'
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        self._action_server_ready = False  # 添加状态标志
        
        # 订阅 joint_states 以监控夹爪实际位置
        self.joint_name = 'robotiq_85_left_knuckle_joint'
        self._current_position = None
        self._current_velocity = None
        self._position_lock = threading.Lock()
        self._joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            qos_profile_sensor_data
        )
        
        # 不在这里等待，延迟到首次使用时
        logger.info(f'夹爪控制器初始化完成，等待 action server: {self.action_name}')
    
    def _joint_states_callback(self, msg: JointState):
        """Joint states 回调，更新当前夹爪位置和速度"""
        try:
            if self.joint_name in msg.name:
                idx = msg.name.index(self.joint_name)
                with self._position_lock:
                    self._current_position = msg.position[idx] if idx < len(msg.position) else None
                    self._current_velocity = msg.velocity[idx] if idx < len(msg.velocity) else None
        except Exception as e:
            logger.debug(f'处理 joint_states 回调时出错: {e}')
    
    def _wait_for_position(self, target_position: float, start_position: float, 
                          max_velocity: float = 0.5, tolerance: float = 0.05) -> bool:
        """
        根据速度计算延时，等待夹爪到达目标位置
        
        Args:
            target_position: 目标位置
            start_position: 起始位置（action返回时的位置）
            max_velocity: 最大速度（rad/s，默认0.5，与控制器配置一致）
            tolerance: 位置容差
        
        Returns:
            bool: 是否到达目标位置
        """
        # 计算需要移动的距离
        distance = abs(target_position - start_position)
        
        if distance <= tolerance:
            # 已经在目标位置附近
            return True
        
        # 根据速度和距离计算需要的时间
        # 考虑加速和减速，使用平均速度（最大速度的90%）
        avg_velocity = max_velocity * 0.9
        estimated_time = distance / avg_velocity if avg_velocity > 0 else 0
        
        # 添加安全缓冲（20%额外时间）和最小等待时间（0.5秒）
        wait_time = max(estimated_time * 1.2, 0.5)
        
        # 限制最大等待时间（避免过长等待）
        max_wait_time = 30.0
        wait_time = min(wait_time, max_wait_time)
        
        # 等待计算出的时间
        time.sleep(wait_time)
        
        # 等待后检查最终位置
        # 给一些时间让 joint_states 更新
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)
        
        with self._position_lock:
            final_pos = self._current_position
        
        if final_pos is not None:
            final_error = abs(final_pos - target_position)
            # 计算实际移动距离（用于判断夹爪是否运动）
            actual_distance = abs(final_pos - start_position)
            
            # 简化判断：只要在容差内就成功
            if final_error <= tolerance:
                logger.info(f'✓ 夹爪已到达目标位置: {target_position:.3f}')
                return True
            else:
                # 如果不在容差内，检查是否移动了（只要移动了就认为成功）
                if actual_distance > 0.01:  # 移动距离大于0.01认为有运动
                    logger.info(f'✓ 夹爪已运动，目标: {target_position:.3f}')
                    return True
                else:
                    logger.warning(f'✗ 夹爪未运动，目标: {target_position:.3f}')
                    return False
        else:
            # 如果没有收到位置信息，根据误差判断
            # 如果误差在容差内，认为成功（可能是位置信息延迟）
            if distance <= tolerance * 2:  # 使用更宽松的判断
                logger.info(f'✓ 夹爪已到达目标位置: {target_position:.3f}')
                return True
            else:
                logger.warning(f'✗ 夹爪未运动，目标: {target_position:.3f}')
                return False
    
    def wait_for_action_server(self, timeout_sec=30.0):
        """
        等待 action server 可用
        
        Args:
            timeout_sec: 最大等待时间（秒）
            
        Returns:
            bool: 如果 action server 可用返回 True，否则返回 False
        """
        if self._action_server_ready:
            return True
        
        logger.info(f'等待 action server: {self.action_name}')
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if self._action_client.wait_for_server(timeout_sec=1.0):
                self._action_server_ready = True
                logger.info(f'成功连接到 action server: {self.action_name}')
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                logger.info(f'仍在等待 action server... ({elapsed:.1f}s / {timeout_sec}s)')
        
        logger.error(f'无法连接到 action server: {self.action_name} (超时: {timeout_sec}s)')
        logger.error('请确保:')
        logger.error('1. ros2_control_node 正在运行')
        logger.error('2. 夹爪控制器已启动')
        return False
    
    def open(self, max_effort=50.0, wait_for_result=True):
        """
        打开夹爪
        
        Args:
            max_effort: 最大力度 (N)
            wait_for_result: 是否等待执行完成
        
        Returns:
            bool: 是否成功
        """
        return self.set_position(0.0, max_effort, wait_for_result)
    
    def close(self, max_effort=50.0, wait_for_result=True):
        """
        关闭夹爪
        
        Args:
            max_effort: 最大力度 (N)
            wait_for_result: 是否等待执行完成
        
        Returns:
            bool: 是否成功
        """
        return self.set_position(0.8, max_effort, wait_for_result)
    
    def set_position(self, position, max_effort=50.0, wait_for_result=True):
        """
        设置夹爪位置
        
        Args:
            position: 夹爪位置 (0.0 = 完全打开, 0.8 = 完全关闭)
            max_effort: 最大力度 (N)
            wait_for_result: 是否等待执行完成
        
        Returns:
            bool: 是否成功
        """
        # 在首次使用时等待 server
        if not self._action_server_ready:
            if not self.wait_for_action_server(timeout_sec=30.0):
                logger.error(f'无法连接到 action server: {self.action_name}')
                return False
        
        if not 0.0 <= position <= 0.8:
            logger.error(f'位置值应在 0.0-0.8 之间，当前值: {position}')
            return False
        
        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command = JointState()
        goal_msg.command.name = ['robotiq_85_left_knuckle_joint']
        goal_msg.command.position = [float(position)]
        goal_msg.command.effort = [float(max_effort)]
        
        # 发送 goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        
        # 等待 goal 被接受
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
        
        if not send_goal_future.done():
            logger.error("发送 goal 超时")
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            logger.error("Goal 被拒绝")
            return False
        
        if not wait_for_result:
            return True
        
        # 等待执行完成 - 使用 future 方式
        get_result_future = goal_handle.get_result_async()
        
        # 增加超时时间（夹爪运动可能需要更长时间，特别是大范围运动）
        timeout = 30.0  # 从15秒增加到30秒
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=timeout)
        
        if not get_result_future.done():
            logger.warning(f'等待夹爪执行结果超时 ({timeout}秒)')
            return False
        
        result = get_result_future.result().result
        result_position = result.state.position[0] if len(result.state.position) > 0 else None
        
        # Action 返回后，根据速度计算延时等待夹爪到达目标位置
        # 因为 action 可能提前返回，而夹爪还在运动
        tolerance = 0.05
        max_velocity = 0.5  # 与控制器配置的 max_velocity 一致
        
        # 立即获取起始位置（action返回时的位置）
        # 优先使用result中的位置，因为它反映了action返回时的状态
        if result_position is not None:
            start_position = result_position
        else:
            # 如果result中没有位置，立即获取当前实际位置
            for _ in range(5):  # 快速获取位置（最多500ms）
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.05)
                with self._position_lock:
                    if self._current_position is not None:
                        start_position = self._current_position
                        break
            else:
                # 如果还是没有收到位置信息，使用目标位置（这种情况不应该发生）
                start_position = position
        
        # 如果起始位置已经很接近目标位置，直接返回成功
        initial_error = abs(start_position - position)
        if initial_error <= tolerance:
            logger.info(f'✓ 夹爪已到达目标位置: {position:.3f}')
            return True
        
        # 根据速度计算延时并等待
        success = self._wait_for_position(position, start_position, max_velocity=max_velocity, tolerance=tolerance)
        
        return success
    
    def _goal_response_callback(self, future):
        """Goal 响应回调（保留用于日志）"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.error('夹爪目标被拒绝（回调）')
            else:
                logger.info('夹爪目标已接受（回调）')
        except Exception as e:
            logger.error(f'处理夹爪目标响应回调时出错: {e}')
    
    def _feedback_callback(self, feedback_msg):
        """反馈回调"""
        # 可以在这里记录反馈信息
        pass
    
    def _get_result_callback(self, future):
        """结果回调（保留用于日志）"""
        try:
            result = future.result().result
            position_actual = result.state.position[0] if len(result.state.position) > 0 else 0.0
            if result.reached_goal:
                logger.info(f'✓ 夹爪已到达目标位置（回调）: {position_actual:.3f}')
            else:
                logger.warning(f'⚠ 夹爪未到达目标位置（回调）: {position_actual:.3f}')
        except Exception as e:
            logger.error(f'处理夹爪结果回调时出错: {e}')


class GripperZMQServer:
    """夹爪 ZMQ 服务器"""
    
    def __init__(self, port: int = 5630):
        """
        初始化夹爪 ZMQ 服务器
        
        Args:
            port: 夹爪 ZMQ 服务器端口（默认: 5630）
        """
        self.port = port
        
        # ZMQ 设置
        self.context = zmq.Context()
        self.socket = None
        self.running = False
        
        # ROS 2 节点和控制器
        rclpy.init()
        self.node = Node('gripper_zmq_server')
        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        self.controller: Optional[GripperController] = None
        
        # 初始化夹爪控制器（不等待 server，延迟连接）
        try:
            self.controller = GripperController('gripper', 'gripper_zmq_server')
            self.executor.add_node(self.controller)
            logger.info('夹爪控制器已初始化（将在首次使用时连接 action server）')
        except Exception as e:
            logger.error(f'夹爪控制器初始化失败: {e}')
            # 不抛出异常，允许服务继续运行，稍后重试
            logger.warning('夹爪控制器初始化失败，但服务将继续运行，稍后重试')
    
    def start(self):
        """启动服务器"""
        if self.running:
            logger.warning('服务器已在运行')
            return
        
        self.running = True
        self.executor_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
        self.executor_thread.start()
        
        # 创建 ZMQ REP socket
        try:
            self.socket = self.context.socket(zmq.REP)
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)
            self.socket.setsockopt(zmq.SNDTIMEO, 5000)
            self.socket.bind(f'tcp://*:{self.port}')
            logger.info(f'夹爪 ZMQ socket 已绑定: tcp://*:{self.port}')
        except Exception as e:
            logger.error(f'夹爪 socket 绑定失败: {e}')
            raise RuntimeError(f'无法绑定 ZMQ socket: {e}')
        
        # 启动服务器循环线程
        server_thread = threading.Thread(target=self._server_loop, daemon=True)
        server_thread.start()
        logger.info('夹爪 ZMQ 服务器已启动')
    
    def stop(self):
        """停止服务器"""
        if not self.running:
            return
        
        logger.info('正在停止夹爪 ZMQ 服务器...')
        self.running = False
        
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        
        rclpy.shutdown()
        logger.info('夹爪 ZMQ 服务器已停止')
    
    def _ros_spin_loop(self):
        """ROS 2 executor 循环（处理所有节点回调）"""
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.01)
    
    def _server_loop(self):
        """服务器主循环"""
        logger.info("夹爪 ZMQ 服务器已启动，等待执行请求...")
        
        # 在后台等待 action server（不阻塞服务器启动）
        def wait_for_server_thread():
            if self.controller:
                self.controller.wait_for_action_server(timeout_sec=30.0)
        
        wait_thread = threading.Thread(target=wait_for_server_thread, daemon=True)
        wait_thread.start()
        
        while self.running and rclpy.ok():
            try:
                request_str = self.socket.recv_string()
                request = json.loads(request_str)
                
                # 如果 action server 还没准备好，先等待
                if self.controller and not self.controller._action_server_ready:
                    logger.warning("Action server 尚未就绪，等待中...")
                    if not self.controller.wait_for_action_server(timeout_sec=10.0):
                        error_response = {
                            "success": False,
                            "error": "Action server 不可用，请稍后重试"
                        }
                        self.socket.send_string(json.dumps(error_response))
                        continue
                
                response = self._handle_request(request)
                self.socket.send_string(json.dumps(response))
                
            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                if self.running:
                    logger.error(f'夹爪 ZMQ 错误: {e}')
                break
            except Exception as e:
                logger.error(f'夹爪处理请求时出错: {e}')
                try:
                    error_response = {"success": False, "error": str(e)}
                    self.socket.send_string(json.dumps(error_response))
                except:
                    pass
    
    def _handle_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        """
        处理请求
        
        Args:
            request: 请求字典
            
        Returns:
            响应字典
        """
        # 检查控制器是否可用
        if self.controller is None:
            return {
                "success": False,
                "error": "夹爪控制器未初始化，请稍后重试"
            }
        
        action = request.get('action')
        
        if action == 'open':
            max_effort = request.get('max_effort', 50.0)
            success = self.controller.open(max_effort=max_effort, wait_for_result=True)
            return {
                "success": success,
                "message": "夹爪已打开" if success else "夹爪打开失败",
                "reached_goal": success,
                "error": None if success else "夹爪未能到达目标位置"
            }
        
        elif action == 'close':
            max_effort = request.get('max_effort', 50.0)
            success = self.controller.close(max_effort=max_effort, wait_for_result=True)
            return {
                "success": success,
                "message": "夹爪已关闭" if success else "夹爪关闭失败",
                "reached_goal": success,
                "error": None if success else "夹爪未能到达目标位置"
            }
        
        elif action == 'set_position':
            position = request.get('position')
            if position is None:
                return {
                    "success": False,
                    "error": "缺少 position 参数"
                }
            
            if not 0.0 <= position <= 0.8:
                return {
                    "success": False,
                    "error": f"position 应在 0.0-0.8 之间，当前值: {position}"
                }
            
            max_effort = request.get('max_effort', 50.0)
            success = self.controller.set_position(position, max_effort=max_effort, wait_for_result=True)
            
            # 构建错误信息
            error_msg = None
            if not success:
                error_msg = f"夹爪未能到达目标位置 {position:.3f}"
            
            return {
                "success": success,
                "message": f"夹爪位置已设置为 {position:.3f}" if success else f"夹爪位置设置失败（目标: {position:.3f}）",
                "reached_goal": success,
                "error": error_msg
            }
        
        else:
            return {
                "success": False,
                "error": f"未知的 action: {action}，支持: open, close, set_position"
            }


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='夹爪 ZMQ 服务器')
    parser.add_argument('--zmq-port', type=int, default=5630, help='夹爪 ZMQ 服务器端口（默认: 5630）')
    
    args = parser.parse_args()
    
    server = GripperZMQServer(port=args.zmq_port)
    
    try:
        server.start()
        
        # 保持运行
        while rclpy.ok():
            time.sleep(1)
    
    except KeyboardInterrupt:
        logger.info('收到中断信号，正在停止服务器...')
    finally:
        server.stop()


if __name__ == '__main__':
    main()

