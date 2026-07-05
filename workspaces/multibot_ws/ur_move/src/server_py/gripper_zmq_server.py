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
        self.action_name = f'{gripper_name}_gripper_controller/gripper_cmd'
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        self._result_received = False
        self._last_result_success = False
        
        # 等待 action server
        if not self._wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'无法连接到 action server: {self.action_name}')
        
        logger.info(f'夹爪控制器已就绪: {self.action_name}')
    
    def _wait_for_server(self, timeout_sec=10.0):
        """等待 action server 可用"""
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            if self._action_client.wait_for_server(timeout_sec=1.0):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
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
        if not 0.0 <= position <= 0.8:
            logger.error(f'位置值应在 0.0-0.8 之间，当前值: {position}')
            return False
        
        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command = JointState()
        goal_msg.command.name = [f'{self.gripper_name}_gripper_robotiq_85_left_knuckle_joint']
        goal_msg.command.position = [float(position)]
        goal_msg.command.effort = [float(max_effort)]
        
        self._result_received = False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
        if wait_for_result:
            start_time = time.time()
            timeout = 15.0
            while not self._result_received and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            if not self._result_received:
                logger.warning(f'等待夹爪执行结果超时 ({timeout}秒)')
                return False
            
            return self._last_result_success
        
        return True
    
    def _goal_response_callback(self, future):
        """Goal 响应回调"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.error('夹爪目标被拒绝')
                self._result_received = True
                self._last_result_success = False
                return
            
            logger.info('夹爪目标已接受，执行中...')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self._get_result_callback)
        except Exception as e:
            logger.error(f'处理夹爪目标响应回调时出错: {e}')
            self._result_received = True
            self._last_result_success = False
    
    def _feedback_callback(self, feedback_msg):
        """反馈回调"""
        pass
    
    def _get_result_callback(self, future):
        """结果回调"""
        try:
            result = future.result().result
            self._result_received = True
            self._last_result_success = result.reached_goal
            
            position = result.state.position[0] if len(result.state.position) > 0 else 0.0
            if result.reached_goal:
                logger.info(f'✓ 夹爪已到达目标位置: {position:.3f}')
            else:
                logger.warning(f'⚠ 夹爪未到达目标位置: {position:.3f}')
        except Exception as e:
            logger.error(f'处理夹爪结果回调时出错: {e}')
            self._result_received = True
            self._last_result_success = False


class GripperZMQServer:
    """夹爪 ZMQ 服务器"""
    
    def __init__(self, left_port: int = 5630, right_port: int = 5640):
        """
        初始化夹爪 ZMQ 服务器
        
        Args:
            left_port: 左手夹爪端口
            right_port: 右手夹爪端口
        """
        self.left_port = left_port
        self.right_port = right_port
        
        # ZMQ 设置
        self.context = zmq.Context()
        self.left_socket = None
        self.right_socket = None
        self.running = False
        
        # ROS 2 节点和控制器
        rclpy.init()
        self.node = Node('gripper_zmq_server')
        from rclpy.executors import SingleThreadedExecutor
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        self.left_controller: Optional[GripperController] = None
        self.right_controller: Optional[GripperController] = None
        
        # 初始化夹爪控制器
        try:
            self.left_controller = GripperController('left', 'gripper_zmq_server_left')
            self.executor.add_node(self.left_controller)
            logger.info('左手夹爪控制器已就绪')
        except Exception as e:
            logger.error(f'左手夹爪控制器初始化失败: {e}')
            self.left_controller = None
        
        try:
            self.right_controller = GripperController('right', 'gripper_zmq_server_right')
            self.executor.add_node(self.right_controller)
            logger.info('右手夹爪控制器已就绪')
        except Exception as e:
            logger.error(f'右手夹爪控制器初始化失败: {e}')
            self.right_controller = None
        
        if self.left_controller is None and self.right_controller is None:
            raise RuntimeError('无法初始化任何夹爪控制器')
    
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
            self.left_socket = self.context.socket(zmq.REP)
            self.left_socket.setsockopt(zmq.RCVTIMEO, 1000)
            self.left_socket.setsockopt(zmq.SNDTIMEO, 5000)
            self.left_socket.bind(f'tcp://*:{self.left_port}')
            logger.info(f'左手夹爪 ZMQ socket 已绑定: tcp://*:{self.left_port}')
        except Exception as e:
            logger.error(f'左手夹爪 socket 绑定失败: {e}')
            self.left_socket = None
        
        try:
            self.right_socket = self.context.socket(zmq.REP)
            self.right_socket.setsockopt(zmq.RCVTIMEO, 1000)
            self.right_socket.setsockopt(zmq.SNDTIMEO, 5000)
            self.right_socket.bind(f'tcp://*:{self.right_port}')
            logger.info(f'右手夹爪 ZMQ socket 已绑定: tcp://*:{self.right_port}')
        except Exception as e:
            logger.error(f'右手夹爪 socket 绑定失败: {e}')
            self.right_socket = None
        
        if self.left_socket is None and self.right_socket is None:
            raise RuntimeError('无法绑定任何 ZMQ socket')
        
        # 启动服务器循环线程
        if self.left_socket and self.left_controller:
            left_thread = threading.Thread(target=self._server_loop, args=('left', self.left_socket, self.left_controller), daemon=True)
            left_thread.start()
            logger.info('左手夹爪服务器线程已启动')
        
        if self.right_socket and self.right_controller:
            right_thread = threading.Thread(target=self._server_loop, args=('right', self.right_socket, self.right_controller), daemon=True)
            right_thread.start()
            logger.info('右手夹爪服务器线程已启动')
        
        logger.info('夹爪 ZMQ 服务器已启动')
    
    def stop(self):
        """停止服务器"""
        if not self.running:
            return
        
        logger.info('正在停止夹爪 ZMQ 服务器...')
        self.running = False
        
        if self.left_socket:
            self.left_socket.close()
        if self.right_socket:
            self.right_socket.close()
        if self.context:
            self.context.term()
        
        rclpy.shutdown()
        logger.info('夹爪 ZMQ 服务器已停止')
    
    def _ros_spin_loop(self):
        """ROS 2 executor 循环（处理所有节点回调）"""
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)
            time.sleep(0.01)
    
    def _server_loop(self, gripper_name: str, socket: zmq.Socket, controller: GripperController):
        """服务器主循环"""
        while self.running and rclpy.ok():
            try:
                request_str = socket.recv_string()
                request = json.loads(request_str)
                response = self._handle_request(gripper_name, request, controller)
                socket.send_string(json.dumps(response))
                
            except zmq.Again:
                continue
            except zmq.ZMQError as e:
                if self.running:
                    logger.error(f'{gripper_name}手夹爪 ZMQ 错误: {e}')
                break
            except Exception as e:
                logger.error(f'{gripper_name}手夹爪处理请求时出错: {e}')
                try:
                    error_response = {"success": False, "error": str(e)}
                    socket.send_string(json.dumps(error_response))
                except:
                    pass
    
    def _handle_request(self, gripper_name: str, request: Dict[str, Any], controller: GripperController) -> Dict[str, Any]:
        """
        处理请求
        
        Args:
            gripper_name: 夹爪名称 ('left' 或 'right')
            request: 请求字典
            controller: 夹爪控制器
            
        Returns:
            响应字典
        """
        action = request.get('action')
        
        if action == 'open':
            max_effort = request.get('max_effort', 50.0)
            success = controller.open(max_effort=max_effort, wait_for_result=True)
            return {
                "success": success,
                "message": "夹爪已打开" if success else "夹爪打开失败",
                "reached_goal": success
            }
        
        elif action == 'close':
            max_effort = request.get('max_effort', 50.0)
            success = controller.close(max_effort=max_effort, wait_for_result=True)
            return {
                "success": success,
                "message": "夹爪已关闭" if success else "夹爪关闭失败",
                "reached_goal": success
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
            success = controller.set_position(position, max_effort=max_effort, wait_for_result=True)
            return {
                "success": success,
                "message": f"夹爪位置已设置为 {position:.3f}" if success else "夹爪位置设置失败",
                "reached_goal": success
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
    parser.add_argument('--left-port', type=int, default=5630, help='左手夹爪端口（默认: 5630）')
    parser.add_argument('--right-port', type=int, default=5640, help='右手夹爪端口（默认: 5640）')
    
    args = parser.parse_args()
    
    server = GripperZMQServer(left_port=args.left_port, right_port=args.right_port)
    
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

