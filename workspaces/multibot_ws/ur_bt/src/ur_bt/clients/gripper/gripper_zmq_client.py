#!/usr/bin/env python3
"""
夹爪 ZMQ 客户端
连接到夹爪 ZMQ 服务器，发送控制命令
本地不需要 ROS 环境
"""

import zmq
import json
import logging
from typing import Dict, Any

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GripperZMQClient:
    """夹爪 ZMQ 客户端"""
    
    def __init__(self, server_host: str = "localhost", port: int = 5630, 
                 gripper_name: str = "left", timeout_ms: int = 20000):
        """
        初始化夹爪 ZMQ 客户端
        
        Args:
            server_host: 服务器地址
            port: 服务器端口（5630=左手, 5640=右手）
            gripper_name: 夹爪名称 ('left' 或 'right')
            timeout_ms: 请求超时时间(毫秒)
        """
        self.server_host = server_host
        self.port = port
        self.gripper_name = gripper_name
        self.server_address = f"tcp://{server_host}:{port}"
        self.timeout_ms = timeout_ms
        
        # ZMQ设置
        self.context = zmq.Context()
        self.socket = None
        self.is_connected = False
    
    def __del__(self):
        """析构函数，自动清理资源"""
        self.close_connection()
    
    def connect(self) -> bool:
        """
        连接到夹爪服务器
        
        Returns:
            bool: 连接是否成功
        """
        try:
            if self.socket:
                self.socket.close()
            
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5秒发送超时
            self.socket.connect(self.server_address)
            
            # 注意：ZMQ connect() 不会立即验证服务器是否可用
            # 实际连接会在发送消息时建立
            self.is_connected = True
            logger.info(f"[{self.gripper_name}手] 已连接到夹爪服务器: {self.server_address}")
            logger.info(f"[{self.gripper_name}手] 提示: 如果服务器未运行，首次请求会超时")
            return True
            
        except Exception as e:
            logger.error(f"[{self.gripper_name}手] 连接服务器失败: {e}")
            self.is_connected = False
            return False
    
    def _send_request(self, action: str, **kwargs) -> Dict[str, Any]:
        """
        发送请求到服务器
        
        Args:
            action: 动作类型 ('open', 'close', 'set_position')
            **kwargs: 其他参数（position, max_effort 等）
            
        Returns:
            Dict: 响应结果
        """
        if not self.socket or not self.is_connected:
            if not self.connect():
                return {"success": False, "error": "无法连接到服务器"}
        
        try:
            # 构建请求消息
            request = {
                "action": action,
                **kwargs
            }
            
            # 发送请求
            request_str = json.dumps(request)
            logger.debug(f"[{self.gripper_name}手] 发送请求: {request}")
            
            self.socket.send_string(request_str)
            
            # 接收响应
            response_str = self.socket.recv_string()
            response = json.loads(response_str)
            
            if response.get("success", False):
                logger.info(f"[{self.gripper_name}手] {response.get('message', '操作成功')}")
            else:
                logger.error(f"[{self.gripper_name}手] 操作失败: {response.get('error', 'Unknown error')}")
            
            return response
            
        except zmq.Again:
            logger.error(f"[{self.gripper_name}手] 请求超时")
            logger.error(f"[{self.gripper_name}手] 可能的原因:")
            logger.error(f"  1. 夹爪 ZMQ 服务器未运行（端口 {self.port}）")
            logger.error(f"  2. 服务器地址不正确: {self.server_address}")
            logger.error(f"  3. 网络连接问题")
            logger.error(f"  请检查服务器是否已启动: ros2 launch ur_move ur_move_server.launch.py")
            return {"success": False, "error": "请求超时"}
        except Exception as e:
            logger.error(f"[{self.gripper_name}手] 请求失败: {e}")
            return {"success": False, "error": str(e)}
    
    def open(self, max_effort: float = 50.0) -> bool:
        """
        打开夹爪
        
        Args:
            max_effort: 最大力度 (N)
            
        Returns:
            bool: 是否成功
        """
        response = self._send_request("open", max_effort=max_effort)
        return response.get("success", False)
    
    def close(self, max_effort: float = 50.0) -> bool:
        """
        关闭夹爪
        
        Args:
            max_effort: 最大力度 (N)
            
        Returns:
            bool: 是否成功
        """
        response = self._send_request("close", max_effort=max_effort)
        return response.get("success", False)
    
    def set_position(self, position: float, max_effort: float = 50.0) -> bool:
        """
        设置夹爪位置
        
        Args:
            position: 夹爪位置 (0.0 = 完全打开, 0.8 = 完全关闭)
            max_effort: 最大力度 (N)
            
        Returns:
            bool: 是否成功
        """
        if not 0.0 <= position <= 0.8:
            logger.error(f"[{self.gripper_name}手] 位置值应在 0.0-0.8 之间，当前值: {position}")
            return False
        
        response = self._send_request("set_position", position=position, max_effort=max_effort)
        return response.get("success", False)
    
    def close_connection(self):
        """关闭连接"""
        self.is_connected = False
        if self.socket:
            self.socket.close()
            self.socket = None
        if self.context:
            self.context.term()
        logger.info(f"[{self.gripper_name}手] 夹爪客户端连接已关闭")
