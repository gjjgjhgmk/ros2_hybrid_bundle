#!/usr/bin/env python3
"""
TF客户端
用于查询坐标变换
"""

import zmq
import json
import logging
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)


class TFClient:
    """TF客户端 - 用于查询坐标变换"""
    
    def __init__(self, server_ip: str = "localhost", server_port: int = 5609, timeout: int = 5):
        """
        初始化TF客户端
        
        Args:
            server_ip: TF服务器IP
            server_port: TF服务器端口
            timeout: 超时时间（秒）
        """
        self.server_address = f"tcp://{server_ip}:{server_port}"
        self.timeout_ms = timeout * 1000
        
        # ZMQ设置
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        self.socket.connect(self.server_address)
        
        self.is_connected = True
        logger.info(f"TF客户端已连接到: {self.server_address}, 超时时间: {timeout}秒")
    
    def _reconnect(self):
        """重新连接socket（用于处理超时后的状态恢复）"""
        try:
            if self.socket:
                self.socket.setsockopt(zmq.LINGER, 0)  # 立即关闭
                self.socket.close()
        except:
            pass
        
        try:
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self.socket.connect(self.server_address)
            logger.debug(f"TF客户端已重新连接: {self.server_address}")
        except Exception as e:
            logger.error(f"重新连接失败: {e}")
            self.is_connected = False
    
    def lookup_transform(self, source_frame: str, target_frame: str) -> Optional[Dict[str, Any]]:
        """
        查询坐标变换
        
        Args:
            source_frame: 源坐标系
            target_frame: 目标坐标系
            
        Returns:
            Optional[Dict[str, Any]]: 变换信息，如果失败返回None
                成功时返回格式:
                {
                    'success': True,
                    'message': '...',
                    'data': {
                        'translation': {'x': float, 'y': float, 'z': float},
                        'rotation': {'x': float, 'y': float, 'z': float, 'w': float}
                    }
                }
        """
        request = {
            'action': 'lookup_transform',
            'data': {
                'source_frame': source_frame,
                'target_frame': target_frame
            }
        }
        
        try:
            # 发送请求
            logger.debug(f"查询变换: {source_frame} -> {target_frame}")
            self.socket.send_string(json.dumps(request))
            
            # 接收响应
            response_str = self.socket.recv_string()
            response = json.loads(response_str)
            
            if response.get('success'):
                logger.debug(f"成功获取变换: {source_frame} -> {target_frame}")
            else:
                logger.warning(f"查询变换失败: {response.get('message', 'Unknown error')}")
            
            return response
            
        except zmq.Again:
            logger.error(f"请求超时 ({self.timeout_ms}ms)")
            # 超时后需要重新连接，因为REQ socket在超时后状态异常
            self._reconnect()
            return None
        except zmq.ZMQError as e:
            # ZMQ错误（如状态错误），需要重新连接
            logger.error(f"ZMQ错误: {e}")
            self._reconnect()
            return None
        except Exception as e:
            logger.error(f"查询变换异常: {e}")
            # 其他异常也可能导致socket状态异常，尝试重新连接
            self._reconnect()
            return None
    
    def get_translation(self, source_frame: str, target_frame: str) -> Optional[Dict[str, float]]:
        """
        获取平移部分
        
        Args:
            source_frame: 源坐标系
            target_frame: 目标坐标系
            
        Returns:
            Optional[Dict[str, float]]: {'x': float, 'y': float, 'z': float}，失败返回None
        """
        response = self.lookup_transform(source_frame, target_frame)
        if response and response.get('success'):
            return response.get('data', {}).get('translation')
        return None
    
    def get_rotation(self, source_frame: str, target_frame: str) -> Optional[Dict[str, float]]:
        """
        获取旋转部分（四元数）
        
        Args:
            source_frame: 源坐标系
            target_frame: 目标坐标系
            
        Returns:
            Optional[Dict[str, float]]: {'x': float, 'y': float, 'z': float, 'w': float}，失败返回None
        """
        response = self.lookup_transform(source_frame, target_frame)
        if response and response.get('success'):
            return response.get('data', {}).get('rotation')
        return None
    
    def close(self):
        """关闭连接"""
        if self.is_connected:
            self.socket.close()
            self.context.term()
            self.is_connected = False
            logger.info("TF客户端连接已关闭")
    
    def __enter__(self):
        """支持with语句"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持with语句"""
        self.close()

