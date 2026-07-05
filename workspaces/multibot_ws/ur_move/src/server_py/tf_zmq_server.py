#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TF ZMQ 服务器
提供基于 ZeroMQ 的 TF2 查询服务，支持：
1. 查询当前 TF 树内的所有 frame_id
2. 获取坐标变换关系
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import zmq
import json
import logging
import threading
import time
import sys
import tf2_ros
from typing import Dict, Any, Optional, List

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TFZMQServer:
    """TF ZMQ 服务器"""
    
    def __init__(self, port: int = 5609, buffer_duration: float = 3.0):
        """
        初始化 TF ZMQ 服务器
        
        Args:
            port: ZMQ 服务器端口
            buffer_duration: TF 缓冲区持续时间（秒）
        """
        self.port = port
        self.buffer_duration = buffer_duration
        
        # ZMQ 设置
        self.context = zmq.Context()
        self.socket = None
        self.running = False
        
        # ROS 2 节点和 TF2
        rclpy.init()
        self.node = Node('tf_zmq_server')
        
        # 初始化 TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        
        # 统计信息
        self._request_count = 0
        
        logger.info(f'TF ZMQ 服务器初始化完成 (端口: {self.port})')
    
    def start(self):
        """启动服务器"""
        if self.running:
            logger.warning('服务器已在运行')
            return
        
        self.running = True
        
        # 创建 ZMQ REP socket
        try:
            self.socket = self.context.socket(zmq.REP)
            self.socket.setsockopt(zmq.RCVTIMEO, 1000)  # 1秒超时
            self.socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5秒超时
            self.socket.bind(f'tcp://*:{self.port}')
            logger.info(f'TF ZMQ socket 已绑定: tcp://*:{self.port}')
        except Exception as e:
            logger.error(f'ZMQ socket 绑定失败: {e}')
            raise
        
        # 启动 ROS2 节点线程
        self.ros_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
        self.ros_thread.start()
        
        # 启动 ZMQ 服务器线程
        self.zmq_thread = threading.Thread(target=self._zmq_server_loop, daemon=True)
        self.zmq_thread.start()
        
        logger.info('TF ZMQ 服务器已启动')
    
    def stop(self):
        """停止服务器"""
        if not self.running:
            return
        
        logger.info('正在停止 TF ZMQ 服务器...')
        self.running = False
        
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        
        rclpy.shutdown()
        logger.info(f'TF ZMQ 服务器已停止，共处理 {self._request_count} 个请求')
    
    def _ros_spin_loop(self):
        """ROS 2 节点循环"""
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                logger.error(f'ROS2 节点运行错误: {e}')
                break
    
    def _zmq_server_loop(self):
        """ZMQ 服务器主循环"""
        while self.running and rclpy.ok():
            try:
                # 设置非阻塞接收，超时 100ms
                if self.socket.poll(100, zmq.POLLIN):
                    message = self.socket.recv_string(zmq.NOBLOCK)
                    response = self._handle_request(message)
                    self.socket.send_string(response)
                    self._request_count += 1
            except zmq.Again:
                # 超时，继续循环
                continue
            except zmq.ZMQError as e:
                if self.running:
                    logger.error(f'ZMQ 错误: {e}')
                break
            except Exception as e:
                logger.error(f'处理请求时出错: {e}')
                # 发送错误响应
                error_response = self._create_error_response(f'服务器内部错误: {str(e)}')
                try:
                    self.socket.send_string(error_response)
                except:
                    pass
    
    def _handle_request(self, message: str) -> str:
        """
        处理客户端请求
        
        Args:
            message: JSON 格式的请求消息
            
        Returns:
            JSON 格式的响应消息
        """
        try:
            # 解析请求
            request = json.loads(message)
            action = request.get('action')
            data = request.get('data', {})
            
            logger.debug(f'收到请求: action={action}, data={data}')
            
            # 路由到具体处理函数
            if action == 'get_all_frames':
                return self._handle_get_all_frames()
            elif action == 'lookup_transform':
                return self._handle_lookup_transform(data)
            else:
                return self._create_error_response(f'未知的操作: {action}')
                
        except json.JSONDecodeError as e:
            return self._create_error_response(f'JSON 解析错误: {str(e)}')
        except Exception as e:
            logger.error(f'请求处理错误: {e}')
            return self._create_error_response(f'请求处理错误: {str(e)}')
    
    def _handle_get_all_frames(self) -> str:
        """
        处理获取所有 frame_id 的请求
        
        Returns:
            包含所有 frame_id 的响应
        """
        try:
            # 获取所有 frame_id
            frames_yaml = self.tf_buffer.all_frames_as_yaml()
            
            # 解析 YAML 字符串获取 frame_id 列表
            frame_ids = []
            if frames_yaml:
                # 简单解析 YAML 获取 frame 名称
                lines = frames_yaml.split('\n')
                for line in lines:
                    if line.strip() and not line.startswith(' ') and ':' in line:
                        frame_id = line.split(':')[0].strip()
                        if frame_id and frame_id not in frame_ids:
                            frame_ids.append(frame_id)
            
            return self._create_success_response(
                message=f'成功获取 {len(frame_ids)} 个 frame_id',
                data={'frame_ids': frame_ids}
            )
            
        except Exception as e:
            logger.error(f'获取 frame_id 失败: {e}')
            return self._create_error_response(f'获取 frame_id 失败: {str(e)}')
    
    def _handle_lookup_transform(self, data: Dict[str, Any]) -> str:
        """
        处理坐标变换查询请求
        
        Args:
            data: 包含 source_frame 和 target_frame 的数据
            
        Returns:
            包含变换信息的响应
        """
        try:
            source_frame = data.get('source_frame')
            target_frame = data.get('target_frame')
            
            if not source_frame or not target_frame:
                return self._create_error_response('缺少 source_frame 或 target_frame 参数')
            
            # 查询变换 - 获取从target_frame到source_frame的变换
            # lookup_transform(A, B) 返回从B到A的变换
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                Time(),  # 获取最新的变换
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # 构造响应数据
            transform_data = {
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                }
            }
            
            return self._create_success_response(
                message=f'成功获取从 {target_frame} 到 {source_frame} 的变换',
                data=transform_data
            )
            
        except tf2_ros.LookupException as e:
            return self._create_error_response(f'变换查找失败: {str(e)}')
        except tf2_ros.ConnectivityException as e:
            return self._create_error_response(f'TF 连接错误: {str(e)}')
        except tf2_ros.ExtrapolationException as e:
            return self._create_error_response(f'TF 外推错误: {str(e)}')
        except Exception as e:
            logger.error(f'坐标变换查询失败: {e}')
            return self._create_error_response(f'坐标变换查询失败: {str(e)}')
    
    def _create_success_response(self, message: str, data: Any = None) -> str:
        """
        创建成功响应
        
        Args:
            message: 响应消息
            data: 响应数据
            
        Returns:
            JSON 格式的成功响应
        """
        response = {
            'success': True,
            'message': message,
            'data': data or {}
        }
        return json.dumps(response, ensure_ascii=False)
    
    def _create_error_response(self, message: str) -> str:
        """
        创建错误响应
        
        Args:
            message: 错误消息
            
        Returns:
            JSON 格式的错误响应
        """
        response = {
            'success': False,
            'message': message,
            'data': {}
        }
        return json.dumps(response, ensure_ascii=False)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='TF ZMQ 服务器')
    parser.add_argument('--port', type=int, default=5609, help='ZMQ 服务器端口（默认: 5609）')
    parser.add_argument('--buffer-duration', type=float, default=3.0, help='TF 缓冲区持续时间（秒，默认: 3.0）')
    
    args = parser.parse_args()
    
    server = TFZMQServer(port=args.port, buffer_duration=args.buffer_duration)
    
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

