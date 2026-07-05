#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹执行客户端
用于规划PC或其他需要发送执行请求的客户端：
通过ZMQ REQ向驱动PC的执行服务器发送轨迹执行请求
"""

import zmq
import json
from typing import Dict, Any, Optional
import logging
import threading

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TrajectoryExecutorClient:
    """轨迹执行客户端"""
    
    def __init__(self, 
                 left_arm_executor_address: Optional[str] = None,
                 right_arm_executor_address: Optional[str] = None,
                 timeout_ms: int = 60000):
        """
        初始化轨迹执行客户端
        
        Args:
            left_arm_executor_address: 左臂执行服务器地址（格式: tcp://host:port，默认: tcp://localhost:5660）
            right_arm_executor_address: 右臂执行服务器地址（格式: tcp://host:port，默认: tcp://localhost:5661）
            timeout_ms: 请求超时时间(毫秒)
        """
        # 默认地址
        self.left_arm_executor_address = left_arm_executor_address or "tcp://localhost:5660"
        self.right_arm_executor_address = right_arm_executor_address or "tcp://localhost:5661"
        self.timeout_ms = timeout_ms
        
        # ZMQ设置
        self.zmq_context = zmq.Context()
        self.left_socket = None
        self.right_socket = None
    
    def __del__(self):
        """析构函数，自动清理资源"""
        self._close()
    
    def _close(self):
        """关闭连接"""
        if self.left_socket:
            self.left_socket.close()
            self.left_socket = None
        if self.right_socket:
            self.right_socket.close()
            self.right_socket = None
        if self.zmq_context:
            self.zmq_context.term()
    
    def _get_socket(self, arm_name: str) -> Optional[zmq.Socket]:
        """
        获取或创建指定手臂的socket
        
        Args:
            arm_name: 手臂名称（left_arm 或 right_arm）
            
        Returns:
            zmq.Socket: ZMQ REQ socket，如果arm_name无效则返回None
        """
        if arm_name == "left_arm":
            if self.left_socket is None:
                self.left_socket = self.zmq_context.socket(zmq.REQ)
                self.left_socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
                self.left_socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5秒发送超时
                self.left_socket.connect(self.left_arm_executor_address)
                logger.info(f"连接到左臂执行服务器: {self.left_arm_executor_address}")
            return self.left_socket
        elif arm_name == "right_arm":
            if self.right_socket is None:
                self.right_socket = self.zmq_context.socket(zmq.REQ)
                self.right_socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
                self.right_socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5秒发送超时
                self.right_socket.connect(self.right_arm_executor_address)
                logger.info(f"连接到右臂执行服务器: {self.right_arm_executor_address}")
            return self.right_socket
        else:
            logger.error(f"不支持的arm_name: {arm_name}")
            return None

    def _get_executor_address(self, arm_name: str) -> Optional[str]:
        """获取指定手臂的执行服务器地址。"""
        if arm_name == "left_arm":
            return self.left_arm_executor_address
        if arm_name == "right_arm":
            return self.right_arm_executor_address
        logger.error(f"不支持的arm_name: {arm_name}")
        return None

    def _execute_trajectory_with_local_socket(
        self,
        arm_name: str,
        trajectory_json: Dict[str, Any]
    ) -> Dict[str, Any]:
        """在线程内创建临时socket执行轨迹，避免ZMQ socket跨线程共享。"""
        address = self._get_executor_address(arm_name)
        if address is None:
            return {
                "success": False,
                "message": f"无法获取执行服务器地址，arm_name: {arm_name}"
            }

        socket = self.zmq_context.socket(zmq.REQ)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, 5000)
        socket.connect(address)

        execute_request = {
            "action": "execute",
            "arm_name": arm_name,
            "trajectory": trajectory_json
        }

        try:
            logger.info(f"向 {arm_name} 发送执行请求: {address}")
            socket.send_string(json.dumps(execute_request))
            response_str = socket.recv_string()
            response = json.loads(response_str)

            if response.get("success", False):
                logger.info(f"{arm_name} 执行成功: {response.get('message', '')}")
            else:
                logger.error(f"{arm_name} 执行失败: {response.get('message', '')}")

            return response
        except zmq.Again:
            logger.error(f"{arm_name} 执行请求超时")
            return {
                "success": False,
                "message": "执行请求超时"
            }
        except Exception as e:
            logger.error(f"{arm_name} 执行请求失败: {e}")
            return {
                "success": False,
                "message": f"执行请求失败: {e}"
            }
        finally:
            socket.close()
    
    def execute_trajectory(self, arm_name: str, trajectory_json: Dict[str, Any]) -> Dict[str, Any]:
        """
        向指定手臂发送轨迹执行请求
        
        Args:
            arm_name: 手臂名称（left_arm 或 right_arm）
            trajectory_json: 轨迹JSON数据
            
        Returns:
            Dict: 执行结果，包含success和message字段
        """
        socket = self._get_socket(arm_name)
        if socket is None:
            return {
                "success": False,
                "message": f"无法创建socket，arm_name: {arm_name}"
            }
        
        # 构建执行请求
        execute_request = {
            "action": "execute",
            "arm_name": arm_name,
            "trajectory": trajectory_json
        }
        
        try:
            logger.info(f"向 {arm_name} 发送执行请求...")
            socket.send_string(json.dumps(execute_request))
            
            # 接收响应
            response_str = socket.recv_string()
            response = json.loads(response_str)
            
            if response.get("success", False):
                logger.info(f"{arm_name} 执行成功: {response.get('message', '')}")
            else:
                logger.error(f"{arm_name} 执行失败: {response.get('message', '')}")
            
            return response
            
        except zmq.Again:
            logger.error(f"{arm_name} 执行请求超时")
            return {
                "success": False,
                "message": "执行请求超时"
            }
        except Exception as e:
            logger.error(f"{arm_name} 执行请求失败: {e}")
            return {
                "success": False,
                "message": f"执行请求失败: {e}"
            }
    
    def execute_trajectories(
        self,
        trajectories: Dict[str, Dict[str, Any]],
        concurrent_execution: bool = False
    ) -> Dict[str, Dict[str, Any]]:
        """
        向多个手臂发送轨迹执行请求
        
        Args:
            trajectories: 轨迹字典，key为arm_name，value为轨迹JSON数据
            例如: {"left_arm": {...}, "right_arm": {...}}
            concurrent_execution: 是否并发向不同手臂executor发送请求
             
        Returns:
            Dict: 执行结果字典，key为arm_name，value为执行结果
        """
        execution_results = {}

        if not concurrent_execution or len(trajectories) <= 1:
            logger.info("顺序发送轨迹执行请求")
            for arm_name, trajectory_json in trajectories.items():
                result = self.execute_trajectory(arm_name, trajectory_json)
                execution_results[arm_name] = result
            return execution_results

        logger.info("并发发送轨迹执行请求")
        result_lock = threading.Lock()
        threads = []

        def execute_one_arm(arm_name: str, trajectory_json: Dict[str, Any]) -> None:
            # ZMQ socket 有线程亲和性；并发模式下每个线程创建并关闭自己的
            # 临时REQ socket，只共享线程安全的ZMQ context。
            try:
                result = self._execute_trajectory_with_local_socket(arm_name, trajectory_json)
            except Exception as e:
                logger.error(f"{arm_name} 并发执行线程异常: {e}")
                result = {
                    "success": False,
                    "message": f"并发执行线程异常: {e}"
                }

            # 多个线程会同时写入结果表，用锁保证结果汇总一致。
            with result_lock:
                execution_results[arm_name] = result

        for arm_name, trajectory_json in trajectories.items():
            thread = threading.Thread(
                target=execute_one_arm,
                args=(arm_name, trajectory_json),
                name=f"trajectory_executor_{arm_name}",
                daemon=True
            )
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        # 理论上每个线程都会写入结果；这里兜底标记缺失结果，避免上层误判成功。
        for arm_name in trajectories:
            if arm_name not in execution_results:
                execution_results[arm_name] = {
                    "success": False,
                    "message": "执行线程未返回结果"
                }
        
        return execution_results
