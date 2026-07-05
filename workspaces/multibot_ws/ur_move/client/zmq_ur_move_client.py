#!/usr/bin/env python3
"""
UR Move ZMQ客户端
连接到ur_move轨迹规划服务器，发送路径点进行规划，然后通过ROS执行
"""

import zmq
import json
import time
from typing import Dict, Any, Optional
import logging

# 注意：执行现在在服务器端（docker）完成，本地不需要 ROS 2

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# 导入轨迹执行客户端（从当前目录）
TrajectoryExecutorClient = None
try:
    from .trajectory_executor_client import TrajectoryExecutorClient
except ImportError:
    logger.warning("无法导入 TrajectoryExecutorClient，plan_and_execute_remote 功能将不可用")
    TrajectoryExecutorClient = None


class UrMoveClient:
    """UR Move轨迹规划客户端"""
    
    # 固定的规划服务器端口
    PLANNER_SERVER_PORT = 5605
    
    def __init__(self, 
                 server_host: str = "localhost", 
                 timeout_ms: int = 60000,
                 left_arm_executor_host: Optional[str] = None,
                 right_arm_executor_host: Optional[str] = None):
        """
        初始化UR Move客户端
        
        Args:
            server_host: ur_move规划服务器主机地址（规划PC），默认: "localhost"
                        仅接受主机地址（如 "192.168.1.100" 或 "localhost"），端口固定为5605，无需指定
            timeout_ms: 请求超时时间(毫秒)
            left_arm_executor_host: 左臂执行服务器主机地址（驱动PC），默认: None（不执行左臂）
            right_arm_executor_host: 右臂执行服务器主机地址（驱动PC），默认: None（不执行右臂）
                                    只有明确配置了地址的手臂才会执行，未配置的手臂即使轨迹中包含也会跳过
        """
        # 验证输入：不允许包含端口号或协议前缀
        if ":" in server_host or server_host.startswith("tcp://"):
            raise ValueError(
                f"server_host 参数仅接受主机地址（如 'localhost' 或 '192.168.1.100'），"
                f"不允许包含端口号或协议前缀。端口固定为 {self.PLANNER_SERVER_PORT}，无需指定。"
                f"当前输入: {server_host}"
            )
        
        self.server_host = server_host
        self.server_address = f"tcp://{server_host}:{self.PLANNER_SERVER_PORT}"
        self.timeout_ms = timeout_ms
        # 保持 None，不自动设置为 localhost（只有明确配置的手臂才会执行）
        self.left_arm_executor_host = left_arm_executor_host
        self.right_arm_executor_host = right_arm_executor_host
        
        # ZMQ设置
        self.context = zmq.Context()
        self.socket = None
    
    def __del__(self):
        """析构函数，自动清理资源"""
        self._close()
        
    def _connect(self) -> bool:
        """
        连接到ur_move服务器
        
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
            
            logger.info(f"已连接到规划服务器: {self.server_host}:{self.PLANNER_SERVER_PORT}")
            return True
            
        except Exception as e:
            logger.error(f"连接服务器失败: {e}")
            return False
    
    def plan_trajectory(self, waypoints: Dict[str, Any], execute: bool = False) -> Dict[str, Any]:
        """
        请求轨迹规划（可选择是否执行）
        
        Args:
            waypoints: 路径点字典
            execute: 是否在规划后立即执行（False 时返回 execution_id）
            
        Returns:
            Dict: 规划结果，包含轨迹数据或 execution_id
        """
        if not self.socket:
            if not self._connect():
                return {"success": False, "error": "无法连接到服务器"}
        
        try:
            # 构建请求消息
            request = {"waypoints": [], "execute": execute}
            
            # 将路径点字典转换为数组格式
            for name, waypoint_data in waypoints.items():
                waypoint_data["name"] = name
                request["waypoints"].append(waypoint_data)
            
            # 发送请求
            request_str = json.dumps(request)
            action = "规划+执行" if execute else "规划"
            logger.info(f"发送{action}请求，包含 {len(request['waypoints'])} 个路径点")
            
            self.socket.send_string(request_str)
            
            # 接收响应
            response_str = self.socket.recv_string()
            response = json.loads(response_str)
            
            if response.get("success", False):
                if execute:
                    trajectories = response.get("trajectories", {})
                    if trajectories:
                        logger.info(f"轨迹规划和执行成功，包含 {len(trajectories)} 个组的轨迹")
                    else:
                        logger.warning("轨迹规划成功，但 trajectories 字段为空")
                else:
                    execution_id = response.get("execution_id")
                    logger.info(f"轨迹规划成功，execution_id: {execution_id}")
            else:
                logger.error(f"轨迹规划失败: {response.get('error', 'Unknown error')}")
            
            return response
            
        except zmq.Again:
            logger.error("请求超时")
            return {"success": False, "error": "请求超时"}
        except Exception as e:
            logger.error(f"请求失败: {e}")
            return {"success": False, "error": str(e)}
    
    def execute_trajectory(self, execution_id: str) -> Dict[str, Any]:
        """
        通过 execution_id 执行已规划的轨迹
        
        Args:
            execution_id: 规划时返回的执行ID
            
        Returns:
            Dict: 执行结果
        """
        if not self.socket:
            if not self._connect():
                return {"success": False, "error": "无法连接到服务器"}
        
        try:
            # 构建执行请求
            request = {
                "action": "execute",
                "execution_id": execution_id
            }
            
            request_str = json.dumps(request)
            logger.info(f"发送执行请求，execution_id: {execution_id}")
            
            self.socket.send_string(request_str)
            
            # 接收响应
            response_str = self.socket.recv_string()
            response = json.loads(response_str)
            
            if response.get("success", False):
                logger.info("轨迹执行已完成")
            else:
                logger.error(f"轨迹执行失败: {response.get('error', 'Unknown error')}")
            
            return response
            
        except zmq.Again:
            logger.error("请求超时")
            return {"success": False, "error": "请求超时"}
        except Exception as e:
            logger.error(f"请求失败: {e}")
            return {"success": False, "error": str(e)}
    
    def plan_and_execute(self, waypoints: Dict[str, Any]) -> Dict[str, Any]:
        """
        规划并执行轨迹（在规划服务器端执行）
        
        Args:
            waypoints: 路径点字典
            
        Returns:
            Dict: 执行结果，包含 success 字段和可能的 error 字段
        """
        return self.plan_trajectory(waypoints, execute=True)
    
    def plan_and_execute_remote(
        self,
        waypoints: Dict[str, Any],
        concurrent_execution: bool = False
    ) -> Dict[str, Any]:
        """
        规划轨迹并在远程驱动PC上执行
        
        这个方法会：
        1. 先调用 plan_trajectory 进行规划（不执行）
        2. 然后调用 execute_remote 在远程驱动PC上执行轨迹
        
        Args:
            waypoints: 路径点字典
            concurrent_execution: 是否并发向左右臂执行服务器发送轨迹
            
        Returns:
            Dict: 执行结果，包含 success 和 message 字段
                  - success: 是否成功
                  - message: 成功或失败的消息
        """
        # 步骤1: 规划轨迹（不执行）
        logger.info("步骤1: 规划轨迹...")
        plan_result = self.plan_trajectory(waypoints, execute=False)
        
        if not plan_result.get("success", False):
            error_msg = plan_result.get("error", "Unknown error")
            logger.error(f"轨迹规划失败: {error_msg}")
            return {
                "success": False,
                "message": f"轨迹规划失败: {error_msg}"
            }
        
        trajectories = plan_result.get("trajectories", {})
        if trajectories:
            logger.info(f"规划成功，包含 {len(trajectories)} 个组的轨迹")
        else:
            logger.warning("规划成功，但 trajectories 字段为空")
        
        # 步骤2: 在远程驱动PC上执行轨迹
        mode = "并发" if concurrent_execution else "顺序"
        logger.info(f"步骤2: 在远程驱动PC上{mode}执行轨迹...")
        execution_result = self.execute_remote(
            plan_result,
            concurrent_execution=concurrent_execution
        )
        
        # 返回简化的结果
        if execution_result.get("success", False):
            logger.info("远程执行成功")
            return {
                "success": True,
                "message": execution_result.get("message", "远程执行成功")
            }
        else:
            error_msg = execution_result.get("message", "Unknown error")
            logger.error(f"远程执行失败: {error_msg}")
            return {
                "success": False,
                "message": f"远程执行失败: {error_msg}"
            }
    
    def execute_remote(
        self,
        plan_result: Dict[str, Any],
        concurrent_execution: bool = False
    ) -> Dict[str, Any]:
        """
        在远程驱动PC上执行已规划的轨迹
        
        Args:
            plan_result: 规划结果字典，应包含 "trajectories" 字段，格式为 {"trajectories": {"left_arm": {...}, "right_arm": {...}}, ...}
            concurrent_execution: 是否并发向不同手臂executor发送执行请求
            
        Returns:
            Dict: 执行结果，包含 success 和 message/error 字段
        """
        # 从规划结果中提取轨迹数据
        if not plan_result.get("success", False):
            error_msg = plan_result.get("error", "Unknown error")
            return {
                "success": False,
                "message": f"规划结果失败: {error_msg}"
            }
        
        trajectories = plan_result.get("trajectories", {})
        if not trajectories:
            return {
                "success": False,
                "message": "规划结果中未包含轨迹数据"
            }
        
        # 检查轨迹中包含哪些手臂
        available_arms = set(trajectories.keys())
        valid_arms = {"left_arm", "right_arm"}
        found_arms = available_arms & valid_arms
        
        if not found_arms:
            return {
                "success": False,
                "message": f"轨迹中未包含有效的手臂组，找到: {available_arms}，期望: {valid_arms}"
            }
        
        # 只对配置了地址的手臂执行（过滤掉未配置的手臂）
        configured_arms = set()
        if "left_arm" in found_arms and self.left_arm_executor_host is not None:
            configured_arms.add("left_arm")
        if "right_arm" in found_arms and self.right_arm_executor_host is not None:
            configured_arms.add("right_arm")
        
        if not configured_arms:
            missing_arms = found_arms - configured_arms
            return {
                "success": False,
                "message": f"未配置任何手臂的执行服务器地址。轨迹包含: {found_arms}，但未配置对应地址（缺失: {missing_arms}）"
            }
        
        # 根据配置的手臂构建执行服务器地址
        # 端口固定: left_arm=5660, right_arm=5661
        left_arm_executor_address = None
        right_arm_executor_address = None
        
        if "left_arm" in configured_arms:
            left_arm_executor_address = f"tcp://{self.left_arm_executor_host}:5660"
        if "right_arm" in configured_arms:
            right_arm_executor_address = f"tcp://{self.right_arm_executor_host}:5661"
        
        # 使用执行客户端发送执行请求
        if TrajectoryExecutorClient is None:
            logger.error("TrajectoryExecutorClient未导入，无法执行远程轨迹")
            return {
                "success": False,
                "message": "TrajectoryExecutorClient未导入"
            }
        
        logger.info(f"执行轨迹，轨迹包含手臂: {', '.join(found_arms)}")
        logger.info(f"配置的手臂（将执行）: {', '.join(configured_arms)}")
        if "left_arm" in found_arms and "left_arm" not in configured_arms:
            logger.warning("轨迹包含左臂，但未配置左臂执行服务器地址，将跳过左臂执行")
        if "right_arm" in found_arms and "right_arm" not in configured_arms:
            logger.warning("轨迹包含右臂，但未配置右臂执行服务器地址，将跳过右臂执行")
        
        if left_arm_executor_address:
            logger.info(f"左臂执行服务器: {left_arm_executor_address}")
        if right_arm_executor_address:
            logger.info(f"右臂执行服务器: {right_arm_executor_address}")
        
        # 只传入需要的手臂地址
        executor_client = TrajectoryExecutorClient(
            left_arm_executor_address=left_arm_executor_address,
            right_arm_executor_address=right_arm_executor_address
        )
        
        try:
            # 只执行配置了地址的手臂
            filtered_trajectories = {arm: trajectories[arm] for arm in configured_arms}
            
            # 执行所有轨迹。并发模式只改变下发方式，不改变最终成功判定：
            # 所有配置且参与执行的手臂都必须返回success=True。
            execution_results = executor_client.execute_trajectories(
                filtered_trajectories,
                concurrent_execution=concurrent_execution
            )
            
            # 检查是否所有执行都成功，并生成消息
            missing_results = configured_arms - set(execution_results.keys())
            all_success = (
                not missing_results and
                bool(execution_results) and
                all(result.get("success", False) for result in execution_results.values())
            )
            
            if all_success:
                # 收集所有成功的消息
                messages = []
                for arm, result in execution_results.items():
                    msg = result.get("message", "执行成功")
                    messages.append(f"{arm}: {msg}")
                message = " | ".join(messages) if messages else "所有手臂执行成功"
            else:
                # 收集所有失败的消息
                errors = []
                for arm, result in execution_results.items():
                    if not result.get("success", False):
                        error = result.get("error") or result.get("message", "执行失败")
                        errors.append(f"{arm}: {error}")
                for arm in sorted(missing_results):
                    errors.append(f"{arm}: 未返回执行结果")
                message = " | ".join(errors) if errors else "执行失败"
            
            return {
                "success": all_success,
                "message": message
            }
        finally:
            # 执行客户端会在析构时自动清理
            del executor_client
    
    def _close(self):
        """关闭连接"""
        if self.socket:
            self.socket.close()
            self.socket = None
        if self.context:
            self.context.term()
