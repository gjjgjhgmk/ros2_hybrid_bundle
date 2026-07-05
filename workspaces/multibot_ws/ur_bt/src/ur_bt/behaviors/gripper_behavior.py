#!/usr/bin/env python3
"""
夹爪行为节点
基于py_trees的行为树节点，封装夹爪操作
支持异步非阻塞执行和同时控制两个夹爪
"""

import py_trees
import logging
import time
import threading
from typing import Optional, Union
from ..clients.gripper.gripper_zmq_client import GripperZMQClient

logger = logging.getLogger(__name__)


class GripperSetPosition(py_trees.behaviour.Behaviour):
    """夹爪设置位置 - 异步执行"""
    
    def __init__(self, client: GripperZMQClient, position: float, 
                 max_effort: float = 50.0, name: Optional[str] = None):
        """
        初始化夹爪设置位置行为
        
        Args:
            client: GripperZMQClient实例
            position: 夹爪位置 (0.0 = 完全打开, 0.8 = 完全关闭)
            max_effort: 最大力度 (N)
            name: 行为名称
        """
        if name is None:
            name = f"GripperSetPosition_{client.gripper_name}_{position:.2f}"
        super().__init__(name=name)
        self.client = client
        self.position = position
        self.max_effort = max_effort
        self.task_thread: Optional[threading.Thread] = None
        self.task_result: Optional[bool] = None
        self.task_started = False
        self.task_completed = False
        self.start_time: Optional[float] = None
        # 超时时间：客户端超时时间 + 5秒缓冲（考虑重连时间）
        self.timeout = (client.timeout_ms / 1000.0) + 5.0
    
    def setup(self, **kwargs):
        """初始化行为"""
        logger.info(f"[{self.client.gripper_name}手] 设置夹爪位置: position={self.position:.3f}, max_effort={self.max_effort}N")
    
    def initialise(self):
        """行为开始执行"""
        if self.task_started:
            logger.warning(f"[{self.client.gripper_name}手] 状态异常，有任务在执行")
            return
        
        logger.info(f"[{self.client.gripper_name}手] 开始执行夹爪位置设置: {self.position:.3f}")
        self.task_started = True
        self.task_completed = False
        self.task_result = None
        self.start_time = time.time()
        
        self.task_thread = threading.Thread(target=self._execute_task, daemon=True)
        self.task_thread.start()
    
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        if not self.task_started:
            self.initialise()
            return py_trees.common.Status.RUNNING
        
        if self.start_time and (time.time() - self.start_time) > self.timeout:
            logger.error(f"[{self.client.gripper_name}手] 夹爪位置设置超时 ({self.timeout}秒)")
            self.task_started = False
            return py_trees.common.Status.FAILURE
        
        if self.task_completed:
            self.task_started = False
            if self.task_result:
                logger.info(f"[{self.client.gripper_name}手] 夹爪位置设置成功")
                return py_trees.common.Status.SUCCESS
            else:
                logger.error(f"[{self.client.gripper_name}手] 夹爪位置设置失败")
                return py_trees.common.Status.FAILURE
        
        return py_trees.common.Status.RUNNING
    
    def _execute_task(self):
        """在后台线程中执行实际任务"""
        try:
            self.task_result = self.client.set_position(self.position, self.max_effort)
        except Exception as e:
            logger.error(f"[{self.client.gripper_name}手] 夹爪位置设置异常: {e}")
            self.task_result = False
        finally:
            self.task_completed = True
    
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info(f"[{self.client.gripper_name}手] 夹爪位置设置行为完成")
        else:
            logger.warning(f"[{self.client.gripper_name}手] 夹爪位置设置行为失败")


class GripperBehavior:
    """夹爪行为节点工厂类"""
    
    def __init__(self, left_client: Optional[GripperZMQClient] = None, 
                 right_client: Optional[GripperZMQClient] = None):
        """
        初始化夹爪行为工厂
        
        Args:
            left_client: 左手夹爪客户端（可选）
            right_client: 右手夹爪客户端（可选）
        """
        self.left_client = left_client
        self.right_client = right_client
        
        if left_client is None and right_client is None:
            logger.warning("未提供任何夹爪客户端")
    
    def open(self, gripper: str = "left", max_effort: float = 50.0, 
             name: Optional[str] = None) -> Union[GripperSetPosition, py_trees.composites.Parallel]:
        """
        创建打开夹爪的行为（set_position(0.0) 的便捷方法）
        
        Args:
            gripper: 夹爪标识 ("left", "right", 或 "both")
            max_effort: 最大力度 (N)
            name: 行为名称（可选）
            
        Returns:
            GripperSetPosition 或 Parallel（同时控制两个夹爪）
        """
        return self.set_position(gripper, 0.0, max_effort, name or f"GripperOpen_{gripper}")
    
    def close(self, gripper: str = "left", max_effort: float = 50.0,
              name: Optional[str] = None) -> Union[GripperSetPosition, py_trees.composites.Parallel]:
        """
        创建关闭夹爪的行为（set_position(0.8) 的便捷方法）
        
        Args:
            gripper: 夹爪标识 ("left", "right", 或 "both")
            max_effort: 最大力度 (N)
            name: 行为名称（可选）
            
        Returns:
            GripperSetPosition 或 Parallel（同时控制两个夹爪）
        """
        return self.set_position(gripper, 0.8, max_effort, name or f"GripperClose_{gripper}")
    
    def set_position(self, gripper: str, position: float, 
                    max_effort: float = 50.0, name: Optional[str] = None) -> Union[GripperSetPosition, py_trees.composites.Parallel]:
        """
        创建设置夹爪位置的行为
        
        Args:
            gripper: 夹爪标识 ("left", "right", 或 "both")
            position: 夹爪位置 (0.0 = 完全打开, 0.8 = 完全关闭)
            max_effort: 最大力度 (N)
            name: 行为名称（可选）
            
        Returns:
            GripperSetPosition 或 Parallel（同时控制两个夹爪）
        """
        if gripper == "both":
            if self.left_client is None or self.right_client is None:
                raise RuntimeError("需要左右手客户端才能同时控制")
            
            left_set = GripperSetPosition(self.left_client, position, max_effort,
                                         name=f"GripperSetPosition_left_{position:.2f}" if name is None else f"{name}_left")
            right_set = GripperSetPosition(self.right_client, position, max_effort,
                                          name=f"GripperSetPosition_right_{position:.2f}" if name is None else f"{name}_right")
            
            return py_trees.composites.Parallel(
                name=name or f"GripperSetPosition_both_{position:.2f}",
                children=[left_set, right_set],
                policy=py_trees.common.ParallelPolicy.SuccessOnAll()
            )
        
        client = self._get_client(gripper)
        if name is None:
            name = f"GripperSetPosition_{gripper}_{position:.2f}"
        return GripperSetPosition(client, position, max_effort, name)
    
    def _get_client(self, gripper: str) -> GripperZMQClient:
        """获取客户端"""
        if gripper == "left":
            if self.left_client is None:
                raise RuntimeError("左手夹爪客户端未初始化")
            return self.left_client
        elif gripper == "right":
            if self.right_client is None:
                raise RuntimeError("右手夹爪客户端未初始化")
            return self.right_client
        else:
            raise ValueError(f"gripper 必须是 'left'、'right' 或 'both'，当前值: {gripper}")
    
    # 快捷方法
    def open_left(self, max_effort: float = 50.0, name: Optional[str] = None) -> GripperSetPosition:
        """快捷方法：打开左手夹爪"""
        return self.open("left", max_effort, name)
    
    def open_right(self, max_effort: float = 50.0, name: Optional[str] = None) -> GripperSetPosition:
        """快捷方法：打开右手夹爪"""
        return self.open("right", max_effort, name)
    
    def close_left(self, max_effort: float = 50.0, name: Optional[str] = None) -> GripperSetPosition:
        """快捷方法：关闭左手夹爪"""
        return self.close("left", max_effort, name)
    
    def close_right(self, max_effort: float = 50.0, name: Optional[str] = None) -> GripperSetPosition:
        """快捷方法：关闭右手夹爪"""
        return self.close("right", max_effort, name)
    
    def set_left_position(self, position: float, max_effort: float = 50.0, 
                        name: Optional[str] = None) -> GripperSetPosition:
        """快捷方法：设置左手夹爪位置"""
        return self.set_position("left", position, max_effort, name)
    
    def set_right_position(self, position: float, max_effort: float = 50.0,
                          name: Optional[str] = None) -> GripperSetPosition:
        """快捷方法：设置右手夹爪位置"""
        return self.set_position("right", position, max_effort, name)

