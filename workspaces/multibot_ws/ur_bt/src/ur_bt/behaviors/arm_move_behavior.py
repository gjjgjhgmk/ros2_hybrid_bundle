#!/usr/bin/env python3
"""
手臂移动行为节点
基于py_trees的行为树节点，封装手臂移动操作
支持异步非阻塞执行
"""

import py_trees
import logging
import time
import threading
from typing import Dict, Any, Optional, List, Tuple
from ..clients.arm import UrMoveClient
from ..blackboard_manager import BlackboardManager, BlackboardError

logger = logging.getLogger(__name__)


class ArmMoveToWaypoints(py_trees.behaviour.Behaviour):
    """机械臂移动到指定位置 (waypoints接口) - 异步执行"""
    
    def __init__(self, client: UrMoveClient, blackboard_manager: BlackboardManager, 
                 waypoint_configs: List[Tuple[str, Optional[float], Optional[float]]], 
                 name: str = "ArmMoveToWaypoints",
                 use_remote_execution: bool = True,
                 concurrent_remote_execution: bool = False):
        """
        初始化机械臂移动行为
        
        Args:
            client: UR Move客户端
            blackboard_manager: 黑板管理器
            waypoint_configs: [(waypoint_name, vel_scale, acc_scale), ...] 格式的配置列表
            name: 行为名称
            use_remote_execution: 是否使用远程执行（True=在远程驱动PC执行，False=在规划服务器执行/模拟）
            concurrent_remote_execution: 远程执行时是否并发向左右臂executor发送轨迹
        """
        super().__init__(name=name)
        self.client = client
        self.blackboard_manager = blackboard_manager
        self.waypoint_configs = waypoint_configs  # [(waypoint_name, vel_scale, acc_scale), ...]
        self.use_remote_execution = use_remote_execution
        self.concurrent_remote_execution = concurrent_remote_execution
        self.task_thread: Optional[threading.Thread] = None
        self.task_result: Optional[Dict[str, Any]] = None
        self.task_started = False
        self.task_completed = False
        self.start_time: Optional[float] = None
        self.timeout = 600.0  # 600秒超时（10分钟），以应对示教器速度设置很慢的情况
        
    def setup(self, **kwargs):
        """初始化行为"""
        waypoint_display = []
        for waypoint_name, vel_scale, acc_scale in self.waypoint_configs:
            vel_str = f"vel={vel_scale}" if vel_scale is not None else "vel=default"
            acc_str = f"acc={acc_scale}" if acc_scale is not None else "acc=default"
            waypoint_display.append(f"{waypoint_name}({vel_str},{acc_str})")
        
        display_str = " -> ".join(waypoint_display) if len(waypoint_display) > 1 else waypoint_display[0]
        logger.info(f"设置机械臂waypoints移动: {display_str}")
        
    def initialise(self):
        """行为开始执行"""
        assert not self.task_started, "状态异常，有任务在执行"
        waypoint_display = []
        for waypoint_name, vel_scale, acc_scale in self.waypoint_configs:
            vel_str = f"vel={vel_scale}" if vel_scale is not None else "vel=default"
            acc_str = f"acc={acc_scale}" if acc_scale is not None else "acc=default"
            waypoint_display.append(f"{waypoint_name}({vel_str},{acc_str})")
        
        display_str = " -> ".join(waypoint_display) if len(waypoint_display) > 1 else waypoint_display[0]
        logger.info(f"开始执行机械臂waypoints移动: {display_str}")
        self.task_started = True
        self.task_completed = False
        self.task_result = None
        self.start_time = time.time()
        
        # 启动异步任务
        self.task_thread = threading.Thread(target=self._execute_task, daemon=True)
        self.task_thread.start()
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态 - 非阻塞"""
        if not self.task_started:
            # 如果任务还没开始，先初始化
            self.initialise()
            return py_trees.common.Status.RUNNING
            
        # 检查超时
        if self.start_time and (time.time() - self.start_time) > self.timeout:
            logger.error(f"机械臂waypoints移动超时 ({self.timeout}秒)")
            return py_trees.common.Status.FAILURE
            
        # 检查任务是否完成
        if self.task_completed:
            if self.task_result and self.task_result.get("success"):
                logger.info("机械臂waypoints移动成功")
                return py_trees.common.Status.SUCCESS
            else:
                # ur_move 可能返回 "error" 或 "message" 字段
                error_msg = self.task_result.get("error") or self.task_result.get("message", "未知错误") if self.task_result else "任务失败"
                logger.error(f"机械臂waypoints移动失败: {error_msg}")
                return py_trees.common.Status.FAILURE
                
        # 任务仍在执行中
        return py_trees.common.Status.RUNNING
        
    def _execute_task(self):
        """在后台线程中执行实际任务"""
        try:
            # 从黑板获取waypoint数据
            waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
            
            # 构建字典格式的waypoints数据 {waypoint_name: waypoint_data, ...}
            waypoints_dict = {}
            for waypoint_name, vel_scale, acc_scale in self.waypoint_configs:
                if waypoint_name not in waypoints_data:
                    raise ValueError(f"Waypoint {waypoint_name} 不存在")
                
                waypoint_data = waypoints_data[waypoint_name]
                
                # 如果未设置，使用waypoint内部的默认值
                current_vel_scale = vel_scale
                current_acc_scale = acc_scale
                
                if current_vel_scale is None:
                    current_vel_scale = waypoint_data.get('max_velocity_scaling_factor', 0.1)
                if current_acc_scale is None:
                    current_acc_scale = waypoint_data.get('max_acceleration_scaling_factor', 0.1)
                
                # 更新waypoint数据中的速度缩放参数
                updated_waypoint_data = waypoint_data.copy()
                updated_waypoint_data['max_velocity_scaling_factor'] = current_vel_scale
                updated_waypoint_data['max_acceleration_scaling_factor'] = current_acc_scale
                
                # 使用字典格式：key是waypoint_name，value是waypoint_data
                waypoints_dict[waypoint_name] = updated_waypoint_data
                
                logger.info(f"准备执行waypoint {waypoint_name}: 速度缩放={current_vel_scale}, 加速度缩放={current_acc_scale}")
            
            # 调试：打印waypoint数据
            logger.info(f"准备发送 {len(waypoints_dict)} 个waypoint")
            for i, (wp_name, wp_data) in enumerate(waypoints_dict.items(), 1):
                logger.info(f"  Waypoint {i} ({wp_name}): group={wp_data.get('group')}, type={wp_data.get('type')}")
            
            # 根据配置选择执行方式
            if self.use_remote_execution:
                mode = "并发" if self.concurrent_remote_execution else "顺序"
                logger.info(f"使用远程执行模式（在远程驱动PC上{mode}下发轨迹）")
                self.task_result = self.client.plan_and_execute_remote(
                    waypoints_dict,
                    concurrent_execution=self.concurrent_remote_execution
                )
            else:
                logger.info("使用本地执行模式（在规划服务器端执行/模拟）")
                self.task_result = self.client.plan_and_execute(waypoints_dict)
        except Exception as e:
            logger.error(f"机械臂waypoints移动异常: {e}")
            self.task_result = {"success": False, "message": str(e)}
        finally:
            self.task_completed = True

    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("机械臂waypoints移动行为完成")
        else:
            logger.warning("机械臂waypoints移动行为失败")


class ArmMoveBehavior:
    """机械臂移动行为节点工厂类"""
    
    def __init__(self, waypoint_client: UrMoveClient, 
                 blackboard_manager: BlackboardManager,
                 use_remote_execution: bool = True,
                 concurrent_remote_execution: bool = False):
        """
        初始化机械臂移动行为工厂
        
        Args:
            waypoint_client: UR Move客户端
            blackboard_manager: 黑板管理器
            use_remote_execution: 默认是否使用远程执行（True=在远程驱动PC执行，False=在规划服务器执行/模拟）
            concurrent_remote_execution: 远程执行时默认是否并发向左右臂executor发送轨迹
        """
        self.waypoint_client = waypoint_client
        self.blackboard_manager = blackboard_manager
        self.use_remote_execution = use_remote_execution
        self.concurrent_remote_execution = concurrent_remote_execution
        
    def move_to_waypoints(self, waypoint_configs: List[Tuple[str, Optional[float], Optional[float]]], 
                          name: str = "ArmMoveToWaypoints",
                          use_remote_execution: Optional[bool] = None,
                          concurrent_remote_execution: Optional[bool] = None) -> ArmMoveToWaypoints:
        """创建移动到指定waypoint配置列表的行为
        
        Args:
            waypoint_configs: [(waypoint_name, vel_scale, acc_scale), ...] 格式的配置列表
            name: 行为名称
            use_remote_execution: 是否使用远程执行（None=使用工厂默认值，True=在远程驱动PC执行，False=在规划服务器执行/模拟）
            concurrent_remote_execution: 远程执行时是否并发下发轨迹（None=使用工厂默认值）
        """
        # 如果未指定，使用工厂的默认值
        if use_remote_execution is None:
            use_remote_execution = self.use_remote_execution
        if concurrent_remote_execution is None:
            concurrent_remote_execution = self.concurrent_remote_execution
             
        return ArmMoveToWaypoints(
            self.waypoint_client, 
            self.blackboard_manager, 
            waypoint_configs, 
            name,
            use_remote_execution=use_remote_execution,
            concurrent_remote_execution=concurrent_remote_execution
        )
