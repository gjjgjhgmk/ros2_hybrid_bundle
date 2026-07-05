#!/usr/bin/env python3
"""
Waypoint管理器
负责加载和管理waypoints数据
"""

import json
import logging
from typing import Dict, Any, Optional, List
from .blackboard_manager import BlackboardManager, AccessType, BlackboardError

logger = logging.getLogger(__name__)


class ArmWaypointManager:
    """机械臂Waypoint管理器"""
    
    def __init__(self, waypoints_path: str, blackboard_manager: BlackboardManager):
        """
        初始化机械臂Waypoint管理器
        
        Args:
            waypoints_path: 机械臂waypoints文件路径
            blackboard_manager: 黑板管理器实例
        """
        self.waypoints_path = waypoints_path
        self.blackboard_manager = blackboard_manager
        self.waypoints_data: Dict[str, Any] = {}
        
        self._load_waypoints()
        self._load_to_blackboard()
        
    def _load_waypoints(self):
        """
        加载waypoints文件
        
        Raises:
            ValueError: 加载失败时抛出
        """
        try:
            with open(self.waypoints_path, 'r', encoding='utf-8') as f:
                self.waypoints_data = json.load(f)
            logger.info(f"Waypoints文件加载成功: {self.waypoints_path}, 共{len(self.waypoints_data)}个waypoint")
        except Exception as e:
            raise ValueError("Waypoints文件加载失败")
    
    def _load_to_blackboard(self):
        """
        将waypoints数据加载到黑板
        
        Raises:
            ValueError: 加载失败时抛出
        """
        if not self.blackboard_manager or not self.waypoints_data:
            raise ValueError("黑板管理器未初始化或waypoints数据为空")
        
        try:
            # 注册waypoints相关变量到黑板
            self.blackboard_manager.register_key("arm_waypoints_data", AccessType.WRITE)
            
            # 将waypoints数据存储到黑板
            self.blackboard_manager.set("arm_waypoints_data", self.waypoints_data)
            
            logger.info(f"机械臂Waypoints数据已加载到黑板，共{len(self.waypoints_data)}个waypoint")
            
        except Exception as e:
            raise ValueError("加载机械臂Waypoints到黑板失败")
    
    def get_waypoint(self, waypoint_name: str) -> Optional[Dict[str, Any]]:
        """
        获取指定的机械臂Waypoint数据
        
        Args:
            waypoint_name: 机械臂Waypoint名称
            
        Returns:
            机械臂Waypoint数据或None
        """
        if not self.blackboard_manager:
            raise ValueError("黑板管理器未初始化")
        
        waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
        if waypoint_name in waypoints_data:
            return waypoints_data[waypoint_name]
        else:
            raise ValueError(f"机械臂Waypoint {waypoint_name} 不存在")
    
    def list_waypoints(self) -> List[str]:
        """
        获取所有waypoint名称列表
        
        Returns:
            waypoint名称列表
        """
        if not self.blackboard_manager:
            raise ValueError("黑板管理器未初始化")
        
        waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
        return list(waypoints_data.keys())
    
    def get_count(self) -> int:
        """
        获取waypoints数量
        
        Returns:
            waypoints数量
        """
        if not self.blackboard_manager:
            raise ValueError("黑板管理器未初始化")
        
        return self.blackboard_manager.get("arm_waypoints_count", 0)