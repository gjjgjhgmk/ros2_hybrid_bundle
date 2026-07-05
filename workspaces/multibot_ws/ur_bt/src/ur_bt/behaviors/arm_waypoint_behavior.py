#!/usr/bin/env python3
"""
手臂路径点行为节点
基于py_trees的行为树节点，封装手臂路径点操作
支持路径点更新、视觉更新、路径点复制等功能
"""

import py_trees
import logging
from typing import Dict, Any, Optional, List
from ..blackboard_manager import BlackboardManager, BlackboardError
from .arm_utils import ArmUtils

logger = logging.getLogger(__name__)


class DofUpdateRule:
    """单个自由度的更新规则"""
    
    def __init__(self, mode: str = "keep", value: float = 0.0, reference: str = "vision"):
        """
        初始化自由度更新规则
        
        Args:
            mode: 更新模式
                - "keep": 保持参考对象的值
                - "offset": 在参考值基础上加偏移
                - "absolute": 设置为绝对值
            value: 数值（偏移量或绝对值）
            reference: 参考对象
                - "vision": 使用视觉识别结果
                - "waypoint": 使用原waypoint配置
                - "source_waypoint": 使用源waypoint配置
        """
        self.mode = mode
        self.value = value
        self.reference = reference
    
    def apply(self, vision_value: float, waypoint_value: float, source_waypoint_value: float = 0.0) -> float:
        """
        应用更新规则
        
        Args:
            vision_value: 视觉识别的值
            waypoint_value: 原waypoint的值
            source_waypoint_value: 源waypoint的值
            
        Returns:
            更新后的值
        """
        # 选择参考值
        if self.reference == "vision":
            ref_value = vision_value
        elif self.reference == "waypoint":
            ref_value = waypoint_value
        elif self.reference == "source_waypoint":
            ref_value = source_waypoint_value
        else:
            logger.warning(f"未知参考类型: {self.reference}，使用waypoint")
            ref_value = waypoint_value
        
        # 应用模式
        if self.mode == "keep":
            return ref_value
        elif self.mode == "offset":
            return ref_value + self.value
        elif self.mode == "absolute":
            return self.value
        else:
            logger.warning(f"未知模式: {self.mode}，使用keep模式")
            return ref_value


class WaypointUpdateConfig:
    """Waypoint更新配置"""
    
    def __init__(self):
        """初始化配置，默认所有自由度保持waypoint原始配置"""
        self.x = DofUpdateRule("keep", 0.0, "waypoint")
        self.y = DofUpdateRule("keep", 0.0, "waypoint")
        self.z = DofUpdateRule("keep", 0.0, "waypoint")
        self.roll = DofUpdateRule("keep", 0.0, "waypoint")
        self.pitch = DofUpdateRule("keep", 0.0, "waypoint")
        self.yaw = DofUpdateRule("keep", 0.0, "waypoint")
    
    @staticmethod
    def from_dict(config: Dict[str, Dict[str, Any]]) -> 'WaypointUpdateConfig':
        """
        从字典创建配置
        
        Args:
            config: 配置字典，例如:
                {
                    "x": {"mode": "offset", "value": 0.0, "reference": "vision"},
                    "y": {"mode": "keep", "reference": "vision"},
                    "z": {"mode": "offset", "value": 0.05, "reference": "vision"},
                    "roll": {"mode": "keep", "reference": "waypoint"},
                    "pitch": {"mode": "keep", "reference": "waypoint"},
                    "yaw": {"mode": "absolute", "value": 1.57}
                }
        
        Returns:
            WaypointUpdateConfig实例
        """
        update_config = WaypointUpdateConfig()
        
        for dof_name in ["x", "y", "z", "roll", "pitch", "yaw"]:
            if dof_name in config:
                dof_config = config[dof_name]
                mode = dof_config.get("mode", "keep")
                value = dof_config.get("value", 0.0)
                reference = dof_config.get("reference", "vision")
                
                setattr(update_config, dof_name, DofUpdateRule(mode, value, reference))
        
        return update_config
    
    def __str__(self) -> str:
        """字符串表示"""
        lines = ["WaypointUpdateConfig:"]
        for dof in ["x", "y", "z", "roll", "pitch", "yaw"]:
            rule = getattr(self, dof)
            lines.append(f"  {dof}: mode={rule.mode}, value={rule.value}, ref={rule.reference}")
        return "\n".join(lines)


class WaypointCopyConfig:
    """路径点复制配置 - 默认使用源路径点"""
    
    def __init__(self):
        """初始化配置，默认所有自由度使用源路径点"""
        self.x = DofUpdateRule("keep", 0.0, "source_waypoint")
        self.y = DofUpdateRule("keep", 0.0, "source_waypoint")
        self.z = DofUpdateRule("keep", 0.0, "source_waypoint")
        self.roll = DofUpdateRule("keep", 0.0, "source_waypoint")
        self.pitch = DofUpdateRule("keep", 0.0, "source_waypoint")
        self.yaw = DofUpdateRule("keep", 0.0, "source_waypoint")
    
    @staticmethod
    def from_dict(config: Dict[str, Dict[str, Any]]) -> 'WaypointCopyConfig':
        """
        从字典创建配置
        
        Args:
            config: 配置字典，例如:
                {
                    "x": {"mode": "offset", "value": 0.1, "reference": "source_waypoint"},
                    "y": {"mode": "keep", "reference": "source_waypoint"},
                    "z": {"mode": "offset", "value": 0.05, "reference": "source_waypoint"},
                    "roll": {"mode": "absolute", "value": 0.0},
                    "pitch": {"mode": "keep", "reference": "source_waypoint"},
                    "yaw": {"mode": "absolute", "value": 1.57}
                }
        
        Returns:
            WaypointCopyConfig实例
        """
        copy_config = WaypointCopyConfig()
        
        for dof_name in ["x", "y", "z", "roll", "pitch", "yaw"]:
            if dof_name in config:
                dof_config = config[dof_name]
                mode = dof_config.get("mode", "keep")
                value = dof_config.get("value", 0.0)
                reference = dof_config.get("reference", "source_waypoint")  # 默认使用源路径点
                
                setattr(copy_config, dof_name, DofUpdateRule(mode, value, reference))
        
        return copy_config
    
    def __str__(self) -> str:
        """字符串表示"""
        lines = ["WaypointCopyConfig:"]
        for dof in ["x", "y", "z", "roll", "pitch", "yaw"]:
            rule = getattr(self, dof)
            lines.append(f"  {dof}: mode={rule.mode}, value={rule.value}, ref={rule.reference}")
        return "\n".join(lines)


class ArmUpdateWaypoint(py_trees.behaviour.Behaviour):
    """机械臂waypoint更新行为 - 同步执行"""
    
    def __init__(self, blackboard_manager: BlackboardManager, 
                 waypoint_name: str, updates: Dict[str, Any], 
                 name: str = "ArmUpdateWaypoint"):
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.waypoint_name = waypoint_name
        self.updates = updates  # 要更新的字段字典
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.info(f"设置机械臂waypoint更新: {self.waypoint_name}, 更新字段: {list(self.updates.keys())}")
        
    def initialise(self):
        """行为开始执行"""
        logger.info(f"开始执行机械臂waypoint更新: {self.waypoint_name}")
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态 - 同步执行"""
        try:
            # 从黑板获取waypoint数据
            waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
            if self.waypoint_name not in waypoints_data:
                logger.error(f"Waypoint {self.waypoint_name} 不存在")
                return py_trees.common.Status.FAILURE
            
            # 获取原始waypoint数据
            original_waypoint = waypoints_data[self.waypoint_name].copy()
            
            # 应用更新
            updated_waypoint = original_waypoint.copy()
            for key, value in self.updates.items():
                updated_waypoint[key] = value
                logger.info(f"更新 {self.waypoint_name}.{key}: {original_waypoint.get(key)} -> {value}")
            
            # 更新黑板数据
            waypoints_data[self.waypoint_name] = updated_waypoint
            self.blackboard_manager.set("arm_waypoints_data", waypoints_data)
            
            logger.info(f"机械臂waypoint更新成功: {self.waypoint_name}")
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            logger.error(f"机械臂waypoint更新异常: {e}")
            return py_trees.common.Status.FAILURE
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("机械臂waypoint更新行为完成")
        else:
            logger.warning("机械臂waypoint更新行为失败")


class ArmUpdateWaypointFromVision(py_trees.behaviour.Behaviour):
    """根据视觉识别结果更新waypoint - 同步执行（支持灵活配置）"""
    
    def __init__(self, blackboard_manager: BlackboardManager,
                 waypoint_name: str,
                 update_config: Optional[WaypointUpdateConfig] = None,
                 target_category: Optional[str] = None,
                 name: str = "ArmUpdateWaypointFromVision"):
        """
        初始化根据视觉结果更新waypoint的行为
        
        Args:
            blackboard_manager: 黑板管理器
            waypoint_name: 要更新的waypoint名称
            update_config: 更新配置（WaypointUpdateConfig实例），如果为None则默认使用视觉识别结果
            target_category: 目标类别，如果为None则选择置信度最高的
            name: 行为名称
        """
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.waypoint_name = waypoint_name
        self.update_config = update_config or WaypointUpdateConfig()
        self.target_category = target_category
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.info(f"设置视觉waypoint更新: {self.waypoint_name}, 目标类别: {self.target_category}")
        
    def initialise(self):
        """行为开始执行"""
        logger.info(f"开始根据视觉结果更新waypoint: {self.waypoint_name}")
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态 - 同步执行"""
        try:
            # 1. 从黑板获取视觉识别结果
            vision_results = self.blackboard_manager.get("vision_results", {})
            if not vision_results or not vision_results.get('success', False):
                logger.error("视觉识别结果不存在或识别失败")
                return py_trees.common.Status.FAILURE
            
            detections = vision_results.get('detections', [])
            if not detections:
                logger.error("没有检测到任何对象")
                return py_trees.common.Status.FAILURE
            
            # 2. 选择目标检测结果
            target_detection = self._select_target(detections)
            if not target_detection:
                logger.error(f"未找到目标类别: {self.target_category}")
                return py_trees.common.Status.FAILURE
            
            # 3. 提取视觉识别的位置和姿态
            pose = target_detection.get('pose', [])
            if len(pose) < 7:
                logger.error(f"姿态数据不完整: {pose}")
                return py_trees.common.Status.FAILURE
            
            # pose格式: [x, y, z, qx, qy, qz, qw]
            vision_position = [pose[0], pose[1], pose[2]]
            vision_orientation = [pose[3], pose[4], pose[5], pose[6]]  # 四元数
            vision_euler = ArmUtils.quaternion_to_euler(vision_orientation)
            
            # 4. 获取原waypoint配置
            waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
            if self.waypoint_name not in waypoints_data:
                logger.error(f"Waypoint {self.waypoint_name} 不存在")
                return py_trees.common.Status.FAILURE
            
            original_waypoint = waypoints_data[self.waypoint_name].copy()
            waypoint_position = original_waypoint.get('position', [0.0, 0.0, 0.0])
            waypoint_orientation = original_waypoint.get('orientation', [0.0, 0.0, 0.0, 1.0])
            waypoint_euler = ArmUtils.quaternion_to_euler(waypoint_orientation)
            
            # 5. 应用更新配置
            updated_position = [
                self.update_config.x.apply(vision_position[0], waypoint_position[0]),
                self.update_config.y.apply(vision_position[1], waypoint_position[1]),
                self.update_config.z.apply(vision_position[2], waypoint_position[2])
            ]
            
            updated_euler = [
                self.update_config.roll.apply(vision_euler[0], waypoint_euler[0]),
                self.update_config.pitch.apply(vision_euler[1], waypoint_euler[1]),
                self.update_config.yaw.apply(vision_euler[2], waypoint_euler[2])
            ]
            
            updated_orientation = ArmUtils.euler_to_quaternion(updated_euler)
            
            # 6. 更新waypoint
            updated_waypoint = original_waypoint.copy()
            updated_waypoint['position'] = updated_position
            updated_waypoint['orientation'] = updated_orientation
            
            waypoints_data[self.waypoint_name] = updated_waypoint
            self.blackboard_manager.set("arm_waypoints_data", waypoints_data)
            
            # 7. 输出详细日志
            logger.info(f"Waypoint {self.waypoint_name} 已更新:")
            logger.info(f"  位置:")
            logger.info(f"    视觉: [{vision_position[0]:.3f}, {vision_position[1]:.3f}, {vision_position[2]:.3f}]")
            logger.info(f"    原配置: [{waypoint_position[0]:.3f}, {waypoint_position[1]:.3f}, {waypoint_position[2]:.3f}]")
            logger.info(f"    更新后: [{updated_position[0]:.3f}, {updated_position[1]:.3f}, {updated_position[2]:.3f}]")
            logger.info(f"  姿态 (欧拉角):")
            logger.info(f"    视觉: [{vision_euler[0]:.3f}, {vision_euler[1]:.3f}, {vision_euler[2]:.3f}] rad")
            logger.info(f"    原配置: [{waypoint_euler[0]:.3f}, {waypoint_euler[1]:.3f}, {waypoint_euler[2]:.3f}] rad")
            logger.info(f"    更新后: [{updated_euler[0]:.3f}, {updated_euler[1]:.3f}, {updated_euler[2]:.3f}] rad")
            logger.info(f"  目标: {target_detection.get('category')} (置信度: {target_detection.get('confidence', 0.0):.3f})")
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            logger.error(f"根据视觉结果更新waypoint异常: {e}")
            import traceback
            traceback.print_exc()
            return py_trees.common.Status.FAILURE
    
    def _select_target(self, detections: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        """
        从检测结果中选择目标
        
        Args:
            detections: 检测结果列表
            
        Returns:
            选中的检测结果，如果没有找到则返回None
        """
        if self.target_category:
            # 筛选指定类别
            filtered = [d for d in detections if d.get('category') == self.target_category]
            if not filtered:
                return None
            # 返回置信度最高的
            return max(filtered, key=lambda d: d.get('confidence', 0.0))
        else:
            # 返回置信度最高的
            return max(detections, key=lambda d: d.get('confidence', 0.0))
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("视觉waypoint更新行为完成")
        else:
            logger.warning("视觉waypoint更新行为失败")


class ArmUpdateWaypointFromWaypoint(py_trees.behaviour.Behaviour):
    """根据源路径点更新目标路径点 - 同步执行（支持灵活配置）"""
    
    def __init__(self, blackboard_manager: BlackboardManager,
                 source_waypoint_name: str,
                 target_waypoint_name: str,
                 update_config: Optional[WaypointCopyConfig] = None,
                 name: str = "ArmUpdateWaypointFromWaypoint"):
        """
        初始化根据源路径点更新目标路径点的行为
        
        Args:
            blackboard_manager: 黑板管理器
            source_waypoint_name: 源路径点名称
            target_waypoint_name: 目标路径点名称
            update_config: 更新配置（WaypointCopyConfig实例），如果为None则默认使用源路径点
            name: 行为名称
        """
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.source_waypoint_name = source_waypoint_name
        self.target_waypoint_name = target_waypoint_name
        self.update_config = update_config or WaypointCopyConfig()
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.info(f"设置路径点复制更新: {self.source_waypoint_name} -> {self.target_waypoint_name}")
        
    def initialise(self):
        """行为开始执行"""
        logger.info(f"开始根据源路径点更新目标路径点: {self.source_waypoint_name} -> {self.target_waypoint_name}")
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态 - 同步执行"""
        try:
            # 1. 从黑板获取路径点数据
            waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
            
            # 2. 检查源路径点是否存在
            if self.source_waypoint_name not in waypoints_data:
                logger.error(f"源路径点 {self.source_waypoint_name} 不存在")
                return py_trees.common.Status.FAILURE
            
            # 3. 检查目标路径点是否存在
            if self.target_waypoint_name not in waypoints_data:
                logger.error(f"目标路径点 {self.target_waypoint_name} 不存在")
                return py_trees.common.Status.FAILURE
            
            # 4. 获取源路径点和目标路径点数据
            source_waypoint = waypoints_data[self.source_waypoint_name].copy()
            target_waypoint = waypoints_data[self.target_waypoint_name].copy()
            
            # 5. 复制源路径点的所有字段到目标路径点（保留目标路径点名称）
            target_waypoint_name = target_waypoint.get('name', self.target_waypoint_name)
            updated_waypoint = source_waypoint.copy()
            updated_waypoint['name'] = target_waypoint_name
            
            # 6. 提取位置和姿态数据
            source_position = source_waypoint.get('position', [0.0, 0.0, 0.0])
            source_orientation = source_waypoint.get('orientation', [0.0, 0.0, 0.0, 1.0])
            source_euler = ArmUtils.quaternion_to_euler(source_orientation)
            
            target_position = target_waypoint.get('position', [0.0, 0.0, 0.0])
            target_orientation = target_waypoint.get('orientation', [0.0, 0.0, 0.0, 1.0])
            target_euler = ArmUtils.quaternion_to_euler(target_orientation)
            
            # 7. 应用更新配置
            updated_position = [
                self.update_config.x.apply(0.0, target_position[0], source_position[0]),
                self.update_config.y.apply(0.0, target_position[1], source_position[1]),
                self.update_config.z.apply(0.0, target_position[2], source_position[2])
            ]
            
            updated_euler = [
                self.update_config.roll.apply(0.0, target_euler[0], source_euler[0]),
                self.update_config.pitch.apply(0.0, target_euler[1], source_euler[1]),
                self.update_config.yaw.apply(0.0, target_euler[2], source_euler[2])
            ]
            
            updated_orientation = ArmUtils.euler_to_quaternion(updated_euler)
            
            # 8. 更新位置和姿态
            updated_waypoint['position'] = updated_position
            updated_waypoint['orientation'] = updated_orientation
            
            # 9. 更新黑板数据
            waypoints_data[self.target_waypoint_name] = updated_waypoint
            self.blackboard_manager.set("arm_waypoints_data", waypoints_data)
            
            # 10. 输出详细日志
            logger.info(f"路径点 {self.target_waypoint_name} 已从 {self.source_waypoint_name} 更新:")
            logger.info(f"  位置:")
            logger.info(f"    源路径点: [{source_position[0]:.3f}, {source_position[1]:.3f}, {source_position[2]:.3f}]")
            logger.info(f"    目标原配置: [{target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f}]")
            logger.info(f"    更新后: [{updated_position[0]:.3f}, {updated_position[1]:.3f}, {updated_position[2]:.3f}]")
            logger.info(f"  姿态 (欧拉角):")
            logger.info(f"    源路径点: [{source_euler[0]:.3f}, {source_euler[1]:.3f}, {source_euler[2]:.3f}] rad")
            logger.info(f"    目标原配置: [{target_euler[0]:.3f}, {target_euler[1]:.3f}, {target_euler[2]:.3f}] rad")
            logger.info(f"    更新后: [{updated_euler[0]:.3f}, {updated_euler[1]:.3f}, {updated_euler[2]:.3f}] rad")
            
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            logger.error(f"根据源路径点更新目标路径点异常: {e}")
            import traceback
            traceback.print_exc()
            return py_trees.common.Status.FAILURE
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("路径点复制更新行为完成")
        else:
            logger.warning("路径点复制更新行为失败")


class ArmWaypointBehavior:
    """机械臂路径点行为节点工厂类"""
    
    def __init__(self, blackboard_manager: BlackboardManager):
        self.blackboard_manager = blackboard_manager
        
    def update_waypoint(self, waypoint_name: str, updates: Dict[str, Any], 
                       name: str = "ArmUpdateWaypoint") -> ArmUpdateWaypoint:
        """创建更新指定waypoint的行为
        
        Args:
            waypoint_name: 要更新的waypoint名称
            updates: 要更新的字段字典，例如:
                {
                    'max_velocity_scaling_factor': 0.1,
                    'max_acceleration_scaling_factor': 0.1,
                    'position': [0.5, 0.6, 0.7],
                    'joint_values': [0, 0, 0, 0, 0, 0, 0]
                }
            name: 行为名称
        """
        return ArmUpdateWaypoint(self.blackboard_manager, waypoint_name, updates, name)
    
    def update_waypoint_from_vision(self, waypoint_name: str,
                                    update_config: Optional[WaypointUpdateConfig] = None,
                                    target_category: Optional[str] = None,
                                    name: str = "ArmUpdateWaypointFromVision") -> ArmUpdateWaypointFromVision:
        """创建根据视觉识别结果更新waypoint的行为（灵活配置）
        
        Args:
            waypoint_name: 要更新的waypoint名称
            update_config: 更新配置（WaypointUpdateConfig实例），如果为None则默认使用视觉识别结果
            target_category: 目标类别，如果为None则选择置信度最高的对象
            name: 行为名称
            
        Returns:
            ArmUpdateWaypointFromVision: 视觉waypoint更新行为节点
        """
        return ArmUpdateWaypointFromVision(
            self.blackboard_manager,
            waypoint_name,
            update_config,
            target_category,
            name
        )
    
    def update_waypoint_from_waypoint(self, source_waypoint_name: str,
                                      target_waypoint_name: str,
                                      update_config: Optional[WaypointCopyConfig] = None,
                                      name: str = "ArmUpdateWaypointFromWaypoint") -> ArmUpdateWaypointFromWaypoint:
        """创建根据源路径点更新目标路径点的行为（灵活配置）
        
        Args:
            source_waypoint_name: 源路径点名称
            target_waypoint_name: 目标路径点名称
            update_config: 更新配置（WaypointCopyConfig实例），如果为None则默认使用源路径点
            name: 行为名称
            
        Returns:
            ArmUpdateWaypointFromWaypoint: 路径点复制更新行为节点
        """
        return ArmUpdateWaypointFromWaypoint(
            self.blackboard_manager,
            source_waypoint_name,
            target_waypoint_name,
            update_config,
            name
        )
