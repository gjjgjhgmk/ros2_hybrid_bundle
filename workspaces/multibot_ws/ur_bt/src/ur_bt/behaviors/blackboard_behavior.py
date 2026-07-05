#!/usr/bin/env python3
"""
黑板行为节点
基于py_trees的行为树节点，用于读写黑板数据
"""

import py_trees
import logging
import time
from typing import Any, Dict, Optional, Callable
from ..blackboard_manager import BlackboardManager, AccessType, BlackboardError

logger = logging.getLogger(__name__)


class BlackboardReader(py_trees.behaviour.Behaviour):
    """黑板数据读取行为节点"""
    
    def __init__(self, 
                 blackboard_manager: BlackboardManager,
                 read_keys: Dict[str, Any],
                 condition_func: Optional[Callable[[Dict[str, Any]], bool]] = None,
                 name: str = "BlackboardReader"):
        """
        初始化黑板读取行为
        
        Args:
            blackboard_manager: 黑板管理器
            read_keys: 要读取的键和默认值的字典
            condition_func: 条件函数，返回True表示成功，False表示失败
            name: 行为名称
        """
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.read_keys = read_keys
        self.condition_func = condition_func
        self.read_values: Dict[str, Any] = {}
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.debug(f"设置黑板读取行为: {self.name}")
        
        # 注册读取权限
        for key in self.read_keys.keys():
            self.blackboard_manager.register_key(key, AccessType.READ)
        
    def initialise(self):
        """行为开始执行"""
        logger.debug(f"开始执行黑板读取: {self.name}")
        
        # 确保注册读取权限
        for key in self.read_keys.keys():
            if key not in self.blackboard_manager.registered_keys:
                self.blackboard_manager.register_key(key, AccessType.READ)
        
        self.read_values.clear()
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        try:
            # 读取所有指定的键
            for key, default_value in self.read_keys.items():
                value = self.blackboard_manager.get(key, default_value)
                self.read_values[key] = value
                logger.debug(f"读取黑板变量: {key} = {value}")
            
            # 如果有条件函数，执行条件检查
            if self.condition_func:
                if self.condition_func(self.read_values):
                    logger.info(f"黑板读取条件满足: {self.name}")
                    return py_trees.common.Status.SUCCESS
                else:
                    logger.error(f"黑板读取条件不满足: {self.name}")
                    return py_trees.common.Status.FAILURE
            else:
                # 没有条件函数，读取成功即返回成功
                logger.info(f"黑板读取成功: {self.name}")
                return py_trees.common.Status.SUCCESS
                
        except Exception as e:
            logger.error(f"黑板读取异常: {self.name} - {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.debug(f"黑板读取行为完成: {self.name}")
        else:
            logger.warning(f"黑板读取行为失败: {self.name}")


class BlackboardWriter(py_trees.behaviour.Behaviour):
    """黑板数据写入行为节点"""
    
    def __init__(self, 
                 blackboard_manager: BlackboardManager,
                 write_data: Dict[str, Any],
                 name: str = "BlackboardWriter"):
        """
        初始化黑板写入行为
        
        Args:
            blackboard_manager: 黑板管理器
            write_data: 要写入的数据字典
            name: 行为名称
        """
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.write_data = write_data
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.debug(f"设置黑板写入行为: {self.name}")
        
        # 注册写入权限
        for key in self.write_data.keys():
            self.blackboard_manager.register_key(key, AccessType.WRITE)
        
    def initialise(self):
        """行为开始执行"""
        logger.debug(f"开始执行黑板写入: {self.name}")
        
        # 确保注册写入权限
        for key in self.write_data.keys():
            if key not in self.blackboard_manager.registered_keys:
                self.blackboard_manager.register_key(key, AccessType.WRITE)
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        try:
            # 写入所有数据
            for key, value in self.write_data.items():
                self.blackboard_manager.set(key, value)
                logger.debug(f"写入黑板变量: {key} = {value}")
            return py_trees.common.Status.SUCCESS
            
        except BlackboardError as e:
            logger.error(f"黑板写入异常: {self.name} - {e}")
            return py_trees.common.Status.FAILURE
        except Exception as e:
            logger.error(f"黑板写入异常: {self.name} - {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.debug(f"黑板写入行为完成: {self.name}")
        else:
            logger.warning(f"黑板写入行为失败: {self.name}")


class BlackboardUpdater(py_trees.behaviour.Behaviour):
    """黑板数据更新行为节点（读取->处理->写入）"""
    
    def __init__(self, 
                 blackboard_manager: BlackboardManager,
                 read_keys: Dict[str, Any],
                 update_func: Callable[[Dict[str, Any]], Dict[str, Any]],
                 name: str = "BlackboardUpdater"):
        """
        初始化黑板更新行为
        
        Args:
            blackboard_manager: 黑板管理器
            read_keys: 要读取的键和默认值的字典
            update_func: 更新函数，接收读取的数据，返回要写入的数据
            name: 行为名称
        """
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.read_keys = read_keys
        self.update_func = update_func
        self.read_values: Dict[str, Any] = {}
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.debug(f"设置黑板更新行为: {self.name}")
        
        # 注册读写权限
        for key in self.read_keys.keys():
            self.blackboard_manager.register_key(key, AccessType.READ)
        
    def initialise(self):
        """行为开始执行"""
        logger.debug(f"开始执行黑板更新: {self.name}")
        
        # 确保注册读取权限
        for key in self.read_keys.keys():
            if key not in self.blackboard_manager.registered_keys:
                self.blackboard_manager.register_key(key, AccessType.READ)
        
        self.read_values.clear()
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        try:
            # 读取所有指定的键
            for key, default_value in self.read_keys.items():
                value = self.blackboard_manager.get(key, default_value)
                self.read_values[key] = value
                logger.debug(f"读取黑板变量: {key} = {value}")
            
            # 执行更新函数
            write_data = self.update_func(self.read_values)
            
            # 写入更新后的数据
            for key, value in write_data.items():
                # 注册写入权限（如果还没有注册）
                if key not in self.blackboard_manager.registered_keys:
                    self.blackboard_manager.register_key(key, AccessType.WRITE)
                
                self.blackboard_manager.set(key, value)
                logger.debug(f"写入黑板变量: {key} = {value}")
            
            logger.debug(f"黑板更新成功: {self.name}")
            return py_trees.common.Status.SUCCESS
            
        except BlackboardError as e:
            logger.error(f"黑板更新异常: {self.name} - {e}")
            return py_trees.common.Status.FAILURE
        except Exception as e:
            logger.error(f"黑板更新异常: {self.name} - {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.debug(f"黑板更新行为完成: {self.name}")
        else:
            logger.warning(f"黑板更新行为失败: {self.name}")


class BlackboardCondition(py_trees.behaviour.Behaviour):
    """黑板条件检查行为节点"""
    
    def __init__(self, 
                 blackboard_manager: BlackboardManager,
                 condition_keys: Dict[str, Any],
                 condition_func: Callable[[Dict[str, Any]], bool],
                 name: str = "BlackboardCondition"):
        """
        初始化黑板条件检查行为
        
        Args:
            blackboard_manager: 黑板管理器
            condition_keys: 条件检查的键和默认值的字典
            condition_func: 条件函数，返回True表示条件满足
            name: 行为名称
        """
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.condition_keys = condition_keys
        self.condition_func = condition_func
        self.condition_values: Dict[str, Any] = {}
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.debug(f"设置黑板条件检查行为: {self.name}")
        
        # 注册读取权限
        for key in self.condition_keys.keys():
            self.blackboard_manager.register_key(key, AccessType.READ)
        
    def initialise(self):
        """行为开始执行"""
        logger.debug(f"开始执行黑板条件检查: {self.name}")
        
        # 确保注册读取权限
        for key in self.condition_keys.keys():
            if key not in self.blackboard_manager.registered_keys:
                self.blackboard_manager.register_key(key, AccessType.READ)
        
        self.condition_values.clear()
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        try:
            # 读取所有条件键
            for key, default_value in self.condition_keys.items():
                value = self.blackboard_manager.get(key, default_value)
                self.condition_values[key] = value
                logger.debug(f"读取条件变量: {key} = {value}")
            
            # 执行条件检查
            if self.condition_func(self.condition_values):
                logger.info(f"黑板条件满足: {self.name}")
                return py_trees.common.Status.SUCCESS
            else:
                logger.error(f"黑板条件不满足: {self.name}, 条件值: {self.condition_values}")
                return py_trees.common.Status.FAILURE
                
        except Exception as e:
            logger.error(f"黑板条件检查异常: {self.name} - {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.debug(f"黑板条件检查通过: {self.name}")
        else:
            logger.error(f"黑板条件检查失败: {self.name}")


class BlackboardBehavior:
    """黑板行为节点工厂类"""
    
    def __init__(self, blackboard_manager: BlackboardManager):
        self.blackboard_manager = blackboard_manager
        
    def reader(self, read_keys: Dict[str, Any], 
               condition_func: Optional[Callable[[Dict[str, Any]], bool]] = None,
               name: str = "BlackboardReader") -> BlackboardReader:
        """创建黑板读取行为"""
        return BlackboardReader(self.blackboard_manager, read_keys, condition_func, name)
        
    def writer(self, write_data: Dict[str, Any], 
               name: str = "BlackboardWriter") -> BlackboardWriter:
        """创建黑板写入行为"""
        return BlackboardWriter(self.blackboard_manager, write_data, name)
        
    def updater(self, read_keys: Dict[str, Any], 
                update_func: Callable[[Dict[str, Any]], Dict[str, Any]],
                name: str = "BlackboardUpdater") -> BlackboardUpdater:
        """创建黑板更新行为"""
        return BlackboardUpdater(self.blackboard_manager, read_keys, update_func, name)
        
    def condition(self, condition_keys: Dict[str, Any], 
                  condition_func: Callable[[Dict[str, Any]], bool],
                  name: str = "BlackboardCondition") -> BlackboardCondition:
        """创建黑板条件检查行为"""
        return BlackboardCondition(self.blackboard_manager, condition_keys, condition_func, name)
