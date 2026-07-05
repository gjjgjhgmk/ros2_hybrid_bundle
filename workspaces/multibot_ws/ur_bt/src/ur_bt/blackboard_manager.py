#!/usr/bin/env python3
"""
黑板管理器
提供py_trees黑板操作的统一接口
"""

import py_trees
import py_trees.blackboard
import logging
from typing import Any, Dict, Optional, List
from enum import Enum

logger = logging.getLogger(__name__)


class BlackboardError(Exception):
    """黑板操作异常"""
    pass


class AccessType(Enum):
    """访问类型枚举"""
    READ = py_trees.common.Access.READ
    WRITE = py_trees.common.Access.WRITE
    EXCLUSIVE_WRITE = py_trees.common.Access.EXCLUSIVE_WRITE


class BlackboardManager:
    """黑板管理器"""
    
    def __init__(self, name: str = "BlackboardManager", namespace: str = "/"):
        """
        初始化黑板管理器
        
        Args:
            name: 客户端名称
            namespace: 命名空间，默认为根命名空间
        """
        self.name = name
        self.namespace = namespace
        self.client: Optional[py_trees.blackboard.Client] = None
        self.registered_keys: Dict[str, AccessType] = {}
        
    def initialize(self) -> None:
        """初始化黑板客户端
        
        Raises:
            BlackboardError: 初始化失败时抛出
        """
        try:
            self.client = py_trees.blackboard.Client(name=self.name, namespace=self.namespace)
            logger.info(f"黑板客户端初始化成功: {self.name} (namespace: {self.namespace})")
        except Exception as e:
            logger.error(f"黑板客户端初始化失败: {e}")
            raise BlackboardError(f"黑板客户端初始化失败: {e}") from e
    
    def register_key(self, key: str, access: AccessType) -> None:
        """
        注册黑板变量
        
        Args:
            key: 变量名
            access: 访问权限
            
        Raises:
            BlackboardError: 注册失败时抛出
        """
        if not self.client:
            logger.error("黑板客户端未初始化")
            raise BlackboardError("黑板客户端未初始化")
            
        try:
            self.client.register_key(key=key, access=access.value)
            self.registered_keys[key] = access
            logger.debug(f"注册黑板变量: {key} (access: {access.name})")
        except Exception as e:
            logger.error(f"注册黑板变量失败 {key}: {e}")
            raise BlackboardError(f"注册黑板变量失败 {key}: {e}") from e
    
    def register_keys(self, keys: Dict[str, AccessType]) -> None:
        """
        批量注册黑板变量
        
        Args:
            keys: 变量名和访问权限的字典
            
        Raises:
            BlackboardError: 任一注册失败时抛出
        """
        for key, access in keys.items():
            self.register_key(key, access)
    
    def set(self, key: str, value: Any) -> None:
        """
        设置黑板变量值
        
        Args:
            key: 变量名
            value: 变量值
            
        Raises:
            BlackboardError: 设置失败时抛出
        """
        if not self.client:
            logger.error("黑板客户端未初始化")
            raise BlackboardError("黑板客户端未初始化")
            
        try:
            self.client.set(key, value)
            # 如果值太长，只显示摘要信息
            value_str = str(value)
            if len(value_str) > 200:
                if isinstance(value, dict):
                    logger.info(f"SetBB: {key} = <dict with {len(value)} keys>")
                elif isinstance(value, list):
                    logger.info(f"SetBB: {key} = <list with {len(value)} items>")
                else:
                    logger.info(f"SetBB: {key} = <{type(value).__name__} (length: {len(value_str)})>")
            else:
                logger.info(f"SetBB: {key} = {value}")
        except BlackboardError:
            raise
        except Exception as e:
            logger.error(f"SetBB failed {key}: {e}")
            raise BlackboardError(f"SetBB failed {key}: {e}") from e
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        获取黑板变量值
        
        Args:
            key: 变量名
            default: 默认值
            
        Returns:
            变量值或默认值
        """
        if not self.client:
            logger.error("黑板客户端未初始化")
            return default
            
        try:
            value = self.client.get(key)
            # 如果值太长，只显示摘要信息
            value_str = str(value)
            if len(value_str) > 100:
                if isinstance(value, dict):
                    logger.info(f"GetBB: {key} = <dict with {len(value)} keys>")
                elif isinstance(value, list):
                    logger.info(f"GetBB: {key} = <list with {len(value)} items>")
                else:
                    logger.info(f"GetBB: {key} = <{type(value).__name__} (length: {len(value_str)})>")
            else:
                logger.info(f"GetBB: {key} = {value}")
            return value
        except Exception as e:
            logger.error(f"GetBB failed {key}: {e}")
            return default
    
    def exists(self, key: str) -> bool:
        """
        检查黑板变量是否存在
        
        Args:
            key: 变量名
            
        Returns:
            是否存在
        """
        if not self.client:
            return False
            
        try:
            return self.client.exists(key)
        except Exception as e:
            logger.error(f"检查黑板变量失败 {key}: {e}")
            return False
    
    def delete(self, key: str) -> None:
        """
        删除黑板变量
        
        Args:
            key: 变量名
            
        Raises:
            BlackboardError: 删除失败时抛出
        """
        if not self.client:
            logger.error("黑板客户端未初始化")
            raise BlackboardError("黑板客户端未初始化")
            
        try:
            self.client.delete(key)
            logger.info(f"删除黑板变量: {key}")
        except Exception as e:
            logger.error(f"删除黑板变量失败 {key}: {e}")
            raise BlackboardError(f"删除黑板变量失败 {key}: {e}") from e
    
    def get_all_variables(self) -> Dict[str, Any]:
        """
        获取所有黑板变量
        
        Returns:
            所有变量的字典
        """
        if not self.client:
            return {}
            
        try:
            # 使用全局黑板方法获取所有变量
            return py_trees.blackboard.Blackboard.storage.copy()
        except Exception as e:
            logger.error(f"获取所有黑板变量失败: {e}")
            return {}
    
    def clear_all(self) -> None:
        """
        清空所有黑板数据
        
        Raises:
            BlackboardError: 清空失败时抛出
        """
        try:
            py_trees.blackboard.Blackboard.clear()
            logger.info("清空所有黑板数据")
        except Exception as e:
            logger.error(f"清空黑板数据失败: {e}")
            raise BlackboardError(f"清空黑板数据失败: {e}") from e
    
    def enable_activity_stream(self, max_size: int = 500) -> None:
        """
        启用黑板活动流
        
        Args:
            max_size: 最大活动记录数
            
        Raises:
            BlackboardError: 启用失败时抛出
        """
        try:
            py_trees.blackboard.Blackboard.enable_activity_stream(max_size)
            logger.info(f"启用黑板活动流 (max_size: {max_size})")
        except Exception as e:
            logger.error(f"启用黑板活动流失败: {e}")
            raise BlackboardError(f"启用黑板活动流失败: {e}") from e
    
    def disable_activity_stream(self) -> None:
        """
        禁用黑板活动流
        
        Raises:
            BlackboardError: 禁用失败时抛出
        """
        try:
            py_trees.blackboard.Blackboard.disable_activity_stream()
            logger.info("禁用黑板活动流")
        except Exception as e:
            logger.error(f"禁用黑板活动流失败: {e}")
            raise BlackboardError(f"禁用黑板活动流失败: {e}") from e
    
    def get_activity_stream(self) -> List[Any]:
        """
        获取黑板活动流
        
        Returns:
            活动流列表
        """
        try:
            if py_trees.blackboard.Blackboard.activity_stream:
                return list(py_trees.blackboard.Blackboard.activity_stream)
            return []
        except Exception as e:
            logger.error(f"获取黑板活动流失败: {e}")
            return []
    
    def display_blackboard(self) -> str:
        """
        显示黑板内容
        
        Returns:
            黑板内容的字符串表示
        """
        try:
            return py_trees.display.unicode_blackboard()
        except Exception as e:
            logger.error(f"显示黑板内容失败: {e}")
            return "无法显示黑板内容"
    
    def __str__(self) -> str:
        """字符串表示"""
        return f"BlackboardManager(name={self.name}, namespace={self.namespace}, registered_keys={len(self.registered_keys)})"
