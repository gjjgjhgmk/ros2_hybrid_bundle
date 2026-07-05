#!/usr/bin/env python3
"""
实用工具行为节点
提供通用的辅助行为，如等待输入、延时等
"""

import py_trees
import logging
import time
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class WaitForInput(py_trees.behaviour.Behaviour):
    """等待用户输入 - 异步执行"""
    
    def __init__(self, prompt: str = "按回车键继续...", 
                 timeout: Optional[float] = None,
                 name: str = "WaitForInput"):
        """
        初始化等待输入行为
        
        Args:
            prompt: 提示信息
            timeout: 超时时间（秒），None表示无限等待（默认：无限等待）
            name: 行为名称
        """
        super().__init__(name=name)
        self.prompt = prompt
        self.timeout = timeout
        self.task_thread: Optional[threading.Thread] = None
        self.task_completed = False
        self.task_started = False
        self.start_time: Optional[float] = None
        self.user_responded = False
        
    def setup(self, **kwargs):
        """初始化行为"""
        timeout_str = f", 超时: {self.timeout}秒" if self.timeout else ", 无限等待"
        logger.info(f"设置等待输入: {self.prompt}{timeout_str}")
        
    def initialise(self):
        """行为开始执行"""
        if self.task_started:
            logger.warning("状态异常，有任务在执行")
            return
            
        logger.info(f"等待用户输入: {self.prompt}")
        self.task_started = True
        self.task_completed = False
        self.user_responded = False
        self.start_time = time.time()
        
        # 启动异步任务等待输入
        self.task_thread = threading.Thread(target=self._wait_for_input, daemon=True)
        self.task_thread.start()
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        if not self.task_started:
            self.initialise()
            return py_trees.common.Status.RUNNING
            
        # 检查超时
        if self.timeout and self.start_time:
            elapsed = time.time() - self.start_time
            if elapsed > self.timeout:
                logger.warning(f"等待输入超时 ({self.timeout}秒)")
                self.task_started = False
                return py_trees.common.Status.FAILURE
                
        # 检查是否完成
        if self.task_completed:
            self.task_started = False
            if self.user_responded:
                logger.info("用户已响应，继续执行")
                return py_trees.common.Status.SUCCESS
            else:
                logger.warning("等待输入被中断")
                return py_trees.common.Status.FAILURE
                
        # 任务仍在执行中
        return py_trees.common.Status.RUNNING
        
    def _wait_for_input(self):
        """在后台线程中等待用户输入"""
        try:
            print(f"\n{'='*60}")
            print(f"⏸️  {self.prompt}")
            print(f"{'='*60}")
            input()  # 阻塞等待用户输入
            self.user_responded = True
        except Exception as e:
            logger.error(f"等待输入异常: {e}")
            self.user_responded = False
        finally:
            self.task_completed = True
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("等待输入行为完成")
        else:
            logger.warning("等待输入行为失败或被中断")


class Sleep(py_trees.behaviour.Behaviour):
    """延时等待 - 异步执行"""
    
    def __init__(self, duration: float, name: str = "Sleep"):
        """
        初始化延时行为
        
        Args:
            duration: 延时时长（秒）
            name: 行为名称
        """
        super().__init__(name=name)
        self.duration = duration
        self.start_time: Optional[float] = None
        
    def setup(self, **kwargs):
        """初始化行为"""
        logger.info(f"设置延时: {self.duration}秒")
        
    def initialise(self):
        """行为开始执行"""
        logger.info(f"开始延时 {self.duration}秒")
        self.start_time = time.time()
        
    def update(self) -> py_trees.common.Status:
        """更新行为状态"""
        if self.start_time is None:
            self.initialise()
            return py_trees.common.Status.RUNNING
            
        elapsed = time.time() - self.start_time
        
        if elapsed >= self.duration:
            logger.info(f"延时 {self.duration}秒 完成")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        if new_status == py_trees.common.Status.SUCCESS:
            logger.debug("延时行为完成")
        else:
            logger.warning("延时行为被中断")


class UtilityBehavior:
    """实用工具行为节点工厂类"""
    
    def __init__(self):
        """初始化实用工具行为工厂"""
        pass
    
    def wait_for_input(self, prompt: str = "按回车键继续...", 
                      timeout: Optional[float] = None,
                      name: Optional[str] = None) -> WaitForInput:
        """
        创建等待用户输入的行为（默认无限等待）
        
        Args:
            prompt: 提示信息
            timeout: 超时时间（秒），None表示无限等待（默认：None，无限等待）
            name: 行为名称（可选，自动生成）
            
        Returns:
            WaitForInput: 等待输入行为节点
            
        Examples:
            # 无限等待（默认）
            utility.wait_for_input("请按回车继续...")
            
            # 带10秒超时
            utility.wait_for_input("请在10秒内按回车...", timeout=10)
        """
        if name is None:
            timeout_str = f"_timeout{timeout}" if timeout else ""
            name = f"WaitForInput{timeout_str}"
        
        return WaitForInput(prompt, timeout, name)
    
    def sleep(self, duration: float, name: Optional[str] = None) -> Sleep:
        """
        创建延时等待的行为
        
        Args:
            duration: 延时时长（秒）
            name: 行为名称（可选，自动生成）
            
        Returns:
            Sleep: 延时行为节点
        """
        if name is None:
            name = f"Sleep_{duration}s"
        
        return Sleep(duration, name)
