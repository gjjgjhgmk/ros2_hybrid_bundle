#!/usr/bin/env python3
"""
行为树管理器
负责创建、执行和监控行为树
"""

import py_trees
import py_trees.console as console
import py_trees.display
import py_trees.visitors
import yaml
import logging
import time
import threading
import os
from typing import Dict, Any, Optional, List
from tqdm import tqdm
from .clients.arm import UrMoveClient
from .clients.vision.vision_client import ZMQVisionClient
from .clients.gripper import GripperZMQClient
from .behaviors.arm_move_behavior import ArmMoveBehavior
from .behaviors.arm_waypoint_behavior import ArmWaypointBehavior
from .behaviors.vision_behavior import VisionBehavior
from .behaviors.gripper_behavior import GripperBehavior
from .behaviors.utility_behavior import UtilityBehavior
from .blackboard_manager import BlackboardManager, BlackboardError
from .behaviors.blackboard_behavior import BlackboardBehavior
from .arm_waypoint_manager import ArmWaypointManager

logger = logging.getLogger(__name__)


class BehaviorTreeManager:
    """行为树管理器"""
    
    def __init__(self, config_path: str = "config.yaml", waypoints_path: str = "waypoints.json", show_progress: bool = True, show_tree: bool = False):
        """初始化行为树管理器"""
        self.config_path = config_path
        self.waypoints_path = waypoints_path
        self.config = self._load_config()
        self.tree: Optional[py_trees.trees.BehaviourTree] = None
        self.arm_waypoint_client: Optional[UrMoveClient] = None
        self.vision_client: Optional[ZMQVisionClient] = None
        self.vision_left_client: Optional[ZMQVisionClient] = None
        self.vision_right_client: Optional[ZMQVisionClient] = None
        self.arm_move_behavior: Optional[ArmMoveBehavior] = None
        self.arm_waypoint_behavior: Optional[ArmWaypointBehavior] = None
        self.vision_behavior: Optional[VisionBehavior] = None
        self.gripper_behavior: Optional[GripperBehavior] = None
        self.utility_behavior: Optional[UtilityBehavior] = None
        self.blackboard_manager: Optional[BlackboardManager] = None
        self.blackboard_behavior: Optional[BlackboardBehavior] = None
        self.waypoint_manager: Optional[ArmWaypointManager] = None
        
        # 自定义行为注册表
        self.custom_behaviors: Dict[str, Any] = {}
        
        self.is_running = False
        self._execution_thread: Optional[threading.Thread] = None
        self.show_progress = show_progress
        self.show_tree = show_tree
        self._progress_bar: Optional[tqdm] = None
        self._start_time: Optional[float] = None
        self._snapshot_visitor: Optional[py_trees.visitors.SnapshotVisitor] = None
        self._last_tree_display_time: float = 0
        self._last_status_update_time: float = 0
        
        # 设置日志
        self._setup_logging()

        self._initialize()
        
    def _load_config(self) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            logger.info(f"配置文件加载成功: {self.config_path}")
            return config
        except Exception as e:
            logger.error(f"配置文件加载失败: {e}")
            raise Exception("配置文件加载失败")
    
            
    def _setup_logging(self):
        """设置日志"""
        log_config = self.config.get('logging', {})
        level = getattr(logging, log_config.get('level', 'INFO').upper())
        logging.basicConfig(
            level=level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_config.get('file', 'ur_bt.log')),
                logging.StreamHandler()
            ]
        )
        
    def _initialize(self):
        """初始化系统"""
        try:
            # 创建手臂客户端
            arm_config = self.config.get('zmq', {}).get('arm', {})
            
            # ur_move配置
            ur_move_config = arm_config.get('ur_move', {})
            ur_move_host = ur_move_config.get('host', 'localhost')
            ur_move_timeout_ms = ur_move_config.get('timeout_ms', 60000)
            
            # 执行服务器配置（用于远程执行）
            executor_config = ur_move_config.get('executor', {})
            left_arm_executor_host = executor_config.get('left_arm_host', None)
            right_arm_executor_host = executor_config.get('right_arm_host', None)
            
            # 创建UR Move客户端（使用 plan_and_execute_remote 方法）
            self.arm_waypoint_client = UrMoveClient(
                server_host=ur_move_host,
                timeout_ms=ur_move_timeout_ms,
                left_arm_executor_host=left_arm_executor_host,
                right_arm_executor_host=right_arm_executor_host
            )
            if not self.arm_waypoint_client.connect():
                logger.warning("UR Move客户端连接失败，将在首次使用时重试")
            
            # 创建视觉客户端（支持左右两个视觉服务）
            vision_config = self.config.get('zmq', {}).get('vision', {})
            
            # 支持新的配置结构（left/right）和旧的配置结构（host/port）
            if 'left' in vision_config and 'right' in vision_config:
                # 新配置结构：左右分离
                left_vision_config = vision_config['left']
                right_vision_config = vision_config['right']
                
                left_vision_host = left_vision_config.get('host', 'localhost')
                left_vision_port = left_vision_config.get('port', 7020)
                left_vision_timeout = left_vision_config.get('timeout', 30)
                
                right_vision_host = right_vision_config.get('host', 'localhost')
                right_vision_port = right_vision_config.get('port', 7020)
                right_vision_timeout = right_vision_config.get('timeout', 30)
                
                self.vision_left_client = ZMQVisionClient(host=left_vision_host, port=left_vision_port, timeout=left_vision_timeout)
                self.vision_right_client = ZMQVisionClient(host=right_vision_host, port=right_vision_port, timeout=right_vision_timeout)
                
                # 为了向后兼容，保留 vision_client 指向 right（默认）
                self.vision_client = self.vision_right_client
            else:
                # 旧配置结构：单一视觉服务
                vision_host = vision_config.get('host', 'localhost')
                vision_port = vision_config.get('port', 7020)
                vision_timeout = vision_config.get('timeout', 30)
                
                self.vision_client = ZMQVisionClient(host=vision_host, port=vision_port, timeout=vision_timeout)
                # 左右都使用同一个客户端（向后兼容）
                self.vision_left_client = self.vision_client
                self.vision_right_client = self.vision_client
            
            # 创建黑板管理器
            self.blackboard_manager = BlackboardManager("BehaviorTreeManager")
            self.blackboard_manager.initialize()
            
            # 读取执行模式配置。
            # concurrent_remote_execution 仅在 use_remote_execution=True 时生效，
            # 用于控制是否并发向左右臂 executor 下发轨迹。
            use_remote_execution = ur_move_config.get('use_remote_execution', True)
            concurrent_remote_execution = ur_move_config.get('concurrent_remote_execution', False)
            
            # 创建手臂行为工厂
            self.arm_move_behavior = ArmMoveBehavior(
                self.arm_waypoint_client, 
                self.blackboard_manager,
                use_remote_execution=use_remote_execution,
                concurrent_remote_execution=concurrent_remote_execution
            )
            self.arm_waypoint_behavior = ArmWaypointBehavior(self.blackboard_manager)
            
            # 创建视觉行为工厂（带TF支持）
            tf_config = self.config.get('zmq', {}).get('tf', {})
            tf_host = tf_config.get('host', '192.168.31.206')
            tf_port = tf_config.get('port', 5609)
            
            self.vision_behavior = VisionBehavior(
                vision_left_client=self.vision_left_client,
                vision_right_client=self.vision_right_client,
                blackboard_manager=self.blackboard_manager,
                tf_server_ip=tf_host,
                tf_server_port=tf_port
            )
            
            # 创建夹爪客户端（可选）
            gripper_config = self.config.get('zmq', {}).get('gripper', {})
            
            # 左手夹爪配置
            if 'left' in gripper_config:
                left_gripper_config = gripper_config['left']
                left_gripper_host = left_gripper_config.get('host', 'localhost')
                left_gripper_port = left_gripper_config.get('port', 5630)
                left_gripper_timeout = left_gripper_config.get('timeout_ms', 10000)
                try:
                    self.gripper_left_client = GripperZMQClient(
                        server_host=left_gripper_host,
                        port=left_gripper_port,
                        gripper_name='left',
                        timeout_ms=left_gripper_timeout
                    )
                    if self.gripper_left_client.connect():
                        logger.info("左手夹爪客户端已连接")
                    else:
                        logger.warning("左手夹爪客户端连接失败")
                        self.gripper_left_client = None
                except Exception as e:
                    logger.warning(f"左手夹爪客户端初始化失败: {e}")
                    self.gripper_left_client = None
            
            # 右手夹爪配置
            if 'right' in gripper_config:
                right_gripper_config = gripper_config['right']
                right_gripper_host = right_gripper_config.get('host', 'localhost')
                right_gripper_port = right_gripper_config.get('port', 5640)
                right_gripper_timeout = right_gripper_config.get('timeout_ms', 10000)
                try:
                    self.gripper_right_client = GripperZMQClient(
                        server_host=right_gripper_host,
                        port=right_gripper_port,
                        gripper_name='right',
                        timeout_ms=right_gripper_timeout
                    )
                    if self.gripper_right_client.connect():
                        logger.info("右手夹爪客户端已连接")
                    else:
                        logger.warning("右手夹爪客户端连接失败")
                        self.gripper_right_client = None
                except Exception as e:
                    logger.warning(f"右手夹爪客户端初始化失败: {e}")
                    self.gripper_right_client = None
            
            # 创建夹爪行为工厂
            self.gripper_behavior = GripperBehavior(self.gripper_left_client, self.gripper_right_client)
            
            # 创建实用工具行为工厂
            self.utility_behavior = UtilityBehavior()
            
            # 创建黑板行为工厂
            self.blackboard_behavior = BlackboardBehavior(self.blackboard_manager)
            
            # 创建waypoint管理器
            self.waypoint_manager = ArmWaypointManager(self.waypoints_path, self.blackboard_manager)
            
            logger.info("系统初始化完成")
            
        except Exception as e:
            logger.error(f"系统初始化失败: {e}")
            raise e
    
    def register_behavior(self, name: str, behavior_factory: Any) -> None:
        """
        注册自定义行为工厂
        
        Args:
            name: 行为名称
            behavior_factory: 行为工厂实例
        """
        self.custom_behaviors[name] = behavior_factory
        logger.info(f"注册自定义行为: {name}")
    
    def get_behavior(self, name: str) -> Any:
        """
        获取注册的行为工厂
        
        Args:
            name: 行为名称
            
        Returns:
            行为工厂实例
            
        Raises:
            KeyError: 如果行为未注册
        """
        if name not in self.custom_behaviors:
            raise KeyError(f"自定义行为 '{name}' 未注册")
        return self.custom_behaviors[name]
    
    
    def _create_tree(self, behaviors: List[py_trees.behaviour.Behaviour]) -> py_trees.trees.BehaviourTree:
        """创建行为树"""
        # 创建根节点
        root = py_trees.composites.Sequence(name="CustomSequence", memory=True)
        root.add_children(behaviors)
        
        # 创建行为树
        tree = py_trees.trees.BehaviourTree(root)
        return tree
        
    def _set_tree(self, tree: py_trees.trees.BehaviourTree):
        """设置行为树"""
        self.tree = tree
        
        # 初始化行为树（调用所有行为的setup方法）
        self.tree.setup(timeout=15)
        
        # 添加SnapshotVisitor用于状态收集
        if self.show_tree:
            self._snapshot_visitor = py_trees.visitors.SnapshotVisitor()
            self.tree.visitors.append(self._snapshot_visitor)
        
        logger.info("行为树设置完成")
    
    def execute(self, behaviors: List[py_trees.behaviour.Behaviour], wait: bool = True) -> bool:
        """
        执行行为树（一键式接口）
        
        Args:
            behaviors: 行为节点列表
            wait: 是否等待执行完成，默认True
            
        Returns:
            bool: 执行是否成功（如果wait=True）或启动是否成功（如果wait=False）
        """
        try:
            # 1. 创建行为树
            tree = self._create_tree(behaviors)
            
            # 2. 设置行为树
            self._set_tree(tree)
            
            # 3. 启动执行
            if not self.start():
                logger.error("行为树启动失败")
                return False
            
            # 4. 如果需要等待，则等待执行完成
            if wait:
                while self.is_running:
                    time.sleep(0.1)
                
                # 返回执行结果
                if self.tree and self.tree.root:
                    return self.tree.root.status == py_trees.common.Status.SUCCESS
                return False
            
            return True
            
        except Exception as e:
            logger.error(f"执行行为树异常: {e}")
            return False
        
    def start(self):
        """开始执行行为树"""
        if not self.tree:
            logger.error("未设置行为树")
            return False
            
        if self.is_running:
            logger.warning("行为树已在运行")
            return False
            
        self.is_running = True
        self._start_time = time.time()
        
        # 初始化进度条
        if self.show_progress:
            self._progress_bar = tqdm(total=100, desc="执行中", unit="s", 
                                    bar_format='{desc} | {elapsed} | {postfix}', 
                                    ncols=100, leave=False, dynamic_ncols=True)
        
        self._execution_thread = threading.Thread(target=self._execution_loop, daemon=True)
        self._execution_thread.start()
        logger.info("行为树开始执行")
        return True
        
    def stop(self):
        """停止执行行为树"""
        self.is_running = False
        if self._execution_thread:
            self._execution_thread.join(timeout=5.0)
        
        # 关闭进度条
        if self._progress_bar:
            self._progress_bar.close()
            self._progress_bar = None
            
        logger.info("行为树停止执行")
        
    def _execution_loop(self):
        """行为树执行循环"""
        tick_rate = self.config.get('behavior_tree', {}).get('tick_rate', 10)
        timeout = self.config.get('behavior_tree', {}).get('timeout', 100.0)
        
        while self.is_running:
            try:
                # 检查超时
                if self._start_time and (time.time() - self._start_time) > timeout:
                    logger.warning("行为树执行超时")
                    if self._progress_bar is not None:
                        self._progress_bar.set_description("执行超时")
                        self._progress_bar.set_postfix_str("停止执行")
                    break
                    
                elapsed = time.time() - self._start_time

                # 更新进度条和状态显示（每0.5秒更新一次状态信息）
                if (self._progress_bar is not None and elapsed > 0 and
                    elapsed - self._last_status_update_time >= 0.5):
                    
                    # 获取当前状态信息
                    status_info = self._get_status_info()
                    self._progress_bar.set_description(f"[{elapsed:.1f}s]")
                    self._progress_bar.set_postfix_str(status_info)
                    self._progress_bar.update(1)
                    self._last_status_update_time = elapsed
                    
                # 显示树结构
                if (self.show_tree and self._snapshot_visitor and 
                    elapsed - self._last_tree_display_time >= 1.0 and elapsed > 0):
                    self._display_tree_status()
                    self._last_tree_display_time = elapsed
                    
                # 执行一次tick
                self.tree.tick()
                
                # 检查是否完成
                if self.tree.root.status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE]:
                    logger.info(f"行为树执行完成，状态: {self.tree.root.status}")
                    if self._progress_bar is not None:
                        final_status = "✓ 执行完成" if self.tree.root.status == py_trees.common.Status.SUCCESS else "✗ 执行失败"
                        self._progress_bar.set_description(final_status)
                        self._progress_bar.set_postfix_str("")
                    break
                    
                # 等待下一个tick
                time.sleep(1.0 / tick_rate)
                
            except Exception as e:
                logger.error(f"行为树执行异常: {e}")
                if self._progress_bar is not None:
                    self._progress_bar.set_description("✗ 执行异常")
                    self._progress_bar.set_postfix_str(f"错误: {str(e)[:50]}")
                break
                
        self.is_running = False
    
    def _get_status_info(self) -> str:
        """获取当前状态信息"""
        if not self.tree or not self.tree.root:
            return "执行中"
            
        # 获取根节点状态
        root_status = self.tree.root.status
        status_map = {
            py_trees.common.Status.SUCCESS: "✓ 完成",
            py_trees.common.Status.FAILURE: "✗ 失败", 
            py_trees.common.Status.RUNNING: "▶ 执行中",
            py_trees.common.Status.INVALID: "? 无效"
        }
        
        status_text = status_map.get(root_status, "? 未知")
        
        # 获取当前执行的行为名称
        current_behavior = self._get_current_behavior()
        
        # 获取执行进度
        progress_info = self._get_execution_progress()
        
        # 构建状态信息，突出显示当前执行的行为
        if current_behavior:
            # 如果有当前执行的行为，显示为主要信息
            if progress_info['total_behaviors'] > 0:
                return f"执行: {current_behavior} | 进度: {progress_info['progress_text']} | {status_text}"
            else:
                return f"执行: {current_behavior} | {status_text}"
        else:
            # 如果没有当前执行的行为，显示整体状态
            if progress_info['total_behaviors'] > 0:
                return f"{status_text} | 进度: {progress_info['progress_text']}"
            else:
                return status_text
    
    def _get_current_behavior(self) -> Optional[str]:
        """获取当前执行的行为名称"""
        if not self.tree or not self.tree.root:
            return None
            
        # 遍历树找到正在运行的行为
        def find_running_behavior(node):
            if node.status == py_trees.common.Status.RUNNING:
                # 如果是复合节点（如Sequence），继续查找子节点
                if hasattr(node, 'children') and node.children:
                    for child in node.children:
                        if child.status == py_trees.common.Status.RUNNING:
                            return find_running_behavior(child)
                        elif child.status == py_trees.common.Status.SUCCESS:
                            continue
                        elif child.status == py_trees.common.Status.FAILURE:
                            return child.name
                return node.name
            if hasattr(node, 'children'):
                for child in node.children:
                    result = find_running_behavior(child)
                    if result:
                        return result
            return None
            
        return find_running_behavior(self.tree.root)
    
    def _get_execution_progress(self) -> Dict[str, Any]:
        """获取执行进度信息"""
        if not self.tree or not self.tree.root:
            return {'current_index': 0, 'total_behaviors': 0, 'progress_text': ''}
        
        # 获取所有子行为
        behaviors = []
        if hasattr(self.tree.root, 'children'):
            behaviors = self.tree.root.children
        
        total_behaviors = len(behaviors)
        if total_behaviors == 0:
            return {'current_index': 0, 'total_behaviors': 0, 'progress_text': ''}
        
        # 计算当前执行的行为索引
        current_index = 0
        for i, behavior in enumerate(behaviors):
            if behavior.status == py_trees.common.Status.RUNNING:
                current_index = i + 1
                break
            elif behavior.status == py_trees.common.Status.SUCCESS:
                current_index = i + 1
            elif behavior.status == py_trees.common.Status.FAILURE:
                current_index = i + 1
                break
        
        progress_text = f"{current_index}/{total_behaviors}"
        return {
            'current_index': current_index,
            'total_behaviors': total_behaviors,
            'progress_text': progress_text
        }
    
    def _get_tree_statistics(self) -> Dict[str, int]:
        """获取树状态统计信息"""
        if not self.tree or not self.tree.root:
            return {'total_nodes': 0, 'success_nodes': 0, 'failure_nodes': 0, 'running_nodes': 0, 'invalid_nodes': 0}
        
        stats = {
            'total_nodes': 0,
            'success_nodes': 0,
            'failure_nodes': 0,
            'running_nodes': 0,
            'invalid_nodes': 0
        }
        
        def count_nodes(node):
            stats['total_nodes'] += 1
            
            if node.status == py_trees.common.Status.SUCCESS:
                stats['success_nodes'] += 1
            elif node.status == py_trees.common.Status.FAILURE:
                stats['failure_nodes'] += 1
            elif node.status == py_trees.common.Status.RUNNING:
                stats['running_nodes'] += 1
            elif node.status == py_trees.common.Status.INVALID:
                stats['invalid_nodes'] += 1
            
            # 递归统计子节点
            if hasattr(node, 'children'):
                for child in node.children:
                    count_nodes(child)
        
        count_nodes(self.tree.root)
        return stats
            
    def get_blackboard_manager(self) -> Optional[BlackboardManager]:
        """获取黑板管理器"""
        return self.blackboard_manager
    
    def get_blackboard_behavior(self) -> Optional[BlackboardBehavior]:
        """获取黑板行为工厂"""
        return self.blackboard_behavior
    
    def get_vision_behavior(self) -> Optional[VisionBehavior]:
        """获取视觉行为工厂"""
        return self.vision_behavior
    
    def get_utility_behavior(self) -> Optional[UtilityBehavior]:
        """获取实用工具行为工厂"""
        return self.utility_behavior
    
    def display_blackboard(self) -> str:
        """显示黑板内容"""
        if self.blackboard_manager:
            return self.blackboard_manager.display_blackboard()
        return "黑板管理器未初始化"
    
    def clear_blackboard(self) -> bool:
        """清空黑板数据"""
        if self.blackboard_manager:
            try:
                self.blackboard_manager.clear_all()
                return True
            except BlackboardError as e:
                logger.error(f"清空黑板失败: {e}")
                return False
        return False
    
    def enable_blackboard_activity_stream(self, max_size: int = 500) -> bool:
        """启用黑板活动流"""
        if self.blackboard_manager:
            try:
                self.blackboard_manager.enable_activity_stream(max_size)
                return True
            except BlackboardError as e:
                logger.error(f"启用黑板活动流失败: {e}")
                return False
        return False
    
    def get_blackboard_activity_stream(self) -> List[Any]:
        """获取黑板活动流"""
        if self.blackboard_manager:
            return self.blackboard_manager.get_activity_stream()
        return []
    
    def get_waypoint_manager(self) -> Optional[ArmWaypointManager]:
        """获取waypoint管理器"""
        return self.waypoint_manager
    
    def cleanup(self):
        """清理资源"""
        self.stop()
        if self.arm_waypoint_client:
            self.arm_waypoint_client.close()
        # 关闭视觉客户端（避免重复关闭）
        if self.vision_left_client and self.vision_left_client != self.vision_right_client:
            self.vision_left_client.close()
        if self.vision_right_client:
            self.vision_right_client.close()
        # 如果 vision_client 是独立的（向后兼容的旧配置），也需要关闭
        if self.vision_client and self.vision_client != self.vision_left_client and self.vision_client != self.vision_right_client:
            self.vision_client.close()
        if self.gripper_left_client:
            self.gripper_left_client.close_connection()
        if self.gripper_right_client:
            self.gripper_right_client.close_connection()
        if self.blackboard_manager:
            try:
                self.blackboard_manager.clear_all()
            except BlackboardError as e:
                logger.warning(f"清理黑板时出错: {e}")
        logger.info("系统清理完成")
