#!/usr/bin/env python3
"""
视觉行为节点
基于py_trees的行为树节点，封装视觉识别操作
支持异步非阻塞执行，将识别结果保存到黑板
"""

from tkinter import N
import py_trees
import logging
import time
import threading
from typing import Dict, Any, Optional, List
from ..clients.vision.vision_client import ZMQVisionClient
from ..clients.tf_client import TFClient
from ..blackboard_manager import BlackboardManager, AccessType, BlackboardError

logger = logging.getLogger(__name__)


class VisionPoseEstimationMask(py_trees.behaviour.Behaviour):
    """视觉姿态估计行为节点 - 异步执行"""
    
    def __init__(self, 
                 vision_client: ZMQVisionClient, 
                 blackboard_manager: BlackboardManager,
                 tf_server_ip: str = "192.168.56.13",
                 tf_server_port: int = 5609,
                 camera_name: str = None, 
                 target_frame: str = None,
                 allowed_categories: List[str] = [],
                 min_score: float = 0.8,
                 max_num: int = 10,
                 mask_params: Optional[Dict[str, Any]] = None,
                 pose_params: Optional[Dict[str, Any]] = None,
                 name: str = "VisionPoseEstimationMask"):
        """
        初始化视觉姿态估计行为（使用掩码位姿估计方法）
        
        Args:
            vision_client: 视觉客户端
            blackboard_manager: 黑板管理器
            tf_server_ip: TF服务器IP地址
            tf_server_port: TF服务器端口
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            target_frame: 最终目标坐标系（默认 "folder_base"）
            allowed_categories: 允许的类别列表（保留用于兼容性）
            min_score: 最小置信度（用于mask_params，如果mask_params为None）
            max_num: 最大检测数量（用于mask_params，如果mask_params为None）
            mask_params: 掩码检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            name: 行为名称
        """

        # 验证必填参数
        if not camera_name or camera_name.strip() == "":
            raise ValueError("camera_name 参数不能为空")
        if not target_frame or target_frame.strip() == "":
            raise ValueError("target_frame 参数不能为空")

        super().__init__(name=name)
        self.vision_client = vision_client
        self.blackboard_manager = blackboard_manager
        self.tf_server_ip = tf_server_ip
        self.tf_server_port = tf_server_port
        self.camera_name = camera_name
        self.target_frame = target_frame
        self.allowed_categories = allowed_categories
        self.min_score = min_score
        self.max_num = max_num
        self.mask_params = mask_params
        self.pose_params = pose_params
        
        # TF客户端（延迟初始化）
        self.tf_client: Optional[TFClient] = None
        
        # 异步执行相关
        self.task_thread: Optional[threading.Thread] = None
        self.task_result: Optional[Dict[str, Any]] = None
        self.task_started = False
        self.task_completed = False
        self.start_time: Optional[float] = None
        self.timeout = 30.0  # 30秒超时
        
    def _get_camera_frame(self) -> str:
        """
        根据相机名称获取对应的坐标系名称
        
        Returns:
            str: 相机坐标系名称（如 "left_camera_link" 或 "right_camera_link"）
        """
        # 支持新的相机名称映射
        if self.camera_name == "left_camera":
            return "left_camera_link"
        elif self.camera_name == "right_camera":
            return "right_camera_link"
        else:
            # 默认情况，假设相机坐标系名称为 camera_name + "_link"
            return f"{self.camera_name}_link"
    
    def setup(self, **kwargs):
        """初始化行为"""
        camera_frame = self._get_camera_frame()
        camera_config = VisionBehavior._get_camera_mask_config(self.camera_name)
        logger.info(f"设置视觉姿态估计（掩码方法）: 相机={self.camera_name}, 相机坐标系={camera_frame}, 目标坐标系={self.target_frame}")
        logger.info(f"相机话题: RGB={camera_config['rgb_topic_name']}, Depth={camera_config['depth_topic_name']}")
        logger.info(f"检测参数: 最小置信度={self.min_score}, 最大数量={self.max_num}")
        if self.mask_params:
            logger.info(f"自定义mask_params: {self.mask_params}")
        if self.pose_params:
            logger.info(f"自定义pose_params: {self.pose_params}")
        
        # 初始化TF客户端
        try:
            self.tf_client = TFClient(
                server_ip=self.tf_server_ip,
                server_port=self.tf_server_port,
                timeout=5
            )
            logger.info(f"TF客户端已连接: {self.tf_server_ip}:{self.tf_server_port}")
        except Exception as e:
            logger.error(f"TF客户端初始化失败: {e}")
            self.tf_client = None
        
        # 注册黑板写权限
        self.blackboard_manager.register_key("vision_results", AccessType.WRITE)
        
    def initialise(self):
        """行为开始执行"""
        assert not self.task_started, "状态异常，有任务在执行"
        logger.info(f"开始执行视觉姿态估计: {self.camera_name}")
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
            logger.error(f"视觉姿态估计超时 ({self.timeout}秒)")
            return py_trees.common.Status.FAILURE
            
        # 检查任务是否完成
        if self.task_completed:
            if self.task_result and self.task_result.get("success", False):
                # 处理并保存结果到黑板
                if self._process_and_save_results():
                    logger.info("视觉姿态估计成功")
                    return py_trees.common.Status.SUCCESS
                else:
                    logger.error("保存视觉识别结果到黑板失败")
                    return py_trees.common.Status.FAILURE
            else:
                error_msg = self.task_result.get("message", "未知错误") if self.task_result else "任务失败"
                logger.error(f"视觉姿态估计失败: {error_msg}")
                return py_trees.common.Status.FAILURE
                
        # 任务仍在执行中
        return py_trees.common.Status.RUNNING
        
    def _execute_task(self):
        """在后台线程中执行实际任务"""
        try:
            # 获取相机配置
            camera_config = VisionBehavior._get_camera_mask_config(self.camera_name)
            camera_frame = self._get_camera_frame()
            
            logger.info(f"发送视觉请求: 相机={self.camera_name}, 目标坐标系={self.target_frame}")
            logger.info(f"相机配置: RGB={camera_config['rgb_topic_name']}, "
                       f"Depth={camera_config['depth_topic_name']}")
            
            # 准备mask_params（如果为None，使用默认值，包含min_score和max_num）
            mask_params = self.mask_params
            if mask_params is None:
                mask_params = {
                    "min_score": self.min_score,
                    "max_num": self.max_num,
                }
            
            # 准备pose_params（如果为None，使用默认值；如果已提供，确保target_frame使用外部传入的目标坐标系）
            pose_params = self.pose_params
            if pose_params is not None:
                # 如果用户提供了自定义pose_params，确保target_frame使用外部传入的目标坐标系
                pose_params = pose_params.copy()  # 避免修改原始字典
                pose_params["target_frame"] = self.target_frame
            # 如果为None，vision_client会在内部使用默认值，并设置target_frame为self.target_frame
            
            # 调用掩码位姿估计方法
            self.task_result = self.vision_client.vision_pose_estimation_mask(
                pose_estimation_service_name=camera_config["pose_estimation_service_name"],
                rgb_topic_name=camera_config["rgb_topic_name"],
                depth_topic_name=camera_config["depth_topic_name"],
                camera_info_topic_name=camera_config["camera_info_topic_name"],
                mask_service_name=camera_config["mask_service_name"],
                mask_params=mask_params,
                pose_params=pose_params,
                target_frame=self.target_frame  # 使用外部传入的目标坐标系
            )
        except Exception as e:
            logger.error(f"视觉姿态估计异常: {e}")
            import traceback
            traceback.print_exc()
            self.task_result = {"success": False, "message": str(e)}
        finally:
            self.task_completed = True
            
    def _quaternion_multiply(self, q1: List[float], q2: List[float]) -> List[float]:
        """
        四元数乘法 [x, y, z, w]
        
        Args:
            q1: 第一个四元数 [x, y, z, w]
            q2: 第二个四元数 [x, y, z, w]
            
        Returns:
            相乘结果 [x, y, z, w]
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        
        return [x, y, z, w]
    
    def _quaternion_rotate_point(self, point: List[float], q: List[float]) -> List[float]:
        """
        使用四元数旋转点
        
        Args:
            point: 点坐标 [x, y, z]
            q: 四元数 [x, y, z, w]
            
        Returns:
            旋转后的点 [x, y, z]
        """
        # 将点转换为四元数 [x, y, z, 0]
        p = [point[0], point[1], point[2], 0]
        
        # 计算 q * p * q^-1
        # q^-1 = [-x, -y, -z, w] (单位四元数的共轭)
        q_conj = [-q[0], -q[1], -q[2], q[3]]
        
        # q * p
        temp = self._quaternion_multiply(q, p)
        # (q * p) * q^-1
        result = self._quaternion_multiply(temp, q_conj)
        
        return [result[0], result[1], result[2]]
    
    def _transform_pose(self, pose: List[float], transform: Dict[str, Any]) -> List[float]:
        """
        使用TF变换来转换位姿
        
        Args:
            pose: 原始位姿 [x, y, z, qx, qy, qz, qw]
            transform: TF变换 {'translation': {'x', 'y', 'z'}, 'rotation': {'x', 'y', 'z', 'w'}}
            
        Returns:
            转换后的位姿 [x, y, z, qx, qy, qz, qw]
        """
        # 提取变换参数
        trans = transform['translation']
        rot = transform['rotation']
        
        t = [trans['x'], trans['y'], trans['z']]
        q_tf = [rot['x'], rot['y'], rot['z'], rot['w']]
        
        # 原始位置和旋转
        p = [pose[0], pose[1], pose[2]]
        q_obj = [pose[3], pose[4], pose[5], pose[6]]
        
        # 转换位置: p_new = q_tf * p * q_tf^-1 + t
        p_rotated = self._quaternion_rotate_point(p, q_tf)
        p_new = [p_rotated[0] + t[0], p_rotated[1] + t[1], p_rotated[2] + t[2]]
        
        # 转换旋转: q_new = q_tf * q_obj
        q_new = self._quaternion_multiply(q_tf, q_obj)
        
        return [p_new[0], p_new[1], p_new[2], q_new[0], q_new[1], q_new[2], q_new[3]]
    
    def _process_and_save_results(self) -> bool:
        """处理识别结果并保存到黑板
        
        Returns:
            bool: 是否成功保存
        """
        try:
            if not self.task_result or not self.task_result.get("success"):
                return False
                
            # 提取检测数据
            # 数据结构支持两种格式:
            # 1. data = {'vision_final_result': {'poses': [...]}}  (字典格式)
            # 2. data = {'vision_final_result': [...]}  (列表格式)
            data = self.task_result.get("data", {})
            if not isinstance(data, dict):
                logger.error(f"数据格式错误: 期望字典类型，实际为 {type(data)}")
                return False
            
            vision_final_result = data.get("vision_final_result", {})
            
            # 处理两种格式
            if isinstance(vision_final_result, dict) and "poses" in vision_final_result:
                # 格式1: {'vision_final_result': {'poses': [...]}}
                detection_data = vision_final_result.get("poses", [])
            elif isinstance(vision_final_result, list):
                # 格式2: {'vision_final_result': [...]}
                detection_data = vision_final_result
            else:
                logger.error(f"vision_final_result 格式错误: 期望字典(包含poses键)或列表类型，实际为 {type(vision_final_result)}")
                return False
            
            if not isinstance(detection_data, list):
                logger.error(f"检测数据格式错误: 期望列表类型，实际为 {type(detection_data)}")
                return False
            
            # 处理每个检测结果
            processed_results = []
            for detection in detection_data:
                # 确保 detection 是字典类型
                if not isinstance(detection, dict):
                    logger.warning(f"跳过非字典类型的检测结果: {type(detection)}, 值: {detection}")
                    continue
                # 处理类别名称：去掉前缀（如 "7|spoon_handle" -> "spoon_handle"）
                category = detection.get("category", "")
                if "|" in category:
                    category = category.split("|", 1)[1]  # 只分割第一个"|"，取后面部分
                
                # 获取位姿（视觉服务已返回目标坐标系下的位姿）
                pose = detection.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
                result_frame_id = detection.get("frame_id", self.target_frame)
                
                # 直接使用视觉服务返回的位姿（已在目标坐标系下）
                # 如果返回的frame_id与目标坐标系不一致，记录警告但继续使用
                if result_frame_id != self.target_frame:
                    logger.warning(f"返回的坐标系 {result_frame_id} 与目标坐标系 {self.target_frame} 不一致，"
                                 f"但坐标变换应在服务端完成，直接使用返回的位姿")
                
                # 使用目标坐标系作为最终frame_id
                final_frame = self.target_frame
                
                processed_detection = {
                    "category": category,
                    "confidence": detection.get("confidence", 0.0),
                    "frame_id": final_frame,
                    "pose": pose,  # [x, y, z, qx, qy, qz, qw] - 已在目标坐标系下
                    "scale": detection.get("scale", [1.0, 1.0, 1.0]),  # [sx, sy, sz]
                    "timestamp": time.time()  # 添加时间戳
                }
                processed_results.append(processed_detection)
                
            # 保存到黑板
            vision_results = {
                "success": True,
                "message": self.task_result.get("message", ""),
                "detections": processed_results,
                "detection_count": len(processed_results),
                "camera_name": self.camera_name,
                "target_frame": self.target_frame,
                "timestamp": time.time()
            }
            
            self.blackboard_manager.set("vision_results", vision_results)
            logger.info(f"视觉识别结果已保存到黑板: 检测到 {len(processed_results)} 个对象 (坐标系: {self.target_frame})")
            
            # 打印检测结果摘要
            for i, detection in enumerate(processed_results):
                pose = detection['pose']
                logger.info(f"  检测 {i+1}: 类别={detection['category']}, 置信度={detection['confidence']:.3f}, "
                          f"位置=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
            
            return True
                
        except BlackboardError as e:
            logger.error(f"保存视觉识别结果到黑板失败: {e}")
            return False
        except Exception as e:
            logger.error(f"处理视觉识别结果时出错: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        # 关闭TF客户端
        if self.tf_client:
            try:
                self.tf_client.close()
                self.tf_client = None
            except Exception as e:
                logger.warning(f"关闭TF客户端时出错: {e}")
        
        # 重置任务状态
        self.task_started = False
        
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("视觉姿态估计行为完成")
        else:
            logger.warning("视觉姿态估计行为失败")


class VisionPoseEstimationBox(py_trees.behaviour.Behaviour):
    """视觉姿态估计行为节点 - 异步执行"""
    
    def __init__(self, 
                 vision_client: ZMQVisionClient, 
                 blackboard_manager: BlackboardManager,
                 tf_server_ip: str = "192.168.56.13",
                 tf_server_port: int = 5609,
                 camera_name: str = None, 
                 target_frame: str = None,
                 allowed_categories: List[str] = [],
                 min_score: float = 0.8,
                 max_num: int = 10,
                 box_params: Optional[Dict[str, Any]] = None,
                 pose_params: Optional[Dict[str, Any]] = None,
                 intrinsic_file_path: Optional[str] = None,
                 pose_board_in_camera: Optional[List[float]] = None,
                 name: str = "VisionPoseEstimationBox"):
        """
        初始化视觉姿态估计行为（使用模板匹配位姿估计方法）
        
        Args:
            vision_client: 视觉客户端
            blackboard_manager: 黑板管理器
            tf_server_ip: TF服务器IP地址
            tf_server_port: TF服务器端口
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            target_frame: 最终目标坐标系（默认 "folder_base"）
            allowed_categories: 允许的类别列表（保留用于兼容性）
            min_score: 最小置信度（用于box_params，如果box_params为None）
            max_num: 最大检测数量（用于box_params，如果box_params为None）
            box_params: Box检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            intrinsic_file_path: 相机内参文件路径（None 表示不使用）
            pose_board_in_camera: 板子在相机坐标系下的位姿 [x, y, z, qx, qy, qz, qw]（None 表示不使用）
            name: 行为名称
        """

        # 验证必填参数
        if not camera_name or camera_name.strip() == "":
            raise ValueError("camera_name 参数不能为空")
        if not target_frame or target_frame.strip() == "":
            raise ValueError("target_frame 参数不能为空")

        super().__init__(name=name)
        self.vision_client = vision_client
        self.blackboard_manager = blackboard_manager
        self.tf_server_ip = tf_server_ip
        self.tf_server_port = tf_server_port
        self.camera_name = camera_name
        self.target_frame = target_frame
        self.allowed_categories = allowed_categories
        self.min_score = min_score
        self.max_num = max_num
        self.box_params = box_params
        self.pose_params = pose_params
        self.intrinsic_file_path = intrinsic_file_path
        self.pose_board_in_camera = pose_board_in_camera
        
        # TF客户端（延迟初始化）
        self.tf_client: Optional[TFClient] = None
        
        # 异步执行相关
        self.task_thread: Optional[threading.Thread] = None
        self.task_result: Optional[Dict[str, Any]] = None
        self.task_started = False
        self.task_completed = False
        self.start_time: Optional[float] = None
        self.timeout = 30.0  # 30秒超时
        
    def _get_camera_frame(self) -> str:
        """
        根据相机名称获取对应的坐标系名称
        
        Returns:
            str: 相机坐标系名称（如 "left_camera_link" 或 "right_camera_link"）
        """
        # 支持新的相机名称映射
        if self.camera_name == "left_camera":
            return "left_camera_link"
        elif self.camera_name == "right_camera":
            return "right_camera_link"
        else:
            # 默认情况，假设相机坐标系名称为 camera_name + "_link"
            return f"{self.camera_name}_link"
    
    def setup(self, **kwargs):
        """初始化行为"""
        camera_frame = self._get_camera_frame()
        camera_config = VisionBehavior._get_camera_box_config(self.camera_name)
        logger.info(f"设置视觉姿态估计（模板匹配方法）: 相机={self.camera_name}, 相机坐标系={camera_frame}, 目标坐标系={self.target_frame}")
        logger.info(f"相机话题: Image={camera_config['image_topic_name']}")
        logger.info(f"检测参数: 最小置信度={self.min_score}, 最大数量={self.max_num}")
        if self.box_params:
            logger.info(f"自定义box_params: {self.box_params}")
        if self.pose_params:
            logger.info(f"自定义pose_params: {self.pose_params}")
        if self.intrinsic_file_path:
            logger.info(f"相机内参文件路径: {self.intrinsic_file_path}")
        if self.pose_board_in_camera:
            logger.info(f"板子在相机坐标系下的位姿: {self.pose_board_in_camera}")
        
        # 初始化TF客户端
        try:
            self.tf_client = TFClient(
                server_ip=self.tf_server_ip,
                server_port=self.tf_server_port,
                timeout=5
            )
            logger.info(f"TF客户端已连接: {self.tf_server_ip}:{self.tf_server_port}")
        except Exception as e:
            logger.error(f"TF客户端初始化失败: {e}")
            self.tf_client = None
        
        # 注册黑板写权限
        self.blackboard_manager.register_key("vision_results", AccessType.WRITE)
        
    def initialise(self):
        """行为开始执行"""
        assert not self.task_started, "状态异常，有任务在执行"
        logger.info(f"开始执行视觉姿态估计: {self.camera_name}")
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
            logger.error(f"视觉姿态估计超时 ({self.timeout}秒)")
            return py_trees.common.Status.FAILURE
            
        # 检查任务是否完成
        if self.task_completed:
            if self.task_result and self.task_result.get("success", False):
                # 处理并保存结果到黑板
                if self._process_and_save_results():
                    logger.info("视觉姿态估计成功")
                    return py_trees.common.Status.SUCCESS
                else:
                    logger.error("保存视觉识别结果到黑板失败")
                    return py_trees.common.Status.FAILURE
            else:
                error_msg = self.task_result.get("message", "未知错误") if self.task_result else "任务失败"
                logger.error(f"视觉姿态估计失败: {error_msg}")
                return py_trees.common.Status.FAILURE
                
        # 任务仍在执行中
        return py_trees.common.Status.RUNNING
        
    def _execute_task(self):
        """在后台线程中执行实际任务"""
        try:
            # 获取相机配置
            camera_config = VisionBehavior._get_camera_box_config(self.camera_name)
            camera_frame = self._get_camera_frame()
            
            logger.info(f"发送视觉请求: 相机={self.camera_name}, 目标坐标系={self.target_frame}")
            logger.info(f"相机配置: Image={camera_config['image_topic_name']}")
            
            # 准备box_params（如果为None，使用默认值，包含min_score和max_num）
            box_params = self.box_params
            if box_params is None:
                box_params = {
                    "min_score": self.min_score,
                    "max_num": self.max_num,
                }
            
            # 准备pose_params（如果为None，使用默认值；如果已提供，确保target_frame使用外部传入的目标坐标系）
            pose_params = self.pose_params
            if pose_params is not None:
                # 如果用户提供了自定义pose_params，确保target_frame使用外部传入的目标坐标系
                pose_params = pose_params.copy()  # 避免修改原始字典
                pose_params["target_frame"] = self.target_frame
            # 如果为None，vision_client会在内部使用默认值，并设置target_frame为self.target_frame
            
            # 调用模板匹配位姿估计方法
            self.task_result = self.vision_client.vision_template_pose_estimation_box(
                pose_estimation_service_name=camera_config["pose_estimation_service_name"],
                image_topic_name=camera_config["image_topic_name"],
                box_service_name=camera_config["box_service_name"],
                box_params=box_params,
                pose_params=pose_params,
                target_frame=self.target_frame,  # 使用外部传入的目标坐标系
                intrinsic_file_path=self.intrinsic_file_path,
                pose_board_in_camera=self.pose_board_in_camera
            )
        except Exception as e:
            logger.error(f"视觉姿态估计异常: {e}")
            import traceback
            traceback.print_exc()
            self.task_result = {"success": False, "message": str(e)}
        finally:
            self.task_completed = True
    
    def _process_and_save_results(self) -> bool:
        """处理识别结果并保存到黑板
        
        Returns:
            bool: 是否成功保存
        """
        try:
            if not self.task_result or not self.task_result.get("success"):
                return False
                
            # 提取检测数据
            # 数据结构支持两种格式:
            # 1. data = {'vision_final_result': {'poses': [...]}}  (字典格式)
            # 2. data = {'vision_final_result': [...]}  (列表格式)
            data = self.task_result.get("data", {})
            if not isinstance(data, dict):
                logger.error(f"数据格式错误: 期望字典类型，实际为 {type(data)}")
                return False
            
            vision_final_result = data.get("vision_final_result", {})
            
            # 处理两种格式
            if isinstance(vision_final_result, dict) and "poses" in vision_final_result:
                # 格式1: {'vision_final_result': {'poses': [...]}}
                detection_data = vision_final_result.get("poses", [])
            elif isinstance(vision_final_result, list):
                # 格式2: {'vision_final_result': [...]}
                detection_data = vision_final_result
            else:
                logger.error(f"vision_final_result 格式错误: 期望字典(包含poses键)或列表类型，实际为 {type(vision_final_result)}")
                return False
            
            if not isinstance(detection_data, list):
                logger.error(f"检测数据格式错误: 期望列表类型，实际为 {type(detection_data)}")
                return False
            
            # 处理每个检测结果
            processed_results = []
            for detection in detection_data:
                # 确保 detection 是字典类型
                if not isinstance(detection, dict):
                    logger.warning(f"跳过非字典类型的检测结果: {type(detection)}, 值: {detection}")
                    continue
                # 处理类别名称：去掉前缀（如 "7|spoon_handle" -> "spoon_handle"）
                category = detection.get("category", "")
                if "|" in category:
                    category = category.split("|", 1)[1]  # 只分割第一个"|"，取后面部分
                
                # 获取位姿（视觉服务已返回目标坐标系下的位姿）
                pose = detection.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
                result_frame_id = detection.get("frame_id", self.target_frame)
                
                # 直接使用视觉服务返回的位姿（已在目标坐标系下）
                # 如果返回的frame_id与目标坐标系不一致，记录警告但继续使用
                if result_frame_id != self.target_frame:
                    logger.warning(f"返回的坐标系 {result_frame_id} 与目标坐标系 {self.target_frame} 不一致，"
                                 f"但坐标变换应在服务端完成，直接使用返回的位姿")
                
                # 使用目标坐标系作为最终frame_id
                final_frame = self.target_frame
                
                processed_detection = {
                    "category": category,
                    "confidence": detection.get("confidence", 0.0),
                    "frame_id": final_frame,
                    "pose": pose,  # [x, y, z, qx, qy, qz, qw] - 已在目标坐标系下
                    "scale": detection.get("scale", [1.0, 1.0, 1.0]),  # [sx, sy, sz]
                    "timestamp": time.time()  # 添加时间戳
                }
                processed_results.append(processed_detection)
            
            # 保存到黑板
            vision_results = {
                "success": True,
                "message": self.task_result.get("message", ""),
                "detections": processed_results,
                "detection_count": len(processed_results),
                "camera_name": self.camera_name,
                "target_frame": self.target_frame,
                "timestamp": time.time()
            }
            
            self.blackboard_manager.set("vision_results", vision_results)
            logger.info(f"视觉识别结果已保存到黑板: 检测到 {len(processed_results)} 个对象 (坐标系: {self.target_frame})")
            
            # 打印检测结果摘要
            for i, detection in enumerate(processed_results):
                pose = detection['pose']
                logger.info(f"  检测 {i+1}: 类别={detection['category']}, 置信度={detection['confidence']:.3f}, "
                          f"位置=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
            
            return True
                
        except BlackboardError as e:
            logger.error(f"保存视觉识别结果到黑板失败: {e}")
            return False
        except Exception as e:
            logger.error(f"处理视觉识别结果时出错: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        # 关闭TF客户端
        if self.tf_client:
            try:
                self.tf_client.close()
                self.tf_client = None
            except Exception as e:
                logger.warning(f"关闭TF客户端时出错: {e}")
        
        # 重置任务状态
        self.task_started = False
        
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("视觉姿态估计行为完成")
        else:
            logger.warning("视觉姿态估计行为失败")


class VisionPoseEstimationOBBMask(py_trees.behaviour.Behaviour):
    """视觉姿态估计行为节点 - 异步执行（使用OBB和掩码的位姿估计方法）"""
    
    def __init__(self, 
                 vision_client: ZMQVisionClient, 
                 blackboard_manager: BlackboardManager,
                 tf_server_ip: str = "192.168.56.13",
                 tf_server_port: int = 5609,
                 camera_name: str = None, 
                 target_frame: str = None,
                 allowed_categories: List[str] = [],
                 min_score: float = 0.8,
                 max_num: int = 10,
                 obb_params: Optional[Dict[str, Any]] = None,
                 mask_params: Optional[Dict[str, Any]] = None,
                 merge_params: Optional[Dict[str, Any]] = None,
                 pose_params: Optional[Dict[str, Any]] = None,
                 name: str = "VisionPoseEstimationOBBMask"):
        """
        初始化视觉姿态估计行为（使用OBB和掩码的位姿估计方法）
        
        Args:
            vision_client: 视觉客户端
            blackboard_manager: 黑板管理器
            tf_server_ip: TF服务器IP地址
            tf_server_port: TF服务器端口
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            target_frame: 最终目标坐标系（默认 "folder_base"）
            allowed_categories: 允许的类别列表（保留用于兼容性）
            min_score: 最小置信度（用于obb_params和mask_params，如果它们为None）
            max_num: 最大检测数量（用于obb_params和mask_params，如果它们为None）
            obb_params: OBB检测参数（None 表示使用默认值）
            mask_params: 掩码检测参数（None 表示使用默认值）
            merge_params: 数据融合参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            name: 行为名称
        """

        # 验证必填参数
        if not camera_name or camera_name.strip() == "":
            raise ValueError("camera_name 参数不能为空")
        if not target_frame or target_frame.strip() == "":
            raise ValueError("target_frame 参数不能为空")

        super().__init__(name=name)
        self.vision_client = vision_client
        self.blackboard_manager = blackboard_manager
        self.tf_server_ip = tf_server_ip
        self.tf_server_port = tf_server_port
        self.camera_name = camera_name
        self.target_frame = target_frame
        self.allowed_categories = allowed_categories
        self.min_score = min_score
        self.max_num = max_num
        self.obb_params = obb_params
        self.mask_params = mask_params
        self.merge_params = merge_params
        self.pose_params = pose_params
        
        # TF客户端（延迟初始化）
        self.tf_client: Optional[TFClient] = None
        
        # 异步执行相关
        self.task_thread: Optional[threading.Thread] = None
        self.task_result: Optional[Dict[str, Any]] = None
        self.task_started = False
        self.task_completed = False
        self.start_time: Optional[float] = None
        self.timeout = 30.0  # 30秒超时
        
    def _get_camera_frame(self) -> str:
        """
        根据相机名称获取对应的坐标系名称
        
        Returns:
            str: 相机坐标系名称（如 "left_camera_link" 或 "right_camera_link"）
        """
        # 支持新的相机名称映射
        if self.camera_name == "left_camera":
            return "left_camera_link"
        elif self.camera_name == "right_camera":
            return "right_camera_link"
        else:
            # 默认情况，假设相机坐标系名称为 camera_name + "_link"
            return f"{self.camera_name}_link"
    
    def setup(self, **kwargs):
        """初始化行为"""
        camera_frame = self._get_camera_frame()
        camera_config = VisionBehavior._get_camera_obb_mask_config(self.camera_name)
        logger.info(f"设置视觉姿态估计（OBB和掩码方法）: 相机={self.camera_name}, 相机坐标系={camera_frame}, 目标坐标系={self.target_frame}")
        logger.info(f"相机话题: RGB={camera_config['rgb_topic_name']}, Depth={camera_config['depth_topic_name']}")
        logger.info(f"检测参数: 最小置信度={self.min_score}, 最大数量={self.max_num}")
        if self.obb_params:
            logger.info(f"自定义obb_params: {self.obb_params}")
        if self.mask_params:
            logger.info(f"自定义mask_params: {self.mask_params}")
        if self.merge_params:
            logger.info(f"自定义merge_params: {self.merge_params}")
        if self.pose_params:
            logger.info(f"自定义pose_params: {self.pose_params}")
        
        # 初始化TF客户端
        try:
            self.tf_client = TFClient(
                server_ip=self.tf_server_ip,
                server_port=self.tf_server_port,
                timeout=5
            )
            logger.info(f"TF客户端已连接: {self.tf_server_ip}:{self.tf_server_port}")
        except Exception as e:
            logger.error(f"TF客户端初始化失败: {e}")
            self.tf_client = None
        
        # 注册黑板写权限
        self.blackboard_manager.register_key("vision_results", AccessType.WRITE)
        
    def initialise(self):
        """行为开始执行"""
        assert not self.task_started, "状态异常，有任务在执行"
        logger.info(f"开始执行视觉姿态估计: {self.camera_name}")
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
            logger.error(f"视觉姿态估计超时 ({self.timeout}秒)")
            return py_trees.common.Status.FAILURE
            
        # 检查任务是否完成
        if self.task_completed:
            if self.task_result and self.task_result.get("success", False):
                # 处理并保存结果到黑板
                if self._process_and_save_results():
                    logger.info("视觉姿态估计成功")
                    return py_trees.common.Status.SUCCESS
                else:
                    logger.error("保存视觉识别结果到黑板失败")
                    return py_trees.common.Status.FAILURE
            else:
                error_msg = self.task_result.get("message", "未知错误") if self.task_result else "任务失败"
                logger.error(f"视觉姿态估计失败: {error_msg}")
                return py_trees.common.Status.FAILURE
                
        # 任务仍在执行中
        return py_trees.common.Status.RUNNING
        
    def _execute_task(self):
        """在后台线程中执行实际任务"""
        try:
            # 获取相机配置
            camera_config = VisionBehavior._get_camera_obb_mask_config(self.camera_name)
            camera_frame = self._get_camera_frame()
            
            logger.info(f"发送视觉请求: 相机={self.camera_name}, 目标坐标系={self.target_frame}")
            logger.info(f"相机配置: RGB={camera_config['rgb_topic_name']}, "
                       f"Depth={camera_config['depth_topic_name']}")
            
            # 准备obb_params（如果为None，使用默认值，包含min_score和max_num）
            obb_params = self.obb_params
            if obb_params is None:
                obb_params = {
                    "min_score": self.min_score,
                    "max_num": self.max_num,
                }
            
            # 准备mask_params（如果为None，使用默认值，包含min_score和max_num）
            mask_params = self.mask_params
            if mask_params is None:
                mask_params = {
                    "min_score": self.min_score,
                    "max_num": self.max_num,
                }
            
            # 准备merge_params（如果为None，使用默认值）
            merge_params = self.merge_params
            
            # 准备pose_params（如果为None，使用默认值；如果已提供，确保target_frame使用外部传入的目标坐标系）
            pose_params = self.pose_params
            if pose_params is not None:
                # 如果用户提供了自定义pose_params，确保target_frame使用外部传入的目标坐标系
                pose_params = pose_params.copy()  # 避免修改原始字典
                pose_params["target_frame"] = self.target_frame
            # 如果为None，vision_client会在内部使用默认值，并设置target_frame为self.target_frame
            
            # 调用OBB和掩码位姿估计方法
            self.task_result = self.vision_client.vision_pose_estimation_obb_mask(
                pose_estimation_service_name=camera_config["pose_estimation_service_name"],
                rgb_topic_name=camera_config["rgb_topic_name"],
                depth_topic_name=camera_config["depth_topic_name"],
                camera_info_topic_name=camera_config["camera_info_topic_name"],
                obb_service_name=camera_config["obb_service_name"],
                mask_service_name=camera_config["mask_service_name"],
                merge_data_service_name=camera_config["merge_data_service_name"],
                merge_params=merge_params,
                pose_params=pose_params,
                obb_params=obb_params,
                mask_params=mask_params,
                target_frame=self.target_frame  # 使用外部传入的目标坐标系
            )
        except Exception as e:
            logger.error(f"视觉姿态估计异常: {e}")
            import traceback
            traceback.print_exc()
            self.task_result = {"success": False, "message": str(e)}
        finally:
            self.task_completed = True
    
    def _process_and_save_results(self) -> bool:
        """处理识别结果并保存到黑板
        
        Returns:
            bool: 是否成功保存
        """
        try:
            if not self.task_result or not self.task_result.get("success"):
                return False
                
            # 提取检测数据
            # 数据结构支持两种格式:
            # 1. data = {'vision_final_result': {'poses': [...]}}  (字典格式)
            # 2. data = {'vision_final_result': [...]}  (列表格式)
            data = self.task_result.get("data", {})
            if not isinstance(data, dict):
                logger.error(f"数据格式错误: 期望字典类型，实际为 {type(data)}")
                return False
            
            vision_final_result = data.get("vision_final_result", {})
            
            # 处理两种格式
            if isinstance(vision_final_result, dict) and "poses" in vision_final_result:
                # 格式1: {'vision_final_result': {'poses': [...]}}
                detection_data = vision_final_result.get("poses", [])
            elif isinstance(vision_final_result, list):
                # 格式2: {'vision_final_result': [...]}
                detection_data = vision_final_result
            else:
                logger.error(f"vision_final_result 格式错误: 期望字典(包含poses键)或列表类型，实际为 {type(vision_final_result)}")
                return False
            
            if not isinstance(detection_data, list):
                logger.error(f"检测数据格式错误: 期望列表类型，实际为 {type(detection_data)}")
                return False
            
            # 处理每个检测结果
            processed_results = []
            for detection in detection_data:
                # 确保 detection 是字典类型
                if not isinstance(detection, dict):
                    logger.warning(f"跳过非字典类型的检测结果: {type(detection)}, 值: {detection}")
                    continue
                # 处理类别名称：去掉前缀（如 "7|spoon_handle" -> "spoon_handle"）
                category = detection.get("category", "")
                if "|" in category:
                    category = category.split("|", 1)[1]  # 只分割第一个"|"，取后面部分
                
                # 获取位姿（视觉服务已返回目标坐标系下的位姿）
                pose = detection.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
                result_frame_id = detection.get("frame_id", self.target_frame)
                
                # 直接使用视觉服务返回的位姿（已在目标坐标系下）
                # 如果返回的frame_id与目标坐标系不一致，记录警告但继续使用
                if result_frame_id != self.target_frame:
                    logger.warning(f"返回的坐标系 {result_frame_id} 与目标坐标系 {self.target_frame} 不一致，"
                                 f"但坐标变换应在服务端完成，直接使用返回的位姿")
                
                # 使用目标坐标系作为最终frame_id
                final_frame = self.target_frame
                
                processed_detection = {
                    "category": category,
                    "confidence": detection.get("confidence", 0.0),
                    "frame_id": final_frame,
                    "pose": pose,  # [x, y, z, qx, qy, qz, qw] - 已在目标坐标系下
                    "scale": detection.get("scale", [1.0, 1.0, 1.0]),  # [sx, sy, sz]
                    "timestamp": time.time()  # 添加时间戳
                }
                processed_results.append(processed_detection)
            
            # 保存到黑板
            vision_results = {
                "success": True,
                "message": self.task_result.get("message", ""),
                "detections": processed_results,
                "detection_count": len(processed_results),
                "camera_name": self.camera_name,
                "target_frame": self.target_frame,
                "timestamp": time.time()
            }
            
            self.blackboard_manager.set("vision_results", vision_results)
            logger.info(f"视觉识别结果已保存到黑板: 检测到 {len(processed_results)} 个对象 (坐标系: {self.target_frame})")
            
            # 打印检测结果摘要
            for i, detection in enumerate(processed_results):
                pose = detection['pose']
                logger.info(f"  检测 {i+1}: 类别={detection['category']}, 置信度={detection['confidence']:.3f}, "
                          f"位置=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
            
            return True
                
        except BlackboardError as e:
            logger.error(f"保存视觉识别结果到黑板失败: {e}")
            return False
        except Exception as e:
            logger.error(f"处理视觉识别结果时出错: {e}")
            import traceback
            traceback.print_exc()
            return False
            
    def terminate(self, new_status: py_trees.common.Status):
        """行为结束"""
        # 关闭TF客户端
        if self.tf_client:
            try:
                self.tf_client.close()
                self.tf_client = None
            except Exception as e:
                logger.warning(f"关闭TF客户端时出错: {e}")
        
        # 重置任务状态
        self.task_started = False
        
        if new_status == py_trees.common.Status.SUCCESS:
            logger.info("视觉姿态估计行为完成")
        else:
            logger.warning("视觉姿态估计行为失败")


class VisionBehavior:
    """视觉行为节点工厂类"""
    
    def __init__(self, 
                 vision_left_client: ZMQVisionClient,
                 vision_right_client: ZMQVisionClient,
                 blackboard_manager: BlackboardManager,
                 tf_server_ip: str = "192.168.31.206",
                 tf_server_port: int = 5609,
                 vision_client: Optional[ZMQVisionClient] = None):
        """
        初始化视觉行为工厂
        
        Args:
            vision_left_client: 左手视觉客户端
            vision_right_client: 右手视觉客户端
            blackboard_manager: 黑板管理器
            tf_server_ip: TF服务器IP地址
            tf_server_port: TF服务器端口
            vision_client: 单一视觉客户端（向后兼容，如果提供则左右都使用它）
        """
        # 如果提供了单一客户端（向后兼容），左右都使用它
        if vision_client is not None:
            self.vision_left_client = vision_client
            self.vision_right_client = vision_client
        else:
            self.vision_left_client = vision_left_client
            self.vision_right_client = vision_right_client
        
        # 为了向后兼容，保留 vision_client 属性（默认指向 right）
        self.vision_client = self.vision_right_client
        
        self.blackboard_manager = blackboard_manager
        self.tf_server_ip = tf_server_ip
        self.tf_server_port = tf_server_port
    
    def _get_vision_client(self, camera_name: str) -> ZMQVisionClient:
        """
        根据相机名称获取对应的视觉客户端
        
        Args:
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            
        Returns:
            ZMQVisionClient: 对应的视觉客户端
        """
        if camera_name and camera_name.startswith("left"):
            return self.vision_left_client
        elif camera_name and camera_name.startswith("right"):
            return self.vision_right_client
        else:
            # 默认使用右手客户端
            logger.warning(f"无法从相机名称 '{camera_name}' 确定使用哪个视觉客户端，默认使用右手客户端")
            return self.vision_right_client
    
    @staticmethod
    def _get_camera_mask_config(camera_name: str) -> Dict[str, str]:
        """
        根据相机名称获取掩码位姿估计的参数配置
        
        Args:
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            
        Returns:
            Dict: 包含相机相关参数的字典
        """
        # 相机名称到参数的映射配置
        if camera_name == "left_camera":
            return {
                "pose_estimation_service_name": "/vision/pose_estimation",
                "rgb_topic_name": "/left_camera/color/image_raw",
                "depth_topic_name": "/left_camera/aligned_depth_to_color/image_raw",
                "camera_info_topic_name": "/left_camera/color/camera_info",
                "mask_service_name": "/vision/detection_mask_cub",
            }
        elif camera_name == "right_camera":
            return {
                "pose_estimation_service_name": "/vision/pose_estimation",
                "rgb_topic_name": "/right_camera/color/image_raw",
                "depth_topic_name": "/right_camera/aligned_depth_to_color/image_raw",
                "camera_info_topic_name": "/right_camera/color/camera_info",
                "mask_service_name": "/vision/detection_mask_cub",
            }
        else:
            # 默认配置，使用相机名称作为前缀
            camera_prefix = camera_name.replace("_camera", "").replace("camera", "")
            if camera_prefix and not camera_prefix.startswith("/"):
                camera_prefix = f"/camera_{camera_prefix}"
            elif not camera_prefix:
                camera_prefix = "/camera"
            
            return {
                "pose_estimation_service_name": "/vision/pose_estimation",
                "rgb_topic_name": f"{camera_prefix}/color/image_raw",
                "depth_topic_name": f"{camera_prefix}/aligned_depth_to_color/image_raw",
                "camera_info_topic_name": f"{camera_prefix}/color/camera_info",
                "mask_service_name": "/vision/detection_mask_cub",
            }
    
    @staticmethod
    def _get_camera_box_config(camera_name: str) -> Dict[str, str]:
        """
        根据相机名称获取模板匹配位姿估计的参数配置
        
        Args:
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            
        Returns:
            Dict: 包含相机相关参数的字典
        """
        # 相机名称到参数的映射配置
        if camera_name == "left_camera":
            return {
                "pose_estimation_service_name": "/vision/template_pose_estimation",
                "image_topic_name": "/left_camera/color/image_raw",
                "box_service_name": "/vision/detection_box",
            }
        elif camera_name == "right_camera":
            return {
                "pose_estimation_service_name": "/vision/template_pose_estimation",
                "image_topic_name": "/right_camera/color/image_raw",
                "box_service_name": "/vision/detection_box",
            }
        else:
            # 默认配置，使用相机名称作为前缀
            camera_prefix = camera_name.replace("_camera", "").replace("camera", "")
            if camera_prefix and not camera_prefix.startswith("/"):
                camera_prefix = f"/camera_{camera_prefix}"
            elif not camera_prefix:
                camera_prefix = "/camera"
            
            return {
                "pose_estimation_service_name": "/vision/template_pose_estimation",
                "image_topic_name": f"{camera_prefix}/color/image_raw",
                "box_service_name": "/vision/detection_box",
            }
    
    @staticmethod
    def _get_camera_obb_mask_config(camera_name: str) -> Dict[str, str]:
        """
        根据相机名称获取OBB和掩码位姿估计的参数配置
        
        Args:
            camera_name: 相机名称（如 "left_camera" 或 "right_camera"）
            
        Returns:
            Dict: 包含相机相关参数的字典
        """
        # 相机名称到参数的映射配置
        if camera_name == "left_camera":
            return {
                "pose_estimation_service_name": "/vision/pose_estimation",
                "rgb_topic_name": "/left_camera/color/image_raw",
                "depth_topic_name": "/left_camera/aligned_depth_to_color/image_raw",
                "camera_info_topic_name": "/left_camera/color/camera_info",
                "obb_service_name": "/vision/detection_box_welding_part",
                "mask_service_name": "/vision/detection_mask_welding",
                "merge_data_service_name": "/type_trans/merge_json_data",
            }
        elif camera_name == "right_camera":
            return {
                "pose_estimation_service_name": "/vision/pose_estimation",
                "rgb_topic_name": "/right_camera/color/image_raw",
                "depth_topic_name": "/right_camera/aligned_depth_to_color/image_raw",
                "camera_info_topic_name": "/right_camera/color/camera_info",
                "obb_service_name": "/vision/detection_box_welding_part",
                "mask_service_name": "/vision/detection_mask_welding",
                "merge_data_service_name": "/type_trans/merge_json_data",
            }
        else:
            # 默认配置，使用相机名称作为前缀
            camera_prefix = camera_name.replace("_camera", "").replace("camera", "")
            if camera_prefix and not camera_prefix.startswith("/"):
                camera_prefix = f"/camera_{camera_prefix}"
            elif not camera_prefix:
                camera_prefix = "/camera"
            
            return {
                "pose_estimation_service_name": "/vision/pose_estimation",
                "rgb_topic_name": f"{camera_prefix}/color/image_raw",
                "depth_topic_name": f"{camera_prefix}/aligned_depth_to_color/image_raw",
                "camera_info_topic_name": f"{camera_prefix}/color/camera_info",
                "obb_service_name": "/vision/detection_box_welding_part",
                "mask_service_name": "/vision/detection_mask_welding",
                "merge_data_service_name": "/type_trans/merge_json_data",
            }
    
    def pose_estimation_box(self, 
                       camera_name: str, 
                       target_frame: str,
                       allowed_categories: List[str] = [],
                       min_score: float = 0.8,
                       max_num: int = 10,
                       box_params: Optional[Dict[str, Any]] = None,
                       pose_params: Optional[Dict[str, Any]] = None,
                       intrinsic_file_path: Optional[str] = None,
                       pose_board_in_camera: Optional[List[float]] = None,
                       name: str = "VisionPoseEstimationBox") -> VisionPoseEstimationBox:
        """
        创建视觉姿态估计行为（使用模板匹配位姿估计方法）
        
        Args:
            camera_name: 相机名称（必填，如 "left_camera" 或 "right_camera"）
            target_frame: 最终目标坐标系（必填，如 "base_link" 或 "folder_base"）
            allowed_categories: 允许的类别列表（保留用于兼容性）
            min_score: 最小置信度（用于box_params，如果box_params为None）
            max_num: 最大检测数量（用于box_params，如果box_params为None）
            box_params: Box检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            intrinsic_file_path: 相机内参文件路径（None 表示不使用）
            pose_board_in_camera: 板子在相机坐标系下的位姿 [x, y, z, qx, qy, qz, qw]（None 表示不使用）
            name: 行为名称
            
        Returns:
            VisionPoseEstimationBox: 视觉姿态估计行为节点
        """
        return VisionPoseEstimationBox(
            self._get_vision_client(camera_name), 
            self.blackboard_manager,
            tf_server_ip=self.tf_server_ip,
            tf_server_port=self.tf_server_port,
            camera_name=camera_name,
            target_frame=target_frame,
            allowed_categories=allowed_categories,
            min_score=min_score,
            max_num=max_num,
            box_params=box_params,
            pose_params=pose_params,
            intrinsic_file_path=intrinsic_file_path,
            pose_board_in_camera=pose_board_in_camera,
            name=name
        )
    
    def pose_estimation_mask(self, 
                            camera_name: str, 
                            target_frame: str,
                            mask_params: Optional[Dict[str, Any]] = None,
                            pose_params: Optional[Dict[str, Any]] = None,
                            name: str = "VisionPoseEstimationMask") -> VisionPoseEstimationMask:
        """
        创建基于掩码的视觉姿态估计行为
        
        Args:
            camera_name: 相机名称（必填，如 "left_camera" 或 "right_camera"）
            target_frame: 最终目标坐标系（必填，如 "base_link" 或 "folder_base"）
            mask_params: 掩码检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            name: 行为名称
            
        Returns:
            VisionPoseEstimationMask: 视觉姿态估计行为节点
        """
        return VisionPoseEstimationMask(
            self._get_vision_client(camera_name), 
            self.blackboard_manager,
            tf_server_ip=self.tf_server_ip,
            tf_server_port=self.tf_server_port,
            camera_name=camera_name,
            target_frame=target_frame,
            allowed_categories=[],
            min_score=0.8,
            max_num=10,
            mask_params=mask_params,
            pose_params=pose_params,
            name=name
        )
    
    def pose_estimation_obb_mask(self, 
                                 camera_name: str, 
                                 target_frame: str,
                                 allowed_categories: List[str] = [],
                                 min_score: float = 0.8,
                                 max_num: int = 10,
                                 obb_params: Optional[Dict[str, Any]] = None,
                                 mask_params: Optional[Dict[str, Any]] = None,
                                 merge_params: Optional[Dict[str, Any]] = None,
                                 pose_params: Optional[Dict[str, Any]] = None,
                                 name: str = "VisionPoseEstimationOBBMask") -> VisionPoseEstimationOBBMask:
        """
        创建基于OBB和掩码的视觉姿态估计行为
        
        Args:
            camera_name: 相机名称（必填，如 "left_camera" 或 "right_camera"）
            target_frame: 最终目标坐标系（必填，如 "base_link" 或 "folder_base"）
            allowed_categories: 允许的类别列表（保留用于兼容性）
            min_score: 最小置信度（用于obb_params和mask_params，如果它们为None）
            max_num: 最大检测数量（用于obb_params和mask_params，如果它们为None）
            obb_params: OBB检测参数（None 表示使用默认值）
            mask_params: 掩码检测参数（None 表示使用默认值）
            merge_params: 数据融合参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            name: 行为名称
            
        Returns:
            VisionPoseEstimationOBBMask: 视觉姿态估计行为节点
        """
        return VisionPoseEstimationOBBMask(
            self._get_vision_client(camera_name), 
            self.blackboard_manager,
            tf_server_ip=self.tf_server_ip,
            tf_server_port=self.tf_server_port,
            camera_name=camera_name,
            target_frame=target_frame,
            allowed_categories=allowed_categories,
            min_score=min_score,
            max_num=max_num,
            obb_params=obb_params,
            mask_params=mask_params,
            merge_params=merge_params,
            pose_params=pose_params,
            name=name
        )
