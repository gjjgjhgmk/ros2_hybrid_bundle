"""
统一相机标定管理类

提供统一的标定接口，支持内参和外参标定，
支持多种图像获取方式（ROS2 topic、文件夹、文件），
支持JSON参数配置。
"""

import os
import time
import datetime
import json
import yaml
import logging
import numpy as np
import cv2
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass, field, asdict
from scipy.spatial.transform import Rotation as SciPyRotation
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo
import shutil

IMAGE_FILE_EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".tiff")


class CalibrationIO:
    """通用IO封装（与具体配置解耦）"""

    def __init__(self, logger: logging.Logger):
        self.logger = logger

    def ensure_dir(self, path: str):
        try:
            os.makedirs(path, exist_ok=True)
        except Exception as e:
            self.logger.error(f"创建目录失败: {path}, {e}")
            raise

    def load_yaml(self, file_path: str, default: Optional[Any] = None):
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return (
                data if data is not None else (default if default is not None else {})
            )
        except FileNotFoundError:
            self.logger.error(f"YAML文件不存在: {file_path}")
            return default if default is not None else {}
        except Exception as e:
            self.logger.error(f"读取YAML失败: {file_path}, {e}")
            return default if default is not None else {}

    def save_yaml(self, data: Any, file_path: str):
        try:
            os.makedirs(os.path.dirname(file_path) or ".", exist_ok=True)
            with open(file_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
        except Exception as e:
            self.logger.error(f"写入YAML失败: {file_path}, {e}")
            raise

    def read_text(self, file_path: str) -> str:
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                return f.read()
        except Exception as e:
            self.logger.error(f"读取文本失败: {file_path}, {e}")
            return ""

    def load_json(self, file_path: str, default: Optional[Any] = None):
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except FileNotFoundError:
            self.logger.error(f"JSON文件不存在: {file_path}")
            return default if default is not None else {}
        except Exception as e:
            self.logger.error(f"读取JSON失败: {file_path}, {e}")
            return default if default is not None else {}

    def list_folders(self, root: str = ".") -> List[str]:
        try:
            folders = [
                item
                for item in os.listdir(root)
                if os.path.isdir(os.path.join(root, item))
            ]
            folders.sort(key=str.lower)
            return folders
        except Exception as e:
            self.logger.error(f"获取文件夹列表失败: {root}, {e}")
            return []

    def folder_structure(self, folder_path: str) -> Dict[str, Any]:
        try:
            files, folders = [], []
            for entry in os.listdir(folder_path):
                full = os.path.join(folder_path, entry)
                if os.path.isfile(full):
                    files.append(full)
                elif os.path.isdir(full):
                    folders.append(full)
            files.sort(key=lambda x: os.path.basename(x).lower())
            folders.sort(key=lambda x: os.path.basename(x).lower())
            return {"files": files, "folders": folders}
        except Exception as e:
            self.logger.error(f"获取目录结构失败: {folder_path}, {e}")
            return {"files": [], "folders": []}

    def list_images(self, folder_path: str, extensions: Tuple[str, ...]) -> List[str]:
        try:
            images = []
            for file in os.listdir(folder_path):
                if file.lower().endswith(extensions):
                    images.append(os.path.join(folder_path, file))
            images.sort(key=lambda x: os.path.basename(x).lower())
            return images
        except Exception as e:
            self.logger.error(f"获取图像列表失败: {folder_path}, {e}")
            return []

    def load_camera_params_from_yaml(
        self, camera_info_file: str
    ) -> Optional[Dict[str, Any]]:
        """
        从YAML文件加载相机内参

        参数:
            camera_info_file: camera_info YAML文件路径

        返回:
            包含相机内参的字典，失败返回None
        """
        try:
            if not os.path.exists(camera_info_file):
                self.logger.error(f"内参文件不存在: {camera_info_file}")
                return None

            data = self.load_yaml(camera_info_file, default={})
            if not data:
                self.logger.error(f"内参文件为空: {camera_info_file}")
                return None

            # 提取相机内参矩阵 (3x3)
            if "camera_matrix" not in data:
                self.logger.error("内参文件中没有找到camera_matrix")
                return None
            camera_matrix_data = data["camera_matrix"]["data"]
            camera_matrix = np.array(camera_matrix_data).reshape(3, 3)

            # 提取畸变系数
            if "distortion_coefficients" not in data:
                self.logger.error("内参文件中没有找到distortion_coefficients")
                return None
            dist_coeffs_data = data["distortion_coefficients"]["data"]
            dist_coeffs = np.array(dist_coeffs_data)
            dist_model = data.get("distortion_model", "plumb_bob")

            # 提取图像尺寸
            image_width = data.get("image_width", 0)
            image_height = data.get("image_height", 0)

            result = {
                "camera_matrix": camera_matrix,
                "dist_coeffs": dist_coeffs,
                "dist_model": dist_model,
                "image_width": image_width,
                "image_height": image_height,
            }

            self.logger.info(f"成功加载相机内参文件: {camera_info_file}")
            return result

        except Exception as e:
            self.logger.error(f"加载相机内参文件失败 {camera_info_file}: {e}")
            return None

    def get_images_from_folder(
        self, folder_path: str, extensions: List[str]
    ) -> List[str]:
        """
        从文件夹获取图像路径

        参数:
            folder_path: 文件夹路径
            extensions: 支持的图像扩展名列表（如 ["jpg", "jpeg", "png"]）

        返回:
            图像文件路径列表（已排序）

        异常:
            ValueError: 如果文件夹不存在或没有找到图像
        """
        if not os.path.exists(folder_path):
            raise ValueError(f"文件夹不存在: {folder_path}")
        exts = tuple([f".{ext}" for ext in extensions])
        image_files = self.list_images(folder_path, exts)
        if not image_files:
            raise ValueError(f"文件夹中没有找到图像: {folder_path}")
        return sorted(image_files)

    def get_images_from_file(self, file_path: str) -> List[str]:
        """
        从文件获取图像路径列表（每行一个路径）

        参数:
            file_path: 文件路径

        返回:
            图像路径列表（去除空行和首尾空格）

        异常:
            ValueError: 如果文件不存在
        """
        if not os.path.exists(file_path):
            raise ValueError(f"文件不存在: {file_path}")
        content = self.read_text(file_path)
        return [line.strip() for line in content.splitlines() if line.strip()]


# ROS2相关导入
import rclpy
from .ros2_image_subscriber import ROS2ImageSubscriber
from .ros2_tf_subscriber import ROS2TFSubscriber
from .ros2_joint_subscriber import ROS2JointSubscriber

# 标定模块导入
from .board_detector import BoardDetector, BoardConfig
from .calibration_handeye import HandEyeCalibrator
from .calibration_intrinsic import IntrinsicCalibrator

from .calobration_sampler import CalibrationSampler


@dataclass
class ImageConfig:
    """图像配置"""

    topic: str = "/camera/color/image_raw"  # ROS2 topic名称
    camera_info_topic: str = ""  # ROS2 CameraInfo topic名称；为空时由图像topic自动推导
    extensions: List[str] = field(
        default_factory=lambda: ["jpg", "jpeg", "png"]
    )  # 支持的图像文件扩展名
    timeout: float = 10.0  # 图像超时时间（秒）
    camera_frame: str = "camera_color_optical_frame"  # 相机坐标系
    board_frame: str = "charuco"  # 标定板坐标系


@dataclass
class EndEffectorConfig:
    """末端执行器配置"""

    base_frame: str = "base_link"  # 基座坐标系（通常是机器人基座）
    tool_frame: str = "tool0"  # 工具坐标系（通常是工具末端）
    joint_topic: str = "/joint_states"  # 关节状态topic
    use_sim_time: bool = False  # 使用仿真时间


@dataclass
class ExtrinsicConfig:
    """外参标定配置"""

    handeye_type: str = (
        "eye_to_hand"  # 手眼标定类型：eye_to_hand（眼在手外）或 eye_in_hand（眼在手上）
    )
    handeye_method: str = (
        "Tsai"  # 手眼标定算法：Tsai, Park, Horaud, Andreff, Daniilidis
    )


@dataclass
class RobotMoveConfig:
    """机器人运动配置"""

    server_address: str = "tcp://localhost:5605"
    timeout_ms: int = 60000
    group: str = "left_arm"
    planner: str = "ptp"  # ptp or lin or ompl
    type: str = "joint"  # joint or cart
    max_velocity_scaling_factor: float = 0.3
    max_acceleration_scaling_factor: float = 0.3
    description: str = "关节空间路径点"
    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
    )


@dataclass
class CalibrationSamplerConfig:
    """标定采样器配置"""

    group_name: str = "left_arm"
    use_sim_time: bool = True
    service_name: str = "example_move_robot"
    camera_base_pose: list[float] = field(
        default_factory=lambda: [0.6, 0.7, 1.36, 3.14159, 0.0, -1.57079]
    )
    position_range: list[list[float]] = field(
        default_factory=lambda: [[-0.15, 0.15], [-0.15, 0.15], [-0.1, 0.2]]
    )
    rpy_range: list[list[float]] = field(
        default_factory=lambda: [[-0.2, 0.2], [-0.2, 0.2], [-1.57079, 1.57079]]
    )
    num_poses: int = 30
    execute: bool = True
    wait_time: float = 3.0
    mode: str = "auto"
    sample_type: str = "random"  # random or file
    sample_file: str = "samples.yaml"
    move_to_pose_timeout: float = 180.0


@dataclass
class CalibrationConfig:
    """标定配置"""

    calibration_type: str = "intrinsic"  # "intrinsic" or "extrinsic"
    root_folder: str = ""  # 标定数据根目录
    board_config: BoardConfig = field(default_factory=BoardConfig)
    image_config: ImageConfig = field(default_factory=ImageConfig)
    end_effector_config: EndEffectorConfig = field(default_factory=EndEffectorConfig)
    extrinsic_config: ExtrinsicConfig = field(default_factory=ExtrinsicConfig)
    calibration_sampler_config: CalibrationSamplerConfig = field(
        default_factory=CalibrationSamplerConfig
    )
    robot_move_config: RobotMoveConfig = field(default_factory=RobotMoveConfig)

    def update_from_dict(
        self, cfg: Dict[str, Any], logger: Optional[logging.Logger] = None
    ):
        if not cfg:
            return self
        logger = logger or logging.getLogger("calibration_config")

        def _set_if_str(name: str):
            val = cfg.get(name)
            if isinstance(val, str) and val:
                setattr(self, name, val)

        _set_if_str("calibration_type")
        _set_if_str("root_folder")

        if "board_config" in cfg:
            for k, v in cfg["board_config"].items():
                if hasattr(self.board_config, k):
                    setattr(self.board_config, k, v)

        if "image_config" in cfg:
            img_conf = cfg["image_config"]
            if "topic" in img_conf:
                self.image_config.topic = img_conf["topic"]
            if "camera_info_topic" in img_conf:
                self.image_config.camera_info_topic = img_conf["camera_info_topic"]
            if "extensions" in img_conf:
                self.image_config.extensions = img_conf["extensions"]
            if "camera_frame" in img_conf:
                self.image_config.camera_frame = img_conf["camera_frame"]
            if "board_frame" in img_conf:
                self.image_config.board_frame = img_conf["board_frame"]

        if "end_effector_config" in cfg:
            end_conf = cfg["end_effector_config"]
            for key in ["base_frame", "tool_frame", "joint_topic", "use_sim_time"]:
                if key in end_conf:
                    setattr(self.end_effector_config, key, end_conf[key])

        if "extrinsic_config" in cfg:
            ext_conf = cfg["extrinsic_config"]
            for key in ["handeye_type", "handeye_method"]:
                if key in ext_conf:
                    setattr(self.extrinsic_config, key, ext_conf[key])

        if "robot_move_config" in cfg:
            robot_move_conf = cfg["robot_move_config"]
            for key in [
                "server_address",
                "timeout_ms",
                "group",
                "planner",
                "type",
                "max_velocity_scaling_factor",
                "max_acceleration_scaling_factor",
                "description",
                "joint_names",
            ]:
                if key in robot_move_conf:
                    setattr(self.robot_move_config, key, robot_move_conf[key])

        if "calibration_sampler_config" in cfg:
            cal_sampler_conf = cfg["calibration_sampler_config"]
            for key in [
                "group_name",
                "use_sim_time",
                "service_name",
                "camera_base_pose",
                "position_range",
                "rpy_range",
                "num_poses",
                "execute",
                "wait_time",
                "mode",
                "sample_type",
                "sample_file",
                "move_to_pose_timeout",
            ]:
                if key in cal_sampler_conf:
                    setattr(self.calibration_sampler_config, key, cal_sampler_conf[key])

        return self

    def to_dict(self) -> Dict[str, Any]:
        return {
            "calibration_type": self.calibration_type,
            "root_folder": self.root_folder,
            "board_config": asdict(self.board_config),
            "image_config": asdict(self.image_config),
            "end_effector_config": asdict(self.end_effector_config),
            "extrinsic_config": asdict(self.extrinsic_config),
            "calibration_sampler_config": asdict(self.calibration_sampler_config),
            "robot_move_config": asdict(self.robot_move_config),
        }

    def save_to_yaml(self, file_path: str, logger: Optional[logging.Logger] = None):
        logger = logger or logging.getLogger("calibration_config")
        try:
            os.makedirs(os.path.dirname(file_path) or ".", exist_ok=True)
            with open(file_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(
                    self.to_dict(), f, allow_unicode=True, default_flow_style=False
                )
            logger.info(f"配置已保存到: {file_path}")
        except Exception as e:
            logger.error(f"保存配置失败: {file_path}, {e}")
            raise

    @classmethod
    def load_from_yaml(cls, file_path: str, logger: Optional[logging.Logger] = None):
        logger = logger or logging.getLogger("calibration_config")
        if not file_path or not os.path.exists(file_path):
            raise ValueError(f"配置文件不存在: {file_path}")
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return cls().update_from_dict(data, logger)
        except Exception as e:
            logger.error(f"加载配置失败: {file_path}, {e}")
            raise


class CalibrationManager:
    """
    统一相机标定管理类

    支持内参和外参标定，多种图像获取方式，JSON参数配置
    """

    def __init__(self, config_file: Optional[str] = None):
        """
        初始化标定管理器

        参数:
            config_file: 配置文件路径，如果为None则从默认配置文件加载
        """
        self.logger = logging.getLogger(f"calibration_manager")
        self.io = CalibrationIO(self.logger)

        # 初始化标定器
        self.board_detector = None
        self.intrinsic_calibrator = None
        self.extrinsic_calibrator = None

        # ROS2相关
        self.ros2_image_subscriber = None
        self.ros2_tf_subscriber = None  # 统一的TF监听器，支持订阅任意TF转换
        self.ros2_joint_subscriber = None
        self.calibration_sampler = None  # 标定采样器

        # 图像管理
        self.current_image_index = 0  # 当前图像索引
        self.added_intrinsic_images = []  # 已添加的内参图像路径
        self.added_extrinsic_images = []  # 已添加的外参图像路径
        self.root_folder = None
        self.calibration_folder = None  # 标定文件夹
        self.image_folder = "origin"
        self.detection_folder = "detection"
        self.used_intrinsic_images_file = "intrinsic_images_used.txt"
        self.used_extrinsic_images_file = "extrinsic_images_used.txt"
        self.intrinsic_result_path = "intrinsic_result.yaml"
        self.extrinsic_result_path = "extrinsic_result.yaml"

        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None

        # 加载配置
        self.config = self._load_config(config_file)
        # 确保根目录存在，若不存在先日志告警再创建
        if not os.path.exists(self.root_folder):
            self.logger.warning(f"root_folder 不存在，自动创建: {self.root_folder}")
        os.makedirs(self.root_folder, exist_ok=True)
        # 切换工作目录
        os.chdir(self.root_folder)

        self.logger.info("CalibrationManager 初始化完成")

    def _normalize_path(self, path: str) -> str:
        """规范化路径：将绝对路径转换为相对于root_folder的相对路径"""
        if not path:
            return path
        if os.path.isabs(path):
            # 绝对路径，转换为相对于root_folder的相对路径
            try:
                rel_path = os.path.relpath(path, self.root_folder)
                # 统一使用正斜杠（跨平台兼容）
                return rel_path.replace(os.sep, "/")
            except ValueError:
                # 如果路径不在同一驱动器（Windows），返回原路径
                return path
        # 相对路径直接返回（统一使用正斜杠）
        return path.replace(os.sep, "/")

    def _get_base_dir(self) -> str:
        """获取标定基础目录"""
        return self.calibration_folder if self.calibration_folder else "."

    def _get_origin_dir(self) -> str:
        """获取原始图像目录"""
        return os.path.join(self._get_base_dir(), self.image_folder)

    def _get_detection_dir(self) -> str:
        """获取检测图像目录"""
        return os.path.join(self._get_base_dir(), self.detection_folder)

    def _get_poses_sample_file_path(self, file_name: str = "") -> str:
        """获取预先设置的需要采集数据的手臂配置文件路径"""
        if not file_name:
            file_name = self.config.calibration_sampler_config.sample_file
        return os.path.join(self._get_base_dir(), file_name)

    def _get_poses_recorded_file_path(self) -> str:
        """获取采集姿态文件路径，固定为标定目录下的 poses_recorded_file.yaml"""
        return os.path.join(self._get_base_dir(), "poses_recorded_file.yaml")

    def _get_intrinsic_images_used_path(self) -> str:
        """获取已使用内参图像列表文件路径"""
        return os.path.join(self._get_base_dir(), self.used_intrinsic_images_file)

    def _get_extrinsic_images_used_path(self) -> str:
        """获取已使用外参图像列表文件路径"""
        return os.path.join(self._get_base_dir(), self.used_extrinsic_images_file)

    def _load_selected_images(self):
        """从已选列表文件加载内外参图像列表"""

        def _load(path: str) -> List[str]:
            if not os.path.exists(path):
                return []
            try:
                content = self.io.read_text(path)
                res = [line.strip() for line in content.splitlines() if line.strip()]
                res.sort(key=lambda p: os.path.basename(p).lower())
                return res
            except Exception as e:
                self.logger.warning(f"加载已选图像列表失败 {path}: {e}")
                return []

        self.added_intrinsic_images = _load(self._get_intrinsic_images_used_path())
        self.added_extrinsic_images = _load(self._get_extrinsic_images_used_path())

    def _get_intrinsic_result_path(self) -> str:
        """获取内参标定结果文件路径"""
        return os.path.join(self._get_base_dir(), self.intrinsic_result_path)

    def _get_extrinsic_result_path(self) -> str:
        """获取外参标定结果文件路径"""
        return os.path.join(self._get_base_dir(), self.extrinsic_result_path)

    def _ensure_directories(self):
        """确保所有必要的目录存在"""
        base_dir = self._get_base_dir()
        origin_dir = self._get_origin_dir()
        detection_dir = self._get_detection_dir()

        for dir_path in [base_dir, origin_dir, detection_dir]:
            self.io.ensure_dir(dir_path)

    def get_all_folders(self):
        """获取所有标定的文件夹列表（按名称排序）"""
        return self.io.list_folders(".")

    def get_folder_structure(self, folder_path: str) -> Dict[str, Any]:
        """
        获取文件夹结构（按名称排序）
        """
        return self.io.folder_structure(folder_path)

    def create_calibration_folder(self, calibration_folder: str = None):
        """创建新的标定文件夹"""
        if calibration_folder is None or calibration_folder == "":
            self.calibration_folder = datetime.datetime.now().strftime(
                "%Y-%m-%d_%H-%M-%S"
            )
        else:
            self.calibration_folder = calibration_folder
        self._ensure_directories()
        self._load_selected_images()
        self.logger.info(f"标定文件夹已创建: {self.calibration_folder}")
        return self.get_calibration_structure()

    def select_calibration_folder(self, calibration_folder: str):
        """选择已有的标定文件夹"""
        self.calibration_folder = calibration_folder
        if not os.path.exists(self.calibration_folder):
            self.logger.error(f"标定文件夹不存在: {self.calibration_folder}")
            return None
        self._ensure_directories()
        self._load_selected_images()
        self.logger.info(f"标定文件夹已选择: {self.calibration_folder}")
        return self.get_calibration_structure()

    def get_calibration_structure(self) -> Dict[str, Any]:
        """
        获取完整的目录结构信息
        """
        structure = {
            "base_dir": {
                "path": self.calibration_folder,
                "exists": os.path.exists(self.calibration_folder),
            },
            "directories": {
                "origin": {
                    "path": self._get_origin_dir(),
                    "exists": os.path.exists(self._get_origin_dir()),
                },
                "detection": {
                    "path": self._get_detection_dir(),
                    "exists": os.path.exists(self._get_detection_dir()),
                },
            },
            "files": {
                "intrinsic_images_used": {
                    "path": self._get_intrinsic_images_used_path(),
                    "exists": os.path.exists(self._get_intrinsic_images_used_path()),
                    "type": "txt",
                },
                "extrinsic_images_used": {
                    "path": self._get_extrinsic_images_used_path(),
                    "exists": os.path.exists(self._get_extrinsic_images_used_path()),
                    "type": "txt",
                },
                "intrinsic_result": {
                    "path": self._get_intrinsic_result_path(),
                    "exists": os.path.exists(self._get_intrinsic_result_path()),
                    "type": "yaml",
                },
                "extrinsic_result": {
                    "path": self._get_extrinsic_result_path(),
                    "exists": os.path.exists(self._get_extrinsic_result_path()),
                    "type": "yaml",
                },
            },
        }

        if structure["directories"]["origin"]["exists"]:
            try:
                origin_files = [
                    {"path": p, "type": "image"}
                    for p in self.io.list_images(
                        self._get_origin_dir(), IMAGE_FILE_EXTS
                    )
                ]
                structure["directories"]["origin"]["files"] = origin_files
            except Exception as e:
                self.logger.error(f"读取origin目录失败: {e}")
                structure["directories"]["origin"]["files"] = []

        if structure["directories"]["detection"]["exists"]:
            try:
                detection_files = [
                    {"path": p, "type": "image"}
                    for p in self.io.list_images(
                        self._get_detection_dir(), IMAGE_FILE_EXTS
                    )
                ]
                structure["directories"]["detection"]["files"] = detection_files
            except Exception as e:
                self.logger.error(f"读取detection目录失败: {e}")
                structure["directories"]["detection"]["files"] = []

        return structure

    def get_file_content(self, file_path: str) -> Optional[Any]:
        """获取文件内容【除了图像】"""
        file_ext = os.path.splitext(file_path)[1]
        if file_ext in (".txt", ".yaml", ".yml"):
            return {"content": self.io.read_text(file_path)}
        if file_ext == ".json":
            data = self.io.load_json(file_path, default={})
            return {"content": json.dumps(data, indent=2, ensure_ascii=False)}
        self.logger.warning(f"不支持的文件类型: {file_ext}")
        return None

    def _init_detectors(self):
        """初始化检测器"""
        try:
            # 创建检测器配置
            detector_config = BoardConfig(
                board_type=self.config.board_config.board_type,
                x_num=self.config.board_config.x_num,
                y_num=self.config.board_config.y_num,
                square_length=self.config.board_config.square_length,
                marker_length=self.config.board_config.marker_length,
                dict_type=self.config.board_config.dict_type,
            )

            # 初始化检测器和内参标定器
            self.board_detector = BoardDetector(detector_config)
            self.logger.info("初始化标定板检测器成功")
            self.intrinsic_calibrator = IntrinsicCalibrator(detector_config)
            self.logger.info("初始化内参标定器成功")
            self.extrinsic_calibrator = HandEyeCalibrator()
            self.logger.info("初始化外参标定器成功")

            if not rclpy.ok():
                rclpy.init()

            # 初始化图像订阅器
            if self.config.image_config.topic:
                if self.ros2_image_subscriber is None:
                    self.ros2_image_subscriber = ROS2ImageSubscriber(
                        self.config.image_config.topic
                    )
                    self.logger.info("初始化图像订阅器成功")

            # 初始化TF监听器和关节订阅器（用于数据采集）
            if self.ros2_tf_subscriber is None:
                self.ros2_tf_subscriber = ROS2TFSubscriber(
                    use_sim_time=self.config.end_effector_config.use_sim_time
                )
                self.logger.info(
                    "初始化TF监听器成功（支持任意TF转换，兼容实时时间与仿真时间）"
                )
            if self.ros2_joint_subscriber is None:
                self.ros2_joint_subscriber = ROS2JointSubscriber(
                    topic_name=self.config.end_effector_config.joint_topic
                )
                self.logger.info("初始化关节订阅器成功")
            if self.calibration_sampler is None:
                self.calibration_sampler = CalibrationSampler(
                    use_sim_time=self.config.calibration_sampler_config.use_sim_time,
                    service_name=self.config.calibration_sampler_config.service_name,
                    tf_subscriber=self.ros2_tf_subscriber,
                    server_address=self.config.robot_move_config.server_address,
                    timeout_ms=self.config.robot_move_config.timeout_ms,
                )
                self.logger.info("初始化标定采样器成功")

        except Exception as e:
            self.logger.error(f"初始化检测器失败: {e}")
            self.board_detector = None
            self.intrinsic_calibrator = None

    def _load_config(self, config_file: Optional[str] = None) -> CalibrationConfig:
        """
        加载配置

        参数:
            config_file: 配置文件路径，如果为None则从默认配置文件加载
        """
        # 如果没有指定配置文件，尝试加载默认配置
        if config_file is None or config_file == "":
            raise ValueError(f"没有指定配置文件")
        if not os.path.exists(config_file):
            raise ValueError(f"配置文件不存在: {config_file}")
        config = CalibrationConfig.load_from_yaml(config_file, self.logger)
        self.root_folder = config.root_folder
        self.logger.info("配置加载成功")
        return config

    def _validate_config(self) -> bool:
        """验证配置"""
        # 验证标定类型
        if self.config.calibration_type not in ["intrinsic", "extrinsic"]:
            self.logger.error(f"无效的标定类型: {self.config.calibration_type}")
            return False

        # 验证标定板配置
        if self.config.board_config.board_type not in ["ChArUco", "Chessboard"]:
            self.logger.error(
                f"无效的标定板类型: {self.config.board_config.board_type}"
            )
            return False

        if self.config.board_config.x_num <= 0 or self.config.board_config.y_num <= 0:
            self.logger.error("无效的标定板尺寸")
            return False

        # 验证图像源配置
        if (
            not self.config.image_config.topic
            and not self.config.image_config.extensions
        ):
            self.logger.error("图像源配置不完整")
            return False

        return True

    def load_camera_params(self, camera_params_file: str = None):
        """
        设置相机内参

        参数:
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数
        """
        if camera_params_file is None or camera_params_file == "":
            camera_params_file = self._get_intrinsic_result_path()
        camera_params = None
        if camera_params_file and os.path.exists(camera_params_file):
            camera_params = self.io.load_camera_params_from_yaml(camera_params_file)
        else:
            self.logger.warning(f"内参文件不存在，尝试从CameraInfo topic加载: {camera_params_file}")
            camera_params = self._load_camera_params_from_topic()
        if camera_params is None:
            self.logger.error(f"加载内参文件失败: {camera_params_file}")
            return False
        self.camera_matrix = camera_params["camera_matrix"]
        self.dist_coeffs = camera_params["dist_coeffs"]
        self.dist_model = camera_params["dist_model"]
        self.image_width = camera_params["image_width"]
        self.image_height = camera_params["image_height"]
        self.logger.info("相机内参已设置 %s", camera_params_file)
        return True

    def _get_camera_info_topic(self) -> str:
        """获取CameraInfo topic，优先使用配置，否则从图像topic推导。"""
        if self.config.image_config.camera_info_topic:
            return self.config.image_config.camera_info_topic

        image_topic = self.config.image_config.topic
        if image_topic.endswith("/image_raw"):
            return image_topic.rsplit("/", 1)[0] + "/camera_info"
        return "/camera/color/camera_info"

    def _load_camera_params_from_topic(self) -> Optional[Dict[str, Any]]:
        """从ROS2 CameraInfo topic读取一帧内参。"""
        topic = self._get_camera_info_topic()
        timeout_sec = self.config.image_config.timeout

        if not rclpy.ok():
            rclpy.init()

        node = rclpy.create_node("calibration_camera_info_loader")
        camera_info_msg = {"msg": None}

        def _callback(msg: CameraInfo):
            camera_info_msg["msg"] = msg

        subscription = node.create_subscription(CameraInfo, topic, _callback, 10)
        start_time = time.time()
        try:
            while camera_info_msg["msg"] is None and (time.time() - start_time) < timeout_sec:
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()

        msg = camera_info_msg["msg"]
        if msg is None:
            self.logger.error(f"从CameraInfo topic读取内参超时: {topic}")
            return None

        camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)
        dist_coeffs = np.array(msg.d, dtype=float)
        result = {
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
            "dist_model": msg.distortion_model,
            "image_width": msg.width,
            "image_height": msg.height,
        }
        self.logger.info(f"成功从CameraInfo topic加载相机内参: {topic}")
        return result

    def _load_image(self, image_path: str = None) -> Optional[Dict[str, Any]]:
        """
        加载图像；若未指定路径则从ROS2 topic获取并生成保存路径。
        """
        res = {"image_path": None, "image": None}

        if image_path:
            if not os.path.exists(image_path):
                self.logger.warning(f"图像文件不存在: {image_path}")
                return res
            image = cv2.imread(image_path)
            if image is None:
                self.logger.warning(f"读取图像失败: {image_path}")
                return res
            res["image_path"] = image_path
        else:
            if self.ros2_image_subscriber is None:
                self.logger.error("ROS2图像订阅器未初始化")
                return res
            image = self.ros2_image_subscriber.get_latest_image(
                timeout=self.config.image_config.timeout, force_new=True
            )
            if image is None:
                self.logger.error("无法从topic获取图像")
                return res
            image_name = f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            res["image_path"] = os.path.join(self._get_origin_dir(), image_name)

        res["image"] = image
        return res

    def get_detection(
        self,
        image_path: str = None,
        calibration_type: str = None,
        ignore_board_pose: bool = False,
        ignore_end_pose: bool = False,
        sync_to_file: bool = True,
    ) -> Optional[Dict[str, Any]]:
        """
        获取图像检测结果

        参数:
            image_path: 图像路径

        返回:
            包含检测结果的字典
        """
        res = {
            "image": None,
            "image_path": None,
            "detection_image": None,
            "detection_image_path": None,
            "detection_success": False,
        }

        # 标定板检测器检查
        if not ignore_board_pose and self.board_detector is None:
            self.logger.error("检测器未初始化")
            res["message"] = "检测器未初始化"
            return res

        # 加载图像
        image_result = self._load_image(image_path)
        if image_result["image"] is None:
            self.logger.error(f"图像加载失败: {image_result['image_path']}")
            res["message"] = "图像加载失败"
            return res

        # 更新结果字典
        res.update(
            {
                "image": image_result["image"],
                "image_path": image_result["image_path"],
            }
        )

        # 标定板检测
        try:
            if not ignore_board_pose and self.camera_matrix is not None and self.dist_coeffs is not None:
                detection_result = self.board_detector.estimate_pose(
                    res["image"], self.camera_matrix, self.dist_coeffs
                )
                if detection_result.get("pose_success", False):
                    res["board_pose"] = {
                        "translation_vector": detection_result.get("translation_vector"),
                        "rotation_vector": detection_result.get("rotation_vector"),
                        "rotation_matrix": detection_result.get("rotation_matrix"),
                        "quaternion": detection_result.get("quaternion"),
                    }
            else:
                detection_result = self.board_detector.detect_board(res["image"])
        except Exception as e:
            self.logger.error(f"标定板检测失败: {e}")
            res["message"] = f"标定板检测失败: {e}"
            return res

        # 跟新检测图像
        if detection_result["detection_success"]:
            image_name = os.path.basename(res["image_path"])
            name_parts = os.path.splitext(image_name)
            detection_name = f"{name_parts[0]}_detected{name_parts[1]}"
            res["detection_image_path"] = os.path.join(
                self._get_detection_dir(), detection_name
            )
            res["detection_image"] = detection_result["detection_image"]
            res["detection_success"] = detection_result["detection_success"]
            res["corners_num"] = detection_result["corners_num"]
            self.logger.info(f"标定板检测成功: {res['detection_image_path']}")
        else:
            self.logger.error("标定板检测失败")
            res["message"] = "标定板检测失败"
            return res

        # 保存图像
        if not os.path.exists(res["image_path"]):
            cv2.imwrite(res["image_path"], res["image"])
            self.logger.info(f"原始图像已保存到: {res['image_path']}")
        if not os.path.exists(res["detection_image_path"]):
            cv2.imwrite(res["detection_image_path"], res["detection_image"])
            self.logger.info(f"检测图像已保存到: {res['detection_image_path']}")

        # 处理外参标定相关逻辑
        return res

    def add_detection(
        self,
        image_path: str,
        add_flag: bool = True,
        calibration_type: str = None,
    ) -> bool:
        """
        将图像路径添加/移除到内参或外参的已选列表，并写入文件。
        路径会被规范化为相对于root_folder的相对路径。
        """
        # 规范化路径：统一转换为相对于root_folder的相对路径
        normalized_path = self._normalize_path(image_path)

        # 检查文件是否存在（使用原始路径或规范化后的路径）
        check_path = normalized_path if not os.path.isabs(image_path) else image_path
        if not os.path.exists(check_path):
            self.logger.warning(f"图像文件不存在: {check_path}")
            if not add_flag:
                # 移除操作时即便文件不存在，也尝试从列表中移除
                self.logger.info(f"尝试从列表移除不存在的文件: {normalized_path}")
            else:
                return False

        if calibration_type is None or calibration_type == "":
            calibration_type = self.config.calibration_type

        if calibration_type == "intrinsic":
            target_list = self.added_intrinsic_images
        elif calibration_type == "extrinsic":
            target_list = self.added_extrinsic_images
        else:
            self.logger.error(f"未知的标定类型: {calibration_type}")
            return False

        if add_flag:
            if normalized_path in target_list:
                self.logger.info(f"图像已在列表中: {normalized_path}")
                return True
            target_list.append(normalized_path)
            target_list.sort(key=lambda p: os.path.basename(p).lower())
            self.logger.info(f"已添加图像: {normalized_path}")
        else:
            if normalized_path not in target_list:
                self.logger.info(f"图像不在列表中: {normalized_path}，依然视为已移除")
                return True
            target_list.remove(normalized_path)
            target_list.sort(key=lambda p: os.path.basename(p).lower())
            self.logger.info(f"已移除图像: {normalized_path}")

        # 写入对应的已选列表文件
        try:
            file_path = (
                self._get_intrinsic_images_used_path()
                if calibration_type == "intrinsic"
                else self._get_extrinsic_images_used_path()
            )
            self.io.ensure_dir(os.path.dirname(file_path) or ".")
            with open(file_path, "w", encoding="utf-8") as f:
                for p in target_list:
                    f.write(p + "\n")
        except Exception as e:
            self.logger.warning(f"同步已选列表到文件失败: {e}")

        return True

    def capture_image(self, save_path: Optional[str] = None) -> Dict[str, Any]:
        """
        采集图像（单独接口）

        参数:
            save_path: 保存路径，如果为None则自动生成

        返回:
            包含图像路径和图像的字典
        """
        result = {
            "success": False,
            "message": "",
            "image_path": None,
            "image": None,
        }

        try:
            if self.ros2_image_subscriber is None:
                result["message"] = "ROS2图像订阅器未初始化"
                return result

            # 从ROS2 topic获取图像
            image = self.ros2_image_subscriber.get_latest_image(
                timeout=self.config.image_config.timeout, force_new=True
            )
            if image is None:
                result["message"] = "无法从topic获取图像"
                return result

            # 确定保存路径
            if save_path is None:
                # 自动生成路径
                image_name = (
                    f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
                )
                save_path = os.path.join(self._get_origin_dir(), image_name)
            else:
                # 确保目录存在
                self.io.ensure_dir(os.path.dirname(save_path) or ".")

            # 保存图像
            cv2.imwrite(save_path, image)
            self.logger.info(f"图像已保存: {save_path}")

            result["success"] = True
            result["message"] = "图像采集成功"
            result["image_path"] = save_path
            result["image"] = image

        except Exception as e:
            self.logger.error(f"采集图像失败: {e}")
            result["message"] = f"采集图像失败: {str(e)}"

        return result

    def capture_end_effector_pose(
        self, base_frame: str, tool_frame: str
    ) -> Dict[str, Any]:
        """
        采集末端执行器位姿（单独接口）

        返回:
            包含末端位姿信息的字典
        """
        result = {
            "success": False,
            "message": "",
            "pose": None,
        }

        try:
            if self.ros2_tf_subscriber is None:
                result["message"] = "ROS2 TF监听器未初始化"
                return result

            # 获取末端位姿（base_frame -> tool_frame）
            pose_dict = self.ros2_tf_subscriber.get_pose_as_dict(
                parent_frame=base_frame,
                child_frame=tool_frame,
                timeout=5.0,
            )

            if pose_dict is None:
                result["message"] = "获取末端位姿失败"
                return result

            result["success"] = True
            result["message"] = "末端位姿采集成功"
            result["pose"] = pose_dict

        except Exception as e:
            self.logger.error(f"采集末端位姿失败: {e}")
            result["message"] = f"采集末端位姿失败: {str(e)}"

        return result

    def capture_joint_angles(self) -> Dict[str, Any]:
        """
        采集关节角度（单独接口）

        返回:
            包含关节角度信息的字典
        """
        result = {
            "success": False,
            "message": "",
            "joint_values": None,
        }

        try:
            if self.ros2_joint_subscriber is None:
                result["message"] = "ROS2关节订阅器未初始化"
                return result

            # 获取关节角度
            joint_values = self.ros2_joint_subscriber.get_latest_joint_state(
                timeout=5.0, force_new=True
            )

            if joint_values is None:
                result["message"] = "获取关节角度失败"
                return result

            result["success"] = True
            result["message"] = "关节角度采集成功"
            result["joint_values"] = joint_values

        except Exception as e:
            self.logger.error(f"采集关节角度失败: {e}")
            result["message"] = f"采集关节角度失败: {str(e)}"

        return result

    def collect_calibration_sample(
        self,
        image_folder: str,
        camera_frame: str,
        board_frame: str,
        tool_frame: str,
        base_frame: str,
    ) -> Dict[str, Any]:
        """
        采集单个标定样本

        参数:
            pose: 位姿配置字典（包含joint_values等）
            image_folder: 图像保存目录
            camera_frame: 相机坐标系
            board_frame: 标定板坐标系
            wait_after_motion: 运动完成后等待时间（秒）

        返回:
            包含采集数据的样本字典
        """
        # 采集图像
        img_res=None
        if image_folder is not None:
            image_name = f"{datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]}.jpg"
            save_path = os.path.join(image_folder,image_name)
            img_res = self.capture_image(save_path=save_path)

        end_pose = None
        if base_frame is not None and tool_frame is not None:
            end_pose_res = self.capture_end_effector_pose(
                base_frame=base_frame, tool_frame=tool_frame
            )
            end_pose = end_pose_res.get("pose") if end_pose_res.get("success") else None

        joint_measured = None
        joint_res = self.capture_joint_angles()
        if joint_res and joint_res.get("success"):
            joint_measured = joint_res.get("joint_values")
        
        board_pose = None
        if camera_frame is not None and board_frame is not None:
        # 标定板TF
            try:
                board_pose = self.ros2_tf_subscriber.get_pose_as_dict(
                    parent_frame=camera_frame,
                    child_frame=board_frame,
                    timeout=5.0,
                )
            except Exception as e:
                self.logger.warning(f"获取标定板TF失败: {e}")

        sample = {
            "image_path": img_res.get("image_path"),
            "joint_values": joint_measured,
            "end_pose": end_pose,
            "board_pose": board_pose,
        }
        return sample

    def collect_calibration_samples(
        self,
        image_topic: str,
        camera_frame: str,
        board_frame: str,
        joints_topic: str,
        tool_frame: str,
        base_frame: str,
        group_name: str,
        wait_after_motion: float = 3.0,
        sample_type: str = "random",
        sample_file: str = "",
        num_poses: Optional[int] = None,
    ) -> Dict[str, Any]:
        """
        按arm_config配置采集数据（仅TF方式获取姿态），保存图像到image_folder，结果写入poses_recorded_path（YAML）。

        输入参数：
            image_topic: 图像topic
            camera_frame: 相机坐标系
            board_frame: 标定板坐标系
            joints_topic: 关节状态topic
            tool_frame: 末端坐标系
            base_frame: 基座坐标系
            group_name: 机械臂组名
            poses_recorded_path: 输出位姿记录文件（YAML，紧凑格式）
            wait_after_motion: 运动完成后等待时间（秒），当前无MoveIt，仅用于保留接口
            random_poses: 是否随机采集点位
        说明：
            - 如果random_poses为True，则随机采集点位
            - 如果random_poses为False，则按照arm_config中的poses顺序采集点位
        """
        result = {
            "success": False,
            "message": "",
            "samples": [],
        }
        self.logger.info(
            f"collect_calibration_samples params: {image_topic}, {camera_frame}, {board_frame}, {tool_frame}, {base_frame}, {group_name}, {wait_after_motion}, {sample_type}, {sample_file}, {num_poses}"
        )
        try:
            # 确保rclpy已初始化（ROS2订阅器需要）
            if not rclpy.ok():
                rclpy.init()

            # 确保目录存在   origin和detection要情况，直接在这里执行rm *之类的操作，直接删掉相应文件夹中的图片
            poses_recorded_path = self._get_poses_recorded_file_path()
            image_folder = self._get_origin_dir()  ##
            self.io.ensure_dir(image_folder)
#--------------------------------------------------------------------------            
            detection_folder = self._get_detection_dir()  # 获取检测结果目录
            self.io.ensure_dir(detection_folder) # 确保检测目录也存在
            # 清空原始图目录
            if os.path.exists(image_folder):
                shutil.rmtree(image_folder)
            os.makedirs(image_folder)
            
            # 清空检测图目录
            if os.path.exists(detection_folder):
                shutil.rmtree(detection_folder)
            os.makedirs(detection_folder)
            # --------------------------
#---------------------------------------------------------------------------
            # 重新初始化图像订阅器（若topic变更）
            if image_topic:
                if (
                    self.ros2_image_subscriber is None
                    or self.config.image_config.topic != image_topic
                ):
                    self._cleanup_image_subscriber()
                    self.ros2_image_subscriber = ROS2ImageSubscriber(image_topic)
                    self.config.image_config.topic = image_topic

            # 重新初始化关节订阅器（若topic变更）
            if joints_topic:
                if (
                    self.ros2_joint_subscriber is None
                    or self.config.end_effector_config.joint_topic != joints_topic
                ):
                    self._cleanup_joint_subscriber()
                    self.ros2_joint_subscriber = ROS2JointSubscriber(
                        topic_name=joints_topic
                    )
                    self.config.end_effector_config.joint_topic = joints_topic

            if self.calibration_sampler is None:
                self.calibration_sampler = CalibrationSampler(
                    use_sim_time=self.config.calibration_sampler_config.use_sim_time,
                    service_name=self.config.calibration_sampler_config.service_name,
                    server_address=self.config.robot_move_config.server_address,
                    timeout_ms=self.config.robot_move_config.timeout_ms,
                )
            samples = []
            self.logger.info(f"initializing calibration sampler")
            if sample_type == "file":
                sample_file = self._get_poses_sample_file_path(file_name=sample_file)
                pose_samples = self.io.load_yaml(sample_file, default={}).get("poses", [])
                for pose_sample in pose_samples:
                    if self.config.robot_move_config.type == "joint":
                        joint_names = self.config.robot_move_config.joint_names
                        joint_values = pose_sample.get("joint_values")
                        if joint_values:
                            joint_values_list = [
                                np.rad2deg(joint_values[joint_name])
                                for joint_name in joint_names
                            ]
                            self.logger.info(
                                f"joint_names: {joint_names}, values: {joint_values_list}"
                            )
                            res = self.calibration_sampler.plan_and_execute_via_joint_values(
                                group=self.config.robot_move_config.group,
                                planner=self.config.robot_move_config.planner,
                                type=self.config.robot_move_config.type,
                                joint_names=joint_names,
                                joint_values=joint_values_list,
                                max_velocity_scaling_factor=self.config.robot_move_config.max_velocity_scaling_factor,
                                max_acceleration_scaling_factor=self.config.robot_move_config.max_acceleration_scaling_factor,
                                description=self.config.robot_move_config.description,
                            )
                            if not res.get("success"):
                                self.logger.error(
                                    f"plan and execute via joint values failed: {res.get('error')}"
                                )
                                continue
                        else:
                            self.logger.error(f"joint_values is None")
                            continue
                    elif self.config.robot_move_config.type == "cart":
                        pose = pose_sample.get("end_pose")
                        if pose:
                            res = self.calibration_sampler.plan_and_execute_via_pose(
                                group=self.config.robot_move_config.group,
                                planner=self.config.robot_move_config.planner,
                                type=self.config.robot_move_config.type,
                                ik_frame=tool_frame,
                                frame_id=base_frame,
                                position=[
                                    pose["position"]["x"],
                                    pose["position"]["y"],
                                    pose["position"]["z"],
                                ],
                                orientation=[
                                    pose["orientation"]["qx"],
                                    pose["orientation"]["qy"],
                                    pose["orientation"]["qz"],
                                    pose["orientation"]["qw"],
                                ],
                                max_velocity_scaling_factor=self.config.robot_move_config.max_velocity_scaling_factor,
                                max_acceleration_scaling_factor=self.config.robot_move_config.max_acceleration_scaling_factor,
                                description=self.config.robot_move_config.description,
                            )
                            if not res.get("success"):
                                self.logger.error(
                                    f"plan and execute via pose failed: {res.get('error')}"
                                )
                                continue
                        else:
                            self.logger.error(f"position or orientation is None")
                            continue
                    else:
                        self.logger.error(f"robot_move_config.type is not supported")
                        continue
                    time.sleep(wait_after_motion)
                    sample = self.collect_calibration_sample(
                        image_folder=image_folder,
                        camera_frame=camera_frame,
                        board_frame=board_frame,
                        tool_frame=tool_frame,
                        base_frame=base_frame,
                    )
                    samples.append(sample)
                    self.io.save_yaml({"poses": samples}, poses_recorded_path)
            elif sample_type == "random":
                # 如果传入了num_poses参数，使用传入的值，否则使用配置中的值
                num_poses_to_use = (
                    num_poses
                    if num_poses is not None
                    else self.config.calibration_sampler_config.num_poses
                )
                self.logger.info(f"num_poses_to_use: {num_poses_to_use}")
                self.logger.info(
                    f"base_pose: {self.config.calibration_sampler_config.camera_base_pose}"
                )
                self.logger.info(
                    f"position_range: {self.config.calibration_sampler_config.position_range}"
                )
                self.logger.info(
                    f"rpy_range: {self.config.calibration_sampler_config.rpy_range}"
                )
                poses = self.calibration_sampler.generate_random_poses(
                    base_pose=self.config.calibration_sampler_config.camera_base_pose,
                    position_range=self.config.calibration_sampler_config.position_range,
                    rpy_range=self.config.calibration_sampler_config.rpy_range,
                    num_poses=num_poses_to_use,
                )
                self.logger.info(f"poses number: {len(poses)}")
                transform = self.ros2_tf_subscriber.get_transform_as_list(
                    parent_frame=camera_frame,
                    child_frame=tool_frame,
                    timeout=5.0,
                )
                for pose in poses:
                    self.logger.info(f"pose: {pose}")
                    self.logger.info(f"transform: {transform}")
                    end_pose = self.calibration_sampler.apply_transform(
                        pose=transform, transform=pose
                    )
                    self.logger.info(f"end_pose: {end_pose}")
                    success = self.calibration_sampler.move_to_pose(
                        pose=end_pose,
                        group_name=group_name,
                        timeout_sec=self.config.calibration_sampler_config.move_to_pose_timeout,
                    )
                    if not success:
                        self.logger.error(f"move to pose failed")
                        continue
                    time.sleep(wait_after_motion)
                    sample = self.collect_calibration_sample(
                        image_folder=image_folder,
                        camera_frame=camera_frame,
                        board_frame=board_frame,
                        tool_frame=tool_frame,
                        base_frame=base_frame,
                    )
                    samples.append(sample)
                    self.io.save_yaml({"poses": samples}, poses_recorded_path)

            self.io.save_yaml({"poses": samples}, poses_recorded_path)
            result["success"] = True
            result["message"] = (
                f"采集完成，共{len(samples)}条，已写入: {poses_recorded_path}"
            )
            result["samples"] = samples
            return result

        except Exception as e:
            self.logger.error(f"采集过程出错: {e}")
            result["message"] = f"采集过程出错: {str(e)}"
            return result

    def save_capture_to_poses_file(
        self, data_captured: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        将采集的数据增量保存到poses_recorded_file。

        参数:
            data_captured: 采集结果字典

        返回:
            包含success和message的字典
        """
        result = {
            "success": False,
            "message": "",
        }

        try:
            # 获取poses_recorded_file路径（固定命名）
            poses_file = self._get_poses_recorded_file_path()

            existing_data = {"poses": []}
            if os.path.exists(poses_file):
                existing_data = self.io.load_yaml(poses_file, default={})
                if not existing_data:
                    existing_data = {"poses": []}
                if "poses" not in existing_data:
                    existing_data["poses"] = []

            if len(data_captured.keys()) > 0:
                existing_data["poses"].append(data_captured)
                self.io.save_yaml(existing_data, poses_file)
                result["success"] = True
                saved_items = []
                if "image_path" in data_captured:
                    saved_items.append("图像")
                if "end_pose" in data_captured:
                    saved_items.append("末端位姿")
                if "board_pose" in data_captured:
                    saved_items.append("标定板位姿")
                if "joint_values" in data_captured:
                    saved_items.append("关节角度")
                result["message"] = (
                    f"已增量保存采集数据到: {poses_file} (包含: {', '.join(saved_items)})"
                )
                self.logger.info(result["message"])
            else:
                result["message"] = "采集数据为空，跳过保存"
                self.logger.warning(result["message"])

        except Exception as e:
            error_msg = f"保存采集数据到poses_recorded_file失败: {e}"
            self.logger.error(error_msg)
            result["message"] = error_msg

        return result

    def remove_pose_records_by_image(self, image_path: str) -> int:
        """
        根据图像路径，从 poses_recorded_file 中移除对应记录。
        返回删除的记录数，失败返回0。
        """
        try:
            poses_file = self._get_poses_recorded_file_path()
            if not os.path.exists(poses_file):
                return 0

            data = self.io.load_yaml(poses_file, default={})
            poses = data.get("poses", [])
            if not poses:
                return 0

            # 构造匹配集合：原路径、归一化路径、绝对路径
            targets = set()
            targets.add(image_path)
            targets.add(self._normalize_path(image_path))
            targets.add(os.path.abspath(image_path))

            new_poses = [p for p in poses if p.get("image_path") not in targets]
            removed = len(poses) - len(new_poses)

            if removed > 0:
                data["poses"] = new_poses
                self.io.save_yaml(data, poses_file)
                self.logger.info(
                    f"已从 poses_recorded_file 移除 {removed} 条记录 (image_path={image_path})"
                )

            return removed
        except Exception as e:
            self.logger.error(f"从 poses_recorded_file 移除记录失败: {e}")
            return 0

    def _save_camera_info_yaml(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_width: int,
        image_height: int,
    ):
        """
        生成并保存ROS camera_info格式的YAML文件

        参数:
            calibration_result: 内参标定结果
        """
        try:
            # 构造camera_info数据
            camera_info = self._build_camera_info_dict(
                camera_matrix,
                dist_coeffs,
                image_width,
                image_height,
            )

            # 保存为YAML文件
            camera_info_path = self._get_intrinsic_result_path()
            self.io.save_yaml(camera_info, camera_info_path)

            self.logger.info(f"camera_info YAML文件已保存到: {camera_info_path}")

        except Exception as e:
            self.logger.error(f"生成camera_info YAML文件失败: {e}")
            raise

    def _build_camera_info_dict(
        self,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        image_width: int,
        image_height: int,
    ) -> Dict[str, Any]:
        """
        构建 ROS camera_info 格式的字典

        参数:
            camera_matrix: 相机内参矩阵 (3x3)
            dist_coeffs: 畸变系数
            image_width: 图像宽度
            image_height: 图像高度

        返回:
            camera_info 字典
        """
        # 相机名称（可以根据需要修改）
        camera_name = "camera"

        # 将numpy数组转换为列表（YAML序列化需要）
        K = [float(x) for x in camera_matrix.flatten()]  # 3x3 -> 9元素列表
        D = [float(x) for x in dist_coeffs.flatten()]  # 畸变系数列表

        # 矫正矩阵（单目相机为单位矩阵）
        R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # 投影矩阵 P = [fx, 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0]
        fx, fy = float(camera_matrix[0, 0]), float(camera_matrix[1, 1])
        cx, cy = float(camera_matrix[0, 2]), float(camera_matrix[1, 2])
        P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        # 构建 camera_info 字典
        camera_info = {
            "image_width": int(image_width),
            "image_height": int(image_height),
            "camera_name": camera_name,
            "camera_matrix": {"rows": 3, "cols": 3, "data": K},
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {"rows": 1, "cols": len(D), "data": D},
            "rectification_matrix": {"rows": 3, "cols": 3, "data": R},
            "projection_matrix": {"rows": 3, "cols": 4, "data": P},
        }

        return camera_info

    def _load_poses_from_file(self, pose_file: str) -> List[Dict[str, Any]]:
        """
        从位姿文件加载数据

        参数:
            pose_file: 位姿文件路径（YAML格式）

        返回:
            位姿数据列表
        """
        try:
            if not os.path.exists(pose_file):
                self.logger.error(f"位姿文件不存在: {pose_file}")
                return []

            data = self.io.load_yaml(pose_file, default={})

            if "poses" not in data:
                self.logger.error("位姿文件中没有找到poses字段")
                return []

            return data["poses"]

        except Exception as e:
            self.logger.error(f"加载位姿文件失败: {e}")
            return []

    def _pose_dict_to_matrix(
        self, pose_dict: Dict[str, Any]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        将位姿字典转换为旋转矩阵和位移向量

        参数:
            pose_dict: 包含position和orientation的字典

        返回:
            (rotation_matrix, translation_vector) 元组
        """
        # 提取位置
        position = pose_dict["position"]
        orientation = pose_dict["orientation"]

        # 提取四元数并转换为旋转矩阵
        quaternion = [
            orientation["qx"],
            orientation["qy"],
            orientation["qz"],
            orientation["qw"],
        ]  # [x, y, z, w]
        rotation_matrix = SciPyRotation.from_quat(quaternion).as_matrix()
        translation_vector = np.array([position["x"], position["y"], position["z"]])

        return rotation_matrix, translation_vector

    def calibrate(
        self,
        calibration_type: Optional[str] = None,
        image_folder: Optional[str] = None,
        pose_file: Optional[str] = None,
        intrinsic_file: Optional[str] = None,
        use_selected_data: bool = False,
    ) -> Dict[str, Any]:
        """
        统一标定入口：根据 calibration_type 分发到内参或外参标定。
        """
        self.logger.info(f" calibration_type: {calibration_type}")
        self.logger.info(f" image_folder: {image_folder}")
        self.logger.info(f" pose_file: {pose_file}")
        self.logger.info(f" intrinsic_file: {intrinsic_file}")
        self.logger.info(f" use_selected_data: {use_selected_data}")
        cali_type = (calibration_type or self.config.calibration_type or "").lower()
        if cali_type not in ("intrinsic", "extrinsic"):
            self.logger.error(f"无效的标定类型: {calibration_type}")
            return {"success": False, "message": f"无效的标定类型: {calibration_type}"}

        folder = image_folder or self._get_origin_dir()
        imgs = None
        if use_selected_data:
            try:
                if cali_type == "intrinsic":
                    imgs = self.io.get_images_from_file(
                        self._get_intrinsic_images_used_path()
                    )
                elif cali_type == "extrinsic":
                    imgs = self.io.get_images_from_file(
                        self._get_extrinsic_images_used_path()
                    )
                    self.logger.info(f"已选图像列表: {imgs}")
                else:
                    self.logger.error(f"无效的标定类型: {calibration_type}")
                    return {
                        "success": False,
                        "message": f"无效的标定类型: {calibration_type}",
                    }
            except Exception as e:
                self.logger.error(f"加载已选图像列表失败: {e}")
                return {"success": False, "message": f"加载已选图像列表失败: {e}"}

        if cali_type == "intrinsic":
            return self.calibrate_intrinsic(folder, imgs)
        elif cali_type == "extrinsic":
            pose_path = pose_file or self._get_poses_recorded_file_path()
            intrinsic_path = intrinsic_file
            return self.calibrate_extrinsic(pose_path, intrinsic_path, imgs)

    def calibrate_intrinsic(
        self,
        image_folder: str,
        selected_images: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """
        内参标定：指定目录和选中的图像文件列表

        参数:
            image_folder: 图像目录路径（必须指定）
            selected_images: 选中的图像文件列表（相对路径或文件名），如果为None则使用目录下所有图像

        返回:
            标定结果
        """
        try:
            # 确保目录存在
            if not os.path.exists(image_folder):
                return {"success": False, "message": f"图像目录不存在: {image_folder}"}

            # 确定要使用的图像列表
            if selected_images is None or len(selected_images) == 0:
                # 使用目录下所有图像
                image_paths = self.io.get_images_from_folder(
                    image_folder, self.config.image_config.extensions
                )
                self.logger.info(f"使用目录下所有图像，共{len(image_paths)}张")
            else:
                # 使用选中的图像列表
                image_paths = []
                for img_file in selected_images:
                    # 路径处理：前端传的路径都是基于root_folder的相对路径
                    # 如果是绝对路径，规范化后使用；否则直接使用（工作目录已是root_folder）
                    img_path = (
                        self._normalize_path(img_file)
                        if os.path.isabs(img_file)
                        else img_file
                    )

                    if os.path.exists(img_path):
                        image_paths.append(img_path)
                    else:
                        self.logger.warning(f"图像文件不存在，已跳过: {img_path}")

                if len(image_paths) == 0:
                    return {"success": False, "message": "没有有效的图像文件"}

                self.logger.info(f"使用选中的图像列表，共{len(image_paths)}张")

            # 执行内参标定
            res = self.intrinsic_calibrator.calibrate_from_image_list(image_paths)

            if res["success"]:
                self.camera_matrix = res["camera_matrix"]
                self.dist_coeffs = res["dist_coeffs"]
                self.logger.info(f"内参标定成功: {res['message']}")

                try:
                    img = cv2.imread(image_paths[0])
                    image_width, image_height = img.shape[1], img.shape[0]
                    self._save_camera_info_yaml(
                        self.camera_matrix, self.dist_coeffs, image_width, image_height
                    )
                    self.logger.info("已生成ROS camera_info YAML文件")
                except Exception as e:
                    self.logger.warning(f"生成camera_info YAML文件失败: {e}")
            else:
                self.logger.error(f"内参标定失败: {res['message']}")

            return res

        except Exception as e:
            self.logger.error(f"内参标定出错: {e}")
            return {"success": False, "message": f"内参标定出错: {str(e)}"}

    def calibrate_extrinsic(
        self,
        pose_file: Optional[str] = None,
        intrinsic_file: Optional[str] = None,
        imgs: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """
        外参标定：支持两种模式
        1. 如果pose_file中有board_pose，直接使用末端位姿和标定板位姿计算
        2. 否则使用图像+内参计算标定板位姿

        参数:
            pose_file: 位姿文件路径（YAML格式），如果为None则使用配置中的pose_file
            intrinsic_file: 内参文件路径，如果为None则使用已加载的内参

        返回:
            标定结果
        """
        try:
            # 确定位姿文件路径
            if pose_file is None:
                pose_file = self._get_poses_recorded_file_path()
            self.logger.info(f"使用位姿文件: {pose_file}")

            if not pose_file or not os.path.exists(pose_file):
                self.logger.error(f"无效的位姿文件: {pose_file}")
                return {"success": False, "message": f"位姿文件不存在: {pose_file}"}

            if intrinsic_file:
                if not os.path.exists(intrinsic_file):
                    self.logger.error(f"不存在内参文件: {intrinsic_file}")
                    return {
                        "success": False,
                        "message": f"内参文件不存在: {intrinsic_file}",
                    }
                if self.camera_matrix is None or self.dist_coeffs is None:
                    if self.load_camera_params(intrinsic_file) is False:
                        self.logger.error(f"无效的内参文件: {intrinsic_file}")
                        return {
                            "success": False,
                            "message": f"加载内参文件失败: {intrinsic_file}",
                        }

            # 加载位姿数据
            poses = self._load_poses_from_file(pose_file)
            if imgs:
                imgs_set = set(imgs)
                for p in poses:
                    print("image path: ", p.get("image_path"))
                poses = [p for p in poses if p.get("image_path") in imgs_set]
                if len(poses) == 0:
                    self.logger.warning("按图像列表过滤后无有效位姿数据")
                    return {
                        "success": False,
                        "message": "按图像列表过滤后无有效位姿数据",
                    }
            if len(poses) < 3:
                self.logger.warning(
                    f"有效数据不足，需要至少3组，当前只有{len(poses)}组"
                )
                return {
                    "success": False,
                    "message": f"有效数据不足，需要至少3组，当前只有{len(poses)}组",
                }
            self.logger.info(f" poses file: {pose_file}")
            # 收集标定数据
            R_gripper2base, t_gripper2base = [], []
            R_target2cam, t_target2cam = [], []

            for pose in poses:
                # 先检查末端位姿是否存在
                if "end_pose" not in pose:
                    self.logger.warning("位姿数据中缺少end_pose，已跳过")
                    continue

                # 判断标定板位姿来源
                if "board_pose" in pose and pose["board_pose"] is not None:
                    # 直接使用提供的标定板位姿
                    R_board, t_board = self._pose_dict_to_matrix(pose["board_pose"])
                    R_target2cam.append(R_board)
                    t_target2cam.append(t_board)
                elif intrinsic_file:
                    if (
                        "image_path" not in pose
                        or not pose["image_path"]
                        or not os.path.exists(pose["image_path"])
                    ):
                        self.logger.warning(
                            f"位姿数据中缺少image_path或图像文件不存在: {pose.get('image_path', 'N/A')}，已跳过该条"
                        )
                        continue

                    # 加载图像
                    image_path = pose["image_path"]
                    image = cv2.imread(image_path)
                    if image is None:
                        self.logger.warning(f"读取图像失败: {image_path}，已跳过该条")
                        continue

                    # 使用board_detector估计标定板位姿
                    try:
                        pose_result = self.board_detector.estimate_pose(
                            image, self.camera_matrix, self.dist_coeffs
                        )

                        if not pose_result.get("pose_success", False):
                            self.logger.warning(
                                f"标定板位姿估计失败: {image_path}，已跳过该条"
                            )
                            continue

                        # 从结果中提取旋转矩阵和平移向量
                        R_board = pose_result.get("rotation_matrix")
                        t_board = pose_result.get("translation_vector")

                        if R_board is None or t_board is None:
                            self.logger.warning(
                                f"标定板位姿数据不完整: {image_path}，已跳过该条"
                            )
                            continue

                        # 确保 t_board 是 numpy 数组
                        if not isinstance(t_board, np.ndarray):
                            t_board = np.array(t_board)

                        # 确保 R_board 是 3x3 矩阵
                        if R_board.shape != (3, 3):
                            self.logger.warning(
                                f"旋转矩阵维度错误: {R_board.shape}，已跳过该条"
                            )
                            continue

                        R_target2cam.append(R_board)
                        t_target2cam.append(t_board)
                        self.logger.info(f"成功使用内参计算标定板位姿: {image_path}")

                    except Exception as e:
                        self.logger.error(
                            f"使用内参计算标定板位姿失败: {image_path}, {e}，已跳过该条"
                        )
                        continue
                else:
                    self.logger.warning(
                        "位姿数据中缺少board_pose且未提供内参文件，已跳过该条"
                    )
                    continue

                # 只有当标定板位姿成功获取后，才添加末端位姿（确保数据一致性）
                R_end, t_end = self._pose_dict_to_matrix(pose["end_pose"])
                R_gripper2base.append(R_end)
                t_gripper2base.append(t_end)

            if len(R_gripper2base) < 3:
                return {
                    "success": False,
                    "message": f"有效数据不足，需要至少3组，当前只有{len(R_gripper2base)}组",
                }

            # 检查数据一致性
            if len(R_gripper2base) != len(R_target2cam):
                self.logger.error(
                    f"数据长度不一致: R_gripper2base={len(R_gripper2base)}, R_target2cam={len(R_target2cam)}"
                )
                return {
                    "success": False,
                    "message": f"数据长度不一致: 末端位姿{len(R_gripper2base)}组，标定板位姿{len(R_target2cam)}组",
                }
            self.logger.info(f" pose number: {len(R_gripper2base)}")
            self.logger.info(
                f" handeye_type: {self.config.extrinsic_config.handeye_type}"
            )
            self.logger.info(
                f" handeye_method: {self.config.extrinsic_config.handeye_method}"
            )
            # 使用手眼标定器
            R, t = self.extrinsic_calibrator.calibration(
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
                cali_type=self.config.extrinsic_config.handeye_type,
                method=self.config.extrinsic_config.handeye_method,
            )

            if R is not None and t is not None:
                # 保存结果
                result_path = self._get_extrinsic_result_path()

                # 计算欧拉角和四元数
                rotation = SciPyRotation.from_matrix(R)
                euler_angles = rotation.as_euler(
                    "xyz", degrees=False
                ).tolist()  # 弧度制
                quaternion_wxyz = rotation.as_quat()  # scipy默认为[x,y,z,w]格式
                quaternion = quaternion_wxyz.tolist()  # [x,y,z,w]格式

                result_data = {
                    "success": True,
                    "cali_type": self.config.extrinsic_config.handeye_type,
                    "method": self.config.extrinsic_config.handeye_method,
                    "num_poses": len(R_gripper2base),
                    "translation": t.flatten().tolist(),
                    "rotation": quaternion,
                    "rotation_euler": euler_angles,
                    "rotation_matrix": R.tolist(),
                    "message": f"手眼标定成功，使用{len(R_gripper2base)}组数据",
                }

                self.io.save_yaml(result_data, result_path)

                return result_data
            else:
                return {"success": False, "message": "手眼标定失败"}

        except Exception as e:
            self.logger.error(f"外参标定出错: {e}")
            return {"success": False, "message": f"外参标定出错: {str(e)}"}

    def _cleanup_image_subscriber(self):
        try:
            if self.ros2_image_subscriber is not None:
                self.ros2_image_subscriber.destroy_subscriber()
                self.ros2_image_subscriber.destroy_node()
                self.logger.info("清理图像订阅器成功")
        except Exception as e:
            self.logger.error(f"清理图像订阅器失败: {e}")

    def _cleanup_tf_listener(self):
        try:
            if self.ros2_tf_subscriber is not None:
                self.ros2_tf_subscriber.destroy_listener()
                self.ros2_tf_subscriber.destroy_node()
                self.logger.info("清理末端位姿TF监听器成功")
        except Exception as e:
            self.logger.error(f"清理末端位姿TF监听器失败: {e}")

    def _cleanup_joint_subscriber(self):
        try:
            if self.ros2_joint_subscriber is not None:
                self.ros2_joint_subscriber.destroy_subscriber()
                self.ros2_joint_subscriber.destroy_node()
                self.logger.info("清理关节订阅器成功")
        except Exception as e:
            self.logger.error(f"清理关节订阅器失败: {e}")

    def cleanup(self):
        """清理资源"""
        if self.ros2_image_subscriber:
            self._cleanup_image_subscriber()
        if self.ros2_tf_subscriber:
            self._cleanup_tf_listener()
        if self.ros2_joint_subscriber:
            self._cleanup_joint_subscriber()
        if rclpy.ok():
            rclpy.shutdown()
