"""
PapJia相机标定服务的REST API服务器模块。

该模块提供基于FastAPI的REST接口用于相机标定操作，
允许前端应用程序和其他HTTP客户端与标定系统进行交互。
"""

import logging
import asyncio
import hashlib
import json
import os
import io
import time
import datetime
from typing import Dict, Any, Optional, List
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Depends, Query, File, UploadFile, Form
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse, FileResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field
import uvicorn
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as SciPyRotation

from .calibration import CalibrationManager
from .config import ServiceConfig


# 请求/响应的Pydantic模型
class CalibrationFolderCreate(BaseModel):
    """创建标定文件夹的模型。"""

    calibration_folder: str = Field("", description="要创建的标定文件夹名称，可以为空")


class CalibrationFolderSelect(BaseModel):
    """选择标定文件夹的模型。"""

    calibration_folder: str = Field(..., description="要选择的标定文件夹名称")


class ConfigUpdate(BaseModel):
    """配置更新的模型。"""

    config: Dict[str, Any] = Field(..., description="要更新的配置数据")


class CameraParamsLoad(BaseModel):
    """加载相机参数的模型。"""

    camera_params_file: str = Field(..., description="相机参数文件路径")


class DetectionAdd(BaseModel):
    """添加检测结果的模型。"""

    image_path: str = Field(..., description="图像文件路径")
    calibration_type: Optional[str] = Field(None, description="标定类型 (内参/外参)")
    add_flag: bool = Field(True, description="是否添加检测结果")


class DetectionGet(BaseModel):
    """获取检测结果的模型。"""

    image_path: str = Field(..., description="图像文件路径")
    calibration_type: Optional[str] = Field(None, description="标定类型")
    ignore_board_pose: bool = Field(False, description="是否忽略标定板姿态")
    ignore_end_pose: bool = Field(False, description="是否忽略末端姿态")


class SingleCaptureRequest(BaseModel):
    """单次采集请求的模型。"""

    enable_image: bool = Field(True, description="是否采集图像")
    image_topic: Optional[str] = Field(None, description="图像话题")
    enable_board_tf: bool = Field(True, description="是否采集标定板TF")
    camera_frame: Optional[str] = Field(None, description="相机Frame")
    board_frame: Optional[str] = Field(None, description="标定板Frame")
    enable_joint: bool = Field(True, description="是否采集关节角度")
    joint_topic: Optional[str] = Field(None, description="关节话题")
    enable_end_tf: bool = Field(True, description="是否采集末端TF")
    tool_frame: Optional[str] = Field(None, description="末端Frame")
    base_frame: Optional[str] = Field(None, description="基坐标系Frame")
    save_path: Optional[str] = Field(None, description="图像保存路径（可选）")


class CalibrationRequest(BaseModel):
    """标定请求的模型。"""

    calibration_type: Optional[str] = Field(None, description="标定类型")
    # 内参标定参数
    image_folder: Optional[str] = Field(None, description="内参标定图像目录")
    use_selected_data: bool = Field(False, description="是否使用已选图像列表文件")
    # 外参标定参数
    pose_file: Optional[str] = Field(None, description="外参标定位姿文件路径")
    intrinsic_file: Optional[str] = Field(None, description="外参标定内参文件路径（可选）")


class AutoCaptureRequest(BaseModel):
    """自动采集请求的模型。"""

    image_topic: Optional[str] = Field(None, description="图像话题")
    camera_frame: Optional[str] = Field(None, description="相机Frame")
    board_frame: Optional[str] = Field(None, description="标定板Frame")
    joints_topic: Optional[str] = Field(None, description="关节话题")
    tool_frame: Optional[str] = Field(None, description="末端Frame")
    base_frame: Optional[str] = Field(None, description="基坐标系Frame")
    group_name: Optional[str] = Field(None, description="机械臂组名")
    wait_after_motion: float = Field(3.0, description="运动后等待时间（秒）")
    sample_type: str = Field("random", description="采样类型：random 或 file")
    sample_file: Optional[str] = Field(None, description="采样文件路径（sample_type为file时使用）")
    sample_count: Optional[int] = Field(None, description="采样数目（sample_type为random时使用）")


class FileContentRequest(BaseModel):
    """文件内容请求的模型。"""

    file_path: str = Field(..., description="文件路径")


class ApiResponse(BaseModel):
    """标准API响应模型。"""

    success: bool = Field(..., description="操作是否成功")
    data: Optional[Any] = Field(None, description="响应数据")
    message: Optional[str] = Field(None, description="可选消息")
    error_code: Optional[str] = Field(None, description="操作失败时的错误代码")


class HealthResponse(BaseModel):
    """健康检查响应模型。"""

    status: str = Field(..., description="服务状态")
    calibration_service_healthy: bool = Field(..., description="标定服务状态")
    version: str = Field(..., description="服务版本")
    available_operations: List[str] = Field(..., description="可用操作")
    message: Optional[str] = Field(None, description="附加信息")


class ImageDetectionResponse(BaseModel):
    """图像检测响应模型。"""

    detection_success: bool = Field(..., description="检测是否成功")
    message: Optional[str] = Field(None, description="检测消息")
    image_path: Optional[str] = Field(None, description="原始图像路径")
    detection_image_path: Optional[str] = Field(None, description="检测结果图像路径")
    metadata: Optional[Dict[str, Any]] = Field(None, description="图像元数据")


# 标定依赖
def get_calibrator(config: ServiceConfig) -> CalibrationManager:
    """获取标定管理器实例。"""
    return CalibrationManager(config.rest_api.calibration_config_file)


class CalibrationAPI:
    """相机标定操作的REST API类。"""

    def __init__(self, config: ServiceConfig):
        """使用配置初始API。"""
        self.config = config
        self.calibrator = None
        self.logger = logging.getLogger(__name__)

        # 验证图像压缩配置
        self._validate_image_compress_config()

        # 创建FastAPI应用
        self.app = self._create_app()

        # 初始化标定器
        self._init_calibrator()

    def _validate_image_compress_config(self):
        """验证图像压缩配置是否正确加载。"""
        required_attrs = [
            "image_compress_enabled",
            "image_compress_max_width",
            "image_compress_max_height",
            "image_compress_quality",
        ]
        missing_attrs = []
        for attr in required_attrs:
            if not hasattr(self.config.rest_api, attr):
                missing_attrs.append(attr)

        if missing_attrs:
            self.logger.warning(
                f"图像压缩配置缺失以下属性: {missing_attrs}。"
                f"请检查配置文件是否包含这些字段，并确保服务已重启。"
                f"当前 rest_api 配置属性: {[a for a in dir(self.config.rest_api) if not a.startswith('_')]}"
            )
        else:
            self.logger.info(
                f"图像压缩配置已加载: "
                f"enabled={self.config.rest_api.image_compress_enabled}, "
                f"max_size={self.config.rest_api.image_compress_max_width}x{self.config.rest_api.image_compress_max_height}, "
                f"quality={self.config.rest_api.image_compress_quality}"
            )

    def _compress_image(self, image_path: str) -> bytes:
        """
        压缩图像用于前端显示（从配置读取压缩参数）

        参数:
            image_path: 图像文件路径

        返回:
            压缩后的图像字节流
        """
        try:
            # 从配置读取压缩参数
            if not hasattr(self.config.rest_api, "image_compress_max_width"):
                raise AttributeError(
                    f"配置对象缺少 image_compress_max_width 属性。"
                    f"请检查配置文件是否包含该字段，并确保服务已重启。"
                    f"当前 rest_api 配置属性: {[attr for attr in dir(self.config.rest_api) if not attr.startswith('_')]}"
                )
            max_width = self.config.rest_api.image_compress_max_width
            max_height = self.config.rest_api.image_compress_max_height
            quality = self.config.rest_api.image_compress_quality

            # 读取图像
            img = cv2.imread(image_path)
            if img is None:
                raise ValueError(f"无法读取图像: {image_path}")

            height, width = img.shape[:2]

            # 如果图像尺寸小于阈值，只进行质量压缩
            if width <= max_width and height <= max_height:
                _, encoded_img = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
                return encoded_img.tobytes()

            # 计算缩放比例，保持宽高比
            scale = min(max_width / width, max_height / height)
            new_width = int(width * scale)
            new_height = int(height * scale)

            # 调整图像尺寸
            resized_img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)

            # 编码为JPEG格式
            _, encoded_img = cv2.imencode(".jpg", resized_img, [cv2.IMWRITE_JPEG_QUALITY, quality])

            return encoded_img.tobytes()
        except AttributeError as e:
            self.logger.error(f"配置属性缺失: {e}，请检查配置文件是否包含图像压缩相关配置")
            raise
        except Exception as e:
            self.logger.error(f"压缩图像失败: {e}")
            raise

    def _create_app(self) -> FastAPI:
        """创建和配置FastAPI应用程序。"""
        app = FastAPI(
            title="PapJia相机标定服务API",
            description="相机标定操作的REST API",
            version="0.1.0",
            docs_url="/docs" if self.config.rest_api.debug else None,
            redoc_url="/redoc" if self.config.rest_api.debug else None,
        )

        # 添加CORS中间件
        if self.config.rest_api.cors_enabled:
            app.add_middleware(
                CORSMiddleware,
                allow_origins=self.config.rest_api.cors_origins,
                allow_credentials=True,
                allow_methods=["*"],
                allow_headers=["*"],
            )

        # 先添加静态文件服务（需要在路由之前，确保静态文件路由优先匹配）
        self._setup_static_files(app)

        # 添加路由
        self._setup_routes(app)

        return app

    def _init_calibrator(self):
        """初始化标定系统。"""
        try:
            self.calibrator = get_calibrator(self.config)
            self.logger.info("标定系统已为REST API初始化")
        except Exception as e:
            self.logger.error(f"为REST API初始化标定系统失败: {e}")
            raise

    def _convert_quaternion_to_rpy(self, quaternion: List[float]) -> List[float]:
        """
        将四元数转换为 [roll, pitch, yaw] 格式

        参数:
            quaternion: 四元数列表，包含 qx, qy, qz, qw

        返回:
            [roll, pitch, yaw] 列表，角度单位为度数
        """
        try:
            from scipy.spatial.transform import Rotation as R

            rpy = R.from_quat(quaternion).as_euler("xyz", degrees=True)
            return [rpy[0], rpy[1], rpy[2]]
        except Exception as e:
            self.logger.error(f"转换TF数据失败: {e}")
            return [0.0, 0.0, 0.0]

    def _convert_joint_radians_to_degrees(self, joint_values: Dict[str, float]) -> Dict[str, float]:
        """
        将关节角度从弧度转换为度数

        参数:
            joint_values: 关节角度字典，键为关节名，值为弧度

        返回:
            关节角度字典，值为度数
        """
        if joint_values is None:
            return None
        return {joint_name: float(np.degrees(angle)) for joint_name, angle in joint_values.items()}

    def _make_json_serializable(self, obj):
        """递归地将numpy数组和其他不可序列化对象转换为JSON可序列化格式。"""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, dict):
            return {key: self._make_json_serializable(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [self._make_json_serializable(item) for item in obj]
        elif isinstance(obj, tuple):
            return tuple(self._make_json_serializable(item) for item in obj)
        else:
            return obj

    def _setup_static_files(self, app: FastAPI):
        """设置静态文件服务。"""
        html_file_path = self.config.rest_api.html_file_path
        html_dir = os.path.dirname(html_file_path)

        # 确保目录存在
        if os.path.exists(html_dir):
            try:
                # 挂载静态文件目录
                app.mount("/static", StaticFiles(directory=html_dir), name="static")
                self.logger.info(f"静态文件服务已挂载: {html_dir} -> /static")
            except Exception as e:
                self.logger.warning(f"挂载静态文件服务失败: {e}")
        else:
            self.logger.warning(f"静态文件目录不存在: {html_dir}")

    def _setup_routes(self, app: FastAPI):
        """设置API路由。"""

        @app.get("/")
        async def root():
            """根端点 - 返回相机标定HTML页面。"""
            html_file_path = self.config.rest_api.html_file_path
            if not os.path.exists(html_file_path):
                self.logger.error(f"HTML文件不存在: {html_file_path}")
                raise HTTPException(
                    status_code=404,
                    detail=f"HTML文件不存在: {html_file_path}。请检查配置文件中的 html_file_path 设置。",
                )
            return FileResponse(html_file_path)

        @app.get("/api/info", response_model=Dict[str, str])
        async def api_info():
            """API信息端点。"""
            return {
                "service": "PapJia相机标定服务",
                "version": "0.1.0",
                "docs": "/docs",
                "health": "/health",
            }

        @app.get("/health", response_model=HealthResponse)
        async def health_check():
            """健康检查端点。"""
            try:
                # 测试标定器连接
                calibration_healthy = self.calibrator is not None
                message = "服务正常" if calibration_healthy else "标定系统未初始化"

                available_operations = [
                    "get_all_folders",
                    "create_calibration_folder",
                    "select_calibration_folder",
                    "get_config",
                    "update_config",
                    "add_detection",
                    "get_detection",
                    "get_file_content",
                    "calibrate",
                    "health_check",
                ]

                return HealthResponse(
                    status="healthy" if calibration_healthy else "unhealthy",
                    calibration_service_healthy=calibration_healthy,
                    version="0.1.0",
                    available_operations=available_operations,
                    message=message,
                )
            except Exception as e:
                self.logger.error(f"健康检查错误: {e}")
                return HealthResponse(
                    status="unhealthy",
                    calibration_service_healthy=False,
                    version="0.1.0",
                    available_operations=[],
                    message=f"健康检查失败: {str(e)}",
                )

        @app.get("/folders/get_all", response_model=ApiResponse)
        async def get_all_folders():
            """获取所有标定文件夹。"""
            try:
                cwd = os.getcwd()
                root = getattr(self.calibrator, "root_folder", "")
                result = self.calibrator.get_all_folders()
                self.logger.info(f"get_all_folders cwd={cwd}, root_folder={root}, count={len(result) if result else 0}")
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(success=True, data=serializable_result, message="文件夹获取成功")
            except Exception as e:
                self.logger.error(f"获取文件夹错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/folders/create", response_model=ApiResponse)
        async def create_calibration_folder(request: CalibrationFolderCreate):
            """创建新的标定文件夹。"""
            try:
                result = self.calibrator.create_calibration_folder(request.calibration_folder)
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(success=True, data=serializable_result, message="标定文件夹创建成功")
            except Exception as e:
                self.logger.error(f"创建文件夹错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/folders/select", response_model=ApiResponse)
        async def select_calibration_folder(request: CalibrationFolderSelect):
            """选择标定文件夹。"""
            try:
                result = self.calibrator.select_calibration_folder(request.calibration_folder)
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(success=True, data=serializable_result, message="标定文件夹选择成功")
            except Exception as e:
                self.logger.error(f"选择文件夹错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.get("/config/get", response_model=ApiResponse)
        async def get_config():
            """获取当前标定配置。"""
            try:
                serializable_result = self._make_json_serializable(self.calibrator.config.to_dict())
                return ApiResponse(success=True, data=serializable_result, message="配置获取成功")
            except Exception as e:
                self.logger.error(f"获取配置错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.get("/config/get_default_capture_params", response_model=ApiResponse)
        async def get_default_capture_params():
            """获取默认采集参数。"""
            try:
                config = self.calibrator.config
                default_params = {
                    "image_topic": (
                        config.image_config.topic if hasattr(config, "image_config") else "/camera/color/image_raw"
                    ),
                    "camera_frame": (
                        config.image_config.camera_frame
                        if hasattr(config, "image_config") and hasattr(config.image_config, "camera_frame")
                        else "camera_color_optical_frame"
                    ),
                    "board_frame": (
                        config.image_config.board_frame
                        if hasattr(config, "image_config") and hasattr(config.image_config, "board_frame")
                        else "charuco"
                    ),
                    "joint_topic": (
                        config.end_effector_config.joint_topic
                        if hasattr(config, "end_effector_config")
                        else "/joint_states"
                    ),
                    "tool_frame": (
                        config.end_effector_config.tool_frame if hasattr(config, "end_effector_config") else "tool0"
                    ),
                    "base_frame": (
                        config.end_effector_config.base_frame if hasattr(config, "end_effector_config") else "base_link"
                    ),
                }
                # 从 calibration_sampler_config 获取采样相关参数
                if hasattr(config, "calibration_sampler_config"):
                    sampler_config = config.calibration_sampler_config
                    if hasattr(sampler_config, "sample_type"):
                        default_params["sample_type"] = sampler_config.sample_type
                    # 返回 sample_file（即使为空也返回，前端需要显示）
                    if hasattr(sampler_config, "sample_file"):
                        default_params["sample_file"] = sampler_config.sample_file or ""
                    if hasattr(sampler_config, "num_poses"):
                        default_params["sample_count"] = sampler_config.num_poses
                return ApiResponse(success=True, data=default_params, message="默认参数获取成功")
            except Exception as e:
                self.logger.error(f"获取默认参数错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.put("/config/update", response_model=ApiResponse)
        async def update_config(request: ConfigUpdate):
            """更新标定配置。"""
            try:
                self.calibrator.config.update_from_dict(request.config, self.logger)
                # 可能需要重建订阅器/校验/重建检测器
                if self.calibrator._validate_config():
                    self.calibrator._init_detectors()
                else:
                    raise ValueError("配置校验失败，未重新初始化检测器")
                serializable_result = self._make_json_serializable(self.calibrator.config.to_dict())
                return ApiResponse(success=True, data=serializable_result, message="配置更新成功")
            except Exception as e:
                self.logger.error(f"更新配置错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/detections/add_detection", response_model=ApiResponse)
        async def add_detection(request: DetectionAdd):
            """添加检测结果。"""
            try:
                result = self.calibrator.add_detection(request.image_path, request.add_flag, request.calibration_type)
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(success=True, data=serializable_result, message="检测结果添加成功")
            except Exception as e:
                self.logger.error(f"添加检测结果错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/detections/get", response_model=Dict[str, Any])
        async def get_detection(request: DetectionGet):
            """获取包含图像的检测结果。"""
            try:
                result = self.calibrator.get_detection(
                    request.image_path,
                    request.calibration_type,
                    request.ignore_board_pose,
                    request.ignore_end_pose,
                )

                if result.get("detection_success", False):
                    # 返回检测结果，不包含图像数据（使用直接文件服务）
                    response_data = {
                        "detection_success": True,
                        "message": result.get("message", "检测成功"),
                        "image_path": result.get("image_path"),
                        "detection_image_path": result.get("detection_image_path"),
                    }

                    return response_data
                else:
                    return {
                        "detection_success": False,
                        "message": result.get("message", "检测失败"),
                        "image_path": request.image_path,
                        "detection_image_path": None,
                    }

            except Exception as e:
                self.logger.error(f"获取检测结果错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/calibrate", response_model=ApiResponse)
        async def calibrate(request: CalibrationRequest):
            """执行标定。"""
            try:
                result = self.calibrator.calibrate(
                    calibration_type=request.calibration_type,
                    image_folder=request.image_folder,
                    pose_file=request.pose_file,
                    intrinsic_file=request.intrinsic_file,
                    use_selected_data=request.use_selected_data,
                )
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(
                    success=result.get("success", False), data=serializable_result, message=result.get("message", "")
                )
            except Exception as e:
                self.logger.error(f"标定错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/capture/image", response_model=ApiResponse)
        async def capture_image(save_path: Optional[str] = Query(None, description="保存路径（可选）")):
            """采集图像（单独接口）。"""
            try:
                result = self.calibrator.capture_image(save_path)
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(
                    success=result.get("success", False), data=serializable_result, message=result.get("message", "")
                )
            except Exception as e:
                self.logger.error(f"采集图像错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/capture/end_effector_pose", response_model=ApiResponse)
        async def capture_end_effector_pose():
            """采集末端执行器位姿（单独接口）。"""
            try:
                result = self.calibrator.capture_end_effector_pose()
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(
                    success=result.get("success", False), data=serializable_result, message=result.get("message", "")
                )
            except Exception as e:
                self.logger.error(f"采集末端位姿错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/capture/joint_angles", response_model=ApiResponse)
        async def capture_joint_angles():
            """采集关节角度（单独接口）。"""
            try:
                result = self.calibrator.capture_joint_angles()
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(
                    success=result.get("success", False), data=serializable_result, message=result.get("message", "")
                )
            except Exception as e:
                self.logger.error(f"采集关节角度错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/capture/single", response_model=ApiResponse)
        async def single_capture(request: SingleCaptureRequest):
            """单次采集接口，支持检查数据是否存在。"""
            try:
                warnings = []
                data_captured = {}
                image_folder = None
                camera_frame = None
                board_frame = None
                tool_frame = None
                base_frame = None
                # 检查并采集图像
                if request.enable_image:
                    image_folder = request.save_path if request.save_path else self.calibrator._get_origin_dir()

                # 检查并采集标定板TF
                if request.enable_board_tf:
                    if not request.camera_frame or not request.board_frame:
                        warnings.append("标定板TF的Frame未设置，跳过标定板TF采集")
                    else:
                        camera_frame = request.camera_frame
                        board_frame = request.board_frame

                # 检查并采集末端TF
                if request.enable_end_tf:
                    if not request.tool_frame or not request.base_frame:
                        warnings.append("末端TF的Frame未设置，跳过末端TF采集")
                    else:
                        base_frame = request.base_frame
                        tool_frame = request.tool_frame

                data_captured = self.calibrator.collect_calibration_sample(
                                image_folder=image_folder,
                                camera_frame=camera_frame,
                                board_frame=board_frame,
                                tool_frame=tool_frame,
                                base_frame=base_frame
                            )
                # 构建响应
                success = len(data_captured.keys()) > 0  # 至少有一个采集成功
                message = "采集完成" if success else "采集失败"
                
                if success:
                   res = self.calibrator.save_capture_to_poses_file(data_captured)
                   message += res["message"]
                if warnings:
                    message += f"，警告: {'; '.join(warnings)}"

                if "joint_values" in data_captured:
                    for key, value in data_captured["joint_values"].items():
                        data_captured["joint_values"][key] = np.rad2deg(value)

                serializable_results = self._make_json_serializable(data_captured)

                return ApiResponse(
                    success=success, data={"results": serializable_results, "warnings": warnings}, message=message
                )
            except Exception as e:
                self.logger.error(f"单次采集错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/capture/auto", response_model=ApiResponse)
        async def auto_capture(request: AutoCaptureRequest):
            """自动采集接口，调用 collect_calibration_samples。"""
            try:
                config = self.calibrator.config

                # 参数取值：请求优先，其次配置默认值
                image_topic = request.image_topic or (
                    config.image_config.topic if hasattr(config, "image_config") else "/camera/color/image_raw"
                )
                camera_frame = request.camera_frame or (
                    config.image_config.camera_frame
                    if hasattr(config, "image_config") and hasattr(config.image_config, "camera_frame")
                    else "camera_color_optical_frame"
                )
                board_frame = request.board_frame or (
                    config.image_config.board_frame
                    if hasattr(config, "image_config") and hasattr(config.image_config, "board_frame")
                    else "charuco"
                )
                joints_topic = request.joints_topic or (
                    config.end_effector_config.joint_topic
                    if hasattr(config, "end_effector_config")
                    else "/joint_states"
                )
                tool_frame = request.tool_frame or (
                    config.end_effector_config.tool_frame if hasattr(config, "end_effector_config") else "tool0"
                )
                base_frame = request.base_frame or (
                    config.end_effector_config.base_frame if hasattr(config, "end_effector_config") else "base_link"
                )
                group_name = request.group_name or (
                    config.calibration_sampler_config.group_name
                    if hasattr(config, "calibration_sampler_config")
                    else "left_arm"
                )

                # 调用 collect_calibration_samples，poses_recorded_path 由函数内部自动生成
                result = self.calibrator.collect_calibration_samples(
                    image_topic=image_topic,
                    camera_frame=camera_frame,
                    board_frame=board_frame,
                    joints_topic=joints_topic,
                    tool_frame=tool_frame,
                    base_frame=base_frame,
                    group_name=group_name,
                    wait_after_motion=request.wait_after_motion,
                    sample_type=request.sample_type,
                    sample_file=request.sample_file or "",
                    num_poses=request.sample_count,
                )

                serializable_result = self._make_json_serializable(result)
                return ApiResponse(
                    success=result.get("success", False), data=serializable_result, message=result.get("message", "")
                )
            except Exception as e:
                self.logger.error(f"自动采集错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.get("/images/{file_path:path}")
        async def get_image_file(file_path: str):
            """
            直接返回图像文件（支持压缩以加快前端显示）。
            压缩参数从配置文件读取。

            参数:
                file_path: 图像文件路径
            """
            try:
                # 检查文件是否存在
                if not os.path.exists(file_path):
                    raise HTTPException(status_code=404, detail=f"文件不存在: {file_path}")

                # 检查是否为图像文件
                if not file_path.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".tiff")):
                    raise HTTPException(status_code=400, detail="不是支持的图像格式")

                # 如果配置中启用了压缩，对所有图像进行压缩
                if self.config.rest_api.image_compress_enabled:
                    try:
                        compressed_data = self._compress_image(file_path)
                        return Response(
                            content=compressed_data,
                            media_type="image/jpeg",
                            headers={"Content-Disposition": f'inline; filename="{os.path.basename(file_path)}"'},
                        )
                    except Exception as e:
                        self.logger.warning(f"图像压缩失败，返回原图: {e}")
                        # 压缩失败时返回原图
                        pass

                # 根据文件扩展名确定MIME类型
                file_ext = os.path.splitext(file_path)[1].lower()
                mime_types = {
                    ".jpg": "image/jpeg",
                    ".jpeg": "image/jpeg",
                    ".png": "image/png",
                    ".bmp": "image/bmp",
                    ".tiff": "image/tiff",
                }
                media_type = mime_types.get(file_ext, "image/jpeg")

                return FileResponse(path=file_path, media_type=media_type, filename=os.path.basename(file_path))

            except HTTPException:
                raise
            except Exception as e:
                self.logger.error(f"获取图像文件错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/files/upload", response_model=ApiResponse)
        async def upload_file(file: UploadFile = File(...), directory: str = Form(...)):
            """上传文件到指定目录。"""
            try:
                # 直接使用directory作为目标路径（已经是全局路径）
                target_dir = directory
                os.makedirs(target_dir, exist_ok=True)

                # 构建文件路径
                file_path = os.path.join(target_dir, file.filename)

                # 保存文件
                with open(file_path, "wb") as buffer:
                    content = await file.read()
                    buffer.write(content)

                self.logger.info(f"文件上传成功: {file_path}")
                return ApiResponse(success=True, data={"file_path": file_path}, message="文件上传成功")

            except Exception as e:
                self.logger.error(f"文件上传错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/files/delete", response_model=ApiResponse)
        async def delete_file(request: FileContentRequest):
            """删除指定文件（仅限标定根目录下的文件），并从 *_used 列表移除路径。"""
            try:
                file_path = request.file_path
                if not file_path:
                    return ApiResponse(success=False, data=None, message="文件路径为空")

                # 标定根目录
                root_dir = os.path.abspath(self.calibrator.root_folder)

                # 规范化路径：相对路径按根目录解析
                abs_path = os.path.abspath(file_path if os.path.isabs(file_path) else os.path.join(root_dir, file_path))

                # 路径安全校验：必须在根目录下
                if os.path.commonpath([abs_path, root_dir]) != root_dir:
                    return ApiResponse(success=False, data=None, message="禁止删除根目录之外的文件")

                if not os.path.exists(abs_path):
                    return ApiResponse(success=False, data=None, message=f"文件不存在: {abs_path}")
                if os.path.isdir(abs_path):
                    return ApiResponse(success=False, data=None, message="不支持删除目录")

                os.remove(abs_path)
                self.logger.info(f"文件已删除: {abs_path}")

                # 同步清理 poses_recorded_file 中的记录
                try:
                    removed = self.calibrator.remove_pose_records_by_image(abs_path)
                    if removed > 0:
                        self.logger.info(f"已从 poses_recorded_file 移除 {removed} 条记录 (image_path={abs_path})")
                except Exception as e:
                    self.logger.warning(f"清理 poses_recorded_file 记录失败: {e}")

                # 从 *_used 列表移除路径
                def _remove_from_used_lists(paths_to_remove):
                    used_files = [
                        self.calibrator._get_intrinsic_images_used_path(),
                        self.calibrator._get_extrinsic_images_used_path(),
                    ]
                    for used_file in used_files:
                        if not os.path.exists(used_file):
                            continue
                        try:
                            with open(used_file, "r", encoding="utf-8") as f:
                                lines = f.readlines()
                            new_lines = [line for line in lines if line.strip() not in paths_to_remove]
                            if len(new_lines) != len(lines):
                                with open(used_file, "w", encoding="utf-8") as f:
                                    f.writelines(new_lines)
                                self.logger.info(f"已从列表移除路径: {paths_to_remove} in {used_file}")
                        except Exception as e:
                            self.logger.warning(f"更新已用列表失败 {used_file}: {e}")

                _remove_from_used_lists({abs_path})

                return ApiResponse(success=True, data={"file_path": abs_path}, message="文件删除成功")
            except Exception as e:
                self.logger.error(f"文件删除错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.post("/files/content", response_model=ApiResponse)
        async def get_file_content(request: FileContentRequest):
            """获取文件内容（非图像文件）。"""
            try:
                file_path = request.file_path

                # 检查文件是否存在
                if not os.path.exists(file_path):
                    return ApiResponse(success=False, data=None, message=f"文件不存在: {file_path}")

                # 图像文件请使用 /images/{file_path} 接口
                if file_path.lower().endswith((".jpg", ".jpeg", ".png", ".bmp", ".tiff")):
                    return ApiResponse(success=False, data=None, message="图像文件请使用 /images/{file_path} 接口获取")

                # 处理非图像文件
                result = self.calibrator.get_file_content(file_path)
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(success=True, data=serializable_result, message="文件内容获取成功")

            except Exception as e:
                self.logger.error(f"获取文件内容错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        @app.get("/folders/structure", response_model=ApiResponse)
        async def get_folder_structure_by_path(folder_path: str = Query(..., description="文件夹路径")):
            """根据路径获取文件夹结构。"""
            try:
                result = self.calibrator.get_folder_structure(folder_path)
                serializable_result = self._make_json_serializable(result)
                return ApiResponse(success=True, data=serializable_result, message="文件夹结构获取成功")
            except Exception as e:
                self.logger.error(f"获取文件夹结构错误: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        # 异常处理器
        @app.exception_handler(HTTPException)
        async def http_exception_handler(request, exc):
            return JSONResponse(
                status_code=exc.status_code,
                content={"success": False, "message": exc.detail, "error_code": f"HTTP_{exc.status_code}"},
            )

        @app.exception_handler(Exception)
        async def general_exception_handler(request, exc):
            self.logger.error(f"未处理的异常: {exc}")
            return JSONResponse(
                status_code=500,
                content={"success": False, "message": "内部服务器错误", "error_code": "INTERNAL_ERROR"},
            )


def create_api(config: ServiceConfig) -> CalibrationAPI:
    """
    创建REST API实例的工厂函数。

    Args:
        config: 服务配置

    Returns:
        CalibrationAPI实例
    """
    return CalibrationAPI(config)


def run_api_server(config_path: Optional[str] = None):
    """
    运行REST API服务器。

    Args:
        config_path: 配置文件路径
    """
    from .config import load_config

    # 加载配置
    config = load_config(config_path)

    if not config.rest_api.enabled:
        print("配置中禁用了REST API服务器")
        return

    # 设置日志
    logging.basicConfig(level=getattr(logging, config.logging.level), format=config.logging.format)

    # 创建API
    api = create_api(config)

    # 运行服务器
    uvicorn.run(
        api.app,
        host=config.rest_api.host,
        port=config.rest_api.port,
        workers=config.rest_api.workers,
        reload=config.rest_api.reload,
        log_level=config.logging.level.lower(),
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="运行PapJia相机标定REST API服务器")
    parser.add_argument(
        "--config",
        "-c",
        help="配置文件路径",
        default="/workspace/src/vision_calibration/config/config.yaml",
    )

    args = parser.parse_args()
    run_api_server(args.config)
