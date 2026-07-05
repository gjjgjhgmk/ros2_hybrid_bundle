"""
PapJia相机标定服务的ZeroMQ服务器模块。

该模块为相机标定操作提供ZeroMQ接口，
直接使用现有的calibration_camera.py和calibration_handeye.py接口。
"""

import zmq
import cv2
import hashlib
import json
import threading
import logging
import signal
import sys
import numpy as np
from typing import Dict, Any, Optional, List, Tuple
from concurrent.futures import ThreadPoolExecutor

from .calibration import CalibrationManager
from .config import ServiceConfig


def encode_image(image, format=".jpg", quality=90):
    """编码图像为字节数据"""
    if format == ".jpg":
        _, img_encoded = cv2.imencode(format, image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    else:
        _, img_encoded = cv2.imencode(format, image)
    return img_encoded.tobytes()


def create_image_dict(image) -> Tuple[Dict[str, Any], bytes]:
    image_bytes = encode_image(image, ".jpg", 90)
    image_hash = hashlib.md5(image_bytes).hexdigest()
    image_dict = {
        "size": len(image_bytes),
        "hash": image_hash,
        "width": image.shape[1],
        "height": image.shape[0],
        "format": "JPEG",
    }
    return image_dict, image_bytes


class ZMQCalibrationServer:
    """
    直接使用现有标定接口的ZeroMQ服务器。

    支持使用JSON消息格式的请求-回复模式进行标定操作。
    """

    def __init__(self, config: ServiceConfig):
        """
        初始化ZeroMQ服务器。

        Args:
            config: 包含ZeroMQ设置的服务配置
        """
        self.config = config
        self.context = zmq.Context()
        self.socket = None
        self.executor = None
        self.running = False
        self.calibrator = None

        # 设置日志
        self.logger = logging.getLogger(__name__)

        # 设置信号处理器以便优雅关闭
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _make_json_serializable(self, obj):
        """递归地将numpy数组和其他不可序列化对象转换为JSON可序列化的格式"""
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

    def _signal_handler(self, signum, frame):
        """优雅地处理关闭信号。"""
        self.logger.info(f"接收到信号 {signum}，正在关闭...")
        self.stop()
        sys.exit(0)

    def start(self):
        """启动ZeroMQ服务器。"""
        if not self.config.zeromq.enabled:
            self.logger.info("ZeroMQ服务器在配置中被禁用")
            return

        try:
            # 创建标定管理器
            self.calibrator = CalibrationManager(self.config.zeromq.calibration_config_file)
            if self.calibrator._validate_config():
                self.calibrator._init_detectors()
                self.calibrator.load_camera_params()
            # 创建套接字
            self.socket = self.context.socket(zmq.REP)
            bind_address = self.config.zeromq.get_bind_address()
            self.socket.bind(bind_address)

            # 创建处理请求的线程池
            self.executor = ThreadPoolExecutor(max_workers=self.config.zeromq.max_workers)

            self.running = True
            self.logger.info(f"ZeroMQ服务器已在 {bind_address} 启动")

            # 主服务器循环
            self._server_loop()

        except Exception as e:
            self.logger.error(f"启动ZeroMQ服务器失败: {e}")
            self.stop()
            raise

    def _server_loop(self):
        """处理传入请求的主服务器循环。"""
        while self.running:
            try:
                # 带超时接收请求
                if self.socket.poll(1000):  # 1秒超时
                    message = self.socket.recv_json(zmq.NOBLOCK)

                    # 异步处理请求
                    future = self.executor.submit(self._process_request, message)
                    response = future.result()  # 等待完成

                    # 发送响应
                    if isinstance(response, list):
                        self.logger.info(f"发送多部分响应: {len(response)} 帧")
                        self.socket.send_multipart(response)
                    else:
                        self.socket.send_json(response)

            except zmq.Again:
                # 超时，继续循环
                continue
            except Exception as e:
                self.logger.error(f"服务器循环中发生错误: {e}")
                # 如果可能的话发送错误响应
                try:
                    error_response = {"status": "error", "message": str(e), "code": "INTERNAL_ERROR"}
                    self.logger.error(f"发送错误响应: {error_response}")
                    self.socket.send_json(error_response)
                except:
                    pass

    def _process_request(self, message: Dict[str, Any]) -> Dict[str, Any]:
        """
        处理标定请求并返回响应。

        Args:
            message: 包含操作和参数的请求消息

        Returns:
            包含操作结果的响应消息
        """
        try:
            # 验证请求格式
            if not isinstance(message, dict) or "operation" not in message:
                return {
                    "status": "error",
                    "message": "无效的请求格式。缺少 'operation' 字段。",
                    "code": "INVALID_REQUEST",
                }

            operation = message["operation"]
            params = message.get("params", {})

            self.logger.info(f"正在处理操作: {operation}")

            # 路由到合适的标定操作
            if operation == "get_all_folders":
                result = self._handle_get_all_folders(params)
            elif operation == "create_calibration_folder":
                result = self._handle_create_calibration_folder(params)
            elif operation == "select_calibration_folder":
                result = self._handle_select_calibration_folder(params)
            elif operation == "get_config":
                result = self._handle_get_config(params)
            elif operation == "update_config":
                result = self._handle_update_config(params)
            elif operation == "load_camera_params":
                result = self._handle_load_camera_params(params)
            elif operation == "load_extrinsic_data":
                result = self._handle_load_extrinsic_data(params)
            elif operation == "add_detection":
                result = self._handle_add_detection(params)
            elif operation == "health_check":
                result = self._handle_health_check()
            elif operation == "get_detection":
                # 此处返回图像，需要特殊处理
                result = self._handle_get_detection(params)
                return result
            elif operation == "get_board_pose":
                result = self._handle_get_board_pose(params)
            elif operation == "get_file_content":
                result = self._handle_get_file_content(params)
            elif operation == "calibrate":
                result = self._handle_calibrate(params)
            else:
                return {
                    "status": "error",
                    "message": f"未知操作: {operation}",
                    "code": "UNKNOWN_OPERATION",
                }

            # 确保结果可以JSON序列化
            serializable_result = self._make_json_serializable(result)
            return {"status": "success", "result": serializable_result}

        except Exception as e:
            self.logger.error(f"处理请求时发生错误: {e}")
            return {"status": "error", "message": str(e), "code": "PROCESSING_ERROR"}

    def _handle_get_all_folders(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理获取所有标定文件夹操作。"""
        return self.calibrator.get_all_folders()

    def _handle_create_calibration_folder(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理创建标定文件夹操作。"""
        calibration_folder = params.get("calibration_folder", None)
        return self.calibrator.create_calibration_folder(calibration_folder)

    def _handle_select_calibration_folder(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理选择标定文件夹操作。"""
        folder = params.get("calibration_folder", None)
        if not folder:
            return {"status": "error", "message": "缺少 'calibration_folder' 参数", "code": "INVALID_REQUEST"}
        return self.calibrator.select_calibration_folder(folder)

    def _handle_get_config(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理获取更新操作。"""
        return self.calibrator.config.to_dict()

    def _handle_update_config(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理更新配置操作。"""
        config = params.get("config", None)
        if not config:
            return {"status": "error", "message": "缺少 'config' 参数", "code": "INVALID_REQUEST"}
        self.calibrator.config.update_from_dict(config, self.logger)
        if self.calibrator._validate_config():
            self.calibrator._init_detectors()
        else:
            raise ValueError("配置校验失败，未重新初始化检测器")
        return self.calibrator.config.to_dict()

    def _handle_load_camera_params(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理加载相机参数操作。"""
        camera_params_file = params.get("camera_params_file", None)
        if not camera_params_file:
            self.logger.warning("缺少 'camera_params_file' 参数，使用默认参数")
        return self.calibrator.load_camera_params(camera_params_file)

    def _handle_load_extrinsic_data(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理加载外参数据操作。"""
        return self.calibrator.load_extrinsic_data()

    def _handle_get_board_pose(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理获取标定板位姿操作。"""
        image_path = params.get("image_path", None)
        calibration_type = "extrinsic"
        ignore_board_pose = False
        ignore_end_pose = params.get("ignore_end_pose", True)
        detection_result = self.calibrator.get_detection(
            image_path, calibration_type, ignore_board_pose, ignore_end_pose
        )
        res = {
            "success": False,
            "message": detection_result.get("message", None),
        }
        if detection_result.get("detection_success", False):
            board_pose = detection_result.get("board_pose", None)
            if not board_pose:
                return {
                    "success": False,
                    "message": "检测到标定板，但未得到board_pose；请确认已加载相机内参",
                }
            position = self._make_json_serializable(board_pose.get("translation_vector"))
            quaternion = self._make_json_serializable(board_pose.get("quaternion"))
            euler_angles = self._make_json_serializable(board_pose.get("rotation_vector"))
            res = {
                "success": True,
                "message": detection_result.get("message", "检测成功"),
                "position": position,
                "quaternion": quaternion,
                "euler_angles": euler_angles,
                "pose": list(position) + list(quaternion),
            }
        return res

    def _handle_add_detection(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理添加检测结果操作。"""
        image_path = params.get("image_path", None)
        calibration_type = params.get("calibration_type", None)
        add_flag = params.get("add_flag", True)
        return self.calibrator.add_detection(image_path, add_flag, calibration_type)

    def _handle_get_detection(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理获取检测结果操作。"""
        image_path = params.get("image_path", None)
        calibration_type = params.get("calibration_type", None)
        ignore_board_pose = params.get("ignore_board_pose", False)
        ignore_end_pose = params.get("ignore_end_pose", False)
        self.logger.info(f"开始获取检测结果: {image_path}, {calibration_type}, {ignore_board_pose}, {ignore_end_pose}")
        res = self.calibrator.get_detection(image_path, calibration_type, ignore_board_pose, ignore_end_pose)
        res_dict = {
            "message": None,
            "image_path": None,
            "detection_image_path": None,
            "detection_success": False,
        }
        image_bytes = None
        detection_bytes = None
        frames = []
        if res.get("detection_success", False):
            image_dict, image_bytes = create_image_dict(res.get("image", None))
            detection_dict, detection_bytes = create_image_dict(res.get("detection_image", None))
            res_dict.update(
                {
                    "message": res.get("message", "检测成功"),
                    "image_path": res.get("image_path", None),
                    "detection_image_path": res.get("detection_image_path", None),
                    "detection_success": res.get("detection_success", False),
                    "metadata": {
                        "image": image_dict,
                        "detection_image": detection_dict,
                    },
                }
            )
            frames.append(json.dumps(res_dict).encode("utf-8"))
            frames.append(image_bytes)
            frames.append(detection_bytes)
        else:
            res_dict = {
                "status": "error",
                "message": res.get("message", "检测失败"),
                "code": "DETECTION_FAILED",
            }
            image_bytes = b"null"
            detection_bytes = b"null"
            frames.append(json.dumps(res_dict).encode("utf-8"))
            frames.append(image_bytes)
            frames.append(detection_bytes)
        return frames

    def _handle_calibrate(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理标定操作。"""
        return self.calibrator.calibrate(
            calibration_type=params.get("calibration_type"),
            image_folder=params.get("image_folder"),
            pose_file=params.get("pose_file"),
            intrinsic_file=params.get("intrinsic_file"),
            use_selected_data=params.get("use_selected_data", False),
        )

    def _handle_get_file_content(self, params: Dict[str, Any]) -> Dict[str, Any]:
        """处理获取文件内容操作。"""
        file_path = params.get("file_path", None)
        return self.calibrator.get_file_content(file_path)

    def _handle_health_check(self) -> Dict[str, Any]:
        """处理健康检查操作。"""
        try:
            return {
                "service_healthy": True,
                "message": "标定服务正常",
                "available_operations": [
                    "calibrate_charuco",
                    "calibrate_chessboard",
                    "calibrate_handeye",
                    "health_check",
                ],
            }
        except Exception as e:
            return {"service_healthy": False, "message": f"服务错误: {str(e)}"}

    def stop(self):
        """优雅地停止ZeroMQ服务器。"""
        self.logger.info("正在停止ZeroMQ服务器...")

        self.running = False

        if self.executor:
            self.executor.shutdown(wait=True)
            self.logger.info("线程池执行器已关闭")

        if self.socket:
            self.socket.close()
            self.logger.info("ZeroMQ套接字已关闭")

        if self.context:
            self.context.term()
            self.logger.info("ZeroMQ上下文已终止")


def create_zmq_server(config: ServiceConfig) -> ZMQCalibrationServer:
    """
    创建ZeroMQ标定服务器的工厂函数。

    Args:
        config: 服务配置

    Returns:
        ZMQCalibrationServer实例
    """
    return ZMQCalibrationServer(config)


def run_zmq_server(config_path: Optional[str] = None):
    """
    作为独立进程运行ZeroMQ服务器。

    Args:
        config_path: 配置文件路径
    """
    from .config import load_config

    # 加载配置
    config = load_config(config_path)

    # 设置日志
    logging.basicConfig(level=getattr(logging, config.logging.level), format=config.logging.format)

    # 创建并启动服务器
    server = create_zmq_server(config)

    try:
        server.start()
    except KeyboardInterrupt:
        logging.info("接收到键盘中断")
    finally:
        server.stop()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="运行PapJia相机标定ZeroMQ服务器")
    parser.add_argument(
        "--config",
        "-c",
        help="配置文件路径",
        default="/workspace/src/vision_calibration/config/config.yaml",
    )

    args = parser.parse_args()
    run_zmq_server(args.config)
