"""
PapJia 规划器任务管理器的 ZeroMQ 客户端模块。

本模块为其他服务提供便捷的客户端接口，
用于通过 ZeroMQ 与任务管理器服务进行通信。
"""

import zmq
import hashlib
import json
import logging
import numpy as np
import cv2
from typing import Dict, Any, Optional, List


def verify_image_integrity(img_bytes, image_info, image_type):
    """验证图像完整性"""
    # 获取元数据中的图像信息
    if image_type != "image" and image_type != "detection_image":
        print(f"错误: 未知的图像类型 {image_type}")
        return False

    # 检查大小
    if len(img_bytes) != image_info["size"]:
        print(f"警告: {image_type} 大小不匹配 (元数据: {image_info['size']}, 实际: {len(img_bytes)})")
        return False
    # 检查哈希值
    received_hash = hashlib.md5(img_bytes).hexdigest()
    if received_hash != image_info["hash"]:
        print(f"错误: {image_type} 哈希不匹配 (元数据: {image_info['hash']}, 实际: {received_hash})")
        return False

    print(f"{image_type} 完整性验证通过")
    return True


def process_image_frame(img_bytes, image_info, image_type):
    """处理单个图像帧"""
    # 验证图像完整性
    if not verify_image_integrity(img_bytes, image_info, image_type):
        print(f"警告: {image_type} 完整性验证失败")

    # 将字节数据转换为图像
    img_array = np.frombuffer(img_bytes, dtype=np.uint8)
    image = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

    # 显示图像信息
    print(f"{image_type} 尺寸: {image.shape[1]}x{image.shape[0]}")

    return image


class ZMQTaskManagerClient:
    """
    用于任务管理操作的 ZeroMQ 客户端。

    提供高级接口，通过 ZeroMQ 消息与任务管理器服务进行通信。
    """

    def __init__(self, server_address: str = "tcp://127.0.0.1:5559", timeout: int = 5000):
        """初始化 ZeroMQ 客户端。"""
        self.server_address = server_address
        self.timeout = timeout
        self.context = zmq.Context()
        self.socket = None
        self.logger = logging.getLogger(__name__)

        # 连接到服务器
        self._connect()

    def _connect(self):
        """连接到 ZeroMQ 服务器。"""
        try:
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.setsockopt(zmq.SNDTIMEO, self.timeout)
            self.socket.connect(self.server_address)
            self.logger.info(f"已连接到 ZeroMQ 服务器: {self.server_address}")
        except Exception as e:
            self.logger.error(f"连接 ZeroMQ 服务器失败: {e}")
            raise

    def _send_request(self, operation: str, params: Dict[str, Any]) -> Dict[str, Any]:
        """向服务器发送请求并返回响应。"""
        request = {"operation": operation, "params": params}

        try:
            # 发送请求
            self.socket.send_json(request)

            # 接收响应
            if operation == "get_detection":
                response = self.socket.recv_multipart()
            else:
                response = self.socket.recv_json()

            return response

        except zmq.Again:
            # 发生超时
            self.logger.error(f"操作请求超时: {operation}")
            # 超时时关闭并重建 socket
            self._reconnect()
            raise TimeoutError(f"操作请求超时: {operation}")

        except Exception as e:
            self.logger.error(f"通信错误: {e}")
            self._reconnect()
            raise ConnectionError(f"与服务器通信失败: {e}")

    def _reconnect(self):
        """在错误后重新连接到服务器。"""
        try:
            if self.socket:
                self.socket.close()
            self._connect()
        except Exception as e:
            self.logger.error(f"重新连接失败: {e}")

    def _handle_response(self, response: Dict[str, Any]) -> Any:
        """处理服务器响应并提取结果或抛出异常。"""
        if response.get("status") == "success":
            return response.get("result")
        elif response.get("status") == "error":
            error_message = response.get("message", "Unknown error")
            error_code = response.get("code", "UNKNOWN_ERROR")
            raise RuntimeError(f"服务器错误 [{error_code}]: {error_message}")
        else:
            raise RuntimeError(f"无效的响应格式: {response}")

    def _handle_response_with_image(self, frames: List[Any]) -> Dict[str, Any]:
        """
        处理服务器响应并提取结果或抛出异常。
        """
        # 打印每个frame的类型和内容（截取前100个字符）
        for i, frame in enumerate(frames):
            frame_type = type(frame)
            if isinstance(frame, bytes):
                try:
                    content_preview = frame[:100].decode("utf-8", errors="ignore")
                except:
                    content_preview = str(frame[:100])
            else:
                content_preview = str(frame)[:100]

        if len(frames) != 3:
            return {"detection_success": False, "message": "响应格式不正确"}
        res_dict, img_bytes, detection_bytes = frames

        res_dict = json.loads(res_dict)
        if res_dict.get("detection_success", False):
            metadata = res_dict.get("metadata", {})
            img_bytes = process_image_frame(img_bytes, metadata.get("image", {}), "image")
            detection_bytes = process_image_frame(
                detection_bytes, metadata.get("detection_image", {}), "detection_image"
            )
            res_dict["image"] = img_bytes
            res_dict["detection_image"] = detection_bytes

        return res_dict

    def calibrate(
        self,
        calibration_type: str = None,
        image_folder: str = None,
        pose_file: str = None,
        intrinsic_file: str = None,
        use_selected_data: bool = False,
    ) -> Dict[str, Any]:
        """
        执行标定。
        """
        response = self._send_request(
            "calibrate",
            {
                "calibration_type": calibration_type,
                "image_folder": image_folder,
                "pose_file": pose_file,
                "intrinsic_file": intrinsic_file,
                "use_selected_data": use_selected_data,
            },
        )
        return self._handle_response(response)

    def get_all_folders(self) -> Dict[str, Any]:
        """
        获取所有标定文件夹。
        """
        response = self._send_request("get_all_folders", {})
        return self._handle_response(response)

    def create_calibration_folder(self, calibration_folder: str) -> Dict[str, Any]:
        """
        创建标定文件夹。
        """
        response = self._send_request("create_calibration_folder", {"calibration_folder": calibration_folder})
        return self._handle_response(response)

    def select_calibration_folder(self, calibration_folder: str) -> Dict[str, Any]:
        """
        选择标定文件夹。
        """
        if not calibration_folder:
            raise ValueError("标定文件夹参数是必需的")
        response = self._send_request("select_calibration_folder", {"calibration_folder": calibration_folder})
        return self._handle_response(response)

    def get_config(self) -> Dict[str, Any]:
        """
        获取标定服务的配置。
        """
        response = self._send_request("get_config", {})
        return self._handle_response(response)

    def update_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        """
        更新标定服务的配置。
        """
        if not config:
            raise ValueError("配置参数是必需的")
        response = self._send_request("update_config", {"config": config})
        return self._handle_response(response)

    def load_camera_params(self, camera_params_file: str) -> Dict[str, Any]:
        """
        加载摄像机参数。
        """
        if not camera_params_file:
            raise ValueError("摄像机参数文件是必需的")
        response = self._send_request("load_camera_params", {"camera_params_file": camera_params_file})
        return self._handle_response(response)

    def load_extrinsic_data(self) -> Dict[str, Any]:
        """
        加载外参数据。
        """
        response = self._send_request("load_extrinsic_data", {})
        return self._handle_response(response)

    def add_detection(self, image_path: str, calibration_type: str, add_flag: bool) -> Dict[str, Any]:
        """
        添加检测。
        """
        if not image_path:
            raise ValueError("图像路径是必需的")
        response = self._send_request(
            "add_detection",
            {
                "image_path": image_path,
                "calibration_type": calibration_type,
                "add_flag": add_flag,
            },
        )

        return self._handle_response(response)

    def get_detection(
        self, image_path: str, calibration_type: str, ignore_board_pose: bool, ignore_end_pose: bool
    ) -> Dict[str, Any]:
        """
        获取检测。
        """
        if not image_path:
            raise ValueError("图像路径是必需的")
        frames = self._send_request(
            "get_detection",
            {
                "image_path": image_path,
                "calibration_type": calibration_type,
                "ignore_board_pose": ignore_board_pose,
                "ignore_end_pose": ignore_end_pose,
            },
        )
        return self._handle_response_with_image(frames)

    def get_board_pose(self, image_path: str, ignore_end_pose: bool) -> Dict[str, Any]:
        """
        获取标定板位姿。
        """
        response = self._send_request(
            "get_board_pose",
            {
                "image_path": image_path,
                "ignore_end_pose": ignore_end_pose,
            },
        )
        return self._handle_response(response)

    def get_file_content(self, file_path: str) -> Dict[str, Any]:
        """
        获取文件内容。
        """
        if not file_path:
            raise ValueError("文件路径是必需的")
        response = self._send_request("get_file_content", {"file_path": file_path})
        return self._handle_response(response)

    def health_check(self) -> Dict[str, Any]:
        """检查任务管理器服务的健康状态。"""
        response = self._send_request("health_check", {})
        return self._handle_response(response)

    def close(self):
        """关闭客户端连接。"""
        if self.socket:
            self.socket.close()
            self.logger.info("ZeroMQ 客户端连接已关闭")

        if self.context:
            self.context.term()
            self.logger.info("ZeroMQ 上下文已终止")

    def __enter__(self):
        """上下文管理器入口。"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口。"""
        self.close()


# 简便使用的便利函数
def create_client(server_address: str = "tcp://127.0.0.1:5559", timeout: int = 5000) -> ZMQTaskManagerClient:
    """创建新的 ZeroMQ 任务管理器客户端。"""
    return ZMQTaskManagerClient(server_address, timeout)
