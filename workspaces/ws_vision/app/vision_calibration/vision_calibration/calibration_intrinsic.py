#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机内参标定模块

提供相机内参标定功能，支持ChArUco和棋盘格标定板
优化版本：重用board_detector.py中的优化功能，集成标定逻辑
"""

import os
import cv2
import numpy as np
import logging
from typing import Dict, Any, Optional, List, Tuple
from .board_detector import BoardDetector, BoardConfig, BoardType


class IntrinsicCalibrator:
    """
    内参标定器

    优化版本：集成标定逻辑，无重复代码，
    提供便利的文件处理接口
    """

    def __init__(self, board_config: BoardConfig):
        """
        初始化内参标定器

        参数:
            board_config: 标定板配置
        """
        self.board_config = board_config
        self.logger = logging.getLogger(__name__)

        # 初始化检测器
        self.detector = BoardDetector(board_config)

        # 支持的图像扩展名
        self.supported_extensions = ["jpg", "jpeg", "png", "bmp", "tiff"]

    def calibrate_from_folder(self, image_folder: str) -> Dict[str, Any]:
        """
        从文件夹进行内参标定

        参数:
            image_folder: 图像文件夹路径

        返回:
            标定结果
        """
        try:
            if not os.path.exists(image_folder):
                self.logger.error(f"文件夹不存在: {image_folder}")
                return {"success": False, "message": f"文件夹不存在: {image_folder}"}

            # 获取图像文件列表
            image_files = self._get_image_files_from_folder(image_folder)

            if not image_files:
                self.logger.error(f"文件夹中没有找到支持的图像文件: {image_folder}")
                return {"success": False, "message": f"文件夹中没有找到图像: {image_folder}"}

            self.logger.info(f"从文件夹找到{len(image_files)}个图像文件: {image_folder}")
            return self.calibrate_from_image_list(image_files)

        except Exception as e:
            self.logger.error(f"从文件夹标定失败: {e}")
            return {"success": False, "message": f"从文件夹标定失败: {str(e)}"}

    def calibrate_from_image_list(self, image_paths: List[str]) -> Dict[str, Any]:
        """
        从图像文件列表进行内参标定

        参数:
            image_paths: 图像文件路径列表

        返回:
            标定结果
        """
        try:
            self.logger.info(
                f"开始内参标定，处理{len(image_paths)}个图像文件，标定板类型: {self.board_config.board_type}"
            )

            all_corners = []
            all_ids = []
            image_size = None
            failed_count = 0
            valid_count = 0

            # 逐个处理图像，节省内存
            for i, image_path in enumerate(image_paths):
                if not os.path.exists(image_path):
                    failed_count += 1
                    self.logger.warning(f"图像文件不存在: {image_path}")
                    continue

                image = cv2.imread(image_path)
                if image is None:
                    failed_count += 1
                    self.logger.warning(f"无法读取图像: {image_path}")
                    continue

                if image_size is None:
                    image_size = (image.shape[1], image.shape[0])

                self.logger.debug(f"处理图像: {os.path.basename(image_path)}")
                detection_result = self.detector.detect_board(image)

                if detection_result["detection_success"]:
                    all_corners.append(detection_result["corners"])
                    all_ids.append(detection_result["ids"])
                    valid_count += 1
                    self.logger.debug(f"图像 {i+1}: 检测到 {detection_result['corner_count']} 个角点")
                else:
                    self.logger.debug(f"图像 {i+1}: 未检测到标定板")

            if failed_count > 0:
                self.logger.warning(f"{failed_count}个图像文件加载失败")

            if len(all_corners) < 3:
                self.logger.error(f"有效标定图像不足: {len(all_corners)}/3")
                return {"success": False, "message": "有效标定图像不足（需要至少3张）"}

            self.logger.info(f"有效图像: {valid_count}, 失败: {failed_count}, 检测成功: {len(all_corners)}")

            # 执行标定
            if self.board_config.board_type == BoardType.CHARUCO:
                result = self._calibrate_charuco_camera(all_corners, all_ids, image_size)
            elif self.board_config.board_type == BoardType.CHESSBOARD:
                result = self._calibrate_chessboard_camera(all_corners, image_size)
            else:
                return {"success": False, "message": f"不支持的标定板类型: {self.board_config.board_type}"}

            if result["success"]:
                self.logger.info(f"内参标定成功: {result['message']}")
            else:
                self.logger.error(f"内参标定失败: {result['message']}")

            return result

        except Exception as e:
            self.logger.error(f"从图像列表标定失败: {e}")
            return {"success": False, "message": f"从图像列表标定失败: {str(e)}"}

    def _get_image_files_from_folder(self, image_folder: str) -> List[str]:
        """
        从文件夹获取支持的图像文件列表

        参数:
            image_folder: 图像文件夹路径

        返回:
            图像文件路径列表
        """
        image_files = []
        for ext in self.supported_extensions:
            image_files.extend(
                [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.lower().endswith(f".{ext}")]
            )
        return sorted(image_files)

    def _calibrate_charuco_camera(
        self, all_corners: List, all_ids: List, image_size: Tuple[int, int]
    ) -> Dict[str, Any]:
        """
        ChArUco相机标定

        参数:
            all_corners: 所有图像的角点列表
            all_ids: 所有图像的ID列表
            image_size: 图像尺寸

        返回:
            标定结果
        """
        try:
            # 执行标定
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                all_corners, all_ids, self.detector.charuco_board, image_size, None, None
            )

            if ret:
                return {
                    "success": True,
                    "camera_matrix": camera_matrix,
                    "dist_coeffs": dist_coeffs,
                    "rvecs": rvecs,
                    "tvecs": tvecs,
                    "reprojection_error": ret,
                    "used_images": len(all_corners),
                    "message": f"ChArUco标定成功，重投影误差: {ret:.4f}",
                }
            else:
                return {"success": False, "message": "ChArUco标定失败"}

        except Exception as e:
            self.logger.error(f"ChArUco标定失败: {e}")
            return {"success": False, "message": f"ChArUco标定失败: {str(e)}"}

    def _calibrate_chessboard_camera(self, all_corners: List, image_size: Tuple[int, int]) -> Dict[str, Any]:
        """
        棋盘格相机标定

        参数:
            all_corners: 所有图像的角点列表
            image_size: 图像尺寸

        返回:
            标定结果
        """
        try:
            # 生成世界坐标点
            objp = np.zeros((self.board_config.x_num * self.board_config.y_num, 3), np.float32)
            objp[:, :2] = np.mgrid[0 : self.board_config.x_num, 0 : self.board_config.y_num].T.reshape(-1, 2)
            objp *= self.board_config.square_length

            # 为每个有效图像复制世界坐标点
            object_points = [objp for _ in all_corners]

            # 执行标定
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                object_points, all_corners, image_size, None, None
            )

            return {
                "success": True,
                "camera_matrix": camera_matrix,
                "dist_coeffs": dist_coeffs,
                "rvecs": rvecs,
                "tvecs": tvecs,
                "reprojection_error": ret,
                "used_images": len(all_corners),
                "message": f"棋盘格标定成功，重投影误差: {ret:.4f}",
            }

        except Exception as e:
            self.logger.error(f"棋盘格标定失败: {e}")
            return {"success": False, "message": f"棋盘格标定失败: {str(e)}"}
