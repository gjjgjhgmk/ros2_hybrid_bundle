#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定板检测和位姿估计模块

提供ChArUco和棋盘格标定板的检测功能，以及基于内参的位姿估计
"""

import cv2
import numpy as np
import logging
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
import scipy.spatial.transform


# 常量定义
class BoardType:
    CHARUCO = "ChArUco"
    CHESSBOARD = "Chessboard"


class DictType:
    DICT_4X4_50 = "DICT_4X4_50"
    DICT_5X5_50 = "DICT_5X5_50"
    DICT_5X5_100 = "DICT_5X5_100"
    DICT_6X6_50 = "DICT_6X6_50"
    DICT_6X6_250 = "DICT_6X6_250"
    DICT_7X7_50 = "DICT_7X7_50"


class HandEyeMethod:
    TSAI = "Tsai"
    PARK = "Park"
    HORAUD = "Horaud"
    ANDREFF = "Andreff"
    DANIILIDIS = "Daniilidis"


# ArUco字典映射
ARUCO_DICT_MAP = {
    DictType.DICT_4X4_50: cv2.aruco.DICT_4X4_50,
    DictType.DICT_5X5_50: cv2.aruco.DICT_5X5_50,
    DictType.DICT_5X5_100: cv2.aruco.DICT_5X5_100,
    DictType.DICT_6X6_50: cv2.aruco.DICT_6X6_50,
    DictType.DICT_6X6_250: cv2.aruco.DICT_6X6_250,
    DictType.DICT_7X7_50: cv2.aruco.DICT_7X7_50,
}

# 手眼标定方法映射
HAND_EYE_METHOD_MAP = {
    HandEyeMethod.TSAI: cv2.CALIB_HAND_EYE_TSAI,
    HandEyeMethod.PARK: cv2.CALIB_HAND_EYE_PARK,
    HandEyeMethod.HORAUD: cv2.CALIB_HAND_EYE_HORAUD,
    HandEyeMethod.ANDREFF: cv2.CALIB_HAND_EYE_ANDREFF,
    HandEyeMethod.DANIILIDIS: cv2.CALIB_HAND_EYE_DANIILIDIS,
}


@dataclass
class BoardConfig:
    """标定板配置"""

    board_type: str = BoardType.CHARUCO  # "ChArUco" or "Chessboard"
    x_num: int = 9
    y_num: int = 6
    square_length: float = 0.025
    marker_length: float = 0.018
    dict_type: str = DictType.DICT_6X6_250


class BoardDetector:
    """标定板检测器"""

    def __init__(self, board_config: BoardConfig):
        self.board_config = board_config
        self.logger = logging.getLogger(__name__)

        # 初始化标定板
        if board_config.board_type == BoardType.CHARUCO:
            self._init_charuco_board()
        elif board_config.board_type == BoardType.CHESSBOARD:
            self._init_chessboard()
        else:
            raise ValueError(f"不支持的标定板类型: {board_config.board_type}")

    def _get_dictionary(self, dict_type: str) -> Any:
        """
        获取ArUco字典

        参数:
            dict_type: 字典类型

        返回:
            ArUco字典对象
        """
        try:
            if dict_type in ARUCO_DICT_MAP:
                aruco_dict_id = ARUCO_DICT_MAP[dict_type]
                dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
                self.logger.info(f"创建字典类型: {dict_type}")
                return dictionary
            else:
                self.logger.warning(f"不支持的字典类型: {dict_type}，使用默认{DictType.DICT_4X4_50}")
                dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_MAP[DictType.DICT_4X4_50])
                return dictionary
        except Exception as e:
            self.logger.error(f"创建字典失败: {e}，使用默认{DictType.DICT_4X4_50}")
            return cv2.aruco.getPredefinedDictionary(ARUCO_DICT_MAP[DictType.DICT_4X4_50])

    def _init_charuco_board(self) -> None:
        """初始化ChArUco标定板"""
        self.aruco_dict = self._get_dictionary(self.board_config.dict_type)
        self.charuco_board = self._create_charuco_board_with_compatibility()

    def _create_charuco_board_with_compatibility(self) -> Any:
        """
        兼容不同版本OpenCV的CharucoBoard创建

        返回:
            CharucoBoard对象
        """
        try:
            # 尝试使用新的API (OpenCV 4.6.0+)
            charuco_board = cv2.aruco.CharucoBoard()
            charuco_board.setDictionary(self.aruco_dict)
            charuco_board.setChessboardSize((self.board_config.x_num, self.board_config.y_num))
            charuco_board.setMarkerLength(self.board_config.marker_length)
            charuco_board.setSquareLength(self.board_config.square_length)
            self.logger.info("使用新版本OpenCV CharucoBoard API")
            return charuco_board
        except AttributeError:
            try:
                # 回退到旧版本API
                charuco_board = cv2.aruco.CharucoBoard_create(
                    self.board_config.x_num,
                    self.board_config.y_num,
                    self.board_config.square_length,
                    self.board_config.marker_length,
                    self.aruco_dict,
                )
                self.logger.info("使用旧版本OpenCV CharucoBoard API")
                return charuco_board
            except Exception as e:
                self.logger.error(f"创建CharucoBoard失败: {e}")
                raise

    def _init_chessboard(self) -> None:
        """初始化棋盘格"""
        self.chessboard_size = (self.board_config.x_num, self.board_config.y_num)
        self.square_size = self.board_config.square_length
        self.logger.info(f"初始化棋盘格: 尺寸{self.chessboard_size}, 方格大小{self.square_size}")

    def _detect_markers_with_compatibility(self, gray: np.ndarray) -> Tuple[List, np.ndarray, Any]:
        """
        兼容不同版本OpenCV的ArUco标记检测

        参数:
            gray: 灰度图像

        返回:
            (corners, ids, rejected) 元组
        """
        try:
            # 尝试新版本API
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict)
            self.logger.debug("使用新版本detectMarkers API")
            return corners, ids, rejected
        except TypeError:
            try:
                # 回退到旧版本API
                corners, ids, rejected = cv2.aruco.detectMarkers(image=gray, dictionary=self.aruco_dict)
                self.logger.debug("使用旧版本detectMarkers API")
                return corners, ids, rejected
            except Exception as e:
                self.logger.error(f"detectMarkers检测失败: {e}")
                return [], np.array([]), None

    def _draw_detected_markers_with_compatibility(
        self, image: np.ndarray, corners: List, ids: np.ndarray
    ) -> np.ndarray:
        """
        兼容不同版本OpenCV的标记绘制

        参数:
            image: 输入图像
            corners: 检测到的角点
            ids: 标记ID

        返回:
            绘制了标记的图像
        """
        try:
            # 尝试新版本API
            result_image = cv2.aruco.drawDetectedMarkers(image, corners, ids)
            self.logger.debug("使用新版本drawDetectedMarkers API")
            return result_image
        except TypeError:
            try:
                # 回退到旧版本API
                result_image = cv2.aruco.drawDetectedMarkers(image=image, corners=corners)
                self.logger.debug("使用旧版本drawDetectedMarkers API")
                return result_image
            except Exception as e:
                self.logger.error(f"绘制检测标记失败: {e}")
                return image

    def _interpolate_charuco_corners_with_compatibility(
        self, corners: List, ids: np.ndarray, gray: np.ndarray
    ) -> Tuple[int, Optional[np.ndarray], Optional[np.ndarray]]:
        """
        兼容不同版本OpenCV的ChArUco角点插值

        参数:
            corners: ArUco角点
            ids: ArUco标记ID
            gray: 灰度图像

        返回:
            (检测数量, charuco_corners, charuco_ids) 元组
        """
        try:
            # 尝试新版本API
            ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.charuco_board
            )
            self.logger.debug("使用新版本interpolateCornersCharuco API")
            return ret, charuco_corners, charuco_ids
        except TypeError:
            try:
                # 回退到旧版本API
                ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    markerCorners=corners, markerIds=ids, image=gray, board=self.charuco_board
                )
                self.logger.debug("使用旧版本interpolateCornersCharuco API")
                return ret, charuco_corners, charuco_ids
            except Exception as e:
                self.logger.error(f"插值ChArUco角点失败: {e}")
                return 0, None, None

    def _draw_charuco_corners_with_compatibility(
        self, image: np.ndarray, charuco_corners: np.ndarray, charuco_ids: np.ndarray
    ) -> np.ndarray:
        """
        兼容不同版本OpenCV的ChArUco角点绘制

        参数:
            image: 输入图像
            charuco_corners: ChArUco角点
            charuco_ids: ChArUco角点ID

        返回:
            绘制了角点的图像
        """
        try:
            # 尝试新版本API
            result_image = cv2.aruco.drawDetectedCornersCharuco(image, charuco_corners, charuco_ids)
            self.logger.debug("使用新版本drawDetectedCornersCharuco API")
            return result_image
        except TypeError:
            try:
                # 回退到旧版本API
                result_image = cv2.aruco.drawDetectedCornersCharuco(
                    image=image, charucoCorners=charuco_corners, charucoIds=charuco_ids
                )
                self.logger.debug("使用旧版本drawDetectedCornersCharuco API")
                return result_image
            except Exception as e:
                self.logger.error(f"绘制ChArUco角点失败: {e}")
                return image

    def detect_board(self, image: np.ndarray) -> Dict[str, Any]:
        """
        检测标定板

        参数:
            image: 输入图像

        返回:
            检测结果字典
        """
        if self.board_config.board_type == BoardType.CHARUCO:
            return self._detect_charuco(image)
        elif self.board_config.board_type == BoardType.CHESSBOARD:
            return self._detect_chessboard(image)
        else:
            return self._create_empty_result(image)

    def _detect_charuco(self, image: np.ndarray) -> Dict[str, Any]:
        """检测ChArUco标定板"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            self.logger.debug(f"灰度图像尺寸: {gray.shape}")
            detection_image = image.copy()

            # 检测ArUco标记
            corners, ids, _ = self._detect_markers_with_compatibility(gray)
            self.logger.debug(f"检测到{len(corners)}个标记, IDs数量: {len(ids) if ids is not None else 0}")

            result = self._create_empty_result(detection_image)

            if len(corners) > 0 and ids is not None and len(ids) > 0:
                # 绘制检测到的标记
                detection_image = self._draw_detected_markers_with_compatibility(detection_image, corners, ids)

                # 插值ChArUco角点
                ret, charuco_corners, charuco_ids = self._interpolate_charuco_corners_with_compatibility(
                    corners, ids, gray
                )

                self.logger.debug(
                    f"插值结果: {ret}个角点, "
                    f"charuco_corners: {len(charuco_corners) if charuco_corners is not None else 0}, "
                    f"charuco_ids: {len(charuco_ids) if charuco_ids is not None else 0}"
                )
                if ret > 0 and charuco_corners is not None and charuco_ids is not None:
                    # 绘制ChArUco角点
                    detection_image = self._draw_charuco_corners_with_compatibility(
                        detection_image, charuco_corners, charuco_ids
                    )

                    result.update(
                        {
                            "detection_image": detection_image,
                            "detection_success": True,
                            "corners": charuco_corners,
                            "ids": charuco_ids,
                            "corners_num": len(charuco_corners),
                            "corner_count": ret,
                            "aruco_corners": corners,
                            "aruco_ids": ids,
                        }
                    )
                    self.logger.info(f"ChArUco检测成功: {ret}个角点")
                else:
                    result["detection_image"] = detection_image
                    self.logger.debug("未检测到足够的ChArUco角点")
            else:
                result["detection_image"] = detection_image
                self.logger.debug("未检测到ArUco标记")

            return result

        except Exception as e:
            self.logger.error(f"ChArUco检测失败: {e}")
            return self._create_empty_result(image)

    def _detect_chessboard(self, image: np.ndarray) -> Dict[str, Any]:
        """检测棋盘格"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
            detection_image = image.copy()

            # 检测棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

            result = self._create_empty_result(detection_image)

            if ret:
                # 精细化角点
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # 绘制角点
                cv2.drawChessboardCorners(detection_image, self.chessboard_size, corners, ret)

                # 生成角点ID（棋盘格按顺序编号）
                ids = np.arange(len(corners)).reshape(-1, 1)

                result.update(
                    {
                        "detection_image": detection_image,
                        "detection_success": True,
                        "corners": corners,
                        "ids": ids,
                        "corner_count": len(corners),
                    }
                )
                self.logger.info(f"棋盘格检测成功: {len(corners)}个角点")
            else:
                result["detection_image"] = detection_image
                self.logger.debug("未检测到棋盘格")

            return result

        except Exception as e:
            self.logger.error(f"棋盘格检测失败: {e}")
            return self._create_empty_result(image)

    def _create_empty_result(self, image: np.ndarray) -> Dict[str, Any]:
        """创建空检测结果"""
        return {
            "detection_image": image.copy(),
            "detection_success": False,
            "corners": None,
            "ids": None,
            "corner_count": 0,
        }

    def estimate_pose(self, image: np.ndarray, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Dict[str, Any]:
        """
        估计标定板位姿

        参数:
            image: 输入图像
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数

        返回:
            位姿估计结果
        """
        # 先检测标定板
        detection_result = self.detect_board(image)

        pose_result = {
            "pose_success": False,
            "rotation_vector": None,
            "translation_vector": None,
            "rotation_matrix": None,
            "quaternion": None,
            **detection_result,
        }

        if not detection_result["detection_success"]:
            return pose_result

        try:
            if self.board_config.board_type == BoardType.CHARUCO:
                pose_result.update(self._estimate_charuco_pose(detection_result, camera_matrix, dist_coeffs))
            elif self.board_config.board_type == BoardType.CHESSBOARD:
                pose_result.update(self._estimate_chessboard_pose(detection_result, camera_matrix, dist_coeffs))

        except Exception as e:
            self.logger.error(f"位姿估计失败: {e}")

        return pose_result

    def _estimate_charuco_pose(
        self, detection_result: Dict[str, Any], camera_matrix: np.ndarray, dist_coeffs: np.ndarray
    ) -> Dict[str, Any]:
        """估计ChArUco标定板位姿"""
        try:
            corners = detection_result["corners"]
            ids = detection_result["ids"]

            if corners is None or len(corners) < 4:
                self.logger.warning("角点数量不足，无法估计位姿")
                return {"pose_success": False}

            # 估计位姿
            valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                corners, ids, self.charuco_board, camera_matrix, dist_coeffs, None, None
            )

            if valid:
                # 转换为旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # 在检测图像上绘制坐标轴
                detection_image = detection_result["detection_image"].copy()
                detection_image = cv2.drawFrameAxes(
                    detection_image, camera_matrix, dist_coeffs, rvec, tvec, self.board_config.square_length
                )

                self.logger.info("成功估计ChArUco标定板位姿")
                return {
                    "pose_success": True,
                    "rotation_vector": rvec.flatten(),
                    "translation_vector": tvec.flatten(),
                    "rotation_matrix": rotation_matrix,
                    "quaternion": scipy.spatial.transform.Rotation.from_rotvec(rvec.flatten()).as_quat(),
                    "detection_image": detection_image,
                }

            self.logger.warning("ChArUco位姿估计失败")
            return {"pose_success": False}

        except Exception as e:
            self.logger.error(f"ChArUco位姿估计失败: {e}")
            return {"pose_success": False}

    def _estimate_chessboard_pose(
        self, detection_result: Dict[str, Any], camera_matrix: np.ndarray, dist_coeffs: np.ndarray
    ) -> Dict[str, Any]:
        """估计棋盘格位姿"""
        try:
            corners = detection_result["corners"]

            if corners is None or len(corners) < 4:
                return {"pose_success": False}

            # 生成棋盘格世界坐标点
            objp = np.zeros((self.chessboard_size[0] * self.chessboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0 : self.chessboard_size[0], 0 : self.chessboard_size[1]].T.reshape(-1, 2)
            objp *= self.square_size

            # 求解PnP问题
            success, rvec, tvec = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs)

            if success:
                # 转换为旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # 在检测图像上绘制坐标轴
                detection_image = detection_result["detection_image"].copy()
                detection_image = cv2.drawFrameAxes(
                    detection_image, camera_matrix, dist_coeffs, rvec, tvec, self.square_size
                )

                return {
                    "pose_success": True,
                    "rotation_vector": rvec.flatten(),
                    "translation_vector": tvec.flatten(),
                    "rotation_matrix": rotation_matrix,
                    "quaternion": scipy.spatial.transform.Rotation.from_rotvec(rvec.flatten()).as_quat(),
                    "detection_image": detection_image,
                }

            return {"pose_success": False}

        except Exception as e:
            self.logger.error(f"棋盘格位姿估计失败: {e}")
            return {"pose_success": False}
