"""
@Descripttion: 摄像头内参标定模块
@version: 1.0
@Author: 崔译文
@Date: 2024-03-15 15:58:02
@LastEditors: 崔译文
@LastEditTime: 2024-04-02 09:18:54
"""

import os
import cv2
import yaml
import numpy as np


class CameraCalibration:
    def __init__(self, board_config_file, image_folder, output_dir):
        cfg = self.load_config(board_config_file)
        self.board_type = cfg["board_type"]
        self.x_num = cfg["x_num"]
        self.y_num = cfg["y_num"]
        print(f"board_type: {self.board_type}", flush=True)
        print(f"x_num: {self.x_num}", flush=True)
        print(f"y_num: {self.y_num}", flush=True)
        if self.board_type not in ["ChArUco", "Chessboard"]:
            raise ValueError(f"Invalid board type: {self.board_type}")
        if self.x_num <= 0 or self.y_num <= 0:
            raise ValueError(f"Invalid x_num or y_num: {self.x_num}, {self.y_num}")
        if self.board_type == "ChArUco":
            self.dictionary = self.get_dictionary(cfg["dict_type"])
            self.square_length = cfg["square_length"]
            self.marker_length = cfg["marker_length"]
            print(f"dictionary: {type(self.dictionary)}", flush=True)
            print(f"square_length: {self.square_length}", flush=True)
            print(f"marker_length: {self.marker_length}", flush=True)
        elif self.board_type == "Chessboard":
            self.square_length = cfg["square_length"]
        self.image_folder = image_folder
        if output_dir == "":
            self.result_folder = os.path.join(image_folder, "calibration_results")
        else:
            self.result_folder = os.path.join(output_dir, "calibration_results")
        os.makedirs(self.result_folder, exist_ok=True)
        self.exts = ["jpg", "jpeg", "png"]
        print(f"image_folder: {self.image_folder}", flush=True)
        print(f"result_folder: {self.result_folder}", flush=True)

    def update_img_folder(self, folder_path):
        self.image_folder = folder_path

    def display_image_with_user_control(self, img, window_title, show):
        """
        显示图像并等待用户操作控制

        Args:
            img: 要显示的图像
            window_title: 窗口标题
            show: 是否显示图像

        Returns:
            bool: True表示继续，False表示退出
        """
        if not show:
            return True

        # 创建用于显示的图像副本
        display_img = img.copy()

        # 如果图像宽度大于1080，进行resize
        if display_img.shape[1] > 1080:
            scale_factor = 1080.0 / display_img.shape[1]
            new_width = int(display_img.shape[1] * scale_factor)
            new_height = int(display_img.shape[0] * scale_factor)
            display_img = cv2.resize(display_img, (new_width, new_height))
            print(f"图像已resize到显示尺寸: {new_width} x {new_height}", flush=True)

        # 显示图像并等待用户操作
        cv2.imshow(f"{window_title}", display_img)

        # 等待用户按键
        while True:
            key = cv2.waitKey(1000) & 0xFF
            if key == ord("n") or key == ord("N"):  # n键继续下一张
                print("用户选择继续下一张图像", flush=True)
                cv2.destroyAllWindows()
                return True
            elif key == 27 or key == ord("q") or key == ord("Q"):  # ESC键或q键退出
                print("用户选择退出标定", flush=True)
                cv2.destroyAllWindows()
                return False

        return True

    def load_config(self, config_file):
        with open(config_file, "r") as f:
            config = yaml.safe_load(f)
        return config

    def get_files_in_folder(self, folder_path):
        """
        返回指定文件夹下所有文件的完整路径列表
        """
        files = []
        names = []
        # 检查文件夹是否存在
        if os.path.exists(folder_path) and os.path.isdir(folder_path):
            # 获取文件夹中的所有文件和文件夹的名称
            items = os.listdir(folder_path)
            for item in items:
                # 使用os.path.join()将文件夹路径与文件名连接起来
                file_path = os.path.join(folder_path, item)
                # 检查是否是文件，如果是则添加到列表中
                if os.path.isfile(file_path) and item.split(".")[-1].lower() in self.exts:
                    files.append(file_path)
                    names.append(item)
        else:
            print(f"文件夹 '{folder_path}' 不存在或不可访问。")
        return files, names

    def get_dictionary(self, dict_type):
        dictionary = None
        try:
            if dict_type == "DICT_4X4_50":
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
                print(f"✓ 创建字典类型: {dict_type}", flush=True)
            elif dict_type == "DICT_5X5_50":
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
                print(f"✓ 创建字典类型: {dict_type}", flush=True)
            elif dict_type == "DICT_5X5_100":
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
                print(f"✓ 创建字典类型: {dict_type}", flush=True)
            elif dict_type == "DICT_6X6_50":
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
                print(f"✓ 创建字典类型: {dict_type}", flush=True)
            elif dict_type == "DICT_7X7_50":
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)
                print(f"✓ 创建字典类型: {dict_type}", flush=True)
            else:
                print(f"⚠ 不支持的字典类型: {dict_type}，使用默认DICT_4X4_50", flush=True)
                dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        except Exception as e:
            print(f"❌ 创建字典失败: {e}，使用默认DICT_4X4_50", flush=True)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        if dictionary is None:
            print("❌ 警告：字典创建失败，使用默认字典", flush=True)
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        return dictionary

    def calibrate(self, show=False):
        if self.board_type == "ChArUco":
            self.cali_with_charuco_board(show)
        elif self.board_type == "Chessboard":
            self.cali_with_chessboard(show)

    def save_calibration_results(self, camera_matrix, dist_coef, image_width, image_height, rvecs=None, tvecs=None):
        """
        保存相机标定结果
        """
        # 创建投影矩阵
        projection_matrix = np.zeros((3, 4))
        projection_matrix[:3, :3] = camera_matrix

        # 准备保存的数据
        data = {
            "image_width": image_width,
            "image_height": image_height,
            "camera_name": "usb_camera",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": camera_matrix.flatten().tolist(),
            },
            "camera_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": 5,
                "data": dist_coef.flatten().tolist(),
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": projection_matrix.flatten().tolist(),
            },
        }

        # 保存标定结果到YAML文件
        file_path = os.path.join(self.result_folder, "calibration.yaml")
        with open(file_path, "w") as out_file:
            yaml.dump(data, out_file, default_flow_style=False, sort_keys=False)

        print(f"标定文件已保存到: {file_path}")

        return file_path

    def cali_with_charuco_board(self, show=False):
        """
        使用ChArUco标定板进行相机标定
        Args:
            show: 是否显示标定过程
        Returns:
            camera_matrix: 相机内参矩阵
            dist_coef: 畸变系数
        """
        corners_all = []
        ids_all = []

        # 检查字典是否有效
        if self.dictionary is None:
            print("错误：字典为空，无法创建CharucoBoard", flush=True)
            return None, None

        print(f"使用字典类型: {type(self.dictionary)}", flush=True)
        print(f"棋盘尺寸: {self.x_num} x {self.y_num}", flush=True)
        print(f"方块长度: {self.square_length}", flush=True)
        print(f"标记长度: {self.marker_length}", flush=True)

        # 针对OpenCV 4.6.0的兼容性修复
        charuco_board = None
        try:
            # 尝试使用新的API (OpenCV 4.6.0+)
            charuco_board = cv2.aruco.CharucoBoard()
            charuco_board.setDictionary(self.dictionary)
            charuco_board.setChessboardSize((self.x_num, self.y_num))
            charuco_board.setMarkerLength(self.marker_length)
            charuco_board.setSquareLength(self.square_length)
            print("使用新版本OpenCV CharucoBoard API", flush=True)
        except AttributeError:
            try:
                # 回退到旧版本API
                charuco_board = cv2.aruco.CharucoBoard_create(
                    self.x_num, self.y_num, self.square_length, self.marker_length, self.dictionary
                )
                print("使用旧版本OpenCV CharucoBoard API", flush=True)
            except Exception as e:
                print(f"创建CharucoBoard失败: {e}", flush=True)
                return None, None

        if charuco_board is None:
            print("错误：CharucoBoard创建失败", flush=True)
            return None, None

        files, file_names = self.get_files_in_folder(self.image_folder)
        if not files:
            print("没有找到图像文件", flush=True)
            return None, None

        for file_path, file_name in zip(files, file_names):
            print(f"\n正在处理文件: {file_path}", flush=True)
            # 读取图像
            img = cv2.imread(file_path)
            if img is None:
                print(f"无法读取图像: {file_path}", flush=True)
                continue

            # 转换为灰度图
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # 检测aruco标记 - OpenCV 4.6.0兼容性
            corners = []
            ids = []
            try:
                # 尝试新版本API
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
                print(f"使用新版本detectMarkers API", flush=True)
            except TypeError:
                try:
                    # 回退到旧版本API
                    corners, ids, _ = cv2.aruco.detectMarkers(image=gray, dictionary=self.dictionary)
                    print(f"使用旧版本detectMarkers API", flush=True)
                except Exception as e:
                    print(f"detectMarkers检测失败: {e}", flush=True)
                    continue

            # 处理返回值可能为None的情况
            if ids is None:
                ids = []
            if corners is None:
                corners = []

            print(f"检测到的角点数量: {len(corners)}", flush=True)
            print(f"检测到的ID数量: {len(ids)}", flush=True)

            # 绘制检测到的aruco标记
            if len(corners) > 0:
                try:
                    # 尝试新版本API
                    img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
                except TypeError:
                    try:
                        # 回退到旧版本API
                        img = cv2.aruco.drawDetectedMarkers(image=img, corners=corners)
                    except Exception as e:
                        print(f"绘制检测标记失败: {e}", flush=True)

            # 从检测到的aruco标记中获取charuco角点和ID
            if len(corners) > 0 and len(ids) > 0:
                try:
                    # 尝试新版本API
                    response, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        corners, ids, gray, charuco_board
                    )
                    print(f"使用新版本interpolateCornersCharuco API", flush=True)
                except TypeError:
                    try:
                        # 回退到旧版本API
                        response, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                            markerCorners=corners, markerIds=ids, image=gray, board=charuco_board
                        )
                        print(f"使用旧版本interpolateCornersCharuco API", flush=True)
                    except Exception as e:
                        print(f"插值Charuco角点失败: {e}", flush=True)
                        continue

                # 检查返回值
                if response is None or charuco_corners is None or charuco_ids is None:
                    print("插值Charuco角点返回空值，跳过此图像", flush=True)
                    continue

                print(
                    f"Charuco响应: {response}, 角点数量: {len(charuco_corners) if charuco_corners is not None else 0}",
                    flush=True,
                )

                # 如果找到Charuco板，收集图像/角点数据
                # 要求至少20个方块
                if response > 20:
                    # 检查数据有效性
                    if charuco_corners is not None and charuco_ids is not None:
                        # 将这些角点和ID添加到标定数组中
                        corners_all.append(charuco_corners)
                        ids_all.append(charuco_ids)
                        print(f"添加有效数据，当前总数: {len(corners_all)}", flush=True)
                    else:
                        print("Charuco数据无效，跳过", flush=True)

                    # 绘制检测到的Charuco板，显示给标定者看
                    try:
                        # 尝试新版本API
                        img = cv2.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
                    except TypeError:
                        try:
                            # 回退到旧版本API
                            img = cv2.aruco.drawDetectedCornersCharuco(
                                image=img, charucoCorners=charuco_corners, charucoIds=charuco_ids
                            )
                        except Exception as e:
                            print(f"绘制Charuco角点失败: {e}", flush=True)

                    cv2.imwrite(os.path.join(self.result_folder, file_name), img)

                    # 显示图像并等待用户操作
                    if not self.display_image_with_user_control(img, "Charuco Calibration", show):
                        return None, None

        if len(corners_all) == 0:
            print("没有检测到有效的Charuco板，无法进行标定", flush=True)
            return None, None

        w, h = img.shape[1], img.shape[0]

        # 相机标定
        try:
            ret, camera_matrix, dist_coef, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
                corners_all, ids_all, charuco_board, (w, h), None, None
            )
        except Exception as e:
            print(f"Charuco相机标定失败: {e}", flush=True)
            return None, None

        if ret:
            print(f"Charuco标定成功! 重投影误差: {ret}", flush=True)

            # 保存标定结果
            self.save_calibration_results(camera_matrix, dist_coef, w, h, rvecs, tvecs)

            return camera_matrix, dist_coef
        else:
            print("Charuco标定失败", flush=True)
            return None, None

    def cali_with_chessboard(self, show=False):
        """
        使用棋盘格进行相机标定
        """
        # 准备棋盘格角点
        objp = np.zeros((self.x_num * self.y_num, 3), np.float32)
        objp[:, :2] = np.mgrid[0 : self.x_num, 0 : self.y_num].T.reshape(-1, 2)
        objp = objp * self.square_length  # 使用square_length作为棋盘格一个方块的大小

        # 存储所有图像的角点
        objpoints = []  # 3D点
        imgpoints = []  # 2D点

        files, file_names = self.get_files_in_folder(self.image_folder)
        if not files:
            print(f"在文件夹 {self.image_folder} 中没有找到图像文件")
            return

        for file_path, file_name in zip(files, file_names):
            img = cv2.imread(file_path)
            if img is None:
                print(f"无法读取图像: {file_path}")
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # 查找棋盘格角点
            ret, corners = cv2.findChessboardCorners(gray, (self.x_num, self.y_num), None)

            if ret:
                # 亚像素角点检测
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # 添加角点
                objpoints.append(objp)
                imgpoints.append(corners2)

                # 绘制角点
                cv2.drawChessboardCorners(img, (self.x_num, self.y_num), corners2, ret)
                cv2.imwrite(os.path.join(self.result_folder, file_name), img)

                if show:
                    # 显示图像并等待用户操作
                    if not self.display_image_with_user_control(img, "棋盘格角点", show):
                        return None, None
                    print(f"成功检测到棋盘格角点: {file_path}")
            else:
                print(f"未能在图像中找到棋盘格角点: {file_path}")

        if len(objpoints) == 0:
            print("没有找到有效的棋盘格图像，无法进行标定")
            return

        # 获取图像尺寸
        h, w = gray.shape[:2]

        # 相机标定
        ret, camera_matrix, dist_coef, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)

        if ret:
            print(f"棋盘格标定成功! 重投影误差: {ret}")

            # 计算重投影误差
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coef)
                error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
                mean_error += error
            print(f"平均重投影误差: {mean_error / len(objpoints)}")

            # 保存标定结果
            self.save_calibration_results(camera_matrix, dist_coef, w, h, rvecs, tvecs)

            return camera_matrix, dist_coef
        else:
            print("棋盘格标定失败")
            return None, None
