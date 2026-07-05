"""
视觉技能客户端参数配置

这个模块提供所有视觉技能函数的默认参数配置，
以字典形式返回，便于客户端调用和参数管理。
"""

from pickle import NONE
from typing import Dict, Any, List, Optional


class VisionConfig:
    """
    视觉技能参数配置类

    提供所有视觉技能函数的默认参数配置，
    以标准化的字典格式返回。
    """

    @staticmethod
    def _rgbd_data_config(
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        rgbd_data_config_key: str = "rgbd_data_config",
    ) -> Dict[str, Any]:
        """
        获取RGBD数据参数配置

        Args:
            rgb_topic_name: RGB图像话题名称
            depth_topic_name: 深度图像话题名称
            camera_info_topic_name: 相机信息话题名称
            rgbd_data_config_key: RGBD数据配置的key，默认是 "rgbd_data_config"。
        Returns:
            Dict: RGBD数据参数配置字典
        """
        rgbd_data_config = {
            "rgb_topic_name": rgb_topic_name,
            "depth_topic_name": depth_topic_name,
            "camera_info_topic_name": camera_info_topic_name,
        }
        config = {rgbd_data_config_key: rgbd_data_config}
        return config

    @staticmethod
    def _vision_detection_config(
        detection_service_name="",
        rgb_topic_name="",
        params={},
        detection_service_key="detection_service_name",
        rgb_topic_key="rgb_topic_name",
        params_key="params",
    ) -> Dict[str, Any]:
        """
        获取检测参数配置字典

        Returns:
            Dict: 检测参数配置字典
        """
        config = {
            rgb_topic_key: rgb_topic_name,
            detection_service_key: detection_service_name,
            params_key: params,
        }
        return config

    @staticmethod
    def _vision_pose_estimation_mask_config(
        pose_estimation_service_name: str,
        rgbd_data_config: Dict[str, Any],
        mask_service_name: str,
        mask_params: Dict[str, Any],
        pose_params: Dict[str, Any],
        pose_estimation_service_key: str = "pose_estimation_service_name",
        rgbd_data_config_key: str = "rgbd_data_config",
        mask_service_key: str = "mask_service_name",
        mask_params_key: str = "mask_params",
        pose_params_key: str = "pose_params",
    ) -> Dict[str, Any]:
        """
        获取位姿估计掩码参数配置

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgbd_data_config: RGBD数据配置
            mask_service_name: 掩码检测服务名称
            mask_params: 掩码检测参数
            pose_params: 位姿估计参数
            pose_estimation_service_key: 位姿估计服务名称的key，默认是 "pose_estimation_service_name"。
            rgbd_data_config_key: RGBD数据配置的key，默认是 "rgbd_data_config"。
            mask_service_key: 掩码检测服务名称的key，默认是 "mask_service_name"。
            mask_params_key: 掩码检测参数的key，默认是 "mask_params"。
            pose_params_key: 位姿估计参数的key，默认是 "pose_params"。
        Returns:
            Dict: 位姿估计掩码参数配置字典
        """
        config = {
            rgbd_data_config_key: rgbd_data_config,
            pose_estimation_service_key: pose_estimation_service_name,
            mask_service_key: mask_service_name,
            mask_params_key: mask_params,
            pose_params_key: pose_params,
        }
        return config
    
    @staticmethod
    def _vision_pose_estimation_box_config(
        pose_estimation_service_name: str,
        rgbd_data_config: Dict[str, Any],
        box_service_name: str,
        box_params: Dict[str, Any],
        pose_params: Dict[str, Any],
        pose_estimation_service_key: str = "pose_estimation_service_name",
        rgbd_data_config_key: str = "rgbd_data_config",
        box_service_key: str = "box_service_name",
        box_params_key: str = "box_params",
        pose_params_key: str = "pose_params",
    ) -> Dict[str, Any]:
        """
        获取位姿估计Box参数配置

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgbd_data_config: RGBD数据配置
            mask_service_name: 掩码检测服务名称
            mask_params: 掩码检测参数
            pose_params: 位姿估计参数
            pose_estimation_service_key: 位姿估计服务名称的key，默认是 "pose_estimation_service_name"。
            rgbd_data_config_key: RGBD数据配置的key，默认是 "rgbd_data_config"。
            box_service_key: Box检测服务名称的key，默认是 "box_service_name"。
            box_params_key: Box检测参数的key，默认是 "box_params"。
            pose_params_key: 位姿估计参数的key，默认是 "pose_params"。
        Returns:
            Dict: 位姿估计Box参数配置字典
        """
        config = {
            rgbd_data_config_key: rgbd_data_config,
            pose_estimation_service_key: pose_estimation_service_name,
            box_service_key: box_service_name,
            box_params_key: box_params,
            pose_params_key: pose_params,
        }
        return config

    @staticmethod
    def _vision_pose_estimation_obb_mask_config(
        pose_estimation_service_name: str,
        rgbd_data_config: Dict[str, Any],
        obb_service_name: str,
        mask_service_name: str,
        merge_data_service_name: str,
        merge_params: Dict[str, Any],
        pose_params: Dict[str, Any],
        obb_params: Dict[str, Any],
        mask_params: Dict[str, Any],
        pose_estimation_service_key: str = "pose_estimation_service_name",
        rgbd_data_config_key: str = "rgbd_data_config",
        obb_service_key: str = "obb_service_name",
        mask_service_key: str = "mask_service_name",
        merge_data_service_key: str = "merge_data_service_name",
        merge_params_key: str = "merge_params",
        pose_params_key: str = "pose_params",
        obb_params_key: str = "obb_params",
        mask_params_key: str = "mask_params",
    ) -> Dict[str, Any]:
        """
        获取基于OBB和掩码的位姿估计参数配置

        Args:
            rgbd_data_config: RGBD数据配置
            obb_service_name: OBB检测服务名称
            mask_service_name: 掩码检测服务名称
            merge_data_service_name: 数据合并服务名称
            merge_params: 数据合并参数
            pose_params: 位姿估计参数
            obb_params: OBB检测参数
            mask_params: 掩码检测参数
            pose_estimation_service_key: 位姿估计服务名称的key，默认是 "pose_estimation_service_name"。
            rgbd_data_config_key: RGBD数据配置的key，默认是 "rgbd_data_config"。
            obb_service_key: OBB检测服务名称的key，默认是 "obb_service_name"。
            mask_service_key: 掩码检测服务名称的key，默认是 "mask_service_name"。
            merge_data_service_key: 数据合并服务名称的key，默认是 "merge_data_service_name"。
            merge_params_key: 数据合并参数的key，默认是 "merge_params"。
            pose_params_key: 位姿估计参数的key，默认是 "pose_params"。
            obb_params_key: OBB检测参数的key，默认是 "obb_params"。
            mask_params_key: 掩码检测参数的key，默认是 "mask_params"。
        Returns:
            Dict: 基于OBB和掩码的位姿估计参数配置字典
        """
        config = {
            pose_estimation_service_key: pose_estimation_service_name,
            rgbd_data_config_key: rgbd_data_config,
            obb_service_key: obb_service_name,
            mask_service_key: mask_service_name,
            merge_data_service_key: merge_data_service_name,
            merge_params_key: merge_params,
            pose_params_key: pose_params,
            obb_params_key: obb_params,
            mask_params_key: mask_params,
        }
        return config

    @staticmethod
    def _vision_template_pose_estimation_box(
        pose_estimation_service_name: str,
        image_topic: str,
        box_service_name: str,
        box_params: Dict[str, Any],
        pose_params: Dict[str, Any],
        pose_estimation_service_key: str = "pose_estimation_service_name",
        image_topic_key: str = "image_topic",
        box_service_key: str = "box_service_name",
        box_params_key: str = "box_params",
        pose_params_key: str = "pose_params",
    ) -> Dict[str, Any]:
        """
        获取基于模板匹配的姿态估计参数配置

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            image_topic: 图像话题名称
            box_service_name: Box检测服务名称
            box_params: Box检测参数
            pose_params: 位姿估计参数
            pose_estimation_service_key: 位姿估计服务名称的key，默认是 "pose_estimation_service_name"。
            image_topic_key: 图像话题名称的key，默认是 "image_topic_name"。
            box_service_key: Box检测服务名称的key，默认是 "box_service_name"。
            box_params_key: Box检测参数的key，默认是 "box_params"。
            pose_params_key: 位姿估计参数的key，默认是 "pose_params"。
        Returns:
            Dict: 基于模板匹配的姿态估计参数配置字典
        """
        config = {
            pose_estimation_service_key: pose_estimation_service_name,
            image_topic_key: image_topic,
            box_service_key: box_service_name,
            box_params_key: box_params,
            pose_params_key: pose_params,
        }
        return config

    @staticmethod
    def get_vision_detection_config(
        image_topic: str,
        service_name: str,
        score=0.8,
        max_num=10,
    ) -> Dict[str, Any]:
        """
        获取检测动作参数配置

        Args:
            image_topic: 图像话题名称
            service_name: 检测服务名称
            score: 检测分数
            max_num: 检测最大数量
        Returns:
            Dict: 检测动作参数配置字典
        """
        data = {}
        params = {
            "min_score": score,
            "max_num": max_num,
        }
        config = VisionConfig._vision_detection_config(
            detection_service_name=service_name,
            rgb_topic_name=image_topic,
            params=params,
        )
        data.update(config)
        print(f"data: {data}")
        return {"action": "vision_detection", "data": data}

    @staticmethod
    def get_vision_pose_estimation_mask_config(
        pose_estimation_service_name: str,
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        mask_service_name: str,
        mask_params: Dict[str, Any] = None,
        pose_params: Dict[str, Any] = None,
        target_frame: str = "",
    ) -> Dict[str, Any]:
        """
        获取位姿估计掩码参数配置

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgb_topic_name: RGB图像话题名称
            depth_topic_name: 深度图像话题名称
            camera_info_topic_name: 相机信息话题名称
            mask_service_name: 掩码检测服务名称
            mask_params: 掩码检测参数
            pose_params: 位姿估计参数
        Returns:
            Dict: 位姿估计掩码动作参数配置字典
        """
        data = {}
        if mask_params == None:
            mask_params = {
                "min_score": 0.8,
                "max_num": 10,
            }
        if pose_params == None:
            pose_params = {
                "target_frame": target_frame,
                "flow": [
                    "transform_to_target_frame",
                    "downsample",
                    "filter_by_seg_cluster",
                    "downsample",
                    "pca_transform_z",
                ],
                "basic_config": {
                    "workspace": [-1.0, -1.0, -0.5, 1.0, 1.0, 0.5],
                    "obj_size_min": [0.005, 0.005, 0.0],
                    "obj_size_max": [0.20, 0.20, 0.20],
                    "enable_workspace_filter": True,
                    "enable_obj_size_filter": True,
                },
                "pointcloud_config": {
                    "leaf_size": 0.002,
                    "cluster_size_min": 100,
                    "cluster_size_max": 100000,
                    "cluster_tolerance": 0.01,
                },
            }

        data.update(
            VisionConfig._vision_pose_estimation_mask_config(
                pose_estimation_service_name=pose_estimation_service_name,
                mask_service_name=mask_service_name,
                mask_params=mask_params,
                pose_params=pose_params,
                rgbd_data_config=None,
            )
        )
        data.update(
            VisionConfig._rgbd_data_config(
                rgb_topic_name=rgb_topic_name,
                depth_topic_name=depth_topic_name,
                camera_info_topic_name=camera_info_topic_name,
            )
        )
        return {"action": "vision_pose_estimation_mask", "data": data}

    @staticmethod
    def get_vision_pose_estimation_obb_mask_config(
        pose_estimation_service_name: str,
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        obb_service_name: str,
        mask_service_name: str,
        merge_data_service_name: str,
        merge_params: Dict[str, Any],
        pose_params: Dict[str, Any],
        obb_params: Dict[str, Any],
        mask_params: Dict[str, Any],
        target_frame: str = "",
    ) -> Dict[str, Any]:
        """
        获取基于掩码的姿态估计参数配置

        Returns:
            Dict: 完整的请求格式 {"action": action, "data": data}
        """
        data = {}
        if mask_params == None:
            mask_params = {
                "min_score": 0.8,
                "max_num": 10,
            }

        if obb_params == None:
            obb_params = {
                "min_score": 0.8,
                "max_num": 10,
            }

        if merge_params == None:
            merge_params = {
                "min_iou": 0.5,
                "angle_correction_config": {
                    "welding_up_side": {
                        "obb_name": "welding_up_side",
                        "relation": "上",
                        "angle_correction_offset": 0.0,
                        "z2up": True,
                        "local_as_base": False,
                    },
                    "welding_down_side": {
                        "obb_name": "welding_down_side",
                        "relation": "上",
                        "angle_correction_offset": -90.0,
                        "z2up": True,
                        "local_as_base": True,
                    },
                },
            }

        if pose_params == None:
            pose_params = {
                "target_frame": target_frame,
                "flow": [
                    "transform_to_target_frame",
                    "downsample",
                    "filter_by_seg_cluster",
                    "set_z_value",
                    "downsample",
                    "icp_transform_rect_rotated",
                ],
                "basic_config": {
                    "workspace": [-1.0, -1.0, -0.5, 1.0, 1.0, 0.5],
                    "obj_size_min": [0.001, 0.001, 0.0],
                    "obj_size_max": [0.5, 0.5, 0.5],
                    "z_values": {
                        "welding_down_side": 0.0,
                        "welding_up_side": 0.0,
                    },
                },
                "pointcloud_config": {
                    "leaf_size": 0.001,
                    "cluster_size_min": 100,
                    "cluster_size_max": 100000,
                    "cluster_tolerance": 0.01,
                    "icp_max_iter": 100,
                    "icp_max_corr_dist": 0.005,
                    "icp_trans_eps": 1e-10,
                    "icp_eucl_eps": 1e-8,
                    "flag_save_cloud": False,
                },
                "init_pose": {
                    "welding_down_side": [0.0, 0.0, 0.0, 0.0, 0.0, 3.14],
                    "welding_up_side": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                },
            }

        data.update(
            VisionConfig._vision_pose_estimation_obb_mask_config(
                pose_estimation_service_name=pose_estimation_service_name,
                rgbd_data_config=None,
                obb_service_name=obb_service_name,
                mask_service_name=mask_service_name,
                merge_data_service_name=merge_data_service_name,
                merge_params=merge_params,
                pose_params=pose_params,
                obb_params=obb_params,
                mask_params=mask_params,
            )
        )

        data.update(
            VisionConfig._rgbd_data_config(
                rgb_topic_name=rgb_topic_name,
                depth_topic_name=depth_topic_name,
                camera_info_topic_name=camera_info_topic_name,
            )
        )
        return {"action": "vision_pose_estimation_obb_mask", "data": data}

    @staticmethod
    def get_vision_pose_estimation_box_config(
        pose_estimation_service_name: str,
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        box_service_name: str,
        box_params: Dict[str, Any] = None,
        pose_params: Dict[str, Any] = None,
        target_frame: str = "",
    ) -> Dict[str, Any]:
        """
        获取位姿估计box参数配置

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgb_topic_name: RGB图像话题名称
            depth_topic_name: 深度图像话题名称
            camera_info_topic_name: 相机信息话题名称
            box_service_name: 掩码检测服务名称
            box_params: 掩码检测参数
            pose_params: 位姿估计参数
        Returns:
            Dict: 位姿估计掩码动作参数配置字典
        """
        data = {}
        if box_params == None:
            box_params = {
                "min_score": 0.8,
                "max_num": 10,
            }
        if pose_params == None:
            pose_params = {
                "target_frame": target_frame,
                "flow": [
                    "transform_to_target_frame",
                    "downsample",
                    "filter_by_seg_cluster",
                    "bounding_box",
                ],
                "basic_config": {
                    "workspace": [-1.0, -1.0, -0.5, 1.0, 1.0, 0.5],
                    "obj_size_min": [0.005, 0.005, 0.0],
                    "obj_size_max": [0.20, 0.20, 0.20],
                    "enable_workspace_filter": True,
                    "enable_obj_size_filter": True,
                },
                "pointcloud_config": {
                    "leaf_size": 0.002,
                    "cluster_size_min": 100,
                    "cluster_size_max": 100000,
                    "cluster_tolerance": 0.01,
                },
            }

        data.update(
            VisionConfig._vision_pose_estimation_box_config(
                pose_estimation_service_name=pose_estimation_service_name,
                box_service_name=box_service_name,
                box_params=box_params,
                pose_params=pose_params,
                rgbd_data_config=None,
            )
        )
        data.update(
            VisionConfig._rgbd_data_config(
                rgb_topic_name=rgb_topic_name,
                depth_topic_name=depth_topic_name,
                camera_info_topic_name=camera_info_topic_name,
            )
        )
        return {"action": "vision_pose_estimation_box", "data": data}

    @staticmethod
    def get_vision_template_pose_estimation_box_config(
        pose_estimation_service_name: str,
        image_topic_name: str,
        box_service_name: str,
        box_params: Dict[str, Any] = None,
        pose_params: Dict[str, Any] = None,
        target_frame: str = "",
        intrinsic_file_path: str = None,
        pose_board_in_camera: List[float] = None,
    ) -> Dict[str, Any]:
        """
        获取基于模板匹配的姿态估计参数配置

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            image_topic_name: 图像话题名称
            box_service_name: Box检测服务名称
            box_params: Box检测参数
            pose_params: 位姿估计参数
            target_frame: 目标坐标系
            intrinsic_file_path: 相机内参文件路径
            pose_board_in_camera: 板子在相机坐标系下的位姿 [x, y, z, qx, qy, qz, qw]
        Returns:
            Dict: 基于模板匹配的姿态估计参数配置字典
        """
        data = {}
        if box_params == None:
            box_params = {
                "min_score": 0.8,
                "max_num": 10,
            }
        if pose_params == None:
            pose_params = {
                "target_frame": target_frame,
                "basic_config": {
                    "workspace": [-1.0, -1.0, -0.5, 1.0, 1.0, 0.5],
                },
                "template_config": {
                    "canny_thresh1": 50,
                    "canny_thresh2": 150,
                    "edge_filter_ratio": 100.0,
                    "angle_range": [-180, 180],
                    "angle_step": 1,
                    "use_coarse_fine": True,
                    "coarse_scale": 4,
                    "coarse_angle_step": 3,
                    "coarse_position_step": 1,
                    "refine_range": 20,
                    "refine_angle_range": [-10.0, 10.0],
                    "refine_angle_step": 1.0,
                    "flag_save_debug_images": False,
                    "save_debug_path": "/home/yw/workspace/data/papjia_vision/images",
                },
            }
        if intrinsic_file_path is not None:
            pose_params["template_config"]["intrinsic_file_path"] = intrinsic_file_path
        if pose_board_in_camera is not None:
            pose_params["template_config"]["pose_board_in_camera"] = pose_board_in_camera
        data.update(
            VisionConfig._vision_template_pose_estimation_box(
                pose_estimation_service_name=pose_estimation_service_name,
                image_topic=image_topic_name,
                box_service_name=box_service_name,
                box_params=box_params,
                pose_params=pose_params,
            )
        )
        return {"action": "vision_template_pose_estimation_box", "data": data}
