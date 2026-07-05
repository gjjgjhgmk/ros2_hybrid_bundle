"""
ZeroMQ 客户端样例 for PapJia Skill Manager

这个模块提供ZeroMQ客户端接口用于调用视觉技能，
通过消息队列与技能管理器服务器通信。
"""

import zmq
import logging
import argparse
import json
from typing import Dict, Any, List, Optional
from .vision_config import VisionConfig


class ZMQVisionClient:
    """
    ZeroMQ视觉技能客户端

    提供与PapJia Skill Manager服务器通信的接口，
    支持各种视觉技能的调用。
    """

    def __init__(
        self,
        host: str,
        port: int,
        timeout: int,
    ):
        """
        初始化ZeroMQ客户端

        Args:
            host: 服务器地址
            port: 服务器端口
            timeout: 请求超时时间（秒）
        """
        self.host = host
        self.port = port
        self.timeout = timeout * 1000  # 转换为毫秒

        # 设置日志
        self.logger = logging.getLogger(__name__)

        # 初始化ZeroMQ上下文和socket
        self.context = None
        self.socket = None
        self._connect()

    def _connect(self):
        """连接到ZeroMQ服务器"""
        try:
            self.context = zmq.Context()
            self.socket = self.context.socket(zmq.REQ)

            # 设置超时
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout)
            self.socket.setsockopt(zmq.SNDTIMEO, 5000)  # 5秒发送超时

            # 连接到服务器
            server_address = f"tcp://{self.host}:{self.port}"
            self.socket.connect(server_address)
            self.logger.info(f"已连接到ZeroMQ服务器: {server_address}")

        except Exception as e:
            self.logger.error(f"连接ZeroMQ服务器失败: {e}")
            raise

    def _send_request(
        self,
        request: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        发送请求到服务器

        Args:
            request: 完整的请求字典，包含 "action" 和 "data" 字段

        Returns:
            Dict: 服务器响应
        """
        try:
            # 构造请求消息
            self.logger.info(f"发送请求: {request}")
            # 发送请求
            self.socket.send_json(request)

            # 接收响应
            response = self.socket.recv_json()
            self.logger.info(f"接收响应: {response}")

            return response

        except zmq.Again:
            self.logger.error(f"请求超时 ({self.timeout/1000}秒)")
            return {"success": False, "message": "请求超时"}
        except Exception as e:
            self.logger.error(f"发送请求时出错: {e}")
            return {"success": False, "message": str(e)}

    def vision_detection(
        self,
        image_topic: str,
        service_name: str,
        score: float,
        max_num: int,
    ) -> Dict[str, Any]:
        """
        视觉检测

        Args:
            image_topic: 图像话题名称
            service_name: 检测服务名称
            score: 检测分数
            max_num: 检测最大数量

        Returns:
            Dict: 执行结果
        """
        request = VisionConfig.get_vision_detection_config(
            image_topic=image_topic,
            service_name=service_name,
            score=score,
            max_num=max_num,
        )
        return self._send_request(request)

    def vision_pose_estimation_mask(
        self,
        pose_estimation_service_name: str,
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        mask_service_name: str,
        mask_params: Optional[Dict[str, Any]],
        pose_params: Optional[Dict[str, Any]],
        target_frame: str,
    ) -> Dict[str, Any]:
        """
        基于掩码的位姿估计

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgb_topic_name: RGB图像话题名称
            depth_topic_name: 深度图像话题名称
            camera_info_topic_name: 相机信息话题名称
            mask_service_name: 掩码检测服务名称
            mask_params: 掩码检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            target_frame: 目标坐标系

        Returns:
            Dict: 执行结果
        """
        request = VisionConfig.get_vision_pose_estimation_mask_config(
            pose_estimation_service_name=pose_estimation_service_name,
            rgb_topic_name=rgb_topic_name,
            depth_topic_name=depth_topic_name,
            camera_info_topic_name=camera_info_topic_name,
            mask_service_name=mask_service_name,
            mask_params=mask_params,
            pose_params=pose_params,
            target_frame=target_frame,
        )
        return self._send_request(request)
    
    def vision_pose_estimation_box(
        self,
        pose_estimation_service_name: str,
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        box_service_name: str,
        box_params: Optional[Dict[str, Any]],
        pose_params: Optional[Dict[str, Any]],
        target_frame: str,
    ) -> Dict[str, Any]:
        """
        基于Box的位姿估计

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgb_topic_name: RGB图像话题名称
            depth_topic_name: 深度图像话题名称
            camera_info_topic_name: 相机信息话题名称
            box_service_name: Box检测服务名称
            box_params: Box检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            target_frame: 目标坐标系

        Returns:
            Dict: 执行结果
        """
        request = VisionConfig.get_vision_pose_estimation_box_config(
            pose_estimation_service_name=pose_estimation_service_name,
            rgb_topic_name=rgb_topic_name,
            depth_topic_name=depth_topic_name,
            camera_info_topic_name=camera_info_topic_name,
            box_service_name=box_service_name,
            box_params=box_params,
            pose_params=pose_params,
            target_frame=target_frame,
        )
        return self._send_request(request)

    def vision_template_pose_estimation_box(
        self,
        pose_estimation_service_name: str,
        image_topic_name: str,
        box_service_name: str,
        box_params: Optional[Dict[str, Any]],
        pose_params: Optional[Dict[str, Any]],
        target_frame: str,
        intrinsic_file_path: Optional[str],
        pose_board_in_camera: Optional[List[float]],
    ) -> Dict[str, Any]:
        """
        基于模板匹配的姿态估计

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            image_topic_name: 图像话题名称
            box_service_name: Box检测服务名称
            box_params: Box检测参数（None 表示使用默认值）
            pose_params: 位姿估计参数（None 表示使用默认值）
            target_frame: 目标坐标系
            intrinsic_file_path: 相机内参文件路径（None 表示不使用）
            pose_board_in_camera: 板子在相机坐标系下的位姿 [x, y, z, qx, qy, qz, qw]（None 表示不使用）

        Returns:
            Dict: 执行结果
        """
        request = VisionConfig.get_vision_template_pose_estimation_box_config(
            pose_estimation_service_name=pose_estimation_service_name,
            image_topic_name=image_topic_name,
            box_service_name=box_service_name,
            box_params=box_params,
            pose_params=pose_params,
            target_frame=target_frame,
            intrinsic_file_path=intrinsic_file_path,
            pose_board_in_camera=pose_board_in_camera,
        )
        return self._send_request(request)

    def vision_pose_estimation_obb_mask(
        self,
        pose_estimation_service_name: str,
        rgb_topic_name: str,
        depth_topic_name: str,
        camera_info_topic_name: str,
        obb_service_name: str,
        mask_service_name: str,
        merge_data_service_name: str,
        merge_params: Optional[Dict[str, Any]],
        pose_params: Optional[Dict[str, Any]],
        obb_params: Optional[Dict[str, Any]],
        mask_params: Optional[Dict[str, Any]],
        target_frame: str = "",
        ) -> Dict[str, Any]:
        """
        基于OBB和掩码的位姿估计

        Args:
            pose_estimation_service_name: 位姿估计服务名称
            rgb_topic_name: RGB图像话题名称
            depth_topic_name: 深度图像话题名称
            camera_info_topic_name: 相机信息话题名称
            obb_service_name: OBB检测服务名称
            mask_service_name: 掩码检测服务名称
            merge_data_service_name: 数据融合服务名称
            merge_params: 数据融合参数
            pose_params: 位姿估计参数（None 表示使用默认值）
            obb_params: OBB检测参数（None 表示使用默认值）
            mask_params: 掩码检测参数（None 表示使用默认值）
            target_frame: 目标坐标系（None 表示使用默认值）

        Returns:
            Dict: 执行结果
        """
        request = VisionConfig.get_vision_pose_estimation_obb_mask_config(
            pose_estimation_service_name=pose_estimation_service_name,
            rgb_topic_name=rgb_topic_name,
            depth_topic_name=depth_topic_name,
            camera_info_topic_name=camera_info_topic_name,
            obb_service_name=obb_service_name,
            mask_service_name=mask_service_name,
            merge_data_service_name=merge_data_service_name,
            merge_params=merge_params,
            pose_params=pose_params,
            obb_params=obb_params,
            mask_params=mask_params,
            target_frame=target_frame,
        )
        return self._send_request(request)
    
    def close(self):
        """关闭连接"""
        if self.socket:
            self.socket.close()
        if self.context:
            self.context.term()
        self.logger.info("ZeroMQ客户端连接已关闭")


# ============================================================================
# 测试函数
# ============================================================================


def test_vision_detection(
    client: ZMQVisionClient,
    image_topic: str,
    service_name: str,
    score: float,
    max_num: int,
) -> Dict[str, Any]:
    """
    测试视觉检测功能

    Args:
        client: ZMQVisionClient 实例
        image_topic: 图像话题名称
        service_name: 检测服务名称
        score: 检测分数
        max_num: 检测最大数量
    """
    print("🔍 测试视觉检测功能...")
    print(f"   参数: image_topic='{image_topic}', service_name='{service_name}', score={score}, max_num={max_num}")

    result = client.vision_detection(
        image_topic=image_topic,
        service_name=service_name,
        score=score,
        max_num=max_num,
    )

    if result.get("success", False):
        print("✅ 视觉检测测试成功")
    else:
        print("❌ 视觉检测测试失败")
        print(f"   错误信息: {result.get('message', '')}")

    return result


def test_vision_pose_estimation_mask(
    client: ZMQVisionClient,
    pose_estimation_service_name: str,
    rgb_topic_name: str,
    depth_topic_name: str,
    camera_info_topic_name: str,
    mask_service_name: str,
    target_frame: str,
) -> Dict[str, Any]:
    """
    测试基于掩码的位姿估计功能

    Args:
        client: ZMQVisionClient 实例
        pose_estimation_service_name: 位姿估计服务名称
        rgb_topic_name: RGB图像话题名称
        depth_topic_name: 深度图像话题名称
        camera_info_topic_name: 相机信息话题名称
        mask_service_name: 掩码检测服务名称
        target_frame: 目标坐标系
        注意: mask_params 和 pose_params 使用 None（方法内部默认值）
    """
    print("🎯 测试基于掩码的位姿估计功能...")
    print(f"   参数: pose_estimation_service_name='{pose_estimation_service_name}'")
    print(f"        rgb_topic_name='{rgb_topic_name}'")
    print(f"        depth_topic_name='{depth_topic_name}'")
    print(f"        camera_info_topic_name='{camera_info_topic_name}'")
    print(f"        mask_service_name='{mask_service_name}'")
    print(f"        target_frame='{target_frame}'")
    print("        mask_params=None (使用默认值)")
    print("        pose_params=None (使用默认值)")

    result = client.vision_pose_estimation_mask(
        pose_estimation_service_name=pose_estimation_service_name,
        rgb_topic_name=rgb_topic_name,
        depth_topic_name=depth_topic_name,
        camera_info_topic_name=camera_info_topic_name,
        mask_service_name=mask_service_name,
        mask_params=None,  # 字典参数使用默认值
        pose_params=None,  # 字典参数使用默认值
        target_frame=target_frame,
    )

    if result.get("success", False):
        print("✅ 基于掩码的位姿估计测试成功")
    else:
        print("❌ 基于掩码的位姿估计测试失败")
        print(f"   错误信息: {result.get('message', '')}")

    return result

def test_vision_pose_estimation_box(
    client: ZMQVisionClient,
    pose_estimation_service_name: str,
    rgb_topic_name: str,
    depth_topic_name: str,
    camera_info_topic_name: str,
    box_service_name: str,
    target_frame: str,
) -> Dict[str, Any]:
    """
    测试基于Box的位姿估计功能

    Args:
        client: ZMQVisionClient 实例
        pose_estimation_service_name: 位姿估计服务名称
        rgb_topic_name: RGB图像话题名称
        depth_topic_name: 深度图像话题名称
        camera_info_topic_name: 相机信息话题名称
        box_service_name: Box检测服务名称
        target_frame: 目标坐标系
        注意: box_params 和 pose_params 使用 None（方法内部默认值）
    """
    print("🎯 测试基于Box的位姿估计功能...")
    print(f"   参数: pose_estimation_service_name='{pose_estimation_service_name}'")
    print(f"        rgb_topic_name='{rgb_topic_name}'")
    print(f"        depth_topic_name='{depth_topic_name}'")
    print(f"        camera_info_topic_name='{camera_info_topic_name}'")
    print(f"        box_service_name='{box_service_name}'")
    print(f"        target_frame='{target_frame}'")
    print("        box_params=None (使用默认值)")
    print("        pose_params=None (使用默认值)")

    result = client.vision_pose_estimation_box(
        pose_estimation_service_name=pose_estimation_service_name,
        rgb_topic_name=rgb_topic_name,
        depth_topic_name=depth_topic_name,
        camera_info_topic_name=camera_info_topic_name,
        box_service_name=box_service_name,
        box_params=None,  # 字典参数使用默认值
        pose_params=None,  # 字典参数使用默认值
        target_frame=target_frame,
    )

    if result.get("success", False):
        print("✅ 基于Box的位姿估计测试成功")
    else:
        print("❌ 基于Box的位姿估计测试失败")
        print(f"   错误信息: {result.get('message', '')}")

    return result


def test_vision_template_pose_estimation_box(
    client: ZMQVisionClient,
    pose_estimation_service_name: str,
    image_topic_name: str,
    box_service_name: str,
    target_frame: str,
    intrinsic_file_path: Optional[str],
    pose_board_in_camera: Optional[List[float]],
) -> Dict[str, Any]:
    """
    测试基于模板匹配的姿态估计功能

    Args:
        client: ZMQVisionClient 实例
        pose_estimation_service_name: 位姿估计服务名称
        image_topic_name: 图像话题名称
        box_service_name: Box检测服务名称
        target_frame: 目标坐标系
        intrinsic_file_path: 相机内参文件路径
        pose_board_in_camera: 板子在相机坐标系下的位姿 [x, y, z, qx, qy, qz, qw]
        注意: box_params, pose_params 使用 None（方法内部默认值）
    """
    print("🎯 测试基于模板匹配的姿态估计功能...")
    print(f"   参数: pose_estimation_service_name='{pose_estimation_service_name}'")
    print(f"        image_topic_name='{image_topic_name}'")
    print(f"        box_service_name='{box_service_name}'")
    print(f"        target_frame='{target_frame}'")
    print(f"        intrinsic_file_path='{intrinsic_file_path or 'None'}'")
    print(f"        pose_board_in_camera='{pose_board_in_camera or 'None'}'")
    print("        box_params=None (使用默认值)")
    print("        pose_params=None (使用默认值)")

    result = client.vision_template_pose_estimation_box(
        pose_estimation_service_name=pose_estimation_service_name,
        image_topic_name=image_topic_name,
        box_service_name=box_service_name,
        box_params=None,  # 字典参数使用默认值
        pose_params=None,  # 字典参数使用默认值
        target_frame=target_frame,
        intrinsic_file_path=intrinsic_file_path,
        pose_board_in_camera=pose_board_in_camera,
    )

    if result.get("success", False):
        print("✅ 基于模板匹配的姿态估计测试成功")
    else:
        print("❌ 基于模板匹配的姿态估计测试失败")
        print(f"   错误信息: {result.get('message', '')}")

    return result


def test_vision_pose_estimation_obb_mask(
    client: ZMQVisionClient,
    pose_estimation_service_name: str,
    rgb_topic_name: str,
    depth_topic_name: str,
    camera_info_topic_name: str,
    mask_service_name: str,
    obb_service_name: str,
    merge_data_service_name: str,
    target_frame: str,
) -> Dict[str, Any]:
    """
    测试基于OBB和掩码的位姿估计功能

    Args:
        client: ZMQVisionClient 实例
        pose_estimation_service_name: 位姿估计服务名称
        rgb_topic_name: RGB图像话题名称
        depth_topic_name: 深度图像话题名称
        camera_info_topic_name: 相机信息话题名称
        mask_service_name: 掩码检测服务名称
        obb_service_name: OBB检测服务名称
        target_frame: 目标坐标系
    """
    print("🎯 测试基于OBB和掩码的位姿估计功能...")
    print(f"   参数: pose_estimation_service_name='{pose_estimation_service_name}'")
    print(f"        rgb_topic_name='{rgb_topic_name}'")
    print(f"        depth_topic_name='{depth_topic_name}'")
    print(f"        camera_info_topic_name='{camera_info_topic_name}'")
    print(f"        mask_service_name='{mask_service_name}'")
    print(f"        obb_service_name='{obb_service_name}'")
    print(f"        merge_data_service_name='{merge_data_service_name}'")
    print(f"        target_frame='{target_frame}'")

    result = client.vision_pose_estimation_obb_mask(
        pose_estimation_service_name=pose_estimation_service_name,
        rgb_topic_name=rgb_topic_name,
        depth_topic_name=depth_topic_name,
        camera_info_topic_name=camera_info_topic_name,
        obb_service_name=obb_service_name,
        mask_service_name=mask_service_name,
        merge_data_service_name=merge_data_service_name,
        merge_params=None,  # 字典参数使用默认值
        pose_params=None,  # 字典参数使用默认值
        obb_params=None,  # 字典参数使用默认值
        mask_params=None,  # 字典参数使用默认值
        target_frame=target_frame,
    )

    if result.get("success", False):
        print("✅ 基于OBB和掩码的位姿估计测试成功")
    else:
        print("❌ 基于OBB和掩码的位姿估计测试失败")
        print(f"   错误信息: {result.get('message', '')}")

    return result

def main():
    """主函数 - 测试ZeroMQ视觉客户端"""
    parser = argparse.ArgumentParser(
        description="PapJia Vision ZeroMQ 客户端测试工具 - 测试三个 get*** 方法",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
测试用例说明:
  1. detection              - 测试视觉检测 (get_vision_detection_config)
  2. pose_estimation_mask   - 测试基于掩码的位姿估计 (get_vision_pose_estimation_mask_config)
  3. template_pose_box       - 测试基于模板匹配的姿态估计 (get_vision_template_pose_estimation_box_config)
  4. all                    - 运行所有测试

示例命令:
  # 运行所有测试
  python vision_client.py --test all

  # 运行单个测试
  python vision_client.py --test detection
  python vision_client.py --test pose_estimation_mask
  python vision_client.py --test template_pose_box

  # 自定义参数
  python vision_client.py --test detection --image_topic /camera/rgb/image_raw --service_name /my_detection_service
        """,
    )

    # 测试选择
    parser.add_argument(
        "--test",
        type=str,
        choices=["detection", "pose_estimation_mask", "template_pose_box", "pose_estimation_box","pose_estimation_obb_mask","all"],
        default="all",
        help="选择要测试的功能 (默认: all)",
    )

    # 服务器配置
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="服务器地址",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=7020,
        help="服务器端口",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=30,
        help="请求超时时间（秒）",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="详细输出",
    )

    # 视觉检测参数
    parser.add_argument(
        "--image_topic",
        type=str,
        default="/left_camera/color/image_raw",
        help="图像话题名称",
    )
    parser.add_argument(
        "--detection_service_name",
        type=str,
        default="/vision/detection_box_bottle",
        help="检测服务名称",
    )
    parser.add_argument(
        "--score",
        type=float,
        default=0.8,
        help="检测分数",
    )
    parser.add_argument(
        "--max_num",
        type=int,
        default=10,
        help="最大检测数量",
    )

    # 基于掩码的位姿估计参数
    parser.add_argument(
        "--pose_estimation_service_name",
        type=str,
        default="/vision/pose_estimation",
        help="位姿估计服务名称",
    )
    parser.add_argument(
        "--rgb_topic_name",
        type=str,
        default="/left_camera/color/image_raw",
        help="RGB图像话题名称",
    )
    parser.add_argument(
        "--depth_topic_name",
        type=str,
        default="/left_camera/aligned_depth_to_color/image_raw",
        help="深度图像话题名称",
    )
    parser.add_argument(
        "--camera_info_topic_name",
        type=str,
        default="/left_camera/color/camera_info",
        help="相机信息话题名称",
    )
    parser.add_argument(
        "--mask_service_name",
        type=str,
        default="/vision/detection_mask_welding",
        help="掩码检测服务名称",
    )
    parser.add_argument(
        "--target_frame",
        type=str,
        default="left_camera_workspace",
        help="目标坐标系名称",
    )
    parser.add_argument(
        "--obb_service_name",
        type=str,
        default="/vision/detection_box_welding_part",
        help="OBB检测服务名称",
    )
    parser.add_argument(
        "--merge_data_service_name",
        type=str,
        default="/type_trans/merge_json_data",
        help="数据合并服务名称",
    )

    # 基于模板匹配的姿态估计参数
    parser.add_argument(
        "--template_pose_estimation_service_name",
        type=str,
        default="/vision/template_pose_estimation",
        help="模板匹配位姿估计服务名称",
    )
    parser.add_argument(
        "--template_image_topic_name",
        type=str,
        default="/left_camera/color/image_raw",
        help="模板匹配图像话题名称",
    )
    parser.add_argument(
        "--box_service_name",
        type=str,
        default="/vision/detection_box",
        help="Box检测服务名称",
    )
    parser.add_argument(
        "--intrinsic_file_path",
        type=str,
        default=None,
        help="相机内参文件路径",
    )
    parser.add_argument(
        "--pose_board_in_camera",
        nargs=7,
        type=float,
        default=None,
        help="板子在相机坐标系下的位姿 [x, y, z, qx, qy, qz, qw]",
    )

    args = parser.parse_args()

    # 设置日志级别
    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")

    print("🚀 开始测试三个 get*** 方法")
    print(f"🌐 服务器地址: {args.host}:{args.port}")
    print(f"⏱️ 超时时间: {args.timeout}秒")
    print("=" * 50)

    try:
        # 创建客户端
        client = ZMQVisionClient(host=args.host, port=args.port, timeout=args.timeout)

        # ========================================================================
        # 1. 测试视觉检测
        # ========================================================================
        if args.test in ["detection", "all"]:
            print("\n" + "=" * 50)
            print("1. 测试视觉检测 (get_vision_detection_config)")
            print("=" * 50)
            result1 = test_vision_detection(
                client,
                image_topic=args.image_topic,
                service_name=args.detection_service_name,
                score=args.score,
                max_num=args.max_num,
            )
            if args.verbose:
                print(f"详细结果: {json.dumps(result1, indent=2, ensure_ascii=False)}")

        # ========================================================================
        # 2. 测试基于掩码的位姿估计
        # ========================================================================
        if args.test in ["pose_estimation_mask", "all"]:
            print("\n" + "=" * 50)
            print("2. 测试基于掩码的位姿估计 (get_vision_pose_estimation_mask_config)")
            print("=" * 50)
            result2 = test_vision_pose_estimation_mask(
                client,
                pose_estimation_service_name=args.pose_estimation_service_name,
                rgb_topic_name=args.rgb_topic_name,
                depth_topic_name=args.depth_topic_name,
                camera_info_topic_name=args.camera_info_topic_name,
                mask_service_name=args.mask_service_name,
                target_frame=args.target_frame,
            )
            if args.verbose:
                print(f"详细结果: {json.dumps(result2, indent=2, ensure_ascii=False)}")

        # ========================================================================
        # 3. 测试基于box的位姿估计
        # ========================================================================
        if args.test in ["pose_estimation_box", "all"]:
            print("\n" + "=" * 50)
            print("3. 测试基于box的位姿估计 (get_vision_pose_estimation_box_config)")
            print("=" * 50)
            result3 = test_vision_pose_estimation_box(
                client,
                pose_estimation_service_name=args.pose_estimation_service_name,
                rgb_topic_name=args.rgb_topic_name,
                depth_topic_name=args.depth_topic_name,
                camera_info_topic_name=args.camera_info_topic_name,
                box_service_name=args.detection_service_name,
                target_frame=args.target_frame,
            )
            if args.verbose:
                print(f"详细结果: {json.dumps(result3, indent=2, ensure_ascii=False)}")

        # ========================================================================
        # 4. 测试基于模板匹配的姿态估计
        # ========================================================================
        if args.test in ["template_pose_box", "all"]:
            print("\n" + "=" * 50)
            print("4. 测试基于模板匹配的姿态估计 (get_vision_template_pose_estimation_box_config)")
            print("=" * 50)
            result4 = test_vision_template_pose_estimation_box(
                client,
                pose_estimation_service_name=args.template_pose_estimation_service_name,
                image_topic_name=args.template_image_topic_name,
                box_service_name=args.box_service_name,
                target_frame=args.target_frame,
                intrinsic_file_path=args.intrinsic_file_path,
                pose_board_in_camera=args.pose_board_in_camera,
            )
            if args.verbose:
                print(f"详细结果: {json.dumps(result4, indent=2, ensure_ascii=False)}")
        
        # ========================================================================
        # 5. 测试基于掩码和obb的位姿估计
        # ========================================================================
        if args.test in ["pose_estimation_obb_mask", "all"]:
            print("\n" + "=" * 50)
            print("5. 测试基于掩码和obb的位姿估计 (get_vision_pose_estimation_obb_mask_config)")
            print("=" * 50)
            result5 = test_vision_pose_estimation_obb_mask(
                client,
                pose_estimation_service_name=args.pose_estimation_service_name,
                rgb_topic_name=args.rgb_topic_name,
                depth_topic_name=args.depth_topic_name,
                camera_info_topic_name=args.camera_info_topic_name,
                mask_service_name=args.mask_service_name,
                obb_service_name=args.obb_service_name,
                merge_data_service_name=args.merge_data_service_name,
                target_frame=args.target_frame,
            )
            if args.verbose:
                print(f"详细结果: {json.dumps(result5, indent=2, ensure_ascii=False)}")

        print("\n" + "=" * 50)
        print("✅ 所有测试完成!")

    except KeyboardInterrupt:
        print("\n⚠️ 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # 关闭客户端连接
        if "client" in locals():
            client.close()


if __name__ == "__main__":
    main()
