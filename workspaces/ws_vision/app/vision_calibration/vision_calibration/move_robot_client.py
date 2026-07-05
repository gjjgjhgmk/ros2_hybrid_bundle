#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MoveRobot 服务客户端

该脚本用于调用 MoveRobot 服务，控制机械臂进行规划和执行。
支持增量规划和目标规划两种模式。
支持坐标系转换功能，可将位姿从arm_base坐标系转换到robot_base坐标系。

使用示例:
    # 增量规划模式（仅规划，不执行）
    python3 move_robot_client.py --group_name arm --planning_mode 0 --execute false --dx 0.1 --dy 0.0 --dz 0.0

    # 增量规划模式（规划并执行）
    python3 move_robot_client.py --group_name arm --planning_mode 0 --execute true --dx 0.1 --dy 0.0 --dz 0.0

    # 目标规划模式（不进行坐标系转换）
    python3 move_robot_client.py --group_name arm --planning_mode 1 --execute true --x 0.6 --y 0.1 --z 0.7

    # 目标规划模式（进行坐标系转换：从arm_base转换到robot_base）
    python3 move_robot_client.py --group_name arm --planning_mode 1 --execute true \\
        --x 0.6 --y 0.1 --z 0.7 \\
        --transform_pose true --robot_base_frame base_link --arm_base_frame left_arm_base
"""

import sys
import argparse
import time
import rclpy
from rclpy.node import Node
from moveit_example.srv import MoveRobot
from geometry_msgs.msg import Pose, Point, Quaternion
from .ros2_tf_subscriber import ROS2TFSubscriber


class MoveRobotClient(Node):
    """MoveRobot 服务客户端类"""

    def __init__(self, service_name="move_robot"):
        super().__init__("move_robot_client")
        self.client = self.create_client(MoveRobot, service_name)

    def call_service(
        self,
        group_name,
        planning_mode,
        execute,
        increment=None,
        target_pose=None,
        timeout_sec=60.0,
    ):
        """
        调用 MoveRobot 服务

        Args:
            group_name: 机械臂组名称
            planning_mode: 规划模式 (0=增量规划, 1=目标规划)
            execute: 是否执行 (True/False)
            increment: 增量规划模式下的增量字典 {"dx": float, "dy": float, "dz": float,
                     "droll": float, "dpitch": float, "dyaw": float}
            target_pose: 目标规划模式下的目标位姿 (geometry_msgs/Pose)
            timeout_sec: 服务调用超时时间（秒），默认60秒

        Returns:
            MoveRobot.Response: 服务响应
        """
        # 创建服务请求
        request = MoveRobot.Request()
        request.group_name = group_name
        request.planning_mode = planning_mode
        request.execute = execute

        if planning_mode == 0:
            # 增量规划模式
            if increment is None:
                increment = {"dx": 0.0, "dy": 0.0, "dz": 0.0, "droll": 0.0, "dpitch": 0.0, "dyaw": 0.0}

            request.dx = increment.get("dx", 0.0)
            request.dy = increment.get("dy", 0.0)
            request.dz = increment.get("dz", 0.0)
            request.droll = increment.get("droll", 0.0)
            request.dpitch = increment.get("dpitch", 0.0)
            request.dyaw = increment.get("dyaw", 0.0)

            self.get_logger().info(
                f"发送增量规划请求: group_name={group_name}, "
                f"dx={request.dx:.4f}, dy={request.dy:.4f}, dz={request.dz:.4f}, "
                f"droll={request.droll:.4f}, dpitch={request.dpitch:.4f}, dyaw={request.dyaw:.4f}, "
                f"execute={execute}"
            )
        elif planning_mode == 1:
            # 目标规划模式
            if target_pose is None:
                self.get_logger().error("目标规划模式需要提供 target_pose")
                return None
            request.target_pose = target_pose
            self.get_logger().info(
                f"发送目标规划请求: group_name={group_name}, "
                f"target_pose=({target_pose.position.x:.4f}, "
                f"{target_pose.position.y:.4f}, {target_pose.position.z:.4f}), "
                f"execute={execute}"
            )
        else:
            self.get_logger().error(f"无效的规划模式: {planning_mode} (应为 0 或 1)")
            return None

        # 同步调用服务
        try:
            self.get_logger().info(f"调用服务，超时时间: {timeout_sec} 秒")
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

            if future.done():
                response = future.result()
                if response is not None:
                    self.get_logger().info(f"服务响应: planned={response.planned}, success={response.success}")
                    return response
                else:
                    self.get_logger().error("服务调用失败: 响应为空")
                    return None
            else:
                self.get_logger().error(f"服务调用超时（超时时间: {timeout_sec} 秒）")
                return None
        except Exception as e:
            self.get_logger().error(f"服务调用异常: {str(e)}")
            return None


def create_target_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    """
    创建目标位姿

    Args:
        x, y, z: 位置坐标
        qx, qy, qz, qw: 四元数

    Returns:
        geometry_msgs/Pose: 位姿对象
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose


def parse_arguments():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="MoveRobot 服务客户端",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 增量规划模式（仅规划）
  %(prog)s --group_name arm --planning_mode 0 --execute false --dx 0.1

  # 增量规划模式（规划并执行）
  %(prog)s --group_name arm --planning_mode 0 --execute true --dx 0.1 --dy 0.0 --dz 0.0

  # 目标规划模式（不进行坐标系转换）
  %(prog)s --group_name arm --planning_mode 1 --execute true --x 0.6 --y 0.1 --z 0.7

  # 目标规划模式（进行坐标系转换：从arm_base转换到robot_base）
  %(prog)s --group_name arm --planning_mode 1 --execute true \\
      --x 0.6 --y 0.1 --z 0.7 \\
      --transform_pose true --robot_base_frame base_link --arm_base_frame left_arm_base
        """,
    )

    # 必需参数
    parser.add_argument(
        "--group_name",
        type=str,
        required=True,
        help="机械臂组名称 (例如: arm, left_arm, right_arm)",
    )

    parser.add_argument(
        "--planning_mode",
        type=int,
        choices=[0, 1],
        required=True,
        help="规划模式: 0=增量规划, 1=目标规划",
    )

    parser.add_argument(
        "--execute",
        type=lambda x: x.lower() == "true",
        required=True,
        help="是否执行: true=规划并执行, false=仅规划",
    )

    # 增量规划模式参数
    parser.add_argument("--dx", type=float, default=0.0, help="X 方向增量 (米)")
    parser.add_argument("--dy", type=float, default=0.0, help="Y 方向增量 (米)")
    parser.add_argument("--dz", type=float, default=0.0, help="Z 方向增量 (米)")
    parser.add_argument("--droll", type=float, default=0.0, help="Roll 角度增量 (弧度)")
    parser.add_argument("--dpitch", type=float, default=0.0, help="Pitch 角度增量 (弧度)")
    parser.add_argument("--dyaw", type=float, default=0.0, help="Yaw 角度增量 (弧度)")

    # 目标规划模式参数
    parser.add_argument("--x", type=float, default=0.0, help="目标 X 坐标 (米)")
    parser.add_argument("--y", type=float, default=0.0, help="目标 Y 坐标 (米)")
    parser.add_argument("--z", type=float, default=0.0, help="目标 Z 坐标 (米)")
    parser.add_argument("--qx", type=float, default=0.0, help="目标四元数 X")
    parser.add_argument("--qy", type=float, default=0.0, help="目标四元数 Y")
    parser.add_argument("--qz", type=float, default=0.0, help="目标四元数 Z")
    parser.add_argument("--qw", type=float, default=1.0, help="目标四元数 W")

    # 服务名称（可选）
    parser.add_argument(
        "--service_name",
        type=str,
        default="example_move_robot",
        help="服务名称 (默认: move_robot)",
    )

    # 坐标系转换参数
    parser.add_argument(
        "--robot_base_frame",
        type=str,
        default="",
        help="机器人基座坐标系名称 (用于坐标系转换)",
    )
    parser.add_argument(
        "--arm_base_frame",
        type=str,
        default="",
        help="机械臂基座坐标系名称 (用于坐标系转换)",
    )
    parser.add_argument(
        "--end_effector_frame",
        type=str,
        default="",
        help="末端执行器坐标系名称 (用于坐标系转换，可选)",
    )
    parser.add_argument(
        "--transform_pose",
        type=lambda x: x.lower() == "true",
        default=False,
        help="是否进行坐标系转换: true=转换, false=不转换",
    )

    # 服务调用超时参数
    parser.add_argument(
        "--service_timeout",
        type=float,
        default=60.0,
        help="服务调用超时时间（秒），默认60秒。复杂规划或长轨迹执行可能需要更长时间",
    )

    return parser.parse_args()


def main():
    # 解析命令行参数
    args = parse_arguments()

    # 初始化 ROS2
    rclpy.init()

    execute = args.execute
    planning_mode = args.planning_mode

    args.qw = -0.001055124550865085
    args.qx = -0.7070914956397442
    args.qy = 0.7071193616394816
    args.qz = 0.001646784959720661
    args.x = 0.4065698105510093
    args.y = 0.14520095465395969
    args.z = 0.3968544648018423

    # 创建TF订阅器（在main中创建，用于位姿转换）
    tf_subscriber = None
    if args.transform_pose:
        tf_subscriber = ROS2TFSubscriber(use_sim_time=True)

    try:
        # 创建客户端
        client = MoveRobotClient(service_name=args.service_name)

        # 根据规划模式准备参数
        target_pose = None
        increment = None
        if planning_mode == 1:
            # 目标规划模式：创建目标位姿
            target_pose = create_target_pose(args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw)

            # 如果指定了坐标系转换参数，进行位姿转换
            if args.transform_pose:
                if not args.robot_base_frame or not args.arm_base_frame:
                    client.get_logger().error("坐标系转换需要指定 --robot_base_frame 和 --arm_base_frame")
                    sys.exit(1)

                client.get_logger().info(f"执行坐标系转换: 从 '{args.arm_base_frame}' 到 '{args.robot_base_frame}'")

                # 转换位姿（在main中进行）
                target_pose = tf_subscriber.transform_pose(
                    pose=target_pose,
                    child_frame=args.arm_base_frame,
                    parent_frame=args.robot_base_frame,
                    timeout_sec=5.0,
                )

                if target_pose is None:
                    client.get_logger().error("位姿转换失败，退出")
                    sys.exit(1)

        # 准备增量参数（合并为一个字典）
        if planning_mode == 0:
            increment = {
                "dx": args.dx,
                "dy": args.dy,
                "dz": args.dz,
                "droll": args.droll,
                "dpitch": args.dpitch,
                "dyaw": args.dyaw,
            }

        # 调用服务
        response = client.call_service(
            group_name=args.group_name,
            planning_mode=planning_mode,
            execute=execute,
            increment=increment,
            target_pose=target_pose,
            timeout_sec=args.service_timeout,
        )

        # 处理响应
        if response is not None:
            if response.planned:
                if execute:
                    if response.success:
                        print("✓ 规划成功，执行成功")
                        sys.exit(0)
                    else:
                        print("✗ 规划成功，但执行失败")
                        sys.exit(1)
                else:
                    print("✓ 规划成功（未执行）")
                    sys.exit(0)
            else:
                print("✗ 规划失败")
                sys.exit(1)
        else:
            print("✗ 服务调用失败")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"错误: {str(e)}")
    finally:
        # 清理TF订阅器资源
        if tf_subscriber is not None:
            tf_subscriber.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
