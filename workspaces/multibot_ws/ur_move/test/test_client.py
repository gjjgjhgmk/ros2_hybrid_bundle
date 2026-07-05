#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR Move 测试脚本
从 waypoints.json 选择路径点并发送到 ur_move 服务进行规划与执行
"""

import sys
import os
import json
import time
import argparse
import yaml
from pathlib import Path
from typing import Dict, Any, List

# 添加父目录到路径，以便导入客户端模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'client'))

try:
    from zmq_ur_move_client import UrMoveClient
    from zmq_gripper_client import GripperZMQClient
    # 确保 TrajectoryExecutorClient 也被导入（用于远程执行功能）
    try:
        from trajectory_executor_client import TrajectoryExecutorClient
    except ImportError:
        pass  # 如果导入失败，zmq_ur_move_client 会处理
    import logging
    GRIPPER_AVAILABLE = True
except ImportError as e:
    print(f"导入失败: {e}")
    print("请确保在正确的工作空间环境中运行此脚本")
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 默认配置
DEFAULT_ROOT = Path(__file__).resolve().parent
DEFAULT_WAYPOINT_PATH = DEFAULT_ROOT / "waypoints.json"
DEFAULT_SERVER_HOST = "localhost"
DEFAULT_GRIPPER_HOST = "localhost"
DEFAULT_GRIPPER_LEFT_PORT = 5630
DEFAULT_GRIPPER_RIGHT_PORT = 5640


def parse_args(argv: List[str]) -> argparse.Namespace:
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="从 waypoints.json 选择路径点并发送到 ur_move 服务进行规划与执行",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 规划执行手臂轨迹
  %(prog)s --mode arm --names "左臂-home" "右臂-home"
  
  # 控制夹爪
  %(prog)s --mode gripper --gripper-action open --gripper-name left
  %(prog)s --mode gripper --gripper-position 0.4 --gripper-name right
  
  # 设置夹爪力
  %(prog)s --mode gripper --gripper-action close --gripper-name left --gripper-effort 100.0
  %(prog)s --mode gripper --gripper-position 0.5 --gripper-name right --gripper-effort 30.0
        """
    )
    parser.add_argument(
        "--server",
        type=str,
        default=DEFAULT_SERVER_HOST,
        help=f"ur_move 规划服务器主机地址（默认: {DEFAULT_SERVER_HOST}，端口固定为5605）"
    )
    parser.add_argument(
        "--waypoints-file",
        type=Path,
        default=DEFAULT_WAYPOINT_PATH,
        help=f"路径点文件路径（默认: {DEFAULT_WAYPOINT_PATH}）"
    )
    parser.add_argument(
        "--names",
        nargs="+",
        help="要规划和执行的路径点名称（arm 模式必需）"
    )
    parser.add_argument(
        "--mode",
        choices=["arm", "gripper", "remote"],
        required=True,
        help="执行模式: arm=规划执行轨迹, gripper=控制夹爪, remote=远程规划执行"
    )
    parser.add_argument(
        "--wait-confirm",
        action="store_true",
        help="规划完成后等待确认再执行"
    )
    parser.add_argument(
        "--no-execute",
        action="store_true",
        help="只规划，不执行轨迹"
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="请求超时时间（秒，默认 60）"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="输出调试日志"
    )
    parser.add_argument(
        "--gripper-position",
        type=float,
        metavar="POS",
        help="设置夹爪位置 (0.0=打开, 0.8=关闭，gripper 模式)"
    )
    parser.add_argument(
        "--gripper-name",
        choices=["left", "right"],
        default="left",
        help="夹爪名称（gripper 模式必需，默认: left）"
    )
    parser.add_argument(
        "--gripper-action",
        choices=["open", "close"],
        help="夹爪动作（gripper 模式，与 --gripper-position 二选一）"
    )
    parser.add_argument(
        "--gripper-effort",
        type=float,
        default=50.0,
        metavar="FORCE",
        help="夹爪最大力 (N)（默认: 50.0，范围: 0-235）"
    )
    parser.add_argument(
        "--gripper-host",
        type=str,
        default=DEFAULT_GRIPPER_HOST,
        help=f"夹爪服务器地址（默认: {DEFAULT_GRIPPER_HOST}）"
    )
    parser.add_argument(
        "--gripper-port",
        type=int,
        help=f"夹爪服务器端口（默认: {DEFAULT_GRIPPER_LEFT_PORT}=左手, {DEFAULT_GRIPPER_RIGHT_PORT}=右手）"
    )
    # 远程执行参数
    parser.add_argument(
        "--left-executor",
        type=str,
        default="localhost",
        help="左臂执行服务器主机地址（默认: localhost）"
    )
    parser.add_argument(
        "--right-executor",
        type=str,
        default="localhost",
        help="右臂执行服务器主机地址（默认: localhost）"
    )
    return parser.parse_args(argv)


def control_gripper(args) -> int:
    """执行夹爪控制（使用 ZMQ 客户端）"""
    if not GRIPPER_AVAILABLE:
        logger.error("夹爪 ZMQ 客户端模块未加载")
        return 1
    
    # 确定端口（根据夹爪名称自动选择，或使用用户指定的端口）
    if args.gripper_port is not None:
        port = args.gripper_port
    else:
        port = DEFAULT_GRIPPER_LEFT_PORT if args.gripper_name == "left" else DEFAULT_GRIPPER_RIGHT_PORT
        
        # 验证力的范围
        if args.gripper_effort < 0 or args.gripper_effort > 235:
            logger.warning(f"夹爪力值 {args.gripper_effort}N 超出范围 (0-235N)，将使用限制值")
            args.gripper_effort = max(0.0, min(235.0, args.gripper_effort))
        
    # 创建 ZMQ 客户端（连接会在首次调用时自动建立）
    client = GripperZMQClient(
        server_host=args.gripper_host,
        port=port,
        gripper_name=args.gripper_name
    )
    
    try:
        # 执行夹爪动作
        # 注意：连接会在 open/close/set_position 内部自动建立
        success = False
        if args.gripper_action == "open":
            logger.info(f"打开 {args.gripper_name} 夹爪（最大力: {args.gripper_effort:.1f}N）...")
            success = client.open(max_effort=args.gripper_effort)
        elif args.gripper_action == "close":
            logger.info(f"关闭 {args.gripper_name} 夹爪（最大力: {args.gripper_effort:.1f}N）...")
            success = client.close(max_effort=args.gripper_effort)
        elif args.gripper_position is not None:
            logger.info(f"设置 {args.gripper_name} 夹爪位置: {args.gripper_position:.3f}（最大力: {args.gripper_effort:.1f}N）...")
            success = client.set_position(args.gripper_position, max_effort=args.gripper_effort)
        else:
            logger.error("未指定夹爪动作")
            return 1
        
        if success:
            logger.info("")
            logger.info("✓ 夹爪操作成功！")
            return 0
        else:
            logger.error("")
            logger.error("✗ 夹爪操作失败！")
            return 1
        
    except Exception as e:
        logger.error(f"夹爪控制失败: {e}", exc_info=args.verbose)
        return 1
    # 注意：client 会在析构时自动关闭连接，无需手动调用 _close()


def test_remote_execution(args) -> int:
    """测试远程执行（规划需要自己调用）"""
    logger.info("=" * 60)
    logger.info("远程执行测试")
    logger.info("=" * 60)
    logger.info("规划服务器: %s", args.server)
    logger.info("左臂执行服务器: %s", args.left_executor)
    logger.info("右臂执行服务器: %s", args.right_executor)
    logger.info("路径点: %s", ", ".join(args.names))
    logger.info("=" * 60)
    
    # 检查路径点文件
    if not args.waypoints_file.exists():
        logger.error(f"路径点文件不存在: {args.waypoints_file}")
        return 1
    
    # 加载路径点
    try:
        with open(args.waypoints_file, 'r', encoding='utf-8') as f:
            all_waypoints = json.load(f)
    except Exception as e:
        logger.error(f"加载路径点文件失败: {e}")
        return 1
    
    # 检查路径点名称
    missing = [name for name in args.names if name not in all_waypoints]
    if missing:
        logger.error(f"未找到路径点: {', '.join(missing)}")
        return 1
    
    # 构建路径点字典
    test_waypoints = {name: all_waypoints[name] for name in args.names}
    
    # 创建客户端（设置执行服务器主机地址）
    # 处理兼容性：如果用户传入了完整地址，提取主机部分
    server_host = args.server
    if server_host.startswith("tcp://"):
        server_host = server_host[7:].split(":")[0]
    elif ":" in server_host:
        server_host = server_host.split(":")[0]
    
    client = UrMoveClient(
        server_host=server_host, 
        timeout_ms=args.timeout * 1000,
        left_arm_executor_host=args.left_executor,
        right_arm_executor_host=args.right_executor
    )
    
    try:
        # 1. 规划轨迹（需要自己调用）
        logger.info("")
        logger.info("步骤1: 规划轨迹...")
        plan_start_time = time.time()
        
        plan_result = client.plan_trajectory(test_waypoints, execute=False)
        
        plan_elapsed = time.time() - plan_start_time
        logger.info(f"规划耗时: {plan_elapsed:.2f} 秒")
        
        if not plan_result.get("success", False):
            logger.error(f"✗ 轨迹规划失败: {plan_result.get('error', 'Unknown error')}")
            return 1
        
        trajectories = plan_result.get("trajectories", {})
        if trajectories:
            logger.info(f"✓ 规划成功，包含 {len(trajectories)} 个组的轨迹")
        
        # 2. 远程执行轨迹
        logger.info("")
        logger.info("步骤2: 远程执行轨迹...")
        exec_start_time = time.time()
        
        result = client.execute_remote(
            plan_result=plan_result
        )
        
        exec_elapsed = time.time() - exec_start_time
        logger.info(f"执行耗时: {exec_elapsed:.2f} 秒")
        
        total_elapsed = time.time() - plan_start_time
        logger.info(f"总耗时: {total_elapsed:.2f} 秒")
        
        if not result.get("success", False):
            logger.error(f"✗ 远程执行失败: {result.get('error', 'Unknown error')}")
            return 1
        
        # 显示执行结果
        execution_results = result.get("execution_results", {})
        if execution_results:
            logger.info("")
            logger.info("执行结果:")
            for arm_name, exec_result in execution_results.items():
                status = "✓" if exec_result.get("success", False) else "✗"
                message = exec_result.get("message", "")
                logger.info(f"  {status} {arm_name}: {message}")
        
        logger.info("")
        logger.info("✓ 远程执行成功！")
        return 0
        
    except Exception as e:
        logger.error(f"执行异常: {e}", exc_info=args.verbose)
        return 1


def test_plan_and_execute_remote_simple(waypoint_names: List[str],
                                       server_host: str = DEFAULT_SERVER_HOST,
                                       waypoints_file: Path = DEFAULT_WAYPOINT_PATH,
                                       timeout: int = 60,
                                       left_arm_executor_host: str = "localhost",
                                       right_arm_executor_host: str = "localhost") -> bool:
    """
    精简版远程执行测试 - 规划需要自己调用
    
    端口根据轨迹中的手臂信息自动选择：
    - left_arm: 5660
    - right_arm: 5661
    
    Args:
        waypoint_names: 路径点名称列表
        server_host: 规划服务器主机地址（端口固定为5605）
        waypoints_file: 路径点文件路径
        timeout: 超时时间（秒）
        left_arm_executor_host: 左臂执行服务器主机地址（默认: localhost）
        right_arm_executor_host: 右臂执行服务器主机地址（默认: localhost）
        
    Returns:
        bool: 是否成功
    """
    # 加载路径点
    with open(waypoints_file, 'r', encoding='utf-8') as f:
        all_waypoints = json.load(f)
    
    # 构建路径点字典
    waypoints = {name: all_waypoints[name] for name in waypoint_names}
    
    # 创建客户端（设置执行服务器主机地址）
    client = UrMoveClient(
        server_host=server_host, 
        timeout_ms=timeout * 1000,
        left_arm_executor_host=left_arm_executor_host,
        right_arm_executor_host=right_arm_executor_host
    )
    
    # 1. 规划轨迹（需要自己调用）
    plan_result = client.plan_trajectory(waypoints, execute=False)
    if not plan_result.get("success", False):
        logger.error(f"轨迹规划失败: {plan_result.get('error', 'Unknown error')}")
        return False
    
    # 2. 远程执行轨迹（服务器主机地址固定为 192.168.1.7，端口根据轨迹中的手臂信息自动选择）
    result = client.execute_remote(
        plan_result=plan_result
    )
    
    return result.get("success", False)


def display_trajectory_info(trajectories: Dict[str, Dict[str, Any]]) -> None:
    """显示轨迹信息"""
    logger.info("")
    logger.info("=" * 60)
    logger.info("轨迹规划结果:")
    logger.info("=" * 60)
    
    total_points = 0
    for group_name, traj in trajectories.items():
        num_points = len(traj.get("points", []))
        total_points += num_points
        joint_names = traj.get("joint_names", [])
        logger.info(f"  {group_name}:")
        logger.info(f"    - 轨迹点数: {num_points}")
        logger.info(f"    - 关节数: {len(joint_names)}")
        if joint_names:
            logger.info(f"    - 关节: {', '.join(joint_names[:3])}{'...' if len(joint_names) > 3 else ''}")
    
    logger.info("")
    logger.info(f"总计: {len(trajectories)} 个组的轨迹，共 {total_points} 个点")
    logger.info("=" * 60)


def main(argv: List[str] = None) -> int:
    """主函数"""
    if argv is None:
        argv = sys.argv[1:]
    
    args = parse_args(argv)
    
    # 设置日志级别
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    logger.info("=" * 60)
    logger.info("UR Move 轨迹规划与执行工具")
    logger.info("=" * 60)
    logger.info("执行模式: %s", args.mode)
    
    # 参数验证
    if args.mode == "arm":
        if not args.names:
            logger.error("arm 模式需要指定 --names 参数")
            return 1
    elif args.mode == "gripper":
        if not args.gripper_action and args.gripper_position is None:
            logger.error("gripper 模式需要指定 --gripper-action 或 --gripper-position")
            return 1
        # 直接执行夹爪控制
        return control_gripper(args)
    elif args.mode == "remote":
        if not args.names:
            logger.error("remote 模式需要指定 --names 参数")
            return 1
        # 执行远程规划执行
        return test_remote_execution(args)
    
    # arm 模式：需要加载路径点
    logger.info("服务器地址: %s", args.server)
    logger.info("路径点文件: %s", args.waypoints_file)
    logger.info("选择路径点: %s", ", ".join(args.names))
    
    if args.no_execute:
        logger.info("执行模式: 只规划，不执行")
    elif args.wait_confirm:
        logger.info("执行模式: 规划完成后等待确认再执行")
    else:
        logger.info("执行模式: 规划完成后自动执行")
    logger.info("=" * 60)
    
    # 检查路径点文件
    if not args.waypoints_file.exists():
        logger.error(f"路径点文件不存在: {args.waypoints_file}")
        return 1
    
    # 加载路径点
    try:
        with open(args.waypoints_file, 'r', encoding='utf-8') as f:
            all_waypoints = json.load(f)
    except Exception as e:
        logger.error(f"加载路径点文件失败: {e}")
        return 1
    
    # 检查路径点名称是否存在
    missing = [name for name in args.names if name not in all_waypoints]
    if missing:
        logger.error(f"未在路径点文件中找到: {', '.join(missing)}")
        logger.info("可用的路径点:")
        for name in sorted(all_waypoints.keys()):
            logger.info(f"  - {name}")
        return 1
    
    # 构建路径点字典（按顺序）
    test_waypoints = {}
    for name in args.names:
        test_waypoints[name] = all_waypoints[name]
    
    # 创建客户端（连接会在首次调用时自动建立）
    # 处理兼容性：如果用户传入了完整地址，提取主机部分
    server_host = args.server
    if server_host.startswith("tcp://"):
        server_host = server_host[7:].split(":")[0]
    elif ":" in server_host:
        server_host = server_host.split(":")[0]
    
    client = UrMoveClient(server_host=server_host, timeout_ms=args.timeout * 1000)
    
    try:
        # 执行规划（根据参数决定是否立即执行）
        # 注意：连接会在 plan_trajectory 内部自动建立
        logger.info("")
        logger.info("正在规划轨迹...")
        start_time = time.time()
        
        # 确定是否立即执行
        should_execute = not args.no_execute and not args.wait_confirm
        result = client.plan_trajectory(test_waypoints, execute=should_execute)
        
        elapsed = time.time() - start_time
        logger.info(f"规划耗时: {elapsed:.2f} 秒")
        
        if not result.get("success", False):
            logger.error(f"✗ 规划失败: {result.get('error', 'Unknown error')}")
            return 1
        
        # 如果只需要规划，直接返回
        if args.no_execute:
            trajectories = result.get("trajectories", {})
            if trajectories:
                display_trajectory_info(trajectories)
            execution_id = result.get("execution_id")
            if execution_id:
                logger.info("")
                logger.info(f"✓ 规划完成（未执行），execution_id: {execution_id}")
            else:
                logger.info("")
                logger.info("✓ 规划完成（未执行）")
            return 0
        
        # 获取轨迹数据（用于显示）
        trajectories = result.get("trajectories", {})
        if trajectories:
            display_trajectory_info(trajectories)
        
        # 如果需要用户确认，使用 execution_id 机制
        if args.wait_confirm:
            execution_id = result.get("execution_id")
            if not execution_id:
                logger.error("✗ 响应中缺少 execution_id 字段")
                return 1
            
            # 用户确认
            num_trajectories = len(trajectories) if trajectories else 1
            if num_trajectories == 1:
                message = "准备执行轨迹..."
            else:
                message = f"准备执行 {num_trajectories} 个轨迹..."
            
            logger.info("")
            logger.info(message)
            user_input = input("是否执行？(y/n): ").strip().lower()
            if user_input not in ['y', 'yes']:
                logger.info("用户取消执行")
                return 0
            
            # 通过 execution_id 执行轨迹（在服务器端执行）
            logger.info("")
            logger.info(f"通过 execution_id 执行轨迹: {execution_id}")
            logger.info("开始执行轨迹...")
            execution_result = client.execute_trajectory(execution_id)
            
            if execution_result.get("success", False):
                logger.info("")
                logger.info("✓ 轨迹执行成功！")
                return 0
            else:
                logger.error("")
                logger.error(f"✗ 轨迹执行失败: {execution_result.get('error', 'Unknown error')}")
                return 1
        
        # auto_execute=True 的情况已经在规划时执行完成
        logger.info("")
        logger.info("✓ 轨迹规划和执行成功！")
        return 0
        
    except Exception as e:
        logger.error(f"执行异常: {e}", exc_info=args.verbose)
        return 1
    # 注意：client 会在析构时自动关闭连接，无需手动调用 close()


if __name__ == "__main__":
    sys.exit(main())

