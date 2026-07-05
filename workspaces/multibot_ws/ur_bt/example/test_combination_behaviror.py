#!/usr/bin/env python3
"""
抓取和放置演示程序 (Pick and Place Demo)
==========================================

这是一个完整的抓取和放置任务演示，展示了如何使用行为树框架控制机械臂和夹爪
完成复杂的协作任务。

演示流程：
1. 移动到初始位置并打开夹爪
2. 移动到预抓取位置
3. 慢速移动到抓取位置
4. 关闭夹爪抓取物体
5. 慢速撤退
6. 移动到预放置位置
7. 慢速移动到放置位置
8. 打开夹爪释放物体
9. 慢速撤退
10. 返回初始位置

使用说明：
- 确保 ZMQ 服务器正在运行（ur_move_server）
- 确保机器人已正确连接并处于安全状态
- 确保配置文件 config.yaml 和 waypoints.json 在 example 目录下
"""

import sys
import os
import logging
import time

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ur_bt import BehaviorTreeManager

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def demo_pick_and_place():
    """
    抓取和放置演示
    
    执行完整的抓取和放置任务序列，展示行为树框架的协作能力。
    """
    print("\n" + "=" * 70)
    print("🤖 抓取和放置演示 (Pick and Place Demo)")
    print("=" * 70)
    print("\n📋 演示流程：")
    print("  1. 移动到初始位置并打开夹爪")
    print("  2. 移动到预抓取位置（快速）")
    print("  3. 慢速移动到抓取位置（精确）")
    print("  4. 关闭夹爪抓取物体")
    print("  5. 慢速撤退")
    print("  6. 移动到预放置位置（快速）")
    print("  7. 慢速移动到放置位置（精确）")
    print("  8. 打开夹爪释放物体")
    print("  9. 慢速撤退")
    print("  10. 返回初始位置")
    print("\n⚠️  注意事项：")
    print("  - 确保机器人周围有足够的空间")
    print("  - 确保抓取和放置位置已正确标定")
    print("  - 演示过程中请保持安全距离")
    print("\n" + "-" * 70)
    
    # 获取当前脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config.yaml')
    waypoints_path = os.path.join(script_dir, 'waypoints.json')
    
    # 检查配置文件是否存在
    if not os.path.exists(config_path):
        logger.error(f"❌ 配置文件不存在: {config_path}")
        print(f"\n❌ 错误: 找不到配置文件 {config_path}")
        print("   请确保 config.yaml 文件在 example 目录下")
        return False
    
    if not os.path.exists(waypoints_path):
        logger.error(f"❌ 路径点文件不存在: {waypoints_path}")
        print(f"\n❌ 错误: 找不到路径点文件 {waypoints_path}")
        print("   请确保 waypoints.json 文件在 example 目录下")
        return False
    
    logger.info("=" * 70)
    logger.info("开始抓取和放置演示")
    logger.info("=" * 70)
    
    bt_manager = BehaviorTreeManager(
        config_path=config_path,
        waypoints_path=waypoints_path,
        show_progress=True,
        show_tree=False
    )
    
    try:
        behaviors = [
            # 初始位置：移动到home并打开夹爪
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-home", 0.1, 0.1)],
                name="移动到初始位置"
            ),
            bt_manager.gripper_behavior.open("left", name="打开夹爪"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待"),
            
            # 抓取阶段
            # 1. 移动到预抓取位置（快速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-预抓取", 0.1, 0.1)],
                name="移动到预抓取位置"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 2. 移动到抓取位置（慢速精确）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-抓取", 0.05, 0.05)],
                name="慢速移动到抓取位置"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),

            # 3. 关闭夹爪（抓取）
            bt_manager.gripper_behavior.close("left", name="关闭夹爪（抓取）"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待抓取完成"),
            
            # 4. 移动到抓取撤退位置（慢速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-抓取撤退", 0.05, 0.05)],
                name="慢速撤退"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 放置阶段
            # 5. 移动到预放置位置（快速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-预放置", 0.1, 0.1)],
                name="移动到预放置位置"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 6. 移动到放置位置（慢速精确）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-放置", 0.05, 0.05)],
                name="慢速移动到放置位置"
            ),
            
            # 7. 打开夹爪（释放）
            bt_manager.gripper_behavior.open("left", name="打开夹爪（释放）"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待释放完成"),
            
            # 8. 移动到放置撤退位置（慢速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-放置撤退", 0.05, 0.05)],
                name="慢速撤退"
            ),
            bt_manager.utility_behavior.sleep(duration=1, name="等待"),
            
            # 返回初始位置
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("右臂-home", 0.1, 0.1)],
                name="返回初始位置"
            ),
        ]
        
        print("\n🚀 开始执行演示...")
        print("   预计耗时: 约 2-3 分钟（取决于机器人速度）")
        print("\n" + "-" * 70 + "\n")
        
        logger.info("开始执行抓取和放置演示...")
        logger.info("流程：预抓取 → 抓取 → 抓取撤退 → 预放置 → 放置 → 放置撤退 → 返回")
        
        success = bt_manager.execute(behaviors, wait=True)
        
        print("\n" + "=" * 70)
        if success:
            logger.info("✅ 抓取和放置演示成功完成！")
            print("✅ 演示成功完成！")
            print("\n📊 执行结果：")
            print("  - 所有动作已成功执行")
            print("  - 机器人已返回初始位置")
            print("  - 任务序列完成")
        else:
            logger.error("❌ 抓取和放置演示失败")
            print("❌ 演示执行失败")
            print("\n⚠️  可能的原因：")
            print("  - ZMQ 服务器未运行或连接失败")
            print("  - 机器人硬件故障或通信中断")
            print("  - 路径点配置错误或不可达")
            print("  - 执行超时")
        print("=" * 70 + "\n")
        
        return success
            
    except KeyboardInterrupt:
        logger.warning("用户中断演示")
        print("\n\n⚠️  演示被用户中断")
        print("   正在清理资源...")
        return False
    except Exception as e:
        logger.error(f"演示异常: {e}")
        print(f"\n❌ 演示过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        bt_manager.cleanup()
        logger.info("资源清理完成")


def print_banner():
    """打印欢迎横幅"""
    banner = """
    ╔══════════════════════════════════════════════════════════════╗
    ║                                                              ║
    ║        🤖 抓取和放置演示 (Pick and Place Demo) 🤖            ║
    ║                                                              ║
    ║  使用行为树框架控制机械臂和夹爪完成协作任务                   ║
    ║                                                              ║
    ╚══════════════════════════════════════════════════════════════╝
    """
    print(banner)


def check_prerequisites():
    """检查运行前提条件"""
    print("\n🔍 检查运行环境...")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, 'config.yaml')
    waypoints_path = os.path.join(script_dir, 'waypoints.json')
    
    issues = []
    
    if not os.path.exists(config_path):
        issues.append(f"  ❌ 配置文件不存在: {config_path}")
    else:
        print(f"  ✅ 配置文件: {config_path}")
    
    if not os.path.exists(waypoints_path):
        issues.append(f"  ❌ 路径点文件不存在: {waypoints_path}")
    else:
        print(f"  ✅ 路径点文件: {waypoints_path}")
    
    if issues:
        print("\n⚠️  发现问题：")
        for issue in issues:
            print(issue)
        print("\n请确保所有必需文件都在 example 目录下")
        return False
    
    print("  ✅ 环境检查通过\n")
    return True


def main():
    """主函数"""
    print_banner()
    
    print("\n📖 使用说明：")
    print("  这是一个完整的抓取和放置任务演示程序")
    print("  演示将执行从抓取位置到放置位置的完整任务序列")
    print("\n⚠️  运行前请确保：")
    print("  1. ZMQ 服务器正在运行（ros2 launch ur_move ur_move_server.launch.py）")
    print("  2. 机器人已正确连接并处于安全状态")
    print("  3. 抓取和放置位置已正确标定")
    print("  4. 机器人周围有足够的操作空间")
    
    # 检查前提条件
    if not check_prerequisites():
        print("\n❌ 环境检查失败，请修复问题后重试")
        return
    
    print("\n" + "=" * 70)
    print("请选择操作:")
    print("  1. 执行抓取和放置演示")
    print("  0. 退出")
    print("=" * 70)
    
    try:
        choice = input("\n请输入选项 (0-1): ").strip()
        
        if choice == "1":
            print("\n" + "=" * 70)
            print("⚠️  准备开始演示")
            print("=" * 70)
            confirm = input("\n确认开始演示？(y/n): ").strip().lower()
            
            if confirm in ['y', 'yes', '是']:
                demo_pick_and_place()
            else:
                print("\n演示已取消")
        elif choice == "0":
            print("\n退出程序")
            return
        else:
            print("\n❌ 无效选项，请重新运行程序")
            return
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断程序")
    except Exception as e:
        print(f"\n❌ 程序执行错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 70)
    print("程序结束")
    print("=" * 70)


if __name__ == "__main__":
    main()

