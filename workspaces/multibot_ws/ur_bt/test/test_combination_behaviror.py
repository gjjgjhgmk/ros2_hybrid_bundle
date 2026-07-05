#!/usr/bin/env python3
"""
测试手臂和夹爪联合运动
使用行为树执行手臂移动和夹爪控制的组合操作
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


def test_arm_and_gripper_behavior():
    """测试手臂和夹爪联合动作"""
    logger.info("=" * 60)
    logger.info("测试1: 手臂和夹爪联合动作")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        behaviors = [
            # 初始位置：移动到home并打开夹爪
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-home", 0.1, 0.1)],
                name="移动到初始位置"
            ),
            bt_manager.gripper_behavior.open("left", name="打开夹爪"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待"),
            
            # 抓取阶段
            # 1. 移动到预抓取位置（快速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-预抓取", 0.1, 0.1)],
                name="移动到预抓取位置"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 2. 移动到抓取位置（慢速精确）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-抓取", 0.05, 0.05)],
                name="慢速移动到抓取位置"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),

            # 3. 关闭夹爪（抓取）
            bt_manager.gripper_behavior.close("left", name="关闭夹爪（抓取）"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待抓取完成"),
            
            # 4. 移动到抓取撤退位置（慢速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-抓取撤退", 0.05, 0.05)],
                name="慢速撤退"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 放置阶段
            # 5. 移动到预放置位置（快速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-预放置", 0.1, 0.1)],
                name="移动到预放置位置"
            ),
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 6. 移动到放置位置（慢速精确）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-放置", 0.05, 0.05)],
                name="慢速移动到放置位置"
            ),
            
            # 7. 打开夹爪（释放）
            bt_manager.gripper_behavior.open("left", name="打开夹爪（释放）"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待释放完成"),
            
            # 8. 移动到放置撤退位置（慢速）
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-放置撤退", 0.05, 0.05)],
                name="慢速撤退"
            ),
            bt_manager.utility_behavior.sleep(duration=1, name="等待"),
            
            # 返回初始位置
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-home", 0.1, 0.1)],
                name="返回初始位置"
            ),
        ]
        
        logger.info("开始执行手臂和夹爪联合动作测试...")
        logger.info("流程：预抓取 → 抓取 → 抓取撤退 → 预放置 → 放置 → 放置撤退 → 返回")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 手臂和夹爪联合动作测试成功！")
        else:
            logger.error("❌ 手臂和夹爪联合动作测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def main():
    """主函数"""
    print("=" * 60)
    print("手臂和夹爪联合动作测试程序")
    print("=" * 60)
    print("\n请选择操作:")
    print("1. 执行手臂和夹爪联合动作测试")
    print("0. 退出")
    
    try:
        choice = input("\n请输入选项 (0-1): ").strip()
        
        if choice == "1":
            test_arm_and_gripper_behavior()
        elif choice == "0":
            print("退出测试")
            return
        else:
            print("无效选项，请重新运行程序")
            return
    except KeyboardInterrupt:
        print("\n\n用户中断测试")
    except Exception as e:
        print(f"\n测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)


if __name__ == "__main__":
    main()

