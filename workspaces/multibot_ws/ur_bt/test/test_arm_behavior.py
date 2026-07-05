#!/usr/bin/env python3
"""
测试路径点运动效果
使用行为树执行单个或多个路径点的运动
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


def test_single_waypoint():
    """测试单个路径点运动"""
    logger.info("=" * 60)
    logger.info("测试1: 单个路径点运动")
    logger.info("=" * 60)
    
    # 1. 创建行为树管理器
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        # 2. 创建移动到单个路径点的行为
        # waypoint_configs 格式: [(waypoint_name, vel_scale, acc_scale), ...]
        move_behavior = bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("左臂-home", 0.1, 0.1)],  # 路径点名称，速度缩放，加速度缩放
            name="移动到home位置"
        )
        
        # 3. 执行行为树
        behaviors = [move_behavior]
        logger.info("开始执行路径点运动...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 路径点运动测试成功！")
        else:
            logger.error("❌ 路径点运动测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        bt_manager.cleanup()


def test_multiple_waypoints():
    """测试多个路径点序列运动"""
    logger.info("\n" + "=" * 60)
    logger.info("测试2: 多个路径点序列运动")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        # 创建多个路径点的序列运动
        behaviors = [
            # 移动到第一个位置
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-测试1", 0.1, 0.1)],
                name="运动到测试1"
            ),
            
            # 添加延时
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待1秒"),
            
            # 移动到第二个位置
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-测试2", 0.1, 0.1)],
                name="运动到测试2"
            ),

            # 添加延时
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待1秒"),

            # 返回第一个位置
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-测试1", 0.1, 0.1)],
                name="运动到测试1"
            ),
        ]
        
        # 执行行为树
        logger.info("开始执行多路径点序列运动...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 多路径点序列运动测试成功！")
        else:
            logger.error("❌ 多路径点序列运动测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def test_dual_arm_movement():
    """测试双臂同时运动"""
    logger.info("\n" + "=" * 60)
    logger.info("测试3: 双臂同时运动")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        # 双臂同时移动到不同位置
        move_behavior = bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[
                ("左臂-home", 0.1, 0.1),      # 左臂
                ("右臂-home", 0.1, 0.1)      # 右臂
            ],
            name="双臂同时移动到home"
        )
        
        behaviors = [move_behavior]
        logger.info("开始执行双臂同时运动...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 双臂同时运动测试成功！")
        else:
            logger.error("❌ 双臂同时运动测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def test_with_different_speeds():
    """测试不同速度的路径点运动"""
    logger.info("\n" + "=" * 60)
    logger.info("测试4: 不同速度的路径点运动")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        behaviors = [
            # 慢速运动
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-测试3", 0.05, 0.05)],  # 5%速度
                name="慢速运动"
            ),
            
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 中速运动
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-home", 0.1, 0.1)],  # 10%速度
                name="中速运动"
            ),
            
            bt_manager.utility_behavior.sleep(duration=1.0, name="等待"),
            
            # 快速运动
            bt_manager.arm_move_behavior.move_to_waypoints(
                waypoint_configs=[("左臂-测试3", 0.2, 0.2)],  # 20%速度
                name="快速运动"
            ),
        ]
        
        logger.info("开始执行不同速度测试...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 不同速度测试成功！")
        else:
            logger.error("❌ 不同速度测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def main():
    """主函数"""
    print("=" * 60)
    print("路径点运动测试程序")
    print("=" * 60)
    print("\n请选择要运行的测试:")
    print("1. 单个路径点运动")
    print("2. 多个路径点序列运动")
    print("3. 双臂同时运动")
    print("4. 不同速度测试")
    print("5. 执行所有测试")
    print("0. 退出")
    
    try:
        choice = input("\n请输入选项 (0-5): ").strip()
        
        if choice == "1":
            test_single_waypoint()
        elif choice == "2":
            test_multiple_waypoints()
        elif choice == "3":
            test_dual_arm_movement()
        elif choice == "4":
            test_with_different_speeds()
        elif choice == "5":
            print("开始执行所有测试")
            test_single_waypoint()
            test_multiple_waypoints()
            test_dual_arm_movement()
            test_with_different_speeds()
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

