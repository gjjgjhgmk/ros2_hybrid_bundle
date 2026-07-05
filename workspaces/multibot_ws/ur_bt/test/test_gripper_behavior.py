#!/usr/bin/env python3
"""
测试夹爪控制效果
使用行为树执行夹爪的打开、关闭和位置设置
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


def test_single_gripper_open_close():
    """测试单个夹爪打开和关闭"""
    logger.info("=" * 60)
    logger.info("测试1: 单个夹爪打开和关闭")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        behaviors = [
            # 打开左手夹爪
            bt_manager.gripper_behavior.open("right", name="打开左手夹爪"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            # 关闭左手夹爪
            bt_manager.gripper_behavior.close("right", name="关闭左手夹爪"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            # 再次打开
            bt_manager.gripper_behavior.open("right", name="再次打开左手夹爪"),
        ]
        
        logger.info("开始执行夹爪打开/关闭测试...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 单个夹爪打开/关闭测试成功！")
        else:
            logger.error("❌ 单个夹爪打开/关闭测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def test_gripper_position():
    """测试设置夹爪位置"""
    logger.info("\n" + "=" * 60)
    logger.info("测试2: 设置夹爪位置")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        behaviors = [
            # 设置到不同位置
            bt_manager.gripper_behavior.set_position("left", 0.0, name="完全打开"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            bt_manager.gripper_behavior.set_position("left", 0.4, name="半开"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            bt_manager.gripper_behavior.set_position("left", 0.8, name="完全关闭"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            bt_manager.gripper_behavior.set_position("left", 0.2, name="部分打开"),
        ]
        
        logger.info("开始执行夹爪位置设置测试...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 夹爪位置设置测试成功！")
        else:
            logger.error("❌ 夹爪位置设置测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def test_dual_gripper_control():
    """测试同时控制两个夹爪"""
    logger.info("\n" + "=" * 60)
    logger.info("测试3: 同时控制两个夹爪")
    logger.info("=" * 60)
    
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        behaviors = [
            # 同时打开两个夹爪
            bt_manager.gripper_behavior.open("both", name="同时打开两个夹爪"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            # 同时关闭两个夹爪
            bt_manager.gripper_behavior.close("both", name="同时关闭两个夹爪"),
            bt_manager.utility_behavior.sleep(duration=2.0, name="等待2秒"),
            
            # 同时设置到不同位置
            bt_manager.gripper_behavior.set_position("both", 0.4, name="同时设置到半开"),
        ]
        
        logger.info("开始执行双夹爪控制测试...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 双夹爪控制测试成功！")
        else:
            logger.error("❌ 双夹爪控制测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bt_manager.cleanup()


def main():
    """主函数"""
    print("=" * 60)
    print("夹爪控制测试程序")
    print("=" * 60)
    print("\n请选择要运行的测试:")
    print("1. 单个夹爪打开和关闭")
    print("2. 设置夹爪位置")
    print("3. 同时控制两个夹爪")
    print("4. 执行所有测试")
    print("0. 退出")
    
    try:
        choice = input("\n请输入选项 (0-4): ").strip()
        
        if choice == "1":
            test_single_gripper_open_close()
        elif choice == "2":
            test_gripper_position()
        elif choice == "3":
            test_dual_gripper_control()
        elif choice == "4":
            print("开始执行所有测试")
            test_single_gripper_open_close()
            test_gripper_position()
            test_dual_gripper_control()
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

