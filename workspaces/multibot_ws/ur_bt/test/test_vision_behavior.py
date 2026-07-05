#!/usr/bin/env python3
"""
测试视觉检测行为
测试基于掩码和Box模板匹配的位姿估计功能
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


def test_vision_pose_estimation():
    """测试视觉位姿估计"""
    logger.info("=" * 60)
    logger.info("测试: 视觉位姿估计（掩码方法）")
    logger.info("=" * 60)
    
    # 创建行为树管理器
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        # 创建视觉位姿估计行为
        vision_behavior = bt_manager.vision_behavior.pose_estimation_mask(
            camera_name="right_camera",
            target_frame="right_interface_link",
            name="视觉位姿估计测试"
        )
        
        # 执行行为树
        behaviors = [vision_behavior]
        logger.info("开始执行视觉位姿估计...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 视觉位姿估计测试成功！")
            
            # 从黑板读取结果
            try:
                vision_results = bt_manager.blackboard_manager.get("vision_results")
                if vision_results:
                    logger.info(f"检测到 {vision_results.get('detection_count', 0)} 个对象")
                    detections = vision_results.get('detections', [])
                    for i, detection in enumerate(detections):
                        pose = detection.get('pose', [])
                        logger.info(f"  对象 {i+1}: 类别={detection.get('category', 'unknown')}, "
                                  f"置信度={detection.get('confidence', 0.0):.3f}, "
                                  f"位置=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
                else:
                    logger.warning("未找到视觉检测结果")
            except Exception as e:
                logger.warning(f"读取视觉结果时出错: {e}")
        else:
            logger.error("❌ 视觉位姿估计测试失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        bt_manager.cleanup()


def test_vision_pose_estimation_box():
    """测试视觉位姿估计（Box方法）"""
    logger.info("=" * 60)
    logger.info("测试: 视觉位姿估计（Box模板匹配方法）")
    logger.info("=" * 60)
    
    # 创建行为树管理器
    bt_manager = BehaviorTreeManager(
        config_path=os.path.join(os.path.dirname(__file__), '..', 'config.yaml'),
        waypoints_path=os.path.join(os.path.dirname(__file__), 'waypoints.json'),
        show_progress=True,
        show_tree=False
    )
    
    try:
        # 创建视觉位姿估计行为（使用Box方法）
        vision_behavior = bt_manager.vision_behavior.pose_estimation_box(
            camera_name="right_camera",
            target_frame="right_interface_link",
            min_score=0.8,
            max_num=10,
            name="视觉位姿估计测试（Box）"
        )
        
        # 执行行为树
        behaviors = [vision_behavior]
        logger.info("开始执行视觉位姿估计（Box方法）...")
        success = bt_manager.execute(behaviors, wait=True)
        
        if success:
            logger.info("✅ 视觉位姿估计测试（Box方法）成功！")
            
            # 从黑板读取结果
            try:
                vision_results = bt_manager.blackboard_manager.get("vision_results")
                if vision_results:
                    logger.info(f"检测到 {vision_results.get('detection_count', 0)} 个对象")
                    detections = vision_results.get('detections', [])
                    for i, detection in enumerate(detections):
                        pose = detection.get('pose', [])
                        logger.info(f"  对象 {i+1}: 类别={detection.get('category', 'unknown')}, "
                                  f"置信度={detection.get('confidence', 0.0):.3f}, "
                                  f"位置=({pose[0]:.3f}, {pose[1]:.3f}, {pose[2]:.3f})")
                else:
                    logger.warning("未找到视觉检测结果")
            except Exception as e:
                logger.warning(f"读取视觉结果时出错: {e}")
        else:
            logger.error("❌ 视觉位姿估计测试（Box方法）失败")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理资源
        bt_manager.cleanup()


def main():
    """主函数"""
    print("=" * 60)
    print("视觉检测行为测试程序")
    print("=" * 60)
    
    try:
        # 测试掩码方法
        # test_vision_pose_estimation()
        
        print("\n")
        
        # 测试Box方法
        test_vision_pose_estimation_box()
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

