#!/usr/bin/env python3
"""
测试 TF 服务功能
测试坐标变换查询服务
"""

import sys
import os
import logging
import time

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ur_bt.clients.tf_client import TFClient

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def test_basic_transform():
    """测试基本的坐标变换查询"""
    logger.info("=" * 60)
    logger.info("测试1: 基本坐标变换查询")
    logger.info("=" * 60)
    
    try:
        # 创建 TF 客户端
        logger.info("正在连接 TF 服务器...")
        tf_client = TFClient(server_ip="localhost", server_port=5609, timeout=5)
        
        # 测试一些常见的坐标系变换
        # 注意：这些坐标系名称需要根据实际的 TF 树来调整
        test_cases = [
            ("base_link", "left_base_link"),  # 左臂基座到基础坐标系
            ("base_link", "right_base_link"),  # 右臂基座到基础坐标系
            ("base_link", "left_ee_link"),  # 左臂末端执行器到基础坐标系
            ("base_link", "right_ee_link"),  # 右臂末端执行器到基础坐标系
        ]
        
        success_count = 0
        for source_frame, target_frame in test_cases:
            logger.info(f"\n查询变换: {source_frame} -> {target_frame}")
            response = tf_client.lookup_transform(source_frame, target_frame)
            
            if response and response.get('success'):
                data = response.get('data', {})
                translation = data.get('translation', {})
                rotation = data.get('rotation', {})
                
                logger.info(f"✅ 成功获取变换:")
                logger.info(f"   平移: x={translation.get('x', 0):.4f}, "
                          f"y={translation.get('y', 0):.4f}, "
                          f"z={translation.get('z', 0):.4f}")
                logger.info(f"   旋转: x={rotation.get('x', 0):.4f}, "
                          f"y={rotation.get('y', 0):.4f}, "
                          f"z={rotation.get('z', 0):.4f}, "
                          f"w={rotation.get('w', 0):.4f}")
                success_count += 1
            else:
                error_msg = response.get('message', 'Unknown error') if response else 'No response'
                logger.warning(f"❌ 查询失败: {error_msg}")
        
        logger.info(f"\n测试结果: {success_count}/{len(test_cases)} 成功")
        
        # 关闭连接
        tf_client.close()
        
        if success_count > 0:
            logger.info("✅ 基本坐标变换测试完成")
        else:
            logger.warning("⚠️ 所有查询都失败，请检查 TF 服务器和坐标系名称")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()


def test_invalid_frames():
    """测试无效坐标系处理"""
    logger.info("\n" + "=" * 60)
    logger.info("测试3: 无效坐标系处理")
    logger.info("=" * 60)
    
    try:
        tf_client = TFClient(server_ip="localhost", server_port=5609, timeout=5)
        
        # 测试不存在的坐标系
        invalid_cases = [
            ("invalid_frame_1", "invalid_frame_2"),
            ("base_link", "nonexistent_frame"),
            ("nonexistent_frame", "base_link"),
        ]
        
        for source_frame, target_frame in invalid_cases:
            logger.info(f"\n查询无效变换: {source_frame} -> {target_frame}")
            response = tf_client.lookup_transform(source_frame, target_frame)
            
            if response and not response.get('success'):
                logger.info(f"✅ 正确处理错误: {response.get('message', 'Unknown error')}")
            elif response and response.get('success'):
                logger.warning(f"⚠️ 意外成功: 应该失败的查询却成功了")
            else:
                logger.warning(f"⚠️ 无响应")
        
        tf_client.close()
        logger.info("✅ 无效坐标系处理测试完成")
        
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()


def test_connection_timeout():
    """测试连接超时处理"""
    logger.info("\n" + "=" * 60)
    logger.info("测试4: 连接超时处理")
    logger.info("=" * 60)
    
    try:
        # 尝试连接到不存在的服务器
        logger.info("尝试连接到不存在的服务器 (localhost:9999)...")
        tf_client = TFClient(server_ip="localhost", server_port=9999, timeout=2)
        
        # 尝试查询（应该超时）
        response = tf_client.lookup_transform("base_link", "world")
        
        if response is None:
            logger.info("✅ 正确处理超时（返回 None）")
        else:
            logger.warning(f"⚠️ 意外收到响应: {response}")
        
        tf_client.close()
        
    except Exception as e:
        logger.info(f"✅ 正确处理连接错误: {e}")


def test_multiple_queries():
    """测试多次连续查询"""
    logger.info("\n" + "=" * 60)
    logger.info("测试5: 多次连续查询")
    logger.info("=" * 60)
    
    try:
        tf_client = TFClient(server_ip="localhost", server_port=5609, timeout=5)
        
        source_frame = "base_link"
        target_frame = "left_base_link"
        
        logger.info(f"连续查询变换: {source_frame} -> {target_frame}")
        
        success_count = 0
        for i in range(5):
            response = tf_client.lookup_transform(source_frame, target_frame)
            if response and response.get('success'):
                success_count += 1
                logger.info(f"  查询 {i+1}/5: ✅ 成功")
            else:
                logger.warning(f"  查询 {i+1}/5: ❌ 失败")
            time.sleep(0.1)  # 短暂延时
        
        tf_client.close()
        
        if success_count == 5:
            logger.info("✅ 多次连续查询测试成功")
        else:
            logger.warning(f"⚠️ 部分查询失败: {success_count}/5 成功")
            
    except Exception as e:
        logger.error(f"测试异常: {e}")
        import traceback
        traceback.print_exc()


def main():
    """主函数"""
    print("=" * 60)
    print("TF 服务测试程序")
    print("=" * 60)
    print("\n请确保 TF ZMQ 服务器正在运行 (端口 5609)")
    print("\n请选择要运行的测试:")
    print("1. 基本坐标变换查询")
    print("2. 无效坐标系处理")
    print("3. 连接超时处理")
    print("4. 多次连续查询")
    print("5. 执行所有测试")
    print("0. 退出")
    
    try:
        choice = input("\n请输入选项 (0-5): ").strip()
        
        if choice == "1":
            test_basic_transform()

        elif choice == "2":
            test_invalid_frames()
        elif choice == "3":
            test_connection_timeout()
        elif choice == "4":
            test_multiple_queries()
        elif choice == "5":
            print("开始执行所有测试")
            test_basic_transform()
            test_invalid_frames()
            test_connection_timeout()
            test_multiple_queries()
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

