#!/usr/bin/env python3
"""
启动集成REST API服务
"""

import os
import sys
import signal
import time
import argparse
from pathlib import Path
from loguru import logger

# 导入公共工具模块
from common_utils import setup_logger, check_docker_environment

# 配置loguru日志格式
setup_logger(check_docker_environment())


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(
        description="启动集成REST API服务",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python start_rest_api.py
  python start_rest_api.py --config config/custom_config.yaml
  python start_rest_api.py --config config/docker/config.yaml --host 127.0.0.1 --port 9090
        """,
    )

    parser.add_argument(
        "--config",
        "-c",
        default="config/docker/config_record.yaml",
        help="配置文件路径 (默认: config/docker/config_record.yaml)",
    )

    parser.add_argument("--host", default=None, help="服务器监听地址 (覆盖配置文件中的设置)")

    parser.add_argument("--port", "-p", type=int, default=None, help="服务器监听端口 (覆盖配置文件中的设置)")

    return parser.parse_args()


# 全局服务器实例（用于信号处理）
_server_instance = None


def signal_handler(signum, frame):
    """信号处理器"""
    global _server_instance
    logger.info(f"\n收到信号 {signum}，正在关闭服务...")

    # 清理容器管理器资源
    if _server_instance and hasattr(_server_instance, "container_manager"):
        try:
            logger.info("正在清理容器管理器资源...")
            _server_instance.container_manager.cleanup_all()
            logger.info("资源清理完成")
        except Exception as e:
            logger.error(f"清理资源时发生错误: {e}")

    sys.exit(0)


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_args()

    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 检查配置文件
    config_path = args.config
    if not os.path.exists(config_path):
        logger.error(f"配置文件不存在: {config_path}")
        sys.exit(1)

    # 导入并启动服务
    try:
        from rest_api import IntegratedRESTAPI

        logger.info(f"正在启动集成REST API服务...")
        logger.info(f"配置文件: {config_path}")
        if args.host:
            logger.info(f"命令行覆盖主机: {args.host}")
        if args.port:
            logger.info(f"命令行覆盖端口: {args.port}")

        # 创建并运行API服务器
        global _server_instance
        server = IntegratedRESTAPI(config_path=config_path, host=args.host, port=args.port)
        _server_instance = server

        print("\n" + "=" * 60)
        print("🚀 集成REST API服务已启动")
        print("=" * 60)
        print(f"📁 配置文件: {config_path}")
        print(f"🌐 监听地址: {server.host}:{server.port}")
        print(f"📖 API文档: http://localhost:{server.port}/")
        print(f"📱 Web监控: http://localhost:{server.port}/monitor")
        print(f"🔍 健康检查: http://localhost:{server.port}/api/health")
        print("=" * 60)
        print("💡 主要功能:")
        print("  • 容器管理 (启动/停止/删除)")
        print("  • 程序管理 (启动/停止)")
        print("  • 实时日志查看")
        print("  • 日志文件下载")
        print("  • 系统状态监控")
        print("=" * 60)
        print("按 Ctrl+C 停止服务")
        print("=" * 60 + "\n")

        server.run(debug=False)

    except KeyboardInterrupt:
        logger.info("收到中断信号，正在关闭服务...")
        # 确保清理资源
        if _server_instance and hasattr(_server_instance, "container_manager"):
            try:
                logger.info("正在清理容器管理器资源...")
                _server_instance.container_manager.cleanup_all()
                logger.info("资源清理完成")
            except Exception as e:
                logger.error(f"清理资源时发生错误: {e}")
    except Exception as e:
        logger.error(f"启动服务失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
