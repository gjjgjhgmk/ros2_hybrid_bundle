#!/usr/bin/env python3
"""
启动配置文件REST API服务
"""

import os
import sys
import signal
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
        description="启动配置文件REST API服务",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  python start_config_rest_api.py
  python start_config_rest_api.py --config-root config --host 127.0.0.1 --port 7002
        """,
    )

    parser.add_argument(
        "--config-root",
        default=None,
        help="配置根目录路径（默认: 项目根目录下的config目录）",
    )

    parser.add_argument("--host", default="0.0.0.0", help="服务器监听地址")

    parser.add_argument("--port", "-p", type=int, default=7002, help="服务器监听端口")

    parser.add_argument("--debug", action="store_true", help="启用调试模式")

    return parser.parse_args()


# 全局服务器实例（用于信号处理）
_server_instance = None


def signal_handler(signum, frame):
    """信号处理器"""
    global _server_instance
    logger.info(f"\n收到信号 {signum}，正在关闭服务...")
    sys.exit(0)


def main():
    """主函数"""
    # 解析命令行参数
    args = parse_args()

    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 导入并启动服务
    try:
        from config_rest_api import ConfigRESTAPI

        logger.info(f"正在启动配置文件REST API服务...")
        if args.config_root:
            logger.info(f"配置根目录: {args.config_root}")
        logger.info(f"监听地址: {args.host}:{args.port}")

        # 创建并运行API服务器
        global _server_instance
        server = ConfigRESTAPI(config_root=args.config_root, host=args.host, port=args.port)
        _server_instance = server

        print("\n" + "=" * 60)
        print("🚀 配置文件REST API服务已启动")
        print("=" * 60)
        print(f"📁 配置根目录: {server.config_root}")
        print(f"🌐 监听地址: {server.host}:{server.port}")
        print(f"🔧 远程配置管理: http://localhost:{server.port}/")
        print(f"📖 配置管理页面: http://localhost:{server.port}/config-manager")
        print(f"🚀 远程启动管理: http://localhost:{server.port}/remote-start")
        print(f"🔍 健康检查: http://localhost:{server.port}/api/health")
        print("=" * 60)
        print("💡 主要功能:")
        print("  • 浏览配置目录结构")
        print("  • 查看和编辑配置文件")
        print("  • 保存配置文件")
        print("  • 远程启动管理")
        print("=" * 60)
        print("按 Ctrl+C 停止服务")
        print("=" * 60 + "\n")

        server.run(debug=args.debug)

    except KeyboardInterrupt:
        logger.info("收到中断信号，正在关闭服务...")
    except Exception as e:
        logger.error(f"启动服务失败: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
