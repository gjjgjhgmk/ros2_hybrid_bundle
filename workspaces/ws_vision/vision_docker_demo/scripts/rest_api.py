#!/usr/bin/env python3
"""
集成REST API服务器
提供容器管理和实时日志查看功能
"""

import os
import sys
import json
import re
import threading
import yaml
from datetime import datetime
from pathlib import Path
from flask import Flask, jsonify, request, Response, send_file
from flask_cors import CORS
from loguru import logger
from container_manager import ContainerManager
from log_manager import get_log_manager

# 导入公共工具模块
from common_utils import setup_logger, check_docker_environment, jsonify_chinese, APIResponseBuilder

# 配置loguru日志格式
setup_logger(check_docker_environment())


def clean_ansi_escape(text: str) -> str:
    """清理ANSI转义序列"""
    # 更全面的ANSI转义序列清理
    ansi_escape = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
    # 只清理危险的控制字符，保留制表符(0x09)、换行符(0x0A)、回车符(0x0D)
    control_chars = re.compile(r"[\x00-\x08\x0B-\x0C\x0E-\x1F\x7F]")
    # 清理多余的空白字符
    whitespace = re.compile(r"[ \t]+")  # 只合并空格和制表符，保留换行

    # 逐步清理
    text = ansi_escape.sub("", text)
    text = control_chars.sub("", text)  # 不清理0x0A(换行)和0x0D(回车)
    text = whitespace.sub(" ", text)

    return text.strip()


class IntegratedRESTAPI:
    """集成REST API服务器类"""

    def __init__(self, config_path: str, host: str = None, port: int = None):
        """
        初始化集成API服务器

        Args:
            config_path: 配置文件路径
            host: 服务器地址（可选，优先使用配置文件中的设置）
            port: 服务器端口（可选，优先使用配置文件中的设置）
        """
        self.config_path = config_path
        self.config = self._load_config()

        # 优先使用配置文件中的设置，命令行参数作为覆盖
        config_rest_api = self.config.get("rest_api", {})

        # 检查是否启用
        if not config_rest_api.get("enabled", True):
            logger.warning("REST API在配置中被禁用，但服务仍将启动")

        # 设置主机和端口（配置文件优先，命令行参数覆盖）
        self.host = host or config_rest_api.get("host", "0.0.0.0")
        self.port = port or config_rest_api.get("port", 8080)

        logger.info(f"配置加载完成:")
        logger.info(f"  配置文件: {config_path}")
        logger.info(f"  REST API启用: {config_rest_api.get('enabled', True)}")
        logger.info(f"  监听地址: {self.host}:{self.port}")

        # 初始化组件
        self.log_manager = get_log_manager()
        self.container_manager = ContainerManager(config_path, self.log_manager)
        self.container_manager_thread = threading.Thread(target=self.container_manager.run, daemon=True)
        self.container_manager_thread.start()

        # 创建Flask应用
        self.app = Flask(__name__)
        self.app.config["JSON_AS_ASCII"] = False
        CORS(self.app)  # 启用CORS支持

        # 注册路由
        self._register_routes()

        logger.info(f"集成REST API服务器初始化完成: {self.host}:{self.port}")

    def _load_config(self) -> dict:
        """加载配置文件"""
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
                logger.info(f"成功加载配置文件: {self.config_path}")
                return config or {}
        except FileNotFoundError:
            logger.error(f"配置文件不存在: {self.config_path}")
            return {}
        except yaml.YAMLError as e:
            logger.error(f"配置文件格式错误: {e}")
            return {}
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            return {}

    def _build_containers_from_config(self):
        """从配置文件构建容器信息，避免扫描日志文件"""
        containers = []
        try:
            for container_key, container_config in self.container_manager.config["containers"].items():
                commands = [cmd["name"] for cmd in container_config["commands"]]
                containers.append({"key": container_key, "commands": sorted(commands)})
        except Exception as e:
            logger.error(f"构建容器信息失败: {e}")
        return containers

    def _register_routes(self):
        """注册API路由"""

        # 根路径 - 提供监控界面
        @self.app.route("/", methods=["GET"])
        def index():
            """提供监控界面"""
            try:
                # 读取HTML文件
                html_path = Path(__file__).parent.parent / "html" / "vision_container.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "监控页面未找到", 404
            except Exception as e:
                logger.error(f"提供监控页面失败: {e}")
                return f"错误: {e}", 500

        # 管理器页面
        @self.app.route("/manager", methods=["GET"])
        def manager():
            """提供管理器页面"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "manager.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "管理器页面未找到", 404
            except Exception as e:
                logger.error(f"提供管理器页面失败: {e}")
                return f"错误: {e}", 500

        # 监视器页面
        @self.app.route("/monitor", methods=["GET"])
        def monitor():
            """提供监视器页面"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "monitor.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "监视器页面未找到", 404
            except Exception as e:
                logger.error(f"提供监视器页面失败: {e}")
                return f"错误: {e}", 500

        # 图像查看器页面
        @self.app.route("/image_viewer_module.html", methods=["GET"])
        def image_viewer():
            """提供图像查看器页面"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "image_viewer_module.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "图像查看器页面未找到", 404
            except Exception as e:
                logger.error(f"提供图像查看器页面失败: {e}")
                return f"错误: {e}", 500

        # 健康检查
        @self.app.route("/api/health", methods=["GET"])
        def health():
            """健康检查接口"""
            return jsonify_chinese(
                {
                    "status": "healthy",
                    "timestamp": datetime.now().isoformat(),
                    "service": "integrated-rest-api",
                    "version": "1.0.0",
                }
            )

        # 系统状态
        @self.app.route("/api/status", methods=["GET"])
        def get_status():
            """获取系统整体状态"""
            try:
                status = self.container_manager.get_status()
                containers = self._build_containers_from_config()

                return jsonify_chinese(
                    {
                        "success": True,
                        "status": status,
                        "log_containers": containers,
                        "timestamp": datetime.now().isoformat(),
                    }
                )
            except Exception as e:
                logger.error(f"获取系统状态失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 容器管理
        @self.app.route("/api/containers", methods=["GET"])
        def get_containers():
            """获取所有容器列表和状态"""
            try:
                status = self.container_manager.get_status()
                return jsonify_chinese(
                    {"success": True, "containers": status["containers"], "processes": status["processes"]}
                )
            except Exception as e:
                logger.error(f"获取容器列表失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/containers/<container_key>/start", methods=["POST"])
        def start_container(container_key: str):
            """启动指定容器"""
            try:
                # 检查容器配置是否存在
                if container_key not in self.container_manager.config.get("containers", {}):
                    return jsonify_chinese({"success": False, "error": f"容器 {container_key} 不存在"}, 404)

                success = self.container_manager.mount_container(container_key)
                return jsonify_chinese(
                    {"success": success, "message": f"容器 {container_key} {'启动成功' if success else '启动失败'}"}
                )
            except Exception as e:
                logger.error(f"启动容器失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/containers/<container_key>/stop", methods=["POST"])
        def stop_container(container_key: str):
            """停止指定容器"""
            try:
                # 检查容器配置是否存在
                if container_key not in self.container_manager.config.get("containers", {}):
                    return jsonify_chinese({"success": False, "error": f"容器 {container_key} 不存在"}, 404)

                success = self.container_manager.unmount_container(container_key)
                return jsonify_chinese(
                    {"success": success, "message": f"容器 {container_key} {'停止成功' if success else '停止失败'}"}
                )
            except Exception as e:
                logger.error(f"停止容器失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/containers/<container_key>/remove", methods=["POST"])
        def remove_container(container_key: str):
            """删除指定容器"""
            try:
                # 检查容器配置是否存在
                if container_key not in self.container_manager.config.get("containers", {}):
                    return jsonify_chinese({"success": False, "error": f"容器 {container_key} 不存在"}, 404)

                success = self.container_manager.remove_container(container_key)
                return jsonify_chinese(
                    {"success": success, "message": f"容器 {container_key} {'删除成功' if success else '删除失败'}"}
                )
            except Exception as e:
                logger.error(f"删除容器失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 程序管理
        @self.app.route("/api/containers/<container_key>/programs/<command_name>/start", methods=["POST"])
        def start_program(container_key: str, command_name: str):
            """启动容器内的程序"""
            try:
                # 检查容器和程序配置是否存在
                if container_key not in self.container_manager.config.get("containers", {}):
                    return jsonify_chinese({"success": False, "error": f"容器 {container_key} 不存在"}, 404)

                container_config = self.container_manager.config["containers"][container_key]
                command_exists = any(cmd["name"] == command_name for cmd in container_config.get("commands", []))
                if not command_exists:
                    return jsonify_chinese({"success": False, "error": f"程序 {command_name} 不存在"}, 404)

                success = self.container_manager.start_program(container_key, command_name)
                return jsonify_chinese(
                    {
                        "success": success,
                        "message": f"程序 {container_key}-{command_name} {'启动成功' if success else '启动失败'}",
                    }
                )
            except Exception as e:
                logger.error(f"启动程序失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/containers/<container_key>/programs/<command_name>/stop", methods=["POST"])
        def stop_program(container_key: str, command_name: str):
            """停止容器内的程序"""
            try:
                # 检查容器和程序配置是否存在
                if container_key not in self.container_manager.config.get("containers", {}):
                    return jsonify_chinese({"success": False, "error": f"容器 {container_key} 不存在"}, 404)

                container_config = self.container_manager.config["containers"][container_key]
                command_exists = any(cmd["name"] == command_name for cmd in container_config.get("commands", []))
                if not command_exists:
                    return jsonify_chinese({"success": False, "error": f"程序 {command_name} 不存在"}, 404)

                success = self.container_manager.stop_program(container_key, command_name)
                return jsonify_chinese(
                    {
                        "success": success,
                        "message": f"程序 {container_key}-{command_name} {'停止成功' if success else '停止失败'}",
                    }
                )
            except Exception as e:
                logger.error(f"停止程序失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 日志管理
        @self.app.route("/api/logs", methods=["GET"])
        def get_logs():
            """获取所有日志文件列表"""
            try:
                containers = self._build_containers_from_config()
                return jsonify_chinese({"success": True, "containers": containers})
            except Exception as e:
                logger.error(f"获取日志列表失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/logs/<container_key>/<command_name>", methods=["GET"])
        def get_logs_content(container_key: str, command_name: str):
            """获取指定程序的日志内容"""
            try:
                # 获取参数
                lines = request.args.get("lines", type=int, default=100)
                clean_ansi = request.args.get("clean_ansi", type=bool, default=True)

                # 获取日志文件
                log_files = self.log_manager.get_log_files(container_key, command_name)

                if not log_files:
                    return jsonify_chinese({"success": False, "message": "未找到日志文件"}, 404)

                # 读取最新的日志文件
                latest_file = log_files[0]

                # 对于大文件使用流式读取，限制最大行数
                MAX_LINES = 10000  # 最多读取1万行，防止内存溢出
                if lines <= 0:
                    # 请求所有行，但限制最大行数
                    lines_to_read = MAX_LINES
                    logger.warning(f"请求读取所有日志行，限制为最多 {MAX_LINES} 行")
                else:
                    lines_to_read = min(lines, MAX_LINES)

                # 使用流式读取，只读取需要的行数
                log_lines = []
                try:
                    with open(latest_file, "r", encoding="utf-8") as f:
                        # 使用deque实现环形缓冲区，只保留最后N行
                        from collections import deque

                        log_lines = deque(f, maxlen=lines_to_read)
                except Exception as e:
                    logger.error(f"读取日志文件失败: {e}")
                    return jsonify_chinese({"success": False, "error": f"读取文件失败: {str(e)}"}, 500)

                # 格式化日志
                logs = []
                for line in log_lines:
                    line = line.strip()
                    if line:
                        # 清理ANSI转义序列
                        if clean_ansi:
                            line = clean_ansi_escape(line)

                        logs.append({"content": line, "raw": line})

                return jsonify_chinese(
                    {
                        "success": True,
                        "logs": logs,
                        "file": str(latest_file),
                        "total_lines": len(logs),
                        "requested_lines": lines,
                        "max_lines": MAX_LINES,
                        "clean_ansi": clean_ansi,
                    }
                )

            except Exception as e:
                logger.error(f"获取日志内容失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/logs/<container_key>/<command_name>/stream", methods=["GET"])
        def stream_logs(container_key: str, command_name: str):
            """实时日志流"""

            def generate():
                import time
                from flask import request as current_request

                logger.info(f"开始流式日志: {container_key}:{command_name}")

                try:
                    log_files = self.log_manager.get_log_files(container_key, command_name)

                    if not log_files:
                        error_msg = f"未找到日志文件: {container_key}:{command_name}"
                        logger.warning(error_msg)
                        yield f"data: {json.dumps({'error': error_msg}, ensure_ascii=False)}\n\n"
                        return

                    latest_file = log_files[0]
                    logger.info(f"监控日志文件: {latest_file}")

                    # 检查文件是否存在
                    if not latest_file.exists():
                        error_msg = f"日志文件不存在: {latest_file}"
                        logger.warning(error_msg)
                        yield f"data: {json.dumps({'error': error_msg}, ensure_ascii=False)}\n\n"
                        return

                    last_size = latest_file.stat().st_size
                    logger.info(f"初始文件大小: {last_size} bytes")

                    # 发送初始状态
                    yield f"data: {json.dumps({'info': f'开始监控日志文件: {latest_file.name}', 'file_size': last_size}, ensure_ascii=False)}\n\n"

                    while True:
                        try:
                            # 检测客户端是否断开连接
                            # 尝试yield一个心跳，如果客户端断开会抛出异常
                            try:
                                # 每10次循环发送一次心跳
                                import random

                                if random.randint(0, 9) == 0:
                                    yield ": heartbeat\n\n"
                            except (GeneratorExit, StopIteration):
                                logger.info(f"客户端断开连接，停止日志流: {container_key}:{command_name}")
                                break

                            # 检查文件是否仍然存在
                            if not latest_file.exists():
                                error_msg = f"日志文件已删除: {latest_file}"
                                logger.warning(error_msg)
                                yield f"data: {json.dumps({'error': error_msg}, ensure_ascii=False)}\n\n"
                                break

                            current_size = latest_file.stat().st_size

                            if current_size > last_size:
                                # 读取新内容
                                with open(latest_file, "r", encoding="utf-8") as f:
                                    f.seek(last_size)
                                    new_content = f.read()

                                if new_content.strip():
                                    logger.debug(f"读取到新内容: {len(new_content)} 字符")
                                    for line in new_content.strip().split("\n"):
                                        if line.strip():
                                            # 清理ANSI转义序列和控制字符
                                            clean_line = clean_ansi_escape(line.strip())

                                            # 如果清理后为空，跳过这一行
                                            if not clean_line.strip():
                                                continue

                                            log_data = {
                                                "timestamp": datetime.now().isoformat(),
                                                "content": clean_line,
                                                "raw": line.strip(),
                                                "container_key": container_key,
                                                "command_name": command_name,
                                            }
                                            yield f"data: {json.dumps(log_data, ensure_ascii=False)}\n\n"

                                last_size = current_size
                            elif current_size < last_size:
                                # 文件被截断或重新创建
                                logger.info(f"文件被重新创建，重置监控位置")
                                last_size = 0

                            time.sleep(1)  # 每秒检查一次

                        except Exception as e:
                            logger.error(f"流式日志错误: {e}")
                            yield f"data: {json.dumps({'error': str(e)}, ensure_ascii=False)}\n\n"
                            break

                except Exception as e:
                    logger.error(f"流式日志初始化失败: {e}")
                    yield f"data: {json.dumps({'error': str(e)}, ensure_ascii=False)}\n\n"

            return Response(
                generate(),
                mimetype="text/event-stream",
                headers={
                    "Cache-Control": "no-cache",
                    "Connection": "keep-alive",
                    "Access-Control-Allow-Origin": "*",
                    "Access-Control-Allow-Headers": "Cache-Control",
                },
            )

        @self.app.route("/api/logs/<container_key>/<command_name>/download", methods=["GET"])
        def download_log_file(container_key: str, command_name: str):
            """下载日志文件"""
            try:
                # 获取日志文件
                log_files = self.log_manager.get_log_files(container_key, command_name)

                if not log_files:
                    return jsonify_chinese({"success": False, "error": "未找到日志文件"}, 404)

                # 下载最新的日志文件
                log_file = log_files[0]

                return send_file(
                    str(log_file.absolute()),
                    as_attachment=True,
                    download_name=f"{container_key}_{command_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log",
                )

            except Exception as e:
                logger.error(f"下载日志文件失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 系统操作
        @self.app.route("/api/system/auto-start", methods=["POST"])
        def auto_start():
            """自动启动所有配置的容器和程序"""
            try:
                containers_success = self.container_manager.auto_start_containers()
                programs_success = self.container_manager.auto_start_programs()

                return jsonify_chinese(
                    {
                        "success": containers_success and programs_success,
                        "containers_started": containers_success,
                        "programs_started": programs_success,
                        "message": "自动启动完成",
                    }
                )
            except Exception as e:
                logger.error(f"自动启动失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        @self.app.route("/api/system/cleanup", methods=["POST"])
        def cleanup():
            """清理所有容器和程序"""
            try:
                self.container_manager.cleanup_all()
                return jsonify_chinese({"success": True, "message": "清理完成"})
            except Exception as e:
                logger.error(f"清理失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # API文档
        @self.app.route("/docs", methods=["GET"])
        def docs():
            """提供API文档页面"""
            api_docs = """
            <!DOCTYPE html>
            <html lang="zh-CN">
            <head>
                <meta charset="UTF-8">
                <meta name="viewport" content="width=device-width, initial-scale=1.0">
                <title>容器管理API文档</title>
                <style>
                    body { font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }
                    .container { max-width: 1200px; margin: 0 auto; background: white; padding: 30px; border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
                    h1 { color: #2c3e50; border-bottom: 2px solid #3498db; padding-bottom: 10px; }
                    h2 { color: #34495e; margin-top: 30px; }
                    .endpoint { background: #ecf0f1; padding: 15px; margin: 10px 0; border-radius: 5px; border-left: 4px solid #3498db; }
                    .method { font-weight: bold; color: #e74c3c; }
                    .path { font-family: monospace; background: #2c3e50; color: white; padding: 5px 10px; border-radius: 3px; }
                    .description { margin-top: 10px; color: #555; }
                    .example { background: #f8f9fa; padding: 10px; border-radius: 3px; font-family: monospace; margin-top: 10px; }
                </style>
            </head>
            <body>
                <div class="container">
                    <h1>🐳 容器管理集成API</h1>
                    <p>提供容器管理和实时日志查看功能的REST API服务</p>
                    
                    <h2>📊 系统状态</h2>
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/api/status</span>
                        <div class="description">获取系统整体状态</div>
                    </div>
                    
                    <h2>🐳 容器管理</h2>
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/api/containers</span>
                        <div class="description">获取所有容器列表和状态</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/containers/{container_key}/start</span>
                        <div class="description">启动指定容器</div>
                        <div class="example">curl -X POST http://localhost:8080/api/containers/相机d405/start</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/containers/{container_key}/stop</span>
                        <div class="description">停止指定容器</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/containers/{container_key}/remove</span>
                        <div class="description">删除指定容器</div>
                    </div>
                    
                    <h2>⚙️ 程序管理</h2>
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/containers/{container_key}/programs/{command_name}/start</span>
                        <div class="description">启动容器内的程序</div>
                        <div class="example">curl -X POST http://localhost:8080/api/containers/相机d405/programs/启动/start</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/containers/{container_key}/programs/{command_name}/stop</span>
                        <div class="description">停止容器内的程序</div>
                    </div>
                    
                    <h2>📝 日志管理</h2>
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/api/logs</span>
                        <div class="description">获取所有日志文件列表</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/api/logs/{container_key}/{command_name}</span>
                        <div class="description">获取指定程序的日志内容</div>
                        <div class="example">curl "http://localhost:8080/api/logs/相机d405/启动?lines=100"</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/api/logs/{container_key}/{command_name}/stream</span>
                        <div class="description">实时日志流 (Server-Sent Events)</div>
                        <div class="example">curl "http://localhost:8080/api/logs/相机d405/启动/stream"</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/api/logs/{container_key}/{command_name}/download</span>
                        <div class="description">下载日志文件</div>
                    </div>
                    
                    <h2>🔧 系统操作</h2>
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/system/auto-start</span>
                        <div class="description">自动启动所有配置的容器和程序</div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">POST</span> <span class="path">/api/system/cleanup</span>
                        <div class="description">清理所有容器和程序</div>
                    </div>
                    
                    <h2>📱 Web界面</h2>
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/</span>
                        <div class="description">主页面 - 容器监控界面</div>
                        <div class="example"><a href="/" target="_blank">http://localhost:8080/</a></div>
                    </div>
                    
                    <div class="endpoint">
                        <span class="method">GET</span> <span class="path">/docs</span>
                        <div class="description">API文档页面</div>
                        <div class="example"><a href="/docs" target="_blank">http://localhost:8080/docs</a></div>
                    </div>
                </div>
            </body>
            </html>
            """
            return api_docs, 200, {"Content-Type": "text/html; charset=utf-8"}

    def run(self, debug: bool = False):
        """运行API服务器"""
        logger.info(f"启动集成REST API服务器: http://{self.host}:{self.port}")
        logger.info(f"主页面: http://{self.host}:{self.port}/")
        logger.info(f"管理器: http://{self.host}:{self.port}/manager")
        logger.info(f"监视器: http://{self.host}:{self.port}/monitor")
        logger.info(f"图像查看器: http://{self.host}:{self.port}/image_viewer_module.html")
        logger.info(f"API文档: http://{self.host}:{self.port}/docs")
        self.app.run(host=self.host, port=self.port, debug=debug, threaded=True)


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="集成REST API服务器")
    parser.add_argument("--config", default="config/docker/config_record.yaml", help="配置文件路径")
    parser.add_argument("--host", default="0.0.0.0", help="服务器地址")
    parser.add_argument("--port", type=int, default=8080, help="服务器端口")
    parser.add_argument("--debug", action="store_true", help="调试模式")
    args = parser.parse_args()

    # 检查配置文件是否存在
    if not os.path.exists(args.config):
        logger.error(f"配置文件不存在: {args.config}")
        return

    # 创建并运行集成API服务器
    server = IntegratedRESTAPI(config_path=args.config, host=args.host, port=args.port)
    server.run(debug=args.debug)


if __name__ == "__main__":
    main()
