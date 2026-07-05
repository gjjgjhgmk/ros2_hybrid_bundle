#!/usr/bin/env python3
"""
配置文件REST API服务器
提供配置文件查看和编辑功能
"""

import os
import sys
import json
import yaml
import shutil
from datetime import datetime
from pathlib import Path
from flask import Flask, jsonify, request, Response
from flask_cors import CORS
from loguru import logger

# 导入公共工具模块
from common_utils import (
    setup_logger,
    jsonify_chinese,
    validate_path_safety,
    PathValidator,
    APIResponseBuilder,
    check_docker_environment,
)

# 配置loguru日志格式
setup_logger(check_docker_environment())


class ConfigRESTAPI:
    """配置文件REST API服务器类"""

    def __init__(self, config_root: str = None, host: str = "0.0.0.0", port: int = 7002):
        """
        初始化配置文件API服务器

        Args:
            config_root: 配置根目录路径（默认: 项目根目录下的config目录）
            host: 服务器地址
            port: 服务器端口
        """
        # 确定配置根目录
        if config_root is None:
            # 默认使用项目根目录下的config目录
            script_dir = Path(__file__).parent
            project_root = script_dir.parent
            self.config_root = project_root / "config"
        else:
            self.config_root = Path(config_root)

        if not self.config_root.exists():
            logger.warning(f"配置根目录不存在: {self.config_root}，将尝试创建")
            self.config_root.mkdir(parents=True, exist_ok=True)

        self.host = host
        self.port = port

        # 初始化路径验证器
        self.path_validator = PathValidator(self.config_root)

        logger.info(f"配置加载完成:")
        logger.info(f"  配置根目录: {self.config_root}")
        logger.info(f"  监听地址: {self.host}:{self.port}")

        # 创建Flask应用
        # 禁用默认的静态文件处理，使用自定义路由
        self.app = Flask(__name__, static_folder=None, static_url_path=None)
        self.app.config["JSON_AS_ASCII"] = False
        CORS(self.app)  # 启用CORS支持

        # 注册静态文件路由（用于提供公共CSS和JS）
        # 必须在其他路由之前注册，确保优先匹配
        @self.app.route("/static/<path:filename>", methods=["GET"])
        def static_files(filename):
            """提供静态文件"""
            try:
                # 计算静态文件目录路径
                script_dir = Path(__file__).parent
                project_root = script_dir.parent
                static_dir = project_root / "html"
                file_path = static_dir / filename

                logger.debug(f"请求静态文件: {filename}, 路径: {file_path}")

                # 安全检查：确保文件在html目录内
                try:
                    file_path.resolve().relative_to(static_dir.resolve())
                except ValueError:
                    logger.warning(f"不安全的静态文件路径: {filename}")
                    return "路径不安全", 403

                if file_path.exists() and file_path.is_file():
                    # 根据文件扩展名设置Content-Type
                    if filename.endswith(".css"):
                        content_type = "text/css"
                    elif filename.endswith(".js"):
                        content_type = "application/javascript"
                    else:
                        content_type = "text/plain"

                    with open(file_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    logger.info(f"成功提供静态文件: {filename} ({len(content)} 字节)")
                    return content, 200, {"Content-Type": f"{content_type}; charset=utf-8"}
                else:
                    logger.warning(f"静态文件不存在: {file_path} (目录存在: {static_dir.exists()})")
                    return f"文件未找到: {filename}", 404
            except Exception as e:
                logger.error(f"提供静态文件失败: {e}", exc_info=True)
                return f"错误: {str(e)}", 500

        # 初始化SSH管理器
        try:
            from ssh_manager import get_ssh_manager

            self.ssh_manager = get_ssh_manager()
            logger.info("SSH管理器初始化成功")
        except ImportError:
            logger.warning("paramiko未安装，SSH远程启动功能将不可用")
            self.ssh_manager = None
        except Exception as e:
            logger.warning(f"SSH管理器初始化失败: {e}")
            self.ssh_manager = None

        # 注册路由
        self._register_routes()

        logger.info(f"配置文件REST API服务器初始化完成: {self.host}:{self.port}")

    def _register_routes(self):
        """注册API路由"""

        # 注意：更具体的路由应该先注册，避免被通用路由匹配

        # 备份配置文件（必须在 /api/config/file 之前注册）
        @self.app.route("/api/config/file/backup", methods=["POST"])
        def backup_config_file():
            """
            备份配置文件

            注意：此功能备份的是后端文件系统中的原始文件，而不是前端编辑的内容。
            即使前端有未保存的修改，备份的仍然是后端文件系统中的当前文件。
            """
            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                file_path = data.get("path")
                if not file_path:
                    return jsonify_chinese({"success": False, "error": "缺少path参数"}, 400)

                # 构建完整路径（后端文件系统路径）
                full_path = self.path_validator.get_full_path(file_path)

                # 安全检查：确保路径在配置根目录内
                is_safe, error_msg = self.path_validator.validate(full_path)
                if not is_safe:
                    return APIResponseBuilder.error(error_msg or "路径不安全，不允许访问配置根目录外的文件", 403)

                if not full_path.exists():
                    return jsonify_chinese({"success": False, "error": f"文件不存在: {file_path}"}, 404)

                if not full_path.is_file():
                    return jsonify_chinese({"success": False, "error": f"路径不是文件: {file_path}"}, 400)

                # 生成备份文件名（添加时间戳）
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                backup_name = f"{full_path.stem}_{timestamp}{full_path.suffix}"
                backup_path = full_path.parent / backup_name

                # 直接从后端文件系统复制文件（不涉及前端编辑的内容）
                try:
                    shutil.copy2(full_path, backup_path)
                    logger.info(f"配置文件备份成功: {full_path} -> {backup_path}")
                    return jsonify_chinese(
                        {
                            "success": True,
                            "message": f"配置文件备份成功: {backup_name}",
                            "original_path": file_path,
                            "backup_path": str(backup_path.relative_to(self.config_root)),
                        }
                    )
                except Exception as e:
                    logger.error(f"备份配置文件失败: {e}")
                    return jsonify_chinese({"success": False, "error": f"备份失败: {str(e)}"}, 500)

            except Exception as e:
                logger.error(f"备份配置文件失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 根路径 - 提供远程配置管理界面（包含配置管理和远程启动两个tab）
        @self.app.route("/", methods=["GET"])
        def index():
            """提供远程配置管理界面（标签页容器）"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "config_manager.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "远程配置管理页面未找到", 404
            except Exception as e:
                logger.error(f"提供远程配置管理页面失败: {e}")
                return f"错误: {e}", 500

        # 配置管理页面路由（标签页容器，与根路径相同）
        @self.app.route("/config-manager", methods=["GET"])
        def config_manager_page():
            """提供配置管理页面（标签页容器）"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "config_manager.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "配置管理页面未找到", 404
            except Exception as e:
                logger.error(f"提供配置管理页面失败: {e}")
                return f"错误: {e}", 500

        # 远程启动管理页面
        @self.app.route("/remote-start", methods=["GET"])
        def remote_start_page():
            """提供远程启动管理页面"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "remote_start.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "远程启动管理页面未找到", 404
            except Exception as e:
                logger.error(f"提供远程启动管理页面失败: {e}")
                return f"错误: {e}", 500

        # 配置文件查看器页面（实际的配置管理功能）
        @self.app.route("/remote-config", methods=["GET"])
        def remote_config_page():
            """提供配置文件查看器页面（实际的配置管理功能）"""
            try:
                html_path = Path(__file__).parent.parent / "html" / "remote_config.html"
                if html_path.exists():
                    with open(html_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    return content, 200, {"Content-Type": "text/html; charset=utf-8"}
                else:
                    return "远程配置管理页面未找到", 404
            except Exception as e:
                logger.error(f"提供远程配置管理页面失败: {e}")
                return f"错误: {e}", 500

        # 健康检查
        @self.app.route("/api/health", methods=["GET"])
        def health():
            """健康检查接口"""
            return jsonify_chinese(
                {
                    "status": "healthy",
                    "timestamp": str(Path(__file__).stat().st_mtime),
                    "service": "config-rest-api",
                    "version": "1.0.0",
                }
            )

        # 获取配置目录树
        @self.app.route("/api/config/tree", methods=["GET"])
        def get_config_tree():
            """获取配置根目录及所有目录结构"""
            try:
                tree = self._build_directory_tree(self.config_root)
                return jsonify_chinese({"success": True, "root": str(self.config_root), "tree": tree})
            except Exception as e:
                logger.error(f"获取配置目录树失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 读取配置文件
        @self.app.route("/api/config/file", methods=["GET"])
        def get_config_file():
            """读取配置文件并转换为JSON"""
            try:
                file_path = request.args.get("path")
                if not file_path:
                    return jsonify_chinese({"success": False, "error": "缺少path参数"}, 400)

                # 构建完整路径
                full_path = self.path_validator.get_full_path(file_path)

                # 安全检查：确保路径在配置根目录内
                is_safe, error_msg = self.path_validator.validate(full_path)
                if not is_safe:
                    return APIResponseBuilder.error(error_msg or "路径不安全，不允许访问配置根目录外的文件", 403)

                if not full_path.exists():
                    return jsonify_chinese({"success": False, "error": f"文件不存在: {file_path}"}, 404)

                if not full_path.is_file():
                    return jsonify_chinese({"success": False, "error": f"路径不是文件: {file_path}"}, 400)

                # 读取并解析YAML文件
                with open(full_path, "r", encoding="utf-8") as f:
                    content = f.read()

                # 尝试解析为YAML
                try:
                    data = yaml.safe_load(content)
                    if data is None:
                        data = {}
                except yaml.YAMLError as e:
                    return jsonify_chinese({"success": False, "error": f"YAML解析失败: {str(e)}"}, 400)

                return jsonify_chinese(
                    {
                        "success": True,
                        "path": file_path,
                        "data": data,
                        "raw_content": content,  # 保留原始内容用于显示
                    }
                )
            except Exception as e:
                logger.error(f"读取配置文件失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 保存配置文件
        @self.app.route("/api/config/file", methods=["POST"])
        def save_config_file():
            """保存配置文件"""
            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                file_path = data.get("path")
                config_data = data.get("data")
                save_path = data.get("save_path", file_path)  # 默认保存到原文件

                if not file_path:
                    return jsonify_chinese({"success": False, "error": "缺少path参数"}, 400)

                if config_data is None:
                    return jsonify_chinese({"success": False, "error": "缺少data参数"}, 400)

                # 构建完整路径
                full_path = self.path_validator.get_full_path(save_path)

                # 安全检查：确保路径在配置根目录内
                is_safe, error_msg = self.path_validator.validate(full_path)
                if not is_safe:
                    return APIResponseBuilder.error(error_msg or "路径不安全，不允许访问配置根目录外的文件", 403)

                # 确保目录存在
                full_path.parent.mkdir(parents=True, exist_ok=True)

                # 将JSON数据转换为YAML并保存
                try:
                    with open(full_path, "w", encoding="utf-8") as f:
                        yaml.dump(config_data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)

                    logger.info(f"配置文件保存成功: {full_path}")
                    return jsonify_chinese(
                        {"success": True, "message": f"配置文件保存成功: {save_path}", "path": save_path}
                    )
                except Exception as e:
                    logger.error(f"保存配置文件失败: {e}")
                    return jsonify_chinese({"success": False, "error": f"保存失败: {str(e)}"}, 500)

            except Exception as e:
                logger.error(f"保存配置文件失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 删除配置文件
        @self.app.route("/api/config/file", methods=["DELETE"])
        def delete_config_file():
            """删除配置文件"""
            try:
                file_path = request.args.get("path")
                if not file_path:
                    return jsonify_chinese({"success": False, "error": "缺少path参数"}, 400)

                # 构建完整路径
                full_path = self.path_validator.get_full_path(file_path)

                # 安全检查：确保路径在配置根目录内
                is_safe, error_msg = self.path_validator.validate(full_path)
                if not is_safe:
                    return APIResponseBuilder.error(error_msg or "路径不安全，不允许访问配置根目录外的文件", 403)

                if not full_path.exists():
                    return jsonify_chinese({"success": False, "error": f"文件不存在: {file_path}"}, 404)

                if not full_path.is_file():
                    return jsonify_chinese({"success": False, "error": f"路径不是文件: {file_path}"}, 400)

                # 删除文件
                try:
                    full_path.unlink()
                    logger.info(f"配置文件删除成功: {full_path}")
                    return jsonify_chinese(
                        {"success": True, "message": f"配置文件删除成功: {file_path}", "path": file_path}
                    )
                except Exception as e:
                    logger.error(f"删除配置文件失败: {e}")
                    return jsonify_chinese({"success": False, "error": f"删除失败: {str(e)}"}, 500)

            except Exception as e:
                logger.error(f"删除配置文件失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 远程启动后端程序
        @self.app.route("/api/remote/start", methods=["POST"])
        def remote_start():
            """通过SSH远程启动目标主机上的后端程序"""
            if not self.ssh_manager:
                return jsonify_chinese(
                    {"success": False, "error": "SSH功能不可用，请安装paramiko: pip install paramiko"}, 503
                )

            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                host = data.get("host")
                port = data.get("port", 22)
                username = data.get("username")
                password = data.get("password")
                command = data.get("command")
                work_dir = data.get("work_dir", "")

                if not host:
                    return jsonify_chinese({"success": False, "error": "缺少host参数"}, 400)
                if not username:
                    return jsonify_chinese({"success": False, "error": "缺少username参数"}, 400)
                if not command:
                    return jsonify_chinese({"success": False, "error": "缺少command参数"}, 400)

                # 构建完整命令（如果指定了工作目录）
                if work_dir:
                    full_command = f"cd {work_dir} && {command}"
                else:
                    full_command = command

                # 执行远程命令
                success, stdout, stderr, exit_code = self.ssh_manager.execute_command(
                    host=host, port=port, username=username, command=full_command, password=password, timeout=60
                )

                if success and exit_code == 0:
                    logger.info(f"远程启动成功: {username}@{host}:{port} - {command[:50]}...")
                    return jsonify_chinese(
                        {
                            "success": True,
                            "message": "远程启动成功",
                            "stdout": stdout,
                            "stderr": stderr,
                            "exit_code": exit_code,
                        }
                    )
                else:
                    error_msg = stderr if stderr else stdout if not success else f"命令执行失败，退出码: {exit_code}"
                    logger.error(f"远程启动失败: {username}@{host}:{port} - {error_msg}")
                    return jsonify_chinese(
                        {
                            "success": False,
                            "error": error_msg,
                            "stdout": stdout,
                            "stderr": stderr,
                            "exit_code": exit_code,
                        },
                        500,
                    )

            except Exception as e:
                logger.error(f"远程启动失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 测试SSH连接
        @self.app.route("/api/remote/test", methods=["POST"])
        def test_ssh_connection():
            """测试SSH连接"""
            if not self.ssh_manager:
                return jsonify_chinese(
                    {"success": False, "error": "SSH功能不可用，请安装paramiko: pip install paramiko"}, 503
                )

            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                host = data.get("host")
                port = data.get("port", 22)
                username = data.get("username")
                password = data.get("password")

                if not host:
                    return jsonify_chinese({"success": False, "error": "缺少host参数"}, 400)
                if not username:
                    return jsonify_chinese({"success": False, "error": "缺少username参数"}, 400)

                # 测试连接
                success, error_msg = self.ssh_manager.connect(
                    host=host, port=port, username=username, password=password, timeout=10
                )

                if success:
                    # 测试执行简单命令
                    test_success, stdout, stderr, exit_code = self.ssh_manager.execute_command(
                        host=host,
                        port=port,
                        username=username,
                        command="echo 'SSH连接测试成功'",
                        password=password,
                        timeout=10,
                    )

                    if test_success:
                        return jsonify_chinese({"success": True, "message": "SSH连接测试成功", "stdout": stdout})
                    else:
                        return jsonify_chinese(
                            {
                                "success": False,
                                "error": f"连接成功但命令执行失败: {stderr}",
                                "stdout": stdout,
                                "stderr": stderr,
                            }
                        )
                else:
                    return jsonify_chinese({"success": False, "error": error_msg}, 400)

            except Exception as e:
                logger.error(f"SSH连接测试失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 上传文件或文件夹到远程主机
        @self.app.route("/api/remote/upload", methods=["POST"])
        def upload_file_or_directory():
            """通过SSH上传本地文件或文件夹到远程主机，支持多种传输模式"""
            if not self.ssh_manager:
                return jsonify_chinese(
                    {"success": False, "error": "SSH功能不可用，请安装paramiko: pip install paramiko"}, 503
                )

            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                host = data.get("host")
                port = data.get("port", 22)
                username = data.get("username")
                password = data.get("password")
                local_path = data.get("local_path")
                remote_path = data.get("remote_path")

                if not host:
                    return jsonify_chinese({"success": False, "error": "缺少host参数"}, 400)
                if not username:
                    return jsonify_chinese({"success": False, "error": "缺少username参数"}, 400)
                if not local_path:
                    return jsonify_chinese({"success": False, "error": "缺少local_path参数"}, 400)
                if not remote_path:
                    return jsonify_chinese({"success": False, "error": "缺少remote_path参数"}, 400)

                # 检查本地路径是否存在
                local_path_obj = Path(local_path)
                if not local_path_obj.exists():
                    return jsonify_chinese({"success": False, "error": f"本地路径不存在: {local_path}"}, 400)

                local_is_file = local_path_obj.is_file()
                local_is_dir = local_path_obj.is_dir()

                # 验证传输模式
                remote_is_dir = remote_path.endswith("/")
                if not remote_is_dir:
                    # 尝试通过SSH检查远程路径类型（如果可能）
                    # 这里先不做检查，由ssh_manager处理
                    pass

                # 模式2: 远程是文件时，本地只能是文件
                if not remote_is_dir and local_is_dir:
                    return jsonify_chinese(
                        {"success": False, "error": "远程路径是文件，但本地路径是文件夹，无法传输"}, 400
                    )

                # 上传文件或文件夹
                success, message, file_count = self.ssh_manager.upload_file_or_directory(
                    host=host,
                    port=port,
                    username=username,
                    local_path=str(local_path_obj),
                    remote_path=remote_path,
                    password=password,
                )

                if success:
                    logger.info(
                        f"传输成功: {local_path} -> {username}@{host}:{port}{remote_path} ({file_count} 个文件)"
                    )
                    return jsonify_chinese(
                        {
                            "success": True,
                            "message": message,
                            "file_count": file_count,
                            "local_path": local_path,
                            "remote_path": remote_path,
                        }
                    )
                else:
                    logger.error(f"传输失败: {message}")
                    return jsonify_chinese({"success": False, "error": message, "file_count": file_count}, 500)

            except Exception as e:
                logger.error(f"传输失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 获取远程目录列表
        @self.app.route("/api/remote/list", methods=["POST"])
        def remote_list_directory():
            """通过SSH获取远程目录的文件和目录列表"""
            if not self.ssh_manager:
                return jsonify_chinese(
                    {"success": False, "error": "SSH功能不可用，请安装paramiko: pip install paramiko"}, 503
                )

            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                host = data.get("host")
                port = data.get("port", 22)
                username = data.get("username")
                password = data.get("password")
                path = data.get("path", "~")

                if not host:
                    return jsonify_chinese({"success": False, "error": "缺少host参数"}, 400)
                if not username:
                    return jsonify_chinese({"success": False, "error": "缺少username参数"}, 400)

                # 处理 ~ 路径，先获取用户主目录
                if path == "~" or path.startswith("~/"):
                    # 获取用户主目录
                    home_command = "echo $HOME"
                    home_success, home_stdout, home_stderr, home_exit = self.ssh_manager.execute_command(
                        host=host, port=port, username=username, command=home_command, password=password, timeout=5
                    )
                    if home_success and home_exit == 0:
                        home_path = home_stdout.strip()
                        if path == "~":
                            actual_path = home_path
                        else:
                            actual_path = path.replace("~", home_path)
                    else:
                        actual_path = path
                else:
                    actual_path = path

                # 执行ls命令获取目录列表
                # 使用 ls -la 获取详细信息，然后解析
                command = f"ls -la '{actual_path}' 2>/dev/null || echo 'ERROR'"
                success, stdout, stderr, exit_code = self.ssh_manager.execute_command(
                    host=host, port=port, username=username, command=command, password=password, timeout=10
                )

                if not success:
                    return jsonify_chinese({"success": False, "error": stderr or "SSH连接失败"}, 500)

                if exit_code != 0 or "ERROR" in stdout:
                    error_msg = stderr.strip() if stderr else "无法访问目录"
                    return jsonify_chinese({"success": False, "error": error_msg}, 500)

                # 解析ls输出
                items = []
                lines = stdout.strip().split("\n")
                for line in lines[1:]:  # 跳过第一行（总计信息）
                    if not line.strip():
                        continue
                    parts = line.split(None, 8)  # 最多分割成9部分
                    if len(parts) < 9:
                        continue

                    # 解析文件权限、类型等信息
                    permissions = parts[0]
                    file_type = "directory" if permissions.startswith("d") else "file"
                    name = parts[8]

                    # 跳过 . 和 ..
                    if name in [".", ".."]:
                        continue

                    # 构建完整路径
                    if path.endswith("/"):
                        full_path = f"{path}{name}"
                    else:
                        full_path = f"{path}/{name}"
                    # 规范化路径
                    full_path = full_path.replace("//", "/").replace("~", f"/home/{username}")

                    items.append({"name": name, "type": file_type, "path": full_path, "permissions": permissions})

                # 排序：目录在前，文件在后，按名称排序
                items.sort(key=lambda x: (x["type"] != "directory", x["name"].lower()))

                return jsonify_chinese({"success": True, "path": path, "items": items})

            except Exception as e:
                logger.error(f"获取远程目录列表失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

        # 删除远程文件或目录
        @self.app.route("/api/remote/delete", methods=["POST"])
        def remote_delete_file_or_directory():
            """通过SSH删除远程文件或目录"""
            if not self.ssh_manager:
                return jsonify_chinese(
                    {"success": False, "error": "SSH功能不可用，请安装paramiko: pip install paramiko"}, 503
                )

            try:
                data = request.get_json()
                if not data:
                    return jsonify_chinese({"success": False, "error": "请求体为空"}, 400)

                host = data.get("host")
                port = data.get("port", 22)
                username = data.get("username")
                password = data.get("password")
                path = data.get("path")
                item_type = data.get("type", "file")  # file 或 directory

                if not host:
                    return jsonify_chinese({"success": False, "error": "缺少host参数"}, 400)
                if not username:
                    return jsonify_chinese({"success": False, "error": "缺少username参数"}, 400)
                if not path:
                    return jsonify_chinese({"success": False, "error": "缺少path参数"}, 400)

                # 构建删除命令
                if item_type == "directory":
                    # 删除目录：使用 rm -rf
                    command = f"rm -rf '{path}'"
                    logger.info(f"执行删除目录命令: {command}")
                else:
                    # 删除文件：使用 rm
                    command = f"rm -f '{path}'"
                    logger.info(f"执行删除文件命令: {command}")

                # 执行删除命令
                success, stdout, stderr, exit_code = self.ssh_manager.execute_command(
                    host=host, port=port, username=username, command=command, password=password, timeout=30
                )

                if not success:
                    return jsonify_chinese({"success": False, "error": stderr or "SSH连接失败"}, 500)

                if exit_code != 0:
                    error_msg = stderr.strip() if stderr else f"删除失败，退出码: {exit_code}"
                    logger.error(f"删除失败: {error_msg}")
                    return jsonify_chinese({"success": False, "error": error_msg}, 500)

                logger.info(f"删除成功: {path}")
                return jsonify_chinese({"success": True, "message": f"成功删除: {path}"})

            except Exception as e:
                logger.error(f"删除远程文件/目录失败: {e}")
                return jsonify_chinese({"success": False, "error": str(e)}, 500)

    def _build_directory_tree(self, root_path: Path, relative_path: str = ""):
        """
        构建目录树结构

        Args:
            root_path: 根目录路径
            relative_path: 相对路径（用于递归）

        Returns:
            目录树字典
        """
        tree = {"name": root_path.name, "type": "directory", "path": relative_path, "children": []}

        try:
            # 获取所有子项，按名称排序
            items = sorted(root_path.iterdir(), key=lambda x: (x.is_file(), x.name.lower()))

            for item in items:
                # 跳过隐藏文件
                if item.name.startswith("."):
                    continue

                item_relative_path = os.path.join(relative_path, item.name).replace("\\", "/")

                if item.is_dir():
                    # 递归处理子目录
                    subtree = self._build_directory_tree(item, item_relative_path)
                    tree["children"].append(subtree)
                elif item.is_file():
                    # 只显示YAML和JSON文件
                    if item.suffix.lower() in [".yaml", ".yml", ".json"]:
                        tree["children"].append({"name": item.name, "type": "file", "path": item_relative_path})

        except PermissionError:
            logger.warning(f"无权限访问目录: {root_path}")
        except Exception as e:
            logger.error(f"构建目录树失败 {root_path}: {e}")

        return tree

    def run(self, debug=False):
        """运行Flask服务器"""
        self.app.run(host=self.host, port=self.port, debug=debug, threaded=True)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="启动配置文件REST API服务")
    parser.add_argument("--config-root", default=None, help="配置根目录路径（默认: 项目根目录下的config目录）")
    parser.add_argument("--host", default="0.0.0.0", help="服务器监听地址")
    parser.add_argument("--port", "-p", type=int, default=7002, help="服务器监听端口")
    parser.add_argument("--debug", action="store_true", help="启用调试模式")

    args = parser.parse_args()

    server = ConfigRESTAPI(config_root=args.config_root, host=args.host, port=args.port)
    print("\n" + "=" * 60)
    print("🚀 配置文件REST API服务已启动")
    print("=" * 60)
    print(f"📁 配置根目录: {server.config_root}")
    print(f"🌐 监听地址: {server.host}:{server.port}")
    print(f"📖 配置查看器: http://localhost:{server.port}/")
    print(f"🔍 健康检查: http://localhost:{server.port}/api/health")
    print("=" * 60)
    print("💡 主要功能:")
    print("  • 浏览配置目录结构")
    print("  • 查看和编辑配置文件")
    print("  • 保存配置文件")
    print("=" * 60)
    print("按 Ctrl+C 停止服务")
    print("=" * 60 + "\n")

    try:
        server.run(debug=args.debug)
    except KeyboardInterrupt:
        logger.info("收到中断信号，正在关闭服务...")
