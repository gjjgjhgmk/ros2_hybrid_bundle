#!/usr/bin/env python3
"""
公共工具模块
提供通用的工具函数和类
"""

import os
import sys
import json
from pathlib import Path
from typing import Dict, Any, Optional, Tuple
from flask import Response
from loguru import logger


def setup_logger(is_docker: bool = False) -> None:
    """
    配置loguru日志格式

    Args:
        is_docker: 是否在Docker环境中
    """
    logger.remove()

    if is_docker:
        # Docker环境：使用简单格式，避免颜色问题
        logger.add(
            sys.stdout,
            format="{time:YYYY-MM-DD HH:mm:ss} | {level: <8} | {name}:{function}:{line} - {message}",
            level="INFO",
            colorize=False,
            backtrace=True,
            diagnose=True,
        )
    else:
        # 本地环境：使用彩色格式
        logger.add(
            sys.stdout,
            format="<green>{time:YYYY-MM-DD HH:mm:ss}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>",
            level="INFO",
            colorize=True,
            backtrace=True,
            diagnose=True,
        )


def jsonify_chinese(data: Dict[str, Any], status_code: int = 200) -> Response:
    """
    自定义jsonify函数，确保中文正确显示

    Args:
        data: 要返回的数据字典
        status_code: HTTP状态码

    Returns:
        Flask Response对象
    """
    return Response(
        json.dumps(data, ensure_ascii=False, indent=2), status=status_code, mimetype="application/json; charset=utf-8"
    )


def validate_path_safety(file_path: Path, root_path: Path) -> Tuple[bool, Optional[str]]:
    """
    验证路径安全性，确保路径在根目录内

    Args:
        file_path: 要验证的文件路径
        root_path: 根目录路径

    Returns:
        (是否安全, 错误信息)
    """
    try:
        file_path.resolve().relative_to(root_path.resolve())
        return True, None
    except ValueError:
        return False, "路径不安全，不允许访问配置根目录外的文件"


def resolve_path_with_fallback(
    path_config: str, fallback_path: str, field_name: str, config_file_path: Optional[Path] = None
) -> str:
    """
    解析路径配置，支持环境变量展开和回退机制

    Args:
        path_config: 路径配置字符串
        fallback_path: 回退路径
        field_name: 字段名称（用于日志）
        config_file_path: 配置文件路径（用于计算默认值）

    Returns:
        解析后的路径
    """
    # 展开环境变量
    resolved_path = os.path.expandvars(path_config)

    # 检查是否还有未展开的变量
    if "$" in resolved_path:
        if os.path.exists(fallback_path):
            resolved_path = fallback_path
            logger.info(f"{field_name}包含未定义的环境变量，使用回退路径: {resolved_path}")
        elif config_file_path:
            # 尝试从配置文件路径计算默认值
            default_path = config_file_path.parent.parent.parent.parent
            if os.path.exists(default_path):
                resolved_path = str(default_path)
                logger.info(f"{field_name}包含未定义的环境变量，使用配置文件路径计算默认值: {resolved_path}")
            else:
                raise ValueError(f"{field_name}包含未定义的环境变量且默认路径不存在: {path_config} -> {fallback_path}")
        else:
            raise ValueError(f"{field_name}包含未定义的环境变量且回退路径不存在: {path_config} -> {fallback_path}")

    return resolved_path


def check_docker_environment() -> bool:
    """
    检查是否在Docker环境中

    Returns:
        是否在Docker环境中
    """
    return os.path.exists("/.dockerenv") or os.environ.get("DOCKER_CONTAINER", False)


class PathValidator:
    """路径验证器类"""

    def __init__(self, root_path: Path):
        """
        初始化路径验证器

        Args:
            root_path: 根目录路径
        """
        self.root_path = Path(root_path).resolve()

    def validate(self, file_path: Path) -> Tuple[bool, Optional[str]]:
        """
        验证路径安全性

        Args:
            file_path: 要验证的文件路径

        Returns:
            (是否安全, 错误信息)
        """
        return validate_path_safety(Path(file_path), self.root_path)

    def get_full_path(self, relative_path: str) -> Path:
        """
        获取完整路径

        Args:
            relative_path: 相对路径

        Returns:
            完整路径对象
        """
        return self.root_path / relative_path.lstrip("/")


class APIResponseBuilder:
    """API响应构建器类"""

    @staticmethod
    def success(data: Dict[str, Any] = None, message: str = None, **kwargs) -> Response:
        """
        构建成功响应

        Args:
            data: 响应数据
            message: 成功消息
            **kwargs: 其他字段

        Returns:
            Flask Response对象
        """
        response_data = {"success": True}
        if data:
            response_data.update(data)
        if message:
            response_data["message"] = message
        response_data.update(kwargs)
        return jsonify_chinese(response_data)

    @staticmethod
    def error(error: str, status_code: int = 400, **kwargs) -> Response:
        """
        构建错误响应

        Args:
            error: 错误信息
            status_code: HTTP状态码
            **kwargs: 其他字段

        Returns:
            Flask Response对象
        """
        response_data = {"success": False, "error": error}
        response_data.update(kwargs)
        return jsonify_chinese(response_data, status_code)


def safe_get_config(config: Dict[str, Any], *keys, default=None):
    """
    安全获取配置值，支持嵌套键

    Args:
        config: 配置字典
        *keys: 键路径
        default: 默认值

    Returns:
        配置值或默认值

    Example:
        safe_get_config(config, 'paths', 'project_root', default='/default')
    """
    value = config
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            return default
    return value if value is not None else default
