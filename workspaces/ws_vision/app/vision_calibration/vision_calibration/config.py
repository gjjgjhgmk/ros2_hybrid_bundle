"""
Configuration module for PapJia Camera Calibration Service.

This module provides configuration management for the camera calibration service,
including ZeroMQ and REST API settings.
"""

import os
import yaml
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class ZMQConfig:
    """ZeroMQ server configuration."""

    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = 7021
    max_workers: int = 2
    calibration_config_file: str = "/home/yw/workspace/config/vision_calibration/calibration_config.yaml"
    root_data_path: str = "/home/yw/workspace/data/vision_calibration"

    def get_bind_address(self) -> str:
        """Get ZeroMQ bind address."""
        return f"tcp://{self.host}:{self.port}"


@dataclass
class RESTAPIConfig:
    """REST API server configuration."""

    enabled: bool = True
    host: str = "0.0.0.0"
    port: int = 7001
    workers: int = 1
    reload: bool = False
    debug: bool = True
    cors_enabled: bool = True
    cors_origins: List[str] = field(default_factory=lambda: ["*"])
    calibration_config_file: str = "/home/yw/workspace/config/vision_calibration/calibration_config.yaml"
    html_file_path: str = "/home/yw/workspace/html/camera_calibration.html"
    root_data_path: str = "/home/yw/workspace/data/vision_calibration"
    # 图像压缩配置
    image_compress_enabled: bool = True  # 是否启用图像压缩
    image_compress_max_width: int = 1920  # 压缩后最大宽度（像素）
    image_compress_max_height: int = 1920  # 压缩后最大高度（像素）
    image_compress_quality: int = 85  # JPEG压缩质量（1-100）

    def get_bind_address(self) -> str:
        """Get REST API bind address."""
        return f"{self.host}:{self.port}"


@dataclass
class LoggingConfig:
    """Logging configuration."""

    level: str = "INFO"
    format: str = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"


@dataclass
class ServiceConfig:
    """Complete service configuration."""

    zeromq: ZMQConfig = field(default_factory=ZMQConfig)
    rest_api: RESTAPIConfig = field(default_factory=RESTAPIConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)


def load_config(config_path: Optional[str] = None) -> ServiceConfig:
    """
    Load configuration from file or use defaults.

    Args:
        config_path: Path to configuration file

    Returns:
        ServiceConfig instance
    """
    config = ServiceConfig()

    if config_path and os.path.exists(config_path):
        try:
            print(f"Loading configuration from {config_path}", flush=True)
            config_dir = os.path.dirname(config_path)
            with open(config_path, "r", encoding="utf-8") as f:
                yaml_config = yaml.safe_load(f)

            # Update configuration from YAML
            if "zeromq" in yaml_config:
                for key, value in yaml_config["zeromq"].items():
                    if key == "calibration_config_file":
                        value = os.path.join(config_dir, value)
                    if hasattr(config.zeromq, key):
                        setattr(config.zeromq, key, value)

            if "rest_api" in yaml_config:
                for key, value in yaml_config["rest_api"].items():
                    if key == "calibration_config_file":
                        # 如果是相对路径，则基于配置目录拼接
                        if not os.path.isabs(value):
                            value = os.path.join(config_dir, value)
                        print(f"calibration_config_file: {value}", flush=True)
                    if key == "html_file_path":
                        # 如果是相对路径，则基于配置目录拼接；绝对路径直接使用
                        if not os.path.isabs(value):
                            value = os.path.join(config_dir, value)
                        print(f"html_file_path: {value}", flush=True)
                    if hasattr(config.rest_api, key):
                        setattr(config.rest_api, key, value)
                        # 对于图像压缩配置，打印加载信息
                        if key.startswith("image_compress"):
                            print(f"已加载图像压缩配置: {key} = {value}", flush=True)
                    else:
                        print(f"警告: rest_api 配置中存在未知字段 '{key}'，将被忽略", flush=True)

            if "logging" in yaml_config:
                for key, value in yaml_config["logging"].items():
                    if hasattr(config.logging, key):
                        setattr(config.logging, key, value)

        except Exception as e:
            print(f"Warning: Failed to load configuration from {config_path}: {e}", flush=True)
            print("Using default configuration", flush=True)

    return config


def save_config(config: ServiceConfig, config_path: str):
    """
    Save configuration to file.

    Args:
        config: Service configuration
        config_path: Path to save configuration file
    """
    print(f"Saving configuration to {config_path}", flush=True)
    yaml_config = {
        "zeromq": {
            "enabled": config.zeromq.enabled,
            "host": config.zeromq.host,
            "port": config.zeromq.port,
            "max_workers": config.zeromq.max_workers,
            "calibration_config_file": config.zeromq.calibration_config_file,
            "root_data_path": config.zeromq.root_data_path,
        },
        "rest_api": {
            "enabled": config.rest_api.enabled,
            "host": config.rest_api.host,
            "port": config.rest_api.port,
            "workers": config.rest_api.workers,
            "reload": config.rest_api.reload,
            "debug": config.rest_api.debug,
            "cors_enabled": config.rest_api.cors_enabled,
            "cors_origins": config.rest_api.cors_origins,
            "calibration_config_file": config.rest_api.calibration_config_file,
            "html_file_path": config.rest_api.html_file_path,
            "root_data_path": config.rest_api.root_data_path,
            "image_compress_enabled": config.rest_api.image_compress_enabled,
            "image_compress_max_width": config.rest_api.image_compress_max_width,
            "image_compress_max_height": config.rest_api.image_compress_max_height,
            "image_compress_quality": config.rest_api.image_compress_quality,
        },
        "logging": {
            "level": config.logging.level,
            "format": config.logging.format,
        },
    }

    os.makedirs(os.path.dirname(config_path), exist_ok=True)
    with open(config_path, "w", encoding="utf-8") as f:
        yaml.dump(yaml_config, f, default_flow_style=False, allow_unicode=True)
