#!/usr/bin/env python3
"""
日志管理器
负责收集、保存和分发容器内程序的日志
"""

import os
import sys
import json
import time
import threading
import asyncio
import websockets
from datetime import datetime
from typing import Dict, List, Optional, Callable, Any
from pathlib import Path
from loguru import logger
import queue
import re

# 导入公共工具模块
from common_utils import setup_logger, check_docker_environment

# 配置loguru日志格式
setup_logger(check_docker_environment())


class LogManager:
    """日志管理器类"""

    # 常量定义
    MAX_QUEUE_SIZE = 10000  # 日志队列最大大小
    LOG_PROCESSOR_TIMEOUT = 1  # 日志处理线程超时时间（秒）

    def __init__(self, log_dir: str = "logs", max_file_size: int = 10 * 1024 * 1024):
        """
        初始化日志管理器

        Args:
            log_dir: 日志保存目录
            max_file_size: 单个日志文件最大大小（字节）
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        self.max_file_size = max_file_size

        # 日志文件句柄缓存
        self.log_files: Dict[str, Dict[str, Any]] = {}

        # 线程安全锁：保护log_files字典和文件操作
        self._file_lock = threading.Lock()

        # 日志队列（用于异步处理）
        self.log_queue = queue.Queue(maxsize=self.MAX_QUEUE_SIZE)

        # 启动日志处理线程
        self._start_log_processor()

        logger.info(f"日志管理器初始化完成，日志目录: {self.log_dir}")

    def _start_log_processor(self):
        """启动日志处理线程"""

        def process_logs():
            while True:
                try:
                    log_data = self.log_queue.get(timeout=self.LOG_PROCESSOR_TIMEOUT)
                    self._process_log_data(log_data)
                except queue.Empty:
                    continue
                except Exception as e:
                    logger.error(f"日志处理错误: {e}")

        thread = threading.Thread(target=process_logs, daemon=True)
        thread.start()

    def _extract_base_command_name(self, filename: str) -> str:
        """从文件名中提取基础命令名，去掉时间戳后缀"""
        try:
            # 文件名格式可能是: "启动" 或 "启动_191018_191019_..."
            # 使用正则表达式匹配时间戳模式
            timestamp_pattern = r"_\d{6}(?:_\d{6})*$"

            # 如果文件名包含时间戳后缀，去掉它
            if re.search(timestamp_pattern, filename):
                # 找到第一个时间戳的位置
                match = re.search(r"_\d{6}", filename)
                if match:
                    base_name = filename[: match.start()]
                    logger.debug(f"提取基础命令名: {filename} -> {base_name}")
                    return base_name

            # 没有时间戳后缀，直接返回原文件名
            logger.debug(f"文件名无时间戳后缀: {filename}")
            return filename

        except Exception as e:
            logger.error(f"提取基础命令名失败: {e}")
            return filename

    def _get_log_file_path(self, container_key: str, command_name: str) -> Path:
        """获取日志文件路径"""
        # 按日期和程序名组织目录结构
        date_str = datetime.now().strftime("%Y-%m-%d")
        program_dir = self.log_dir / container_key / date_str
        program_dir.mkdir(parents=True, exist_ok=True)

        return program_dir / f"{command_name}.log"

    def _get_log_file_handle(self, container_key: str, command_name: str):
        """获取或创建日志文件句柄"""
        key = f"{container_key}:{command_name}"

        with self._file_lock:
            if key not in self.log_files:
                log_path = self._get_log_file_path(container_key, command_name)
                try:
                    file_handle = open(log_path, "a", encoding="utf-8")
                    self.log_files[key] = {
                        "file": file_handle,
                        "path": log_path,
                        "size": log_path.stat().st_size if log_path.exists() else 0,
                        "last_write": time.time(),
                    }
                except Exception as e:
                    logger.error(f"打开日志文件失败: {log_path}, 错误: {e}")
                    raise

            return self.log_files[key]

    def _rotate_log_file(self, container_key: str, command_name: str):
        """轮转日志文件"""
        key = f"{container_key}:{command_name}"

        with self._file_lock:
            if key in self.log_files:
                try:
                    # 关闭当前文件
                    self.log_files[key]["file"].close()

                    # 重命名当前文件（使用简单的时间戳，避免累积）
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    old_path = self.log_files[key]["path"]
                    new_path = old_path.parent / f"{command_name}_{timestamp}.log"
                    old_path.rename(new_path)

                    logger.info(f"日志文件轮转: {old_path.name} -> {new_path.name}")

                    # 创建新文件
                    new_file = open(old_path, "w", encoding="utf-8")
                    self.log_files[key]["file"] = new_file
                    self.log_files[key]["path"] = old_path
                    self.log_files[key]["size"] = 0
                except Exception as e:
                    logger.error(f"日志文件轮转失败: {key}, 错误: {e}")
                    # 尝试创建新文件以继续日志记录
                    try:
                        log_path = self._get_log_file_path(container_key, command_name)
                        new_file = open(log_path, "a", encoding="utf-8")
                        self.log_files[key]["file"] = new_file
                        self.log_files[key]["size"] = 0
                    except Exception as inner_e:
                        logger.error(f"恢复日志文件失败: {inner_e}")

    def _process_log_data(self, log_data: Dict[str, Any]):
        """处理日志数据"""
        container_key = log_data["container_key"]
        command_name = log_data["command_name"]
        log_line = log_data["log_line"]
        timestamp = log_data["timestamp"]

        # 格式化日志行 - 简化格式，避免重复时间戳
        formatted_line = f"{log_line}\n"
        # 写入文件
        try:
            log_info = self._get_log_file_handle(container_key, command_name)
            log_info["file"].write(formatted_line)
            log_info["file"].flush()
            log_info["size"] += len(formatted_line.encode("utf-8"))
            log_info["last_write"] = time.time()

            # 检查文件大小，必要时轮转
            if log_info["size"] > self.max_file_size:
                self._rotate_log_file(container_key, command_name)

        except Exception as e:
            logger.error(f"写入日志文件失败: {e}")

    def add_log(self, container_key: str, command_name: str, log_line: str):
        """
        添加日志

        Args:
            container_key: 容器键值
            command_name: 命令名称
            log_line: 日志行内容
        """
        log_data = {
            "container_key": container_key,
            "command_name": command_name,
            "log_line": log_line,
            "timestamp": datetime.now().isoformat(),
        }

        # 异步处理日志（队列满时丢弃旧日志）
        try:
            self.log_queue.put_nowait(log_data)
        except queue.Full:
            logger.warning(f"日志队列已满，丢弃日志: {container_key}:{command_name}")
            # 可选：尝试清理一些旧日志
            try:
                self.log_queue.get_nowait()
                self.log_queue.put_nowait(log_data)
            except:
                pass

    def get_log_files(self, container_key: str, command_name: str) -> List[Path]:
        """
        获取指定程序的日志文件列表

        Args:
            container_key: 容器键值
            command_name: 命令名称

        Returns:
            List[Path]: 日志文件路径列表
        """
        container_dir = self.log_dir / container_key
        if not container_dir.exists():
            return []

        log_files = []
        for date_dir in container_dir.iterdir():
            if date_dir.is_dir():
                log_file = date_dir / f"{command_name}.log"
                if log_file.exists():
                    log_files.append(log_file)

        return sorted(log_files, key=lambda x: x.stat().st_mtime, reverse=True)

    def read_log_file(self, log_file_path: Path, lines: Optional[int] = None) -> List[str]:
        """
        读取日志文件

        Args:
            log_file_path: 日志文件路径
            lines: 读取行数，None表示读取全部

        Returns:
            List[str]: 日志行列表
        """
        try:
            with open(log_file_path, "r", encoding="utf-8") as f:
                if lines:
                    return f.readlines()[-lines:]
                else:
                    return f.readlines()
        except Exception as e:
            logger.error(f"读取日志文件失败: {e}")
            return []

    def get_containers(self) -> List[Dict[str, Any]]:
        """
        获取所有容器信息

        Returns:
            List[Dict]: 容器信息列表
        """
        containers = []

        if not self.log_dir.exists():
            return containers

        for container_dir in self.log_dir.iterdir():
            if container_dir.is_dir():
                container_key = container_dir.name

                # 获取该容器的所有命令
                commands = set()
                for date_dir in container_dir.iterdir():
                    if date_dir.is_dir():
                        for log_file in date_dir.glob("*.log"):
                            base_name = self._extract_base_command_name(log_file.stem)
                            commands.add(base_name)

                containers.append({"key": container_key, "commands": sorted(list(commands))})

        return containers

    def close_all(self):
        """关闭所有日志文件句柄"""
        with self._file_lock:
            for key, log_info in self.log_files.items():
                try:
                    if "file" in log_info and log_info["file"] and not log_info["file"].closed:
                        log_info["file"].close()
                        logger.info(f"关闭日志文件: {key}")
                except Exception as e:
                    logger.error(f"关闭日志文件失败: {key}, 错误: {e}")
            self.log_files.clear()


# 全局单例实例
_instance = None


def get_log_manager() -> LogManager:
    """获取日志管理器单例"""
    global _instance
    if _instance is None:
        _instance = LogManager()
    return _instance
