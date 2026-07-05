#!/usr/bin/env python3
"""
容器管理器
基于配置文件管理Docker容器和程序
"""

import os
import yaml
import subprocess
import docker
import time
import signal
import sys
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Optional, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from log_manager import LogManager
from docker import DockerClient
from docker.models.containers import Container
from loguru import logger

# 导入公共工具模块
from common_utils import setup_logger, check_docker_environment, resolve_path_with_fallback

# 配置loguru日志格式
setup_logger(check_docker_environment())

from log_manager import get_log_manager


class ContainerManager:
    """容器管理器类"""

    # 常量定义
    CONTAINER_START_WAIT = 2  # 容器启动等待时间（秒）
    LOG_THREAD_TIMEOUT = 2.0  # 日志线程超时时间（秒）
    STOP_TIMEOUT = 3.0  # 停止操作超时时间（秒）
    PARALLEL_STOP_TIMEOUT = 3.0  # 并行停止超时时间（秒）
    MAX_PARALLEL_WORKERS = 12  # 最大并行工作线程数

    def __init__(self, config_path: str, log_manager: Optional["LogManager"] = None):
        """
        初始化容器管理器

        Args:
            config_path: 配置文件路径
            log_manager: 日志管理器实例，如果为None则使用默认的日志管理器
        """
        self.config_path = config_path
        self.config = self._load_config()
        try:
            self.docker_client = docker.from_env()
        except docker.errors.DockerException as e:
            logger.error(f"无法使用默认Docker环境: {e}")
            try:
                self.docker_client = docker.DockerClient(base_url="unix:///var/run/docker.sock")
                logger.info("使用unix socket连接Docker")
            except Exception as inner_e:
                logger.error(f"无法连接Docker: {inner_e}")
                raise
        self.running_containers: Dict[str, Container] = {}
        self.running_processes: Dict[str, Dict[str, Dict]] = {}  # 存储进程信息

        # 线程安全锁：保护共享数据结构
        self._processes_lock = threading.Lock()

        # 设置日志管理器
        if log_manager is not None:
            self.log_manager = log_manager
        else:
            self.log_manager = get_log_manager()  # 使用默认的日志管理器

        self._setup_signal_handlers()

    def _resolve_path_with_fallback(self, path_config: str, fallback_path: str, field_name: str) -> str:
        """
        解析路径配置，支持环境变量展开和回退机制

        Args:
            path_config: 路径配置字符串
            fallback_path: 回退路径
            field_name: 字段名称（用于日志）

        Returns:
            解析后的路径
        """
        return resolve_path_with_fallback(path_config, fallback_path, field_name, Path(self.config_path))

    def _load_config(self) -> Dict[str, Any]:
        """加载配置文件并处理环境变量"""
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)

            # 验证必需字段
            required_fields = [("paths", "project_root"), ("paths", "docker_compose_dir"), ("logging", "log_dir")]

            for section, field in required_fields:
                if section not in config or field not in config[section]:
                    raise ValueError(f"配置文件中缺少{section}.{field}字段")
                if not isinstance(config[section][field], str):
                    raise ValueError(f"{section}.{field}字段必须是字符串")

            # 解析project_root（支持通过配置文件路径设置默认值）
            config_file_path = Path(self.config_path).resolve()
            default_project_root = config_file_path.parent.parent.parent.parent
            project_root = resolve_path_with_fallback(
                config["paths"]["project_root"], str(default_project_root), "project_root", config_file_path
            )

            # 解析其他路径（支持回退）
            docker_compose_dir = resolve_path_with_fallback(
                config["paths"]["docker_compose_dir"],
                str(config_file_path.parent),
                "docker_compose_dir",
                config_file_path,
            )
            log_dir = resolve_path_with_fallback(
                config["logging"]["log_dir"], os.path.join(project_root, "logs"), "log_dir", config_file_path
            )
            # 更新配置
            config["paths"]["project_root"] = project_root
            config["paths"]["docker_compose_dir"] = docker_compose_dir
            config["logging"]["log_dir"] = log_dir
            # 输出配置信息
            logger.info(f"项目根目录: {project_root}")
            logger.info(f"Docker Compose目录: {docker_compose_dir}")
            logger.info(f"日志目录: {log_dir}")

            return config
        except Exception as e:
            logger.error(f"加载配置文件失败: {e}")
            sys.exit(1)

    def _setup_signal_handlers(self):
        """设置信号处理器"""
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """信号处理器"""
        logger.info(f"\n收到信号 {signum}，正在清理...")
        self.cleanup_all()
        sys.exit(0)

    def _get_docker_compose_dir(self) -> str:
        """获取docker-compose目录"""
        compose_dir = self.config["paths"]["docker_compose_dir"]
        # 展开环境变量
        compose_dir = os.path.expandvars(compose_dir)
        return compose_dir

    def _generate_ros2_params(self, params: Dict[str, Any]) -> str:
        """
        生成ROS2参数格式字符串

        Args:
            params: 参数字典

        Returns:
            str: ROS2参数格式字符串，如 "param_name:=value param2:=value2"
        """
        if not params:
            return ""

        param_list = []
        for key, value in params.items():
            if isinstance(value, bool):
                param_value = "true" if value else "false"
            elif isinstance(value, str):
                param_value = value
            else:
                param_value = str(value)
            param_list.append(f"{key}:={param_value}")

        return " ".join(param_list)

    def _generate_argparse_params(self, params: Dict[str, Any]) -> str:
        """
        生成argparse参数格式字符串

        Args:
            params: 参数字典

        Returns:
            str: argparse参数格式字符串，如 "--param_name value --param2 value2"
        """
        if not params:
            return ""

        param_list = []
        for key, value in params.items():
            if isinstance(value, bool):
                if value:  # 只添加True的布尔参数
                    param_list.append(f"--{key}")
            else:
                param_list.append(f"--{key}")
                param_list.append(str(value))

        return " ".join(param_list)

    def _needs_shell_execution(self, command: str) -> bool:
        """
        检测命令是否需要shell执行

        Args:
            command: 要执行的命令

        Returns:
            bool: 是否需要shell执行
        """
        shell_keywords = ["source", "&&", "||", "|", ">", ">>", "<", "<<", ";", "`", "$("]
        return any(keyword in command for keyword in shell_keywords)

    def _wrap_command_for_shell(self, command: str) -> List[str]:
        """
        将命令包装为shell执行格式

        Args:
            command: 原始命令

        Returns:
            List[str]: 包装后的命令列表，用于exec_run
        """
        if self._needs_shell_execution(command):
            return ["bash", "-c", command]
        else:
            # 对于简单命令，直接分割
            return command.split()

    def _run_docker_compose_command(self, command: str, service_name: Optional[str] = None) -> bool:
        """执行docker compose命令"""
        compose_dir = self._get_docker_compose_dir()
        cmd = f"docker compose {command}"
        if service_name:
            cmd += f" {service_name}"

        try:
            result = subprocess.run(cmd, shell=True, cwd=compose_dir, capture_output=True, text=True, check=True)
            return True
        except subprocess.CalledProcessError as e:
            logger.error(f"执行命令失败: {cmd}")
            logger.error(f"错误: {e.stderr}")
            return False

    def _get_container_by_service(self, service_name: str) -> Optional[Container]:
        """根据服务名获取容器"""
        try:
            container = self.docker_client.containers.get(service_name)
            if container:
                return container
            return None
        except docker.errors.NotFound:
            # 容器不存在，这是正常情况，记录为warning
            logger.warning(f"容器不存在: {service_name}")
            return None
        except Exception as e:
            # 其他异常（网络、权限等）记录为error
            logger.error(f"获取容器失败: {e}")
            return None

    def mount_container(self, container_key: str) -> bool:
        """
        挂载容器

        Args:
            container_key: 容器键值（如"海康相机"）

        Returns:
            bool: 是否成功
        """
        if container_key not in self.config["containers"]:
            logger.error(f"未找到容器配置: {container_key}")
            return False

        container_config = self.config["containers"][container_key]
        service_name = container_config["service_name"]

        # 检查容器是否存在及其状态
        container = self._get_container_by_service(service_name)

        if container:
            if container.status == "running":
                logger.info(f"容器 {container_key} 已在运行")
                self.running_containers[container_key] = container
                return True
            else:
                # 容器存在但未运行，使用 start 命令（保持现有配置）
                logger.info(f"启动已存在的容器: {container_key}")
                success = self._run_docker_compose_command("start", service_name)
        else:
            # 容器不存在，使用 up -d 创建并启动
            logger.info(f"创建并启动容器: {container_key}")
            success = self._run_docker_compose_command("up -d", service_name)

        if success:
            # 等待容器启动
            time.sleep(self.CONTAINER_START_WAIT)
            container = self._get_container_by_service(service_name)
            if container:
                self.running_containers[container_key] = container
                logger.info(f"容器 {container_key} 启动成功")
                return True

        return False

    def unmount_container(self, container_key: str) -> bool:
        """
        关闭容器

        Args:
            container_key: 容器键值

        Returns:
            bool: 是否成功
        """
        if container_key not in self.config["containers"]:
            logger.error(f"未找到容器配置: {container_key}")
            return False

        container_config = self.config["containers"][container_key]
        service_name = container_config["service_name"]

        # 停止容器内的程序
        if container_key in self.running_processes:
            self.stop_all_programs(container_key)

        # 停止容器
        logger.info(f"停止容器: {container_key}")
        success = self._run_docker_compose_command("stop", service_name)
        if success:
            if container_key in self.running_containers:
                del self.running_containers[container_key]
            logger.info(f"容器 {container_key} 已停止")
            return True

        return False

    def remove_container(self, container_key: str) -> bool:
        """
        删除容器

        Args:
            container_key: 容器键值

        Returns:
            bool: 是否成功
        """
        if container_key not in self.config["containers"]:
            logger.error(f"未找到容器配置: {container_key}")
            return False

        container_config = self.config["containers"][container_key]
        service_name = container_config["service_name"]

        # 先停止容器
        self.unmount_container(container_key)

        # 删除容器
        logger.info(f"删除容器: {container_key}")
        success = self._run_docker_compose_command("rm -f", service_name)
        if success:
            logger.info(f"容器 {container_key} 已删除")
            return True

        return False

    def start_program(self, container_key: str, command_name: str) -> bool:
        """
        启动容器内的程序

        Args:
            container_key: 容器键值
            command_name: 命令名称

        Returns:
            bool: 是否成功
        """
        if container_key not in self.config["containers"]:
            logger.error(f"未找到容器配置: {container_key}")
            return False

        container_config = self.config["containers"][container_key]

        # 查找命令配置
        command_config = None
        for cmd in container_config["commands"]:
            if cmd["name"] == command_name:
                command_config = cmd
                break

        if not command_config:
            logger.error(f"未找到命令配置: {command_name}")
            return False

        # 确保容器已挂载
        if container_key not in self.running_containers:
            if not self.mount_container(container_key):
                return False

        container = self.running_containers[container_key]

        # 构建环境变量
        env_vars = {}
        env_vars.update(container_config.get("environment", {}))
        env_vars.update(command_config.get("environment", {}))

        # 构建命令
        full_command = command_config["command"]

        # 生成参数并追加到命令
        param_type = command_config.get("param_type")
        params = command_config.get("params", {})

        if param_type and params:
            if param_type == "ros2":
                param_string = self._generate_ros2_params(params)
            elif param_type == "argparse":
                param_string = self._generate_argparse_params(params)
            else:
                logger.warning(f"未知的参数类型 {param_type}，跳过参数生成")
                param_string = ""

            if param_string:
                full_command = f"{full_command} {param_string}"
                logger.debug(f"生成的参数: {param_string}")

        # 执行命令
        logger.info(f"启动程序: {container_key} - {command_name}")

        # 线程安全地检查程序是否已经在运行
        with self._processes_lock:
            if container_key in self.running_processes and command_name in self.running_processes[container_key]:
                logger.warning(f"程序 {container_key} - {command_name} 已经在运行")
                logger.debug(f"当前运行的程序: {list(self.running_processes.keys())}")
                for key, progs in self.running_processes.items():
                    logger.debug(f"  {key}: {list(progs.keys())}")
                return False

            # 清理可能残留的记录（防御性编程）
            if container_key in self.running_processes:
                if command_name in self.running_processes[container_key]:
                    logger.warning(f"发现残留记录，清理: {container_key} - {command_name}")
                    del self.running_processes[container_key][command_name]

        try:
            # 使用container.exec_run执行命令
            detach_mode = command_config.get("detach", True)
            interactive_mode = command_config.get("interactive", False)

            # 检测是否需要shell执行并包装命令
            exec_command = self._wrap_command_for_shell(full_command)
            logger.debug(f"执行命令: {exec_command}")

            if detach_mode:
                # 后台运行程序，使用exec_create + exec_start
                try:
                    # 使用exec_create创建执行实例
                    exec_create_result = self.docker_client.api.exec_create(
                        container.id,
                        exec_command,
                        environment=env_vars,
                        stdout=True,
                        stderr=True,
                        tty=interactive_mode,
                    )
                    logger.debug(f"exec_create_result: {exec_create_result}")
                    # 获取真实的exec_id
                    exec_id = exec_create_result["Id"]
                    logger.debug(f"创建的exec_id: {exec_id}")

                    # 使用exec_start启动执行（流式模式）
                    exec_start_result = self.docker_client.api.exec_start(
                        exec_id,
                        detach=False,  # 设置为False以获取流式输出
                        stream=True,
                        tty=interactive_mode,
                    )

                    # 创建停止标志
                    stop_event = threading.Event()

                    # 启动日志捕获线程
                    def log_capture_thread(cont_key, cmd_name, stop_flag, exec_stream):
                        try:
                            logger.info(f"日志捕获线程启动: {cont_key} - {cmd_name}")

                            # 使用非阻塞方式读取，避免长时间阻塞
                            import select
                            import sys

                            while not stop_flag.is_set():
                                try:
                                    # 使用真正的超时读取，避免无限阻塞
                                    chunk = None
                                    try:
                                        # 使用select实现超时读取（0.1秒超时）
                                        import select
                                        import sys

                                        # 检查exec_stream是否有数据可读
                                        if hasattr(exec_stream, "fileno"):
                                            ready, _, _ = select.select([exec_stream], [], [], 0.1)
                                            if ready:
                                                chunk = next(exec_stream, None)
                                        else:
                                            # 对于没有fileno的对象，使用短时间等待
                                            chunk = next(exec_stream, None)

                                    except (OSError, AttributeError):
                                        # select不可用或对象不支持，使用默认方式
                                        chunk = next(exec_stream, None)

                                    if chunk is None:
                                        # 没有数据，短暂休眠后继续循环检查停止标志
                                        time.sleep(0.001)
                                        continue

                                    if chunk:
                                        log_line = chunk.decode("utf-8").strip()
                                        if log_line:
                                            # 发送到日志管理器
                                            self.log_manager.add_log(cont_key, cmd_name, log_line)

                                    # 短暂休眠，避免CPU占用过高
                                    time.sleep(0.01)

                                except StopIteration:
                                    logger.info(f"exec_stream迭代结束: {cont_key} - {cmd_name}")
                                    break
                                except Exception as e:
                                    logger.warning(f"日志读取异常: {e}")
                                    break

                            logger.info(f"日志捕获线程自然退出: {cont_key} - {cmd_name}")
                        except Exception as e:
                            logger.error(f"日志捕获线程错误: {e}")
                            logger.error(f"container_key: {cont_key}, command_name: {cmd_name}")

                    # 启动日志捕获线程
                    log_thread = threading.Thread(
                        target=log_capture_thread,
                        args=(container_key, command_name, stop_event, exec_start_result),
                        daemon=True,
                    )
                    log_thread.start()

                    # 线程安全地记录进程信息
                    with self._processes_lock:
                        if container_key not in self.running_processes:
                            self.running_processes[container_key] = {}
                        self.running_processes[container_key][command_name] = {
                            "exec_id": exec_id,
                            "container": container,
                            "command_key_word": command_config.get("command_key_word"),
                            "command": full_command,
                            "detach": True,
                            "interactive": interactive_mode,
                            "start_time": time.time(),
                            "log_thread": log_thread,
                            "exec_stream": exec_start_result,
                            "stop_event": stop_event,
                        }

                    logger.info(
                        f"程序 {container_key} - {command_name} 启动成功 (后台运行, 交互式: {interactive_mode}, exec_id: {exec_id})"
                    )

                except Exception as e:
                    logger.error(f"启动后台程序失败: {e}")
                    return False
            else:
                # 前台程序，直接执行并等待完成
                result = container.exec_run(
                    exec_command, environment=env_vars, stdout=True, stderr=True, tty=interactive_mode
                )

                # 输出结果并记录到日志
                if result.output:
                    output_text = result.output.decode("utf-8")
                    logger.info(f"[{container_key}] 输出:")
                    logger.info(output_text)

                    # 将输出保存到日志文件
                    for line in output_text.split("\n"):
                        if line.strip():
                            self.log_manager.add_log(container_key, command_name, line.strip())

                if result.exit_code != 0:
                    logger.error(f"程序 {container_key} - {command_name} 执行失败，退出码: {result.exit_code}")
                    return False

                logger.info(f"程序 {container_key} - {command_name} 执行完成 (交互式: {interactive_mode})")
                return True

            return True

        except Exception as e:
            logger.error(f"启动程序失败: {e}")
            return False

    def stop_program(self, container_key: str, command_name: str) -> bool:
        """
        停止容器内的程序

        Args:
            container_key: 容器键值
            command_name: 命令名称

        Returns:
            bool: 是否成功
        """
        # 线程安全地获取进程信息
        with self._processes_lock:
            if container_key not in self.running_processes:
                logger.warning(f"容器 {container_key} 没有运行的程序")
                return False

            if command_name not in self.running_processes[container_key]:
                logger.warning(f"程序 {command_name} 没有运行")
                return False

            process_info = self.running_processes[container_key][command_name].copy()

        try:
            logger.info(f"正在停止程序 {container_key} - {command_name}...")

            if process_info.get("detach", False):
                # 后台程序，停止日志捕获线程和进程
                log_thread = process_info.get("log_thread")
                exec_stream = process_info.get("exec_stream")
                exec_id = process_info.get("exec_id")

                # 停止日志捕获线程（通过设置标志或中断）
                stop_event = process_info.get("stop_event")
                if stop_event:
                    stop_event.set()
                    logger.info(f"已设置停止标志: {container_key} - {command_name}")

                if log_thread and log_thread.is_alive():
                    log_thread.join(timeout=self.LOG_THREAD_TIMEOUT)

                # 停止流式执行的进程
                if exec_stream:
                    try:
                        # 关闭流式输出
                        if hasattr(exec_stream, "close"):
                            exec_stream.close()
                        logger.info(f"已关闭exec_stream流: {container_key} - {command_name}")
                    except Exception as e:
                        logger.warning(f"关闭exec_stream失败: {e}")

                # 根据 command_key_word 查找进程
                command_key_word = process_info.get("command_key_word")
                logger.info(f"command_key_word: {command_key_word}")
                if command_key_word:
                    container = process_info.get("container")
                    if container:
                        pgrep_result = container.exec_run(f"pgrep -f '{command_key_word}'")
                        output = pgrep_result.output.decode("utf-8").strip()
                        # 过滤空字符串
                        pid_list = [pid for pid in output.split("\n") if pid.strip()]
                        if pid_list:
                            for pid in pid_list:
                                container.exec_run(f"kill -9 {pid}")
                                logger.info(f"已 kill 程序 {command_name} 的PID={pid} (exec_id={exec_id})")
                        else:
                            logger.warning(f"没有找到进程: {command_key_word}")
                    else:
                        logger.warning(f"容器不存在: {container_key} - {command_name}")

                logger.info(f"程序 {container_key} - {command_name} 已停止")
            else:
                # 前台程序通常已经完成，直接清理记录
                logger.info(f"程序 {container_key} - {command_name} 已清理")

            # 线程安全地从记录中移除
            with self._processes_lock:
                if container_key in self.running_processes and command_name in self.running_processes[container_key]:
                    del self.running_processes[container_key][command_name]
                    logger.debug(f"已从running_processes中移除: {container_key} - {command_name}")

                    logger.debug(f"剩余运行的程序: {list(self.running_processes.keys())}")
                    for key, progs in self.running_processes.items():
                        logger.debug(f"  {key}: {list(progs.keys())}")

            return True

        except Exception as e:
            logger.error(f"停止程序失败: {e}")
            # 不强制清理记录，保持状态一致性
            # 让用户知道程序停止失败，需要手动处理
            logger.warning(f"程序 {container_key} - {command_name} 停止失败，记录保持原状")
            return False

    def stop_all_programs(self, container_key: str) -> bool:
        """
        停止容器内的所有程序

        Args:
            container_key: 容器键值

        Returns:
            bool: 是否成功
        """
        if container_key not in self.running_processes:
            return True

        success = True
        for command_name in list(self.running_processes[container_key].keys()):
            if not self.stop_program(container_key, command_name):
                success = False

        return success

    def stop_all_programs_parallel(self) -> bool:
        """
        并行停止所有程序（超时3秒）

        Returns:
            bool: 是否成功
        """
        if not self.running_processes:
            logger.info("没有运行中的程序需要停止")
            return True

        logger.info("并行停止所有程序（超时3秒）...")

        all_tasks = []
        # 创建字典的副本，避免在迭代时字典被修改
        try:
            running_processes_copy = dict(self.running_processes)
        except RuntimeError:
            logger.warning("无法复制running_processes，跳过程序停止")
            return True

        for container_key, programs in running_processes_copy.items():
            try:
                programs_copy = dict(programs)
                for command_name in programs_copy.keys():
                    all_tasks.append((container_key, command_name))
            except RuntimeError:
                logger.warning(f"无法复制容器 {container_key} 的程序列表，跳过")
                continue

        if not all_tasks:
            return True

        success_count = 0
        failed_tasks = []

        # 使用线程池并行停止所有程序
        with ThreadPoolExecutor(max_workers=min(len(all_tasks), self.MAX_PARALLEL_WORKERS)) as executor:
            # 提交所有停止任务
            future_to_task = {
                executor.submit(self.stop_program, container_key, command_name): (container_key, command_name)
                for container_key, command_name in all_tasks
            }

            # 等待所有任务完成
            try:
                for future in as_completed(future_to_task, timeout=self.PARALLEL_STOP_TIMEOUT):
                    container_key, command_name = future_to_task[future]
                    try:
                        result = future.result()
                        if result:
                            success_count += 1
                            logger.info(f"程序 {container_key}-{command_name} 停止成功")
                        else:
                            failed_tasks.append((container_key, command_name))
                            logger.warning(f"程序 {container_key}-{command_name} 停止失败")
                    except Exception as e:
                        failed_tasks.append((container_key, command_name))
                        logger.error(f"停止程序 {container_key}-{command_name} 时发生异常: {e}")
            except TimeoutError:
                logger.warning("程序停止超时（3秒），强制继续下一步")
                # 取消未完成的任务
                for future in future_to_task:
                    if not future.done():
                        future.cancel()
                        container_key, command_name = future_to_task[future]
                        failed_tasks.append((container_key, command_name))
                        logger.warning(f"程序 {container_key}-{command_name} 因超时被强制停止")

                # 强制停止所有未完成的程序
                logger.info("强制停止所有未完成的程序...")
                self._force_stop_all_programs()

        logger.info(f"并行停止程序完成: {success_count}/{len(all_tasks)} 个程序成功停止")
        if failed_tasks:
            logger.warning(f"停止失败的程序: {failed_tasks}")

        return len(failed_tasks) == 0

    def _force_stop_all_programs(self):
        """
        强制停止所有程序（超时后的最后手段）
        """
        logger.info("执行强制停止所有程序...")

        # 创建字典的副本，避免在迭代时字典被修改
        try:
            running_processes_copy = dict(self.running_processes)
        except RuntimeError:
            logger.warning("无法复制running_processes，跳过强制停止")
            return

        for container_key, programs in running_processes_copy.items():
            try:
                # 创建程序字典的副本
                programs_copy = dict(programs)
            except RuntimeError:
                logger.warning(f"无法复制容器 {container_key} 的程序列表，跳过")
                continue

            for command_name, process_info in programs_copy.items():
                try:
                    command_key_word = process_info.get("command_key_word")
                    if command_key_word:
                        container = process_info.get("container")
                        if container:
                            # 强制杀死所有匹配的进程
                            logger.info(f"强制停止程序: {container_key}-{command_name}")
                            container.exec_run(f"pkill -9 -f '{command_key_word}'")
                            logger.info(f"已强制停止程序 {container_key}-{command_name}")
                except Exception as e:
                    logger.warning(f"强制停止程序 {container_key}-{command_name} 失败: {e}")

    def stop_and_remove_containers_parallel(self) -> bool:
        """
        并行停止并删除所有容器

        Returns:
            bool: 是否成功
        """
        container_keys = list(self.config["containers"].keys())
        if not container_keys:
            logger.info("没有容器需要停止和删除")
            return True

        logger.info(f"并行停止并删除所有容器: {container_keys}")

        success_count = 0
        failed_containers = []

        # 使用线程池并行停止并删除所有容器
        with ThreadPoolExecutor(max_workers=min(len(container_keys), 4)) as executor:
            # 提交所有停止和删除任务
            future_to_container = {
                executor.submit(self._stop_and_remove_single_container, container_key): container_key
                for container_key in container_keys
            }

            # 等待所有任务完成
            for future in as_completed(future_to_container, timeout=10.0):
                container_key = future_to_container[future]
                try:
                    result = future.result()
                    if result:
                        success_count += 1
                        logger.info(f"容器 {container_key} 停止并删除成功")
                    else:
                        failed_containers.append(container_key)
                        logger.warning(f"容器 {container_key} 停止并删除失败")
                except Exception as e:
                    failed_containers.append(container_key)
                    logger.error(f"停止并删除容器 {container_key} 时发生异常: {e}")

        logger.info(f"并行停止并删除容器完成: {success_count}/{len(container_keys)} 个容器成功处理")
        if failed_containers:
            logger.warning(f"处理失败的容器: {failed_containers}")

        return len(failed_containers) == 0

    def _stop_and_remove_single_container(self, container_key: str) -> bool:
        """
        停止并删除单个容器

        Args:
            container_key: 容器键值

        Returns:
            bool: 是否成功
        """
        try:
            container_config = self.config["containers"][container_key]
            service_name = container_config["service_name"]

            # 停止容器
            logger.info(f"停止容器: {container_key}")
            stop_success = self._run_docker_compose_command("stop --timeout 3", service_name)

            if stop_success:
                # 删除容器
                logger.info(f"删除容器: {container_key}")
                remove_success = self._run_docker_compose_command("rm -f", service_name)

                if remove_success:
                    # 清理内存中的状态记录
                    if container_key in self.running_containers:
                        del self.running_containers[container_key]
                    if container_key in self.running_processes:
                        del self.running_processes[container_key]
                    return True
                else:
                    logger.error(f"删除容器失败: {container_key}")
                    return False
            else:
                logger.error(f"停止容器失败: {container_key}")
                return False

        except Exception as e:
            logger.error(f"处理容器 {container_key} 时发生异常: {e}")
            return False

    def auto_start_containers(self) -> bool:
        """自动启动配置为auto_up的容器"""
        success = True
        for container_key, container_config in self.config["containers"].items():
            if container_config.get("auto_up", False):
                if not self.mount_container(container_key):
                    success = False

        return success

    def auto_start_programs(self) -> bool:
        """自动启动配置为auto_start的程序，但只有在容器允许auto_up的情况下才启动"""
        success = True
        for container_key, container_config in self.config["containers"].items():
            # 检查容器是否允许auto_up
            if not container_config.get("auto_up", False):
                logger.info(f"容器 {container_key} 未启用 auto_up，跳过自动启动程序")
                continue

            for command_config in container_config["commands"]:
                if command_config.get("auto_start", False):
                    logger.info(f"正在自动启动容器 {container_key} 中的程序 {command_config['name']}")
                    if not self.start_program(container_key, command_config["name"]):
                        success = False

        return success

    def cleanup_all(self):
        """清理所有容器和程序（并行优化版本）"""
        logger.info("正在并行清理所有容器和程序...")

        # 1. 并行停止所有程序（超时3秒）
        logger.info("步骤1: 并行停止所有程序（超时3秒）")
        try:
            self.stop_all_programs_parallel()
        except Exception as e:
            logger.warning(f"程序停止过程中发生异常: {e}，继续执行容器停止")

        # 2. 并行停止并删除所有容器
        logger.info("步骤2: 并行停止并删除所有容器")
        try:
            self.stop_and_remove_containers_parallel()
        except Exception as e:
            logger.error(f"容器停止过程中发生异常: {e}")

        logger.info("并行清理完成")

    def get_status(self) -> Dict[str, Any]:
        """获取状态信息"""
        status = {"containers": {}, "processes": {}}

        # 容器状态
        for container_key in self.config["containers"].keys():
            container_config = self.config["containers"][container_key]
            service_name = container_config["service_name"]
            container = self._get_container_by_service(service_name)

            status["containers"][container_key] = {
                "service_name": service_name,
                "running": container is not None and container.status == "running",
                "status": (
                    "running"
                    if container and container.status == "running"
                    else "stopped" if container else "not_found"
                ),
            }

        # 程序状态 - 遍历所有配置的程序
        for container_key, container_config in self.config["containers"].items():
            status["processes"][container_key] = {}

            # 遍历该容器的所有命令
            for command_config in container_config["commands"]:
                command_name = command_config["name"]

                # 检查程序是否正在运行
                is_running = (
                    container_key in self.running_processes and command_name in self.running_processes[container_key]
                )

                if is_running:
                    # 程序记录存在，需要检查是否真的在运行
                    process_info = self.running_processes[container_key][command_name]
                    exec_id = process_info.get("exec_id")

                    # 检查后台进程的真实状态
                    actually_running = True
                    if process_info.get("detach", False) and exec_id:
                        try:
                            exec_info = self.docker_client.api.exec_inspect(exec_id)
                            actually_running = exec_info.get("Running", False)

                            # 如果进程已停止，清理记录
                            if not actually_running:
                                logger.info(f"发现已停止的进程，清理记录: {container_key} - {command_name}")
                                del self.running_processes[container_key][command_name]
                                # 如果容器没有其他运行的程序，清理容器记录
                                if not self.running_processes[container_key]:
                                    del self.running_processes[container_key]

                        except Exception as e:
                            logger.warning(f"检查进程状态失败: {e}")
                            # 如果检查失败，假设进程仍在运行
                            actually_running = True

                    if actually_running:
                        # 程序正在运行
                        status["processes"][container_key][command_name] = {
                            "running": True,
                            "exec_id": exec_id,
                            "detach": process_info.get("detach", False),
                            "interactive": process_info.get("interactive", False),
                            "start_time": process_info.get("start_time"),
                        }
                    else:
                        # 程序已停止
                        status["processes"][container_key][command_name] = {
                            "running": False,
                            "exec_id": None,
                            "detach": command_config.get("detach", True),
                            "interactive": command_config.get("interactive", False),
                            "start_time": None,
                        }
                else:
                    # 程序未运行
                    status["processes"][container_key][command_name] = {
                        "running": False,
                        "exec_id": None,
                        "detach": command_config.get("detach", True),
                        "interactive": command_config.get("interactive", False),
                        "start_time": None,
                    }

        return status

    def list_containers(self):
        """列出所有容器配置"""
        logger.info("可用容器:")
        for container_key, container_config in self.config["containers"].items():
            logger.info(f"  {container_key}: {container_config['service_name']}")
            logger.info(f"    自动启动: {container_config.get('auto_up', False)}")
            logger.info(f"    命令:")
            for cmd in container_config["commands"]:
                logger.info(f"      - {cmd['name']}: {cmd['command']}")

    def run(self):
        """运行容器管理器"""
        logger.info("启动容器管理器...")

        # 自动启动容器
        if not self.auto_start_containers():
            logger.warning("部分容器启动失败")

        # 自动启动程序
        if not self.auto_start_programs():
            logger.warning("部分程序启动失败")

        logger.info("容器管理器启动完成")
        logger.info("按 Ctrl+C 退出")

        try:
            # 保持进程运行，监控子进程
            while True:
                time.sleep(1)
                # 检查是否有进程意外退出（这里简化处理，因为exec_run的监控比较复杂）
                # 实际应用中可以通过检查exec实例状态来判断
                pass
        except KeyboardInterrupt:
            self.cleanup_all()


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="容器管理器")
    parser.add_argument("--config", default="config/docker/config_record.yaml", help="配置文件路径")
    args = parser.parse_args()

    # 创建管理器
    manager = ContainerManager(args.config)
    manager.run()


if __name__ == "__main__":
    main()
