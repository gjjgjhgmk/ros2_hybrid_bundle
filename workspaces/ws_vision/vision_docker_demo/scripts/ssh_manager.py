#!/usr/bin/env python3
"""
SSH连接管理器
提供SSH连接和远程命令执行功能
"""

import paramiko
import socket
import os
import subprocess
from pathlib import Path
from typing import Optional, Dict, Tuple
from loguru import logger


class SSHManager:
    """SSH连接管理器类"""

    def __init__(self):
        """初始化SSH管理器"""
        self.connections: Dict[str, paramiko.SSHClient] = {}

    def connect(
        self,
        host: str,
        port: int = 22,
        username: str = "root",
        password: Optional[str] = None,
        key_filename: Optional[str] = None,
        timeout: int = 10,
    ) -> Tuple[bool, str]:
        """
        建立SSH连接

        Args:
            host: 主机IP地址
            port: SSH端口（默认22）
            username: 用户名
            password: 密码（如果使用密钥认证，可以为None）
            key_filename: SSH密钥文件路径（可选）
            timeout: 连接超时时间（秒）

        Returns:
            (成功标志, 错误信息)
        """
        connection_key = f"{username}@{host}:{port}"

        # 如果连接已存在，先关闭
        if connection_key in self.connections:
            try:
                self.connections[connection_key].close()
            except Exception:
                pass
            del self.connections[connection_key]

        try:
            # 创建SSH客户端
            ssh_client = paramiko.SSHClient()
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # 尝试连接
            ssh_client.connect(
                hostname=host,
                port=port,
                username=username,
                password=password,
                key_filename=key_filename,
                timeout=timeout,
                look_for_keys=False,
                allow_agent=False,
            )

            # 保存连接
            self.connections[connection_key] = ssh_client
            logger.info(f"SSH连接成功: {connection_key}")
            return True, "连接成功"

        except paramiko.AuthenticationException:
            error_msg = f"SSH认证失败: {connection_key}"
            logger.error(error_msg)
            return False, error_msg

        except paramiko.SSHException as e:
            error_msg = f"SSH连接错误: {str(e)}"
            logger.error(error_msg)
            return False, error_msg

        except socket.timeout:
            error_msg = f"SSH连接超时: {connection_key}"
            logger.error(error_msg)
            return False, error_msg

        except Exception as e:
            error_msg = f"SSH连接失败: {str(e)}"
            logger.error(error_msg)
            return False, error_msg

    def execute_command(
        self,
        host: str,
        port: int,
        username: str,
        command: str,
        password: Optional[str] = None,
        key_filename: Optional[str] = None,
        timeout: int = 30,
    ) -> Tuple[bool, str, str, int]:
        """
        执行远程命令

        Args:
            host: 主机IP地址
            port: SSH端口
            username: 用户名
            command: 要执行的命令
            password: 密码（如果使用密钥认证，可以为None）
            key_filename: SSH密钥文件路径（可选）
            timeout: 命令执行超时时间（秒）

        Returns:
            (成功标志, 标准输出, 标准错误, 退出码)
        """
        connection_key = f"{username}@{host}:{port}"

        # 检查连接是否存在
        if connection_key not in self.connections:
            # 尝试连接
            success, error_msg = self.connect(host, port, username, password, key_filename)
            if not success:
                return False, "", error_msg, -1

        try:
            ssh_client = self.connections[connection_key]

            # 执行命令
            logger.info(f"执行SSH命令: {command}")
            stdin, stdout, stderr = ssh_client.exec_command(command, timeout=timeout)

            # 读取输出
            stdout_text = stdout.read().decode("utf-8", errors="ignore")
            stderr_text = stderr.read().decode("utf-8", errors="ignore")
            exit_code = stdout.channel.recv_exit_status()

            logger.info(f"命令执行完成: {command} (退出码: {exit_code})")
            if stdout_text:
                logger.debug(f"标准输出: {stdout_text[:200]}")
            if stderr_text:
                logger.debug(f"标准错误: {stderr_text[:200]}")

            return True, stdout_text, stderr_text, exit_code

        except socket.timeout:
            error_msg = f"命令执行超时: {command[:50]}..."
            logger.error(error_msg)
            return False, "", error_msg, -1

        except Exception as e:
            error_msg = f"命令执行失败: {str(e)}"
            logger.error(error_msg)
            return False, "", error_msg, -1

    def disconnect(self, host: str, port: int, username: str):
        """
        断开SSH连接

        Args:
            host: 主机IP地址
            port: SSH端口
            username: 用户名
        """
        connection_key = f"{username}@{host}:{port}"

        if connection_key in self.connections:
            try:
                self.connections[connection_key].close()
                logger.info(f"SSH连接已关闭: {connection_key}")
            except Exception:
                pass
            finally:
                del self.connections[connection_key]

    def upload_file_or_directory(
        self,
        host: str,
        port: int,
        username: str,
        local_path: str,
        remote_path: str,
        password: Optional[str] = None,
        key_filename: Optional[str] = None,
    ) -> Tuple[bool, str, int]:
        """
        上传本地文件或文件夹到远程主机，支持多种传输模式

        Args:
            host: 主机IP地址
            port: SSH端口
            username: 用户名
            local_path: 本地文件或文件夹路径
            remote_path: 远程目标路径
            password: 密码（如果使用密钥认证，可以为None）
            key_filename: SSH密钥文件路径（可选）

        Returns:
            (成功标志, 错误信息, 传输的文件数量)
        """
        connection_key = f"{username}@{host}:{port}"

        # 检查连接是否存在
        if connection_key not in self.connections:
            # 尝试连接
            success, error_msg = self.connect(host, port, username, password, key_filename)
            if not success:
                return False, error_msg, 0

        try:
            local_path_obj = Path(local_path)
            if not local_path_obj.exists():
                return False, f"本地路径不存在: {local_path}", 0

            file_count = 0
            error_messages = []

            local_is_file = local_path_obj.is_file()
            local_is_dir = local_path_obj.is_dir()

            # 判断远程路径是文件还是目录（通过检查是否以/结尾）
            remote_is_dir = remote_path.endswith("/")

            # 处理不同的传输模式
            if local_is_file:
                if remote_is_dir:
                    # 模式3: 本地文件 -> 远程目录（文件复制到目录下）
                    remote_file_path = f"{remote_path.rstrip('/')}/{local_path_obj.name}"
                    logger.info(f"执行文件上传: 本地文件 -> 远程目录")
                    logger.info(f"  本地路径: {local_path}")
                    logger.info(f"  远程目录: {remote_path}")
                    logger.info(f"  目标文件路径: {remote_file_path}")
                    try:
                        # 确保远程目录存在
                        mkdir_cmd = f"mkdir -p '{remote_path.rstrip('/')}'"
                        logger.info(f"  执行命令: {mkdir_cmd}")
                        mkdir_success, mkdir_stdout, mkdir_stderr, mkdir_exit = self.execute_command(
                            host=host, port=port, username=username, command=mkdir_cmd, password=password, timeout=10
                        )
                        if not mkdir_success:
                            return False, f"创建远程目录失败: {mkdir_stderr}", 0

                        # 使用scp上传文件（通过SSH命令执行）
                        scp_success, scp_stdout, scp_stderr, scp_exit = self._upload_file_via_scp(
                            host=host,
                            port=port,
                            username=username,
                            password=password,
                            local_file=str(local_path_obj),
                            remote_file=remote_file_path,
                        )
                        if scp_success:
                            file_count = 1
                            logger.info(f"文件上传成功: {local_path} -> {remote_file_path}")
                        else:
                            return False, f"SCP上传失败: {scp_stderr}", 0
                    except Exception as e:
                        error_msg = f"上传文件失败: {str(e)}"
                        logger.error(f"上传文件失败详情: {error_msg}")
                        logger.error(f"  异常类型: {type(e).__name__}")
                        logger.error(f"  异常详情: {repr(e)}")
                        return False, error_msg, 0
                else:
                    # 模式2: 本地文件 -> 远程文件（文件到文件）
                    logger.info(f"执行文件上传: 本地文件 -> 远程文件")
                    logger.info(f"  本地路径: {local_path}")
                    logger.info(f"  远程路径: {remote_path}")
                    try:
                        # 先检查远程路径是否存在以及类型
                        ssh_client = self.connections[connection_key]
                        sftp_check = ssh_client.open_sftp()
                        try:
                            remote_stat = sftp_check.stat(remote_path)
                            # 如果远程路径存在且是目录，则上传到该目录下
                            if remote_stat.st_mode & 0o040000:  # 检查是否是目录
                                logger.info(f"  远程路径是目录，调整为: {remote_path}/{local_path_obj.name}")
                                remote_path = f"{remote_path}/{local_path_obj.name}"
                        except IOError:
                            # 远程路径不存在，检查父目录
                            remote_dir = "/".join(remote_path.split("/")[:-1])
                            if remote_dir:
                                try:
                                    sftp_check.stat(remote_dir)
                                except IOError:
                                    # 父目录不存在，创建它
                                    mkdir_cmd = f"mkdir -p '{remote_dir}'"
                                    logger.info(f"  执行命令: {mkdir_cmd}")
                                    mkdir_success, mkdir_stdout, mkdir_stderr, mkdir_exit = self.execute_command(
                                        host=host,
                                        port=port,
                                        username=username,
                                        command=mkdir_cmd,
                                        password=password,
                                        timeout=10,
                                    )
                                    if not mkdir_success:
                                        sftp_check.close()
                                        return False, f"创建远程目录失败: {mkdir_stderr}", 0
                        sftp_check.close()

                        # 使用SFTP上传文件
                        scp_success, scp_stdout, scp_stderr, scp_exit = self._upload_file_via_scp(
                            host=host,
                            port=port,
                            username=username,
                            password=password,
                            local_file=str(local_path_obj),
                            remote_file=remote_path,
                        )
                        if scp_success:
                            file_count = 1
                            logger.info(f"文件上传成功: {local_path} -> {remote_path}")
                        else:
                            return False, f"文件上传失败: {scp_stderr}", 0
                    except Exception as e:
                        error_msg = f"上传文件失败: {str(e)}"
                        logger.error(f"上传文件失败详情: {error_msg}")
                        logger.error(f"  异常类型: {type(e).__name__}")
                        logger.error(f"  异常详情: {repr(e)}")
                        return False, error_msg, 0

            elif local_is_dir:
                if remote_is_dir:
                    # 模式4: 本地文件夹 -> 远程文件夹
                    if remote_path.endswith("/"):
                        # 远程路径以/结尾：本地文件夹拷贝至目标文件夹目录下
                        target_remote_dir = f"{remote_path.rstrip('/')}/{local_path_obj.name}"
                        logger.info(f"执行文件夹上传: 本地文件夹 -> 远程目录（拷贝到目录下）")
                    else:
                        # 远程路径不以/结尾：直接文件夹替换
                        target_remote_dir = remote_path
                        logger.info(f"执行文件夹上传: 本地文件夹 -> 远程文件夹（直接替换）")
                    logger.info(f"  本地路径: {local_path}")
                    logger.info(f"  远程路径: {remote_path}")
                    logger.info(f"  目标远程目录: {target_remote_dir}")

                    try:
                        # 确保目标目录的父目录存在
                        if remote_path.endswith("/"):
                            parent_dir = remote_path.rstrip("/")
                        else:
                            parent_dir = "/".join(remote_path.split("/")[:-1])

                        if parent_dir:
                            mkdir_cmd = f"mkdir -p '{parent_dir}'"
                            logger.info(f"  执行命令: {mkdir_cmd}")
                            mkdir_success, mkdir_stdout, mkdir_stderr, mkdir_exit = self.execute_command(
                                host=host,
                                port=port,
                                username=username,
                                command=mkdir_cmd,
                                password=password,
                                timeout=10,
                            )
                            if not mkdir_success:
                                return False, f"创建远程目录失败: {mkdir_stderr}", 0

                        # 使用scp -r递归上传文件夹（通过SSH命令执行）
                        logger.info(f"  执行SCP递归上传: {str(local_path_obj)} -> {target_remote_dir}")
                        scp_success, scp_stdout, scp_stderr, scp_exit = self._upload_directory_via_scp(
                            host=host,
                            port=port,
                            username=username,
                            password=password,
                            local_dir=str(local_path_obj),
                            remote_dir=target_remote_dir,
                        )
                        if scp_success:
                            # 统计文件数量
                            file_count = self._count_files_in_directory(local_path_obj)
                            logger.info(f"文件夹上传成功: {local_path} -> {target_remote_dir} ({file_count} 个文件)")
                        else:
                            return False, f"SCP上传失败: {scp_stderr}", 0
                    except Exception as e:
                        error_msg = f"上传文件夹失败: {str(e)}"
                        logger.error(f"上传文件夹失败详情: {error_msg}")
                        logger.error(f"  异常类型: {type(e).__name__}")
                        logger.error(f"  异常详情: {repr(e)}")
                        return False, error_msg, 0
                else:
                    # 远程是文件但本地是文件夹，错误
                    return False, "远程路径是文件，但本地路径是文件夹，无法传输", 0

            if error_messages:
                return True, f"部分文件上传失败:\n" + "\n".join(error_messages[:10]), file_count
            else:
                logger.info(f"传输成功: {local_path} -> {remote_path} ({file_count} 个文件)")
                return True, f"成功传输 {file_count} 个文件", file_count

        except Exception as e:
            error_msg = f"传输失败: {str(e)}"
            logger.error(error_msg)
            return False, error_msg, 0

    def _upload_file_via_scp(
        self, host: str, port: int, username: str, password: str, local_file: str, remote_file: str
    ) -> Tuple[bool, str, str, int]:
        """使用paramiko的SFTP上传单个文件（通过SSH连接）"""
        connection_key = f"{username}@{host}:{port}"

        try:
            # 使用已有的SSH连接或创建新连接
            if connection_key not in self.connections:
                success, error_msg = self.connect(host, port, username, password)
                if not success:
                    return False, "", error_msg, -1

            ssh_client = self.connections[connection_key]

            logger.info(f"  使用SFTP上传文件: {local_file} -> {remote_file}")

            # 确保远程文件的父目录存在
            remote_dir = "/".join(remote_file.split("/")[:-1])
            if remote_dir:
                try:
                    sftp = ssh_client.open_sftp()
                    sftp.stat(remote_dir)
                    sftp.close()
                except IOError:
                    # 父目录不存在，创建它
                    mkdir_cmd = f"mkdir -p '{remote_dir}'"
                    logger.info(f"  执行命令创建父目录: {mkdir_cmd}")
                    mkdir_success, mkdir_stdout, mkdir_stderr, mkdir_exit = self.execute_command(
                        host=host, port=port, username=username, command=mkdir_cmd, password=password, timeout=10
                    )
                    if not mkdir_success:
                        return False, "", f"创建远程目录失败: {mkdir_stderr}", -1

            # 使用paramiko的SFTP上传文件
            sftp = ssh_client.open_sftp()
            try:
                sftp.put(local_file, remote_file)
                logger.info(f"  文件上传成功")
                return True, "", "", 0
            finally:
                sftp.close()
        except Exception as e:
            error_msg = f"文件上传异常: {str(e)}"
            logger.error(f"  {error_msg}")
            logger.error(f"  异常类型: {type(e).__name__}")
            logger.error(f"  异常详情: {repr(e)}")
            import traceback

            logger.error(f"  堆栈跟踪: {traceback.format_exc()}")
            return False, "", error_msg, -1

    def _upload_directory_via_scp(
        self, host: str, port: int, username: str, password: str, local_dir: str, remote_dir: str
    ) -> Tuple[bool, str, str, int]:
        """使用paramiko的SFTP递归上传文件夹（通过SSH连接）"""
        connection_key = f"{username}@{host}:{port}"

        try:
            # 使用已有的SSH连接或创建新连接
            if connection_key not in self.connections:
                success, error_msg = self.connect(host, port, username, password)
                if not success:
                    return False, "", error_msg, -1

            ssh_client = self.connections[connection_key]

            logger.info(f"  使用SFTP递归上传文件夹: {local_dir} -> {remote_dir}")

            # 使用paramiko的SFTP递归上传
            sftp = ssh_client.open_sftp()

            def upload_recursive(local_dir_path: Path, remote_dir_path: str):
                """递归上传目录"""
                try:
                    sftp.mkdir(remote_dir_path)
                    logger.debug(f"    创建远程目录: {remote_dir_path}")
                except IOError:
                    # 目录可能已存在
                    pass

                for item in local_dir_path.iterdir():
                    remote_item_path = f"{remote_dir_path}/{item.name}"
                    if item.is_dir():
                        logger.debug(f"    处理子目录: {item} -> {remote_item_path}")
                        upload_recursive(item, remote_item_path)
                    elif item.is_file():
                        logger.debug(f"    上传文件: {item} -> {remote_item_path}")
                        sftp.put(str(item), remote_item_path)

            upload_recursive(Path(local_dir), remote_dir)
            sftp.close()

            logger.info(f"  文件夹上传成功")
            return True, "", "", 0
        except Exception as e:
            error_msg = f"文件夹上传异常: {str(e)}"
            logger.error(f"  {error_msg}")
            logger.error(f"  异常类型: {type(e).__name__}")
            logger.error(f"  异常详情: {repr(e)}")
            return False, "", error_msg, -1

    def _count_files_in_directory(self, directory: Path) -> int:
        """统计目录中的文件数量"""
        count = 0
        try:
            for item in directory.rglob("*"):
                if item.is_file():
                    count += 1
        except Exception:
            pass
        return count

    def upload_directory(
        self,
        host: str,
        port: int,
        username: str,
        local_path: str,
        remote_path: str,
        password: Optional[str] = None,
        key_filename: Optional[str] = None,
    ) -> Tuple[bool, str, int]:
        """
        上传本地文件夹到远程主机（兼容旧接口）

        Args:
            host: 主机IP地址
            port: SSH端口
            username: 用户名
            local_path: 本地文件夹路径
            remote_path: 远程目标路径
            password: 密码（如果使用密钥认证，可以为None）
            key_filename: SSH密钥文件路径（可选）

        Returns:
            (成功标志, 错误信息, 传输的文件数量)
        """
        return self.upload_file_or_directory(host, port, username, local_path, remote_path, password, key_filename)

    def disconnect_all(self):
        """断开所有SSH连接"""
        for connection_key in list(self.connections.keys()):
            try:
                self.connections[connection_key].close()
            except Exception:
                pass
        self.connections.clear()
        logger.info("所有SSH连接已关闭")

    def __del__(self):
        """析构函数，确保连接关闭"""
        self.disconnect_all()


# 全局SSH管理器实例
_ssh_manager = None


def get_ssh_manager() -> SSHManager:
    """获取全局SSH管理器实例"""
    global _ssh_manager
    if _ssh_manager is None:
        _ssh_manager = SSHManager()
    return _ssh_manager
