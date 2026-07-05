import psutil
import subprocess
import yaml
import time
import os
import argparse
from typing import Dict, List, Optional
import logging
from concurrent.futures import ThreadPoolExecutor, as_completed


class ProcessMonitor:
    def __init__(self, config_file: str = "monitor_config.yaml"):
        """
        初始化进程监控器

        Args:
            config_file: YAML配置文件路径
        """
        self.config_file = config_file
        self.config = self._load_config()
        self.logger = self._setup_logger()

    def _load_config(self) -> Dict:
        """加载YAML配置文件"""
        try:
            with open(self.config_file, "r", encoding="utf-8") as file:
                config = yaml.safe_load(file)
                self._validate_config(config)
                return config
        except FileNotFoundError:
            raise FileNotFoundError(f"配置文件 {self.config_file} 不存在")
        except yaml.YAMLError as e:
            raise ValueError(f"配置文件格式错误: {e}")

    def _validate_config(self, config: Dict):
        """验证配置文件格式"""
        if not config:
            raise ValueError("配置文件为空")

        if "monitor_processes" not in config:
            raise ValueError("配置文件中缺少 'monitor_processes' 字段")

        if not isinstance(config["monitor_processes"], list):
            raise ValueError("'monitor_processes' 应该是一个列表")

    def _setup_logger(self) -> logging.Logger:
        """设置日志，可同时输出到控制台和文件（可选）"""
        logger = logging.getLogger("ProcessMonitor")
        if logger.handlers:
            return logger

        monitor_config = self.config.get("monitor_config", {})

        # 设置日志级别
        log_level = getattr(logging, monitor_config.get("log_level", "INFO"), logging.INFO)
        logger.setLevel(log_level)

        # 创建格式器
        formatter = logging.Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s", datefmt="%Y-%m-%d %H:%M:%S"
        )

        # 创建控制台处理器（始终启用）
        console_handler = logging.StreamHandler()
        console_handler.setLevel(log_level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

        # 根据配置决定是否创建文件处理器
        log_to_file = monitor_config.get("log_to_file", True)  # 默认启用文件输出
        if log_to_file:
            log_file = monitor_config.get("log_file", "process_monitor.log")
            # 确保日志目录存在
            log_dir = os.path.dirname(log_file) if os.path.dirname(log_file) else "."
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)

            file_handler = logging.FileHandler(log_file, encoding="utf-8")
            file_handler.setLevel(log_level)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

        return logger

    def _get_single_process_info(self, proc_info: Dict, process_name: str, cpu_interval: float) -> Optional[Dict]:
        """
        获取单个进程的CPU和内存信息（用于多线程并行处理）

        Args:
            proc_info: 进程信息字典
            process_name: 要匹配的进程名
            cpu_interval: CPU采样间隔（秒）

        Returns:
            进程信息字典，如果不匹配则返回None
        """
        try:
            proc = psutil.Process(proc_info["pid"])

            # 获取完整命令行字符串（不截断）
            full_cmdline = " ".join(proc_info["cmdline"]) if proc_info["cmdline"] else ""

            # 检查进程名是否包含目标字符串
            # 1. 检查进程名
            # 2. 检查完整命令行（不截断，完整匹配）
            process_match = process_name.lower() in proc_info["name"].lower() or (
                full_cmdline and process_name.lower() in full_cmdline.lower()
            )

            if not process_match:
                return None

            # 获取CPU使用率（使用指定的采样间隔）
            cpu_percent = proc.cpu_percent(interval=cpu_interval)

            # 获取内存信息
            memory_info = proc.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024
            memory_percent = proc.memory_percent()

            return {
                "pid": proc_info["pid"],
                "name": proc_info["name"],
                "cmdline": full_cmdline,  # 完整命令行，不截断
                "cpu_percent": round(cpu_percent, 2),
                "memory_mb": round(memory_mb, 2),
                "memory_percent": round(memory_percent, 2),
            }

        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            return None
        except Exception:
            return None

    def get_process_cpu_memory(self, process_name: str, cpu_interval: Optional[float] = None) -> List[Dict]:
        """
        获取指定进程名的CPU和内存使用情况（使用多线程并行获取）

        Args:
            process_name: 进程名（支持部分匹配，会匹配完整命令行）
            cpu_interval: CPU采样间隔（秒），如果为None则使用配置中的值或监控间隔

        Returns:
            进程信息列表
        """
        # 获取CPU采样间隔
        if cpu_interval is None:
            monitor_config = self.config.get("monitor_config", {})
            cpu_interval = monitor_config.get("cpu_interval", None)
            if cpu_interval is None:
                # 如果没有配置，使用监控间隔，但最小0.1秒
                monitoring_interval = monitor_config.get("interval", 5)
                cpu_interval = max(0.1, monitoring_interval)

        processes = []
        matched_procs = []

        # 第一步：快速扫描所有进程，找出匹配的进程
        for proc in psutil.process_iter(["pid", "name", "cpu_percent", "memory_info", "memory_percent", "cmdline"]):
            try:
                full_cmdline = " ".join(proc.info["cmdline"]) if proc.info["cmdline"] else ""
                process_match = process_name.lower() in proc.info["name"].lower() or (
                    full_cmdline and process_name.lower() in full_cmdline.lower()
                )
                if process_match:
                    matched_procs.append(proc.info)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue

        # 第二步：使用多线程并行获取CPU使用率（这是耗时的操作）
        if matched_procs:
            with ThreadPoolExecutor(max_workers=min(len(matched_procs), 20)) as executor:
                # 提交所有任务
                future_to_proc = {
                    executor.submit(self._get_single_process_info, proc_info, process_name, cpu_interval): proc_info
                    for proc_info in matched_procs
                }

                # 收集结果
                for future in as_completed(future_to_proc):
                    try:
                        result = future.result()
                        if result:
                            processes.append(result)
                    except Exception:
                        continue

        return processes

    def get_process_gpu_usage(self, process_name: str) -> List[Dict]:
        """
        获取指定进程名的GPU使用情况

        Args:
            process_name: 进程名（支持部分匹配）

        Returns:
            GPU信息列表
        """
        gpu_processes = []

        try:
            # 执行nvidia-smi命令获取GPU进程信息
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-compute-apps=pid,process_name,used_memory,gpu_name,used_gpu_memory",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )

            for line in result.stdout.strip().split("\n"):
                if line:
                    parts = [part.strip() for part in line.split(",")]
                    if len(parts) >= 4 and process_name.lower() in parts[1].lower():
                        gpu_processes.append(
                            {
                                "pid": int(parts[0]),
                                "process_name": parts[1],
                                "gpu_memory_mb": int(parts[2]),
                                "gpu_name": parts[3],
                                "gpu_utilization": int(parts[4]) if len(parts) > 4 else 0,
                            }
                        )

        except FileNotFoundError:
            # nvidia-smi不存在，静默处理（可能没有GPU）
            pass
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
            # 其他错误才记录警告
            self.logger.debug(f"获取GPU信息失败: {e}")

        return gpu_processes

    def get_system_wide_gpu_usage(self) -> Dict:
        """
        获取系统整体GPU使用情况

        Returns:
            GPU整体使用信息
        """
        try:
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=index,name,utilization.gpu,memory.used,memory.total,temperature.gpu",
                    "--format=csv,noheader,nounits",
                ],
                capture_output=True,
                text=True,
                timeout=5,
            )

            gpus = []
            for line in result.stdout.strip().split("\n"):
                if line:
                    parts = [part.strip() for part in line.split(",")]
                    if len(parts) >= 6:
                        gpus.append(
                            {
                                "index": int(parts[0]),
                                "name": parts[1],
                                "gpu_utilization": int(parts[2]),
                                "memory_used_mb": int(parts[3]),
                                "memory_total_mb": int(parts[4]),
                                "temperature": int(parts[5]),
                            }
                        )

            return {"gpus": gpus}

        except FileNotFoundError:
            # nvidia-smi不存在，静默处理（可能没有GPU）
            return {"gpus": []}
        except Exception as e:
            # 其他错误才记录调试信息
            self.logger.debug(f"获取系统GPU信息失败: {e}")
            return {"gpus": []}

    def get_process_bpu_usage(self, process_name: str) -> List[Dict]:
        """
        获取指定进程名的BPU使用情况

        Args:
            process_name: 进程名（支持部分匹配）

        Returns:
            BPU信息列表
        """
        """获取 BPU 核心的总占用率以及详细进程信息"""
        bpu_data = []
        core_num_path = f"/sys/devices/system/bpu/core_num"
        core_num = self._read_file(core_num_path)
        if core_num is None:
            self.logger.debug(f"获取BPU核心数失败")
            return bpu_data
        self.bpu_num = int(core_num)
        for i in range(self.bpu_num):
            bpu_info = {"id": i, "total_ratio": 0, "users": []}

            # 1. 读取总占用率
            ratio_path = f"/sys/devices/system/bpu/bpu{i}/ratio"
            ratio_content = self._read_file(ratio_path)
            if ratio_content and ratio_content.isdigit():
                bpu_info["total_ratio"] = int(ratio_content)
            else:
                bpu_info["total_ratio"] = None  # 离线

            # 2. 读取详细用户进程 (解析 users 文件)
            # 格式参考:
            # *User via BPU core(0)*
            # user              ratio
            # 71546           12
            users_path = f"/sys/devices/system/bpu/bpu{i}/users"
            users_content = self._read_file(users_path)

            if users_content:
                lines = users_content.splitlines()
                for line in lines:
                    parts = line.split()
                    if not parts:
                        continue

                    # 跳过表头和分割线
                    if parts[0].startswith("*") or parts[0].lower() == "user":
                        continue

                    # 解析 PID 和 Ratio
                    if len(parts) >= 2 and parts[0].isdigit():
                        pid = parts[0]
                        u_ratio = parts[1]
                        proc_name = self.get_process_name(int(pid))
                        bpu_info["users"].append({"pid": pid, "name": proc_name, "ratio": int(u_ratio)})

            bpu_data.append(bpu_info)
        return bpu_data

    def _read_file(self, path: str) -> Optional[str]:
        """读取文件内容的辅助函数"""
        try:
            with open(path, "r") as f:
                return f.read().strip()
        except FileNotFoundError:
            pass
        return None

    def get_process_name(self, pid: int) -> str:
        """
        获取指定进程的进程名

        Args:
            pid: 进程ID

        Returns:
            进程名
        """
        try:
            process = psutil.Process(pid)
            return process.name()
        except psutil.NoSuchProcess:
            return "Unknown"

    def get_all_monitor_info(self) -> Dict:
        """
        获取配置文件中所有监控进程的完整信息

        Returns:
            包含所有监控进程信息的字典
        """
        monitor_results = {
            "timestamp": time.time(),
            "timestamp_str": time.strftime("%Y-%m-%d %H:%M:%S"),
            "monitored_processes": [],
        }

        # 获取系统CPU信息
        cpu_count = psutil.cpu_count(logical=True)  # 逻辑CPU核心数
        cpu_count_physical = psutil.cpu_count(logical=False)  # 物理CPU核心数

        # 获取每个CPU核心的频率
        cpu_freqs = psutil.cpu_freq(percpu=True)  # 每个核心的频率列表
        cpu_freq_info = []
        total_freq_mhz = 0.0
        max_freq_mhz = 0.0
        min_freq_mhz = float("inf")

        if cpu_freqs:
            for i, freq in enumerate(cpu_freqs):
                if freq:
                    current_freq = freq.current if freq.current else 0
                    max_freq = freq.max if freq.max else 0
                    min_freq = freq.min if freq.min else 0
                    cpu_freq_info.append(
                        {
                            "core": i,
                            "current_mhz": round(current_freq, 2),
                            "max_mhz": round(max_freq, 2),
                            "min_mhz": round(min_freq, 2),
                        }
                    )
                    total_freq_mhz += current_freq
                    max_freq_mhz = max(max_freq_mhz, max_freq if max_freq > 0 else current_freq)
                    min_freq_mhz = min(min_freq_mhz, min_freq if min_freq > 0 else current_freq)

        # 计算平均频率
        avg_freq_mhz = total_freq_mhz / len(cpu_freq_info) if cpu_freq_info else 0

        monitor_results["cpu_count"] = cpu_count
        monitor_results["cpu_count_physical"] = cpu_count_physical
        monitor_results["cpu_freq_info"] = cpu_freq_info
        monitor_results["cpu_avg_freq_mhz"] = round(avg_freq_mhz, 2)
        monitor_results["cpu_max_freq_mhz"] = round(max_freq_mhz, 2)
        monitor_results["cpu_min_freq_mhz"] = round(min_freq_mhz, 2) if min_freq_mhz != float("inf") else 0
        monitor_results["total_cpu_percent"] = cpu_count * 100.0  # 总CPU资源（如8核=800%）

        # 获取系统GPU信息
        system_gpu_info = self.get_system_wide_gpu_usage()
        monitor_results.update(system_gpu_info)

        # 遍历配置中的每个进程名
        for process_name in self.config["monitor_processes"]:
            cpu_memory_info = self.get_process_cpu_memory(process_name)
            gpu_info = self.get_process_gpu_usage(process_name)
            bpu_info = self.get_process_bpu_usage(process_name)

            # 计算该进程组的总CPU使用率（所有匹配进程的CPU使用率之和）
            total_cpu_usage = sum(proc["cpu_percent"] for proc in cpu_memory_info)
            # 计算相对于总CPU资源的百分比
            cpu_percent_of_total = (
                (total_cpu_usage / monitor_results["total_cpu_percent"]) * 100
                if monitor_results["total_cpu_percent"] > 0
                else 0
            )

            process_info = {
                "process_name": process_name,
                "cpu_memory_info": cpu_memory_info,
                "gpu_info": gpu_info,
                "bpu_info": bpu_info,
                "total_cpu_usage": round(total_cpu_usage, 2),  # 总CPU使用率（所有进程之和）
                "cpu_percent_of_total": round(cpu_percent_of_total, 2),  # 占系统总CPU资源的百分比
            }
            monitor_results["monitored_processes"].append(process_info)

        return monitor_results

    def print_monitor_info(self, monitor_data: Dict):
        """
        打印监控信息

        Args:
            monitor_data: 监控数据
        """
        print(f"\n{'='*80}")
        print(f"监控时间: {monitor_data['timestamp_str']}")
        print(f"{'='*80}")

        # 打印系统CPU信息
        cpu_count = monitor_data.get("cpu_count", 0)
        cpu_count_physical = monitor_data.get("cpu_count_physical", 0)
        total_cpu_percent = monitor_data.get("total_cpu_percent", 0)
        cpu_freq_info = monitor_data.get("cpu_freq_info", [])
        avg_freq_mhz = monitor_data.get("cpu_avg_freq_mhz", 0)
        max_freq_mhz = monitor_data.get("cpu_max_freq_mhz", 0)
        min_freq_mhz = monitor_data.get("cpu_min_freq_mhz", 0)

        print(f"\n系统CPU信息:")
        print(f"  CPU核心数: {cpu_count_physical} 物理核心 / {cpu_count} 逻辑核心")
        print(f"  总CPU资源: {total_cpu_percent:.1f}% (100% × {cpu_count}核心)")

        if cpu_freq_info:
            freq_range = f"{min_freq_mhz:.0f} - {max_freq_mhz:.0f}" if min_freq_mhz > 0 and max_freq_mhz > 0 else "N/A"
            print(f"  CPU频率: 平均 {avg_freq_mhz:.0f} MHz, 范围 {freq_range} MHz")

            # 检查是否有不同频率的核心（大小核架构）
            if len(cpu_freq_info) > 1:
                current_freqs = [f["current_mhz"] for f in cpu_freq_info if f["current_mhz"] > 0]
                if current_freqs:
                    unique_freqs = set(round(f, -2) for f in current_freqs)  # 四舍五入到百位
                    if len(unique_freqs) > 1:
                        print(f"  注意: 检测到不同频率的CPU核心（可能是大小核架构）")
                        # 显示频率分组
                        freq_groups = {}
                        for f in cpu_freq_info:
                            freq_key = round(f["current_mhz"], -2)
                            if freq_key not in freq_groups:
                                freq_groups[freq_key] = []
                            freq_groups[freq_key].append(f["core"])

                        for freq_key, cores in sorted(freq_groups.items()):
                            if freq_key > 0:
                                print(f"    核心 {cores}: {freq_key:.0f} MHz ({len(cores)}个核心)")

        # 打印系统GPU信息
        if monitor_data["gpus"]:
            print("\n系统GPU信息:")
            for gpu in monitor_data["gpus"]:
                memory_usage = (gpu["memory_used_mb"] / gpu["memory_total_mb"]) * 100
                print(f"  GPU {gpu['index']}: {gpu['name']}")
                print(
                    f"    使用率: {gpu['gpu_utilization']}% | "
                    f"显存: {gpu['memory_used_mb']}/{gpu['memory_total_mb']} MB ({memory_usage:.1f}%) | "
                    f"温度: {gpu['temperature']}°C"
                )

        # 打印进程信息
        for process_group in monitor_data["monitored_processes"]:
            process_name = process_group["process_name"]
            cpu_memory_info = process_group["cpu_memory_info"]
            gpu_info = process_group["gpu_info"]
            bpu_info = process_group["bpu_info"]
            total_cpu_usage = process_group.get("total_cpu_usage", 0)
            cpu_percent_of_total = process_group.get("cpu_percent_of_total", 0)

            print(f"\n进程: {process_name}")
            print("-" * 60)

            if not cpu_memory_info:
                print("  未找到运行中的进程")
                continue

            # 打印该进程组的总CPU使用情况
            print(
                f"  总CPU使用: {total_cpu_usage}% / {total_cpu_percent:.1f}% = {cpu_percent_of_total:.2f}% (占系统总CPU资源)"
            )
            print()

            # 合并CPU内存信息和GPU信息
            for proc in cpu_memory_info:
                pid = proc["pid"]
                gpu_data = next((gpu for gpu in gpu_info if gpu["pid"] == pid), None)

                # 要遍历所有BPU核心的用户进程来匹配PID
                matched_bpu_info = []
                for bpu_core in bpu_info:
                    for user in bpu_core.get("users", []):
                        if str(user.get("pid", "")) == str(pid):
                            matched_bpu_info.append({"bpu_id": bpu_core["id"], "ratio": user.get("ratio", 0)})

                print(f"  PID: {pid}")
                print(f"    名称: {proc['name']}")
                if proc["cmdline"]:
                    print(f"    命令行: {proc['cmdline']}")  # 完整命令行，不截断
                print(f"    CPU使用率: {proc['cpu_percent']}% (单进程)")
                print(f"    内存使用: {proc['memory_mb']} MB ({proc['memory_percent']:.2f}%)")

                if gpu_data:
                    print(f"    GPU显存: {gpu_data['gpu_memory_mb']} MB")
                    print(f"    GPU利用率: {gpu_data['gpu_utilization']}%")
                    print(f"    GPU设备: {gpu_data['gpu_name']}")
                else:
                    print(f"    GPU显存: 0 MB")
                    print(f"    GPU利用率: 0%")

                if matched_bpu_info:
                    print(f"    BPU使用情况:")
                    for bpu in matched_bpu_info:
                        print(f"    BPU核心 {bpu['bpu_id']}: 占用率 {bpu['ratio']}%")
                else:
                    print(f"BPU使用情况: 无")

    def start_real_time_monitoring(self, interval: int = 5, duration: Optional[int] = None):
        """
        开始实时监控

        Args:
            interval: 监控间隔（秒）
            duration: 监控持续时间（秒），None表示无限监控
        """
        self.logger.info(f"开始实时监控，间隔: {interval}秒")
        start_time = time.time()

        try:
            while True:
                # 检查是否达到监控时长
                if duration and (time.time() - start_time) >= duration:
                    self.logger.info("监控时长达到，停止监控")
                    break

                # 获取并显示监控信息
                monitor_data = self.get_all_monitor_info()
                self.print_monitor_info(monitor_data)

                # 等待下一次监控
                time.sleep(interval)

        except KeyboardInterrupt:
            self.logger.info("用户中断监控")


def main():
    """主函数，支持命令行参数"""
    parser = argparse.ArgumentParser(description="进程监控工具")
    parser.add_argument(
        "-c",
        "--config",
        type=str,
        default="monitor_config.yaml",
        help="YAML配置文件路径（默认: monitor_config.yaml）",
    )
    parser.add_argument(
        "-i",
        "--interval",
        type=int,
        default=None,
        help="监控间隔（秒），如果不指定则使用配置文件中的值",
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=int,
        default=None,
        help="监控持续时间（秒），如果不指定则无限监控",
    )

    args = parser.parse_args()

    try:
        # 创建进程监控器实例
        monitor = ProcessMonitor(config_file=args.config)

        # 获取监控间隔（优先使用命令行参数，否则使用配置文件中的值）
        monitor_config = monitor.config.get("monitor_config", {})
        interval = args.interval if args.interval is not None else monitor_config.get("interval", 5)

        # 启动实时监控
        monitor.start_real_time_monitoring(interval=interval, duration=args.duration)

    except FileNotFoundError as e:
        print(f"错误: {e}")
        return 1
    except ValueError as e:
        print(f"配置错误: {e}")
        return 1
    except Exception as e:
        print(f"运行错误: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
