#!/bin/bash
set -e

# UR Move Docker 启动脚本
echo "=== UR Move 轨迹规划和执行服务器启动 ==="

# 设置ROS环境
source /opt/ros/${ROS_DISTRO}/setup.bash

# 设置工作空间环境
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
    echo "✓ 工作空间环境已加载"
else
    echo "⚠ 警告: 工作空间环境文件不存在，请先编译工作空间"
fi

# 设置环境变量
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# 显示环境信息
echo "ROS版本: $ROS_DISTRO"
echo "ROS域ID: $ROS_DOMAIN_ID"
echo "RMW实现: $RMW_IMPLEMENTATION"
echo "工作目录: $(pwd)"

# 检查工作空间结构
echo ""
echo "工作空间结构:"
if [ -d "/workspace/src/ur_move" ]; then
    echo "  ✓ ur_move 已挂载"
else
    echo "  ✗ ur_move 未找到"
fi

if [ -d "/workspace/src/dual_arm" ]; then
    echo "  ✓ dual_arm 已挂载"
else
    echo "  ✗ dual_arm 未找到"
fi

# 信号处理函数
cleanup() {
    echo "收到终止信号，正在清理..."
    # 这里可以添加清理逻辑
    exit 0
}

# 设置信号处理
trap cleanup SIGTERM SIGINT

echo ""
echo "=== 启动完成，执行命令: $@ ==="
echo ""

# 执行传入的命令
exec "$@"

