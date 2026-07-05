#!/bin/bash

if [ -z "$1" ]; then
    echo "使用方法: $0 <端口号>"
    exit 1
fi

PORT=$1

echo "查找端口 $PORT 的占用情况..."

# 先检查端口是否被占用（不需要sudo）
if ! ss -tln 2>/dev/null | grep -q ":$PORT " && ! netstat -tln 2>/dev/null | grep -q ":$PORT "; then
    echo "端口 $PORT 未被占用"
    exit 0
fi

# 端口被占用，使用sudo获取PID
PID=$(sudo lsof -t -i :$PORT 2>/dev/null)

if [ -z "$PID" ]; then
    echo "警告: 检测到端口被占用，但无法获取进程信息，尝试强制终止..."
    sudo fuser -k -9 $PORT/tcp 2>/dev/null
    echo "端口 $PORT 已释放"
else
    echo "正在终止进程 $PID..."
    sudo kill -9 $PID
    
    # 验证是否终止成功
    sleep 2
    if sudo lsof -t -i :$PORT > /dev/null; then
        echo "强制终止进程..."
        sudo fuser -k -9 $PORT/tcp
    fi
    
    echo "端口 $PORT 已释放"
fi