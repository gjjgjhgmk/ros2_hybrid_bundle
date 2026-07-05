#!/bin/bash
# Docker 构建脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 切换到工作空间根目录
cd "$SCRIPT_DIR/.."

echo "=== 构建 UR Move Docker 镜像 ==="
docker compose -f docker/docker-compose.yml build

echo ""
echo "=== 构建完成 ==="
echo "使用以下命令启动容器:"
echo "  docker compose up ur-move-server"
echo ""
echo "进入容器:"
echo "  docker compose exec ur-move-server bash"

