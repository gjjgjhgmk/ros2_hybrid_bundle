#!/bin/bash
# Docker 启动脚本 - 启动 ur-move-server

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 切换到工作空间根目录
cd "$SCRIPT_DIR/.."

SERVICE_NAME="ur-move-server"
COMPOSE_FILE="docker/docker-compose.yml"

echo "=== 启动 UR Move 服务器 ==="
echo "容器名称: $SERVICE_NAME"
echo ""

# 检查镜像是否存在
if ! docker images | grep -q "ur-move-server"; then
    echo "镜像不存在，正在构建..."
    docker compose -f "$COMPOSE_FILE" build
fi

# 启动容器
xhost +local:docker
docker compose -f "$COMPOSE_FILE" up -d "$SERVICE_NAME"

echo ""
echo "=== 容器已启动 ==="
echo "容器名称: $SERVICE_NAME"
echo ""
echo "进入容器:"
echo "  docker compose exec $SERVICE_NAME bash"
echo ""
echo "查看日志:"
echo "  docker logs -f $SERVICE_NAME"
echo ""
echo "停止容器:"
echo "  docker compose stop $SERVICE_NAME"
echo "或停止并删除:"
echo "  docker compose down $SERVICE_NAME"

