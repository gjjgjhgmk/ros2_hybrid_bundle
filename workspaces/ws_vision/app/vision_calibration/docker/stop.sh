# 切换到脚本所在目录（确保 docker-compose.yml 路径正确）
cd "$(dirname "$0")"

# 1. 停止容器
docker compose down vision_calibration_local
# docker compose down ur_move_server

# 2. 杀死端口占用进程
./kill_port.sh 7001
# ./kill_port.sh 5605