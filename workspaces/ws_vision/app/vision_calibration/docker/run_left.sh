# 切换到脚本所在目录（确保 docker-compose.yml 路径正确）
cd "$(dirname "$0")"

# 0. 停止容器
./stop.sh

# 1. 启动容器（如果未运行）
docker compose up -d vision_calibration_local

# 等待容器启动
echo "等待容器启动..."
sleep 10
echo "容器启动完成"

# 2. 启动 ROS2 charuco_detector 节点（后台运行，输出到日志文件）
echo "启动 ROS2 charuco_detector 节点..."
docker compose exec -d vision_calibration_local bash -c \
"source /home/yw/.env_noninteractive && export ROS_DOMAIN_ID=27 && \
ros2 launch charuco_detector charuco_detector.launch.py \
  node_name:='charuco_service_node' \
  ros_param_file:='/home/yw/workspace/config/charuco/ros_left.yaml' \
  charuco_param_file:='/home/yw/workspace/config/charuco/charuco.yaml' \
  > /tmp/log/charuco_detector.log 2>&1"

# 3. 启动 REST API 服务器（后台运行，输出到日志文件）
echo "启动 REST API 服务器..."
docker compose exec -d vision_calibration_local bash -c \
"source /home/yw/.env_noninteractive && export ROS_DOMAIN_ID=27 && \
cd /home/yw/workspace/src/vision_calibration && \
python3 -m vision_calibration.rest_api --config '/home/yw/workspace/config/vision_calibration/config_left.yaml' \
  > /tmp/log/rest_api.log 2>&1"

# 4. 启动 UR Move 服务器
# echo "启动 UR Move 容器..."
# docker compose up -d ur_move_server

# echo "等待容器启动..."
# sleep 10
# echo "容器启动完成"

# 5. 启动 UR Move 服务器
# echo "启动 UR Move 服务器..."
# docker compose exec -d ur_move_server bash -c \
# "source install/setup.bash && ros2 launch ur_move ur_move_server.launch.py use_mock_hardware:=true use_fake_gripper_hardware:=true > /tmp/log/ur_move_server.log 2>&1"
