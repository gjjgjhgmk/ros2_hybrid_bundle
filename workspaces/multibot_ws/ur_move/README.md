# UR Move - 手臂与夹爪控制功能包

基于 UR7e 和 robotiq 的轨迹规划和执行系统

## 功能特性
- 集成 手臂规划控制 与 夹爪控制功能
- 支持 ZMQ 通信，提供客户端与服务器

## 项目结构

```
ur_move/
├── client/                          # ZMQ 客户端
├── src/                             # ZMQ 服务端            
├── include/ur_move/                 # C++ 头文件
├── launch/                          # 服务器节点启动文件
├── test/                            # 客户端测试文件和配置
├── CMakeLists.txt                   # CMake 构建配置
├── package.xml                      # ROS 2 包配置
└── README.md                        # 项目包说明文档
```

### 组件说明

1. **C++ 服务器（src/server_cpp/）**
   - 轨迹规划服务器，必须依赖 ROS2 环境（Docker）与驱动包
   - 支持多种规划器：`ptp`、`lin`、`ompl`
   - 提供实时碰撞检测
   - 规划与执行分离

2. **Python 服务器（src/server_py/）**
   - 夹爪控制服务器，必须依赖 ROS2 环境（Docker）与驱动包
   - 支持左右手夹爪独立控制

3. **客户端（client/）**
   - 可在本地环境运行，无需 ROS 2
   - 通过 ZMQ 与服务器通信

```

## 快速开始

### 1. 编译

进入 Docker 环境：

```bash
colcon build
source install/setup.bash

### 2. 启动服务器

```bash
# 使用模拟硬件（默认）
ros2 launch ur_move ur_move_server.launch.py use_mock_hardware:=true use_fake_gripper_hardware:=true

# 使用实际机器人硬件
ros2 launch ur_move ur_move_server.launch.py use_mock_hardware:=false use_fake_gripper_hardware:=false
```
**注意**：使用实际硬件时，还需要：
- 设置正确的机器人 IP 地址（通过 `control.launch.py` 的 `left_robot_ip` 和 `right_robot_ip` 参数）
- 确保机器人已连接并处于远程控制模式

### 3. 使用客户端

详细的 API 使用说明、参数说明和代码示例请参考：[客户端 API 文档](client/README.md) 


## 注意事项

1. **服务器要求**
   - 确保 `dual_arm` 和 `dual_arm_moveit_config` 已正确编译
   - 确保驱动包（`ur_robot_driver`、`ur_controllers`、`robotiq_driver`）已正确编译
   - 确保描述包（`ur_description`、`robotiq_description`）已正确编译
   - 确保 ros2_control 节点和控制器正在运行
   - 服务器需要在 Docker 容器中运行（提供 ROS2 环境）

2. **客户端要求**
   - 客户端可以在本地运行
   - 需要安装 `pyzmq` 库：`pip install pyzmq`
   - 确保服务器正在运行且端口可访问

