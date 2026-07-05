# 双臂机器人控制系统

基于 UR7e 和 Robotiq 的双臂机器人控制系统，支持轨迹规划、运动控制和夹爪控制。

## 系统特性

- 🤖 **双臂协调控制**：支持左右臂独立或协调运动
- 🎯 **轨迹规划**：基于 MoveIt2 的运动规划
- 🦾 **夹爪控制**：Robotiq 夹爪独立控制
- 🌳 **行为树系统**：基于 py-trees 的任务编排
- 🐳 **Docker 支持**：容器化部署，环境自动配置
- 🔌 **ZMQ 通信**：分布式架构，支持多机部署

## 快速开始

### 1. 启动 Docker 服务

项目提供了多个 Docker 服务，可根据需要启动：

```bash
cd docker

# 启动 UR Move 服务器（轨迹规划和夹爪控制）
docker compose up -d ur-move-server

# 启动规划服务器（仅轨迹规划）
docker compose up -d ur-plan-server

# 启动左臂驱动服务
docker compose up -d ur-left-driver

# 启动右臂驱动服务
docker compose up -d ur-right-driver
```

### 2. 进入容器并启动服务

```bash
# 进入容器（环境已自动加载，无需手动 source）
docker compose exec -it ur-move-server bash

# 启动服务器（使用模拟硬件）
ros2 launch ur_move ur_move_server.launch.py use_mock_hardware:=true use_fake_gripper_hardware:=true

# 或使用实际机器人硬件
ros2 launch single_arm_config left_arm.launch.py
```

### 3. 使用客户端 API

详细的 API 使用说明、参数说明和代码示例请参考：[客户端 API 文档](ur_move/client/README.md)

## 项目结构

```
multibot_ws/
├── docker/                    # Docker 配置
│   ├── docker-compose.yml     # 服务配置
│   └── start.sh               # 启动脚本
├── ur_move/                   # 轨迹规划和夹爪控制
│   ├── client/                # ZMQ 客户端
│   ├── src/                   # 服务器代码
│   └── launch/                # 启动文件
├── ur_bt/                     # 行为树决策系统
│   ├── src/ur_bt/             # 核心代码
│   ├── example/               # 示例代码
│   └── tasks/                 # 自定义任务
├── single_arm/                # 单臂配置
├── dual_arm/                  # 双臂配置
└── robotiq_driver/            # 夹爪驱动
```

## Docker 服务说明

| 服务名称 | 说明 | ROS_DOMAIN_ID |
|---------|------|---------------|
| `ur-move-server` | UR Move 服务器（轨迹规划 + 夹爪控制） | 66 |
| `ur-plan-server` | 规划服务器（仅轨迹规划） | 27 |
| `ur-left-driver` | 左臂驱动服务 | 18 |
| `ur-right-driver` | 右臂驱动服务 | 19 |

## 更多文档

- [UR Move 详细文档](ur_move/README.md) - 轨迹规划和夹爪控制
- [行为树系统文档](ur_bt/README.md) - 行为树框架使用
- [ZMQ端口设计文档](docs/ZMQ_PORT_DESIGN.md) - 通信端口说明
- [完整项目文档](docs/项目文档.md) - 详细技术文档

## 技术栈

- **机器人框架**: ROS2 Jazzy
- **运动规划**: MoveIt2
- **通信协议**: ZMQ (ZeroMQ)
- **决策系统**: py-trees
- **容器化**: Docker & Docker Compose
- **编程语言**: Python 3, C++
