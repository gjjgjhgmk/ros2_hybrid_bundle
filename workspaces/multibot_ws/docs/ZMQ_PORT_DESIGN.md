# ZMQ 端口统一设计文档

## 概述

本文档定义了 multibot_ws 项目中所有 ZMQ 通信端口的统一设计规范。

## 端口分配策略

### 端口范围分配
- **5600-5609**: 核心服务端口（UR Move、TF 等）
- **5630-5649**: 夹爪服务端口
- **5650-5659**: 关节状态发布端口
- **5660-5669**: 轨迹执行服务端口

## 端口详细列表

### 1. 核心服务端口 (5600-5609)

| 端口 | 服务名称 | Socket 类型 | 方向 | 说明 | 文件位置 |
|------|---------|------------|------|------|----------|
| 5605 | UR Move 服务器 | REP | 绑定 | 轨迹规划服务器主端口 | `ur_move/client/zmq_ur_move_client.py` |
| 5609 | TF 服务器 | REP | 绑定 | TF 变换发布服务器 | `ur_move/src/server_py/tf_zmq_server.py` |

### 2. 夹爪服务端口 (5630-5649)

| 端口 | 服务名称 | Socket 类型 | 方向 | 说明 | 文件位置 |
|------|---------|------------|------|------|----------|
| 5630 | 左手夹爪服务器 | REP | 绑定 | 左手夹爪控制服务 | `ur_move/src/server_py/gripper_zmq_server.py` |
| 5640 | 右手夹爪服务器 | REP | 绑定 | 右手夹爪控制服务 | `ur_move/src/server_py/gripper_zmq_server.py` |

### 3. 关节状态发布端口 (5650-5659)

| 端口 | 服务名称 | Socket 类型 | 方向 | 说明 | 文件位置 |
|------|---------|------------|------|------|----------|
| 5650 | 左臂关节状态发布 | PUB | 绑定 | 左臂关节状态发布（驱动PC） | `single_arm/single_arm_config/scripts/joint_states_zmq_publisher.py` |
| 5651 | 右臂关节状态发布 | PUB | 绑定 | 右臂关节状态发布（驱动PC） | `single_arm/single_arm_config/scripts/joint_states_zmq_publisher.py` |

**注意**: 规划PC通过 SUB socket 连接到这些端口（`ur_move/src/server_py/joint_states_zmq_relay.py`）

### 4. 轨迹执行服务端口 (5660-5669)

| 端口 | 服务名称 | Socket 类型 | 方向 | 说明 | 文件位置 |
|------|---------|------------|------|------|----------|
| 5660 | 左臂轨迹执行服务器 | REP | 绑定 | 左臂轨迹执行服务（驱动PC） | `single_arm/single_arm_config/scripts/trajectory_executor_server.py` |
| 5661 | 右臂轨迹执行服务器 | REP | 绑定 | 右臂轨迹执行服务（驱动PC） | `single_arm/single_arm_config/scripts/trajectory_executor_server.py` |

**注意**: 规划PC通过 REQ socket 连接到这些端口（`ur_move/client/trajectory_executor_client.py`）

## 通信架构

### 驱动PC → 规划PC

```
驱动PC (左臂)                   规划PC
┌─────────────────┐            ┌─────────────────┐
│ joint_states    │  PUB 5650  │  SUB (Relay)    │
│ zmq_publisher   │ ──────────>│                 │
└─────────────────┘            └─────────────────┘

驱动PC (右臂)                   规划PC
┌─────────────────┐            ┌─────────────────┐
│ joint_states    │  PUB 5651  │  SUB (Relay)    │
│ zmq_publisher   │ ──────────>│                 │
└─────────────────┘            └─────────────────┘
```

### 规划PC → 驱动PC

```
规划PC                          驱动PC (左臂)
┌─────────────────┐            ┌─────────────────┐
│ trajectory      │  REQ      │  REP 5660       │
│ executor_client │ ─────────>│ trajectory      │
└─────────────────┘            │ executor_server │
                               └─────────────────┘

规划PC                          驱动PC (右臂)
┌─────────────────┐            ┌─────────────────┐
│ trajectory      │  REQ      │  REP 5661       │
│ executor_client │ ─────────>│ trajectory      │
└─────────────────┘            │ executor_server │
                               └─────────────────┘
```

### 规划PC 内部服务

```
外部客户端                     规划PC
┌─────────────┐               ┌─────────────────┐
│             │  REQ 5605     │  REP            │
│ UR Move     │ ─────────────>│ UR Move Server  │
│ Client      │               └─────────────────┘
└─────────────┘

外部客户端                     规划PC
┌─────────────┐               ┌─────────────────┐
│             │  REQ 5630     │  REP            │
│ Gripper     │ ─────────────>│ Left Gripper    │
│ Client      │               │ Server          │
└─────────────┘               └─────────────────┘

外部客户端                     规划PC
┌─────────────┐               ┌─────────────────┐
│             │  REQ 5640     │  REP            │
│ Gripper     │ ─────────────>│ Right Gripper   │
│ Client      │               │ Server          │
└─────────────┘               └─────────────────┘

外部客户端                     规划PC
┌─────────────┐               ┌─────────────────┐
│             │  REQ 5609     │  REP            │
│ TF Client   │ ─────────────>│ TF Server       │
└─────────────┘               └─────────────────┘
```

## 端口配置位置

### Launch 文件配置

主要配置在 `ur_move/launch/ur_move_planner_server.launch.py`:

```python
# 核心服务
ur_move_port: 5605
tf_server_port: 5609

# 夹爪服务
gripper_left_port: 5630
gripper_right_port: 5640

# 关节状态（从驱动PC接收）
left_arm_zmq_port: 5650
right_arm_zmq_port: 5651
```

### 驱动PC配置

左臂驱动PC (`single_arm/single_arm_config/scripts/`):
- `joint_states_zmq_publisher.py --zmq-port 5650`
- `trajectory_executor_server.py --zmq-port 5660 --arm-name left_arm`

右臂驱动PC (`single_arm/single_arm_config/scripts/`):
- `joint_states_zmq_publisher.py --zmq-port 5651`
- `trajectory_executor_server.py --zmq-port 5661 --arm-name right_arm`

## 端口使用统计

| 类别 | 端口数量 | 端口范围 |
|------|---------|---------|
| 核心服务 | 2 | 5605, 5609 |
| 夹爪服务 | 2 | 5630, 5640 |
| 关节状态 | 2 | 5650, 5651 |
| 轨迹执行 | 2 | 5660, 5661 |
| **总计** | **8** | **5605-5661** |

## 设计原则

1. **端口范围分离**: 不同功能使用不同的端口范围，便于管理和识别
2. **左右对称**: 左右臂使用相邻端口（如 5650/5651, 5660/5661）
3. **REQ/REP 模式**: 所有服务使用 REQ/REP 模式，保证请求-响应语义
4. **PUB/SUB 模式**: 关节状态使用 PUB/SUB 模式，支持一对多订阅
5. **可配置**: 所有端口都可通过 launch 参数或命令行参数配置

## 未来扩展建议

### 预留端口范围
- **5670-5679**: 预留用于未来扩展
- **5680-5689**: 预留用于传感器数据
- **5690-5699**: 预留用于状态监控

### 单臂模式支持
当前设计支持双臂，单臂模式可以：
- 只使用左臂端口（5650, 5660）
- 或只使用右臂端口（5651, 5661）

## 注意事项

1. **防火墙配置**: 确保驱动PC和规划PC之间的防火墙允许这些端口通信
2. **端口冲突**: 确保没有其他服务占用这些端口
3. **网络配置**: 确保驱动PC和规划PC在同一网络或可路由
4. **端口绑定**: PUB 和 REP socket 在服务端绑定，SUB 和 REQ socket 在客户端连接