# simple_fmp_v1

ROS2 Humble + UR7e 的混合规划工程。  
目标：在 ROS2 中复刻 MATLAB 主逻辑（危险检测 + 分段 RRT + 一次 FMP 全局调制 + 一次性轨迹下发），并支持仿真/实机迁移。

## 1. 当前架构

### 1.1 Python 算法编排层
- 主节点：[intent_hybrid_planner_node.py](/home/woody/simple_fmp_v1/intent_hybrid_planner/intent_hybrid_planner/intent_hybrid_planner_node.py)
- 离线主流程：
  - Step 1/4：全局碰撞扫描（nominal 全轨迹）
  - Step 2/4：危险段 RRT 生成 via
  - Step 3/4：FMP 一次性调制全轨迹
  - Step 4/4：下发控制器执行
- 核心模块：
  - FMP：[fmp_core.py](/home/woody/simple_fmp_v1/intent_hybrid_planner/intent_hybrid_planner/fmp_core.py)
  - RRT：[intent_biased_rrt.py](/home/woody/simple_fmp_v1/intent_hybrid_planner/intent_hybrid_planner/intent_biased_rrt.py)

### 1.2 C++ Runtime Bridge 层
- 包：`intent_hybrid_runtime_cpp`
- 节点：`intent_runtime_bridge`
- 职责：
  - 批量碰撞服务（MoveIt 服务链路）
  - FK + Marker 发布服务
  - 轨迹下发服务（FollowJointTrajectory）

### 1.3 接口层
- 包：`intent_hybrid_interfaces`
- 内容：Python 与 C++ 之间的 `srv` 接口定义

### 1.4 一键启动编排
- 脚本：[one_click.sh](/home/woody/simple_fmp_v1/one_click.sh)
- 职责：清理进程、启动仿真、控制器激活、预对齐、可选障碍注入、启动 runtime bridge 与辅助节点

## 2. 最新推荐运行流程（2026-05-25 实测）

### 2.1 构建
```bash
cd ~/simple_fmp_v1
source /opt/ros/humble/setup.bash
source ~/ws_ur_sim/install/setup.bash
colcon build --packages-select intent_hybrid_planner intent_hybrid_runtime_cpp --symlink-install
```

### 2.2 启动仿真（含障碍）
```bash
cd ~/simple_fmp_v1
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export RUNTIME_BACKEND=cpp_bridge
export ENABLE_OBSTACLES=1
./one_click.sh start --prealign
```

### 2.3 启动规划节点（CPP Bridge 碰撞）
```bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source ~/ws_ur_sim/install/setup.bash
source ~/simple_fmp_v1/install/setup.bash

ros2 run intent_hybrid_planner intent_hybrid_planner_node --ros-args \
  -p runtime_backend:=cpp_bridge \
  -p cpp_bridge_collision_required:=true \
  -p use_sim_time:=true \
  -p execution_mode:=offline \
  -p hybrid_mode:=matlab_compat \
  -p trajectory_action_name:=/joint_trajectory_controller/follow_joint_trajectory
```

### 2.4 结束
```bash
cd ~/simple_fmp_v1
./one_click.sh stop
```

## 3. 本次实测执行产物（最新一轮）

本轮 one-click 日志目录：
- [/tmp/one_click_20260525_064554](/tmp/one_click_20260525_064554)

关键日志文件：
- 仿真总日志：[ur_sim_moveit.log](/tmp/one_click_20260525_064554/ur_sim_moveit.log)
- runtime bridge：[intent_runtime_bridge.log](/tmp/one_click_20260525_064554/intent_runtime_bridge.log)
- 障碍注入：[spawn_obstacles.log](/tmp/one_click_20260525_064554/spawn_obstacles.log)
- 预对齐动作：[prealign_action.log](/tmp/one_click_20260525_064554/prealign_action.log)
- 末端轨迹 marker：[ee_trace_marker.log](/tmp/one_click_20260525_064554/ee_trace_marker.log)

规划结果文件：
- JSON：[hybrid_2obs_20260525_064635.json](/home/woody/simple_fmp_v1/result/hybrid_2obs_20260525_064635.json)
- CSV：[hybrid_2obs_20260525_064635.csv](/home/woody/simple_fmp_v1/result/hybrid_2obs_20260525_064635.csv)
- 轨迹图：[offline_traj_compare_20260525_064635.png](/home/woody/simple_fmp_v1/png/offline_traj_compare_20260525_064635.png)

本轮日志判据：
- 障碍注入成功：`spawn_obstacles.log` 含 `Gazebo obstacle spawn succeeded` 与 `Published planning_scene obstacles`
- 离线流程执行：Step1 检测到碰撞点、进入 Step2、Step3 完成、Step4 `sent_success`

## 4. 参数与命令含义

| 参数/命令 | 含义 |
|---|---|
| `ROS_LOCALHOST_ONLY=1` | 固定本机回环通信，避免多网卡互相不可见 |
| `ROS_DOMAIN_ID=0` | 统一 DDS 域，所有终端必须一致 |
| `RUNTIME_BACKEND=cpp_bridge` | one_click 启动 C++ runtime bridge |
| `ENABLE_OBSTACLES=1` | 开启 Gazebo + PlanningScene 双通道障碍注入 |
| `runtime_backend:=cpp_bridge` | 规划节点使用 cpp bridge 客户端 |
| `cpp_bridge_collision_required:=true` | 碰撞检查强制走 cpp bridge，失败即报错终止 |
| `execution_mode:=offline` | 走 one-shot 全局规划与执行 |
| `hybrid_mode:=matlab_compat` | MATLAB 兼容分段/预算语义 |
| `trajectory_action_name:=...joint_trajectory_controller...` | 指定有效控制器 action server |

## 5. 从开始到现在的简要总结

### 5.1 主要完成的工作
- 完成 MATLAB 语义向 ROS2 离线四步流水线迁移（保留 online/offline 双轨）。
- 将运行时关键能力下沉到 C++ bridge（碰撞/FK/下发），Python 保留 RRT/FMP 编排。
- one_click 一键化完善：控制器就绪检查、预对齐、防隔离错配、可选障碍注入、日志归档。
- 增加结果导出与可视化：`result/*.json|csv` 与 `png/offline_traj_compare_*.png`。

### 5.2 关键改进
- 解决 action server 名称不匹配导致的 `server not ready`。
- 增加起点预对齐，减少首段断层导致的 path tolerance abort。
- 障碍注入升级为统一配置源（JSON）驱动双通道一致化。
- 新增 `cpp_bridge_collision_required`，实现“碰撞必须走 CPP Bridge，失败即停并给出原因”。

### 5.3 遇到的问题与结论
- `moveit_py` 在 Humble 常不可用：已切换为 C++ MoveIt 服务链路，不依赖 moveit_py。
- 终端环境不一致（`ROS_LOCALHOST_ONLY`/`ROS_DOMAIN_ID`）会导致服务互相不可见。
- 若 `ENABLE_OBSTACLES=0`，Gazebo 不会出现障碍物，算法也无法形成真实避障闭环。
- `analytic_obstacles_json` 传参容易因引号转义失败，优先建议用 one_click + JSON 配置。
