你是我的 ROS2 Humble + MoveIt2 + UR ROS2 Driver 工程助手。请在我当前工作区 `/home/woody/simple_fmp_v1` 内改造现有项目，不要重写整个项目，不要删除现有功能。当前结构如下：

* Python 编排包：`intent_hybrid_planner`

  * 主节点：`intent_hybrid_planner/intent_hybrid_planner_node.py`
  * FMP：`intent_hybrid_planner/fmp_core.py`
  * Python RRT baseline：`intent_hybrid_planner/intent_biased_rrt.py`
* C++ runtime bridge 包：`intent_hybrid_runtime_cpp`

  * 当前节点：`intent_runtime_bridge.cpp`
  * 当前职责：批量碰撞服务、FK + Marker 发布服务、轨迹下发服务
* 接口包：`intent_hybrid_interfaces`

  * 当前 srv：`CheckStatesBatch.srv`、`DispatchJointTrajectory.srv`、`PublishPlanningMarkers.srv`

我的目标是把当前系统升级为：

Python 负责：

1. nominal trajectory / demo 生成或读取
2. FMP train / modulation
3. danger segment detection
4. 调用 C++ local planner 生成局部 via path
5. 调用 C++ post validation
6. 调用 C++ trajectory dispatch
7. benchmark logging

C++ 负责：

1. 通过 MoveIt2 PlanningSceneMonitor 直接做碰撞检测，不要在 RRT 内部通过 `/check_state_validity` 服务逐点查询
2. 提供 `isStateValid(q)` 和 `isEdgeValid(q1,q2)`
3. 提供 batch motion validation service
4. 提供 C++ Intent-Biased RRT-Connect 局部规划 service/action
5. 保留原有轨迹下发和 marker 发布能力

请按照以下步骤执行。

============================================================
一、总体原则
======

1. 不要删除现有 Python RRT：`intent_biased_rrt.py` 保留为 baseline / fallback，但实机主流程要能选择使用 C++ local planner。
2. 不要删除现有 C++ bridge：在 `intent_hybrid_runtime_cpp` 内扩展它，而不是另建完全无关的包。
3. 所有新功能必须通过 ROS2 参数开关控制，默认尽量保持原有行为不崩。
4. C++ planner 的目标不是 RRT*，而是先实现实时友好的 Intent-Biased RRT-Connect。
5. 实机执行前必须做整条轨迹 post validation：既检查 states，也检查相邻 edges。
6. 如果 FMP 调制后 post validation 失败，默认不要执行轨迹，返回明确错误日志。
7. 保持 ROS2 Humble 兼容。
8. 代码要可编译、可运行、日志清晰。
9. 添加必要的 CMakeLists.txt、package.xml、srv 依赖更新。
10. 每完成一个阶段，运行 `colcon build --symlink-install`，修复编译错误。

============================================================
二、接口层改造：新增 srv
==============

在 `intent_hybrid_interfaces/srv/` 下新增两个服务。

---

1. 新增 `CheckMotionBatch.srv`

---

内容如下：

```
string group_name
string[] joint_names
uint32 dof
float64[] states_flat
bool check_edges
float64 edge_resolution
---
bool ok
bool[] state_valid
bool[] edge_valid
int32 first_invalid_state
int32 first_invalid_edge
string error_message
float64 elapsed_ms
uint32 collision_queries
```

语义：

* `states_flat` 表示 N × dof 的轨迹点，row-major。
* `state_valid[i]` 表示第 i 个状态是否 collision-free。
* 如果 `check_edges=true`，则 `edge_valid[i]` 表示第 i 条边 `state_i -> state_{i+1}` 是否有效，长度应为 `N-1`。
* `first_invalid_state` 和 `first_invalid_edge` 没有无效项时为 `-1`。
* `collision_queries` 统计内部调用状态碰撞检测的次数。

---

2. 新增 `PlanLocalSegment.srv`

---

内容如下：

```
string group_name
string[] joint_names
uint32 dof

float64[] start
float64[] goal

float64[] intent_flat
uint32 intent_points
float64 t_start
float64 t_end

float64[] state_min
float64[] state_max

float64 timeout_sec
uint32 max_iter
float64 step_size
float64 goal_tolerance
float64 edge_resolution

float64 p_intent
float64 p_goal
float64 p_uniform
float64 sigma_intent

uint32 rng_seed
---
bool ok
float64[] path_flat
uint32 path_points
float64[] via_times
string stop_reason
string error_message
float64 elapsed_ms
uint32 iter_used
uint32 collision_queries
```

语义：

* `intent_flat` 表示 M × dof 的 intent path，row-major。
* 返回 `path_flat` 表示 K × dof 的局部路径，row-major。
* `via_times` 长度为 K，根据路径弧长比例在 `[t_start,t_end]` 上分配。
* `state_min/state_max` 可为空；为空时使用 MoveIt joint bounds 或宽松 bounds。
* `p_intent/p_goal/p_uniform` 自动归一化。
* `sigma_intent` 是 intent 高斯扰动标准差，先支持标量。
* `goal_tolerance` 是 q-space 中认为双树连接成功的距离阈值。

更新：

* `intent_hybrid_interfaces/CMakeLists.txt`
* `intent_hybrid_interfaces/package.xml`
  确保新 srv 能生成。

============================================================
三、C++ Runtime Core：新增 CollisionChecker
======================================

在 `intent_hybrid_runtime_cpp/include/intent_hybrid_runtime_cpp/` 新增：

* `collision_checker.hpp`
* `planner_types.hpp`
* `intent_rrt_connect.hpp`

在 `intent_hybrid_runtime_cpp/src/` 新增：

* `collision_checker.cpp`
* `intent_rrt_connect.cpp`

---

1. `CollisionChecker` 目标

---

实现一个类：

```
class CollisionChecker {
public:
  struct Options {
    std::string group_name;
    std::vector<std::string> joint_names;
    double edge_resolution{0.02};
  };

  CollisionChecker(
    const rclcpp::Node::SharedPtr& node,
    const std::string& robot_description_param = "robot_description");

  bool initialize();

  bool isReady() const;

  bool isStateValid(
    const std::vector<double>& q,
    const std::string& group_name,
    const std::vector<std::string>& joint_names,
    std::string* error = nullptr);

  bool isEdgeValid(
    const std::vector<double>& q1,
    const std::vector<double>& q2,
    const std::string& group_name,
    const std::vector<std::string>& joint_names,
    double edge_resolution,
    std::string* error = nullptr);

  std::vector<bool> checkStatesBatch(...);

  // check both states and edges, return first invalid indices and query count.
};
```

要求：

1. 内部使用 `planning_scene_monitor::PlanningSceneMonitor`。
2. 启动：

   * `startSceneMonitor()`
   * `startWorldGeometryMonitor()`
   * `startStateMonitor()`
3. 碰撞检测不要通过 `/check_state_validity` 服务。
4. 每次 batch / local planning 开始时，获取 `LockedPlanningSceneRO`，在一次函数调用内部复用 scene snapshot。
5. 复用 `moveit::core::RobotState`，不要每个 state 都重新构造昂贵对象。
6. 用 `setJointGroupPositions(jmg, q)` 设置规划组关节。
7. 调用 `state.update()` 后做 collision check。
8. 如果能用 `planning_scene->isStateColliding(state, group_name)`，优先使用这个接口。
9. 如果 group/joint_names 错误，要返回清楚错误。
10. state 维度必须等于 joint_names 数量。
11. edge check 用最大关节差决定插值点数：

```
N = ceil(max_i(abs(q2[i]-q1[i])) / edge_resolution)
```

并检查 `k=0...N` 所有插值点。

注意：

* 需要包含 MoveIt2 头文件和 CMake 依赖：

  * `moveit_ros_planning`
  * `moveit_core`
  * `planning_scene_monitor`
  * `robot_state`
  * `robot_model`
  * `collision_detection`
* 如果 Humble 下 include 名称有差异，请根据实际编译错误修正。

============================================================
四、C++ Local Planner：Intent-Biased RRT-Connect
=============================================

新增 `IntentRRTConnect` 类。

---

1. 输入参数结构

---

在 `planner_types.hpp` 中定义：

```
struct LocalPlanRequestData {
  std::string group_name;
  std::vector<std::string> joint_names;
  size_t dof;

  std::vector<double> start;
  std::vector<double> goal;

  std::vector<std::vector<double>> intent_path;  // M x dof

  std::vector<double> state_min;
  std::vector<double> state_max;

  double t_start;
  double t_end;

  double timeout_sec;
  uint32_t max_iter;
  double step_size;
  double goal_tolerance;
  double edge_resolution;

  double p_intent;
  double p_goal;
  double p_uniform;
  double sigma_intent;

  uint32_t rng_seed;
};

struct LocalPlanResultData {
  bool ok;
  std::vector<std::vector<double>> path;  // K x dof
  std::vector<double> via_times;
  std::string stop_reason;
  std::string error_message;
  double elapsed_ms;
  uint32_t iter_used;
  uint32_t collision_queries;
};
```

---

2. RRT-Connect 算法要求

---

实现双树：

```
Tree A starts from start
Tree B starts from goal

for iter:
    q_rand = sample_mixture()
    q_new = extend(TreeA, q_rand)
    if q_new added:
        q_connect = connect(TreeB, q_new)
        if connected:
            return combined path
    swap(TreeA, TreeB)
```

采样混合：

* `p_intent`: 从 intent_path 随机选一个点，加 `sigma_intent * normal(0,1)` 扰动，然后 clamp 到 bounds。
* `p_goal`: 采样 goal。
* `p_uniform`: 在 bounds 内均匀采样。
* 三个概率自动归一化。

nearest：

* 线性扫描即可，先保证正确。

steer：

```
delta = q_target - q_near
if norm(delta) <= step_size: q_new = q_target
else q_new = q_near + step_size * delta / norm(delta)
```

有效性：

* `q_new` 必须 `isStateValid`
* `q_near -> q_new` 必须 `isEdgeValid`

连接成功条件：

* 若两树新端点距离 <= `goal_tolerance`
* 或者一次 steer 到对方节点并 edge valid

路径合并：

* 返回从 start 到 goal 的连续 path。
* 注意 tree swap 后方向不要错。
* 去掉重复连接点。

时间分配：

* 根据 q-space 弧长分配 via_times 到 `[t_start,t_end]`。
* 如果路径总长度接近 0，则线性分配。

失败策略：

* 如果超时，返回 `ok=false`，`stop_reason="timeout"`。
* 如果 max_iter 用尽，返回 `ok=false`，`stop_reason="max_iter"`。
* 不要返回碰撞路径作为 ok=true。
* error message 要清楚。

============================================================
五、改造 `intent_runtime_bridge.cpp`
================================

保留原有功能，同时新增两个服务：

* `/intent_runtime/check_motion_batch`
* `/intent_runtime/plan_local_segment`

---

1. 参数新增

---

在 bridge 节点声明参数：

```
use_planning_scene_monitor: bool = true
robot_description_param: string = "robot_description"
default_edge_resolution: double = 0.02
default_planner_timeout_sec: double = 0.1
default_planner_max_iter: int = 500
default_planner_step_size: double = 0.15
default_goal_tolerance: double = 0.08
```

---

2. 初始化 CollisionChecker

---

构造节点时：

* 创建 `CollisionChecker`
* 调用 initialize
* 如果失败，保留旧 `/check_state_validity` service fallback，但打印 warning。
* 如果成功，新服务都优先使用 PlanningSceneMonitor。

---

3. 实现 `handle_check_motion_batch`

---

流程：

1. parse `states_flat` 为 rows。
2. 检查 dof / joint_names。
3. 调 CollisionChecker 做 state valid。
4. 如果 `check_edges=true`，做 edge valid。
5. 填充 response。
6. 统计 elapsed_ms 和 collision_queries。
7. 遇到错误时 `ok=false`，error_message 明确。

---

4. 实现 `handle_plan_local_segment`

---

流程：

1. parse start / goal / intent_flat。
2. 检查 dof / joint_names / 参数合法性。
3. 构造 `LocalPlanRequestData`。
4. 调 `IntentRRTConnect::plan(...)`。
5. 填充 response。
6. 如果 planner 失败，`ok=false`，不要生成假路径。
7. 日志输出：

   * iter_used
   * elapsed_ms
   * collision_queries
   * stop_reason
   * path_points

---

5. 保留旧 `CheckStatesBatch`

---

保留 `/intent_runtime/check_states_batch`，但内部优先改成使用 CollisionChecker，而不是继续逐个调用 `/check_state_validity`。
如果 CollisionChecker 不 ready，再 fallback 到旧 service 链路。

============================================================
六、Python 编排层改造
==============

修改 `intent_hybrid_planner_node.py`。

---

1. 新增参数

---

新增 ROS2 参数：

```
use_cpp_local_planner: bool = true
cpp_local_planner_service: string = "/intent_runtime/plan_local_segment"
cpp_motion_check_service: string = "/intent_runtime/check_motion_batch"

planner_type: string = "rrt_connect"
cpp_planner_timeout_sec: double = 0.10
cpp_planner_max_iter: int = 500
cpp_planner_step_size: double = 0.15
cpp_planner_goal_tolerance: double = 0.08
cpp_edge_resolution: double = 0.02

execute_only_if_postcheck_passed: bool = true
postcheck_check_edges: bool = true
```

---

2. 新增 service clients

---

导入新增 srv：

* `CheckMotionBatch`
* `PlanLocalSegment`

创建 clients：

* `/intent_runtime/check_motion_batch`
* `/intent_runtime/plan_local_segment`

等待 service ready 的逻辑要有 timeout 和清楚日志。

---

3. 把危险段局部规划改为可选 C++ planner

---

当前 Step 2/4 “危险段 RRT 生成 via” 里，如果 `use_cpp_local_planner=true`：

* 对每个 danger segment：

  * 取 `local_start`
  * 取 `local_goal`
  * 取 `intent_local`
  * 调 `/intent_runtime/plan_local_segment`
  * 如果 success，把返回 path 作为 via_points
  * 如果失败：

    * 记录 stop_reason
    * 如果允许 fallback，则调用现有 Python `IntentBiasedRRT`
    * 否则本次规划失败，不执行

如果 `use_cpp_local_planner=false`，保持原 Python RRT 逻辑。

---

4. post validation 改成 motion batch

---

FMP 调制生成 `modulated_traj` 后，调用 `/intent_runtime/check_motion_batch`：

* states_flat = `modulated_traj.T.reshape(-1)`
* check_edges = `postcheck_check_edges`
* edge_resolution = `cpp_edge_resolution`

如果返回：

* `ok=false`：报错，不执行。
* `first_invalid_state>=0` 或 `first_invalid_edge>=0`：报错，不执行，除非参数明确允许实验模式继续。
* 全部有效：进入 dispatch。

---

5. danger scan 也升级为 edge-aware

---

原来如果只检查 nominal states，请补充 edge scan：

* 用 `/intent_runtime/check_motion_batch` 对 nominal 全轨迹检查 states + edges。
* danger_indices 同时来自：

  * invalid state i
  * invalid edge i：将 i 和 i+1 都加入 danger set
* 再合并成 danger segments。

---

6. 日志与 benchmark

---

在现有 benchmark JSON/CSV 中追加字段：

```
cpp_local_planner_used
cpp_plan_success_count
cpp_plan_failure_count
cpp_plan_time_ms_mean/p95/max
cpp_plan_collision_queries_mean/p95/max
postcheck_first_invalid_state
postcheck_first_invalid_edge
postcheck_elapsed_ms
postcheck_collision_queries
```

保留原有 RRT baseline 指标。

============================================================
七、构建系统更新
========

更新 `intent_hybrid_runtime_cpp/CMakeLists.txt`：

添加源文件：

* `src/collision_checker.cpp`
* `src/intent_rrt_connect.cpp`

添加 include dirs。

添加依赖：

* `rclcpp`
* `rclcpp_action`
* `moveit_core`
* `moveit_ros_planning`
* `moveit_ros_planning_interface`
* `planning_scene_monitor`
* `robot_state`
* `robot_model`
* `collision_detection`
* `intent_hybrid_interfaces`
* `trajectory_msgs`
* `control_msgs`
* `sensor_msgs`
* `geometry_msgs`
* `visualization_msgs`

如果某些依赖名在 Humble 中不对，请按实际包名修正。

更新 `package.xml` 对应依赖。

更新 Python 包依赖导入，确保新 srv 可 import。

============================================================
八、测试要求
======

完成代码后，请执行：

```
cd /home/woody/simple_fmp_v1
colcon build --symlink-install
source install/setup.bash
```

如果编译失败，请继续修复。

然后提供以下测试命令或说明：

1. 启动 UR MoveIt / fake hardware 后，启动 runtime bridge。
2. 调用 `/intent_runtime/check_motion_batch` 测试一条简单轨迹。
3. 调用 `/intent_runtime/plan_local_segment` 测试 start 到 goal 的局部规划。
4. 启动 Python planner，设置：

   * `use_cpp_local_planner:=true`
   * `postcheck_check_edges:=true`
   * `execute_only_if_postcheck_passed:=true`

如果没有真实 UR7e，就以 fake hardware / URSim 为目标。

============================================================
九、代码质量要求
========

1. C++ 不要在 RRT 内部用 ROS service 调 `/check_state_validity`。
2. C++ 碰撞检测必须支持 self collision + environment collision，即使用 MoveIt PlanningScene。
3. C++ batch check 不要每个 state 都重新初始化 PlanningSceneMonitor。
4. RRT 中必须检查 edge，不允许只检查 node。
5. 所有路径执行前必须 post-check states + edges。
6. Python fallback 逻辑要清晰。
7. 所有错误都要有明确日志，不要 silent failure。
8. 参数必须有合理默认值。
9. 不要破坏现有 `DispatchJointTrajectory` 和 `PublishPlanningMarkers`。
10. 不要删除旧 `CheckStatesBatch`，但可以内部优化实现。

============================================================
十、最终交付内容
========

请完成代码修改后，给我总结：

1. 新增/修改了哪些文件。
2. 新增了哪些 ROS2 参数。
3. 新增了哪些 service。
4. C++ RRT-Connect 的碰撞检测链路如何工作。
5. Python 主节点如何调用 C++ planner。
6. 如何运行 colcon build。
7. 如何测试 check_motion_batch。
8. 如何测试 plan_local_segment。
9. 当前还没实现或需要后续优化的内容。
