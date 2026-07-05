你是我的 ROS2 Humble + MoveIt2 + UR ROS2 Driver 工程助手。请在当前公开仓库中执行“阶段 A 收尾补丁”，只修安全正确性，不做性能优化、不重写算法、不做评估体系。

仓库：
https://github.com/gjjgjhgmk/ros2_hybrid_bundle

背景：
当前仓库已经有：
- `CheckMotionBatch.srv`
- `PlanLocalSegment.srv`
- C++ `collision_checker.cpp`
- C++ `intent_rrt_connect.cpp`
- Python 主节点 `intent_hybrid_planner_node.py`
- C++ runtime bridge `intent_runtime_bridge.cpp`

但是阶段 A 还没有完全收尾。请只补以下 6 个点：

============================================================
目标 1：补 C++ RRT-Connect direct shortcut
============================================================

文件主要在：
- `intent_hybrid_runtime_cpp/src/intent_rrt_connect.cpp`
- 可能涉及 `intent_hybrid_runtime_cpp/include/.../intent_rrt_connect.hpp`
- 可能涉及 `intent_runtime_bridge.cpp` 的 PlanLocalSegment handler

要求：

1. 在 start / goal 都通过 state validity 检查之后，进入随机 RRT 前，先检查：

```text
isEdgeValid(start, goal)
如果 start→goal 这条 direct edge 有效，直接返回：
ok = true
stop_reason = "direct"
path = [start, goal]
path_points = 2
path_flat = row-major 的 2 × dof
via_times = [t_start, t_end]
不进入随机 RRT，不生成额外 via，不扰动安全的局部段。
日志打印：
PlanLocalSegment direct path valid; return 2-point path.

验收：

无障碍或局部段本来安全时，PlanLocalSegment 应返回 stop_reason=direct、path_points=2。
============================================================
目标 2：给 connect_tree() 加最大步数保护

当前 connect_tree() 里如果存在无保护 while(true)，请改成有界循环。

要求：

每次 connect 开始时根据当前 tree 端点和 target 的距离计算：
max_connect_steps = ceil(distance(q_from, q_target) / step_size) + 2
connect 循环最多执行 max_connect_steps 次。
如果超过最大步数还没有连接成功，返回失败或 progress 状态，但不能死循环。
防止 step_size <= 0、distance 非法、NaN 等异常输入导致死循环。
日志或 debug 信息中可以记录 max_connect_steps，但不要过度刷屏。

验收：

intent_rrt_connect.cpp 中不应再有无保护的 while(true)。
connect 阶段不会无限循环。
============================================================
目标 3：成功 path 返回前必须做最终验证 validatePath()

请新增或补齐一个路径最终验证函数，名称可为：

validatePath(...)

或者等价逻辑。

成功返回前必须验证：

path 非空，至少 2 个点。
path[0] 数值上接近 start。
path[-1] 数值上接近 goal。
path 中每个点维度等于 dof。
每个点都是 state valid。
每对相邻点的 edge 都是 valid。
如果最后一个点只是接近 goal 但不等于 goal：
尝试追加真正 goal；
验证倒数第二点到 goal 的 edge；
如果 valid，则追加 goal；
如果 invalid，则不能认为成功。

建议容差：

endpoint_tolerance = min(goal_tolerance, 1e-3 或合理小值)

如果需要更宽松，至少必须保证最后 path 真实包含 goal，而不是只接近 goal。

如果 validatePath 失败：

ok = false
stop_reason = "failed_connect" 或 "invalid_solution"
path_points = 0
path_flat = empty
error_message 写清楚失败原因

验收：

RRT 成功返回的 path 首点必须是 start，末点必须是 goal。
所有相邻 edge 必须通过 collision check。
不允许只因两树端点距离小于 goal_tolerance 就返回未连接到 goal 的路径。
============================================================
目标 4：失败时绝不返回假路径

请检查所有 PlanLocalSegment / IntentRRTConnect 失败路径。

失败情况包括但不限于：

invalid_request
collision_start
collision_goal
timeout
max_iter
failed_connect
invalid_solution

要求：

所有失败情况下必须：
ok = false
path_points = 0
path_flat.clear()
via_times.clear()
不允许返回 [start, goal] 假路径。
不允许返回 partial tree path 当作成功 path。
Python 收到 ok=false 时，不得把空 path 或 start-goal 当作 via。
stop_reason 必须明确，不要只写 "failed"。

验收：

timeout/max_iter 情况下 path_points=0。
path_flat 为空。
Python 不会继续把失败结果进入 FMP。
============================================================
目标 5：Python 侧确认 FMP 后 post-check 是唯一安全门

文件：

intent_hybrid_planner/intent_hybrid_planner/intent_hybrid_planner_node.py

要求：

FMP modulate() 完成后，任何 dispatch 前，必须调用：
/intent_runtime/check_motion_batch
check_edges = true
edge_resolution = postcheck_edge_resolution
新增或确认参数：
execute_only_if_postcheck_passed: true
postcheck_check_edges: true
postcheck_edge_resolution: 0.02
post-check 通过条件必须严格为：
response.ok == true
state_valid 长度 == trajectory point count
state_valid 全 true
edge_valid 长度 == trajectory point count - 1
edge_valid 全 true
first_invalid_state == -1
first_invalid_edge == -1
如果 N < 2，视为轨迹无效，不允许 dispatch。
如果 post-check 失败：
打印 error：
first_invalid_state
first_invalid_edge
invalid state count
invalid edge count
state_valid length
edge_valid length
trajectory point count
elapsed_ms
collision_queries
offline planning 返回失败
不调用 execute_trajectory_offline()
不调用 C++ dispatch
不发送 FollowJointTrajectory action
如果用户显式设置：
execute_only_if_postcheck_passed: false

可以允许实验模式继续，但必须打印明显 warning：

Unsafe experimental mode: dispatching trajectory even though post-check failed.

默认必须是 true。

benchmark/log 至少记录：
postcheck_passed
postcheck_state_invalid_count
postcheck_edge_invalid_count
postcheck_first_invalid_state
postcheck_first_invalid_edge
postcheck_elapsed_ms
postcheck_collision_queries

验收：

FMP 后轨迹只要 state 或 edge 有一个 invalid，就不会 dispatch。
CheckMotionBatch 响应尺寸异常也不会 dispatch。
============================================================
目标 6：Python 侧复核局部 path，包括 C++ path 和 fallback path

要求：

C++ PlanLocalSegment 返回 ok=true 后，Python 不要直接把 path 送入 densify / via merge。

必须先调用：

CheckMotionBatch(check_edges=true)

对这个局部 path 进行复核。

局部 path 复核通过后，才允许进入：
densify / via merge / FMP modulation
如果 C++ local path 复核失败：
打印 error，包含 first_invalid_state / first_invalid_edge
如果 allow_cpp_local_planner_fallback=true，尝试 Python RRT fallback
如果 fallback=false，本轮 planning 失败，不进入 FMP，不 dispatch
Python RRT fallback 生成的 path 也必须经过同样的 CheckMotionBatch(check_edges=true) 复核。
fallback path 复核失败时：
本轮 planning 失败
不进入 FMP
不 dispatch
新增或确认参数：
allow_cpp_local_planner_fallback: true

开发阶段默认 true，实机严格测试时用户会设为 false。

验收：

所有进入 FMP 的 via path 都经过 motion-level 复核。
C++ planner 和 Python fallback 都不能绕过局部 path 复核。
============================================================
不要做的事情

本轮不要做：

不要搭建最小评估体系
不要做参数 sweep
不要做 pRRTC/GPU
不要重写 FMP
不要删除 Python RRT fallback
不要删除旧服务
不要做 CollisionChecker 性能优化
不要大规模重构 dispatch
不要实现完整 Ruckig
不要改控制器配置

只做阶段 A 收尾的 correctness patch。

============================================================
构建要求

修改完成后执行：

cd /home/woody/simple_fmp_v1
colcon build --symlink-install
source install/setup.bash

如果接口包或 C++ 包需要单独构建，按顺序：

colcon build --packages-select intent_hybrid_interfaces --symlink-install
source install/setup.bash

colcon build --packages-select intent_hybrid_runtime_cpp intent_hybrid_planner --symlink-install
source install/setup.bash
============================================================
测试要求

请提供并尽量执行以下测试：

测试 1：direct path

构造一个 start→goal 无碰撞的 local segment，调用：

/intent_runtime/plan_local_segment

期望：

ok=true
stop_reason=direct
path_points=2
path_flat 非空
via_times 长度=2
测试 2：start collision

构造碰撞 start。

期望：

ok=false
stop_reason=collision_start
path_points=0
path_flat=[]
测试 3：goal collision

构造碰撞 goal。

期望：

ok=false
stop_reason=collision_goal
path_points=0
path_flat=[]
测试 4：timeout/max_iter failure

人为设置很小 timeout 或 max_iter，让 planner 失败。

期望：

ok=false
path_points=0
path_flat=[]
stop_reason=timeout 或 max_iter 或 failed_connect
测试 5：FMP 后 post-check failure

让调制后轨迹故意穿过障碍，或用 mock/构造轨迹触发 CheckMotionBatch invalid。

期望：

postcheck_passed=false
不调用 dispatch
offline planning 返回失败
日志包含 first_invalid_state 或 first_invalid_edge
测试 6：local path 复核

让 C++ planner 返回 path 后，确认 Python 日志中出现 local path check_motion_batch 复核结果。

如果 local path invalid：

不进入 FMP
或进入 fallback
fallback path 也必须复核
============================================================
Acceptance Criteria

本轮完成的验收标准：

direct start-goal edge valid 时返回 stop_reason=direct。
connect_tree() 不存在无保护 while(true)。
成功 path 首点为 start，末点为 goal。
成功 path 所有相邻 edges valid。
失败时 path_points=0，path_flat 为空。
失败时不返回 [start, goal] 假路径。
FMP 后 post-check 使用 CheckMotionBatch states+edges。
FMP 后 post-check 失败时绝不 dispatch。
CheckMotionBatch 响应尺寸异常时绝不 dispatch。
C++ local path 和 Python fallback path 进入 FMP 前都经过 CheckMotionBatch 复核。
本轮不破坏现有 FMP、Marker、Dispatch、旧接口和 Python fallback。

完成后请总结：

修改了哪些文件
每个目标如何实现
哪些测试已执行
哪些测试需要用户在 URSim / fake hardware 下执行
是否已经可以进入“最小评估体系搭建”
