# 双机械臂同时执行探索方案

## 结论

当前系统已经具备一部分双臂同时执行能力，但能力边界取决于执行模式。

1. `ur_move` 本地执行模式，即 `use_remote_execution: false` 或直接请求规划服务器规划并执行，支持在同一个 ZMQ 请求里包含 `left_arm` 和 `right_arm`，服务端会按 group 拆分轨迹，并用两个线程同时向左右臂 controller 发送执行 goal。
2. `ur_move` 规划服务器不支持多个独立 ZMQ 请求同时处理。它使用单个 `ZMQ_REP` socket 和单个 server loop，一个请求会阻塞到规划/执行结束后才回复并处理下一个请求。
3. `ur_bt` 现有 `move_to_waypoints()` API 已经可以表达“双臂同一动作阶段同时执行”：把左右臂 waypoint 放入同一个 `ArmMoveToWaypoints` 行为即可，不需要先新增行为树 API。
4. 真实硬件远程执行模式，即 `use_remote_execution: true`，当前不能确认是真正同时执行。规划结果里可以包含双臂轨迹，但 `TrajectoryExecutorClient.execute_trajectories()` 当前按字典顺序逐个调用 `execute_trajectory()`，会等待左臂返回后再发右臂，或反过来。因此远程执行要实现同时执行，优先应改 ZMQ 执行客户端的发送策略，而不是先改行为树 API。
5. 现有实现不是严格同步启动。即使本地模式使用两个线程，也只是“近同时发送两个 action goal”，没有统一时间戳、同步屏障或控制器级同步启动机制。它适合验证双臂是否能并行运动，不等价于工业级时间同步。

推荐第一阶段不增加行为树 API，主要用 demo 验证现有同一请求双臂执行路径；如果目标是远程真实硬件双臂同时执行，则做一个很小的底层客户端修改：让 `TrajectoryExecutorClient.execute_trajectories()` 并发向左右臂执行服务器发送请求。行为树层继续使用现有 `move_to_waypoints()`。

## 依据

### `ur_move` 的功能边界

`ur_move` 是双臂系统中的运动服务层，向上通过 ZMQ 接收请求，向下对接 MoveIt、控制器、夹爪和 TF 等能力。它的核心输入和输出可以简单理解为：

```text
输入：waypoints / execute request / gripper request / TF query
输出：规划轨迹 / 执行结果 / 夹爪结果 / TF 查询结果
```

其中最核心的是 waypoint 规划与执行请求。一次 waypoint 请求通常包含：

```text
waypoints:
  - group: left_arm 或 right_arm
  - type: joint 或 cart
  - planner: ptp / lin / ompl
  - joint_names / joint_values，或 frame_id / ik_frame / position / orientation
  - max_velocity_scaling_factor
  - max_acceleration_scaling_factor
execute: true 或 false
```

`execute=true` 时，`ur_move` 会规划后立即执行；`execute=false` 时，`ur_move` 返回 `execution_id` 和轨迹数据，后续可以再用 `execution_id` 触发执行。

### 一次 waypoint 请求包含多个 waypoint 的处理规则

`ur_move` 收到一次 waypoint 请求后，不是按请求整体直接生成一条轨迹，而是先按 `group` 分组，再分别规划：

```text
一次 waypoint 请求
  -> 解析所有 waypoints
  -> 按 group 分组，例如 left_arm / right_arm
  -> 每个 group 内按请求顺序逐个 waypoint 规划
  -> 同一个 group 的多段轨迹拼接成一条连续轨迹
  -> 不同 group 得到不同轨迹
  -> execute=true 时，不同 group 的轨迹并行执行
```

如果一次请求包含两个 group：

```text
waypoints:
  - 左臂-home，group=left_arm
  - 右臂-home，group=right_arm
```

处理结果是：

```text
left_arm  -> 规划 left_arm trajectory
right_arm -> 规划 right_arm trajectory

execute=true:
  left_arm trajectory  -> 左臂执行线程
  right_arm trajectory -> 右臂执行线程
```

如果一次请求包含同一个 group 的两个 waypoint：

```text
waypoints:
  - 左臂-A，group=left_arm
  - 左臂-B，group=left_arm
```

处理结果是：

```text
left_arm:
  左臂-A -> 规划第一段轨迹
  左臂-B -> 规划第二段轨迹
  第一段 + 第二段 -> 拼接成一条连续 left_arm trajectory

execute=true:
  向 left_arm controller 发送一个完整轨迹 goal
  执行顺序是 A -> B
```

因此，同一个 group 内的多个 waypoint 是顺序执行；不同 group 的轨迹才会在执行阶段并行。

混合情况可以理解为：

```text
请求：
  left_arm:  A -> B
  right_arm: C -> D

执行：
  left_arm  顺序执行 A -> B
  right_arm 顺序执行 C -> D
  两个 group 之间并行执行
```

关键结论：

```text
同一 group 的多个 waypoint：顺序规划、拼接、顺序执行
不同 group 的 waypoint：分别规划、分别成轨迹、执行阶段并行
```

### ZMQ 规划/执行服务端

`ur_move/src/server_cpp/trajectory_planner_server.cpp`：

1. 服务端使用单个 `ZMQ_REP` socket，绑定一个端口，并启动单个 `serverLoop` 线程。见 `start()` 中创建 `ZMQ_REP`、`bind()` 和 `server_thread_ = std::thread(...)`，以及 `serverLoop()` 中循环 `recv()` 后直接调用 `handleRequest()`。
2. 这意味着多个独立客户端请求不会被服务端同时处理。REQ/REP 模式也要求一次请求对应一次响应，当前 server loop 在 `handleRequest()` 返回前不会接收下一个请求。
3. 请求解析后，服务端会从所有 waypoint 中收集 `group`，并调用 `planner.planTrajectoriesByGroup(...)`。这说明单个请求可以包含多个 group，例如 `left_arm` 和 `right_arm`。
4. `execute=true` 时，服务端对 `trajectories_by_group` 中每个 group 创建一个 `std::thread`，线程内各自创建独立 ROS 2 node 和 `TrajectoryExecutor`，然后调用 `executeTrajectory(trajectory, group_name, true)`。随后主线程 `join()` 等待所有执行线程完成。这是本地模式双臂并行执行的核心依据。
5. `execute=false` 后再用 `execution_id` 执行时，`executePendingTrajectory()` 也对每个 group 创建执行线程并 `join()`，所以延迟执行路径在本地服务端同样支持同一 `execution_id` 内多 group 并行执行。

`ur_move/src/server_cpp/moveit_planner.cpp`：

1. `initializeMoveGroups()` 固定初始化 `left_arm` 和 `right_arm` 两个 MoveGroupInterface。
2. `planTrajectoriesByGroup()` 会先按 `waypoint.getGroup()` 分组，再逐个 group、逐个 waypoint 规划，最后返回 `map<group, trajectory>`。
3. 这里的规划是按 group 顺序进行，不是并行规划；当前“同时”主要发生在执行阶段。
4. 当前不是联合双臂规划。两个 group 的轨迹分别规划，缺少同一时间轴上的双臂碰撞/同步约束验证。

`ur_move/src/server_cpp/trajectory_executor.cpp`：

1. `getActionName()` 将 `left_arm` 映射到 `left_arm_controller/follow_joint_trajectory`，将 `right_arm` 映射到 `right_arm_controller/follow_joint_trajectory`。
2. `executeTrajectory()` 内部向对应 action server 发送 FollowJointTrajectory goal，并等待结果。
3. 在规划服务器为每个 group 启动独立线程时，左右臂会分别向各自 controller 发送 goal。

`ur_move/launch/ur_move_server.launch.py`：

1. 启动 `trajectory_planner_server`，并加载 `dual_arm_moveit_config`。
2. `initial_joint_controller` 默认值是 `both`，说明启动意图是左右臂控制器都可用。
3. 同时启动夹爪和 TF ZMQ 服务。

### 远程执行链路

`ur_bt` 中配置 `executor` 是为了支持远程执行模式，并不表示所有执行都绕过 `ur_move`。

```text
use_remote_execution=false:
  ur_bt -> ur_move:5605
  ur_move 负责规划并直接执行

use_remote_execution=true:
  ur_bt -> ur_move:5605 只请求规划
  ur_bt -> left/right executor:5660/5661 发送轨迹执行请求
```

因此，`executor.left_arm_host` 和 `executor.right_arm_host` 表示左右臂轨迹执行服务器所在机器；只有远程执行模式才需要它们。

`ur_bt/src/ur_bt/clients/arm/ur_move_client.py`：

1. `plan_and_execute_remote()` 先 `plan_trajectory(..., execute=False)`，再调用 `execute_remote(plan_result)`。
2. `execute_remote()` 会从规划结果的 `trajectories` 中筛选 `left_arm` 和 `right_arm`，根据配置生成 `tcp://host:5660` 和 `tcp://host:5661`。
3. 它之后创建 `TrajectoryExecutorClient` 并调用 `execute_trajectories(filtered_trajectories)`。

`ur_bt/src/ur_bt/clients/arm/trajectory_executor_client.py`：

1. 左臂和右臂各有独立 ZMQ REQ socket，默认地址分别是 `tcp://localhost:5660` 和 `tcp://localhost:5661`。
2. 但 `execute_trajectories()` 当前是普通 `for arm_name, trajectory_json in trajectories.items()`，每次调用 `execute_trajectory()` 后等待响应，再处理下一个 arm。
3. 因为 `execute_trajectory()` 内部会 `send_string()` 后立即 `recv_string()` 等待该手臂执行完成，当前远程执行路径是顺序发送，不是同时发送。

`single_arm/single_arm_config/scripts/trajectory_executor_server.py`：

1. 每个执行服务器只服务一个 arm，通过 `--zmq-port` 和 `--arm-name` 启动，支持 `left_arm` 或 `right_arm`。
2. 左右臂可分别运行在 5660/5661 两个端口上。
3. 单个执行服务器内部仍是 REP 请求处理，收到一个轨迹后执行并等待结果再回复。由于左右臂是两个端口/两个进程，只要客户端同时向两个端口发送请求，它们可以并行执行。
4. 当前阻止远程双臂同时执行的主要因素是客户端顺序发送，而不是执行服务器端口模型。

### 夹爪服务端和行为树

`ur_move/src/server_py/gripper_zmq_server.py`：

1. 左右夹爪分别绑定 `left_port` 和 `right_port`，默认 5630/5640。
2. `start()` 中左右 socket 各自启动 `_server_loop` 线程。
3. 服务端结构支持左右夹爪并行接收请求。

`ur_bt/src/ur_bt/behaviors/gripper_behavior.py`：

1. `gripper="both"` 时，工厂创建左右两个 `GripperSetPosition` 子行为，并返回 `py_trees.composites.Parallel(..., policy=SuccessOnAll())`。
2. 夹爪层已经有显式的“both” API。

### `ur_bt` 行为树执行模型

`ur_bt/src/ur_bt/behavior_tree.py`：

1. `BehaviorTreeManager._create_tree()` 始终把传入的 behaviors 包装成 `py_trees.composites.Sequence(memory=True)`。
2. `execute(behaviors)` 的默认语义是按列表顺序执行；只有某个元素本身是 `Parallel` 复合节点时，才会并行 tick 它的子节点。
3. 行为树执行循环在独立线程里按 `tick_rate` tick root。

`ur_bt/src/ur_bt/behaviors/arm_move_behavior.py`：

1. `ArmMoveToWaypoints` 是异步行为，`initialise()` 启动后台线程，`update()` 非阻塞返回 `RUNNING`，完成后返回 `SUCCESS/FAILURE`。
2. `_execute_task()` 从黑板读取 waypoint，并把传入的 `waypoint_configs` 汇总成一个 `waypoints_dict`，然后调用 `plan_and_execute_remote()` 或 `plan_and_execute()`。
3. 因此，如果一个 `ArmMoveToWaypoints` 行为同时包含左臂和右臂 waypoint，它会发出一个包含两个 group 的规划请求。这是当前最安全的双臂同时执行表达方式。
4. 不建议在 demo 里直接创建两个独立 `ArmMoveToWaypoints`，再用 `py_trees.Parallel` 并行 tick。两个子行为会共享同一个 `UrMoveClient` 和同一个 ZMQ REQ socket，REQ socket 并不适合多线程同时 send/recv；同时规划服务器也是单 REP loop，两个独立请求不会真正并行执行。

`ur_bt/test/test_arm_behavior.py`：

1. 已有 `test_dual_arm_movement()`，它创建一个 `move_to_waypoints()`，其中 waypoint_configs 同时包含 `("左臂-home", ...)` 和 `("右臂-home", ...)`。
2. 这个测试验证的正是“同一行为、同一请求、双 group”的路径。

`ur_bt/example/welding/welding_demo.py`：

1. home 和 return home 阶段已经使用一个 `move_to_waypoints()` 同时包含 `("左臂-home", ...)` 和 `("右臂-home", ...)`。
2. 焊接目标跟踪阶段则按 target 顺序逐点生成行为，因此左右臂目标不会在这一阶段自动并行。

## 能力判断

### 已支持

1. 单个规划请求中同时包含 `left_arm` 和 `right_arm` waypoint。
2. 本地规划服务器执行模式下，同一请求内左右臂轨迹并行发送到各自 controller。
3. 延迟执行 `execution_id` 路径下，同一个 `execution_id` 中的多 group 轨迹并行执行。
4. 左右夹爪并行控制。
5. 行为树层可以通过“一个 arm move 行为包含两个 arm waypoint”的方式表达双臂同阶段动作。

### 未完全支持或有风险

1. 多个独立 ZMQ 请求不会并发处理。
2. 远程真实硬件执行路径当前按手臂顺序发送执行请求，不是真正同时发送。
3. 当前没有严格同步启动机制，只能做到近同时。
4. 当前不是联合双臂规划，缺少双臂同时运动下的整体碰撞检查和时间同步约束。
5. 用行为树 `Parallel` 包两个独立 arm move 行为存在共享 ZMQ REQ socket 的并发风险，不建议作为 demo 主路径。

## 两种可能方案

### 方案 1：一次请求，`ur_move` 规划并执行

```text
ur_bt -> ur_move:5605
一次 waypoint 请求包含 left_arm + right_arm
execute=true
ur_move 按 group 拆分轨迹并并行执行
```

适合仿真或 `ur_move` 能直接访问左右臂控制器的本地执行模式。优点是改动小；限制是实机分布式部署时，执行通常不在 `ur_move` 所在机器上完成。

### 方案 2：一次规划，行为树并行发送左右轨迹

```text
Sequence
  -> PlanBothArms: ur_bt -> ur_move:5605, execute=false
  -> Parallel
       -> ExecuteLeftTrajectory:  left executor:5660
       -> ExecuteRightTrajectory: right executor:5661
```

适合真实硬件远程执行。关键是规划仍然应是一次请求、两个 group；并行发生在“发送左右轨迹执行”阶段，而不是并行创建两个现有 `ArmMoveToWaypoints` 去分别请求规划。

## 推荐方案

### 第一阶段：不新增行为树 API，先做 demo 验证

目标是验证当前系统在本地执行模式下双臂是否能同时动。

实现方式：新增一个 demo 文件，例如 `ur_bt/example/both_execution_demo.py`，但当前文档阶段不改代码。

demo 核心结构：

```python
behaviors = [
    manager.arm_move_behavior.move_to_waypoints(
        [("左臂-home", 0.1, 0.1), ("右臂-home", 0.1, 0.1)],
        name="both_home",
        use_remote_execution=False,
    ),
    manager.utility_behavior.sleep(1.0, name="settle"),
    manager.arm_move_behavior.move_to_waypoints(
        [("左臂-测试1", 0.1, 0.1), ("右臂-测试1", 0.1, 0.1)],
        name="both_test_1",
        use_remote_execution=False,
    ),
    manager.utility_behavior.sleep(1.0, name="settle"),
    manager.arm_move_behavior.move_to_waypoints(
        [("左臂-home", 0.1, 0.1), ("右臂-home", 0.1, 0.1)],
        name="both_return_home",
        use_remote_execution=False,
    ),
]
success = manager.execute(behaviors, wait=True)
```

验证点：

1. 服务端日志应显示同一个请求中规划出 `left_arm` 和 `right_arm` 两组轨迹。
2. 执行阶段应看到左右臂 execution thread 近同时开始。
3. RViz 或真实机器人应表现为左右臂同一阶段同时移动。
4. demo 不使用 `py_trees.Parallel` 包两个 arm move 行为，避免共享 `UrMoveClient` 的 ZMQ socket 并发问题。

配置建议：

1. 如果用 Docker/仿真/同机规划服务验证，使用 `use_remote_execution: false`。
2. waypoint 文件必须包含一组左臂和一组右臂可达、安全、不互相干涉的测试点。
3. 优先使用关节空间 `ptp` home/测试点，降低笛卡尔 IK 和路径碰撞导致的干扰。
4. 速度和加速度先用低比例，例如 `0.05` 到 `0.1`。

### 第二阶段：支持远程真实硬件同时执行

如果要在 `use_remote_execution: true` 下验证真实左右臂同时执行，建议最小修改底层客户端，而不是先新增行为树 API。

修改点：

1. 修改 `ur_bt/src/ur_bt/clients/arm/trajectory_executor_client.py` 的 `execute_trajectories()`，对每个 arm 启动一个线程，每个线程调用 `execute_trajectory()`，最后 join 并汇总结果。
2. 同步修改 `ur_move/client/trajectory_executor_client.py`，避免两个客户端实现不一致。
3. 保持 `ArmMoveBehavior.move_to_waypoints()` API 不变。demo 仍然使用一个行为包含左右臂 waypoint。

预期效果：

1. 规划仍由规划服务器一次完成，返回 `left_arm` 和 `right_arm` 轨迹。
2. 客户端同时向 5660 和 5661 发送执行请求。
3. 两个单臂执行服务器分别阻塞等待各自 action result，但因为它们是两个端口/进程，会并行执行。

注意点：

1. 这种修改只能做到“同时发请求”，不是严格同步控制器启动。
2. 如果两个执行服务器运行在不同机器上，网络延迟会带来毫秒到几十毫秒级启动偏差。
3. 若需要更严格同步，后续应设计带统一 `start_time` 的轨迹 goal，或引入上层同步屏障和控制器时间基准。

### 是否新增行为树 API

当前不建议第一步新增行为树 API。

原因：

1. 现有 `move_to_waypoints()` 已经支持一个行为包含多个 arm 的 waypoint。
2. 双臂同时执行的关键不是行为树是否能并行 tick，而是底层 ZMQ/执行客户端是否把同一阶段的左右臂作为同一批轨迹处理。
3. 新增一个 `move_both_to_waypoints()` 只会是 `move_to_waypoints()` 的薄包装，不能解决远程执行顺序发送问题。
4. 如果直接用行为树 `Parallel` 包两个独立 arm move 行为，反而会引入共享 ZMQ socket 的线程安全风险。

可以后续添加一个很薄的便捷 API，但它不是第一优先级：

```python
manager.arm_move_behavior.move_both_to_waypoints(
    left=[("左臂-home", 0.1, 0.1)],
    right=[("右臂-home", 0.1, 0.1)],
    name="both_home",
)
```

这个 API 内部应只做参数拼接，然后调用现有 `move_to_waypoints()`，避免产生两个独立 arm move 行为。

## Demo 设计

### Demo 1：本地执行模式双臂同时运动

文件建议：`ur_bt/example/both_execution_demo.py`

用途：验证当前不改代码时，规划服务器本地执行模式是否能让两个机械臂同时动。

流程：

1. 加载 `ur_bt/example/welding/config.yaml` 或新增 demo 专用 config，并设置 `use_remote_execution: false`。
2. 加载包含左右臂 home 和测试点的 waypoints。
3. 执行 `both_home`：一个 `move_to_waypoints()` 同时包含 `左臂-home` 和 `右臂-home`。
4. 暂停 1 秒。
5. 执行 `both_test`：一个 `move_to_waypoints()` 同时包含左臂测试点和右臂测试点。
6. 暂停 1 秒。
7. 执行 `both_return_home`：同一行为返回左右臂 home。

成功标准：

1. 行为树最终返回 SUCCESS。
2. 服务端返回的 trajectories 包含两个 group。
3. 观察到两臂在同一动作阶段同时开始并同时处于运动中。

失败排查：

1. 如果只有一臂运动，检查 waypoint 的 `group` 是否正确是 `left_arm`/`right_arm`。
2. 如果规划失败，先降低速度，改用 joint waypoint，确认单臂分别可达。
3. 如果执行失败，检查左右臂 controller/action server 是否都启动。

### Demo 2：远程执行模式双臂同时运动

前置条件：先完成第二阶段客户端并发发送修改。

配置：

```yaml
zmq:
  arm:
    ur_move:
      use_remote_execution: true
      executor:
        left_arm_host: <left-driver-ip>
        right_arm_host: <right-driver-ip>
```

流程与 Demo 1 相同，仍然使用一个 `move_to_waypoints()` 包含左右臂 waypoint。

额外验证：

1. 左臂执行服务器 5660 和右臂执行服务器 5661 应几乎同时收到请求。
2. 两边日志中的“收到执行请求”和“Goal已接受”时间差应明显小于单臂轨迹执行时长。
3. 如果仍然串行，优先检查 `TrajectoryExecutorClient.execute_trajectories()` 是否仍为顺序 for loop。

## 不推荐方案

### 不推荐：在 demo 中用 `py_trees.Parallel` 包两个独立 arm move 行为

示例：

```python
py_trees.composites.Parallel(
    name="bad_parallel_arms",
    children=[
        manager.arm_move_behavior.move_to_waypoints([("左臂-home", 0.1, 0.1)]),
        manager.arm_move_behavior.move_to_waypoints([("右臂-home", 0.1, 0.1)]),
    ],
    policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
)
```

不推荐原因：

1. 两个行为共享同一个 `UrMoveClient` 实例和同一个 ZMQ REQ socket。
2. 两个后台线程可能同时对同一个 REQ socket `send_string()`/`recv_string()`，容易出现状态错误或响应错配。
3. 规划服务器单 REP loop 也不会真正并行处理两个规划请求。
4. 即使没有报错，也更可能是“两个请求排队执行”，不是我们要验证的同一阶段双臂并行。

### 不推荐：先做复杂行为树 API

原因：

1. API 层不能绕过底层远程执行顺序发送的问题。
2. 当前 demo 和测试已经证明 `move_to_waypoints()` 能表达双臂 waypoint。
3. 先加 API 容易掩盖真正的并发边界，增加维护成本。

## 最小实施顺序

1. 先写 Demo 1，不改底层代码，使用 `use_remote_execution: false` 验证本地规划服务双臂并行执行。
2. 如果 Demo 1 成功，再写日志/时间统计，确认两个 action goal 的开始时间差。
3. 如果要上真实远程硬件，修改 `TrajectoryExecutorClient.execute_trajectories()` 为并发发送，并同步更新 `ur_bt` 和 `ur_move` 两处客户端副本。
4. 写 Demo 2，使用同样的行为树表达方式验证远程双臂执行。
5. 若用户希望 API 更语义化，再添加 `move_both_to_waypoints()` 作为薄包装，但不要用它创建两个独立 arm move 行为。

## 最终建议

第一版以 demo 为主，不新增行为树 API。

理由是现有行为树 API 已经足够表达“双臂同一阶段动作”，而当前真正需要确认和补齐的是执行链路：本地模式已支持同一请求内并行执行，远程模式需要把客户端从顺序发送改成并发发送。新增行为树 API 可以作为后续易用性优化，但不应作为验证双臂同时执行的核心方案。
