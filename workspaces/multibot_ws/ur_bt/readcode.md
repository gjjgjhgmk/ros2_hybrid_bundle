有这么一些对象:

- xxx behavior
- behavior tree
- xxx client
- blackboard
- behavior_tree_manager
- blackboard_manager
- xxx behavior factory
- waypoint

## 行为树的基本含义

行为树是一种任务编排框架。它把复杂任务拆成多个行为节点，并用树结构决定这些行为节点的执行顺序、成功/失败传播方式，以及是否继续等待某个动作完成。

本项目使用的是 `py_trees`。每个行为节点被 tick 之后会返回一个状态：

- `SUCCESS`: 行为完成并成功
- `FAILURE`: 行为完成但失败
- `RUNNING`: 行为还在执行中
- `INVALID`: 行为还没有运行或已被重置

当前项目里 `BehaviorTreeManager._create_tree()` 创建的是一个 `Sequence` 根节点：

```text
Sequence
├── behavior 1
├── behavior 2
├── behavior 3
└── behavior n
```

`Sequence` 的含义是：从上到下依次执行。前一个 behavior 返回 `SUCCESS` 后，才执行下一个；如果某个 behavior 返回 `FAILURE`，整个序列失败；如果某个 behavior 返回 `RUNNING`，行为树会继续 tick，等待它完成。

## xxx behavior

`xxx behavior` 表示行为树中的一个具体行为节点，也就是任务中的一个动作。

例如：

- `ArmMoveToWaypoints`: 让机械臂移动到一个或多个 waypoint
- `VisionPoseEstimationMask`: 调用视觉服务并保存识别结果
- `GripperSetPosition`: 控制夹爪位置
- `Sleep`: 等待一段时间
- `BlackboardWriter`: 往黑板写数据

这些 behavior 通常继承自 `py_trees.behaviour.Behaviour`，并通过这些生命周期函数工作：

- `setup()`: 行为树启动前的准备
- `initialise()`: 行为第一次开始执行时调用
- `update()`: 每次 tick 时调用，返回 `SUCCESS`、`FAILURE` 或 `RUNNING`
- `terminate()`: 行为结束时调用

本项目里的机械臂、视觉、夹爪 behavior 多数是异步的：`initialise()` 中启动后台线程，`update()` 中检查后台任务是否完成。

## xxx behavior factory

`xxx behavior factory` 负责创建/组装具体的 behavior 节点。

它本身通常不是具体动作，而是一个用于构造动作节点的工厂对象。

例如：

- `ArmMoveBehavior` 是 factory，调用 `move_to_waypoints()` 后创建 `ArmMoveToWaypoints`
- `VisionBehavior` 是 factory，调用 `pose_estimation_mask()` 后创建 `VisionPoseEstimationMask`
- `GripperBehavior` 是 factory，调用 `open()`、`close()`、`set_position()` 后创建 `GripperSetPosition`
- `UtilityBehavior` 是 factory，调用 `sleep()` 后创建 `Sleep`

所以关系是：

```text
xxx Behavior factory
  ↓ 创建
具体 xxx behavior node
```

## behavior tree

`behavior tree` 是由多个 behavior 节点组成的树。

具体执行动作的 behavior 一般分布在行为树的叶子节点。内部节点通常是控制节点，用来决定哪些叶子节点被 tick，以及以什么规则推进执行。

常见结构可以理解为：

```text
控制节点
├── 控制节点
│   ├── 具体 behavior 叶子节点
│   └── 具体 behavior 叶子节点
└── 具体 behavior 叶子节点
```

也就是说，`ArmMoveToWaypoints`、`VisionPoseEstimationMask`、`GripperSetPosition`、`Sleep` 这类具体动作节点通常是叶子节点；`Sequence`、`Selector`、`Parallel` 这类节点主要负责控制执行逻辑。

在当前项目中，demo 通常传入一个 behavior 列表：

```python
behaviors = [behavior1, behavior2, behavior3]
bt_manager.execute(behaviors, wait=True)
```

`BehaviorTreeManager` 会把这个列表包装成一个 `Sequence` 行为树，然后循环 tick 这棵树。

当前项目主要把行为树用作“顺序任务执行器”。行为树本身还可以表达选择、并行、条件检查、恢复策略等复杂逻辑，但当前 demo 中使用得比较简单。

### Sequence

`Sequence` 表示顺序执行。

```text
Sequence
├── A: 移动到 home
├── B: 打开夹爪
└── C: 等待 1 秒
```

执行规则：

- A 返回 `SUCCESS` 后执行 B
- B 返回 `SUCCESS` 后执行 C
- A/B/C 全部 `SUCCESS`，整个 `Sequence` 才 `SUCCESS`
- 任意一个子节点返回 `FAILURE`，整个 `Sequence` 立即 `FAILURE`
- 当前子节点返回 `RUNNING`，整个 `Sequence` 返回 `RUNNING`，下次 tick 继续等待它

适合表达：先做 A，再做 B，再做 C。

### Selector

`Selector` 表示从多个方案中选择一个可行方案，也叫 fallback。

```text
Selector
├── A: 使用视觉识别焊点
├── B: 使用预设焊点
└── C: 报错退出
```

执行规则：

- A 返回 `SUCCESS`，整个 `Selector` 立即 `SUCCESS`，不再执行 B/C
- A 返回 `FAILURE`，继续尝试 B
- B 返回 `SUCCESS`，整个 `Selector` 立即 `SUCCESS`
- 所有子节点都 `FAILURE`，整个 `Selector` 才 `FAILURE`
- 当前子节点返回 `RUNNING`，整个 `Selector` 返回 `RUNNING`，下次 tick 继续等待它

适合表达：优先尝试 A，如果 A 失败就尝试 B，如果 B 失败再尝试 C。

### Parallel

`Parallel` 表示同时 tick 多个子节点，再根据策略决定整体状态。

```text
Parallel
├── A: 左臂移动到安全位
└── B: 右臂移动到准备位
```

常见策略是 `SuccessOnAll`，表示所有子节点都成功才成功：

- A 和 B 都返回 `SUCCESS`，整个 `Parallel` 返回 `SUCCESS`
- 任意一个子节点返回 `FAILURE`，整个 `Parallel` 返回 `FAILURE`
- 只要还有子节点是 `RUNNING`，整个 `Parallel` 返回 `RUNNING`

适合表达：多个动作可以同时进行，全部完成后再继续。

## xxx client

`xxx client` 是外部系统的通信适配器。它不负责行为树调度，而是负责通过 ZMQ 等协议和外部服务通信。

例如：

- `UrMoveClient`: 和机械臂轨迹规划/执行服务通信
- `ZMQVisionClient`: 和视觉服务通信
- `GripperZMQClient`: 和夹爪服务通信
- `TFClient`: 和 TF 坐标变换服务通信

典型关系是：

```text
BehaviorTreeManager
  ↓ 初始化 client
xxx client
  ↓ 传给 behavior factory
xxx behavior factory
  ↓ 创建 behavior 时注入 client
xxx behavior
  ↓ 执行时调用 client
外部 ZMQ 服务
```

以机械臂为例：

```text
BehaviorTreeManager 创建 UrMoveClient
  ↓
UrMoveClient 被传给 ArmMoveBehavior factory
  ↓
ArmMoveBehavior.move_to_waypoints() 创建 ArmMoveToWaypoints
  ↓
ArmMoveToWaypoints._execute_task() 调用 UrMoveClient.plan_and_execute_remote()
```

不是所有 behavior 都需要 client。例如 `Sleep` 不需要 client，`BlackboardWriter` 只需要 `BlackboardManager`，`ArmUpdateWaypointFromVision` 主要读写黑板，不直接调用视觉 client。

## waypoint

`waypoint` 表示机械臂目标点，但项目中没有一个明确的 `Waypoint` class 与它对应。它是一个隐式定义的数据协议：

- 数据来源由 `waypoints.json` 给出
- 数据加载由 `ArmWaypointManager` 完成
- 数据存储在 blackboard 的 `arm_waypoints_data` 中
- 数据格式由 `ArmMoveToWaypoints`、`ArmUpdateWaypointFromVision`、`ArmUpdateWaypointFromWaypoint`、`UrMoveClient` 等使用方共同约定

也就是说，waypoint 的“定义”不集中在一个 class 中，而是分散在“加载方式”和“使用语义”中。

### 格式

`waypoints.json` 顶层是一个字典，key 是 waypoint 名称，value 是该 waypoint 的字段字典。

```json
{
  "右臂-home": {
    "group": "right_arm",
    "planner": "ptp",
    "type": "joint",
    "joint_names": ["right_joint1", "right_joint2"],
    "joint_values": [90.0, -90.0]
  }
}
```

通用字段：

- `group`: 目标属于哪条机械臂，例如 `left_arm` 或 `right_arm`。底层规划服务会根据这个字段决定使用哪组机器人关节和执行器。
- `planner`: 规划方式，例如 `ptp`、`lin`、`ompl`。`ptp` 通常表示点到点运动，`lin` 通常表示笛卡尔直线运动，具体含义由底层 `ur_move` 服务解释。
- `description`: 人类可读描述，主要用于说明，不是运动执行的核心字段。
- `type`: waypoint 类型，当前常见值是 `joint` 和 `cart`。
- `max_velocity_scaling_factor`: 默认速度缩放系数。如果 `move_to_waypoints()` 调用时传入了速度缩放，会覆盖这个值。
- `max_acceleration_scaling_factor`: 默认加速度缩放系数。如果 `move_to_waypoints()` 调用时传入了加速度缩放，会覆盖这个值。

`joint` 类型字段：

- `joint_names`: 关节名称列表。
- `joint_values`: 与 `joint_names` 对应的关节目标值列表。

`cart` 类型字段：

- `ik_frame`: 做逆运动学时使用的末端执行器坐标系，例如 `right_ee_link`。
- `frame_id`: `position` 和 `orientation` 所在的参考坐标系，例如 `right_interface_link`、`left_base_link`。
- `position`: 笛卡尔位置 `[x, y, z]`。
- `orientation`: 四元数姿态 `[qx, qy, qz, qw]`。

### 语义

代码中引用 waypoint 时通常只传名称：

```python
bt_manager.arm_move_behavior.move_to_waypoints(
    waypoint_configs=[("右臂-home", 0.1, 0.1)]
)
```

`ArmMoveToWaypoints` 会根据这个名称从 `blackboard["arm_waypoints_data"]` 中取出具体数据。

`ArmMoveToWaypoints` 使用 waypoint 的方式：

- 从 blackboard 读取 `arm_waypoints_data`
- 按 waypoint 名称查找字段字典
- 如果调用时传入了速度/加速度缩放，则覆盖 waypoint 内部默认值
- 把多个 waypoint 组装成 `{waypoint_name: waypoint_data}` 字典
- 调用 `UrMoveClient.plan_and_execute_remote()` 或 `UrMoveClient.plan_and_execute()`

`waypoint_configs` 三元组的语义是：

- 第 1 项：waypoint 名称，用于在 `arm_waypoints_data` 中查找具体数据
- 第 2 项：本次执行使用的速度缩放；如果是 `None`，使用 waypoint 自带默认值
- 第 3 项：本次执行使用的加速度缩放；如果是 `None`，使用 waypoint 自带默认值

`UrMoveClient` 会把 JSON 顶层 key 自动补成 `name` 字段，因此 `name` 通常不需要预先写在 JSON 中。

`ArmWaypointBehavior` 使用 waypoint 的方式：

- `ArmUpdateWaypoint` 直接更新某个 waypoint 的字段
- `ArmUpdateWaypointFromVision` 从 `vision_results` 取位姿，更新目标 waypoint 的 `position` 和 `orientation`
- `ArmUpdateWaypointFromWaypoint` 从源 waypoint 复制并偏移，更新目标 waypoint 的 `position` 和 `orientation`

结论：waypoint 是项目内部约定的一种字典格式，而不是一个强类型对象。修改或新增 waypoint 时，要同时满足 JSON 文件格式、更新行为的字段假设，以及底层 `ur_move` 规划服务对字段的要求。

## blackboard

`blackboard` 是行为树节点之间共享数据的空间。

不同 behavior 之间通常不直接互相调用，而是通过 blackboard 传递数据。

例如视觉到机械臂 waypoint 的数据流：

```text
VisionPoseEstimationMask
  ↓ 写入
blackboard["vision_results"]
  ↓ 读取
ArmUpdateWaypointFromVision
  ↓ 更新
blackboard["arm_waypoints_data"]
  ↓ 读取
ArmMoveToWaypoints
  ↓ 执行机械臂运动
```

当前项目里最重要的 blackboard key：

- `arm_waypoints_data`: 从 `waypoints.json` 加载来的所有机械臂 waypoint 数据
- `vision_results`: 视觉识别行为保存的识别结果

blackboard 的作用是解耦行为节点：视觉行为只负责写识别结果，waypoint 更新行为只负责读识别结果并改 waypoint，机械臂运动行为只负责读 waypoint 并执行。

## blackboard_manager

`BlackboardManager` 是对 `py_trees.blackboard` 的封装，提供统一访问接口。

常用接口：

```python
set(key, value)
get(key, default)
register_key(key, access)
clear_all()
display_blackboard()
```

可以理解为：

```text
Blackboard 是真实的数据空间
BlackboardManager 是访问这个数据空间的工具类
```

## behavior_tree_manager

`BehaviorTreeManager` 是本项目的总调度器和运行时容器。

它负责：

- 读取 `config.yaml`
- 初始化机械臂、视觉、夹爪、TF 等 client
- 初始化 `BlackboardManager`
- 通过 `ArmWaypointManager` 把 `waypoints.json` 加载到 blackboard 的 `arm_waypoints_data`
- 创建各种 behavior factory
- 接收 demo 传入的 behavior 列表
- 创建 behavior tree
- 启动 tick 循环并等待执行结果
- 清理资源

可以把它理解成：

```text
BehaviorTreeManager
├── config
├── clients
├── blackboard_manager
├── waypoint_manager
├── behavior factories
└── execute()
```

## 总体关系

整体调用关系：

```text
业务 demo
  ↓ 调用
BehaviorTreeManager
  ↓ 创建
Behavior Tree
  ↓ tick
xxx behavior
  ↓ 调用
xxx client
  ↓ ZMQ
外部机器人/视觉/夹爪/TF 服务
```

数据共享关系：

```text
xxx behavior
  ↓ get/set
BlackboardManager
  ↓ access
Blackboard
```

简化记忆：

- `BehaviorTreeManager`: 总调度器
- `behavior tree`: 行为节点组成的执行结构
- `xxx behavior`: 一个具体动作节点
- `xxx behavior factory`: 创建具体动作节点的工厂
- `xxx client`: 外部服务通信对象
- `blackboard`: 行为节点之间共享数据的地方
- `blackboard_manager`: 访问 blackboard 的封装工具

### 做焊点序列demo

支持定义笛卡尔空间中的目标，设定好目标序列之后逐个执行应该就行了