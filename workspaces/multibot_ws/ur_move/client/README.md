# UR Move ZMQ 客户端库

## 简介

`zmq_ur_move_client.py` 轨迹规划服务器 ZMQ 客户端库。支持通过 ZMQ 发送路径点进行轨迹规划，并执行轨迹
`zmq_gripper_client.py` 夹爪服务器的 ZMQ 客户端库。支持通过 ZMQ 发送指令控制夹爪运动
`zmq_tf_client.py` TF服务器的 ZMQ 客户端库。支持通过 ZMQ 发送指令查询TF转换关系

## 安装依赖

```bash
pip install pyzmq
```

## 端口说明

- **5605**: 轨迹规划服务端口
- **5630**: 左手夹爪服务端口
- **5640**: 右手夹爪服务端口
- **5609**: TF 服务端口

---

## 轨迹规划与运动客户端 API

#### 初始化

```python
UrMoveClient(server_host="localhost", timeout_ms=60000)
```

**参数：**
- `server_host` (str): 规划服务器主机地址，默认 `"localhost"`。
- `timeout_ms` (int): 请求超时时间（毫秒），默认 `60000`
- `left_arm_executor_host` (str, optional): 左臂执行服务器主机地址（驱动PC），默认 `None`
- `right_arm_executor_host` (str, optional): 右臂执行服务器主机地址（驱动PC），默认 `None`

**返回：** `UrMoveClient` 实例

---

#### 规划轨迹

```python
plan_trajectory(waypoints: Dict[str, Any]) -> Dict[str, Any]
```

**输入：**
- `waypoints` (Dict[str, Any]): 路径点字典，格式见下方路径点格式说明

**输出：**
- `Dict[str, Any]`: 包含以下字段
  - `success` (bool): 是否成功
  - `execution_id` (str): 用于后续执行
  - `trajectories` (Dict): 包含各组的轨迹数据
  - `error` (str): 失败时的错误信息

**示例：**
```python
# 只规划，不执行
result = client.plan_trajectory(waypoints)
if result['success']:
    execution_id = result['execution_id']
    print(f"规划成功，execution_id: {execution_id}")
```

---

#### 执行已规划的轨迹

```python
execute_trajectory(execution_id: str) -> Dict[str, Any]
```

**输入：**
- `execution_id` (str): 规划时返回的执行ID

**输出：**
- `Dict[str, Any]`: 包含以下字段
  - `success` (bool): 是否成功
  - `error` (str): 失败时的错误信息

**示例：**
```python
# 先规划
result = client.plan_trajectory(waypoints)
if result['success']:
    # 再执行
    exec_result = client.execute_trajectory(result['execution_id'])
    if exec_result['success']:
        print("执行成功")
```

---

#### 规划并执行轨迹

```python
plan_and_execute(waypoints: Dict[str, Any]) -> Dict[str, Any]
```

**输入：**
- `waypoints` (Dict[str, Any]): 路径点字典

**输出：**
- `Dict[str, Any]`: 包含以下字段
  - `success` (bool): 是否成功
  - `trajectories` (Dict): 包含各组的轨迹数据，key为组名称（"left_arm" 或 "right_arm"）
  - `error` (str): 失败时的错误信息

**示例：**
```python
result = client.plan_and_execute(waypoints)
if result['success']:
    trajectories = result['trajectories']
    print(f"规划和执行成功，包含 {len(trajectories)} 个组的轨迹")
else:
    print(f"执行失败: {result.get('error')}")
```

---

#### 在远程驱动PC上执行已规划的轨迹

```python
execute_remote(plan_result: Dict[str, Any]) -> Dict[str, Any]
```

**输入：**
- `plan_result` (Dict[str, Any]): 规划结果字典，应包含 `trajectories` 字段

**输出：**
- `Dict[str, Any]`: 包含以下字段
  - `success` (bool): 是否成功
  - `message` (str): 成功或失败的消息

**示例：**
```python
# 先规划
plan_result = client.plan_trajectory(waypoints)
if plan_result['success']:
    # 再在远程执行
    exec_result = client.execute_remote(plan_result)
    if exec_result['success']:
        print(exec_result['message'])
```

---


#### 规划并在远程驱动PC上执行轨迹

```python
plan_and_execute_remote(waypoints: Dict[str, Any]) -> Dict[str, Any]
```

**输入：**
- `waypoints` (Dict[str, Any]): 路径点字典

**输出：**
- `Dict[str, Any]`: 包含以下字段
  - `success` (bool): 是否成功
  - `message` (str): 成功或失败的消息

**示例：**
```python
# 初始化时配置执行服务器地址
client = UrMoveClient(
    server_host="规划PC地址",
    left_arm_executor_host="驱动PC1",
    right_arm_executor_host="驱动PC2"
)

result = client.plan_and_execute_remote(waypoints)
if result['success']:
    print(result['message'])
```

---

## 路径点格式

JSON 格式路径点，key是路径点名称，value是路径点配置。

### 关节空间路径点（type: "joint"）

```json
{
    "路径点名称": {
        "group": "left_arm",
        "planner": "ptp",
        "type": "joint",
        "joint_names": ["left_joint1", "left_joint2", ...],
        "joint_values": [0.0, -90.0, 90.0, ...],
        "max_velocity_scaling_factor": 0.3,
        "max_acceleration_scaling_factor": 0.3
    }
}
```

**必需字段：**
- `group` (str): 机械臂组名称（"left_arm" 或 "right_arm"）
- `planner` (str): 规划器类型（"ptp", "lin", "ompl"）
- `type` (str): "joint"
- `joint_names` (List[str]): 关节名称数组
- `joint_values` (List[float]): 关节角度值数组（单位：度）

**可选字段：**
- `description` (str): 路径点描述
- `max_velocity_scaling_factor` (float): 最大速度缩放因子（0.0-1.0）
- `max_acceleration_scaling_factor` (float): 最大加速度缩放因子（0.0-1.0）

**注意**：关节名称支持以下格式：
1. **推荐格式**：
   - `left_joint1`-`left_joint6` 或 `right_joint1`-`right_joint6`（带前缀，更清晰）
   - `left_joint1` → `left_shoulder_pan_joint`
   - `right_joint1` → `right_shoulder_pan_joint`
2. **兼容格式**：
   - `joint1`-`joint6`（系统会根据 `group` 字段自动添加前缀）
   - `left_arm` + `joint1` → `left_shoulder_pan_joint`
   - `right_arm` + `joint1` → `right_shoulder_pan_joint`
3. **直接使用实际关节名称**（如 `left_shoulder_pan_joint`），系统也会正常处理

### 笛卡尔空间路径点（type: "cart"）

```json
{
    "路径点名称": {
        "group": "left_arm",
        "planner": "lin",
        "type": "cart",
        "ik_frame": "left_ee_link",
        "frame_id": "left_interface_link",
        "position": [0.6, 0.2, 0.35],
        "orientation": [0.382683, 0, 0, 0.923880],
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1
    }
}
```

**必需字段：**
- `group` (str): 机械臂组名称
- `planner` (str): 规划器类型（"ptp", "lin", "ompl"）
- `type` (str): 必须为 "cart"
- `ik_frame` (str): 逆运动学参考坐标系（通常是末端执行器坐标系）
- `frame_id` (str): 目标参考坐标系（通常是机器人基坐标系）
- `position` (List[float]): 目标位置 [x, y, z]（单位：米）
- `orientation` (List[float]): 目标姿态四元数 [x, y, z, w]

**可选字段：**
- `description` (str): 路径点描述
- `max_velocity_scaling_factor` (float): 最大速度缩放因子（0.0-1.0）
- `max_acceleration_scaling_factor` (float): 最大加速度缩放因子（0.0-1.0）

### 规划器类型

- `ptp`: 点到点规划（Point-to-Point），适用于关节空间运动
- `lin`: 直线规划（Linear），适用于笛卡尔空间的直线运动
- `ompl`: OMPL 规划器，适用于复杂路径规划

---

## 夹爪客户端 API

#### 初始化

```python
GripperZMQClient(server_host="localhost", port=5630, gripper_name="left", timeout_ms=20000)
```

**参数：**
- `server_host` (str): 服务器地址，默认 `"localhost"`
- `port` (int): 服务器端口，默认 `5630`（左手）或 `5640`（右手）
- `gripper_name` (str): 夹爪名称，默认 `"left"`（"left" 或 "right"）
- `timeout_ms` (int): 请求超时时间（毫秒），默认 `20000`

**返回：** `GripperZMQClient` 实例

---

#### 打开夹爪

```python
open(max_effort: float = 50.0) -> bool
```

**输入：**
- `max_effort` (float): 最大力度（N），默认 `50.0`

**输出：**
- `bool`: 是否成功

**示例：**
```python
gripper = GripperZMQClient(gripper_name="left", port=5630)
if gripper.open(max_effort=50.0):
    print("夹爪打开成功")
```

---

#### 关闭夹爪

```python
close(max_effort: float = 50.0) -> bool
```

**输入：**
- `max_effort` (float): 最大力度（N），默认 `50.0`

**输出：**
- `bool`: 是否成功

**示例：**
```python
if gripper.close(max_effort=50.0):
    print("夹爪关闭成功")
```

---

#### 设置夹爪位置

```python
set_position(position: float, max_effort: float = 50.0) -> bool
```

**输入：**
- `position` (float): 夹爪位置，范围 0.0-0.8（0.0 = 完全打开, 0.8 = 完全关闭）
- `max_effort` (float): 最大力度（N），默认 `50.0`

**输出：**
- `bool`: 是否成功

**示例：**
```python
# 设置夹爪到半开位置
if gripper.set_position(position=0.4, max_effort=50.0):
    print("夹爪位置设置成功")
```

---

## TF 客户端 API

#### 初始化

```python
TFZMQClient(server_host="localhost", server_port=5609, timeout=5)
```

**参数：**
- `server_host` (str): TF服务器IP，默认 `"localhost"`
- `server_port` (int): TF服务器端口，默认 `5609`
- `timeout` (int): 超时时间（秒），默认 `5`

**返回：** `TFZMQClient` 实例

---

#### 查询坐标变换

```python
lookup_transform(source_frame: str, target_frame: str) -> Optional[Dict[str, Any]]
```

**输入：**
- `source_frame` (str): 源坐标系
- `target_frame` (str): 目标坐标系

**输出：**
- `Optional[Dict[str, Any]]`: 成功时返回包含以下字段的字典，失败返回 `None`
  - `success` (bool): 是否成功
  - `message` (str): 消息
  - `data` (Dict): 包含以下字段
    - `translation` (Dict): 平移 {'x': float, 'y': float, 'z': float}
    - `rotation` (Dict): 旋转（四元数）{'x': float, 'y': float, 'z': float, 'w': float}

**示例：**
```python
tf_client = TFZMQClient()
transform = tf_client.lookup_transform("left_interface_link", "left_ee_link")
if transform and transform.get('success'):
    translation = transform['data']['translation']
    rotation = transform['data']['rotation']
    print(f"平移: {translation}")
    print(f"旋转: {rotation}")
```

---

#### 获取平移部分

```python
get_translation(source_frame: str, target_frame: str) -> Optional[Dict[str, float]]
```

**输入：**
- `source_frame` (str): 源坐标系
- `target_frame` (str): 目标坐标系

**输出：**
- `Optional[Dict[str, float]]`: 平移字典 {'x': float, 'y': float, 'z': float}，失败返回 `None`

**示例：**
```python
translation = tf_client.get_translation("left_interface_link", "left_ee_link")
if translation:
    print(f"平移: x={translation['x']}, y={translation['y']}, z={translation['z']}")
```

---

#### 获取旋转部分

```python
get_rotation(source_frame: str, target_frame: str) -> Optional[Dict[str, float]]
```

**输入：**
- `source_frame` (str): 源坐标系
- `target_frame` (str): 目标坐标系

**输出：**
- `Optional[Dict[str, float]]`: 旋转字典（四元数）{'x': float, 'y': float, 'z': float, 'w': float}，失败返回 `None`

**示例：**
```python
rotation = tf_client.get_rotation("left_interface_link", "left_ee_link")
if rotation:
    print(f"旋转: x={rotation['x']}, y={rotation['y']}, z={rotation['z']}, w={rotation['w']}")
```

---

#### 使用 with 语句

```python
with TFZMQClient() as tf_client:
    transform = tf_client.lookup_transform("left_interface_link", "left_ee_link")
    # 连接会在退出 with 块时自动关闭
```

---

## 注意事项

1. 必须确保服务器已启动
2. 路径点必须是有效的 JSON 格式
3. 关节角度单位是度（degrees）
4. 位置单位是米（meters）
5. 四元数格式为 [x, y, z, w]
6. 连接会在首次调用时自动建立，资源会在对象销毁时自动清理
7. 左右夹爪独立端口：左手 5630，右手 5640
8. TF 客户端支持 with 语句，可自动管理连接生命周期
9. 远程执行需要配置对应手臂的执行服务器地址，未配置的手臂会被跳过
