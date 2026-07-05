# 测试文件说明

本目录包含用于测试行为树功能的测试脚本。

## 文件说明

- `test_arm_behavior.py`: 手臂行为测试脚本
- `test_gripper_behavior.py`: 夹爪行为测试脚本
- `test_combination_behaviror.py`: 手臂和夹爪联合行为测试脚本
- `test_tf_behavior.py`: TF 服务测试脚本
- `test_vision_behavior.py`: 视觉检测行为测试脚本
- `waypoints.json`: 测试用的路径点配置文件

## 使用方法

### 1. 手臂行为测试

```bash
cd /home/yanjiapei/multibot_ws/ur_bt/test
python test_arm_behavior.py
```
**测试1: 单个路径点运动**
测试单个路径点的运动效果，例如移动到home位置。

**测试2: 多个路径点序列运动**
测试多个路径点按顺序执行的运动效果，包括延时控制。

**测试3: 双臂同时运动**
测试左右臂同时移动到不同位置的运动效果。

**测试4: 不同速度测试**
测试不同速度缩放系数下的运动效果（慢速、中速、快速）。

**测试5: 自定义路径点运动**
自动检测waypoints.json中的路径点，并测试第一个可用路径点。

### 2. 夹爪行为测试

```bash
cd /home/yanjiapei/multibot_ws/ur_bt/test
python test_gripper_behavior.py
```
**测试1: 单个夹爪打开和关闭**
测试单个夹爪的打开和关闭操作。

**测试2: 设置夹爪位置**
测试设置夹爪到不同位置（完全打开、半开、完全关闭等）。

**测试3: 同时控制两个夹爪**
测试同时控制左右两个夹爪的操作。

### 3. 手臂和夹爪联合行为测试

```bash
cd /home/yanjiapei/multibot_ws/ur_bt/test
python test_combination_behaviror.py
```
**测试1: 抓取和放置动作**
测试完整抓取和放置流程：移动到抓取位置 → 打开夹爪 → 移动到目标位置 → 关闭夹爪 → 移动到放置位置 → 打开夹爪。

**测试2: 双臂和双夹爪联合运动**
测试双臂和双夹爪的协调运动，包括同时移动和同时控制夹爪。

**测试3: 运动过程中控制夹爪**
测试在手臂运动过程中控制夹爪位置的变化。

**测试4: 复杂序列操作**
测试包含多个步骤的复杂操作序列，模拟实际应用场景。

### 4. TF 服务测试

```bash
cd /home/yanjiapei/multibot_ws/ur_bt/test
python test_tf_behavior.py
```
**测试1: 基本坐标变换查询**
测试基本的坐标变换查询功能，验证 TF 服务是否正常工作。

**测试2: 无效坐标系处理**
测试当查询不存在的坐标系时，服务是否能正确处理错误。

**测试3: 连接超时处理**
测试当 TF 服务器不可用时，客户端是否能正确处理超时。

**测试4: 多次连续查询**
测试连续多次查询坐标变换的稳定性和性能。

### 5. 视觉检测行为测试

```bash
cd /home/yanjiapei/multibot_ws/ur_bt/test
python test_vision_behavior.py
```
**测试: 视觉位姿估计（掩码方法）**
测试基于掩码的视觉位姿估计功能，验证视觉检测和位姿估计是否正常工作。测试完成后会显示检测到的对象数量和位姿信息。


## 配置要求

### 1. 配置文件
确保 `../config.yaml` 文件配置正确，特别是：
- `zmq.arm.ur_move.host`: ur_move服务器地址
- `zmq.arm.ur_move.port`: ur_move服务器端口（默认5605）
- `zmq.gripper.left.port`: 左手夹爪服务器端口（默认5630）
- `zmq.gripper.right.port`: 右手夹爪服务器端口（默认5640）
- `zmq.tf.port`: TF服务端口（默认5609）
- `zmq.vision.host`: 视觉服务器地址
- `zmq.vision.port`: 视觉服务器端口

### 2. 路径点文件
`waypoints.json` 文件应包含要测试的路径点定义。格式如下：

```json
{
  "路径点名称": {
    "name": "路径点名称",
    "type": "cart" 或 "joint",
    "group": "left_arm" 或 "right_arm",
    "planner": "ptp" 或 "lin",
    "max_velocity_scaling_factor": 0.1,
    "max_acceleration_scaling_factor": 0.1,
    "position": [x, y, z],
    "orientation": [qx, qy, qz, qw]
  }
}
```

## 注意事项

1. **安全第一**: 测试前请确保机器人周围没有障碍物
2. **速度设置**: 测试使用较低的速度（0.05-0.1）
3. **路径点检查**: 确保waypoints.json中的路径点在机器人工作空间内
4. **服务器连接**: 
   - 确保ur_move服务器正在运行（轨迹规划和执行）
   - 确保夹爪ZMQ服务器正在运行（夹爪控制）
   - 确保TF ZMQ服务器正在运行（坐标变换查询，端口5609）
   - 确保视觉ZMQ服务器正在运行（视觉检测和位姿估计）
5. **ROS 2环境**: 确保ROS 2环境已正确配置（用于轨迹执行、夹爪控制和TF树）
6. **TF树**: 运行TF测试前，确保ROS 2 TF树中有相应的坐标系发布
7. **相机话题**: 运行视觉测试前，确保相机话题正在发布（RGB、深度图像和相机信息）
8. **联合测试**: 运行联合运动测试前，建议先单独测试手臂和夹爪功能

## 故障排除

### 连接失败
- 检查ur_move服务器是否运行
- 检查TF ZMQ服务器是否运行（端口5609）
- 检查config.yaml中的服务器地址和端口

### TF查询失败
- 检查TF ZMQ服务器是否正在运行
- 检查ROS 2 TF树中是否存在要查询的坐标系
- 使用 `ros2 run tf2_tools view_frames` 查看当前TF树结构
- 检查坐标系名称拼写是否正确

### 路径点不存在
- 检查waypoints.json中是否包含要测试的路径点名称
- 检查路径点名称拼写是否正确

### 执行超时
- 检查机器人硬件状态
- 检查路径点是否在机器人工作空间内
- 增加超时时间配置

### 轨迹规划失败
- 检查路径点数据格式是否正确
- 检查路径点是否可达
- 查看日志获取详细错误信息

### 视觉检测失败
- 检查视觉ZMQ服务器是否正在运行
- 检查相机话题是否正在发布（RGB、深度图像和相机信息）
- 检查相机名称是否正确（left_camera 或 right_camera）
- 检查目标坐标系是否存在
- 查看日志获取详细错误信息

## 日志

测试过程中的日志会输出到控制台，详细日志保存在 `../ur_bt.log` 文件中。

