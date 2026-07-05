# 机器人控制测试脚本

## 概述

本目录包含两个测试脚本：
- `test_arm_control.py` - 机械臂控制脚本
- `test_gripper_control.py` - 夹爪控制脚本

## 前置条件

确保已启动机器人控制系统：
```bash
ros2 launch dual_arm_config control.launch.py use_fake_gripper_hardware:=true
```

## 机械臂控制

### 基本用法

```bash
# 移动左臂到指定关节位置（6个关节，单位：弧度）
ros2 run dual_arm_config test_arm_control.py --arm left --position "0.0 -1.57 0.0 -1.57 -1.57 0.0"

# 移动右臂到零位
ros2 run dual_arm_config test_arm_control.py --arm right --position "0.0 0.0 0.0 0.0 0.0 0.0"
```

### 交互模式

```bash
ros2 run dual_arm_config test_arm_control.py --arm left --interactive
```

在交互模式下，可以输入：
- 6个关节位置值（空格分隔，单位：弧度），例如：`0.0 -1.57 0.0 -1.57 -1.57 0.0`
- `q` 或 `quit` - 退出

### 参数说明

- `--arm`: 选择要控制的手臂 (`left` 或 `right`，默认: `left`)
- `--position`: 6个关节位置（弧度），空格分隔
- `--interactive`: 启动交互模式

### 关节顺序

UR7e 机器人有 6 个关节，按顺序为：
1. `shoulder_pan_joint` - 肩部旋转
2. `shoulder_lift_joint` - 肩部抬升
3. `elbow_joint` - 肘部
4. `wrist_1_joint` - 腕部1
5. `wrist_2_joint` - 腕部2
6. `wrist_3_joint` - 腕部3

## 夹爪控制

### 基本用法

```bash
# 打开左夹爪 (0.0 = 完全打开, 0.8 = 完全关闭)
ros2 run dual_arm_config test_gripper_control.py --gripper left --position 0.0

# 关闭右夹爪
ros2 run dual_arm_config test_gripper_control.py --gripper right --position 0.8
```

### 交互模式

```bash
ros2 run dual_arm_config test_gripper_control.py --gripper left --interactive
```

在交互模式下，可以使用：
- `open` - 完全打开
- `close` - 完全关闭
- `half` - 半开
- `0.0-0.8` - 具体位置值
- `q` - 退出

### 参数说明

- `--gripper`: 选择夹爪 (`left` 或 `right`，默认: `left`)
- `--position`: 位置值 (0.0 = 打开, 0.8 = 关闭)
- `--max-effort`: 最大力度，单位 N (默认: 50.0)
- `--interactive`: 启动交互模式

## 故障排除

### 无法连接到 action server

**检查步骤：**

1. 检查控制器状态：
   ```bash
   ros2 control list_controllers
   ```

2. 检查 action topics：
   ```bash
   ros2 action list
   ```
   
   应该看到：
   - `/left_arm_controller/follow_joint_trajectory`
   - `/right_arm_controller/follow_joint_trajectory`
   - `/left_gripper_controller/gripper_cmd`
   - `/right_gripper_controller/gripper_cmd`

3. 如果控制器未启动，手动激活：
   ```bash
   ros2 control switch_controllers --activate left_arm_controller
   ros2 control switch_controllers --activate right_arm_controller
   ros2 control switch_controllers --activate left_gripper_controller
   ros2 control switch_controllers --activate right_gripper_controller
   ```

## 注意事项

1. 这些脚本主要用于测试，在模拟硬件模式下使用
2. 机械臂关节位置单位为弧度
3. 夹爪位置值范围应在 0.0 到 0.8 之间
4. 确保在运行脚本前，机器人控制系统已正确启动
