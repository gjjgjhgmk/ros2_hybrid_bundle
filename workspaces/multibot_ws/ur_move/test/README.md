# UR Move 测试

## 文件说明

- `waypoints.json` - 路径点配置文件
- `test_client.py` - 测试脚本（支持手臂轨迹规划和夹爪控制）


## 快速开始

### 1. 启动服务器

```bash
# 在 docker 中启动轨迹规划服务器
ros2 launch ur_move ur_move_server.launch.py use_fake_gripper_hardware:=true
```

### 2. 运行测试

```bash
cd ur_move/test
```

### 参数说明

`--mode` 参数选择执行模式（必需）：
- `arm`: 规划 执行手臂
- `gripper`: 控制夹爪

### 手臂轨迹规划（arm 模式）

```bash
# 只规划，不执行（返回 execution_id）
python3 test_client.py --mode arm --names "左臂-home" --no-execute

# 规划并等待确认后执行（使用 execution_id 机制）
python3 test_client.py --mode arm --names "左臂-home" "右臂-home" --wait-confirm

# 规划后自动执行（服务器端立即执行）
python3 test_client.py --mode arm --names "左臂-home"
```

### 夹爪控制（gripper 模式）

```bash
# 打开夹爪（使用默认力 50N）
python3 test_client.py --mode gripper --gripper-action open --gripper-name left

# 关闭夹爪（使用默认力 50N）
python3 test_client.py --mode gripper --gripper-action close --gripper-name right

# 设置夹爪位置（0.0=打开, 0.8=关闭，使用默认力 50N）
python3 test_client.py --mode gripper --gripper-position 0.4 --gripper-name left

# 设置夹爪力（范围: 0-235N）
python3 test_client.py --mode gripper --gripper-position 0.5 --gripper-name right --gripper-effort 30.0
```

## 路径点格式

### 关节空间路径点

```json
"左臂-home": {
        "group": "left_arm",
        "planner": "ptp",
        "description": "左臂运动到home位置",
        "type": "joint",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "joint_names": [
            "left_joint1",
            "left_joint2",
            "left_joint3",
            "left_joint4",
            "left_joint5",
            "left_joint6"
        ],
        "joint_values": [
            0.0,
            -90.0,
            0.0,
            -90.0,
            -90.0,
            0.0
        ]
    }
```

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

### 笛卡尔空间路径点

```json
"左臂-笛卡尔目标": {
        "group": "left_arm",
        "planner": "ptp",
        "description": "左臂运动到笛卡尔目标点",
        "type": "cart",
        "max_velocity_scaling_factor": 0.1,
        "max_acceleration_scaling_factor": 0.1,
        "ik_frame": "left_ee_link",
        "frame_id": "left_base_link",
        "position": [
            0.6,
            0.2,
            0.3
        ],
        "orientation": [
            0, 
            0, 
            0, 
            1
        ]
    }
```
**注意**：
1. **关键条目解释**：
   - `ik_frame` ：逆运动学坐标系
   - `frame_id` ：基坐标系
   - `orientation` ：四元数x,y,z,w

### 支持的规划器

- `ptp`: Pilz PTP 规划器（点到点）
- `lin`: Pilz LIN 规划器（直线）
- `ompl`: OMPL RRTConnect 规划器（默认）


### 注意事项

1. 路径点中的关节值使用角度（度），内部会自动转换为弧度
2. 轨迹规划和执行在服务器端完成，确保服务器正在运行