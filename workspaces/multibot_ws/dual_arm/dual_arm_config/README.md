# Distillation Config

蒸馏机器人描述和配置包。

## 包含内容

- **description/urdf/**: URDF/Xacro 机器人描述文件
  - `scene.urdf.xacro`: 场景文件
  - `dual_arm.urdf.xacro`: 双臂机器人宏定义
  
- **description/meshes/**: STL 网格文件
  - `base.STL`: 底座模型
  - `body.STL`: 主体模型
  - `left_interface_link.STL`: 左接口连接模型
  - `right_interface_link.STL`: 右接口连接模型

- **launch/**: 启动文件
  - `view_distillation.launch.py`: 在 RViz 中可视化机器人

- **config/**: 配置文件
  - `view_robot.rviz`: RViz 配置

## 使用方法

### 编译包

```bash
cd ~/ur_ws
colcon build --packages-select dual_arm_config
source install/setup.bash
```

### 可视化机器人

```bash
ros2 launch dual_arm_config view_distillation.launch.py
```

### 可选参数

- `ur_type`: UR机器人类型 (默认: ur5)
- `use_rviz`: 是否启动RViz (默认: true)
- `safety_limits`: 是否启用安全限制 (默认: true)

示例：

```bash
ros2 launch dual_arm_config view_distillation.launch.py ur_type:=ur10 use_rviz:=true
```

## 依赖

- ur_description
- joint_state_publisher_gui
- robot_state_publisher
- rviz2
- xacro

