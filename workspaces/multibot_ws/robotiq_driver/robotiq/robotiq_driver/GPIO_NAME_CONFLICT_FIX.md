# Robotiq 夹爪 GPIO 名称冲突修复文档

## 问题描述

在使用多个 Robotiq 夹爪时（例如左夹爪和右夹爪），无法同时激活两个夹爪的激活控制器。错误信息显示：

```
Resource conflict for controller 'right_gripper_activation_controller'. 
Command interface 'reactivate_gripper/reactivate_gripper_cmd' is already claimed.
```

## 问题原因

1. **URDF 中的硬编码 GPIO 名称**：原始 `robotiq_description` 包中的 ros2_control 配置文件将 GPIO 名称硬编码为 `reactivate_gripper`，没有使用 `prefix` 参数，导致多个夹爪共享同一个 GPIO 名称。

2. **硬件接口使用硬编码名称**：`RobotiqGripperHardwareInterface` 在导出命令接口时硬编码使用 `"reactivate_gripper"` 作为 GPIO 名称。

3. **控制器无法区分多个 GPIO**：`RobotiqActivationController` 无法区分不同夹爪的 GPIO 接口，导致资源冲突。

## 解决方案

### 核心思路

通过创建修复版的 ros2_control 配置文件，使用 `${prefix}reactivate_gripper` 作为 GPIO 名称，使每个夹爪拥有唯一的 GPIO 接口名称。

### 主要修改

1. **创建修复版的 ros2_control 配置文件**：为 2F-85 和 2F-140 夹爪创建修复版配置文件，使用 `${prefix}reactivate_gripper` 作为 GPIO 名称。

2. **修改硬件接口**：`RobotiqGripperHardwareInterface` 从 URDF 中读取 GPIO 名称，而不是硬编码。

3. **修改激活控制器**：`RobotiqActivationController` 支持从参数读取 GPIO 名称，如果未配置则从控制器名称推断。

4. **更新控制器配置**：在 `ros2_controllers.yaml` 中为每个激活控制器显式配置 `gpio_name` 参数。

### 修改的文件

- **新建文件**：
  - `dual_arm_config/description/urdf/gripper_ros2_control_2f_85_fixed.xacro`
  - `dual_arm_config/description/urdf/gripper_ros2_control_2f_140_fixed.xacro`

- **修改的文件**：
  - `dual_arm_config/description/urdf/left_gripper/left_gripper.urdf.xacro`
  - `dual_arm_config/description/urdf/right_gripper/right_gripper.urdf.xacro`
  - `robotiq_driver/src/hardware_interface.cpp`
  - `robotiq_controllers/include/robotiq_controllers/robotiq_activation_controller.hpp`
  - `robotiq_controllers/src/robotiq_activation_controller.cpp`
  - `dual_arm_moveit_config/config/ros2_controllers.yaml`

## 验证方法

1. 启动系统后，检查控制器状态，应该看到两个激活控制器都处于 `active` 状态。
2. 检查 GPIO 接口，应该看到两个不同的 GPIO 接口（`left_gripper_reactivate_gripper` 和 `right_gripper_reactivate_gripper`）。

## 注意事项

1. 此修复通过创建修复版的 ros2_control 配置文件来覆盖原始包中的硬编码定义，不需要修改原始的 `robotiq_description` 包。

2. 如果添加了新的夹爪，需要在 URDF 中使用唯一的 `prefix`，并在控制器配置文件中为新的激活控制器配置 `gpio_name` 参数。

3. GPIO 名称的优先级：首先使用配置文件中显式设置的 `gpio_name` 参数，如果未设置则从控制器名称推断，最后使用默认值 `reactivate_gripper`。

