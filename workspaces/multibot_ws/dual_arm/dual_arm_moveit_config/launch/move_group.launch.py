from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_left_drawing_tool = LaunchConfiguration("use_left_drawing_tool")
    moveit_config = (
        MoveItConfigsBuilder("双臂机器人", package_name="dual_arm_moveit_config")
        .robot_description(mappings={"use_left_drawing_tool": use_left_drawing_tool})
        .to_moveit_configs()
    )
    launch_description = generate_move_group_launch(moveit_config)
    launch_description.add_action(
        DeclareLaunchArgument(
            "use_left_drawing_tool",
            default_value="false",
            description="Attach the fixed drawing pen tool behind left_ee_link.",
        )
    )
    return launch_description
