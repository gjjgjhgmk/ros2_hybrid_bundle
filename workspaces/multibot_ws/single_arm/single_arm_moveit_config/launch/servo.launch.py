import os
import yaml

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def declare_arguments():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Using or not time from simulation",
            ),
        ]
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name="single_arm", package_name="single_arm_moveit_config")
        .robot_description_semantic(Path("srdf") / "single_arm.srdf.xacro", {"name": "single_arm"})
        .to_moveit_configs()
    )

    ld = LaunchDescription()
    ld.add_entity(declare_arguments())

    # Wait for robot_description to be available
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )
    ld.add_action(wait_robot_description)

    # Load servo configuration
    servo_yaml = load_yaml("single_arm_moveit_config", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_yaml} if servo_yaml else {}

    # Servo node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            servo_params,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # Start servo_node after robot_description is available
    ld.add_action(
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[servo_node],
            )
        ),
    )

    return ld

