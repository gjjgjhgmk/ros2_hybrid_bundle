#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """生成右臂启动描述"""
    declared_arguments = []
    
    # ros2_control arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock sensor commands for ros2_control.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.102",
            description="IP address of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="192.168.56.122",
            description="IP address of the reverse connection.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="true",
            description="Enable tool communication.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_com_port",
            default_value="/tmp/ttyUR",
            description="Gripper communication port.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_slave_address",
            default_value="0x02",
            description="Gripper slave address (Modbus RTU slave ID) in hexadecimal format (e.g., 0x03).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for multi-robot setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout for controller spawner.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            choices=[
                "scaled_joint_trajectory_controller",
                "joint_trajectory_controller",
                "forward_velocity_controller",
                "forward_position_controller",
                "freedrive_mode_controller",
                "passthrough_trajectory_controller",
            ],
            description="Initially loaded robot controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate joint controller on startup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Start RViz automatically with the launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("single_arm_config"), "config", "display.rviz"]
            ),
            description="RViz config file to use when launching rviz.",
        )
    )
    
    # Include control.launch.py with right_arm configuration
    right_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("single_arm_config"), "launch", "control.launch.py"]
            )
        ),
        launch_arguments={
            "arm_name": "right_arm",
            "mock_sensor_commands": LaunchConfiguration("mock_sensor_commands"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "reverse_ip": LaunchConfiguration("reverse_ip"),
            "use_tool_communication": LaunchConfiguration("use_tool_communication"),
            "gripper_com_port": LaunchConfiguration("gripper_com_port"),
            "gripper_slave_address": LaunchConfiguration("gripper_slave_address"),
            "tool_device_name": LaunchConfiguration("tool_device_name"),
            "tool_tcp_port": LaunchConfiguration("tool_tcp_port"),
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "controller_spawner_timeout": LaunchConfiguration("controller_spawner_timeout"),
            "initial_joint_controller": LaunchConfiguration("initial_joint_controller"),
            "activate_joint_controller": LaunchConfiguration("activate_joint_controller"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
            "rviz_config_file": LaunchConfiguration("rviz_config_file"),
        }.items(),
    )
    
    return LaunchDescription(
        declared_arguments + [right_arm_launch]
    )

