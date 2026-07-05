#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    
    # Arm name argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_name",
            default_value="left_arm",
            description="Name of the arm: left_arm or right_arm",
            choices=["left_arm", "right_arm"],
        )
    )
    
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
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated. If empty, will be set based on arm_name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address of the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="192.168.56.1",
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
    
    # URDF file argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("single_arm_config"), "description", "single_arm.urdf.xacro"]
            ),
            description="URDF/XACRO description file with the robot.",
        )
    )
    
    # RViz arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
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

    # Initialize Arguments
    arm_name = LaunchConfiguration("arm_name")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    gripper_com_port = LaunchConfiguration("gripper_com_port")
    tf_prefix = LaunchConfiguration("tf_prefix")
    description_file = LaunchConfiguration("description_file")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # Include RSP launch file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("single_arm_config"), "launch", "rsp.launch.py"]
            )
        ),
        launch_arguments={
            "arm_name": arm_name,
            "mock_sensor_commands": mock_sensor_commands,
            "use_mock_hardware": use_mock_hardware,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "use_tool_communication": use_tool_communication,
            "gripper_com_port": gripper_com_port,
            "tf_prefix": tf_prefix,
            "description_file": description_file,
        }.items(),
    )

    # Joint State Publisher GUI node (for manual joint control)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    
    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=conditions.IfCondition(use_rviz),
    )

    nodes_to_start = [
        rsp_launch,
        joint_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

