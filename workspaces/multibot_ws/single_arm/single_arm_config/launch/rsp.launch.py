#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


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

    # tf_prefix argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="tf_prefix of the joint names, useful for "
            "multi-robot setup. If changed, also joint names in the controllers' configuration "
            "have to be updated. If empty, will be set based on arm_name.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_slave_address",
            default_value="0x09",
            description="Gripper slave address (Modbus RTU slave ID) in hexadecimal format (e.g., 0x09).",
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
    
    # UR robot driver file arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "script_filename",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_client_library"),
                    "resources",
                    "external_control.urscript",
                ]
            ),
            description="Script filename for UR robot driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "input_recipe_filename",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "resources",
                    "rtde_input_recipe.txt",
                ]
            ),
            description="Input recipe filename for UR robot driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "output_recipe_filename",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("ur_robot_driver"),
                    "resources",
                    "rtde_output_recipe.txt",
                ]
            ),
            description="Output recipe filename for UR robot driver.",
        )
    )

    # Initialize Arguments
    arm_name = LaunchConfiguration("arm_name")
    tf_prefix = LaunchConfiguration("tf_prefix")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    gripper_com_port = LaunchConfiguration("gripper_com_port")
    gripper_slave_address = LaunchConfiguration("gripper_slave_address")
    description_file = LaunchConfiguration("description_file")
    script_filename = LaunchConfiguration("script_filename")
    input_recipe_filename = LaunchConfiguration("input_recipe_filename")
    output_recipe_filename = LaunchConfiguration("output_recipe_filename")

    # Generate robot description using xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "arm_name:=",
            arm_name,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "reverse_ip:=",
            reverse_ip,
            " ",
            "use_tool_communication:=",
            use_tool_communication,
            " ",
            "gripper_com_port:=",
            gripper_com_port,
            " ",
            "gripper_slave_address:=",
            gripper_slave_address,
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
        ]
    )
    
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=["--ros-args", "--log-level", "info"],
    )

    nodes_to_start = [
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

