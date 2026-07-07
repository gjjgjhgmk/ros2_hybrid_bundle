#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def launch_setup(context):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    gripper_type = LaunchConfiguration("gripper_type")
    controllers_file = LaunchConfiguration("controllers_file")
    description_file = LaunchConfiguration("description_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")

    # Gripper hardware arguments
    use_fake_gripper_hardware = LaunchConfiguration("use_fake_gripper_hardware")

    # Generate robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            description_file,
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "gripper_type:=",
            gripper_type,
            " ",
            "use_fake_gripper_hardware:=",
            use_fake_gripper_hardware,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }


    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"publish_frequency": 50.0},
        ],
    )

    # ros2_control_node for the entire robot (handles both arms)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
    )

    # Controller spawner helper function
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    # Determine which controllers to activate
    controllers_active = ["joint_state_broadcaster"]
    controllers_inactive = [
        "left_arm_controller",
        "right_arm_controller",
    ]

    if activate_joint_controller.perform(context) == "true":
        initial_controller = initial_joint_controller.perform(context)
        if initial_controller == "left_arm_controller":
            controllers_active.append("left_arm_controller")
            controllers_inactive.remove("left_arm_controller")
        elif initial_controller == "right_arm_controller":
            controllers_active.append("right_arm_controller")
            controllers_inactive.remove("right_arm_controller")
        elif initial_controller == "both":
            controllers_active.extend(["left_arm_controller", "right_arm_controller"])
            controllers_inactive.clear()

    controller_spawners = [
        controller_spawner(controllers_active),
    ]

    if controllers_inactive:
        controller_spawners.append(controller_spawner(controllers_inactive, active=False))
    
    # Phase-one arm-only simulation does not need the Robotiq hardware plugin.
    # Keep the real-hardware behavior unchanged when fake grippers are disabled.
    if use_fake_gripper_hardware.perform(context).lower() != "true":
        controller_spawners.append(controller_spawner(["left_gripper_activation_controller"], active=True))
        controller_spawners.append(controller_spawner(["right_gripper_activation_controller"], active=True))
        controller_spawners.append(controller_spawner(["left_gripper_controller"], active=True))
        controller_spawners.append(controller_spawner(["right_gripper_controller"], active=True))

    # RViz
    rviz_node = Node(
        package="rviz2",
        condition=conditions.IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Start nodes (using mock hardware, no helper nodes needed)
    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
    ] + controller_spawners
    
    nodes_to_start.append(rviz_node)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            default_value="ur7e",
            choices=[
                "ur3",
                "ur5",
                "ur10",
                "ur3e",
                "ur5e",
                "ur7e",
                "ur10e",
                "ur12e",
                "ur16e",
                "ur8long",
                "ur15",
                "ur18",
                "ur20",
                "ur30",
            ],
        )
    )

    # Gripper arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_type",
            default_value="robotiq_2f_85",
            description="Type of gripper",
            choices=["robotiq_2f_85"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_gripper_hardware",
            default_value="true",
            description="Start gripper with fake hardware (for simulation/testing).",
        )
    )

    # ros2_control arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. "
            "Used only if 'use_mock_hardware' parameter is true.",
        )
    )

    # Configuration files
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("dual_arm_moveit_config"), "config", "ros2_controllers.yaml"]
            ),
            description="YAML file with the controllers configuration.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("dual_arm_moveit_config"), "config", "双臂机器人.urdf.xacro"]
            ),
            description="URDF/XACRO description file (absolute path) with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("dual_arm_config"), "config", "initial_positions.yaml"]
            ),
            description="YAML file with initial joint positions.",
        )
    )

    # Controller arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawning controllers.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="both",
            choices=[
                "left_arm_controller",
                "right_arm_controller",
                "both",
            ],
            description="Initially loaded robot controller(s).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller(s).",
        )
    )

    # Visualization
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("dual_arm_config"), "config", "view_robot.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
