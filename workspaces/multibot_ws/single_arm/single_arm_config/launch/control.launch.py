#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile


def launch_setup(context):
    # Initialize Arguments
    arm_name = LaunchConfiguration("arm_name")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    reverse_ip = LaunchConfiguration("reverse_ip")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    gripper_com_port = LaunchConfiguration("gripper_com_port")
    gripper_slave_address = LaunchConfiguration("gripper_slave_address")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    # UR robot driver file paths
    script_filename = LaunchConfiguration("script_filename")
    input_recipe_filename = LaunchConfiguration("input_recipe_filename")
    output_recipe_filename = LaunchConfiguration("output_recipe_filename")

    # Include RSP launch file
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("single_arm_config"), "launch", "rsp.launch.py"]
            )
        ),
        launch_arguments={
            "arm_name": arm_name,
            "tf_prefix": tf_prefix,
            "mock_sensor_commands": mock_sensor_commands,
            "use_mock_hardware": use_mock_hardware,
            "robot_ip": robot_ip,
            "reverse_ip": reverse_ip,
            "use_tool_communication": use_tool_communication,
            "gripper_com_port": gripper_com_port,
            "gripper_slave_address": gripper_slave_address,
            "description_file": description_file,
            "script_filename": script_filename,
            "input_recipe_filename": input_recipe_filename,
            "output_recipe_filename": output_recipe_filename,
        }.items(),
    )

    # ros2_control_node
    # Note: tf_prefix is used as substitution in controllers_file, so we pass it as a parameter
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(controllers_file, allow_substs=True),
            {"tf_prefix": tf_prefix},
        ],
        output="screen",
    )

    # Controller spawne. helper function
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

    # Build list of controllers to activate (following ur_control.launch.py pattern)
    controllers_active = [
        "joint_state_broadcaster",
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "ur_configuration_controller",
        "robotiq_activation_controller",
        "robotiq_gripper_controller",
    ]
    
    # Build list of controllers to load but not activate
    controllers_inactive = [
        "scaled_joint_trajectory_controller",
        "joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        "forward_effort_controller",
        "force_mode_controller",
        "passthrough_trajectory_controller",
        "freedrive_mode_controller",
        "tool_contact_controller",
    ]
    
    # Add tcp_pose_broadcaster only for real hardware
    use_mock_hardware_val = use_mock_hardware.perform(context)
    if use_mock_hardware_val.lower() != "true":
        controllers_active.append("tcp_pose_broadcaster")
    
    # Activate joint controller if requested
    if activate_joint_controller.perform(context) == "true":
        initial_controller = initial_joint_controller.perform(context)
        if initial_controller in controllers_inactive:
            controllers_active.append(initial_controller)
            controllers_inactive.remove(initial_controller)

    # Create controller spawners
    controller_spawners = [
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
    ]

    # Tool communication node
    tool_communication_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(use_tool_communication),
        executable="tool_communication.py",
        name="ur_tool_comm",
        output="screen",
        parameters=[
            {
                "robot_ip": robot_ip,
                "tcp_port": tool_tcp_port,
                "device_name": tool_device_name,
            }
        ],
    )
    
    # Event handler: start control_node after tool_communication_node starts
    # This ensures tool_communication_node starts first when use_tool_communication is true
    control_node_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=tool_communication_node,
            on_start=[control_node],
        )
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    # ZMQ nodes - 根据 arm_name 自动选择端口
    # 左臂: 5650 (joint_states), 5660 (trajectory_executor), 5630 (gripper)
    # 右臂: 5651 (joint_states), 5661 (trajectory_executor), 5640 (gripper)
    arm_name_val = arm_name.perform(context)
    if arm_name_val == "left_arm":
        joint_states_zmq_port = "5650"
        trajectory_executor_zmq_port = "5660"
        gripper_zmq_port = "5630"
    else:
        joint_states_zmq_port = "5651"
        trajectory_executor_zmq_port = "5661"
        gripper_zmq_port = "5640"

    # 获取脚本安装路径
    single_arm_config_package_share = FindPackageShare('single_arm_config')
    joint_states_script_path = PathJoinSubstitution([
        single_arm_config_package_share,
        '..', '..', 'lib', 'single_arm_config', 'joint_states_zmq_publisher.py'
    ])
    trajectory_executor_script_path = PathJoinSubstitution([
        single_arm_config_package_share,
        '..', '..', 'lib', 'single_arm_config', 'trajectory_executor_server.py'
    ])
    gripper_control_script_path = PathJoinSubstitution([
        single_arm_config_package_share,
        '..', '..', 'lib', 'single_arm_config', 'gripper_control_server.py'
    ])
    
    # 获取实际路径
    joint_states_script = joint_states_script_path.perform(context)
    trajectory_executor_script = trajectory_executor_script_path.perform(context)
    gripper_control_script = gripper_control_script_path.perform(context)

    # Joint States ZMQ Publisher
    joint_states_zmq_publisher = ExecuteProcess(
        cmd=['python3', joint_states_script, '--zmq-port', joint_states_zmq_port, '--arm-name', arm_name_val],
        output='screen',
        name='joint_states_zmq_publisher',
        shell=False,
        emulate_tty=True
    )

    # Trajectory Executor Server
    trajectory_executor_server = ExecuteProcess(
        cmd=['python3', trajectory_executor_script, '--zmq-port', trajectory_executor_zmq_port, '--arm-name', arm_name_val],
        output='screen',
        name='trajectory_executor_server',
        shell=False,
        emulate_tty=True
    )

    # Gripper Control Server (根据arm_name选择端口：左臂5630，右臂5640)
    gripper_control_server = ExecuteProcess(
        cmd=['python3', gripper_control_script, '--zmq-port', gripper_zmq_port],
        output='screen',
        name='gripper_control_server',
        shell=False,
        emulate_tty=True
    )

    # Build nodes list
    # Note: control_node will be started by event handler when tool_communication_node starts
    # If tool_communication is disabled, we need to add control_node directly
    use_tool_communication_val = use_tool_communication.perform(context)
    nodes_to_start = [
        rsp_launch,
        tool_communication_node,  # Start tool_communication_node first
        joint_states_zmq_publisher,
        trajectory_executor_server,
        gripper_control_server,
        rviz_node,
    ]
    
    # Add control_node directly if tool_communication is disabled
    # Otherwise, it will be started by the event handler
    if use_tool_communication_val.lower() != "true":
        nodes_to_start.insert(1, control_node)  # Insert after rsp_launch
    else:
        # Add the event handler to start control_node after tool_communication_node
        nodes_to_start.append(control_node_start_handler)
    
    nodes_to_start.extend(controller_spawners)

    return nodes_to_start


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
    
    # tf_prefix argument (used in controllers configuration)
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
            default_value="false",
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
            default_value="192.168.56.14",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor that will be generated for the tool communication device. "
            "The user has to be allowed to write to this location. "
            "Only effective, if use_tool_communication is set to True.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port that will be used for bridging the tool's serial device. "
            "Only effective, if use_tool_communication is set to True.",
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
    
    # Controllers file argument
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("single_arm_config"), "config", "ros2_controllers.yaml"]
            ),
            description="YAML file with the controllers configuration.",
        )
    )
    
    # Controller spawner timeout
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout for controller spawner.",
        )
    )
    
    # Initial joint controller
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
    
    # RViz arguments
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

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
