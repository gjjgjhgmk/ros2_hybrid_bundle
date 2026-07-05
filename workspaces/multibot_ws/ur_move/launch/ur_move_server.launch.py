#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent, IncludeLaunchDescription, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'ur_move_port',
            default_value='5605',
            description='UR Move服务器端口'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz',
            default_value='True',
            description='Whether to start RVIZ'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='true',
            description='Start robot with mock hardware mirroring command to its states'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_gripper_hardware',
            default_value='true',
            description='Use fake gripper hardware'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper_left_port',
            default_value='5630',
            description='左手夹爪 ZMQ 服务器端口'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gripper_right_port',
            default_value='5640',
            description='右手夹爪 ZMQ 服务器端口'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tf_server_port',
            default_value='5609',
            description='TF ZMQ 服务器端口'
        )
    )
    # 控制器参数
    declared_arguments.append(
        DeclareLaunchArgument(
            'initial_joint_controller',
            default_value='both',
            description='初始加载的关节控制器'
        )
    )

    ur_move_port = LaunchConfiguration('ur_move_port')
    rviz = LaunchConfiguration('rviz')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_fake_gripper_hardware = LaunchConfiguration('use_fake_gripper_hardware')
    gripper_left_port = LaunchConfiguration('gripper_left_port')
    gripper_right_port = LaunchConfiguration('gripper_right_port')
    tf_server_port = LaunchConfiguration('tf_server_port')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')

    # MoveIt 配置 - 使用 dual_arm_moveit_config
    moveit_config = MoveItConfigsBuilder("双臂机器人", package_name="dual_arm_moveit_config").to_moveit_configs()

    # ros2_control 启动文件（包含 robot_state_publisher 和 joint_state_broadcaster）
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dual_arm_config'),
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_mock_hardware': use_mock_hardware,
            'use_fake_gripper_hardware': use_fake_gripper_hardware,
            'launch_rviz': 'false',  # 不启用 control.launch.py 中的 RViz
            'initial_joint_controller': initial_joint_controller,
        }.items()
    )

    # move_group
    arm_move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dual_arm_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ])
    )

    # 轨迹规划服务器节点
    trajectory_planner_server_node = Node(
        package='ur_move',
        executable='trajectory_planner_server',
        name='trajectory_planner_server',
        output='screen',
        parameters=[
            {
                'bind_port': ur_move_port,
            },
            moveit_config.to_dict()  # 加载所有 MoveIt 配置
        ]
    )

    # 夹爪 ZMQ 服务器
    ur_move_package_share = FindPackageShare('ur_move')
    installed_gripper_script = PathJoinSubstitution([
        ur_move_package_share,
        '..', '..', 'lib', 'ur_move', 'gripper_zmq_server.py'
    ])
    
    gripper_zmq_server = ExecuteProcess(
        cmd=['python3', installed_gripper_script, '--left-port', gripper_left_port, '--right-port', gripper_right_port],
        output='screen',
        name='gripper_zmq_server',
        shell=False,
        emulate_tty=True,
        condition=UnlessCondition(use_fake_gripper_hardware),
    )
    
    # 添加启动日志，帮助诊断问题
    gripper_server_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=gripper_zmq_server,
            on_start=[
                LogInfo(msg="夹爪 ZMQ 服务器正在启动...")
            ]
        )
    )
    
    # 添加退出处理器，捕获夹爪服务器启动失败
    handler_gripper_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gripper_zmq_server,
            on_exit=[
                LogInfo(msg="警告: 夹爪 ZMQ 服务器已退出，请检查日志以查看错误信息"),
                LogInfo(msg="可能的原因: 1) 脚本路径错误 2) 导入错误 3) 端口被占用")
            ]
        )
    )
    
    # TF ZMQ 服务器
    installed_tf_script = PathJoinSubstitution([
        ur_move_package_share,
        '..', '..', 'lib', 'ur_move', 'tf_zmq_server.py'
    ])
    
    tf_zmq_server = ExecuteProcess(
        cmd=['python3', installed_tf_script, '--port', tf_server_port],
        output='screen',
        name='tf_zmq_server',
        shell=False,
        emulate_tty=True
    )
    
    # 添加启动日志，帮助诊断问题
    tf_server_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=tf_zmq_server,
            on_start=[
                LogInfo(msg="TF ZMQ 服务器正在启动...")
            ]
        )
    )
    
    # 添加退出处理器，捕获 TF 服务器启动失败
    handler_tf_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=tf_zmq_server,
            on_exit=[
                LogInfo(msg="警告: TF ZMQ 服务器已退出，请检查日志以查看错误信息"),
                LogInfo(msg="可能的原因: 1) 脚本路径错误 2) 导入错误 3) 端口被占用")
            ]
        )
    )

    # rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('dual_arm_moveit_config'), 'config', 'rviz', 'view_robot.rviz']
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    # 创建关键节点失败时的退出处理器
    handler_trajectory_planner_server_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_planner_server_node,
            on_exit=[
                LogInfo(msg="轨迹规划服务器节点已退出，正在关闭整个系统..."),
                EmitEvent(event=Shutdown(reason='轨迹规划服务器节点退出'))
            ]
        )
    )

    nodes = [
        control_launch,
        arm_move_group_launch,
        trajectory_planner_server_node,
        gripper_zmq_server,
        tf_zmq_server, 
        rviz_node,
        handler_trajectory_planner_server_exit,
        gripper_server_start_handler,
        handler_gripper_server_exit,
        tf_server_start_handler,
        handler_tf_server_exit,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
