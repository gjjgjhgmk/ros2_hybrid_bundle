#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR Move 规划服务器启动文件
用于规划PC（第三台电脑）：
- 启动轨迹规划服务器
- 启动MoveIt move_group
- 启动关节状态ZMQ桥接（接收左右臂驱动PC的joint_states）
- 启动TF ZMQ服务器

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, EmitEvent, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

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
            'tf_server_port',
            default_value='5609',
            description='TF ZMQ 服务器端口'
        )
    )
    # 关节状态桥接参数
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_arm_host',
            default_value='192.168.56.222',
            description='左臂驱动PC地址（可选，如: 192.168.1.101）'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_arm_host',
            default_value='192.168.56.122',
            description='右臂驱动PC地址（可选，如: 192.168.1.102）'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'left_arm_zmq_port',
            default_value='5650',
            description='左臂 ZMQ 端口'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'right_arm_zmq_port',
            default_value='5651',
            description='右臂 ZMQ 端口'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'debug_joint_states',
            default_value='false',
            description='启用关节状态调试模式（打印合并后的关节状态）'
        )
    )

    ur_move_port = LaunchConfiguration('ur_move_port')
    rviz = LaunchConfiguration('rviz')
    tf_server_port = LaunchConfiguration('tf_server_port')
    left_arm_host = LaunchConfiguration('left_arm_host')
    right_arm_host = LaunchConfiguration('right_arm_host')
    left_arm_zmq_port = LaunchConfiguration('left_arm_zmq_port')
    right_arm_zmq_port = LaunchConfiguration('right_arm_zmq_port')
    debug_joint_states = LaunchConfiguration('debug_joint_states')

    # MoveIt 配置 - 使用 dual_arm_moveit_config
    moveit_config = MoveItConfigsBuilder("双臂机器人", package_name="dual_arm_moveit_config").to_moveit_configs()

    # Robot State Publisher（仅用于发布TF，不启动驱动）
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 50.0},
        ],
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

    # 关节状态ZMQ桥接服务器
    ur_move_package_share = FindPackageShare('ur_move')
    installed_joint_states_relay_script = PathJoinSubstitution([
        ur_move_package_share,
        '..', '..', 'lib', 'ur_move', 'joint_states_zmq_relay.py'
    ])
    
    # 使用 OpaqueFunction 返回 ExecuteProcess action（因为需要处理可选参数）
    def create_joint_states_relay_process(context):
        cmd = ['python3']
        script_path = installed_joint_states_relay_script.perform(context)
        cmd.append(script_path)
        
        left_host_val = left_arm_host.perform(context)
        right_host_val = right_arm_host.perform(context)
        
        # 只有当主机地址不为空时才添加到命令中
        # 注意：空字符串或只包含空白字符的字符串视为未设置
        if left_host_val and left_host_val.strip():
            cmd.extend(['--left-arm-host', left_host_val])
        if right_host_val and right_host_val.strip():
            cmd.extend(['--right-arm-host', right_host_val])
        
        cmd.extend(['--left-arm-port', left_arm_zmq_port.perform(context)])
        cmd.extend(['--right-arm-port', right_arm_zmq_port.perform(context)])
        
        if debug_joint_states.perform(context).lower() == 'true':
            cmd.append('--debug')
        
        process = ExecuteProcess(
            cmd=cmd,
            output='screen',
            name='joint_states_zmq_relay',
            shell=False,
            emulate_tty=True
        )
        
        # 创建事件处理器（需要在同一个函数中创建，因为它们需要引用 process）
        start_handler = RegisterEventHandler(
            OnProcessStart(
                target_action=process,
                on_start=[
                    LogInfo(msg="关节状态ZMQ桥接服务器正在启动...")
                ]
            )
        )
        
        exit_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=process,
                on_exit=[
                    LogInfo(msg="警告: 关节状态ZMQ桥接服务器已退出，请检查日志以查看错误信息"),
                    LogInfo(msg="可能的原因: 1) 脚本路径错误 2) 导入错误 3) 端口被占用 4) 驱动PC未启动")
                ]
            )
        )
        
        return [process, start_handler, exit_handler]
    
    joint_states_zmq_relay_action = OpaqueFunction(function=create_joint_states_relay_process)

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
    
    # 添加启动日志
    tf_server_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=tf_zmq_server,
            on_start=[
                LogInfo(msg="TF ZMQ 服务器正在启动...")
            ]
        )
    )
    
    # 添加退出处理器
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
        robot_state_publisher_node,
        joint_states_zmq_relay_action,  # OpaqueFunction 返回 [ExecuteProcess, 事件处理器]
        arm_move_group_launch, # 需要robot_state_publisher和joint_states
        trajectory_planner_server_node,
        tf_zmq_server,
        rviz_node,
        handler_trajectory_planner_server_exit,
        tf_server_start_handler,
        handler_tf_server_exit,
    ]
    
    return LaunchDescription(declared_arguments + nodes)

