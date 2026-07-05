#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂控制测试脚本

使用方法:
    # 移动到预设位置
    python3 test_arm_control.py --arm left --position "0.0 -1.57 0.0 -1.57 -1.57 0.0"
    
    # 交互模式
    python3 test_arm_control.py --arm left --interactive
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import argparse
import sys
import time


class ArmControlTest(Node):
    def __init__(self, arm_name='left'):
        super().__init__('arm_control_test')
        self.arm_name = arm_name
        self.action_name = f'{arm_name}_arm_controller/follow_joint_trajectory'
        
        # 定义关节名称（UR 机器人 6 自由度）
        self.joint_names = [
            f'{arm_name}_shoulder_pan_joint',
            f'{arm_name}_shoulder_lift_joint',
            f'{arm_name}_elbow_joint',
            f'{arm_name}_wrist_1_joint',
            f'{arm_name}_wrist_2_joint',
            f'{arm_name}_wrist_3_joint'
        ]
        
        self.get_logger().info(f'连接到机械臂控制器: {self.action_name}')
        self._action_client = ActionClient(self, FollowJointTrajectory, self.action_name)
        
        # 等待 action server
        self.get_logger().info(f'等待 action server: {self.action_name}')
        max_wait_time = 10.0
        start_time = time.time()
        while (time.time() - start_time) < max_wait_time:
            if self._action_client.wait_for_server(timeout_sec=1.0):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error(f'无法连接到 action server: {self.action_name}')
            self.get_logger().error('请确保:')
            self.get_logger().error('1. ros2_control_node 正在运行')
            self.get_logger().error('2. 机械臂控制器已启动')
            self.get_logger().error('3. 使用正确的手臂名称 (left/right)')
            raise RuntimeError(f'无法连接到 {self.action_name}')
        
        self.get_logger().info(f'成功连接到 {self.action_name}')
        self._result_received = False
        self._goal_handle = None

    def send_joint_positions(self, positions, duration=5.0, wait_for_result=True):
        """
        发送关节位置命令
        
        Args:
            positions: 关节位置列表（弧度），长度为6
            duration: 运动时间（秒）
            wait_for_result: 是否等待结果
        """
        if len(positions) != 6:
            self.get_logger().error(f'位置列表长度应为6，当前为{len(positions)}')
            return
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        # 显示命令信息
        pos_str = ', '.join([f'{p:.3f}' for p in positions])
        self.get_logger().info(f'发送命令: 关节位置=[{pos_str}], 持续时间={duration:.1f}s')
        
        self._result_received = False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
        if wait_for_result:
            start_time = time.time()
            timeout = duration + 10.0
            while not self._result_received and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.05)
            
            if not self._result_received:
                self.get_logger().warn('等待结果超时')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            self._result_received = True
            return
        
        self.get_logger().info('目标已接受，执行中...')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if feedback.actual.positions:
            pos_str = ', '.join([f'{p:.3f}' for p in feedback.actual.positions])
            self.get_logger().info(f'反馈: 当前位置=[{pos_str}]')

    def _get_result_callback(self, future):
        result = future.result().result
        self._result_received = True
        
        error_code = result.error_code
        error_string = result.error_string
        
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✓ 机械臂已到达目标位置')
        else:
            self.get_logger().warn(f'⚠ 机械臂未到达目标位置: {error_string} (错误码: {error_code})')

    def interactive_mode(self):
        """交互模式"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'{self.arm_name.upper()} 机械臂交互控制模式')
        self.get_logger().info('=' * 60)
        self.get_logger().info('命令:')
        self.get_logger().info('  home     - 回到初始位置 [0.0, -1.57, 0.0, -1.57, -1.57, 0.0]')
        self.get_logger().info('  zero     - 所有关节归零 [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')
        self.get_logger().info('  <6个值>  - 设置关节位置 (例如: 0.0 -1.57 1.57 -1.57 -1.57 0.0)')
        self.get_logger().info('  q/quit   - 退出')
        self.get_logger().info('=' * 60)
        self.get_logger().info('关节顺序:')
        for i, name in enumerate(self.joint_names, 1):
            self.get_logger().info(f'  {i}. {name}')
        self.get_logger().info('=' * 60)
        
        while rclpy.ok():
            try:
                user_input = input(f'\n[{self.arm_name}手臂] > ').strip().lower()
                
                if user_input in ['q', 'quit', 'exit']:
                    self.get_logger().info('退出交互模式')
                    break
                elif user_input == 'home':
                    self.send_joint_positions([0.0, -1.57, 0.0, -1.57, -1.57, 0.0], wait_for_result=False)
                elif user_input == 'zero':
                    self.send_joint_positions([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait_for_result=False)
                elif user_input == '':
                    continue
                else:
                    # 尝试解析为6个浮点数
                    try:
                        values = [float(x) for x in user_input.split()]
                        if len(values) == 6:
                            self.send_joint_positions(values, wait_for_result=False)
                        else:
                            self.get_logger().warn(f'需要6个值，当前输入{len(values)}个值')
                    except ValueError:
                        self.get_logger().warn(f'未知命令或无效输入: {user_input}')
                
                # 处理 action 回调
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                self.get_logger().info('\n收到中断信号，退出...')
                break
            except Exception as e:
                self.get_logger().error(f'错误: {e}')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='机械臂控制测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 移动到初始位置
  %(prog)s --arm left --position "0.0 -1.57 0.0 -1.57 -1.57 0.0"
  
  # 所有关节归零
  %(prog)s --arm right --position "0.0 0.0 0.0 0.0 0.0 0.0"
  
  # 交互模式
  %(prog)s --arm left --interactive

关节顺序:
  1. shoulder_pan_joint
  2. shoulder_lift_joint
  3. elbow_joint
  4. wrist_1_joint
  5. wrist_2_joint
  6. wrist_3_joint
        """
    )
    parser.add_argument(
        '--arm',
        choices=['left', 'right'],
        default='left',
        help='要控制的手臂 (默认: left)'
    )
    parser.add_argument(
        '--position',
        type=str,
        help='关节位置 (6个值，空格分隔，单位: 弧度)'
    )
    parser.add_argument(
        '--duration',
        type=float,
        default=5.0,
        help='运动持续时间 (秒) (默认: 5.0)'
    )
    parser.add_argument(
        '--interactive',
        action='store_true',
        help='启动交互模式'
    )
    
    args = parser.parse_args()
    
    if not args.interactive and args.position is None:
        parser.error('必须指定 --position 或使用 --interactive 模式')
    
    rclpy.init(args=None)
    
    try:
        node = ArmControlTest(arm_name=args.arm)
        
        if args.interactive:
            node.interactive_mode()
        else:
            # 解析位置字符串
            try:
                positions = [float(x) for x in args.position.split()]
                if len(positions) != 6:
                    parser.error(f'位置值应为6个，当前为{len(positions)}个')
                node.send_joint_positions(positions, duration=args.duration, wait_for_result=True)
            except ValueError:
                parser.error(f'无效的位置值: {args.position}')
        
    except RuntimeError as e:
        print(f'错误: {e}', file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print('\n收到中断信号')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

