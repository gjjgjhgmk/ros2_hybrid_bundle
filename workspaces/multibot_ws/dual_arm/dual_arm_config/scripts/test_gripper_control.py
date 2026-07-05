#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robotiq 夹爪控制测试脚本

使用方法:
    # 打开左夹爪
    python3 test_gripper_control.py --gripper left --position 0.0
    
    # 关闭右夹爪
    python3 test_gripper_control.py --gripper right --position 0.8
    
    # 半开左夹爪
    python3 test_gripper_control.py --gripper left --position 0.4
    
    # 交互模式
    python3 test_gripper_control.py --gripper left --interactive
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand
from sensor_msgs.msg import JointState
import argparse
import sys


class GripperControlTest(Node):
    def __init__(self, gripper_name='left'):
        super().__init__('gripper_control_test')
        self.gripper_name = gripper_name
        self.action_name = f'{gripper_name}_gripper_controller/gripper_cmd'
        
        self.get_logger().info(f'连接到夹爪控制器: {self.action_name}')
        self._action_client = ActionClient(self, ParallelGripperCommand, self.action_name)
        
        # 等待 action server
        self.get_logger().info(f'等待 action server: {self.action_name}')
        # 增加等待时间并定期 spin，确保能够发现 action server
        import time
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
            self.get_logger().error('2. 夹爪控制器已启动')
            self.get_logger().error('3. 使用正确的夹爪名称 (left/right)')
            raise RuntimeError(f'无法连接到 {self.action_name}')
        
        self.get_logger().info(f'成功连接到 {self.action_name}')

    def send_gripper_command(self, position, max_effort=50.0, wait_for_result=True):
        """
        发送夹爪控制命令
        
        Args:
            position: 夹爪位置 (0.0 = 完全打开, 0.8 = 完全关闭)
            max_effort: 最大力度 (N)
            wait_for_result: 是否等待结果（用于非交互模式）
        """
        goal_msg = ParallelGripperCommand.Goal()
        # ParallelGripperCommand 使用 JointState 作为命令
        goal_msg.command = JointState()
        goal_msg.command.name = [f'{self.gripper_name}_gripper_robotiq_85_left_knuckle_joint']
        goal_msg.command.position = [float(position)]
        goal_msg.command.effort = [float(max_effort)]  # max_effort 作为 effort
        
        self.get_logger().info(f'发送命令: position={position:.3f}, max_effort={max_effort:.1f}N')
        
        self._result_received = False
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
        if wait_for_result:
            # 等待结果
            import time
            start_time = time.time()
            timeout = 10.0
            while not self._result_received and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.05)
            
            if not self._result_received:
                self.get_logger().warn('等待结果超时')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return
        
        self.get_logger().info('目标已接受，执行中...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # ParallelGripperCommand 使用 JointState 作为 state
        position = feedback.state.position[0] if len(feedback.state.position) > 0 else 0.0
        effort = feedback.state.effort[0] if len(feedback.state.effort) > 0 else 0.0
        self.get_logger().info(
            f'反馈: position={position:.3f}, '
            f'effort={effort:.2f}N, '
            f'stalled={feedback.stalled}, '
            f'reached_goal={feedback.reached_goal}'
        )

    def _get_result_callback(self, future):
        result = future.result().result
        self._result_received = True
        # ParallelGripperCommand 使用 JointState 作为 state
        position = result.state.position[0] if len(result.state.position) > 0 else 0.0
        effort = result.state.effort[0] if len(result.state.effort) > 0 else 0.0
        self.get_logger().info(
            f'完成: position={position:.3f}, '
            f'effort={effort:.2f}N, '
            f'stalled={result.stalled}, '
            f'reached_goal={result.reached_goal}'
        )
        if result.reached_goal:
            self.get_logger().info('✓ 夹爪已到达目标位置')
        else:
            self.get_logger().warn('⚠ 夹爪未到达目标位置')

    def interactive_mode(self):
        """交互模式"""
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'{self.gripper_name.upper()} 夹爪交互控制模式')
        self.get_logger().info('=' * 50)
        self.get_logger().info('命令:')
        self.get_logger().info('  open     - 完全打开 (position=0.0)')
        self.get_logger().info('  close    - 完全关闭 (position=0.8)')
        self.get_logger().info('  half     - 半开 (position=0.4)')
        self.get_logger().info('  <0.0-0.8> - 设置位置 (例如: 0.5)')
        self.get_logger().info('  q/quit   - 退出')
        self.get_logger().info('=' * 50)
        
        while rclpy.ok():
            try:
                user_input = input(f'\n[{self.gripper_name}夹爪] > ').strip().lower()
                
                if user_input in ['q', 'quit', 'exit']:
                    self.get_logger().info('退出交互模式')
                    break
                elif user_input == 'open':
                    self.send_gripper_command(0.0, wait_for_result=False)
                elif user_input == 'close':
                    self.send_gripper_command(0.8, wait_for_result=False)
                elif user_input == 'half':
                    self.send_gripper_command(0.4, wait_for_result=False)
                elif user_input.replace('.', '').isdigit():
                    position = float(user_input)
                    if 0.0 <= position <= 0.8:
                        self.send_gripper_command(position, wait_for_result=False)
                    else:
                        self.get_logger().warn(f'位置值应在 0.0-0.8 之间，当前值: {position}')
                elif user_input == '':
                    continue
                else:
                    self.get_logger().warn(f'未知命令: {user_input}')
                
                # 处理 action 回调
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                self.get_logger().info('\n收到中断信号，退出...')
                break
            except ValueError:
                self.get_logger().warn('无效的数值输入')
            except Exception as e:
                self.get_logger().error(f'错误: {e}')


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Robotiq 夹爪控制测试脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 打开左夹爪
  %(prog)s --gripper left --position 0.0
  
  # 关闭右夹爪
  %(prog)s --gripper right --position 0.8
  
  # 交互模式
  %(prog)s --gripper left --interactive
        """
    )
    parser.add_argument(
        '--gripper',
        choices=['left', 'right'],
        default='left',
        help='要控制的夹爪 (默认: left)'
    )
    parser.add_argument(
        '--position',
        type=float,
        help='夹爪位置 (0.0=完全打开, 0.8=完全关闭)'
    )
    parser.add_argument(
        '--max-effort',
        type=float,
        default=50.0,
        help='最大力度 (N) (默认: 50.0)'
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
        node = GripperControlTest(gripper_name=args.gripper)
        
        if args.interactive:
            node.send_gripper_command(0.0, wait_for_result=False)  # 初始化，不等待
            node.interactive_mode()
        else:
            node.send_gripper_command(args.position, args.max_effort, wait_for_result=True)
        
    except RuntimeError as e:
        print(f'错误: {e}', file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print('\n收到中断信号')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

