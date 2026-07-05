from rclpy.action import ActionClient
from control_msgs.action import ParallelGripperCommand

action_client = ActionClient(node, ParallelGripperCommand,
                              'left_gripper_controller/gripper_cmd')
goal_msg = ParallelGripperCommand.Goal()
goal_msg.command.position = 0.0  # 0.0=打开, 0.8=关闭
goal_msg.command.max_effort = 50.0
action_client.send_goal_async(goal_msg)