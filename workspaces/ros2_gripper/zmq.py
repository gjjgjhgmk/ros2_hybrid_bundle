from ur_move.client.zmq_gripper_client import GripperZMQClient

# 初始化
left_gripper = GripperZMQClient(
    server_host="localhost",
    port=5630,
    gripper_name="left"
)

# 打开夹爪
left_gripper.open(max_effort=50.0)

# 关闭夹爪
left_gripper.close(max_effort=50.0)

# 设置位置
left_gripper.set_position(position=0.4, max_effort=50.0)