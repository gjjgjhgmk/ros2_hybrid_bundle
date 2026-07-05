#!/usr/bin/env python3
"""
UR Robot Servo GUI Control
PyQt5-based GUI control application
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                                 QHBoxLayout, QPushButton, QLabel, QGroupBox,
                                 QGridLayout, QMessageBox)
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt5.QtGui import QFont
except ImportError:
    print("Error: PyQt5 is required")
    print("Install with: pip install PyQt5")
    sys.exit(1)


# UR Robot Configuration
# These should match servo.yaml configuration
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOINT_TOPIC = "/servo_node/delta_joint_cmds"
PLANNING_FRAME_ID = "base_link"  # Must match servo.yaml: planning_frame
EE_FRAME_ID = "arm_interface_link"  # Must match servo.yaml: robot_link_command_frame and ee_frame

# UR Robot Joint Names (6 joints)
UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Joint Display Names (simplified)
JOINT_DISPLAY_NAMES = [
    "Shoulder Pan",
    "Shoulder Lift",
    "Elbow",
    "Wrist 1",
    "Wrist 2",
    "Wrist 3",
]


class ROS2Node(Node):
    """ROS2 Node running in a separate thread"""
    
    def __init__(self):
        super().__init__('servo_gui_node')
        
        # Create publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            TWIST_TOPIC,
            10
        )
        
        self.joint_pub = self.create_publisher(
            JointJog,
            JOINT_TOPIC,
            10
        )
        
        # Create service client
        self.switch_input_client = self.create_client(
            ServoCommandType,
            "servo_node/switch_command_type"
        )
        
        # Wait for service availability
        if not self.switch_input_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "Service 'servo_node/switch_command_type' not available."
            )
        
        # State variables
        # joint_vel_cmd is a multiplier for joint velocity commands
        # For speed_units mode, this should be 1.0 (commands are already in rad/s)
        self.joint_vel_cmd = 1.0
        self.command_frame_id = PLANNING_FRAME_ID
        self.current_mode = "TWIST"  # TWIST or JOINT
        
        # Current command state (for continuous publishing)
        self.current_twist_cmd = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.current_joint_cmd = {"index": -1, "velocity": 0.0}
        
        # Command publishing timer (publish at ~50Hz to keep servo active)
        self.cmd_timer = None
        
        self.get_logger().info("Servo GUI ROS2 node initialized")
    
    def publish_twist(self, linear_x=0.0, linear_y=0.0, linear_z=0.0):
        """Publish Twist command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # For Twist commands, use robot_link_command_frame as per servo.yaml
        # According to MoveIt Servo docs, Twist commands must be in robot_link_command_frame
        # In servo.yaml: robot_link_command_frame: tool0
        msg.header.frame_id = EE_FRAME_ID  # Use tool0 (robot_link_command_frame) for twist commands
        # Ensure all values are float type
        msg.twist.linear.x = float(linear_x)
        msg.twist.linear.y = float(linear_y)
        msg.twist.linear.z = float(linear_z)
        # Initialize angular components to 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)
    
    def set_twist_command(self, linear_x=0.0, linear_y=0.0, linear_z=0.0):
        """Set twist command (for continuous publishing)"""
        self.current_twist_cmd = {
            "x": float(linear_x),
            "y": float(linear_y),
            "z": float(linear_z)
        }
        # Immediately publish
        self.publish_twist(linear_x, linear_y, linear_z)
    
    def publish_joint(self, joint_index, velocity):
        """Publish joint command"""
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = PLANNING_FRAME_ID
        msg.joint_names = UR_JOINT_NAMES.copy()
        msg.velocities = [0.0] * 6
        if 0 <= joint_index < 6:
            # Ensure velocity is float type
            msg.velocities[joint_index] = float(velocity * self.joint_vel_cmd)
        self.joint_pub.publish(msg)
    
    def set_joint_command(self, joint_index, velocity):
        """Set joint command (for continuous publishing)"""
        self.current_joint_cmd = {
            "index": joint_index,
            "velocity": float(velocity)
        }
        # Immediately publish
        self.publish_joint(joint_index, velocity)
    
    def stop_all_commands(self):
        """Stop all commands"""
        self.current_twist_cmd = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.current_joint_cmd = {"index": -1, "velocity": 0.0}
        self.publish_twist(0.0, 0.0, 0.0)
        self.publish_joint(-1, 0.0)
    
    def switch_command_type(self, command_type):
        """Switch command type"""
        if not self.switch_input_client.service_is_ready():
            self.get_logger().warn("Service not ready for switching command type")
            return False
        
        request = ServoCommandType.Request()
        request.command_type = command_type
        
        # Wait for service with longer timeout
        if not self.switch_input_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Service wait timeout")
            return False
        
        future = self.switch_input_client.call_async(request)
        
        # Wait for response with multiple spin calls
        timeout_count = 0
        while not future.done() and timeout_count < 20:
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_count += 1
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.current_mode = "TWIST" if command_type == ServoCommandType.Request.TWIST else "JOINT"
                    self.get_logger().info(f"Command type switched to: {self.current_mode}")
                    return True
                else:
                    self.get_logger().warn(f"Service returned success=False")
                    return False
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return False
        else:
            self.get_logger().warn("Service call timeout")
            return False


class ServoGUI(QMainWindow):
    """Servo Control GUI Main Window"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize ROS2
        rclpy.init()
        self.ros_node = ROS2Node()
        
        # Create ROS2 executor thread
        self.ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self.ros_thread.start()
        
        # Initialize UI
        self.init_ui()
        
        # Create timer for periodic ROS2 callback processing
        self.timer = QTimer()
        self.timer.timeout.connect(self._process_ros)
        self.timer.start(10)  # 10ms interval
        
        # Create timer for continuous command publishing (publish at ~50Hz = 20ms)
        # This ensures servo_node receives commands frequently enough
        self.cmd_publish_timer = QTimer()
        self.cmd_publish_timer.timeout.connect(self._publish_current_commands)
        self.cmd_publish_timer.start(20)  # 20ms = 50Hz
        
        # Set default command type after a delay to ensure service is ready
        # Wait longer to ensure servo_node is fully initialized
        QTimer.singleShot(1000, self._set_default_command_type)
    
    def _ros_spin(self):
        """ROS2 spin loop (runs in separate thread)"""
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.ros_node)
        try:
            executor.spin()
        except Exception as e:
            print(f"ROS2 executor error: {e}")
    
    def _process_ros(self):
        """Process ROS2 callbacks"""
        rclpy.spin_once(self.ros_node, timeout_sec=0.0)
    
    def _publish_current_commands(self):
        """Continuously publish current commands to keep servo active"""
        # Publish current twist command
        if any(v != 0.0 for v in self.ros_node.current_twist_cmd.values()):
            self.ros_node.publish_twist(
                self.ros_node.current_twist_cmd["x"],
                self.ros_node.current_twist_cmd["y"],
                self.ros_node.current_twist_cmd["z"]
            )
        # Publish current joint command
        if self.ros_node.current_joint_cmd["index"] >= 0:
            self.ros_node.publish_joint(
                self.ros_node.current_joint_cmd["index"],
                self.ros_node.current_joint_cmd["velocity"]
            )
    
    def _set_default_command_type(self):
        """Set default command type to TWIST"""
        if self.ros_node.switch_input_client.service_is_ready():
            # Wait a bit more to ensure servo_node is fully initialized
            QTimer.singleShot(200, lambda: self._do_set_command_type())
        else:
            # Retry after another delay
            QTimer.singleShot(500, self._set_default_command_type)
    
    def _do_set_command_type(self):
        """Actually set the command type"""
        if self.ros_node.switch_command_type(ServoCommandType.Request.TWIST):
            self.update_status()
            self.ros_node.get_logger().info("Default command type set to TWIST")
        else:
            self.ros_node.get_logger().warn("Failed to set default command type, retrying...")
            # Retry after delay
            QTimer.singleShot(1000, self._set_default_command_type)
    
    def init_ui(self):
        """Initialize user interface"""
        self.setWindowTitle("UR Robot Servo Control")
        self.setGeometry(100, 100, 800, 600)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout()
        central_widget.setLayout(main_layout)
        
        # Status display area
        status_group = QGroupBox("Status Information")
        status_layout = QVBoxLayout()
        self.status_label = QLabel(f"Mode: TWIST | Command Frame: {EE_FRAME_ID} (robot_link_command_frame)")
        self.status_label.setFont(QFont("Arial", 10))
        status_layout.addWidget(self.status_label)
        status_group.setLayout(status_layout)
        main_layout.addWidget(status_group)
        
        # Cartesian space control
        cartesian_group = QGroupBox("Cartesian Space Control (Twist)")
        cartesian_layout = QGridLayout()
        
        # Create direction buttons
        self.btn_up = QPushButton("↑\nForward")
        self.btn_down = QPushButton("↓\nBackward")
        self.btn_left = QPushButton("←\nLeft")
        self.btn_right = QPushButton("→\nRight")
        self.btn_up_z = QPushButton("↑\nUp")
        self.btn_down_z = QPushButton("↓\nDown")
        
        # Set button size
        btn_size = 80
        for btn in [self.btn_up, self.btn_down, self.btn_left, self.btn_right, 
                    self.btn_up_z, self.btn_down_z]:
            btn.setMinimumSize(btn_size, btn_size)
            btn.setMaximumSize(btn_size, btn_size)
        
        # Layout buttons (cross shape)
        cartesian_layout.addWidget(self.btn_up, 0, 1)
        cartesian_layout.addWidget(self.btn_left, 1, 0)
        cartesian_layout.addWidget(self.btn_right, 1, 2)
        cartesian_layout.addWidget(self.btn_down, 2, 1)
        cartesian_layout.addWidget(self.btn_up_z, 1, 3)
        cartesian_layout.addWidget(self.btn_down_z, 1, 4)
        
        # Connect signals - use wrapper functions to avoid lambda closure issues
        # Use smaller values (0.1-0.2 m/s) for speed_units mode
        # servo.yaml has command_in_type: "speed_units", so values are in m/s
        def make_twist_handler(x, y, z):
            def handler(checked=False):
                self.ros_node.set_twist_command(linear_x=float(x), linear_y=float(y), linear_z=float(z))
            return handler
        
        # Use 0.15 m/s for reasonable speed (adjust as needed)
        self.btn_up.pressed.connect(make_twist_handler(0.15, 0.0, 0.0))
        self.btn_down.pressed.connect(make_twist_handler(-0.15, 0.0, 0.0))
        self.btn_left.pressed.connect(make_twist_handler(0.0, -0.15, 0.0))
        self.btn_right.pressed.connect(make_twist_handler(0.0, 0.15, 0.0))
        self.btn_up_z.pressed.connect(make_twist_handler(0.0, 0.0, 0.15))
        self.btn_down_z.pressed.connect(make_twist_handler(0.0, 0.0, -0.15))
        
        # Stop when released (released signal also passes checked parameter)
        def stop_twist(checked=False):
            self.ros_node.stop_all_commands()
        for btn in [self.btn_up, self.btn_down, self.btn_left, self.btn_right, 
                    self.btn_up_z, self.btn_down_z]:
            btn.released.connect(stop_twist)
        
        cartesian_group.setLayout(cartesian_layout)
        main_layout.addWidget(cartesian_group)
        
        # Joint space control
        joint_group = QGroupBox("Joint Space Control (Joint)")
        joint_layout = QGridLayout()
        
        # Create joint control buttons
        self.joint_buttons = []
        
        def stop_joint(checked=False):
            """Stop all joint motion"""
            self.ros_node.stop_all_commands()
        
        for i, joint_name in enumerate(JOINT_DISPLAY_NAMES):
            # Label
            label = QLabel(joint_name)
            label.setAlignment(Qt.AlignCenter)
            joint_layout.addWidget(label, i, 0)
            
            # Negative direction button
            btn_neg = QPushButton("-")
            btn_neg.setMinimumSize(60, 40)
            # Use wrapper function to correctly capture loop variable
            # For speed_units mode, command is in rad/s
            # servo.yaml has joint: 0.01 as max, so use 0.01 rad/s
            def make_joint_neg_handler(idx):
                def handler(checked=False):
                    self.ros_node.set_joint_command(idx, -0.01)
                return handler
            btn_neg.pressed.connect(make_joint_neg_handler(i))
            btn_neg.released.connect(stop_joint)
            joint_layout.addWidget(btn_neg, i, 1)
            
            # Positive direction button
            btn_pos = QPushButton("+")
            btn_pos.setMinimumSize(60, 40)
            def make_joint_pos_handler(idx):
                def handler(checked=False):
                    self.ros_node.set_joint_command(idx, 0.01)
                return handler
            btn_pos.pressed.connect(make_joint_pos_handler(i))
            btn_pos.released.connect(stop_joint)
            joint_layout.addWidget(btn_pos, i, 2)
            
            self.joint_buttons.append((btn_neg, btn_pos))
        
        joint_group.setLayout(joint_layout)
        main_layout.addWidget(joint_group)
        
        # Control buttons area
        control_group = QGroupBox("Control Options")
        control_layout = QHBoxLayout()
        
        # Mode switch buttons
        self.btn_mode_twist = QPushButton("Switch to Twist Mode")
        self.btn_mode_joint = QPushButton("Switch to Joint Mode")
        self.btn_mode_twist.clicked.connect(self.switch_to_twist)
        self.btn_mode_joint.clicked.connect(self.switch_to_joint)
        control_layout.addWidget(self.btn_mode_twist)
        control_layout.addWidget(self.btn_mode_joint)
        
        # Command frame switch buttons
        self.btn_frame_planning = QPushButton("Planning Frame")
        self.btn_frame_ee = QPushButton("End Effector Frame")
        self.btn_frame_planning.clicked.connect(self.switch_to_planning_frame)
        self.btn_frame_ee.clicked.connect(self.switch_to_ee_frame)
        control_layout.addWidget(self.btn_frame_planning)
        control_layout.addWidget(self.btn_frame_ee)
        
        # Reverse joint direction
        self.btn_reverse = QPushButton("Reverse Joint Direction")
        self.btn_reverse.clicked.connect(self.reverse_joint_direction)
        control_layout.addWidget(self.btn_reverse)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # Exit button
        exit_layout = QHBoxLayout()
        self.btn_exit = QPushButton("Exit")
        self.btn_exit.setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;")
        self.btn_exit.clicked.connect(self.close)
        exit_layout.addStretch()
        exit_layout.addWidget(self.btn_exit)
        exit_layout.addStretch()
        main_layout.addLayout(exit_layout)
        
        # Add stretch space
        main_layout.addStretch()
    
    def switch_to_twist(self):
        """Switch to Twist mode"""
        if self.ros_node.switch_command_type(ServoCommandType.Request.TWIST):
            self.update_status()
            QMessageBox.information(self, "Success", "Switched to Twist mode")
        else:
            QMessageBox.warning(self, "Failed", "Failed to switch to Twist mode")
    
    def switch_to_joint(self):
        """Switch to Joint mode"""
        if self.ros_node.switch_command_type(ServoCommandType.Request.JOINT_JOG):
            self.update_status()
            QMessageBox.information(self, "Success", "Switched to Joint mode")
        else:
            QMessageBox.warning(self, "Failed", "Failed to switch to Joint mode")
    
    def switch_to_planning_frame(self):
        """Switch to planning frame"""
        # Note: For Twist commands, servo.yaml uses robot_link_command_frame (tool0)
        # This setting is mainly for reference, actual Twist commands use tool0
        self.ros_node.command_frame_id = PLANNING_FRAME_ID
        self.update_status()
        QMessageBox.information(self, "Info", 
            f"Note: Twist commands use {EE_FRAME_ID} frame (robot_link_command_frame) as per servo.yaml configuration")
    
    def switch_to_ee_frame(self):
        """Switch to end effector frame"""
        # Twist commands always use robot_link_command_frame from servo.yaml (tool0)
        self.ros_node.command_frame_id = EE_FRAME_ID
        self.update_status()
        QMessageBox.information(self, "Info", 
            f"Twist commands use {EE_FRAME_ID} frame (robot_link_command_frame from servo.yaml)")
    
    def reverse_joint_direction(self):
        """Reverse joint motion direction"""
        self.ros_node.joint_vel_cmd *= -1
        direction = "Forward" if self.ros_node.joint_vel_cmd > 0 else "Reverse"
        QMessageBox.information(self, "Success", f"Joint direction switched to: {direction}")
    
    def update_status(self):
        """Update status display"""
        mode = self.ros_node.current_mode
        frame = self.ros_node.command_frame_id
        self.status_label.setText(f"Mode: {mode} | Command Frame: {frame}")
    
    def closeEvent(self, event):
        """Window close event"""
        reply = QMessageBox.question(
            self, 
            'Confirm Exit',
            'Are you sure you want to exit?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            # Stop all commands
            self.ros_node.stop_all_commands()
            # Stop timers
            self.timer.stop()
            self.cmd_publish_timer.stop()
            # Shutdown ROS2
            self.ros_node.destroy_node()
            rclpy.shutdown()
            event.accept()
        else:
            event.ignore()


def main(args=None):
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    window = ServoGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()

