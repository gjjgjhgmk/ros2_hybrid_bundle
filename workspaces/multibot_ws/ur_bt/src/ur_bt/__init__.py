"""
PARM BT - 基于py_trees的机器人决策系统框架
"""

__version__ = "0.1.0"
__author__ = "PARM Team"

from .behavior_tree import BehaviorTreeManager
from .behaviors.arm_move_behavior import ArmMoveBehavior
from .behaviors.arm_waypoint_behavior import ArmWaypointBehavior
from .blackboard_manager import BlackboardError

__all__ = [
    "BehaviorTreeManager",
    "ArmMoveBehavior",
    "ArmWaypointBehavior",
    "BlackboardError",
]
