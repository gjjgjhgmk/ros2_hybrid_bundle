import sys
import os
import logging
import time

# 添加项目路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from ur_bt import BehaviorTreeManager, behavior_tree
from ur_bt.behaviors.arm_waypoint_behavior import WaypointUpdateConfig, WaypointCopyConfig


class PickAndPlace:
    def __init__(self, debug: bool = False):
        self.bt_manager = BehaviorTreeManager(
            config_path="config.yaml",
            waypoints_path="waypoints.json",
            show_progress=True,
            show_tree=False
        )
        self.task_name = "抓取放置demo"
        self.debug = debug
        self.vel_scale = 0.1
        self.acc_scale = 0.1
        
    def ready(self):
        """准备"""
        # 右臂到Home位置
        move_to_home = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("右臂-home", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：准备-右臂到home"
        )
        
        # 手爪准备
        ready_right_gripper = self.bt_manager.gripper_behavior.open("right", name="打开右手夹爪")
        # ready_left_gripper = self.bt_manager.gripper_behavior.open("left", name="打开左手夹爪")
        sleep = self.bt_manager.utility_behavior.sleep(duration=1.0, name=f"{self.task_name}：延时")

        behaviors = [
            move_to_home,
            ready_right_gripper,
            # ready_left_gripper,
            sleep,
        ]

        if self.debug:
            behaviors = [
                move_to_home,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                ready_right_gripper,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                # ready_left_gripper,
                # self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
            ]
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")
    
    def recognize_object(self):
        """识别物体"""
        wait_stable = self.bt_manager.utility_behavior.sleep(duration=1.0, name=f"{self.task_name}：识别-物体-等待稳定")
        right_camera_recognize_object = self.bt_manager.vision_behavior.pose_estimation_mask(
            camera_name="right_camera",
            target_frame="right_interface_link",
            name="识别-物体"
        )

        update_pre_grasp_object_waypoint = self.bt_manager.arm_waypoint_behavior.update_waypoint_from_vision(
            waypoint_name="右臂-预抓取",
            update_config=WaypointUpdateConfig.from_dict({
                "x": {"mode": "offset", "value": 0.0, "reference": "vision"},
                "y": {"mode": "offset", "value": 0.0, "reference": "vision"},
                "z": {"mode": "keep", "value": 0.0, "reference": "waypoint"},
                "yaw": {"mode": "offset", "value": 0.0, "reference": "vision"},
            }),
            target_category=None,
            name=f"{self.task_name}：识别-物体-更新预抓取物体点位"
        )

        update_grasp_object_waypoint = self.bt_manager.arm_waypoint_behavior.update_waypoint_from_vision(
            waypoint_name="右臂-抓取",
            update_config=WaypointUpdateConfig.from_dict({
                "x": {"mode": "offset", "value": 0.0, "reference": "vision"},
                "y": {"mode": "offset", "value": 0.0, "reference": "vision"},
                "z": {"mode": "keep", "value": 0.0, "reference": "waypoint"},
                "yaw": {"mode": "offset", "value": 0, "reference": "vision"},
            }),
            target_category=None,
            name=f"{self.task_name}：识别-物体-更新抓取物体点位"
        )
        update_withdraw_grasp_object_waypoint = self.bt_manager.arm_waypoint_behavior.update_waypoint_from_waypoint(
            source_waypoint_name="右臂-抓取",
            target_waypoint_name="右臂-抓取撤退",
            update_config=WaypointCopyConfig.from_dict({
                "z": {"mode": "offset", "value": 0.15},
            }),
            name=f"{self.task_name}：识别-物体-更新抓取撤退物体点位"
        )

        behaviors = [
            wait_stable,
            right_camera_recognize_object,
            update_pre_grasp_object_waypoint,
            update_grasp_object_waypoint,
            update_withdraw_grasp_object_waypoint
        ]
        if self.debug:
            behaviors.append(self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."))
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")
    
    def grasp_object(self):
        """抓取物体"""
        pre_grasp_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("右臂-预抓取", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：右臂-预抓取-物体"
        )
        grasp_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("右臂-抓取", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：右臂-抓取-物体"
        )
        right_gripper_grasp_object = self.bt_manager.gripper_behavior.close("right", name="关闭右手夹爪")
        sleep = self.bt_manager.utility_behavior.sleep(duration=1.0, name=f"{self.task_name}：延时")

        grasp_withdraw_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("右臂-抓取撤退", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：右臂-抓取撤退-物体"
        )
        behaviors = [
            pre_grasp_object,
            grasp_object,
            right_gripper_grasp_object,
            sleep,
            grasp_withdraw_object,
        ]
        if self.debug:
            behaviors = [
                pre_grasp_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                grasp_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                right_gripper_grasp_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                grasp_withdraw_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
            ]
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")

    def right_handover_object(self):
        """右手交接物体"""
        right_handover_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("右臂-交接", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：右臂-交接-物体"
        )
        behaviors = [
            right_handover_object,
        ]
        if self.debug:
            behaviors = [
                right_handover_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
            ]  
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")

    def left_handover_object(self):
        """左手交接物体"""
        left_pre_handover_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("左臂-预交接", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：左臂-预交接-物体"
        )
        left_handover_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("左臂-交接", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：左臂-交接-物体"
        )
        left_gripper_handover_object = self.bt_manager.gripper_behavior.close("left", name="关闭左手夹爪")
        sleep = self.bt_manager.utility_behavior.sleep(duration=1.0, name=f"{self.task_name}：延时")
        right_gripper_release_object = self.bt_manager.gripper_behavior.open("right", name="打开右手夹爪")
        sleep_2 = self.bt_manager.utility_behavior.sleep(duration=1.0, name=f"{self.task_name}：延时")
        behaviors = [
            left_pre_handover_object,
            left_handover_object,
            left_gripper_handover_object,
            sleep,
            right_gripper_release_object,
            sleep_2,
        ]
        if self.debug:
            behaviors = [
                left_pre_handover_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                left_handover_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                left_gripper_handover_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                right_gripper_release_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
            ]  
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")

    def left_put_object(self):
        """左手放置物体"""
        left_pre_put_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("左臂-预放置", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：左臂-预放置-物体"
        )
        left_put_object = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("左臂-放置", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：左臂-放置-物体"
        )
        left_gripper_put_object = self.bt_manager.gripper_behavior.open("left", name="打开左手夹爪")
        sleep = self.bt_manager.utility_behavior.sleep(duration=1.0, name=f"{self.task_name}：延时")
        behaviors = [
            left_pre_put_object,
            left_put_object,
            left_gripper_put_object,
            sleep,
        ]
        if self.debug:
            behaviors = [
                left_pre_put_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                left_put_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                left_gripper_put_object,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
            ]  
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")  
    
    def return_home(self):
        """撤退并回home"""
        
        return_home = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("右臂-home", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：回home"
        )
        left_return_home = self.bt_manager.arm_move_behavior.move_to_waypoints(
            waypoint_configs=[("左臂-home", self.vel_scale, self.acc_scale)],
            name=f"{self.task_name}：左臂-回home"
        )
        behaviors = [
            return_home,
            left_return_home
        ]
        if self.debug:
            behaviors = [
                return_home,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
                left_return_home,
                self.bt_manager.utility_behavior.wait_for_input("请按回车键继续..."),
            ]
        success = self.bt_manager.execute(behaviors, wait=True)
        if not success:
            raise RuntimeError("执行失败")
    
    def execute(self):
        """执行任务"""
        self.ready(),
        self.recognize_object(),
        self.grasp_object(),
        self.right_handover_object(),
        self.left_handover_object(),
        self.left_put_object(),
        self.return_home()

if __name__ == "__main__":
    pick_and_place = PickAndPlace(debug=True)
    pick_and_place.execute()