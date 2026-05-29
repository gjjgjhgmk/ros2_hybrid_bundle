#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatusArray
from control_msgs.msg import JointTrajectoryControllerState
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


class OfflineDebugRecorder(Node):
    def __init__(self, out_dir: Path, frame_id: str, ee_link: str) -> None:
        super().__init__("offline_debug_recorder")
        self._out_dir = out_dir
        self._frame_id = frame_id
        self._ee_link = ee_link
        self._out_dir.mkdir(parents=True, exist_ok=True)

        self._joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self._name_to_idx = {}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_warned = False

        self._ee_rows: List[List[object]] = []
        self._ctrl_rows: List[List[object]] = []
        self._rosout_rows: List[List[object]] = []
        self._status_rows: List[List[object]] = []

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 50)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/controller_state",
            self._on_controller_state,
            50,
        )
        self.create_subscription(Log, "/rosout", self._on_rosout, 200)
        self.create_subscription(
            GoalStatusArray,
            "/joint_trajectory_controller/follow_joint_trajectory/_action/status",
            self._on_action_status,
            50,
        )

        self.get_logger().info(
            "offline_debug_recorder started: "
            f"out_dir={self._out_dir}, frame={self._frame_id}, ee_link={self._ee_link}"
        )

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _lookup_ee(self) -> List[object]:
        try:
            tfm = self._tf_buffer.lookup_transform(self._frame_id, self._ee_link, rclpy.time.Time())
            return [
                float(tfm.transform.translation.x),
                float(tfm.transform.translation.y),
                float(tfm.transform.translation.z),
                1,
            ]
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self._tf_warned:
                self.get_logger().warn(
                    f"TF lookup failed for {self._frame_id}->{self._ee_link}, recording NaN EE points."
                )
                self._tf_warned = True
            return [math.nan, math.nan, math.nan, 0]

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        if not self._name_to_idx:
            self._name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in self._name_to_idx for n in self._joint_names):
            return
        idx = [self._name_to_idx[n] for n in self._joint_names]
        if len(msg.position) < max(idx) + 1:
            return
        q = [float(msg.position[i]) for i in idx]
        ee_x, ee_y, ee_z, tf_ok = self._lookup_ee()
        t_sec = self._stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec != 0 else self.get_clock().now().nanoseconds * 1e-9
        self._ee_rows.append([t_sec] + q + [ee_x, ee_y, ee_z, tf_ok])

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        desired = [float(x) for x in msg.reference.positions]
        actual = [float(x) for x in msg.actual.positions]
        error = [float(x) for x in msg.error.positions]
        max_abs_err = max((abs(v) for v in error), default=0.0)
        self._ctrl_rows.append(
            [
                now,
                max_abs_err,
                json.dumps(desired, separators=(",", ":")),
                json.dumps(actual, separators=(",", ":")),
                json.dumps(error, separators=(",", ":")),
            ]
        )

    def _on_rosout(self, msg: Log) -> None:
        # 只保留与当前问题强相关的日志，减小文件体积。
        name = str(msg.name)
        if (
            "move_group" not in name
            and "joint_trajectory_controller" not in name
            and "intent_runtime_bridge" not in name
            and "intent_hybrid_planner" not in name
            and "tolerances" not in name
        ):
            return
        t_sec = self._stamp_to_sec(msg.stamp) if msg.stamp.sec != 0 else self.get_clock().now().nanoseconds * 1e-9
        self._rosout_rows.append([t_sec, int(msg.level), name, str(msg.msg)])

    def _on_action_status(self, msg: GoalStatusArray) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        status_codes = [int(s.status) for s in msg.status_list]
        self._status_rows.append([now, json.dumps(status_codes, separators=(",", ":"))])

    def dump(self) -> None:
        ee_path = self._out_dir / "ee_trace.csv"
        ctrl_path = self._out_dir / "controller_state.csv"
        rosout_path = self._out_dir / "rosout_filtered.csv"
        status_path = self._out_dir / "action_status.csv"
        meta_path = self._out_dir / "meta.json"

        with ee_path.open("w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "t_sec",
                    "q0",
                    "q1",
                    "q2",
                    "q3",
                    "q4",
                    "q5",
                    "ee_x",
                    "ee_y",
                    "ee_z",
                    "tf_ok",
                ]
            )
            w.writerows(self._ee_rows)

        with ctrl_path.open("w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_sec", "max_abs_error", "desired_positions", "actual_positions", "error_positions"])
            w.writerows(self._ctrl_rows)

        with rosout_path.open("w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_sec", "level", "logger_name", "message"])
            w.writerows(self._rosout_rows)

        with status_path.open("w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_sec", "status_codes"])
            w.writerows(self._status_rows)

        meta = {
            "frame_id": self._frame_id,
            "ee_link": self._ee_link,
            "rows": {
                "ee_trace": len(self._ee_rows),
                "controller_state": len(self._ctrl_rows),
                "rosout_filtered": len(self._rosout_rows),
                "action_status": len(self._status_rows),
            },
        }
        meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        self.get_logger().info(f"Debug artifacts written to: {self._out_dir}")


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", default="", help="Output directory for CSV/log artifacts.")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="tool0")
    ns, ros_args = parser.parse_known_args(args=args)

    out_dir = Path(ns.out_dir).expanduser() if ns.out_dir else (Path.cwd() / "debug" / "latest")
    rclpy.init(args=ros_args)
    node = OfflineDebugRecorder(out_dir=out_dir, frame_id=ns.frame_id, ee_link=ns.ee_link)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.dump()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

