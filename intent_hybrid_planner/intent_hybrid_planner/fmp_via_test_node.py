#!/usr/bin/env python3
"""
UR7e empty-environment FMP via-point execution test node.

Scope of this node:
- No obstacle scene.
- No hybrid planner orchestration.
- Only verifies: nominal -> via points -> FMP modulation -> trajectory execution.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import fmp_core


class FMPViaTestNode(Node):
    """Standalone ROS2 node for FMP via-point execution validation."""

    UR7E_JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    def __init__(self) -> None:
        super().__init__("fmp_via_test_node")

        self.declare_parameter(
            "trajectory_action_name",
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("action_wait_timeout_sec", 5.0)
        self.declare_parameter("state_wait_timeout_sec", 5.0)
        self.declare_parameter("terminal_tolerance", 0.08)
        self.declare_parameter("result_dir", "result")

        self.trajectory_action_name = (
            self.get_parameter("trajectory_action_name").get_parameter_value().string_value
        )
        self.joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self.action_wait_timeout_sec = (
            self.get_parameter("action_wait_timeout_sec").get_parameter_value().double_value
        )
        self.state_wait_timeout_sec = (
            self.get_parameter("state_wait_timeout_sec").get_parameter_value().double_value
        )
        self.terminal_tolerance = (
            self.get_parameter("terminal_tolerance").get_parameter_value().double_value
        )
        self.result_dir = (
            self.get_parameter("result_dir").get_parameter_value().string_value or "result"
        )

        self._q_now = np.zeros(len(self.UR7E_JOINT_NAMES), dtype=float)
        self._has_joint_state = False
        self._joint_name_to_idx: Dict[str, int] = {}
        self._start_monotonic = time.monotonic()
        self._goal_sent = False
        self._finished = False
        self._expected_final_q: Optional[np.ndarray] = None
        self._last_result: Dict[str, object] = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.create_subscription(JointState, self.joint_state_topic, self._on_joint_states, qos)

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
        )
        self._tick_timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            "FMP via test node started. "
            f"action={self.trajectory_action_name}, joint_state_topic={self.joint_state_topic}"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        if not self._joint_name_to_idx:
            self._joint_name_to_idx = {name: i for i, name in enumerate(msg.name)}

        if any(j not in self._joint_name_to_idx for j in self.UR7E_JOINT_NAMES):
            return

        idx = [self._joint_name_to_idx[j] for j in self.UR7E_JOINT_NAMES]
        if len(msg.position) < max(idx) + 1:
            return

        self._q_now = np.array([msg.position[i] for i in idx], dtype=float)
        self._has_joint_state = True

    def _build_nominal_trajectory(self, q_start: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        n = 150
        t = np.linspace(0.0, 10.0, n)
        start_q = np.asarray(q_start, dtype=float).reshape(-1)
        if start_q.size != len(self.UR7E_JOINT_NAMES):
            raise ValueError("q_start shape mismatch.")
        # Keep movement conservative in empty-scene validation to avoid path tolerance aborts.
        delta = np.array([0.25, -0.12, 0.18, -0.10, 0.08, 0.0], dtype=float)
        end_q = start_q + delta
        blend = np.linspace(0.0, 1.0, n, dtype=float)
        nominal = start_q[:, None] + (end_q - start_q)[:, None] * blend[None, :]
        return nominal, t

    def _build_via_points(
        self,
        nominal: np.ndarray,
        t: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        via_indices = np.array([45, 75, 105], dtype=int)
        via_times = t[via_indices]

        offsets = np.array(
            [
                [0.08, 0.05, 0.07],  # shoulder_pan
                [0.00, 0.00, 0.00],  # shoulder_lift
                [0.06, -0.04, 0.05],  # elbow
                [0.00, 0.00, 0.00],  # wrist_1
                [0.03, 0.03, 0.02],  # wrist_2
                [0.00, 0.00, 0.00],  # wrist_3
            ],
            dtype=float,
        )
        via_points = nominal[:, via_indices] + offsets
        return via_points, via_times

    def _build_trajectory_msg(self, traj: np.ndarray, t: np.ndarray) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = list(self.UR7E_JOINT_NAMES)
        for i in range(traj.shape[1]):
            pt = JointTrajectoryPoint()
            pt.positions = traj[:, i].tolist()
            pt.velocities = [0.0] * traj.shape[0]
            pt.accelerations = [0.0] * traj.shape[0]
            sec = int(t[i])
            nanosec = int((float(t[i]) - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(pt)
        return msg

    def _tick(self) -> None:
        if self._finished:
            return

        now = time.monotonic()
        elapsed = now - self._start_monotonic

        if not self._has_joint_state:
            if elapsed > self.state_wait_timeout_sec:
                self._finalize(
                    status="FAIL_NO_JOINT_STATE",
                    message=(
                        f"No valid joint state within {self.state_wait_timeout_sec:.2f}s "
                        f"from topic {self.joint_state_topic}"
                    ),
                )
            return

        if self._goal_sent:
            return

        if not self.action_client.wait_for_server(timeout_sec=0.05):
            if elapsed > self.action_wait_timeout_sec:
                self._finalize(
                    status="FAIL_ACTION_SERVER_NOT_READY",
                    message=(
                        f"Action server not ready within {self.action_wait_timeout_sec:.2f}s: "
                        f"{self.trajectory_action_name}"
                    ),
                )
            return

        try:
            nominal, t = self._build_nominal_trajectory(self._q_now)
            via_points, via_times = self._build_via_points(nominal, t)

            model = fmp_core.train_fmp_model(
                demo_traj=nominal,
                time_axis=t,
                N_C=20,
                alpha=0.1,
            )
            modulated = fmp_core.modulate_trajectory(
                fmp_model=model,
                demo_traj=nominal,
                time_axis=t,
                via_points=via_points,
                via_times=via_times,
                transition_ratio=0.1,
                transition_gamma=1.0,
            )
            # Stitch first point to current hardware state to satisfy controller path tolerance.
            modulated[:, 0] = self._q_now.copy()
            self._expected_final_q = modulated[:, -1].copy()

            traj_msg = self._build_trajectory_msg(modulated, t)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj_msg

            self._goal_sent = True
            send_future = self.action_client.send_goal_async(goal)
            send_future.add_done_callback(self._on_goal_response)
            self.get_logger().info(
                "Trajectory goal sent for FMP via test. "
                f"points={modulated.shape[1]}, vias={via_times.size}"
            )
        except Exception as exc:  # pylint: disable=broad-except
            self._finalize(status="FAIL_BUILD_OR_SEND_EXCEPTION", message=str(exc))

    def _on_goal_response(self, fut) -> None:
        if self._finished:
            return
        try:
            goal_handle = fut.result()
        except Exception as exc:  # pylint: disable=broad-except
            self._finalize(status="FAIL_SEND_GOAL_EXCEPTION", message=str(exc))
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finalize(status="FAIL_GOAL_REJECTED", message="Goal rejected by action server.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, fut) -> None:
        if self._finished:
            return
        try:
            wrapped = fut.result()
            result = wrapped.result
            status = int(wrapped.status)
            error_code = int(getattr(result, "error_code", -999))
            error_string = str(getattr(result, "error_string", ""))
        except Exception as exc:  # pylint: disable=broad-except
            self._finalize(status="FAIL_RESULT_EXCEPTION", message=str(exc))
            return

        terminal_error = None
        pass_terminal = False
        if self._expected_final_q is not None and self._has_joint_state:
            terminal_error = float(np.max(np.abs(self._q_now - self._expected_final_q)))
            pass_terminal = terminal_error <= self.terminal_tolerance

        success = (status == 4) and (error_code == 0) and pass_terminal
        if success:
            self._finalize(
                status="PASS",
                message="Goal executed successfully and terminal error within tolerance.",
                extra={
                    "action_status": status,
                    "error_code": error_code,
                    "error_string": error_string,
                    "terminal_error_max_abs": terminal_error,
                    "terminal_tolerance": self.terminal_tolerance,
                },
            )
            return

        self._finalize(
            status="FAIL_EXECUTION",
            message="Goal result or terminal error did not satisfy pass criteria.",
            extra={
                "action_status": status,
                "error_code": error_code,
                "error_string": error_string,
                "terminal_error_max_abs": terminal_error,
                "terminal_tolerance": self.terminal_tolerance,
            },
        )

    def _finalize(
        self,
        status: str,
        message: str,
        extra: Optional[Dict[str, object]] = None,
    ) -> None:
        if self._finished:
            return
        self._finished = True
        if self._tick_timer is not None:
            self._tick_timer.cancel()

        payload = {
            "status": status,
            "message": message,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "node": "fmp_via_test_node",
            "action_name": self.trajectory_action_name,
            "joint_state_topic": self.joint_state_topic,
            "elapsed_sec": float(time.monotonic() - self._start_monotonic),
            "has_joint_state": bool(self._has_joint_state),
            "goal_sent": bool(self._goal_sent),
        }
        if extra:
            payload.update(extra)
        self._last_result = payload

        out_dir = Path.cwd() / self.result_dir
        out_dir.mkdir(parents=True, exist_ok=True)
        filename = time.strftime("fmp_via_test_%Y%m%d_%H%M%S.json", time.localtime())
        out_path = out_dir / filename
        out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

        log_fn = self.get_logger().info if status == "PASS" else self.get_logger().error
        log_fn(f"[{status}] {message}")
        self.get_logger().info(f"Result saved to: {out_path}")

        try:
            self.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FMPViaTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.context.ok():
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
