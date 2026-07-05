#!/usr/bin/env python3
"""
Single unified ROS 2 node for hybrid intent-based planning.

Target:
- ROS 2 Humble
- MoveIt 2 state validity service
- FollowJointTrajectory action adapter (placeholder execution)
"""

import time
import csv
import json
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject, MoveItErrorCodes, PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetPositionIK, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray

from . import fmp_core
from . import hybrid_matlab_compat as hm_compat
from .intent_biased_rrt import IntentBiasedRRT

try:
    import yaml
except Exception:  # pragma: no cover - optional dependency
    yaml = None

try:
    from moveit.planning import MoveItPy
except Exception:  # pragma: no cover - optional ROS dependency
    MoveItPy = None

try:
    from moveit.core.robot_state import RobotState as MoveItRobotState
except Exception:  # pragma: no cover - optional ROS dependency
    MoveItRobotState = None

try:
    import ruckig as ruckig_lib
except Exception:  # pragma: no cover - optional dependency
    ruckig_lib = None

try:
    from intent_hybrid_interfaces.srv import (
        CheckMotionBatch,
        CheckStatesBatch,
        DispatchJointTrajectory,
        PlanLocalSegment,
        PublishPlanningMarkers,
    )
except Exception:  # pragma: no cover - optional dependency
    CheckMotionBatch = None
    CheckStatesBatch = None
    DispatchJointTrajectory = None
    PlanLocalSegment = None
    PublishPlanningMarkers = None


class MoveItSceneMonitor:
    """
    Lightweight MoveIt state validity monitor using async service calls.

    The orchestrator should use request_collision_check() + poll_collision_result()
    to keep a non-blocking control loop.
    """

    def __init__(self, node: Node, joint_names: List[str], group_name: str = "manipulator") -> None:
        self._node = node
        self._joint_names = joint_names
        self._group_name = group_name
        self._client = node.create_client(GetStateValidity, "/check_state_validity")
        self._pending_future = None
        self._pending_angles: Optional[List[float]] = None
        self._last_result: Optional[bool] = None
        self._last_not_ready_warn_ts = 0.0
        self._not_ready_warn_interval_sec = 5.0

    def _build_request(self, joint_angles: List[float]) -> GetStateValidity.Request:
        request = GetStateValidity.Request()
        request.group_name = self._group_name

        robot_state = RobotState()
        joint_state = JointState()
        joint_state.name = list(self._joint_names)
        joint_state.position = [float(x) for x in joint_angles]
        robot_state.joint_state = joint_state

        request.robot_state = robot_state
        return request

    def has_pending_request(self) -> bool:
        return self._pending_future is not None

    def request_collision_check(self, joint_angles: List[float]) -> bool:
        """
        Start an async collision check request.

        Returns True when request is sent.
        Returns False if service is unavailable or a request is already pending.
        """
        if len(joint_angles) != len(self._joint_names):
            self._node.get_logger().error(
                "Collision check request rejected: joint vector size mismatch "
                f"(got {len(joint_angles)}, expected {len(self._joint_names)})."
            )
            return False

        if self._pending_future is not None:
            self._node.get_logger().debug("Collision check request skipped: previous request still pending.")
            return False

        if not self._client.service_is_ready():
            now = time.monotonic()
            if now - self._last_not_ready_warn_ts >= self._not_ready_warn_interval_sec:
                self._node.get_logger().warn(
                    "MoveIt service /check_state_validity is not ready. "
                    "Skip this cycle and keep loop non-blocking."
                )
                self._last_not_ready_warn_ts = now
            return False

        request = self._build_request(joint_angles)
        self._pending_future = self._client.call_async(request)
        self._pending_angles = list(joint_angles)
        return True

    def poll_collision_result(self) -> Optional[bool]:
        """
        Poll current async request.

        Returns:
        - True  : collision-free
        - False : in collision or service call failed
        - None  : still pending / no request yet
        """
        if self._pending_future is None:
            return None

        if not self._pending_future.done():
            return None

        try:
            response = self._pending_future.result()
            is_valid = bool(response.valid)
            self._last_result = is_valid
            return is_valid
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"Collision check future failed: {exc}")
            self._last_result = False
            return False
        finally:
            self._pending_future = None
            self._pending_angles = None

    def check_state_collision_free(self, joint_angles: List[float]) -> bool:
        """
        Unified entry method required by API.

        Non-blocking behavior:
        - If no request is pending: start async request and return last known result
          (or True when no history exists, optimistic default).
        - If request is pending: poll once and return available result or optimistic default.
        """
        if self._pending_future is None:
            started = self.request_collision_check(joint_angles)
            if not started:
                return self._last_result if self._last_result is not None else True
            return self._last_result if self._last_result is not None else True

        polled = self.poll_collision_result()
        if polled is None:
            return self._last_result if self._last_result is not None else True
        return polled


class FMPCore:
    """真实 FMP 调制模块。"""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._node.get_logger().info("Initializing FMP Model...")

        self.fmp_model = None
        self.time_axis, self.nominal_intent = self._build_default_nominal()
        self.retrain_model()

    def _build_default_nominal(self) -> Tuple[np.ndarray, np.ndarray]:
        # 横向基座扫掠名义轨迹，时间轴按 demo_len/demo_dt 构造，和 MATLAB 语义对齐。
        demo_len = int(max(getattr(self._node, "demo_len", 150), 2))
        demo_dt = float(max(getattr(self._node, "demo_dt", 0.1), 1e-3))
        t = np.arange(demo_len, dtype=float) * demo_dt
        start_q = np.array([-0.8, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=float)
        end_q = np.array([0.8, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=float)
        blend = np.linspace(0.0, 1.0, demo_len, dtype=float)
        nominal = start_q[:, None] + (end_q - start_q)[:, None] * blend[None, :]
        return t, nominal

    def set_nominal_trajectory(self, nominal_traj: np.ndarray, time_axis: np.ndarray) -> bool:
        traj = np.asarray(nominal_traj, dtype=float)
        t = np.asarray(time_axis, dtype=float).reshape(-1)
        if traj.ndim != 2 or t.size != traj.shape[1] or t.size < 2:
            self._node.get_logger().error(
                f"Invalid nominal trajectory shape/time axis: traj={traj.shape}, t={t.shape}"
            )
            return False

        self.nominal_intent = traj
        self.time_axis = t
        return self.retrain_model()

    def retrain_model(self) -> bool:
        # 离线训练真实 FMP 模型（失败则回退名义轨迹）
        try:
            self.fmp_model = fmp_core.train_fmp_model(
                demo_traj=self.nominal_intent,
                time_axis=self.time_axis,
                N_C=20,
                alpha=0.1,
            )
            return True
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"FMP Train Error: {exc}")
            self.fmp_model = None
            return False

    def modulate(
        self,
        nominal_point: List[float],
        via_points: Optional[np.ndarray] = None,
        via_times: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        返回轨迹矩阵，形状固定为 (6, N)。
        """
        _ = nominal_point
        if via_points is None or via_times is None or self.fmp_model is None:
            return self.nominal_intent

        via_points_arr = np.asarray(via_points, dtype=float)
        via_times_arr = np.asarray(via_times, dtype=float).reshape(-1)
        if via_points_arr.size == 0 or via_times_arr.size == 0:
            return self.nominal_intent

        # via_points 必须是 (6, M)，且与 via_times 的 M 对齐。
        if via_points_arr.ndim != 2:
            return self.nominal_intent
        if via_points_arr.shape[0] != self.nominal_intent.shape[0]:
            return self.nominal_intent
        if via_points_arr.shape[1] != via_times_arr.size:
            return self.nominal_intent

        try:
            return fmp_core.modulate_trajectory(
                fmp_model=self.fmp_model,
                demo_traj=self.nominal_intent,
                time_axis=self.time_axis,
                via_points=via_points_arr,
                via_times=via_times_arr,
            )
        except Exception as exc:  # pylint: disable=broad-except
            self._node.get_logger().error(f"FMP Modulate Error: {exc}")
            return self.nominal_intent

class IntentHybridPlannerNode(Node):
    """Single unified orchestrator node."""

    DEFAULT_JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    @staticmethod
    def _stats_from_samples(samples: List[float]) -> Dict[str, float]:
        if not samples:
            return {"mean": 0.0, "p95": 0.0, "max": 0.0, "count": 0}
        arr = np.asarray(samples, dtype=float)
        return {
            "mean": float(np.mean(arr)),
            "p95": float(np.percentile(arr, 95)),
            "max": float(np.max(arr)),
            "count": int(arr.size),
        }

    @staticmethod
    def _normalize_vec(v: np.ndarray, eps: float = 1e-9) -> Optional[np.ndarray]:
        arr = np.asarray(v, dtype=float).reshape(-1)
        if arr.size != 3:
            return None
        n = float(np.linalg.norm(arr))
        if n <= eps:
            return None
        return arr / n

    @staticmethod
    def _quat_from_matrix(rot: np.ndarray) -> Quaternion:
        r = np.asarray(rot, dtype=float)
        q = Quaternion()
        tr = float(r[0, 0] + r[1, 1] + r[2, 2])
        if tr > 0.0:
            s = np.sqrt(tr + 1.0) * 2.0
            q.w = 0.25 * s
            q.x = (r[2, 1] - r[1, 2]) / s
            q.y = (r[0, 2] - r[2, 0]) / s
            q.z = (r[1, 0] - r[0, 1]) / s
            return q
        if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
            s = np.sqrt(max(1.0 + r[0, 0] - r[1, 1] - r[2, 2], 1e-12)) * 2.0
            q.w = (r[2, 1] - r[1, 2]) / s
            q.x = 0.25 * s
            q.y = (r[0, 1] + r[1, 0]) / s
            q.z = (r[0, 2] + r[2, 0]) / s
            return q
        if r[1, 1] > r[2, 2]:
            s = np.sqrt(max(1.0 + r[1, 1] - r[0, 0] - r[2, 2], 1e-12)) * 2.0
            q.w = (r[0, 2] - r[2, 0]) / s
            q.x = (r[0, 1] + r[1, 0]) / s
            q.y = 0.25 * s
            q.z = (r[1, 2] + r[2, 1]) / s
            return q
        s = np.sqrt(max(1.0 + r[2, 2] - r[0, 0] - r[1, 1], 1e-12)) * 2.0
        q.w = (r[1, 0] - r[0, 1]) / s
        q.x = (r[0, 2] + r[2, 0]) / s
        q.y = (r[1, 2] + r[2, 1]) / s
        q.z = 0.25 * s
        return q

    def _build_plane_basis(self) -> bool:
        n = self._normalize_vec(self.ee_plane_normal)
        if n is None:
            self.get_logger().error("ee_plane_normal_xyz is invalid.")
            return False
        origin = np.asarray(self.ee_plane_origin, dtype=float).reshape(-1)
        if origin.size != 3:
            self.get_logger().error("ee_plane_origin_xyz must be a 3D vector.")
            return False

        helper = np.array([0.0, 0.0, 1.0], dtype=float)
        if abs(float(np.dot(helper, n))) > 0.9:
            helper = np.array([1.0, 0.0, 0.0], dtype=float)
        u = np.cross(helper, n)
        u = self._normalize_vec(u)
        if u is None:
            self.get_logger().error("Failed to construct plane axis u.")
            return False
        v = np.cross(n, u)
        v = self._normalize_vec(v)
        if v is None:
            self.get_logger().error("Failed to construct plane axis v.")
            return False

        self._plane_origin = origin
        self._plane_normal = n
        self._plane_u = u
        self._plane_v = v
        return True

    def _parse_ee_path_param(self) -> Dict[str, Any]:
        raw = (self.ee_path_json or "").strip()
        default_samples = int(max(getattr(self, "demo_len", 150), 2))
        default_demo_dt = float(max(getattr(self, "demo_dt", 0.1), 1e-3))
        default = {
            "waypoints_uv": [[0.35, -0.15], [0.55, 0.15]],
            "z_offset": 0.0,
            "samples": default_samples,
            "duration_sec": float(default_samples * default_demo_dt),
        }
        if not raw:
            return default
        try:
            obj = json.loads(raw)
            waypoints = obj.get("waypoints_uv", default["waypoints_uv"])
            samples = int(obj.get("samples", default["samples"]))
            duration_sec = float(obj.get("duration_sec", default["duration_sec"]))
            z_offset = float(obj.get("z_offset", default["z_offset"]))
            out_wp: List[List[float]] = []
            if isinstance(waypoints, list):
                for w in waypoints:
                    if isinstance(w, (list, tuple)) and len(w) == 2:
                        out_wp.append([float(w[0]), float(w[1])])
            if len(out_wp) < 2:
                out_wp = list(default["waypoints_uv"])
            samples = int(max(samples, 2))
            duration_sec = float(max(duration_sec, 0.5))
            return {
                "waypoints_uv": out_wp,
                "z_offset": z_offset,
                "samples": samples,
                "duration_sec": duration_sec,
            }
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"ee_path_json parse failed: {exc}. Use default path.")
            return default

    def _parse_plane_obstacles_param(self) -> List[Dict[str, float]]:
        raw = (self.plane_obstacles_json or "").strip()
        if not raw:
            return [
                {"u": 0.45, "v": 0.00, "radius": 0.08, "z": 0.0},
                {"u": 0.45, "v": 0.20, "radius": 0.08, "z": 0.0},
            ]
        try:
            parsed = json.loads(raw)
            out: List[Dict[str, float]] = []
            if isinstance(parsed, list):
                for item in parsed:
                    if not isinstance(item, dict):
                        continue
                    r = float(item.get("radius", 0.0))
                    if r <= 0.0:
                        continue
                    out.append(
                        {
                            "u": float(item.get("u", 0.0)),
                            "v": float(item.get("v", 0.0)),
                            "radius": r,
                            "z": float(item.get("z", 0.0)),
                        }
                    )
            if out:
                return out
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"plane_obstacles_json parse failed: {exc}. Use default plane obstacles.")
        return [
            {"u": 0.45, "v": 0.00, "radius": 0.08, "z": 0.0},
            {"u": 0.45, "v": 0.20, "radius": 0.08, "z": 0.0},
        ]

    def _map_uvz_to_xyz(self, u: float, v: float, z_offset: float) -> np.ndarray:
        return self._plane_origin + self._plane_u * float(u) + self._plane_v * float(v) + self._plane_normal * float(z_offset)

    def _xyz_to_plane_uv(self, xyz: np.ndarray) -> np.ndarray:
        d = np.asarray(xyz, dtype=float).reshape(3) - self._plane_origin
        return np.array([np.dot(d, self._plane_u), np.dot(d, self._plane_v)], dtype=float)

    def _build_ee_plane_nominal_path(self) -> Tuple[np.ndarray, List[Quaternion], np.ndarray]:
        cfg = self._parse_ee_path_param()
        waypoints = np.asarray(cfg["waypoints_uv"], dtype=float)
        n_samples = int(cfg["samples"])
        duration_sec = float(cfg["duration_sec"])
        z_offset = float(cfg["z_offset"])

        seg_lens = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
        total_len = float(np.sum(seg_lens))
        if total_len <= 1e-9:
            waypoints[1, :] = waypoints[0, :] + np.array([0.2, 0.0], dtype=float)
            seg_lens = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
            total_len = float(np.sum(seg_lens))

        s_samples = np.linspace(0.0, total_len, n_samples)
        cum = np.concatenate(([0.0], np.cumsum(seg_lens)))
        uv_points = np.zeros((n_samples, 2), dtype=float)
        tangents = np.zeros((n_samples, 2), dtype=float)
        for i, s in enumerate(s_samples):
            idx = int(np.searchsorted(cum, s, side="right") - 1)
            idx = min(max(idx, 0), len(seg_lens) - 1)
            seg_s = s - cum[idx]
            denom = max(seg_lens[idx], 1e-9)
            r = seg_s / denom
            p0 = waypoints[idx]
            p1 = waypoints[idx + 1]
            uv_points[i, :] = p0 + (p1 - p0) * r
            tangents[i, :] = (p1 - p0) / denom

        poses = np.zeros((3, n_samples), dtype=float)
        quats: List[Quaternion] = []
        z_axis = self._plane_normal
        fallback_x = self._plane_u
        for i in range(n_samples):
            p = uv_points[i, :]
            poses[:, i] = self._map_uvz_to_xyz(p[0], p[1], z_offset)
            t = np.array(
                [
                    tangents[i, 0] * self._plane_u[0] + tangents[i, 1] * self._plane_v[0],
                    tangents[i, 0] * self._plane_u[1] + tangents[i, 1] * self._plane_v[1],
                    tangents[i, 0] * self._plane_u[2] + tangents[i, 1] * self._plane_v[2],
                ],
                dtype=float,
            )
            x_axis = self._normalize_vec(t)
            if x_axis is None:
                x_axis = fallback_x
            y_axis = self._normalize_vec(np.cross(z_axis, x_axis))
            if y_axis is None:
                y_axis = self._plane_v
            x_axis = self._normalize_vec(np.cross(y_axis, z_axis))
            if x_axis is None:
                x_axis = fallback_x
            rot = np.column_stack((x_axis, y_axis, z_axis))
            quats.append(self._quat_from_matrix(rot))

        time_axis = np.linspace(0.0, duration_sec, n_samples, dtype=float)
        return poses, quats, time_axis

    def _joint_vector_from_state(self, state: JointState, fallback: np.ndarray) -> np.ndarray:
        out = np.asarray(fallback, dtype=float).reshape(-1).copy()
        name_to_pos = {}
        for i, n in enumerate(state.name):
            if i < len(state.position):
                name_to_pos[str(n)] = float(state.position[i])
        for i, n in enumerate(self.joint_names):
            if n in name_to_pos:
                out[i] = name_to_pos[n]
        return out

    def _solve_ik_pose(self, pose: Pose, seed_q: np.ndarray) -> Optional[np.ndarray]:
        if not self.ik_client.service_is_ready():
            if self._ik_debug_log_count < 5:
                self.get_logger().warn("IK client not ready when solving pose.")
                self._ik_debug_log_count += 1
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.moveit_group_name
        req.ik_request.ik_link_name = self.ik_link_name
        req.ik_request.avoid_collisions = False
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.ee_plane_frame
        req.ik_request.pose_stamped.pose = pose
        req.ik_request.robot_state = RobotState()
        req.ik_request.robot_state.joint_state = JointState()
        req.ik_request.robot_state.joint_state.name = list(self.joint_names)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed_q]
        req.ik_request.timeout = Duration(
            sec=int(self.ik_timeout_sec),
            nanosec=int(max(self.ik_timeout_sec - int(self.ik_timeout_sec), 0.0) * 1e9),
        )
        fut = self.ik_client.call_async(req)
        deadline = time.monotonic() + max(self.ik_timeout_sec * 2.0, 0.1)
        while (not fut.done()) and (time.monotonic() < deadline):
            time.sleep(0.005)
        if not fut.done():
            if self._ik_debug_log_count < 5:
                self.get_logger().warn("IK future timeout before completion.")
                self._ik_debug_log_count += 1
            return None
        try:
            resp = fut.result()
        except Exception:  # pylint: disable=broad-except
            if self._ik_debug_log_count < 5:
                self.get_logger().warn("IK future raised exception.")
                self._ik_debug_log_count += 1
            return None
        if resp is None:
            if self._ik_debug_log_count < 5:
                self.get_logger().warn("IK response is None.")
                self._ik_debug_log_count += 1
            return None
        if int(resp.error_code.val) != int(MoveItErrorCodes.SUCCESS):
            self._ik_last_error_code = int(resp.error_code.val)
            if self._ik_debug_log_count < 5:
                self.get_logger().warn(
                    f"IK failed: code={self._ik_last_error_code}, frame={self.ee_plane_frame}, "
                    f"pos=({pose.position.x:.3f},{pose.position.y:.3f},{pose.position.z:.3f}), "
                    f"quat=({pose.orientation.x:.3f},{pose.orientation.y:.3f},"
                    f"{pose.orientation.z:.3f},{pose.orientation.w:.3f})"
                )
                self._ik_debug_log_count += 1
            return None
        return self._joint_vector_from_state(resp.solution.joint_state, seed_q)

    def _build_joint_nominal_from_ee_path(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if self.ee_plane_frame != "base_link":
            self.get_logger().error(
                f"Only ee_plane_frame=base_link is supported in current implementation, got {self.ee_plane_frame}."
            )
            return None
        if self.ee_orientation_mode != "z_axis_lock":
            self.get_logger().warn(
                f"Unsupported ee_orientation_mode={self.ee_orientation_mode}, fallback to z_axis_lock."
            )
            self.ee_orientation_mode = "z_axis_lock"
        if not self._build_plane_basis():
            return None

        ee_xyz, ee_quats, time_axis = self._build_ee_plane_nominal_path()
        n = ee_xyz.shape[1]
        q_nominal = np.zeros((len(self.joint_names), n), dtype=float)
        if self._has_joint_state:
            seed = self._q_now.copy()
        else:
            seed = np.asarray(self.fmp_core.nominal_intent[:, 0], dtype=float).reshape(-1)

        fail_idx: List[int] = []
        for i in range(n):
            pose = Pose()
            pose.position.x = float(ee_xyz[0, i])
            pose.position.y = float(ee_xyz[1, i])
            pose.position.z = float(ee_xyz[2, i])
            pose.orientation = ee_quats[i]

            solved = None
            for _ in range(max(self.ik_retry_per_point, 1)):
                solved = self._solve_ik_pose(pose, seed)
                if solved is not None:
                    break
            if solved is None:
                fail_idx.append(i)
                q_nominal[:, i] = seed
                continue
            q_nominal[:, i] = solved
            seed = solved

        fail_ratio = float(len(fail_idx)) / float(max(n, 1))
        self._ik_fail_ratio_last = fail_ratio
        self._ik_fail_indices_last = fail_idx
        if fail_idx:
            self.get_logger().warn(
                f"IK failed on {len(fail_idx)}/{n} samples (ratio={fail_ratio:.3f}), "
                f"first_idx={fail_idx[0]}, last_idx={fail_idx[-1]}"
            )
        if fail_ratio > float(self.ik_fail_max_ratio):
            self.get_logger().error(
                f"IK fail ratio {fail_ratio:.3f} exceeds threshold {self.ik_fail_max_ratio:.3f}."
            )
            return None

        self._nominal_ee_points = ee_xyz.copy()
        return q_nominal, time_axis

    def _compute_ee_metrics(
        self,
        modulated_traj: np.ndarray,
    ) -> Dict[str, float]:
        if self._nominal_ee_points is None:
            return {
                "ee_plane_deviation_mean": 0.0,
                "ee_plane_deviation_max": 0.0,
                "obstacle_clearance_min": 0.0,
                "obstacle_clearance_p95": 0.0,
            }

        q = np.asarray(modulated_traj, dtype=float)
        n = int(q.shape[1]) if q.ndim == 2 else 0
        if n <= 0:
            return {
                "ee_plane_deviation_mean": 0.0,
                "ee_plane_deviation_max": 0.0,
                "obstacle_clearance_min": 0.0,
                "obstacle_clearance_p95": 0.0,
            }

        ee_mod = np.zeros((3, n), dtype=float)
        for i in range(n):
            p = self._get_marker_fk_point(q[:, i])
            ee_mod[:, i] = [p.x, p.y, p.z]

        uv_nom = np.zeros((2, n), dtype=float)
        uv_mod = np.zeros((2, n), dtype=float)
        for i in range(n):
            uv_nom[:, i] = self._xyz_to_plane_uv(self._nominal_ee_points[:, i])
            uv_mod[:, i] = self._xyz_to_plane_uv(ee_mod[:, i])
        dev = np.linalg.norm(uv_mod - uv_nom, axis=0)

        clearances: List[float] = []
        for i in range(n):
            uv = uv_mod[:, i]
            min_c = float("inf")
            for obs in self._plane_obstacles:
                d = float(np.linalg.norm(uv - np.array([obs["u"], obs["v"]], dtype=float)) - obs["radius"])
                if d < min_c:
                    min_c = d
            if not np.isfinite(min_c):
                min_c = 0.0
            clearances.append(min_c)
        c_arr = np.asarray(clearances, dtype=float)
        return {
            "ee_plane_deviation_mean": float(np.mean(dev)) if dev.size > 0 else 0.0,
            "ee_plane_deviation_max": float(np.max(dev)) if dev.size > 0 else 0.0,
            "obstacle_clearance_min": float(np.min(c_arr)) if c_arr.size > 0 else 0.0,
            "obstacle_clearance_p95": float(np.percentile(c_arr, 95)) if c_arr.size > 0 else 0.0,
        }

    def _write_benchmark_results(self) -> None:
        result_dir = Path.cwd() / "result"
        result_dir.mkdir(parents=True, exist_ok=True)

        ts_str = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        stem = f"hybrid_2obs_{ts_str}"
        csv_path = result_dir / f"{stem}.csv"
        json_path = result_dir / f"{stem}.json"

        timing_stats = {
            "collision_check_ms": self._stats_from_samples(self._timing_samples["collision_check_ms"]),
            "rrt_plan_ms": self._stats_from_samples(self._timing_samples["rrt_plan_ms"]),
            "fmp_modulate_ms": self._stats_from_samples(self._timing_samples["fmp_modulate_ms"]),
            "execute_trajectory_ms": self._stats_from_samples(self._timing_samples["execute_trajectory_ms"]),
            "orchestrator_total_ms": self._stats_from_samples(self._timing_samples["orchestrator_total_ms"]),
        }
        jerk_stats = self._stats_from_samples(self._trajectory_jerk_proxy_samples)
        rrt_iter_stats = self._stats_from_samples(self._rrt_iter_samples)
        rrt_time_stats = self._stats_from_samples(self._rrt_time_ms_samples)
        rrt_queries_stats = self._stats_from_samples(self._rrt_collision_queries_samples)
        cpp_plan_time_stats = self._stats_from_samples(self._cpp_plan_time_ms_samples)
        cpp_plan_queries_stats = self._stats_from_samples(self._cpp_plan_collision_queries_samples)
        segment_stats = self._stats_from_samples(self._segment_count_samples)
        refine_budget_stats = self._stats_from_samples(self._refine_budget_samples)

        data = {
            "benchmark": "hybrid_2obs",
            "timestamp": ts_str,
            "hybrid_mode": self.hybrid_mode,
            "nominal_source": self.nominal_source,
            "window_sec": float(self._stats_window_sec),
            "counts": {
                "danger_count": int(self._danger_count),
                "danger_event_count": int(self._danger_event_count),
                "nominal_count": int(self._nominal_count),
                "rrt_call_count": int(self._rrt_call_count),
                "rrt_timeout_count": int(self._rrt_timeout_count),
                "orchestrator_phase_b_count": int(self._orchestrator_phase_b_count),
            },
            "rrt": {
                "iter": {
                    "mean": float(rrt_iter_stats["mean"]),
                    "max": float(rrt_iter_stats["max"]),
                    "count": int(rrt_iter_stats["count"]),
                },
                "time_ms": rrt_time_stats,
                "collision_queries": {
                    "mean": float(rrt_queries_stats["mean"]),
                    "max": float(rrt_queries_stats["max"]),
                    "count": int(rrt_queries_stats["count"]),
                },
                "stop_reason_counts": dict(self._rrt_stop_reason_counts),
            },
            "cpp_local_planner": {
                "used": bool(self._cpp_local_planner_used),
                "success_count": int(self._cpp_plan_success_count),
                "failure_count": int(self._cpp_plan_failure_count),
                "time_ms": cpp_plan_time_stats,
                "collision_queries": cpp_plan_queries_stats,
            },
            "postcheck": {
                "passed": bool(self._postcheck_passed),
                "first_invalid_state": int(self._postcheck_first_invalid_state),
                "first_invalid_edge": int(self._postcheck_first_invalid_edge),
                "state_invalid_count": int(self._postcheck_state_invalid_count),
                "edge_invalid_count": int(self._postcheck_edge_invalid_count),
                "elapsed_ms": float(self._postcheck_elapsed_ms),
                "collision_queries": int(self._postcheck_collision_queries),
                "check_edges": True,
            },
            "dispatch_safety": {
                "execution_aborted": bool(self._execution_aborted),
                "dispatch_action_status": int(self._dispatch_action_status),
                "dispatch_error_code": int(self._dispatch_error_code),
                "dispatch_error_string": str(self._dispatch_error_string),
                "trajectory_duration_sec": float(self._trajectory_duration_sec),
                "trajectory_point_count": int(self._trajectory_point_count),
                "trajectory_max_joint_delta": float(self._trajectory_max_joint_delta),
                "trajectory_estimated_max_velocity": float(self._trajectory_estimated_max_velocity),
                "trajectory_estimated_max_acceleration": float(self._trajectory_estimated_max_acceleration),
                "time_scaling_factor": float(self._trajectory_time_scaling_factor),
                "start_state_error_max": float(self._start_state_error_max),
                "start_state_error_norm": float(self._start_state_error_norm),
            },
            "compat": {
                "segment_count": segment_stats,
                "refine_budget": refine_budget_stats,
            },
            "timings_ms": timing_stats,
            "trajectory_jerk_proxy": jerk_stats,
            "decision_counters": dict(self._decision_counters),
            "ee_metrics": dict(self._ee_metrics_last),
            "ik_stats": {
                "fail_ratio": float(self._ik_fail_ratio_last),
                "fail_count": int(len(self._ik_fail_indices_last)),
            },
        }

        csv_row = {
            "benchmark": data["benchmark"],
            "timestamp": data["timestamp"],
            "hybrid_mode": data["hybrid_mode"],
            "window_sec": data["window_sec"],
            "danger_count": data["counts"]["danger_count"],
            "danger_event_count": data["counts"]["danger_event_count"],
            "nominal_count": data["counts"]["nominal_count"],
            "rrt_call_count": data["counts"]["rrt_call_count"],
            "rrt_timeout_count": data["counts"]["rrt_timeout_count"],
            "rrt_iter_mean": data["rrt"]["iter"]["mean"],
            "rrt_iter_max": data["rrt"]["iter"]["max"],
            "rrt_time_ms_mean": data["rrt"]["time_ms"]["mean"],
            "rrt_time_ms_p95": data["rrt"]["time_ms"]["p95"],
            "rrt_time_ms_max": data["rrt"]["time_ms"]["max"],
            "rrt_collision_queries_mean": data["rrt"]["collision_queries"]["mean"],
            "cpp_local_planner_used": int(data["cpp_local_planner"]["used"]),
            "cpp_plan_success_count": data["cpp_local_planner"]["success_count"],
            "cpp_plan_failure_count": data["cpp_local_planner"]["failure_count"],
            "cpp_plan_time_ms_mean": data["cpp_local_planner"]["time_ms"]["mean"],
            "cpp_plan_time_ms_p95": data["cpp_local_planner"]["time_ms"]["p95"],
            "cpp_plan_time_ms_max": data["cpp_local_planner"]["time_ms"]["max"],
            "cpp_plan_collision_queries_mean": data["cpp_local_planner"]["collision_queries"]["mean"],
            "cpp_plan_collision_queries_p95": data["cpp_local_planner"]["collision_queries"]["p95"],
            "cpp_plan_collision_queries_max": data["cpp_local_planner"]["collision_queries"]["max"],
            "postcheck_passed": int(data["postcheck"]["passed"]),
            "postcheck_first_invalid_state": data["postcheck"]["first_invalid_state"],
            "postcheck_first_invalid_edge": data["postcheck"]["first_invalid_edge"],
            "postcheck_state_invalid_count": data["postcheck"]["state_invalid_count"],
            "postcheck_edge_invalid_count": data["postcheck"]["edge_invalid_count"],
            "postcheck_elapsed_ms": data["postcheck"]["elapsed_ms"],
            "postcheck_collision_queries": data["postcheck"]["collision_queries"],
            "execution_aborted": int(data["dispatch_safety"]["execution_aborted"]),
            "dispatch_action_status": data["dispatch_safety"]["dispatch_action_status"],
            "dispatch_error_code": data["dispatch_safety"]["dispatch_error_code"],
            "dispatch_error_string": data["dispatch_safety"]["dispatch_error_string"],
            "trajectory_duration_sec": data["dispatch_safety"]["trajectory_duration_sec"],
            "trajectory_point_count": data["dispatch_safety"]["trajectory_point_count"],
            "trajectory_max_joint_delta": data["dispatch_safety"]["trajectory_max_joint_delta"],
            "trajectory_estimated_max_velocity": data["dispatch_safety"]["trajectory_estimated_max_velocity"],
            "trajectory_estimated_max_acceleration": data["dispatch_safety"]["trajectory_estimated_max_acceleration"],
            "time_scaling_factor": data["dispatch_safety"]["time_scaling_factor"],
            "start_state_error_max": data["dispatch_safety"]["start_state_error_max"],
            "start_state_error_norm": data["dispatch_safety"]["start_state_error_norm"],
            "segment_count_mean": data["compat"]["segment_count"]["mean"],
            "segment_count_p95": data["compat"]["segment_count"]["p95"],
            "refine_budget_mean": data["compat"]["refine_budget"]["mean"],
            "refine_budget_p95": data["compat"]["refine_budget"]["p95"],
            "collision_check_ms_mean": data["timings_ms"]["collision_check_ms"]["mean"],
            "collision_check_ms_p95": data["timings_ms"]["collision_check_ms"]["p95"],
            "collision_check_ms_max": data["timings_ms"]["collision_check_ms"]["max"],
            "rrt_plan_ms_mean": data["timings_ms"]["rrt_plan_ms"]["mean"],
            "rrt_plan_ms_p95": data["timings_ms"]["rrt_plan_ms"]["p95"],
            "rrt_plan_ms_max": data["timings_ms"]["rrt_plan_ms"]["max"],
            "fmp_modulate_ms_mean": data["timings_ms"]["fmp_modulate_ms"]["mean"],
            "fmp_modulate_ms_p95": data["timings_ms"]["fmp_modulate_ms"]["p95"],
            "fmp_modulate_ms_max": data["timings_ms"]["fmp_modulate_ms"]["max"],
            "execute_trajectory_ms_mean": data["timings_ms"]["execute_trajectory_ms"]["mean"],
            "execute_trajectory_ms_p95": data["timings_ms"]["execute_trajectory_ms"]["p95"],
            "execute_trajectory_ms_max": data["timings_ms"]["execute_trajectory_ms"]["max"],
            "orchestrator_total_ms_mean": data["timings_ms"]["orchestrator_total_ms"]["mean"],
            "orchestrator_total_ms_p95": data["timings_ms"]["orchestrator_total_ms"]["p95"],
            "orchestrator_total_ms_max": data["timings_ms"]["orchestrator_total_ms"]["max"],
            "jerk_proxy_mean": data["trajectory_jerk_proxy"]["mean"],
            "jerk_proxy_p95": data["trajectory_jerk_proxy"]["p95"],
            "jerk_proxy_max": data["trajectory_jerk_proxy"]["max"],
            "decision_sent": int(self._decision_counters.get("sent", 0)),
            "decision_preempted": int(self._decision_counters.get("preempted", 0)),
            "decision_skipped_duplicate": int(self._decision_counters.get("skipped_duplicate", 0)),
            "decision_skipped_rate_limit": int(self._decision_counters.get("skipped_rate_limit", 0)),
            "decision_skipped_stale_state": int(self._decision_counters.get("skipped_stale_state", 0)),
            "decision_continued_active_goal": int(self._decision_counters.get("continued_active_goal", 0)),
            "ee_plane_deviation_mean": float(self._ee_metrics_last["ee_plane_deviation_mean"]),
            "ee_plane_deviation_max": float(self._ee_metrics_last["ee_plane_deviation_max"]),
            "obstacle_clearance_min": float(self._ee_metrics_last["obstacle_clearance_min"]),
            "obstacle_clearance_p95": float(self._ee_metrics_last["obstacle_clearance_p95"]),
            "ik_fail_ratio": float(self._ik_fail_ratio_last),
        }

        with csv_path.open("w", encoding="utf-8", newline="") as f_csv:
            writer = csv.DictWriter(f_csv, fieldnames=list(csv_row.keys()))
            writer.writeheader()
            writer.writerow(csv_row)

        with json_path.open("w", encoding="utf-8") as f_json:
            json.dump(data, f_json, ensure_ascii=False, indent=2)

        self.get_logger().info(
            f"Benchmark results written: {csv_path} and {json_path}"
        )

    def __init__(self) -> None:
        super().__init__("intent_hybrid_planner")

        # -------------------------
        # Runtime parameters
        # -------------------------
        self.declare_parameter(
            "trajectory_action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.declare_parameter("min_send_interval", 0.3)
        self.declare_parameter("min_preempt_interval", 0.2)
        self.declare_parameter("trajectory_diff_q_eps", 0.01)
        self.declare_parameter("danger_replan_hysteresis", True)
        self.declare_parameter("danger_hold_sec", 0.5)
        self.declare_parameter("state_stale_timeout", 0.2)
        self.declare_parameter("demo_len", 150)
        self.declare_parameter("demo_dt", 0.1)
        self.declare_parameter("sync_nominal_dt_with_demo_dt", True)
        self.declare_parameter("nominal_dt", 0.1)
        self.declare_parameter("execution_mode", "online")
        self.declare_parameter("offline_start_delay_sec", 1.0)
        self.declare_parameter("nominal_source", "joint")
        self.declare_parameter("ee_plane_frame", "base_link")
        self.declare_parameter("ee_plane_origin_xyz", [0.0, 0.0, 0.25])
        self.declare_parameter("ee_plane_normal_xyz", [0.0, 0.0, 1.0])
        self.declare_parameter("ee_orientation_mode", "z_axis_lock")
        self.declare_parameter("ee_path_json", "")
        self.declare_parameter("plane_obstacles_json", "")
        self.declare_parameter("obstacle_config_file", "")
        self.declare_parameter("obstacle_config_apply_to_planning_scene", True)
        self.declare_parameter("obstacle_config_apply_timeout_sec", 5.0)
        self.declare_parameter("ik_fail_max_ratio", 0.1)
        self.declare_parameter("ik_retry_per_point", 2)
        self.declare_parameter("ik_timeout_sec", 0.05)
        self.declare_parameter("ik_link_name", "tool0")
        self.declare_parameter("runtime_backend", "cpp_bridge")
        self.declare_parameter("cpp_bridge_service_ns", "/intent_runtime")
        self.declare_parameter("cpp_bridge_timeout_sec", 0.2)
        self.declare_parameter("cpp_bridge_collision_required", True)
        self.declare_parameter("fk_vis_ee_link", "tool0")
        self.declare_parameter("use_cpp_local_planner", True)
        self.declare_parameter("allow_cpp_local_planner_fallback", True)
        self.declare_parameter("cpp_local_planner_service", "/intent_runtime/plan_local_segment")
        self.declare_parameter("cpp_motion_check_service", "/intent_runtime/check_motion_batch")
        self.declare_parameter("planner_type", "rrt_connect")
        self.declare_parameter("ompl_simplify_enable", False)
        self.declare_parameter("ompl_simplify_timeout_sec", 0.05)
        self.declare_parameter("cpp_planner_timeout_sec", 0.10)
        self.declare_parameter("cpp_planner_max_iter", 500)
        self.declare_parameter("cpp_planner_step_size", 0.15)
        self.declare_parameter("cpp_planner_goal_tolerance", 0.08)
        self.declare_parameter("cpp_edge_resolution", 0.02)
        self.declare_parameter("postcheck_edge_resolution", 0.02)
        self.declare_parameter("execute_only_if_postcheck_passed", True)
        self.declare_parameter("postcheck_check_edges", True)
        self.declare_parameter("offline_export_plot_enable", True)
        self.declare_parameter("offline_export_plot_dir", str(Path.cwd() / "png"))
        self.declare_parameter("offline_export_eval_input_enable", True)
        self.declare_parameter("offline_export_eval_input_dir", str(Path.cwd() / "evaluation_inputs"))
        self.declare_parameter("joint_names", list(self.DEFAULT_JOINT_NAMES))
        self.declare_parameter("hybrid_mode", "legacy")
        self.declare_parameter("segment_gap", 10)
        self.declare_parameter("segment_pad", 4)
        self.declare_parameter("via_interp_dist", 0.5)
        self.declare_parameter("via_densify_enable", True)
        self.declare_parameter("via_trim_sec", 0.05)
        self.declare_parameter("via_global_dedup_enable", True)
        self.declare_parameter("refine_fixed_budget", 100)
        self.declare_parameter("refine_budget_candidates", [25, 50, 100, 150, 200])
        self.declare_parameter("rrt_step_size", 0.1)
        self.declare_parameter("rrt_neighbor_radius", 0.9)
        self.declare_parameter("rrt_sigma_intent", 0.0)
        self.declare_parameter("rrt_edge_sample_step", 0.0)
        self.declare_parameter("adaptive_scale_enable", True)
        self.declare_parameter("adaptive_interp_step_ratio", 0.3333333333333333)
        self.declare_parameter("adaptive_neighbor_step_ratio", 2.6666666666666665)
        self.declare_parameter("adaptive_sigma_step_ratio", 0.8)
        self.declare_parameter("adaptive_edge_step_ratio", 0.3333333333333333)
        self.declare_parameter("rrt_rng_seed", 42)
        self.declare_parameter("rrt_max_iter", 300)
        self.declare_parameter("rrt_timeout_sec", 0.25)
        self.declare_parameter("rrt_max_edge_samples", 8)
        self.declare_parameter("offline_rrt_min_iter", 3000)
        self.declare_parameter("offline_rrt_min_timeout_sec", 1.5)
        self.declare_parameter("offline_rrt_min_edge_samples", 10)
        self.declare_parameter("rrt_collision_backend", "moveit_py")
        self.declare_parameter("moveit_group_name", "ur_manipulator")
        self.declare_parameter("moveit_py_strict", False)
        self.declare_parameter("analytic_obstacles_json", "")
        self.declare_parameter("time_param_backend", "finite_diff")
        self.declare_parameter("jerk_warn_threshold", 80.0)
        self.declare_parameter("action_path_tolerance_rad", 0.0)
        self.declare_parameter("action_goal_tolerance_rad", 0.0)
        self.declare_parameter("action_goal_time_tolerance_sec", 0.0)
        self.declare_parameter("offline_postcheck_collision_enable", True)
        self.declare_parameter("offline_postcheck_max_colliding_points", 0)
        self.declare_parameter("offline_postcheck_repair_with_via", True)
        self.declare_parameter("offline_postcheck_repair_margin_points", 2)
        self.declare_parameter("offline_wait_action_result", True)
        self.declare_parameter("offline_action_result_timeout_sec", 20.0)
        self.declare_parameter("offline_allow_stale_state_stitch", True)
        self.declare_parameter("offline_stitch_start_from_current", False)
        self.declare_parameter("start_state_tolerance", 0.03)
        self.declare_parameter("joint_limit_margin", 0.05)
        self.declare_parameter("velocity_scale", 0.15)
        self.declare_parameter("acceleration_scale", 0.15)
        self.declare_parameter("enable_conservative_time_scaling", True)
        self.declare_parameter("max_time_scaling_factor", 5.0)
        self.declare_parameter("minimum_dt", 0.05)
        self.declare_parameter(
            "joint_position_lower_limit",
            [-2.0 * np.pi] * len(self.DEFAULT_JOINT_NAMES),
        )
        self.declare_parameter(
            "joint_position_upper_limit",
            [2.0 * np.pi] * len(self.DEFAULT_JOINT_NAMES),
        )
        self.declare_parameter("joint_limits_file", "")
        self.declare_parameter("max_joint_velocity", [1.0] * len(self.DEFAULT_JOINT_NAMES))
        self.declare_parameter("max_joint_acceleration", [2.0] * len(self.DEFAULT_JOINT_NAMES))

        self.trajectory_action_name = (
            self.get_parameter("trajectory_action_name").get_parameter_value().string_value
        )
        self.min_send_interval = (
            self.get_parameter("min_send_interval").get_parameter_value().double_value
        )
        self.min_preempt_interval = (
            self.get_parameter("min_preempt_interval").get_parameter_value().double_value
        )
        self.trajectory_diff_q_eps = (
            self.get_parameter("trajectory_diff_q_eps").get_parameter_value().double_value
        )
        self.danger_replan_hysteresis = (
            self.get_parameter("danger_replan_hysteresis").get_parameter_value().bool_value
        )
        self.danger_hold_sec = self.get_parameter("danger_hold_sec").get_parameter_value().double_value
        self.state_stale_timeout = (
            self.get_parameter("state_stale_timeout").get_parameter_value().double_value
        )
        self.demo_len = int(self.get_parameter("demo_len").get_parameter_value().integer_value)
        self.demo_dt = float(self.get_parameter("demo_dt").get_parameter_value().double_value)
        self.sync_nominal_dt_with_demo_dt = (
            self.get_parameter("sync_nominal_dt_with_demo_dt").get_parameter_value().bool_value
        )
        self.nominal_dt = self.get_parameter("nominal_dt").get_parameter_value().double_value
        self.execution_mode = (
            self.get_parameter("execution_mode").get_parameter_value().string_value
        )
        self.offline_start_delay_sec = (
            self.get_parameter("offline_start_delay_sec").get_parameter_value().double_value
        )
        self.nominal_source = (
            self.get_parameter("nominal_source").get_parameter_value().string_value
        )
        self.ee_plane_frame = (
            self.get_parameter("ee_plane_frame").get_parameter_value().string_value
        )
        self.ee_plane_origin = [
            float(v)
            for v in self.get_parameter("ee_plane_origin_xyz").get_parameter_value().double_array_value
        ]
        self.ee_plane_normal = [
            float(v)
            for v in self.get_parameter("ee_plane_normal_xyz").get_parameter_value().double_array_value
        ]
        self.ee_orientation_mode = (
            self.get_parameter("ee_orientation_mode").get_parameter_value().string_value
        )
        self.ee_path_json = (
            self.get_parameter("ee_path_json").get_parameter_value().string_value
        )
        self.plane_obstacles_json = (
            self.get_parameter("plane_obstacles_json").get_parameter_value().string_value
        )
        self.obstacle_config_file = (
            self.get_parameter("obstacle_config_file").get_parameter_value().string_value
        )
        self.obstacle_config_apply_to_planning_scene = (
            self.get_parameter("obstacle_config_apply_to_planning_scene").get_parameter_value().bool_value
        )
        self.obstacle_config_apply_timeout_sec = float(
            self.get_parameter("obstacle_config_apply_timeout_sec").get_parameter_value().double_value
        )
        self.ik_fail_max_ratio = float(
            self.get_parameter("ik_fail_max_ratio").get_parameter_value().double_value
        )
        self.ik_retry_per_point = int(
            self.get_parameter("ik_retry_per_point").get_parameter_value().integer_value
        )
        self.ik_timeout_sec = float(
            self.get_parameter("ik_timeout_sec").get_parameter_value().double_value
        )
        self.ik_link_name = (
            self.get_parameter("ik_link_name").get_parameter_value().string_value
        )
        self.runtime_backend = (
            self.get_parameter("runtime_backend").get_parameter_value().string_value
        )
        self.cpp_bridge_service_ns = (
            self.get_parameter("cpp_bridge_service_ns").get_parameter_value().string_value
        )
        self.cpp_bridge_timeout_sec = float(
            self.get_parameter("cpp_bridge_timeout_sec").get_parameter_value().double_value
        )
        self.cpp_bridge_collision_required = (
            self.get_parameter("cpp_bridge_collision_required").get_parameter_value().bool_value
        )
        self.fk_vis_ee_link = (
            self.get_parameter("fk_vis_ee_link").get_parameter_value().string_value
        )
        self.use_cpp_local_planner = (
            self.get_parameter("use_cpp_local_planner").get_parameter_value().bool_value
        )
        self.allow_cpp_local_planner_fallback = (
            self.get_parameter("allow_cpp_local_planner_fallback").get_parameter_value().bool_value
        )
        self.cpp_local_planner_service = (
            self.get_parameter("cpp_local_planner_service").get_parameter_value().string_value
        )
        self.cpp_motion_check_service = (
            self.get_parameter("cpp_motion_check_service").get_parameter_value().string_value
        )
        self.planner_type = (
            self.get_parameter("planner_type").get_parameter_value().string_value
        )
        self.ompl_simplify_enable = bool(
            self.get_parameter("ompl_simplify_enable").get_parameter_value().bool_value
        )
        self.ompl_simplify_timeout_sec = float(
            self.get_parameter("ompl_simplify_timeout_sec").get_parameter_value().double_value
        )
        self.cpp_planner_timeout_sec = float(
            self.get_parameter("cpp_planner_timeout_sec").get_parameter_value().double_value
        )
        self.cpp_planner_max_iter = int(
            self.get_parameter("cpp_planner_max_iter").get_parameter_value().integer_value
        )
        self.cpp_planner_step_size = float(
            self.get_parameter("cpp_planner_step_size").get_parameter_value().double_value
        )
        self.cpp_planner_goal_tolerance = float(
            self.get_parameter("cpp_planner_goal_tolerance").get_parameter_value().double_value
        )
        self.cpp_edge_resolution = float(
            self.get_parameter("cpp_edge_resolution").get_parameter_value().double_value
        )
        self.postcheck_edge_resolution = float(
            self.get_parameter("postcheck_edge_resolution").get_parameter_value().double_value
        )
        self.execute_only_if_postcheck_passed = (
            self.get_parameter("execute_only_if_postcheck_passed").get_parameter_value().bool_value
        )
        self.postcheck_check_edges = (
            self.get_parameter("postcheck_check_edges").get_parameter_value().bool_value
        )
        self.offline_export_plot_enable = (
            self.get_parameter("offline_export_plot_enable").get_parameter_value().bool_value
        )
        self.offline_export_plot_dir = (
            self.get_parameter("offline_export_plot_dir").get_parameter_value().string_value
        )
        self.offline_export_eval_input_enable = (
            self.get_parameter("offline_export_eval_input_enable").get_parameter_value().bool_value
        )
        self.offline_export_eval_input_dir = (
            self.get_parameter("offline_export_eval_input_dir").get_parameter_value().string_value
        )
        self.joint_names = [
            str(v)
            for v in self.get_parameter("joint_names").get_parameter_value().string_array_value
        ]
        if not self.joint_names:
            self.joint_names = list(self.DEFAULT_JOINT_NAMES)
        self.hybrid_mode = self.get_parameter("hybrid_mode").get_parameter_value().string_value
        self.segment_gap = int(self.get_parameter("segment_gap").get_parameter_value().integer_value)
        self.segment_pad = int(self.get_parameter("segment_pad").get_parameter_value().integer_value)
        self.via_interp_dist = float(
            self.get_parameter("via_interp_dist").get_parameter_value().double_value
        )
        self.via_densify_enable = bool(
            self.get_parameter("via_densify_enable").get_parameter_value().bool_value
        )
        self.via_trim_sec = float(self.get_parameter("via_trim_sec").get_parameter_value().double_value)
        self.via_global_dedup_enable = (
            self.get_parameter("via_global_dedup_enable").get_parameter_value().bool_value
        )
        self.refine_fixed_budget = int(
            self.get_parameter("refine_fixed_budget").get_parameter_value().integer_value
        )
        self.refine_budget_candidates = [
            int(v)
            for v in self.get_parameter("refine_budget_candidates")
            .get_parameter_value()
            .integer_array_value
        ]
        self.rrt_step_size = float(
            self.get_parameter("rrt_step_size").get_parameter_value().double_value
        )
        self.rrt_neighbor_radius = float(
            self.get_parameter("rrt_neighbor_radius").get_parameter_value().double_value
        )
        self.rrt_sigma_intent = float(
            self.get_parameter("rrt_sigma_intent").get_parameter_value().double_value
        )
        self.rrt_edge_sample_step = float(
            self.get_parameter("rrt_edge_sample_step").get_parameter_value().double_value
        )
        self.adaptive_scale_enable = (
            self.get_parameter("adaptive_scale_enable").get_parameter_value().bool_value
        )
        self.adaptive_interp_step_ratio = float(
            self.get_parameter("adaptive_interp_step_ratio").get_parameter_value().double_value
        )
        self.adaptive_neighbor_step_ratio = float(
            self.get_parameter("adaptive_neighbor_step_ratio").get_parameter_value().double_value
        )
        self.adaptive_sigma_step_ratio = float(
            self.get_parameter("adaptive_sigma_step_ratio").get_parameter_value().double_value
        )
        self.adaptive_edge_step_ratio = float(
            self.get_parameter("adaptive_edge_step_ratio").get_parameter_value().double_value
        )
        self.rrt_rng_seed = int(
            self.get_parameter("rrt_rng_seed").get_parameter_value().integer_value
        )
        self.rrt_max_iter = int(
            self.get_parameter("rrt_max_iter").get_parameter_value().integer_value
        )
        self.rrt_timeout_sec = float(
            self.get_parameter("rrt_timeout_sec").get_parameter_value().double_value
        )
        self.rrt_max_edge_samples = int(
            self.get_parameter("rrt_max_edge_samples").get_parameter_value().integer_value
        )
        self.offline_rrt_min_iter = int(
            self.get_parameter("offline_rrt_min_iter").get_parameter_value().integer_value
        )
        self.offline_rrt_min_timeout_sec = float(
            self.get_parameter("offline_rrt_min_timeout_sec").get_parameter_value().double_value
        )
        self.offline_rrt_min_edge_samples = int(
            self.get_parameter("offline_rrt_min_edge_samples").get_parameter_value().integer_value
        )
        self.rrt_collision_backend = (
            self.get_parameter("rrt_collision_backend").get_parameter_value().string_value
        )
        self.moveit_group_name = (
            self.get_parameter("moveit_group_name").get_parameter_value().string_value
        )
        self.moveit_py_strict = (
            self.get_parameter("moveit_py_strict").get_parameter_value().bool_value
        )
        self.analytic_obstacles_json = (
            self.get_parameter("analytic_obstacles_json").get_parameter_value().string_value
        )
        self.time_param_backend = (
            self.get_parameter("time_param_backend").get_parameter_value().string_value
        )
        self.jerk_warn_threshold = (
            self.get_parameter("jerk_warn_threshold").get_parameter_value().double_value
        )
        self.action_path_tolerance_rad = float(
            self.get_parameter("action_path_tolerance_rad").get_parameter_value().double_value
        )
        self.action_goal_tolerance_rad = float(
            self.get_parameter("action_goal_tolerance_rad").get_parameter_value().double_value
        )
        self.action_goal_time_tolerance_sec = float(
            self.get_parameter("action_goal_time_tolerance_sec").get_parameter_value().double_value
        )
        self.offline_postcheck_collision_enable = (
            self.get_parameter("offline_postcheck_collision_enable").get_parameter_value().bool_value
        )
        self.offline_postcheck_max_colliding_points = int(
            self.get_parameter("offline_postcheck_max_colliding_points").get_parameter_value().integer_value
        )
        self.offline_postcheck_repair_with_via = (
            self.get_parameter("offline_postcheck_repair_with_via").get_parameter_value().bool_value
        )
        self.offline_postcheck_repair_margin_points = int(
            self.get_parameter("offline_postcheck_repair_margin_points").get_parameter_value().integer_value
        )
        self.offline_wait_action_result = (
            self.get_parameter("offline_wait_action_result").get_parameter_value().bool_value
        )
        self.offline_action_result_timeout_sec = float(
            self.get_parameter("offline_action_result_timeout_sec").get_parameter_value().double_value
        )
        self.offline_allow_stale_state_stitch = (
            self.get_parameter("offline_allow_stale_state_stitch").get_parameter_value().bool_value
        )
        self.offline_stitch_start_from_current = (
            self.get_parameter("offline_stitch_start_from_current").get_parameter_value().bool_value
        )
        self.start_state_tolerance = float(
            self.get_parameter("start_state_tolerance").get_parameter_value().double_value
        )
        self.joint_limit_margin = float(
            self.get_parameter("joint_limit_margin").get_parameter_value().double_value
        )
        self.velocity_scale = float(
            self.get_parameter("velocity_scale").get_parameter_value().double_value
        )
        self.acceleration_scale = float(
            self.get_parameter("acceleration_scale").get_parameter_value().double_value
        )
        self.enable_conservative_time_scaling = (
            self.get_parameter("enable_conservative_time_scaling").get_parameter_value().bool_value
        )
        self.max_time_scaling_factor = float(
            self.get_parameter("max_time_scaling_factor").get_parameter_value().double_value
        )
        self.minimum_dt = float(
            self.get_parameter("minimum_dt").get_parameter_value().double_value
        )
        self.joint_position_lower_limit = np.asarray(
            self.get_parameter("joint_position_lower_limit").get_parameter_value().double_array_value,
            dtype=float,
        )
        self.joint_position_upper_limit = np.asarray(
            self.get_parameter("joint_position_upper_limit").get_parameter_value().double_array_value,
            dtype=float,
        )
        self.joint_limits_file = (
            self.get_parameter("joint_limits_file").get_parameter_value().string_value
        )
        if len(self.joint_names) != len(self.DEFAULT_JOINT_NAMES):
            self.get_logger().warn(
                f"joint_names expects {len(self.DEFAULT_JOINT_NAMES)} joints in current implementation; "
                "fallback to default UR-style joint names."
            )
            self.joint_names = list(self.DEFAULT_JOINT_NAMES)
        if self.hybrid_mode not in ("legacy", "matlab_compat"):
            self.get_logger().warn(
                f"Unsupported hybrid_mode={self.hybrid_mode}, fallback to legacy."
            )
            self.hybrid_mode = "legacy"
        if self.execution_mode not in ("online", "offline"):
            self.get_logger().warn(
                f"Unsupported execution_mode={self.execution_mode}, fallback to online."
            )
            self.execution_mode = "online"
        if self.runtime_backend not in ("python", "cpp_bridge"):
            self.get_logger().warn(
                f"Unsupported runtime_backend={self.runtime_backend}, fallback to python."
            )
            self.runtime_backend = "python"
        self.cpp_bridge_collision_required = bool(self.cpp_bridge_collision_required)
        self.use_cpp_local_planner = bool(self.use_cpp_local_planner)
        self.allow_cpp_local_planner_fallback = bool(self.allow_cpp_local_planner_fallback)
        self.cpp_local_planner_service = self.cpp_local_planner_service.strip() or "/intent_runtime/plan_local_segment"
        self.cpp_motion_check_service = self.cpp_motion_check_service.strip() or "/intent_runtime/check_motion_batch"
        if self.planner_type not in ("rrt_connect", "ompl_rrt_connect"):
            self.get_logger().warn(
                f"Unsupported planner_type={self.planner_type}, fallback to rrt_connect."
            )
            self.planner_type = "rrt_connect"
        self.ompl_simplify_enable = bool(self.ompl_simplify_enable)
        self.ompl_simplify_timeout_sec = max(float(self.ompl_simplify_timeout_sec), 0.0)
        self.cpp_planner_timeout_sec = max(float(self.cpp_planner_timeout_sec), 0.0)
        self.cpp_planner_max_iter = max(int(self.cpp_planner_max_iter), 1)
        self.cpp_planner_step_size = max(float(self.cpp_planner_step_size), 1e-4)
        self.cpp_planner_goal_tolerance = max(float(self.cpp_planner_goal_tolerance), 1e-4)
        self.cpp_edge_resolution = max(float(self.cpp_edge_resolution), 1e-5)
        self.postcheck_edge_resolution = max(float(self.postcheck_edge_resolution), 1e-5)
        self.execute_only_if_postcheck_passed = bool(self.execute_only_if_postcheck_passed)
        self.postcheck_check_edges = bool(self.postcheck_check_edges)
        if self.nominal_source not in ("joint", "ee_plane"):
            self.get_logger().warn(
                f"Unsupported nominal_source={self.nominal_source}, fallback to joint."
            )
            self.nominal_source = "joint"
        if self.ee_orientation_mode not in ("z_axis_lock",):
            self.get_logger().warn(
                f"Unsupported ee_orientation_mode={self.ee_orientation_mode}, fallback to z_axis_lock."
            )
            self.ee_orientation_mode = "z_axis_lock"
        self.demo_len = max(int(self.demo_len), 2)
        self.demo_dt = max(float(self.demo_dt), 1e-3)
        self.sync_nominal_dt_with_demo_dt = bool(self.sync_nominal_dt_with_demo_dt)
        if self.sync_nominal_dt_with_demo_dt:
            self.nominal_dt = float(self.demo_dt)
        self.obstacle_config_file = str(Path(self.obstacle_config_file).expanduser()) if self.obstacle_config_file else ""
        self.obstacle_config_apply_to_planning_scene = bool(self.obstacle_config_apply_to_planning_scene)
        self.obstacle_config_apply_timeout_sec = max(float(self.obstacle_config_apply_timeout_sec), 0.1)
        self.ik_fail_max_ratio = float(np.clip(self.ik_fail_max_ratio, 0.0, 1.0))
        self.ik_retry_per_point = int(max(self.ik_retry_per_point, 1))
        self.ik_timeout_sec = float(max(self.ik_timeout_sec, 0.01))
        self.offline_start_delay_sec = max(float(self.offline_start_delay_sec), 0.0)
        self.via_global_dedup_enable = bool(self.via_global_dedup_enable)
        self.via_densify_enable = bool(self.via_densify_enable)
        self.action_path_tolerance_rad = max(float(self.action_path_tolerance_rad), 0.0)
        self.action_goal_tolerance_rad = max(float(self.action_goal_tolerance_rad), 0.0)
        self.action_goal_time_tolerance_sec = max(float(self.action_goal_time_tolerance_sec), 0.0)
        self.offline_postcheck_collision_enable = bool(self.offline_postcheck_collision_enable)
        self.offline_postcheck_max_colliding_points = max(
            int(self.offline_postcheck_max_colliding_points), 0
        )
        self.offline_postcheck_repair_with_via = bool(self.offline_postcheck_repair_with_via)
        self.offline_postcheck_repair_margin_points = max(
            int(self.offline_postcheck_repair_margin_points), 0
        )
        self.offline_wait_action_result = bool(self.offline_wait_action_result)
        self.offline_action_result_timeout_sec = max(
            float(self.offline_action_result_timeout_sec), 0.1
        )
        self.offline_allow_stale_state_stitch = bool(self.offline_allow_stale_state_stitch)
        self.offline_stitch_start_from_current = bool(self.offline_stitch_start_from_current)
        self.start_state_tolerance = max(float(self.start_state_tolerance), 1e-6)
        self.joint_limit_margin = max(float(self.joint_limit_margin), 0.0)
        self.velocity_scale = max(float(self.velocity_scale), 1e-6)
        self.acceleration_scale = max(float(self.acceleration_scale), 1e-6)
        self.enable_conservative_time_scaling = bool(self.enable_conservative_time_scaling)
        self.max_time_scaling_factor = max(float(self.max_time_scaling_factor), 1.0)
        self.minimum_dt = max(float(self.minimum_dt), 1e-6)
        if self.joint_position_lower_limit.size != len(self.joint_names):
            self.joint_position_lower_limit = np.ones(len(self.joint_names), dtype=float) * (-2.0 * np.pi)
        if self.joint_position_upper_limit.size != len(self.joint_names):
            self.joint_position_upper_limit = np.ones(len(self.joint_names), dtype=float) * (2.0 * np.pi)
        self.joint_position_upper_limit = np.maximum(
            self.joint_position_upper_limit,
            self.joint_position_lower_limit + 1e-3,
        )
        self.offline_export_plot_dir = self.offline_export_plot_dir.strip()
        if not self.offline_export_plot_dir:
            self.offline_export_plot_dir = str(Path.cwd() / "png")
        self.offline_export_eval_input_enable = bool(self.offline_export_eval_input_enable)
        self.offline_export_eval_input_dir = self.offline_export_eval_input_dir.strip()
        if not self.offline_export_eval_input_dir:
            self.offline_export_eval_input_dir = str(Path.cwd() / "evaluation_inputs")
        if not self.refine_budget_candidates:
            self.refine_budget_candidates = [25, 50, 100, 150, 200]
        self.rrt_step_size = max(float(self.rrt_step_size), 1e-4)
        self.rrt_neighbor_radius = max(float(self.rrt_neighbor_radius), self.rrt_step_size)
        self.rrt_sigma_intent = float(self.rrt_sigma_intent)
        self.rrt_edge_sample_step = float(self.rrt_edge_sample_step)
        self.adaptive_scale_enable = bool(self.adaptive_scale_enable)
        self.adaptive_interp_step_ratio = max(float(self.adaptive_interp_step_ratio), 1e-6)
        self.adaptive_neighbor_step_ratio = max(float(self.adaptive_neighbor_step_ratio), 1.0)
        self.adaptive_sigma_step_ratio = max(float(self.adaptive_sigma_step_ratio), 0.0)
        self.adaptive_edge_step_ratio = max(float(self.adaptive_edge_step_ratio), 1e-6)

        if self.adaptive_scale_enable:
            self.via_interp_dist = max(
                float(self.rrt_step_size * self.adaptive_interp_step_ratio),
                1e-6,
            )
            self.rrt_neighbor_radius = max(
                float(self.rrt_step_size * self.adaptive_neighbor_step_ratio),
                self.rrt_step_size,
            )
            if self.rrt_sigma_intent <= 0.0:
                self.rrt_sigma_intent = float(self.rrt_step_size * self.adaptive_sigma_step_ratio)
            if self.rrt_edge_sample_step <= 0.0:
                self.rrt_edge_sample_step = max(
                    float(self.rrt_step_size * self.adaptive_edge_step_ratio),
                    1e-6,
                )
        else:
            if self.rrt_sigma_intent <= 0.0:
                self.rrt_sigma_intent = 0.08
            if self.rrt_edge_sample_step <= 0.0:
                self.rrt_edge_sample_step = max(float(self.rrt_step_size / 3.0), 1e-6)

        self.rrt_sigma_intent = max(float(self.rrt_sigma_intent), 0.0)
        self.rrt_edge_sample_step = max(float(self.rrt_edge_sample_step), 1e-6)
        if int(self.rrt_rng_seed) < 0:
            self.rrt_rng_seed = None
        else:
            self.rrt_rng_seed = int(self.rrt_rng_seed)
        self.rrt_max_iter = max(int(self.rrt_max_iter), 1)
        self.rrt_timeout_sec = float(self.rrt_timeout_sec)
        if self.rrt_timeout_sec < 0.0:
            self.rrt_timeout_sec = 0.0
        self.rrt_max_edge_samples = max(int(self.rrt_max_edge_samples), 2)
        self.offline_rrt_min_iter = max(int(self.offline_rrt_min_iter), 1)
        self.offline_rrt_min_timeout_sec = float(self.offline_rrt_min_timeout_sec)
        if self.offline_rrt_min_timeout_sec < 0.0:
            self.offline_rrt_min_timeout_sec = 0.0
        self.offline_rrt_min_edge_samples = max(int(self.offline_rrt_min_edge_samples), 2)
        self.rrt_step_size_effective = float(self.rrt_step_size)
        self.rrt_neighbor_radius_effective = float(self.rrt_neighbor_radius)
        self.rrt_max_iter_effective = int(self.rrt_max_iter)
        self.rrt_timeout_sec_effective = float(self.rrt_timeout_sec)
        self.rrt_max_edge_samples_effective = int(self.rrt_max_edge_samples)
        self.rrt_sigma_intent_effective = float(self.rrt_sigma_intent)
        self.rrt_edge_sample_step_effective = float(self.rrt_edge_sample_step)
        if self.execution_mode == "offline":
            self.rrt_max_iter_effective = max(
                self.rrt_max_iter_effective,
                self.offline_rrt_min_iter,
            )
            if self.rrt_timeout_sec_effective > 0.0 and self.offline_rrt_min_timeout_sec > 0.0:
                self.rrt_timeout_sec_effective = max(
                    self.rrt_timeout_sec_effective,
                    self.offline_rrt_min_timeout_sec,
                )
            else:
                # 0 means disable wall-time cutoff for offline demo/debug runs.
                self.rrt_timeout_sec_effective = 0.0
            self.rrt_max_edge_samples_effective = max(
                self.rrt_max_edge_samples_effective,
                self.offline_rrt_min_edge_samples,
            )
        if self.rrt_collision_backend not in ("moveit_py", "analytic"):
            self.get_logger().warn(
                f"Unsupported rrt_collision_backend={self.rrt_collision_backend}, fallback to analytic."
            )
            self.rrt_collision_backend = "analytic"
        if self.time_param_backend not in ("finite_diff", "ruckig"):
            self.get_logger().warn(
                f"Unsupported time_param_backend={self.time_param_backend}, fallback to finite_diff."
            )
            self.time_param_backend = "finite_diff"
        self._ruckig_warned = False
        self._jerk_warned = False
        self._plot_export_warned = False
        self._joint_bounds_warned = False

        self.vel_limits, self.acc_limits = self._load_joint_limits()

        # Internal modules in a unified process (no extra ROS nodes).
        self.scene_monitor = MoveItSceneMonitor(
            node=self,
            joint_names=self.joint_names,
            group_name=self.moveit_group_name,
        )
        self.fmp_core = FMPCore(self)
        self._vis_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.vis_pub = self.create_publisher(MarkerArray, "/planning_vis", self._vis_qos)
        self.collision_object_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self._ik_cb_group = ReentrantCallbackGroup()
        self.ik_client = self.create_client(
            GetPositionIK,
            "/compute_ik",
            callback_group=self._ik_cb_group,
        )
        self.fk_client = self.create_client(
            GetPositionFK,
            "/compute_fk",
            callback_group=self._ik_cb_group,
        )
        self._cpp_runtime_clients_ready = False
        self._runtime_cb_group = ReentrantCallbackGroup()
        self._cpp_check_states_service_name = ""
        self._cpp_motion_check_service_name = ""
        self._cpp_dispatch_service_name = ""
        self._cpp_local_planner_service_name = ""
        self._cpp_publish_markers_service_name = ""
        self.cpp_check_states_client = None
        self.cpp_motion_check_client = None
        self.cpp_dispatch_client = None
        self.cpp_local_planner_client = None
        self.cpp_publish_markers_client = None
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
            callback_group=self._runtime_cb_group,
        )
        self._cpp_rrt_fallback_checker: Optional[Callable[[np.ndarray], bool]] = None
        self._cpp_rrt_fallback_edge_checker = None
        self._cpp_rrt_fallback_warned = False
        self._cpp_rrt_runtime_failed = False
        self._cpp_rrt_runtime_error = ""
        self._last_cpp_collision_error = ""
        if (
            CheckMotionBatch is not None
            and CheckStatesBatch is not None
            and DispatchJointTrajectory is not None
            and PlanLocalSegment is not None
            and PublishPlanningMarkers is not None
        ):
            ns = (self.cpp_bridge_service_ns or "/intent_runtime").rstrip("/")
            if not ns:
                ns = "/intent_runtime"
            self._cpp_check_states_service_name = f"{ns}/check_states_batch"
            self._cpp_motion_check_service_name = self.cpp_motion_check_service
            self._cpp_dispatch_service_name = f"{ns}/dispatch_joint_trajectory"
            self._cpp_local_planner_service_name = self.cpp_local_planner_service
            self._cpp_publish_markers_service_name = f"{ns}/publish_planning_markers"
            self.cpp_check_states_client = self.create_client(
                CheckStatesBatch,
                self._cpp_check_states_service_name,
                callback_group=self._runtime_cb_group,
            )
            self.cpp_motion_check_client = self.create_client(
                CheckMotionBatch,
                self._cpp_motion_check_service_name,
                callback_group=self._runtime_cb_group,
            )
            self.cpp_dispatch_client = self.create_client(
                DispatchJointTrajectory,
                self._cpp_dispatch_service_name,
                callback_group=self._runtime_cb_group,
            )
            self.cpp_local_planner_client = self.create_client(
                PlanLocalSegment,
                self._cpp_local_planner_service_name,
                callback_group=self._runtime_cb_group,
            )
            self.cpp_publish_markers_client = self.create_client(
                PublishPlanningMarkers,
                self._cpp_publish_markers_service_name,
                callback_group=self._runtime_cb_group,
            )
            self._cpp_runtime_clients_ready = True
        elif self.runtime_backend == "cpp_bridge":
            self.get_logger().warn(
                "runtime_backend=cpp_bridge requested but intent_hybrid_interfaces is unavailable. "
                "Fallback to python runtime."
            )
            self.runtime_backend = "python"
        self._fk_service_warn_count = 0
        self._fk_timeout_sec = 0.2

        self._plane_origin = np.zeros(3, dtype=float)
        self._plane_normal = np.array([0.0, 0.0, 1.0], dtype=float)
        self._plane_u = np.array([1.0, 0.0, 0.0], dtype=float)
        self._plane_v = np.array([0.0, 1.0, 0.0], dtype=float)
        if not self._build_plane_basis():
            self.get_logger().warn("Fallback to default base_link XY plane for ee_plane definition.")
        self._plane_obstacles = self._parse_plane_obstacles_param()
        self._nominal_ee_points: Optional[np.ndarray] = None
        self._ik_fail_ratio_last = 0.0
        self._ik_fail_indices_last: List[int] = []
        self._ik_last_error_code = 0
        self._ik_debug_log_count = 0
        self._ee_metrics_last = {
            "ee_plane_deviation_mean": 0.0,
            "ee_plane_deviation_max": 0.0,
            "obstacle_clearance_min": 0.0,
            "obstacle_clearance_p95": 0.0,
        }

        self._analytic_obstacles = self._parse_analytic_obstacles_param()
        self.analytic_checker = self._make_analytic_rrt_checker()
        self._rrt_collision_checker = self.analytic_checker
        self.moveit_core = None
        self.robot_model = None
        self.planning_scene_monitor = None
        self._moveit_collision_call_style: Optional[str] = None
        self._moveit_update_call_style: Optional[str] = None
        self._moveit_check_total = 0
        self._moveit_check_true = 0
        self._moveit_check_warned = False
        self._moveit_py_available = self._init_moveit_py_backend()

        self._rrt_detail_sampling = {
            "p_intent_pre": 0.55,
            "p_goal_pre": 0.15,
            "p_uniform_pre": 0.30,
            "p_intent_post": 0.65,
            "p_informed_post": 0.30,
            "p_goal_post": 0.05,
        }

        self.intent_rrt = IntentBiasedRRT(
            collision_checker_fn=self._rrt_collision_checker,
            step_size=self.rrt_step_size_effective,
            r_neighbor=self.rrt_neighbor_radius_effective,
            max_iter=self.rrt_max_iter_effective,
            timeout_sec=self.rrt_timeout_sec_effective,
            max_edge_samples=self.rrt_max_edge_samples_effective,
            edge_sample_step=self.rrt_edge_sample_step_effective,
            sigma_intent=self.rrt_sigma_intent_effective,
            rng_seed=self.rrt_rng_seed,
        )

        # Trajectory execution adapter (placeholder).
        self.trajectory_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
        )
        self.get_logger().info(
            f"Trajectory action target: {self.trajectory_action_name}"
        )
        self.get_logger().info(
            "Action tolerances: "
            f"path={self.action_path_tolerance_rad:.3f} rad, "
            f"goal={self.action_goal_tolerance_rad:.3f} rad, "
            f"goal_time={self.action_goal_time_tolerance_sec:.3f} s"
        )
        self.get_logger().info(
            f"Nominal source mode: {self.nominal_source}, ee_orientation_mode: {self.ee_orientation_mode}"
        )
        self.get_logger().info(
            "Runtime backend: "
            f"{self.runtime_backend} (cpp_clients_ready={self._cpp_runtime_ready()}, "
            f"service_ns={self.cpp_bridge_service_ns})"
        )
        self.get_logger().info(
            "CPP collision required: "
            f"{self.cpp_bridge_collision_required} "
            f"(check_service={self._cpp_check_states_service_name or 'n/a'})"
        )
        self.get_logger().info(
            "CPP local planner: "
            f"enable={self.use_cpp_local_planner}, fallback={self.allow_cpp_local_planner_fallback}, "
            f"type={self.planner_type}, service={self._cpp_local_planner_service_name or 'n/a'}, "
            f"motion_check={self._cpp_motion_check_service_name or 'n/a'}, "
            f"timeout={self.cpp_planner_timeout_sec:.3f}s, max_iter={self.cpp_planner_max_iter}, "
            f"step={self.cpp_planner_step_size:.3f}, edge_res={self.cpp_edge_resolution:.3f}, "
            f"ompl_simplify_request={self.ompl_simplify_enable}, "
            f"ompl_simplify_timeout={self.ompl_simplify_timeout_sec:.3f}s"
        )
        if self.planner_type == "ompl_rrt_connect":
            self.get_logger().info(
                "planner_type=ompl_rrt_connect selected in Python; "
                "make sure intent_runtime_bridge is launched with planner_type:=ompl_rrt_connect. "
                "OMPL simplify is applied inside intent_runtime_bridge; check bridge startup log."
            )
        self.get_logger().info(
            "RRT params: "
            f"step_size={self.rrt_step_size_effective:.3f}, "
            f"neighbor_radius={self.rrt_neighbor_radius_effective:.3f}, "
            f"sigma_intent={self.rrt_sigma_intent_effective:.4f}, "
            f"edge_sample_step={self.rrt_edge_sample_step_effective:.4f}, "
            f"max_iter={self.rrt_max_iter_effective}, "
            f"timeout_sec={self.rrt_timeout_sec_effective:.3f}, "
            f"max_edge_samples={self.rrt_max_edge_samples_effective}, "
            f"refine_fixed_budget={self.refine_fixed_budget}, "
            f"rng_seed={self.rrt_rng_seed}"
        )
        self.get_logger().info(
            "Demo timing: "
            f"demo_len={self.demo_len}, demo_dt={self.demo_dt:.4f}, "
            f"sync_nominal_dt={self.sync_nominal_dt_with_demo_dt}, nominal_dt={self.nominal_dt:.4f}"
        )
        self.get_logger().info(
            "Adaptive scale: "
            f"enable={self.adaptive_scale_enable}, "
            f"interp_ratio={self.adaptive_interp_step_ratio:.3f}, "
            f"neighbor_ratio={self.adaptive_neighbor_step_ratio:.3f}, "
            f"sigma_ratio={self.adaptive_sigma_step_ratio:.3f}, "
            f"edge_ratio={self.adaptive_edge_step_ratio:.3f}"
        )
        self.get_logger().info(
            "Offline safety: "
            f"postcheck_enable={self.offline_postcheck_collision_enable}, "
            f"postcheck_allow_colliding_points={self.offline_postcheck_max_colliding_points}, "
            f"postcheck_edge_resolution={self.postcheck_edge_resolution:.4f}, "
            f"wait_action_result={self.offline_wait_action_result}, "
            f"allow_stale_state_stitch={self.offline_allow_stale_state_stitch}, "
            f"stitch_start_from_current={self.offline_stitch_start_from_current}, "
            f"start_state_tolerance={self.start_state_tolerance:.4f}, "
            f"action_result_timeout={self.offline_action_result_timeout_sec:.2f}s"
        )
        self.get_logger().info(
            "Dispatch dynamics safety: "
            f"joint_limit_margin={self.joint_limit_margin:.4f}, "
            f"velocity_scale={self.velocity_scale:.3f}, "
            f"acceleration_scale={self.acceleration_scale:.3f}, "
            f"enable_conservative_time_scaling={self.enable_conservative_time_scaling}, "
            f"max_time_scaling_factor={self.max_time_scaling_factor:.2f}, "
            f"minimum_dt={self.minimum_dt:.4f}"
        )
        if not self.postcheck_check_edges:
            self.get_logger().warn(
                "postcheck_check_edges=false is ignored in offline strict safety mode; "
                "FMP post-check always checks states and edges."
            )
        self.get_logger().info(
            "Via settings: "
            f"densify={self.via_densify_enable}, "
            f"interp_dist={self.via_interp_dist:.4f}, "
            f"trim_sec={self.via_trim_sec:.4f}, "
            f"global_dedup_enable={self.via_global_dedup_enable}"
        )
        if self.execution_mode == "offline" and (
            self.rrt_max_iter_effective != self.rrt_max_iter
            or abs(self.rrt_timeout_sec_effective - self.rrt_timeout_sec) > 1e-9
            or self.rrt_max_edge_samples_effective != self.rrt_max_edge_samples
        ):
            self.get_logger().info(
                "Offline RRT budget uplift applied: "
                f"iter {self.rrt_max_iter}->{self.rrt_max_iter_effective}, "
                f"timeout {self.rrt_timeout_sec:.3f}->{self.rrt_timeout_sec_effective:.3f}, "
                f"edge_samples {self.rrt_max_edge_samples}->{self.rrt_max_edge_samples_effective}"
            )

        # Placeholder nominal joint point cache (6D zero pose initially).
        self.current_nominal_point = [0.0] * len(self.joint_names)

        # -------------------------
        # Joint state cache for trajectory stitching
        # -------------------------
        self._q_now = np.zeros(len(self.joint_names), dtype=float)
        self._dq_now = np.zeros(len(self.joint_names), dtype=float)
        self._ddq_now = np.zeros(len(self.joint_names), dtype=float)
        self._state_updated_monotonic = 0.0
        self._last_joint_state_monotonic: Optional[float] = None
        self._has_joint_state = False
        self._joint_name_to_idx: Dict[str, int] = {}
        self._last_stale_warn_ts = 0.0
        self._stale_warn_interval_sec = 5.0
        self._joint_state_cb_group = ReentrantCallbackGroup()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            qos,
            callback_group=self._joint_state_cb_group,
        )

        # -------------------------
        # Action/dispatch state
        # -------------------------
        self._active_goal_handle = None
        self._pending_goal_future = None
        self._pending_cancel_future = None
        self._last_goal_send_monotonic = 0.0
        self._last_action_result_monotonic = 0.0
        self._last_action_result_status: Optional[int] = None
        self._last_action_result_error_code: Optional[int] = None
        self._last_action_result_error_text = ""
        self._last_send_monotonic = 0.0
        self._last_preempt_monotonic = 0.0
        self._last_sent_traj: Optional[np.ndarray] = None
        self._last_sent_signature: Optional[str] = None
        self._last_sent_meta: Dict[str, Any] = {}
        self._last_danger_flag = False
        self._danger_latched_until = 0.0
        self._decision_counters: Dict[str, int] = {
            "sent": 0,
            "skipped_duplicate": 0,
            "skipped_rate_limit": 0,
            "skipped_stale_state": 0,
            "preempted": 0,
            "continued_active_goal": 0,
        }

        # 2-obstacle benchmark runtime statistics.
        self._stats_window_sec = 60.0
        self._stats_start_monotonic = time.monotonic()
        self._stats_written = False
        self._pending_collision_request_ts: Optional[float] = None

        self._danger_count = 0
        self._danger_event_count = 0
        self._nominal_count = 0
        self._rrt_call_count = 0
        self._rrt_timeout_count = 0
        self._orchestrator_phase_b_count = 0

        self._rrt_iter_samples: List[float] = []
        self._rrt_time_ms_samples: List[float] = []
        self._rrt_collision_queries_samples: List[float] = []
        self._trajectory_jerk_proxy_samples: List[float] = []
        self._segment_count_samples: List[float] = []
        self._refine_budget_samples: List[float] = []
        self._rrt_stop_reason_counts: Dict[str, int] = {}
        self._cpp_local_planner_used = False
        self._cpp_plan_success_count = 0
        self._cpp_plan_failure_count = 0
        self._cpp_plan_time_ms_samples: List[float] = []
        self._cpp_plan_collision_queries_samples: List[float] = []
        self._postcheck_passed = False
        self._postcheck_first_invalid_state = -1
        self._postcheck_first_invalid_edge = -1
        self._postcheck_state_invalid_count = 0
        self._postcheck_edge_invalid_count = 0
        self._postcheck_elapsed_ms = 0.0
        self._postcheck_collision_queries = 0
        self._execution_aborted = False
        self._dispatch_action_status = -1
        self._dispatch_error_code = 0
        self._dispatch_error_string = ""
        self._trajectory_duration_sec = 0.0
        self._trajectory_point_count = 0
        self._trajectory_max_joint_delta = 0.0
        self._trajectory_estimated_max_velocity = 0.0
        self._trajectory_estimated_max_acceleration = 0.0
        self._trajectory_time_scaling_factor = 1.0
        self._start_state_error_max = 0.0
        self._start_state_error_norm = 0.0
        self._last_dispatch_diag: Dict[str, Any] = {}
        self._timing_samples: Dict[str, List[float]] = {
            "collision_check_ms": [],
            "rrt_plan_ms": [],
            "fmp_modulate_ms": [],
            "execute_trajectory_ms": [],
            "orchestrator_total_ms": [],
        }

        # Non-blocking state for two-phase orchestrator flow.
        self._waiting_collision_result = False

        self.orchestrator_timer = None
        self._offline_once_timer = None
        if self.execution_mode == "online":
            self.orchestrator_timer = self.create_timer(0.1, self.orchestrator_loop)
            self.get_logger().info("IntentHybridPlannerNode started with 10 Hz online orchestrator.")
        else:
            self._offline_once_timer = self.create_timer(
                self.offline_start_delay_sec, self._offline_once_timer_cb
            )
            self.get_logger().info(
                "IntentHybridPlannerNode started in offline mode: one-shot batch planning is scheduled."
            )

    def _make_analytic_rrt_checker(self) -> Callable[[np.ndarray], bool]:
        obstacle_spheres = list(self._analytic_obstacles)
        l1 = 0.163
        l2 = 0.479
        l3 = 0.392
        safety_margin = 0.10

        def checker(state: np.ndarray) -> bool:
            q = np.asarray(state, dtype=float).reshape(-1)
            if q.size < 3:
                return False

            q1 = float(q[0])
            q2 = float(q[1])
            q3 = float(q[2])

            c1 = np.cos(q1)
            s1 = np.sin(q1)
            c2 = np.cos(q2)
            s2 = np.sin(q2)
            c23 = np.cos(q2 + q3)
            s23 = np.sin(q2 + q3)

            shoulder = np.array([0.0, 0.0, l1], dtype=float)
            elbow = np.array([c1 * l2 * c2, s1 * l2 * c2, l1 + l2 * s2], dtype=float)
            wrist = np.array(
                [
                    c1 * (l2 * c2 + l3 * c23),
                    s1 * (l2 * c2 + l3 * c23),
                    l1 + l2 * s2 + l3 * s23,
                ],
                dtype=float,
            )

            for obs in obstacle_spheres:
                center = obs[:3]
                threshold = float(obs[3] + safety_margin)
                for p in (shoulder, elbow, wrist):
                    if float(np.linalg.norm(p - center)) < threshold:
                        return False
            return True

        return checker

    def _parse_analytic_obstacles_param(self) -> List[np.ndarray]:
        default_obs: List[np.ndarray] = []
        nominal_source = getattr(self, "nominal_source", "joint")
        plane_obs = getattr(self, "_plane_obstacles", [])
        if nominal_source == "ee_plane" and plane_obs:
            for obs in plane_obs:
                center = self._map_uvz_to_xyz(obs["u"], obs["v"], obs.get("z", 0.0))
                default_obs.append(
                    np.array([center[0], center[1], center[2], float(obs["radius"])], dtype=float)
                )
        if not default_obs:
            default_obs = [
                np.array([0.45, 0.00, 0.40, 0.12], dtype=float),
                np.array([0.45, 0.20, 0.40, 0.12], dtype=float),
            ]
        raw = (self.analytic_obstacles_json or "").strip()
        if not raw:
            return default_obs
        try:
            parsed = json.loads(raw)
            out: List[np.ndarray] = []
            if isinstance(parsed, list):
                for item in parsed:
                    if not isinstance(item, (list, tuple)) or len(item) != 4:
                        continue
                    arr = np.asarray(item, dtype=float).reshape(-1)
                    if arr.size != 4 or (arr[3] <= 0.0):
                        continue
                    out.append(arr)
            if out:
                return out
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"analytic_obstacles_json parse failed: {exc}. Use default spheres.")
        return default_obs

    def _load_obstacle_config_scene(self) -> Optional[Dict[str, Any]]:
        path_text = str(getattr(self, "obstacle_config_file", "") or "").strip()
        if not path_text:
            return None
        path = Path(path_text).expanduser()
        if not path.exists():
            self.get_logger().error(f"obstacle_config_file does not exist: {path}")
            return {"error": f"missing obstacle config: {path}", "obstacles": []}
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"obstacle_config_file parse failed: {path}: {exc}")
            return {"error": f"parse failed: {exc}", "obstacles": []}

        frame_id = str(raw.get("frame_id", "base_link")).strip() or "base_link"
        world_name = str(raw.get("world_name", "empty")).strip() or "empty"
        obstacles: List[Dict[str, Any]] = []
        for i, item in enumerate(raw.get("obstacles", []), start=1):
            if not isinstance(item, dict):
                continue
            obs_type = str(item.get("type", "cylinder")).strip().lower()
            if obs_type != "cylinder":
                self.get_logger().warn(f"Unsupported obstacle type in config ignored: {obs_type}")
                continue
            try:
                radius = float(item.get("radius", 0.08))
                height = float(item.get("height", 0.8))
                if radius <= 0.0 or height <= 0.0:
                    continue
                obstacles.append(
                    {
                        "id": str(item.get("id", f"pillar_{i:02d}")).strip() or f"pillar_{i:02d}",
                        "type": "cylinder",
                        "x": float(item.get("x", 0.0)),
                        "y": float(item.get("y", 0.0)),
                        "z": float(item.get("z", 0.0)),
                        "radius": radius,
                        "height": height,
                        "frame_id": frame_id,
                    }
                )
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(f"Invalid obstacle entry ignored: {exc}")

        return {
            "path": str(path),
            "world_name": world_name,
            "frame_id": frame_id,
            "obstacles": obstacles,
        }

    def _build_collision_objects_from_obstacle_config(
        self, scene_cfg: Dict[str, Any]
    ) -> List[CollisionObject]:
        frame_id = str(scene_cfg.get("frame_id", "base_link") or "base_link")
        out: List[CollisionObject] = []
        for obs in scene_cfg.get("obstacles", []):
            obj = CollisionObject()
            obj.header = Header(frame_id=frame_id)
            obj.id = str(obs.get("id", "obstacle"))
            obj.operation = CollisionObject.ADD

            prim = SolidPrimitive()
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = [float(obs["height"]), float(obs["radius"])]

            pose = Pose()
            pose.position.x = float(obs["x"])
            pose.position.y = float(obs["y"])
            pose.position.z = float(obs["z"])
            pose.orientation.w = 1.0

            obj.primitives.append(prim)
            obj.primitive_poses.append(pose)
            out.append(obj)
        return out

    def _sync_obstacle_config_to_planning_scene(self, where: str) -> bool:
        if not str(getattr(self, "obstacle_config_file", "") or "").strip():
            return True
        scene_cfg = self._load_obstacle_config_scene()
        if not scene_cfg or scene_cfg.get("error"):
            return False
        objs = self._build_collision_objects_from_obstacle_config(scene_cfg)
        if not objs:
            self.get_logger().error(
                f"obstacle_config_file has no valid collision objects ({where}): "
                f"{scene_cfg.get('path', self.obstacle_config_file)}"
            )
            return False

        if self.obstacle_config_apply_to_planning_scene:
            if self.apply_planning_scene_client is None:
                self.get_logger().error("ApplyPlanningScene client is unavailable.")
                return False
            timeout_sec = float(self.obstacle_config_apply_timeout_sec)
            if not self.apply_planning_scene_client.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error("/apply_planning_scene is not ready for obstacle sync.")
                return False
            req = ApplyPlanningScene.Request()
            req.scene = PlanningScene()
            req.scene.is_diff = True
            req.scene.world.collision_objects = objs
            fut = self.apply_planning_scene_client.call_async(req)
            resp = self._wait_future_blocking(fut, timeout_sec)
            if resp is None or not bool(getattr(resp, "success", False)):
                self.get_logger().error("/apply_planning_scene failed during obstacle sync.")
                return False

        for _ in range(3):
            for obj in objs:
                self.collision_object_pub.publish(obj)
            time.sleep(0.05)

        ids = ",".join(obj.id for obj in objs)
        self.get_logger().info(
            "Obstacle config synced to MoveIt PlanningScene and /collision_object "
            f"({where}): ids=[{ids}], frame={scene_cfg.get('frame_id')}, "
            f"source={scene_cfg.get('path')}"
        )
        return True

    def _scene_obstacles_for_export(self) -> List[Any]:
        scene_cfg = self._load_obstacle_config_scene()
        if scene_cfg and not scene_cfg.get("error") and scene_cfg.get("obstacles"):
            return list(scene_cfg.get("obstacles", []))

        obstacles: List[Any] = []
        if self._plane_obstacles:
            for obs in self._plane_obstacles:
                p_xyz = self._map_uvz_to_xyz(obs["u"], obs["v"], obs.get("z", 0.0))
                obstacles.append(
                    {
                        "x": float(p_xyz[0]),
                        "y": float(p_xyz[1]),
                        "z": float(p_xyz[2]),
                        "radius": float(obs.get("radius", 0.0)),
                    }
                )
        elif self._analytic_obstacles:
            for obs in self._analytic_obstacles:
                arr = np.asarray(obs, dtype=float).reshape(-1)
                if arr.size == 4:
                    obstacles.append([float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])])
        return obstacles

    def _init_moveit_py_backend(self) -> bool:
        if self.rrt_collision_backend != "moveit_py":
            self.get_logger().info("RRT collision backend: analytic")
            return False
        if MoveItPy is None:
            msg = (
                "MoveItPy import failed or moveit_py is not installed "
                "(ROS2 Humble apt repositories often do not provide moveit_py)."
            )
            if self.moveit_py_strict:
                raise RuntimeError(msg)
            self.get_logger().warn(f"{msg} Fallback to analytic.")
            self.get_logger().info("RRT collision backend: analytic")
            return False

        try:
            self.moveit_core = MoveItPy(node_name=self.get_name())
            self.robot_model = self.moveit_core.get_robot_model()
            self.planning_scene_monitor = self.moveit_core.get_planning_scene_monitor()
        except Exception as exc:  # pylint: disable=broad-except
            if self.moveit_py_strict:
                raise
            self.moveit_core = None
            self.robot_model = None
            self.planning_scene_monitor = None
            self.get_logger().warn(f"MoveItPy init failed: {exc}. Fallback to analytic.")
            self.get_logger().info("RRT collision backend: analytic")
            return False

        self.get_logger().info("RRT collision backend: moveit_py")
        return True

    @contextmanager
    def _moveit_read_only_scene(self):
        if self.planning_scene_monitor is None:
            raise RuntimeError("PlanningSceneMonitor is not initialized.")
        if hasattr(self.planning_scene_monitor, "read_only"):
            with self.planning_scene_monitor.read_only() as scene:
                yield scene
            return
        if hasattr(self.planning_scene_monitor, "readOnly"):
            with self.planning_scene_monitor.readOnly() as scene:
                yield scene
            return
        raise RuntimeError("PlanningSceneMonitor has no read_only/readOnly API.")

    def _create_moveit_robot_state(self) -> Any:
        if self.robot_model is None:
            raise RuntimeError("MoveIt robot model is not initialized.")
        if hasattr(self.robot_model, "create_robot_state"):
            return self.robot_model.create_robot_state()
        if MoveItRobotState is not None:
            return MoveItRobotState(self.robot_model)
        raise RuntimeError("Cannot create MoveIt RobotState from current API.")

    def _set_moveit_state_positions(self, robot_state: Any, q: np.ndarray) -> None:
        values = np.asarray(q, dtype=float).reshape(-1).tolist()
        robot_state.set_joint_group_positions(self.moveit_group_name, values)
        if self._moveit_update_call_style == "update_true":
            robot_state.update(True)
            return
        if self._moveit_update_call_style == "update":
            robot_state.update()
            return
        if not hasattr(robot_state, "update"):
            return
        try:
            robot_state.update(True)
            self._moveit_update_call_style = "update_true"
        except Exception:
            robot_state.update()
            self._moveit_update_call_style = "update"

    def _is_moveit_state_colliding(self, scene: Any, robot_state: Any) -> bool:
        if self._moveit_collision_call_style == "state":
            return bool(scene.is_state_colliding(robot_state))
        if self._moveit_collision_call_style == "state_group":
            return bool(scene.is_state_colliding(robot_state, self.moveit_group_name))
        if self._moveit_collision_call_style == "kw_state":
            return bool(scene.is_state_colliding(robot_state=robot_state))
        if self._moveit_collision_call_style == "kw_state_group":
            return bool(
                scene.is_state_colliding(
                    robot_state=robot_state,
                    joint_model_group_name=self.moveit_group_name,
                )
            )

        attempts = (
            ("state", lambda: scene.is_state_colliding(robot_state)),
            ("state_group", lambda: scene.is_state_colliding(robot_state, self.moveit_group_name)),
            ("kw_state", lambda: scene.is_state_colliding(robot_state=robot_state)),
            (
                "kw_state_group",
                lambda: scene.is_state_colliding(
                    robot_state=robot_state,
                    joint_model_group_name=self.moveit_group_name,
                ),
            ),
        )
        last_exc: Optional[Exception] = None
        for style, call in attempts:
            try:
                result = bool(call())
                self._moveit_collision_call_style = style
                return result
            except Exception as exc:  # pylint: disable=broad-except
                last_exc = exc
        raise RuntimeError(f"scene.is_state_colliding API mismatch: {last_exc}")

    def _make_moveit_batch_checker(self, locked_scene: Any, test_state: Any) -> Callable[[np.ndarray], bool]:
        def checker(state_array: np.ndarray) -> bool:
            self._set_moveit_state_positions(test_state, state_array)
            ok = not self._is_moveit_state_colliding(locked_scene, test_state)
            self._moveit_check_total += 1
            if ok:
                self._moveit_check_true += 1
            if (not self._moveit_check_warned) and self._moveit_check_total >= 200:
                true_ratio = self._moveit_check_true / float(max(self._moveit_check_total, 1))
                if true_ratio > 0.995:
                    self.get_logger().warn(
                        "MoveItPy collision checker returns almost all True; "
                        "please verify state update and planning scene sync."
                    )
                    self._moveit_check_warned = True
            return ok

        return checker

    def _moveit_backend_active(self) -> bool:
        return self.rrt_collision_backend == "moveit_py" and self._moveit_py_available

    def _cpp_runtime_requested(self) -> bool:
        return self.runtime_backend == "cpp_bridge"

    def _cpp_runtime_ready(self) -> bool:
        return (
            self._cpp_runtime_clients_ready
            and self.cpp_check_states_client is not None
            and self.cpp_dispatch_client is not None
            and self.cpp_publish_markers_client is not None
        )

    def _use_cpp_runtime_for_offline(self) -> bool:
        return self.execution_mode == "offline" and self._cpp_runtime_requested() and self._cpp_runtime_ready()

    def _cpp_collision_required_for_offline(self) -> bool:
        return bool(getattr(self, "cpp_bridge_collision_required", False)) and (
            getattr(self, "execution_mode", "online") == "offline"
        )

    def _set_last_cpp_collision_error(self, msg: str) -> None:
        self._last_cpp_collision_error = str(msg).strip()

    def _describe_cpp_collision_unavailable(self) -> str:
        reasons: List[str] = []
        if self.execution_mode != "offline":
            reasons.append(f"execution_mode={self.execution_mode} (need offline)")
        if not self._cpp_runtime_requested():
            reasons.append(f"runtime_backend={self.runtime_backend} (need cpp_bridge)")
        if not self._cpp_runtime_clients_ready:
            reasons.append("cpp runtime clients not initialized")
        if self.cpp_check_states_client is None:
            reasons.append("check_states client is None")
        elif not self.cpp_check_states_client.service_is_ready():
            svc = self._cpp_check_states_service_name or "/intent_runtime/check_states_batch"
            reasons.append(f"{svc} not ready")
        if not reasons:
            return "unknown reason"
        return "; ".join(reasons)

    def _wait_for_cpp_check_service_ready(self, wait_sec: float) -> bool:
        if self.cpp_check_states_client is None:
            return False
        if self.cpp_check_states_client.service_is_ready():
            return True
        timeout_sec = float(max(wait_sec, 0.01))
        try:
            if self.cpp_check_states_client.wait_for_service(timeout_sec=timeout_sec):
                return True
        except Exception:  # pylint: disable=broad-except
            return bool(self.cpp_check_states_client.service_is_ready())
        return bool(self.cpp_check_states_client.service_is_ready())

    def _wait_for_cpp_motion_service_ready(self, wait_sec: float) -> bool:
        if self.cpp_motion_check_client is None:
            return False
        if self.cpp_motion_check_client.service_is_ready():
            return True
        try:
            return bool(self.cpp_motion_check_client.wait_for_service(timeout_sec=float(max(wait_sec, 0.01))))
        except Exception:  # pylint: disable=broad-except
            return bool(self.cpp_motion_check_client.service_is_ready())

    def _wait_for_cpp_local_planner_ready(self, wait_sec: float) -> bool:
        if self.cpp_local_planner_client is None:
            return False
        if self.cpp_local_planner_client.service_is_ready():
            return True
        try:
            return bool(self.cpp_local_planner_client.wait_for_service(timeout_sec=float(max(wait_sec, 0.01))))
        except Exception:  # pylint: disable=broad-except
            return bool(self.cpp_local_planner_client.service_is_ready())

    def _ensure_cpp_collision_runtime_available(self, where: str) -> bool:
        if not self._cpp_collision_required_for_offline():
            return True
        if (
            self.execution_mode == "offline"
            and self._cpp_runtime_requested()
            and self.cpp_check_states_client is not None
            and (not self.cpp_check_states_client.service_is_ready())
        ):
            self._wait_for_cpp_check_service_ready(max(self.cpp_bridge_timeout_sec, 1.0))
        reason = self._describe_cpp_collision_unavailable()
        if reason != "unknown reason":
            self._set_last_cpp_collision_error(reason)
            self.get_logger().error(
                f"cpp_bridge collision is required ({where}) but unavailable: {reason}"
            )
            return False
        return True

    def _assert_cpp_collision_bridge_ready(self, where: str, probe_state: Optional[np.ndarray] = None) -> bool:
        if not self._ensure_cpp_collision_runtime_available(where):
            return False
        if not self._cpp_collision_required_for_offline():
            return True
        if probe_state is None:
            return True
        q = np.asarray(probe_state, dtype=float).reshape(1, -1)
        out = self._check_states_batch_cpp(q)
        if out is not None and out.size == 1:
            return True
        reason = self._last_cpp_collision_error or "probe failed without explicit reason"
        self.get_logger().error(
            f"cpp_bridge collision preflight failed ({where}): {reason}"
        )
        return False

    def _wait_future_blocking(self, fut, timeout_sec: float):
        deadline = time.monotonic() + max(float(timeout_sec), 0.01)
        while (not fut.done()) and (time.monotonic() < deadline):
            time.sleep(0.002)
        if not fut.done():
            return None
        try:
            return fut.result()
        except Exception:  # pylint: disable=broad-except
            return None

    def _apply_action_tolerances(self, goal: FollowJointTrajectory.Goal) -> None:
        path_tol = float(self.action_path_tolerance_rad)
        goal_tol = float(self.action_goal_tolerance_rad)
        goal_time_tol = float(self.action_goal_time_tolerance_sec)
        if path_tol > 0.0:
            goal.path_tolerance = []
            for name in self.joint_names:
                tol = JointTolerance()
                tol.name = str(name)
                tol.position = float(path_tol)
                goal.path_tolerance.append(tol)
        if goal_tol > 0.0:
            goal.goal_tolerance = []
            for name in self.joint_names:
                tol = JointTolerance()
                tol.name = str(name)
                tol.position = float(goal_tol)
                goal.goal_tolerance.append(tol)
        if goal_time_tol > 0.0:
            sec = int(goal_time_tol)
            nanosec = int((goal_time_tol - sec) * 1e9)
            goal.goal_time_tolerance = Duration(sec=sec, nanosec=nanosec)

    def _check_states_batch_cpp(self, states_row_major: np.ndarray) -> Optional[np.ndarray]:
        if not self._use_cpp_runtime_for_offline():
            self._set_last_cpp_collision_error(self._describe_cpp_collision_unavailable())
            return None
        states = np.asarray(states_row_major, dtype=float)
        if states.ndim != 2 or states.shape[1] != len(self.joint_names):
            self._set_last_cpp_collision_error(
                f"invalid state batch shape {states.shape}, expected (N,{len(self.joint_names)})"
            )
            return None
        req = CheckStatesBatch.Request()
        req.group_name = self.moveit_group_name
        req.joint_names = list(self.joint_names)
        req.dof = int(len(self.joint_names))
        req.states_flat = states.reshape(-1).tolist()
        if not self._wait_for_cpp_check_service_ready(max(self.cpp_bridge_timeout_sec, 0.5)):
            svc = self._cpp_check_states_service_name or "/intent_runtime/check_states_batch"
            self._set_last_cpp_collision_error(f"{svc} not ready")
            return None
        fut = self.cpp_check_states_client.call_async(req)
        timeout_sec = max(self.cpp_bridge_timeout_sec + 0.01 * float(states.shape[0]), 4.0)
        resp = self._wait_future_blocking(fut, timeout_sec)
        if resp is None:
            self._set_last_cpp_collision_error(
                f"check_states_batch timeout/no-response (N={states.shape[0]}, timeout={timeout_sec:.2f}s)"
            )
            self.get_logger().warn("cpp_bridge check_states_batch timeout/no-response.")
            return None
        if not bool(resp.ok):
            self._set_last_cpp_collision_error(
                f"check_states_batch error: {str(resp.error_message)}"
            )
            self.get_logger().warn(
                f"cpp_bridge check_states_batch failed: {str(resp.error_message)}"
            )
            return None
        out = np.asarray(list(resp.collision_free), dtype=bool).reshape(-1)
        if out.size != states.shape[0]:
            self._set_last_cpp_collision_error(
                f"check_states_batch size mismatch: expect {states.shape[0]}, got {out.size}"
            )
            return None
        self._set_last_cpp_collision_error("")
        return out

    def _check_motion_batch_cpp(
        self,
        states_row_major: np.ndarray,
        *,
        check_edges: bool,
        where: str,
        edge_resolution: Optional[float] = None,
    ) -> Optional[Dict[str, Any]]:
        if self.execution_mode != "offline":
            self._set_last_cpp_collision_error("CheckMotionBatch is only used by the offline safety path")
            return None
        if not self._cpp_runtime_ready():
            self._set_last_cpp_collision_error(self._describe_cpp_collision_unavailable())
            return None
        states = np.asarray(states_row_major, dtype=float)
        if states.ndim != 2 or states.shape[1] != len(self.joint_names):
            self._set_last_cpp_collision_error(
                f"invalid motion batch shape {states.shape}, expected (N,{len(self.joint_names)})"
            )
            return None
        if self.cpp_motion_check_client is None:
            self._set_last_cpp_collision_error("check_motion_batch client is None")
            return None
        if not self._wait_for_cpp_motion_service_ready(max(self.cpp_bridge_timeout_sec, 0.5)):
            svc = self._cpp_motion_check_service_name or "/intent_runtime/check_motion_batch"
            self._set_last_cpp_collision_error(f"{svc} not ready")
            return None

        req = CheckMotionBatch.Request()
        req.group_name = self.moveit_group_name
        req.joint_names = list(self.joint_names)
        req.dof = int(len(self.joint_names))
        req.states_flat = states.reshape(-1).tolist()
        req.check_edges = bool(check_edges)
        req.edge_resolution = float(
            self.cpp_edge_resolution if edge_resolution is None else edge_resolution
        )
        fut = self.cpp_motion_check_client.call_async(req)
        timeout_sec = max(
            self.cpp_bridge_timeout_sec + 0.01 * float(states.shape[0]) + (0.02 * float(states.shape[0]) if check_edges else 0.0),
            4.0,
        )
        resp = self._wait_future_blocking(fut, timeout_sec)
        if resp is None:
            self._set_last_cpp_collision_error(
                f"check_motion_batch timeout/no-response in {where} (N={states.shape[0]}, timeout={timeout_sec:.2f}s)"
            )
            self.get_logger().warn("cpp_bridge check_motion_batch timeout/no-response.")
            return None
        if not bool(resp.ok):
            self._set_last_cpp_collision_error(
                f"check_motion_batch error in {where}: {str(resp.error_message)}"
            )
            return None
        state_valid = np.asarray(list(resp.state_valid), dtype=bool).reshape(-1)
        edge_valid = np.asarray(list(resp.edge_valid), dtype=bool).reshape(-1)
        if state_valid.size != states.shape[0]:
            self._set_last_cpp_collision_error(
                f"check_motion_batch state size mismatch: expect {states.shape[0]}, got {state_valid.size}"
            )
            return None
        if check_edges and states.shape[0] >= 2 and edge_valid.size != states.shape[0] - 1:
            self._set_last_cpp_collision_error(
                f"check_motion_batch edge size mismatch: expect {states.shape[0] - 1}, got {edge_valid.size}"
            )
            return None
        self._set_last_cpp_collision_error("")
        return {
            "state_valid": state_valid,
            "edge_valid": edge_valid,
            "first_invalid_state": int(resp.first_invalid_state),
            "first_invalid_edge": int(resp.first_invalid_edge),
            "elapsed_ms": float(resp.elapsed_ms),
            "collision_queries": int(resp.collision_queries),
        }

    @staticmethod
    def _motion_invalid_counts(motion: Optional[Dict[str, Any]]) -> Tuple[int, int]:
        if motion is None:
            return 0, 0
        state_valid = np.asarray(motion.get("state_valid", []), dtype=bool).reshape(-1)
        edge_valid = np.asarray(motion.get("edge_valid", []), dtype=bool).reshape(-1)
        return int(np.count_nonzero(~state_valid)), int(np.count_nonzero(~edge_valid))

    def _record_postcheck_metrics(self, motion: Optional[Dict[str, Any]], *, passed: bool) -> None:
        state_bad, edge_bad = self._motion_invalid_counts(motion)
        self._postcheck_passed = bool(passed)
        self._postcheck_state_invalid_count = int(state_bad)
        self._postcheck_edge_invalid_count = int(edge_bad)
        self._postcheck_first_invalid_state = int(motion.get("first_invalid_state", -1)) if motion else -1
        self._postcheck_first_invalid_edge = int(motion.get("first_invalid_edge", -1)) if motion else -1
        self._postcheck_elapsed_ms = float(motion.get("elapsed_ms", 0.0)) if motion else 0.0
        self._postcheck_collision_queries = int(motion.get("collision_queries", 0)) if motion else 0

    def _strict_motion_check_passed(
        self,
        motion: Optional[Dict[str, Any]],
        *,
        expected_states: int,
        require_edges: bool,
    ) -> bool:
        if motion is None:
            return False
        state_valid = np.asarray(motion.get("state_valid", []), dtype=bool).reshape(-1)
        edge_valid = np.asarray(motion.get("edge_valid", []), dtype=bool).reshape(-1)
        if state_valid.size != int(expected_states):
            return False
        expected_edges = max(int(expected_states) - 1, 0) if require_edges else 0
        if require_edges and edge_valid.size != expected_edges:
            return False
        if not bool(np.all(state_valid)):
            return False
        if require_edges and not bool(np.all(edge_valid)):
            return False
        if int(motion.get("first_invalid_state", -2)) != -1:
            return False
        if require_edges and int(motion.get("first_invalid_edge", -2)) != -1:
            return False
        return True

    def _log_motion_check_failure(
        self,
        label: str,
        motion: Optional[Dict[str, Any]],
        *,
        expected_states: int,
    ) -> None:
        state_bad, edge_bad = self._motion_invalid_counts(motion)
        state_size = int(np.asarray(motion.get("state_valid", []), dtype=bool).size) if motion else 0
        edge_size = int(np.asarray(motion.get("edge_valid", []), dtype=bool).size) if motion else 0
        expected_edges = max(int(expected_states) - 1, 0)
        self.get_logger().error(
            f"{label} failed: "
            f"first_invalid_state={int(motion.get('first_invalid_state', -1)) if motion else -1}, "
            f"first_invalid_edge={int(motion.get('first_invalid_edge', -1)) if motion else -1}, "
            f"invalid_state_count={state_bad}, invalid_edge_count={edge_bad}, "
            f"trajectory_point_count={int(expected_states)}, "
            f"state_valid_len={state_size}/{int(expected_states)}, "
            f"edge_valid_len={edge_size}/{expected_edges}, "
            f"elapsed_ms={float(motion.get('elapsed_ms', 0.0)) if motion else 0.0:.3f}, "
            f"collision_queries={int(motion.get('collision_queries', 0)) if motion else 0}, "
            f"reason={self._last_cpp_collision_error or 'strict motion check failed'}"
        )

    def _motion_invalid_indices(self, motion: Optional[Dict[str, Any]]) -> List[int]:
        if motion is None:
            return []
        out = set()
        state_valid = np.asarray(motion.get("state_valid", []), dtype=bool).reshape(-1)
        for i, v in enumerate(state_valid.tolist()):
            if not bool(v):
                out.add(int(i))
        edge_valid = np.asarray(motion.get("edge_valid", []), dtype=bool).reshape(-1)
        for i, v in enumerate(edge_valid.tolist()):
            if not bool(v):
                out.add(int(i))
                out.add(int(i + 1))
        n = int(state_valid.size)
        return sorted(int(i) for i in out if 0 <= int(i) < n)

    def _validate_local_path_with_cpp_motion(self, path: np.ndarray, label: str) -> bool:
        arr = np.asarray(path, dtype=float)
        if arr.ndim != 2 or arr.shape[0] != len(self.joint_names) or arr.shape[1] < 2:
            self.get_logger().error(f"{label} local path rejected: invalid shape {arr.shape}.")
            return False
        motion = self._check_motion_batch_cpp(
            arr.T,
            check_edges=True,
            where=label,
            edge_resolution=self.postcheck_edge_resolution,
        )
        ok = self._strict_motion_check_passed(
            motion,
            expected_states=int(arr.shape[1]),
            require_edges=True,
        )
        if not ok:
            self._log_motion_check_failure(label, motion, expected_states=int(arr.shape[1]))
        return ok

    def _plan_local_segment_cpp(
        self,
        *,
        start: np.ndarray,
        goal: np.ndarray,
        intent_path: np.ndarray,
        t_start: float,
        t_end: float,
        segment_label: str,
    ) -> Optional[Dict[str, Any]]:
        if not (self.execution_mode == "offline" and self.use_cpp_local_planner):
            return None
        if self.planner_type not in ("rrt_connect", "ompl_rrt_connect"):
            self.get_logger().warn(f"Unsupported C++ planner type: {self.planner_type}")
            return None
        if not self._use_cpp_runtime_for_offline():
            self._set_last_cpp_collision_error(self._describe_cpp_collision_unavailable())
            return None
        if self.cpp_local_planner_client is None:
            self._set_last_cpp_collision_error("plan_local_segment client is None")
            return None
        if not self._wait_for_cpp_local_planner_ready(max(self.cpp_bridge_timeout_sec, 0.5)):
            svc = self._cpp_local_planner_service_name or "/intent_runtime/plan_local_segment"
            self._set_last_cpp_collision_error(f"{svc} not ready")
            return None

        start_arr = np.asarray(start, dtype=float).reshape(-1)
        goal_arr = np.asarray(goal, dtype=float).reshape(-1)
        intent = np.asarray(intent_path, dtype=float)
        if intent.ndim != 2:
            self._set_last_cpp_collision_error("intent_path must be 2D")
            return None
        if intent.shape[1] != len(self.joint_names) and intent.shape[0] == len(self.joint_names):
            intent = intent.T
        if start_arr.size != len(self.joint_names) or goal_arr.size != len(self.joint_names):
            self._set_last_cpp_collision_error("local planner start/goal size mismatch")
            return None
        if intent.shape[1] != len(self.joint_names):
            self._set_last_cpp_collision_error(f"intent_path shape mismatch: {intent.shape}")
            return None

        req = PlanLocalSegment.Request()
        req.group_name = self.moveit_group_name
        req.joint_names = list(self.joint_names)
        req.dof = int(len(self.joint_names))
        req.start = start_arr.tolist()
        req.goal = goal_arr.tolist()
        req.intent_flat = intent.reshape(-1).tolist()
        req.intent_points = int(intent.shape[0])
        req.t_start = float(t_start)
        req.t_end = float(t_end)
        req.state_min = (np.ones(len(self.joint_names), dtype=float) * -2.0 * np.pi).tolist()
        req.state_max = (np.ones(len(self.joint_names), dtype=float) * 2.0 * np.pi).tolist()
        req.timeout_sec = float(self.cpp_planner_timeout_sec)
        req.max_iter = int(self.cpp_planner_max_iter)
        req.step_size = float(self.cpp_planner_step_size)
        req.goal_tolerance = float(self.cpp_planner_goal_tolerance)
        req.edge_resolution = float(self.cpp_edge_resolution)
        req.p_intent = 0.55
        req.p_goal = 0.20
        req.p_uniform = 0.25
        req.sigma_intent = float(max(self.rrt_sigma_intent_effective, self.cpp_planner_step_size * 0.5))
        req.rng_seed = int(self.rrt_rng_seed if self.rrt_rng_seed is not None else 42)

        fut = self.cpp_local_planner_client.call_async(req)
        timeout_sec = max(self.cpp_bridge_timeout_sec + self.cpp_planner_timeout_sec + 1.0, 2.0)
        resp = self._wait_future_blocking(fut, timeout_sec)
        if resp is None:
            self._cpp_plan_failure_count += 1
            self._set_last_cpp_collision_error(f"plan_local_segment timeout/no-response ({segment_label})")
            return None

        self._cpp_local_planner_used = True
        self._cpp_plan_time_ms_samples.append(float(resp.elapsed_ms))
        self._cpp_plan_collision_queries_samples.append(float(resp.collision_queries))
        if not bool(resp.ok):
            self._cpp_plan_failure_count += 1
            self.get_logger().warn(
                "C++ local planner failed "
                f"({segment_label}): stop_reason={resp.stop_reason}, "
                f"iter={int(resp.iter_used)}, time_ms={float(resp.elapsed_ms):.1f}, "
                f"queries={int(resp.collision_queries)}, msg={str(resp.error_message)}"
            )
            return {
                "path_first": np.empty((len(self.joint_names), 0), dtype=float),
                "path_refine": np.empty((len(self.joint_names), 0), dtype=float),
                "stop_reason": str(resp.stop_reason or "cpp_failed"),
                "meta": {
                    "iter_used": int(resp.iter_used),
                    "time_ms": float(resp.elapsed_ms),
                    "collision_queries": int(resp.collision_queries),
                    "timeout_hit": str(resp.stop_reason) == "timeout",
                    "backend": "cpp_rrt_connect",
                },
            }

        path_points = int(resp.path_points)
        flat = np.asarray(list(resp.path_flat), dtype=float)
        if path_points <= 0 or flat.size != path_points * len(self.joint_names):
            self._cpp_plan_failure_count += 1
            self._set_last_cpp_collision_error("plan_local_segment returned invalid path shape")
            return None
        path = flat.reshape(path_points, len(self.joint_names)).T
        if not self._validate_local_path_with_cpp_motion(
            path,
            f"C++ local planner path post-check ({segment_label})",
        ):
            self._cpp_plan_failure_count += 1
            return {
                "path_first": np.empty((len(self.joint_names), 0), dtype=float),
                "path_refine": np.empty((len(self.joint_names), 0), dtype=float),
                "stop_reason": "failed_motion_postcheck",
                "meta": {
                    "iter_used": int(resp.iter_used),
                    "time_ms": float(resp.elapsed_ms),
                    "collision_queries": int(resp.collision_queries),
                    "timeout_hit": False,
                    "backend": "cpp_rrt_connect",
                },
            }
        self._cpp_plan_success_count += 1
        self.get_logger().info(
            "C++ local planner success "
            f"({segment_label}): iter={int(resp.iter_used)}, "
            f"time_ms={float(resp.elapsed_ms):.1f}, queries={int(resp.collision_queries)}, "
            f"path_points={path_points}, stop_reason={str(resp.stop_reason)}, "
            f"detail={str(resp.error_message)}"
        )
        return {
            "path_first": path,
            "path_refine": path,
            "stop_reason": str(resp.stop_reason or "success"),
            "meta": {
                "iter_used": int(resp.iter_used),
                "time_ms": float(resp.elapsed_ms),
                "collision_queries": int(resp.collision_queries),
                "timeout_hit": False,
                "backend": "cpp_rrt_connect",
            },
        }

    def _cpp_collision_checker_single_state(self, state_array: np.ndarray) -> bool:
        q = np.asarray(state_array, dtype=float).reshape(1, -1)
        out = self._check_states_batch_cpp(q)
        if out is not None and out.size == 1:
            return bool(out[0])
        self._cpp_rrt_runtime_failed = True
        reason = getattr(self, "_last_cpp_collision_error", "") or "unknown checker failure"
        self._cpp_rrt_runtime_error = f"state check: {reason}"
        if self._cpp_collision_required_for_offline():
            return False

        fallback_checker = getattr(self, "_cpp_rrt_fallback_checker", None)
        if callable(fallback_checker):
            self._warn_cpp_rrt_runtime_fallback_once("state check")
            try:
                return bool(fallback_checker(np.asarray(state_array, dtype=float).reshape(-1)))
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(
                    f"cpp_bridge fallback checker failed in state check: {exc}"
                )
        return False

    def _cpp_collision_checker_edge(self, p1: np.ndarray, p2: np.ndarray, samples: int) -> bool:
        n = int(max(samples, 2))
        a_vals = np.linspace(0.0, 1.0, n)
        p1_arr = np.asarray(p1, dtype=float).reshape(-1)
        p2_arr = np.asarray(p2, dtype=float).reshape(-1)
        pts = np.zeros((n, p1_arr.size), dtype=float)
        for i, a in enumerate(a_vals):
            pts[i, :] = p1_arr + float(a) * (p2_arr - p1_arr)
        out = self._check_states_batch_cpp(pts)
        if out is not None and out.size == n:
            return bool(np.all(out))
        self._cpp_rrt_runtime_failed = True
        reason = getattr(self, "_last_cpp_collision_error", "") or "unknown checker failure"
        self._cpp_rrt_runtime_error = f"edge check: {reason}"
        if self._cpp_collision_required_for_offline():
            return False

        fallback_edge_checker = getattr(self, "_cpp_rrt_fallback_edge_checker", None)
        if callable(fallback_edge_checker):
            self._warn_cpp_rrt_runtime_fallback_once("edge check")
            try:
                return bool(fallback_edge_checker(p1, p2, samples))
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(
                    f"cpp_bridge fallback edge checker failed: {exc}"
                )

        fallback_checker = getattr(self, "_cpp_rrt_fallback_checker", None)
        if callable(fallback_checker):
            self._warn_cpp_rrt_runtime_fallback_once("edge state-sampling")
            try:
                for row in pts:
                    if not bool(fallback_checker(np.asarray(row, dtype=float).reshape(-1))):
                        return False
                return True
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().warn(
                    f"cpp_bridge fallback checker failed in edge state-sampling: {exc}"
                )
        return False

    def _warn_cpp_rrt_runtime_fallback_once(self, where: str) -> None:
        if bool(getattr(self, "_cpp_rrt_fallback_warned", False)):
            return
        self._cpp_rrt_fallback_warned = True
        self.get_logger().warn(
            "cpp_bridge checker became unavailable during RRT "
            f"({where}); fallback checker is used for this plan because "
            "cpp_bridge_collision_required is false."
        )

    def _build_obstacle_xyzr_flat_for_markers(self) -> List[float]:
        flat: List[float] = []
        if self._plane_obstacles:
            for obs in self._plane_obstacles:
                p_xyz = self._map_uvz_to_xyz(obs["u"], obs["v"], obs.get("z", 0.0))
                flat.extend([float(p_xyz[0]), float(p_xyz[1]), float(p_xyz[2]), float(obs["radius"])])
            return flat
        for obs in self._analytic_obstacles:
            arr = np.asarray(obs, dtype=float).reshape(-1)
            if arr.size != 4:
                continue
            flat.extend([float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])])
        return flat

    def _publish_debug_markers_cpp(
        self,
        nominal_traj: np.ndarray,
        via_points: Optional[np.ndarray],
        modulated_traj: np.ndarray,
    ) -> bool:
        if not self._use_cpp_runtime_for_offline():
            return False
        if not self.cpp_publish_markers_client.service_is_ready():
            return False
        nominal = np.asarray(nominal_traj, dtype=float)
        modulated = np.asarray(modulated_traj, dtype=float)
        if nominal.ndim != 2 or modulated.ndim != 2:
            return False
        dof = len(self.joint_names)
        if nominal.shape[0] != dof or modulated.shape[0] != dof:
            return False
        req = PublishPlanningMarkers.Request()
        req.base_frame = "base_link"
        req.ee_link = str(self.fk_vis_ee_link or self.ik_link_name or "tool0")
        req.joint_names = list(self.joint_names)
        req.dof = int(dof)
        req.nominal_q_flat = nominal.T.reshape(-1).tolist()
        req.modulated_q_flat = modulated.T.reshape(-1).tolist()
        if via_points is not None:
            via = np.asarray(via_points, dtype=float)
            if via.ndim == 2 and via.shape[0] == dof and via.shape[1] > 0:
                req.via_q_flat = via.T.reshape(-1).tolist()
        if self._nominal_ee_points is not None and self._nominal_ee_points.shape[0] == 3:
            req.nominal_ee_xyz_flat = self._nominal_ee_points.T.reshape(-1).tolist()
        req.obstacle_xyzr_flat = self._build_obstacle_xyzr_flat_for_markers()
        fut = self.cpp_publish_markers_client.call_async(req)
        resp = self._wait_future_blocking(fut, max(self.cpp_bridge_timeout_sec + 0.2, 3.0))
        if resp is None:
            self.get_logger().warn("cpp_bridge publish_planning_markers timeout/no-response.")
            return False
        if not bool(resp.ok):
            self.get_logger().warn(
                f"cpp_bridge publish_planning_markers failed: {str(resp.error_message)}"
            )
            return False
        return True

    def _dispatch_trajectory_cpp_offline(
        self,
        trajectory_matrix: np.ndarray,
        *,
        nominal_dt_override: Optional[float] = None,
    ) -> str:
        if not self._use_cpp_runtime_for_offline():
            return "failed_cpp_runtime_not_enabled"
        if not self.cpp_dispatch_client.service_is_ready():
            return "failed_cpp_dispatch_service_not_ready"
        traj = np.asarray(trajectory_matrix, dtype=float)
        if traj.ndim == 1:
            traj = traj.reshape(1, -1)
        dof = len(self.joint_names)
        if traj.ndim != 2 or traj.shape[0] != dof:
            return "rejected_bad_shape"
        dispatch_dt = float(self.nominal_dt if nominal_dt_override is None else nominal_dt_override)
        self._update_dispatch_trajectory_diag(
            traj,
            duration_sec=float(max(traj.shape[1] - 1, 0) * max(dispatch_dt, 1e-6)),
        )
        if not self._validate_offline_dispatch_start(traj, where="cpp_bridge offline dispatch"):
            return "failed_start_state_mismatch"
        req = DispatchJointTrajectory.Request()
        req.action_name = str(self.trajectory_action_name)
        req.joint_names = list(self.joint_names)
        req.dof = int(dof)
        req.q_flat = traj.T.reshape(-1).tolist()
        req.nominal_dt = dispatch_dt
        req.vel_limits = np.asarray(self.vel_limits, dtype=float).reshape(-1).tolist()
        req.acc_limits = np.asarray(self.acc_limits, dtype=float).reshape(-1).tolist()
        req.stitch_from_current = bool(self.offline_stitch_start_from_current)
        req.path_tolerance_rad = float(self.action_path_tolerance_rad)
        req.goal_tolerance_rad = float(self.action_goal_tolerance_rad)
        req.goal_time_tolerance_sec = float(self.action_goal_time_tolerance_sec)
        fut = self.cpp_dispatch_client.call_async(req)
        if self.offline_wait_action_result:
            expected_exec_sec = max(
                float(max(traj.shape[1] - 1, 1)) * max(dispatch_dt, 1e-3),
                1.0,
            )
            wait_sec = max(
                float(self.cpp_bridge_timeout_sec) + expected_exec_sec + 6.0,
                float(self.offline_action_result_timeout_sec) + 1.0,
                # Keep client-side wait >= runtime bridge default result wait (30s)
                # plus a small margin, otherwise Python may timeout before bridge replies.
                35.0,
                6.0,
            )
        else:
            wait_sec = max(float(self.cpp_bridge_timeout_sec) + 0.5, 4.0)
        resp = self._wait_future_blocking(fut, wait_sec)
        if resp is None:
            self.get_logger().warn(
                f"cpp_bridge dispatch_joint_trajectory timeout/no-response (wait={wait_sec:.2f}s)."
            )
            return "failed_cpp_dispatch_timeout"
        if (not bool(resp.accepted)) or str(resp.result_code) != "sent_success":
            result_code = str(resp.result_code)
            error_msg = str(resp.error_message)
            status = int(GoalStatus.STATUS_ABORTED) if result_code == "failed_action_aborted" else -1
            self._record_dispatch_failure(
                status=status,
                error_code=self._extract_follow_error_code(error_msg),
                error_string=error_msg,
                aborted=(result_code == "failed_action_aborted"),
            )
            self.get_logger().warn(
                "cpp_bridge dispatch_joint_trajectory failed: "
                f"accepted={bool(resp.accepted)}, result={result_code}, "
                f"msg={error_msg}"
            )
            if str(resp.result_code).startswith("failed_action_"):
                self._log_action_failure_with_diag("cpp_bridge FollowJointTrajectory failed")
        return str(resp.result_code) if str(resp.result_code) else ("sent_success" if bool(resp.accepted) else "failed_cpp_dispatch")

    def _run_rrt_with_collision_backend(self, *, detailed: bool, **kwargs):
        cpp_required = self._cpp_collision_required_for_offline()
        if self.execution_mode == "offline" and self._cpp_runtime_requested():
            if not self._ensure_cpp_collision_runtime_available("RRT entry"):
                if cpp_required:
                    raise RuntimeError(
                        "cpp_bridge collision is required for RRT but runtime is unavailable: "
                        f"{self._last_cpp_collision_error or self._describe_cpp_collision_unavailable()}"
                    )
            else:
                start_state = np.asarray(
                    kwargs.get("start", np.zeros((len(self.joint_names),))),
                    dtype=float,
                ).reshape(1, -1)
                probe = self._check_states_batch_cpp(start_state)
                if probe is not None and probe.size == 1:
                    prev_checker = self.intent_rrt.collision_checker_fn
                    prev_edge_checker = self.intent_rrt.edge_checker_fn
                    self._cpp_rrt_fallback_checker = (
                        prev_checker if callable(prev_checker) else self.analytic_checker
                    )
                    self._cpp_rrt_fallback_edge_checker = (
                        prev_edge_checker if callable(prev_edge_checker) else None
                    )
                    self._cpp_rrt_fallback_warned = False
                    self._cpp_rrt_runtime_failed = False
                    self._cpp_rrt_runtime_error = ""
                    self.intent_rrt.collision_checker_fn = self._cpp_collision_checker_single_state
                    self.intent_rrt.edge_checker_fn = self._cpp_collision_checker_edge
                    try:
                        result = (
                            self.intent_rrt.plan_detailed(**kwargs)
                            if detailed
                            else self.intent_rrt.plan(**kwargs)
                        )
                    finally:
                        self.intent_rrt.collision_checker_fn = prev_checker
                        self.intent_rrt.edge_checker_fn = prev_edge_checker
                        self._cpp_rrt_fallback_checker = None
                        self._cpp_rrt_fallback_edge_checker = None
                    if cpp_required and self._cpp_rrt_runtime_failed:
                        reason = self._cpp_rrt_runtime_error or self._last_cpp_collision_error or "unknown"
                        raise RuntimeError(
                            "cpp_bridge collision checker became unavailable during RRT: "
                            f"{reason}"
                        )
                    return result
                probe_reason = self._last_cpp_collision_error or "probe failed without explicit reason"
                if cpp_required:
                    raise RuntimeError(
                        "cpp_bridge collision checker probe failed before RRT: "
                        f"{probe_reason}"
                    )
                self.get_logger().warn(
                    "cpp_bridge collision checker probe failed, fallback to python collision backend. "
                    f"reason={probe_reason}"
                )

        if self._moveit_backend_active():
            try:
                with self._moveit_read_only_scene() as locked_scene:
                    test_state = self._create_moveit_robot_state()
                    checker = self._make_moveit_batch_checker(locked_scene, test_state)
                    # Probe once outside RRT so API errors are not swallowed by _is_state_valid().
                    checker(np.asarray(kwargs["start"], dtype=float))
                    self.intent_rrt.collision_checker_fn = checker
                    return (
                        self.intent_rrt.plan_detailed(**kwargs)
                        if detailed
                        else self.intent_rrt.plan(**kwargs)
                    )
            except Exception as exc:  # pylint: disable=broad-except
                if self.moveit_py_strict:
                    raise
                self._moveit_py_available = False
                self.get_logger().warn(f"MoveItPy RRT checker failed: {exc}. Fallback to analytic.")
            finally:
                self.intent_rrt.collision_checker_fn = self.analytic_checker

        self.intent_rrt.collision_checker_fn = self.analytic_checker
        try:
            return self.intent_rrt.plan_detailed(**kwargs) if detailed else self.intent_rrt.plan(**kwargs)
        finally:
            self.intent_rrt.collision_checker_fn = self.analytic_checker

    def _sync_moveit_collision_free(self, joint_angles: np.ndarray) -> bool:
        if not self._moveit_backend_active():
            raise RuntimeError("MoveItPy backend is not active.")
        with self._moveit_read_only_scene() as locked_scene:
            test_state = self._create_moveit_robot_state()
            checker = self._make_moveit_batch_checker(locked_scene, test_state)
            return bool(checker(np.asarray(joint_angles, dtype=float)))

    def _load_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Priority:
        1) joint_limits_file YAML
        2) max_joint_velocity/max_joint_acceleration parameters
        """
        vel_default = np.array(
            self.get_parameter("max_joint_velocity").get_parameter_value().double_array_value,
            dtype=float,
        )
        acc_default = np.array(
            self.get_parameter("max_joint_acceleration").get_parameter_value().double_array_value,
            dtype=float,
        )
        if vel_default.size != len(self.joint_names):
            vel_default = np.ones(len(self.joint_names), dtype=float)
        if acc_default.size != len(self.joint_names):
            acc_default = np.ones(len(self.joint_names), dtype=float) * 2.0

        limits_path = self.joint_limits_file.strip()
        if not limits_path:
            self.get_logger().warn(
                "joint_limits_file is empty. Fallback to conservative default limits."
            )
            return vel_default, acc_default

        if yaml is None:
            self.get_logger().warn(
                "PyYAML is unavailable. Fallback to conservative default limits."
            )
            return vel_default, acc_default

        p = Path(limits_path)
        if not p.exists():
            self.get_logger().warn(
                f"joint_limits_file does not exist: {limits_path}. "
                "Fallback to conservative default limits."
            )
            return vel_default, acc_default

        try:
            data = yaml.safe_load(p.read_text(encoding="utf-8"))
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(
                f"Failed to parse joint limits YAML: {exc}. "
                "Fallback to conservative default limits."
            )
            return vel_default, acc_default

        jl = data.get("joint_limits", {}) if isinstance(data, dict) else {}
        vel = vel_default.copy()
        acc = acc_default.copy()
        for i, name in enumerate(self.joint_names):
            info = jl.get(name, {})
            v = info.get("max_velocity", vel[i])
            a = info.get("max_acceleration", acc[i])
            try:
                vel[i] = max(float(v), 1e-3)
                acc[i] = max(float(a), 1e-3)
            except Exception:
                pass
        self.get_logger().info(f"Loaded joint limits from {limits_path}")
        return vel, acc

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name:
            return

        if not self._joint_name_to_idx:
            self._joint_name_to_idx = {n: i for i, n in enumerate(msg.name)}

        if any(j not in self._joint_name_to_idx for j in self.joint_names):
            # Joint names not complete in this message.
            return

        idx = [self._joint_name_to_idx[n] for n in self.joint_names]
        if not msg.position or len(msg.position) < max(idx) + 1:
            return

        q_new = np.array([msg.position[i] for i in idx], dtype=float)

        if msg.velocity and len(msg.velocity) >= max(idx) + 1:
            dq_new = np.array([msg.velocity[i] for i in idx], dtype=float)
        else:
            dq_new = self._dq_now.copy()

        now = time.monotonic()
        if self._last_joint_state_monotonic is not None:
            dt = max(now - self._last_joint_state_monotonic, 1e-6)
            ddq_new = (dq_new - self._dq_now) / dt
        else:
            ddq_new = np.zeros_like(dq_new)

        # Clamp acceleration estimate to configured limits to avoid spikes.
        ddq_new = np.clip(ddq_new, -self.acc_limits, self.acc_limits)

        self._q_now = q_new
        self._dq_now = dq_new
        self._ddq_now = ddq_new
        self._state_updated_monotonic = now
        self._last_joint_state_monotonic = now
        self._has_joint_state = True
        self.current_nominal_point = q_new.tolist()

    def _is_state_stale(self) -> bool:
        if not self._has_joint_state:
            return True
        age = time.monotonic() - self._state_updated_monotonic
        return age > self.state_stale_timeout

    def _wait_for_fresh_joint_state(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while time.monotonic() < deadline:
            if self._has_joint_state and (not self._is_state_stale()):
                return True
            time.sleep(0.01)
        return self._has_joint_state and (not self._is_state_stale())

    def _trajectory_signature(self, traj: np.ndarray) -> str:
        flat_head = np.round(traj[:, 0], 4).tolist()
        flat_tail = np.round(traj[:, -1], 4).tolist()
        return f"{traj.shape[1]}|{flat_head}|{flat_tail}|dt={self.nominal_dt:.4f}"

    def _trajectory_diff_max(self, a: np.ndarray, b: np.ndarray) -> float:
        m = min(a.shape[1], b.shape[1])
        if m <= 0:
            return float("inf")
        return float(np.max(np.abs(a[:, :m] - b[:, :m])))

    def _update_dispatch_trajectory_diag(self, traj: np.ndarray, duration_sec: Optional[float] = None) -> None:
        arr = np.asarray(traj, dtype=float)
        if arr.ndim != 2 or arr.shape[1] <= 0:
            self._trajectory_point_count = 0
            self._trajectory_duration_sec = 0.0
            self._trajectory_max_joint_delta = 0.0
            return
        self._trajectory_point_count = int(arr.shape[1])
        if duration_sec is None:
            self._trajectory_duration_sec = float(max(arr.shape[1] - 1, 0) * max(float(self.nominal_dt), 1e-6))
        else:
            self._trajectory_duration_sec = float(max(duration_sec, 0.0))
        if arr.shape[1] >= 2:
            self._trajectory_max_joint_delta = float(np.max(np.abs(np.diff(arr, axis=1))))
        else:
            self._trajectory_max_joint_delta = 0.0
        self._last_dispatch_diag = {
            "trajectory_point_count": self._trajectory_point_count,
            "trajectory_duration_sec": self._trajectory_duration_sec,
            "trajectory_max_joint_delta": self._trajectory_max_joint_delta,
            "trajectory_estimated_max_velocity": self._trajectory_estimated_max_velocity,
            "trajectory_estimated_max_acceleration": self._trajectory_estimated_max_acceleration,
            "time_scaling_factor": self._trajectory_time_scaling_factor,
            "start_state_error_max": self._start_state_error_max,
            "start_state_error_norm": self._start_state_error_norm,
        }

    def _validate_offline_dispatch_start(self, traj: np.ndarray, *, where: str) -> bool:
        arr = np.asarray(traj, dtype=float)
        if arr.ndim != 2 or arr.shape[0] != len(self.joint_names) or arr.shape[1] <= 0:
            self.get_logger().error(f"{where} start-state check rejected invalid trajectory shape {arr.shape}.")
            return False
        wait_sec = max(0.5, float(self.state_stale_timeout) + 0.2)
        if not self._wait_for_fresh_joint_state(wait_sec):
            self._dispatch_error_string = "no fresh joint state for start-state check"
            self.get_logger().error(
                f"{where} rejected: no fresh joint state for start-state check (waited {wait_sec:.2f}s)."
            )
            return False
        q_first = arr[:, 0].copy()
        q_current = self._q_now.reshape(-1).copy()
        err = np.abs(q_first - q_current)
        self._start_state_error_max = float(np.max(err)) if err.size else 0.0
        self._start_state_error_norm = float(np.linalg.norm(err)) if err.size else 0.0
        if self._start_state_error_max <= float(self.start_state_tolerance):
            return True
        if self.offline_stitch_start_from_current:
            self.get_logger().warn(
                f"{where}: start-state mismatch will be handled by explicit stitching "
                f"(max_error={self._start_state_error_max:.4f}, norm={self._start_state_error_norm:.4f}, "
                f"tolerance={self.start_state_tolerance:.4f})."
            )
            return True
        self._dispatch_error_string = "start state mismatch"
        self.get_logger().error(
            f"{where} rejected: trajectory start does not match current joint state "
            f"(max_error={self._start_state_error_max:.4f}, norm={self._start_state_error_norm:.4f}, "
            f"tolerance={self.start_state_tolerance:.4f}, "
            f"q_current={np.round(q_current, 6).tolist()}, "
            f"q_first={np.round(q_first, 6).tolist()})."
        )
        return False

    def _resolve_joint_position_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        lower = np.asarray(self.joint_position_lower_limit, dtype=float).reshape(-1)
        upper = np.asarray(self.joint_position_upper_limit, dtype=float).reshape(-1)
        if lower.size != len(self.joint_names) or upper.size != len(self.joint_names):
            lower = np.ones(len(self.joint_names), dtype=float) * (-2.0 * np.pi)
            upper = np.ones(len(self.joint_names), dtype=float) * (2.0 * np.pi)
        upper = np.maximum(upper, lower + 1e-3)
        if (not self._joint_bounds_warned) and np.allclose(lower, -2.0 * np.pi) and np.allclose(upper, 2.0 * np.pi):
            self.get_logger().warn(
                "Dispatch safety is using default joint position bounds [-2pi, 2pi]. "
                "Configure joint_position_lower_limit/joint_position_upper_limit for tighter protection."
            )
            self._joint_bounds_warned = True
        return lower, upper

    def _estimate_dynamics(self, traj: np.ndarray, dt: float) -> Tuple[float, float, np.ndarray, np.ndarray]:
        arr = np.asarray(traj, dtype=float)
        if arr.ndim != 2 or arr.shape[1] <= 1:
            z = np.zeros((arr.shape[0],), dtype=float)
            return 0.0, 0.0, z, z
        step = max(float(dt), self.minimum_dt, 1e-6)
        vel = np.abs(np.diff(arr, axis=1)) / step
        vel_peak = np.max(vel, axis=1) if vel.size > 0 else np.zeros((arr.shape[0],), dtype=float)
        if vel.shape[1] >= 2:
            acc = np.abs(np.diff(vel, axis=1)) / step
            acc_peak = np.max(acc, axis=1) if acc.size > 0 else np.zeros((arr.shape[0],), dtype=float)
        else:
            acc_peak = np.zeros((arr.shape[0],), dtype=float)
        return float(np.max(vel_peak)), float(np.max(acc_peak)), vel_peak, acc_peak

    def _apply_dispatch_dynamics_safety(self, traj: np.ndarray) -> Tuple[bool, float]:
        arr = np.asarray(traj, dtype=float)
        if arr.ndim != 2 or arr.shape[0] != len(self.joint_names) or arr.shape[1] <= 0:
            self._dispatch_error_string = "invalid trajectory shape for safety check"
            return False, float(self.nominal_dt)

        lower, upper = self._resolve_joint_position_bounds()
        margin = float(self.joint_limit_margin)
        lo = lower + margin
        hi = upper - margin
        if np.any(lo >= hi):
            self._dispatch_error_string = "joint limit margin is too large"
            self.get_logger().error(
                "Dispatch safety rejected: joint_limit_margin leaves no valid interval "
                f"(margin={margin:.4f})."
            )
            return False, float(self.nominal_dt)

        for i in range(arr.shape[1]):
            q = arr[:, i]
            bad = np.where((q < lo) | (q > hi))[0]
            if bad.size > 0:
                j = int(bad[0])
                self._dispatch_error_string = "joint limit violation"
                self.get_logger().error(
                    "Dispatch safety rejected by joint limit margin: "
                    f"idx={i}, joint={self.joint_names[j]}, value={float(q[j]):.6f}, "
                    f"lower={float(lower[j]):.6f}, upper={float(upper[j]):.6f}, margin={margin:.6f}"
                )
                return False, float(self.nominal_dt)

        base_dt = max(float(self.nominal_dt), self.minimum_dt)
        max_vel, max_acc, vel_peak, acc_peak = self._estimate_dynamics(arr, base_dt)
        allowed_vel = np.maximum(self.vel_limits * float(self.velocity_scale), 1e-6)
        allowed_acc = np.maximum(self.acc_limits * float(self.acceleration_scale), 1e-6)
        vel_ratio = float(np.max(vel_peak / allowed_vel)) if vel_peak.size > 0 else 0.0
        acc_ratio = float(np.max(acc_peak / allowed_acc)) if acc_peak.size > 0 else 0.0

        scale_factor = 1.0
        if vel_ratio > 1.0 or acc_ratio > 1.0:
            if not self.enable_conservative_time_scaling:
                self._dispatch_error_string = "velocity/acceleration exceeded and conservative scaling is disabled"
                self.get_logger().error(
                    "Dispatch safety rejected: dynamics exceeded "
                    f"(vel_ratio={vel_ratio:.3f}, acc_ratio={acc_ratio:.3f})."
                )
                return False, base_dt
            scale_factor = max(1.0, vel_ratio, np.sqrt(max(acc_ratio, 0.0)))
            if scale_factor > float(self.max_time_scaling_factor):
                self._dispatch_error_string = "required time scaling exceeds max_time_scaling_factor"
                self.get_logger().error(
                    "Dispatch safety rejected: required time scaling exceeds configured max "
                    f"(required={scale_factor:.3f}, max={self.max_time_scaling_factor:.3f})."
                )
                return False, base_dt

        scaled_dt = base_dt * scale_factor
        max_vel_s, max_acc_s, vel_peak_s, acc_peak_s = self._estimate_dynamics(arr, scaled_dt)
        vel_ratio_s = float(np.max(vel_peak_s / allowed_vel)) if vel_peak_s.size > 0 else 0.0
        acc_ratio_s = float(np.max(acc_peak_s / allowed_acc)) if acc_peak_s.size > 0 else 0.0
        if vel_ratio_s > 1.0 + 1e-6 or acc_ratio_s > 1.0 + 1e-6:
            self._dispatch_error_string = "dynamics still exceed limits after conservative scaling"
            self.get_logger().error(
                "Dispatch safety rejected after conservative scaling: "
                f"vel_ratio={vel_ratio_s:.3f}, acc_ratio={acc_ratio_s:.3f}, "
                f"scaled_dt={scaled_dt:.4f}, scale_factor={scale_factor:.4f}"
            )
            return False, base_dt

        self._trajectory_estimated_max_velocity = float(max_vel_s)
        self._trajectory_estimated_max_acceleration = float(max_acc_s)
        self._trajectory_time_scaling_factor = float(scale_factor)
        if scale_factor > 1.0:
            self.get_logger().warn(
                "Dispatch conservative time scaling applied: "
                f"factor={scale_factor:.4f}, nominal_dt={base_dt:.4f} -> {scaled_dt:.4f}"
            )
        return True, scaled_dt

    def _record_dispatch_failure(
        self,
        *,
        status: int,
        error_code: int,
        error_string: str,
        aborted: bool,
    ) -> None:
        self._dispatch_action_status = int(status)
        self._dispatch_error_code = int(error_code)
        self._dispatch_error_string = str(error_string)
        self._execution_aborted = bool(aborted)

    @staticmethod
    def _extract_follow_error_code(text: str) -> int:
        marker = "error_code="
        s = str(text)
        idx = s.find(marker)
        if idx < 0:
            return -1
        idx += len(marker)
        end = idx
        while end < len(s) and (s[end].isdigit() or s[end] in ("-", "+")):
            end += 1
        try:
            return int(s[idx:end])
        except Exception:
            return -1

    def _log_action_failure_with_diag(self, prefix: str) -> None:
        self.get_logger().error(
            f"{prefix}: action_status={self._dispatch_action_status}, "
            f"error_code={self._dispatch_error_code}, error_string={self._dispatch_error_string}, "
            f"trajectory_point_count={self._trajectory_point_count}, "
            f"trajectory_duration_sec={self._trajectory_duration_sec:.3f}, "
            f"trajectory_max_joint_delta={self._trajectory_max_joint_delta:.4f}, "
            f"trajectory_estimated_max_velocity={self._trajectory_estimated_max_velocity:.4f}, "
            f"trajectory_estimated_max_acceleration={self._trajectory_estimated_max_acceleration:.4f}, "
            f"time_scaling_factor={self._trajectory_time_scaling_factor:.4f}, "
            f"start_state_error_max={self._start_state_error_max:.4f}, "
            f"start_state_error_norm={self._start_state_error_norm:.4f}"
        )

    def _is_new_trajectory_better(self, meta: Dict[str, Any]) -> bool:
        """
        Heuristic:
        - Prefer larger min_obstacle_distance
        - Prefer smaller risk_window_sec
        """
        old = self._last_sent_meta
        if not old:
            return True
        new_dist = float(meta.get("min_obstacle_distance", -1.0))
        old_dist = float(old.get("min_obstacle_distance", -1.0))
        new_win = float(meta.get("risk_window_sec", 1e9))
        old_win = float(old.get("risk_window_sec", 1e9))
        if new_dist > old_dist + 1e-4:
            return True
        if new_win < old_win - 1e-4:
            return True
        return False

    def _build_time_array(self, n_points: int, dt: float) -> np.ndarray:
        return np.arange(n_points, dtype=float) * max(dt, 1e-3)

    def _scale_time_for_limits(self, q: np.ndarray, t: np.ndarray) -> np.ndarray:
        if q.shape[1] < 2:
            return t

        dt_seg = np.diff(t)
        dt_seg = np.maximum(dt_seg, 1e-6)
        dq = np.diff(q, axis=1)
        vel = np.abs(dq / dt_seg[None, :])
        vel_ratio = np.max(vel / self.vel_limits[:, None])

        # Acceleration estimation (centered on inner points)
        acc_ratio = 1.0
        if q.shape[1] >= 3:
            v = dq / dt_seg[None, :]
            dt_mid = (dt_seg[:-1] + dt_seg[1:]) / 2.0
            a = np.abs(np.diff(v, axis=1) / np.maximum(dt_mid[None, :], 1e-6))
            acc_ratio = np.max(a / self.acc_limits[:, None])

        scale = max(1.0, float(vel_ratio), float(acc_ratio))
        if scale <= 1.0:
            return t
        return t * scale

    def _compute_vel_acc(self, q: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        n = q.shape[1]
        v = np.zeros_like(q)
        a = np.zeros_like(q)

        # Finite difference for velocities.
        for k in range(1, n):
            dt = max(t[k] - t[k - 1], 1e-6)
            v[:, k] = (q[:, k] - q[:, k - 1]) / dt
        v[:, 0] = self._dq_now

        # Finite difference for accelerations.
        for k in range(1, n):
            dt = max(t[k] - t[k - 1], 1e-6)
            a[:, k] = (v[:, k] - v[:, k - 1]) / dt
        a[:, 0] = self._ddq_now

        # Clamp to configured limits.
        v = np.clip(v, -self.vel_limits[:, None], self.vel_limits[:, None])
        a = np.clip(a, -self.acc_limits[:, None], self.acc_limits[:, None])
        return v, a

    def _compute_jerk_proxy(self, acc: np.ndarray, t: np.ndarray) -> float:
        if acc.shape[1] < 2:
            return 0.0
        dt = np.diff(t)
        dt = np.maximum(dt, 1e-6)
        jerk = np.abs(np.diff(acc, axis=1) / dt[None, :])
        if jerk.size <= 0:
            return 0.0
        return float(np.max(jerk))

    def _time_parameterize(self, q: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        # Default finite-difference backend.
        if self.time_param_backend == "finite_diff":
            return self._compute_vel_acc(q, t)

        # Optional ruckig backend: fallback to finite_diff if unavailable.
        if ruckig_lib is None:
            if not self._ruckig_warned:
                self.get_logger().warn("Ruckig backend requested but ruckig is not installed. Fallback to finite_diff.")
                self._ruckig_warned = True
            return self._compute_vel_acc(q, t)

        # Keep conservative behavior until full Ruckig batch parameterization is validated.
        if not self._ruckig_warned:
            self.get_logger().warn("Ruckig backend is selected but batch parameterization is not enabled yet. Fallback to finite_diff.")
            self._ruckig_warned = True
        return self._compute_vel_acc(q, t)

    def _send_goal_async(self, traj_msg: JointTrajectory) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj_msg
        self._apply_action_tolerances(goal)
        self._last_goal_send_monotonic = time.monotonic()
        self._last_action_result_monotonic = 0.0
        self._last_action_result_status = None
        self._last_action_result_error_code = None
        self._last_action_result_error_text = ""

        self._pending_goal_future = self.trajectory_action_client.send_goal_async(goal)

        def _on_goal_response(fut):
            try:
                gh = fut.result()
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"send_goal_async failed: {exc}")
                self._pending_goal_future = None
                return
            self._pending_goal_future = None
            if gh is None or not gh.accepted:
                self.get_logger().warn("Trajectory goal rejected by action server.")
                return
            self._active_goal_handle = gh
            result_future = gh.get_result_async()

            def _on_result(res_fut):
                try:
                    res = res_fut.result()
                    status = int(getattr(res, "status", -1))
                    result_msg = getattr(res, "result", None)
                    error_code = int(getattr(result_msg, "error_code", -9999)) if result_msg is not None else -9999
                    error_text = str(getattr(result_msg, "error_string", "")) if result_msg is not None else ""
                    self._last_action_result_monotonic = time.monotonic()
                    self._last_action_result_status = status
                    self._last_action_result_error_code = error_code
                    self._last_action_result_error_text = error_text
                    if status != int(GoalStatus.STATUS_SUCCEEDED) or error_code != 0:
                        self._record_dispatch_failure(
                            status=status,
                            error_code=error_code,
                            error_string=error_text,
                            aborted=(status == int(GoalStatus.STATUS_ABORTED)),
                        )
                        self._log_action_failure_with_diag("Trajectory action finished with non-success")
                except Exception as exc2:  # pylint: disable=broad-except
                    self.get_logger().warn(f"Trajectory result future failed: {exc2}")
                finally:
                    self._active_goal_handle = None

            result_future.add_done_callback(_on_result)

        self._pending_goal_future.add_done_callback(_on_goal_response)

    def _wait_offline_action_result_python(self, timeout_sec: float) -> str:
        deadline = time.monotonic() + max(float(timeout_sec), 0.01)
        start_stamp = float(self._last_goal_send_monotonic)
        while time.monotonic() < deadline:
            if self._last_action_result_monotonic >= start_stamp and start_stamp > 0.0:
                status = int(self._last_action_result_status) if self._last_action_result_status is not None else -1
                error_code = (
                    int(self._last_action_result_error_code)
                    if self._last_action_result_error_code is not None
                    else -9999
                )
                if status == int(GoalStatus.STATUS_SUCCEEDED) and error_code == 0:
                    return "succeeded"
                return f"failed(status={status}, error_code={error_code})"
            time.sleep(0.01)
        return "timeout"

    def _cancel_active_goal(self) -> None:
        if self._active_goal_handle is None:
            return
        self._pending_cancel_future = self._active_goal_handle.cancel_goal_async()

        def _on_cancel_done(_):
            self._pending_cancel_future = None
            # Let next goal callback overwrite handle when accepted.

        self._pending_cancel_future.add_done_callback(_on_cancel_done)

    def execute_trajectory(self, trajectory_matrix: np.ndarray, meta: Optional[Dict[str, Any]] = None) -> str:
        """
        Execute trajectory with stitching + de-dup + preemption decisions.
        Returns decision status:
        sent / skipped_duplicate / skipped_rate_limit / skipped_stale_state
        / preempted / continued_active_goal
        """
        meta = dict(meta or {})
        danger = bool(meta.get("danger", False))

        traj = np.asarray(trajectory_matrix, dtype=float)
        if traj.ndim == 1:
            traj = traj.reshape(1, -1)
        if traj.shape[0] != len(self.joint_names):
            self.get_logger().error(
                f"execute_trajectory rejected: expected {len(self.joint_names)}xN, got {traj.shape}"
            )
            return "rejected_bad_shape"

        if self._is_state_stale():
            now = time.monotonic()
            if now - self._last_stale_warn_ts > self._stale_warn_interval_sec:
                self.get_logger().warn(
                    "Joint state is stale or unavailable. Skip trajectory send this cycle."
                )
                self._last_stale_warn_ts = now
            self._decision_counters["skipped_stale_state"] += 1
            return "skipped_stale_state"

        # Stitch trajectory start to current hardware state.
        stitched = np.hstack([self._q_now.reshape(-1, 1), traj])
        n_points = stitched.shape[1]
        if n_points < 2:
            stitched = np.hstack([stitched, stitched])
            n_points = 2

        # Build monotonic time and enforce dynamics limits by scaling.
        t = self._build_time_array(n_points, self.nominal_dt)
        t = self._scale_time_for_limits(stitched, t)
        vel, acc = self._time_parameterize(stitched, t)

        # Hard constraints for first point alignment.
        vel[:, 0] = self._dq_now
        acc[:, 0] = self._ddq_now
        jerk_proxy = self._compute_jerk_proxy(acc, t)
        self._trajectory_jerk_proxy_samples.append(jerk_proxy)
        if (not self._jerk_warned) and jerk_proxy > float(max(self.jerk_warn_threshold, 0.0)):
            self.get_logger().warn(
                f"Trajectory jerk proxy is high: {jerk_proxy:.3f} > threshold {self.jerk_warn_threshold:.3f}"
            )
            self._jerk_warned = True

        now = time.monotonic()
        signature = self._trajectory_signature(stitched)
        diff = (
            float("inf")
            if self._last_sent_traj is None
            else self._trajectory_diff_max(stitched, self._last_sent_traj)
        )

        # Skip duplicate in nominal-safe cycles.
        if (not danger) and (self._last_sent_signature == signature or diff < self.trajectory_diff_q_eps):
            self._decision_counters["skipped_duplicate"] += 1
            return "skipped_duplicate"

        # Rate limit for non-danger cycles.
        if (not danger) and (now - self._last_send_monotonic < self.min_send_interval):
            self._decision_counters["skipped_rate_limit"] += 1
            return "skipped_rate_limit"

        # Decide preemption.
        has_active = self._active_goal_handle is not None
        can_preempt = now - self._last_preempt_monotonic >= self.min_preempt_interval
        should_preempt = danger and has_active and can_preempt and self._is_new_trajectory_better(meta)
        if danger and has_active and (not should_preempt):
            self._decision_counters["continued_active_goal"] += 1
            return "continued_active_goal"

        if should_preempt:
            self._cancel_active_goal()
            self._last_preempt_monotonic = now
            decision = "preempted"
            self._decision_counters["preempted"] += 1
        else:
            decision = "sent"
            self._decision_counters["sent"] += 1

        # Construct action goal trajectory.
        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(self.joint_names)
        for k in range(n_points):
            pt = JointTrajectoryPoint()
            pt.positions = stitched[:, k].tolist()
            pt.velocities = vel[:, k].tolist()
            pt.accelerations = acc[:, k].tolist()
            sec = int(t[k])
            nanosec = int((t[k] - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj_msg.points.append(pt)

        if not self.trajectory_action_client.server_is_ready():
            self.get_logger().warn("Action server is not ready, skip send this cycle.")
            return "skipped_action_not_ready"

        self._send_goal_async(traj_msg)
        self._last_send_monotonic = now
        self._last_sent_traj = stitched.copy()
        self._last_sent_signature = signature
        self._last_sent_meta = dict(meta)

        self.get_logger().info(
            "execute_trajectory: "
            f"decision={decision}, points={n_points}, diff={diff:.5f}, "
            f"danger={danger}, sent_count={self._decision_counters['sent']}, "
            f"skip_dup={self._decision_counters['skipped_duplicate']}"
        )
        return decision

    def _get_fk_wrist(self, q: np.ndarray) -> Point:
        arr = np.asarray(q, dtype=float).reshape(-1)
        if arr.size < 3:
            return Point()

        L1 = 0.163
        L2 = 0.479
        L3 = 0.392

        q1 = float(arr[0])
        q2 = float(arr[1])
        q3 = float(arr[2])

        c1 = np.cos(q1)
        s1 = np.sin(q1)
        c2 = np.cos(q2)
        s2 = np.sin(q2)
        c23 = np.cos(q2 + q3)
        s23 = np.sin(q2 + q3)

        p = Point()
        p.x = float(c1 * (L2 * c2 + L3 * c23))
        p.y = float(s1 * (L2 * c2 + L3 * c23))
        p.z = float(L1 + L2 * s2 + L3 * s23)
        return p

    def _get_fk_tool_from_service(self, q: np.ndarray) -> Optional[Point]:
        if self.fk_client is None:
            return None
        if not self.fk_client.service_is_ready():
            if self._fk_service_warn_count < 3:
                self.get_logger().warn("FK service /compute_fk is not ready, fallback to analytic FK markers.")
                self._fk_service_warn_count += 1
            return None

        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        fk_link = self.fk_vis_ee_link if self.fk_vis_ee_link else self.ik_link_name
        req.fk_link_names = [fk_link if fk_link else "tool0"]
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState()
        req.robot_state.joint_state.name = list(self.joint_names)
        req.robot_state.joint_state.position = [float(v) for v in np.asarray(q, dtype=float).reshape(-1)]

        fut = self.fk_client.call_async(req)
        deadline = time.monotonic() + max(self._fk_timeout_sec, 0.05)
        while (not fut.done()) and (time.monotonic() < deadline):
            time.sleep(0.003)
        if not fut.done():
            if self._fk_service_warn_count < 5:
                self.get_logger().warn("FK future timeout, fallback to analytic FK markers.")
                self._fk_service_warn_count += 1
            return None
        try:
            resp = fut.result()
        except Exception:  # pylint: disable=broad-except
            if self._fk_service_warn_count < 5:
                self.get_logger().warn("FK future raised exception, fallback to analytic FK markers.")
                self._fk_service_warn_count += 1
            return None
        if resp is None:
            if self._fk_service_warn_count < 5:
                self.get_logger().warn("FK response is None, fallback to analytic FK markers.")
                self._fk_service_warn_count += 1
            return None
        if int(resp.error_code.val) != int(MoveItErrorCodes.SUCCESS) or (not resp.pose_stamped):
            if self._fk_service_warn_count < 5:
                self.get_logger().warn(
                    f"FK failed: code={int(resp.error_code.val)}, fallback to analytic FK markers."
                )
                self._fk_service_warn_count += 1
            return None
        out = Point()
        out.x = float(resp.pose_stamped[0].pose.position.x)
        out.y = float(resp.pose_stamped[0].pose.position.y)
        out.z = float(resp.pose_stamped[0].pose.position.z)
        return out

    def _get_marker_fk_point(self, q: np.ndarray) -> Point:
        p = self._get_fk_tool_from_service(q)
        if p is not None:
            return p
        return self._get_fk_wrist(q)

    def _traj_to_marker_points(self, traj: np.ndarray) -> List[Point]:
        out: List[Point] = []
        arr = np.asarray(traj, dtype=float)
        if arr.ndim != 2 or arr.shape[0] < 3:
            return out
        for i in range(arr.shape[1]):
            out.append(self._get_marker_fk_point(arr[:, i]))
        return out

    @staticmethod
    def _marker_points_to_xyz_array(points: List[Point]) -> np.ndarray:
        if not points:
            return np.empty((0, 3), dtype=float)
        out = np.zeros((len(points), 3), dtype=float)
        for i, p in enumerate(points):
            out[i, 0] = float(p.x)
            out[i, 1] = float(p.y)
            out[i, 2] = float(p.z)
        return out

    def _export_offline_plot(
        self,
        nominal_traj: np.ndarray,
        via_points: Optional[np.ndarray],
        modulated_traj: np.ndarray,
    ) -> None:
        if not self.offline_export_plot_enable:
            return
        try:
            import matplotlib
            matplotlib.use("Agg", force=True)
            import matplotlib.pyplot as plt
            from matplotlib.patches import Circle
        except Exception as exc:  # pragma: no cover - optional dependency
            if not self._plot_export_warned:
                self.get_logger().warn(
                    f"offline plot export disabled: matplotlib unavailable ({exc})."
                )
                self._plot_export_warned = True
            return

        nominal_xyz = self._marker_points_to_xyz_array(
            self._traj_to_marker_points(np.asarray(nominal_traj, dtype=float))
        )
        modulated_xyz = self._marker_points_to_xyz_array(
            self._traj_to_marker_points(np.asarray(modulated_traj, dtype=float))
        )
        via_xyz = np.empty((0, 3), dtype=float)
        if via_points is not None:
            via_arr = np.asarray(via_points, dtype=float)
            if via_arr.ndim == 2 and via_arr.shape[1] > 0:
                via_xyz = self._marker_points_to_xyz_array(self._traj_to_marker_points(via_arr))

        obstacle_centers: List[np.ndarray] = []
        obstacle_radii: List[float] = []
        if self._plane_obstacles:
            for obs in self._plane_obstacles:
                p_xyz = self._map_uvz_to_xyz(obs["u"], obs["v"], obs.get("z", 0.0))
                obstacle_centers.append(np.asarray(p_xyz, dtype=float).reshape(3))
                obstacle_radii.append(float(max(obs["radius"], 1e-3)))
        elif self._analytic_obstacles:
            for obs in self._analytic_obstacles:
                obs_arr = np.asarray(obs, dtype=float).reshape(-1)
                if obs_arr.size != 4:
                    continue
                obstacle_centers.append(obs_arr[:3].copy())
                obstacle_radii.append(float(max(obs_arr[3], 1e-3)))

        if nominal_xyz.shape[0] == 0 and modulated_xyz.shape[0] == 0:
            self.get_logger().warn("offline plot export skipped: trajectory marker points are empty.")
            return

        out_dir = Path(self.offline_export_plot_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"offline_traj_compare_{time.strftime('%Y%m%d_%H%M%S')}.png"

        fig, axes = plt.subplots(1, 2, figsize=(13, 5))
        ax_xy, ax_xz = axes[0], axes[1]

        if nominal_xyz.shape[0] > 0:
            ax_xy.plot(nominal_xyz[:, 0], nominal_xyz[:, 1], "b--", lw=1.5, label="nominal")
            ax_xz.plot(nominal_xyz[:, 0], nominal_xyz[:, 2], "b--", lw=1.5, label="nominal")
        if modulated_xyz.shape[0] > 0:
            ax_xy.plot(modulated_xyz[:, 0], modulated_xyz[:, 1], "g-", lw=2.0, label="modulated")
            ax_xz.plot(modulated_xyz[:, 0], modulated_xyz[:, 2], "g-", lw=2.0, label="modulated")
        if via_xyz.shape[0] > 0:
            ax_xy.scatter(via_xyz[:, 0], via_xyz[:, 1], c="r", s=14, label="via")
            ax_xz.scatter(via_xyz[:, 0], via_xyz[:, 2], c="r", s=14, label="via")

        for center, radius in zip(obstacle_centers, obstacle_radii):
            ax_xy.add_patch(Circle((float(center[0]), float(center[1])), radius, color="orange", alpha=0.35))
            ax_xz.add_patch(Circle((float(center[0]), float(center[2])), radius, color="orange", alpha=0.35))

        ax_xy.set_title("XY Projection")
        ax_xy.set_xlabel("X (m)")
        ax_xy.set_ylabel("Y (m)")
        ax_xz.set_title("XZ Projection")
        ax_xz.set_xlabel("X (m)")
        ax_xz.set_ylabel("Z (m)")

        for ax in axes:
            ax.grid(True, alpha=0.25)
            ax.axis("equal")
            ax.legend(loc="best")

        fig.suptitle("Offline Hybrid Trajectory Comparison")
        fig.tight_layout()
        fig.savefig(out_path, dpi=180)
        plt.close(fig)
        self.get_logger().info(f"Offline trajectory comparison plot written: {out_path}")

    def _export_offline_eval_input(
        self,
        nominal_traj: np.ndarray,
        via_points: Optional[np.ndarray],
        via_times: Optional[np.ndarray],
        modulated_traj: np.ndarray,
        time_axis: np.ndarray,
        *,
        dispatch_result: str = "",
    ) -> None:
        if not self.offline_export_eval_input_enable:
            return
        out_dir = Path(self.offline_export_eval_input_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        out_path = out_dir / f"offline_eval_input_{stamp}.json"
        latest_path = out_dir / "offline_eval_input_latest.json"

        obstacles: List[Any] = self._scene_obstacles_for_export()

        via_arr = np.asarray(via_points, dtype=float) if via_points is not None else np.empty((len(self.joint_names), 0))
        via_time_arr = np.asarray(via_times, dtype=float).reshape(-1) if via_times is not None else np.empty((0,))
        payload = {
            "scenario_name": "offline_latest",
            "group_name": self.moveit_group_name,
            "joint_names": list(self.joint_names),
            "nominal_dt": float(self.nominal_dt),
            "time_axis": np.asarray(time_axis, dtype=float).reshape(-1).tolist(),
            "nominal_traj": np.asarray(nominal_traj, dtype=float).tolist(),
            "modulated_traj": np.asarray(modulated_traj, dtype=float).tolist(),
            "via_points": via_arr.tolist(),
            "via_times": via_time_arr.tolist(),
            "dispatch_result": str(dispatch_result),
            "dispatch_action_status": int(self._dispatch_action_status),
            "dispatch_error_code": int(self._dispatch_error_code),
            "execution_aborted": bool(self._execution_aborted),
            "rrt_stop_reason": max(self._rrt_stop_reason_counts, key=self._rrt_stop_reason_counts.get)
            if self._rrt_stop_reason_counts
            else "",
            "rrt_elapsed_ms": float(np.mean(self._rrt_time_ms_samples)) if self._rrt_time_ms_samples else 0.0,
            "rrt_collision_queries": int(np.mean(self._rrt_collision_queries_samples)) if self._rrt_collision_queries_samples else 0,
            "edge_resolution": float(self.postcheck_edge_resolution),
            "ee_link": str(self.fk_vis_ee_link),
            "base_frame": "base_link",
            "scene": {"obstacles": obstacles},
        }
        text = json.dumps(payload, ensure_ascii=False, indent=2)
        out_path.write_text(text, encoding="utf-8")
        latest_path.write_text(text, encoding="utf-8")
        self.get_logger().info(f"Offline evaluator input written: {out_path}")

    def _publish_debug_markers(
        self,
        nominal_traj: np.ndarray,
        via_points: Optional[np.ndarray],
        modulated_traj: np.ndarray,
    ) -> None:
        arr = MarkerArray()

        nominal = np.asarray(nominal_traj, dtype=float)
        modulated = np.asarray(modulated_traj, dtype=float)
        via = None if via_points is None else np.asarray(via_points, dtype=float)

        # Marker 0: nominal intent (blue line).
        m0 = Marker()
        m0.header.frame_id = "base_link"
        m0.ns = "planning_vis"
        m0.id = 0
        m0.type = Marker.LINE_STRIP
        m0.action = Marker.ADD
        m0.pose.orientation.w = 1.0
        m0.scale.x = 0.01
        m0.color.r = 0.0
        m0.color.g = 0.0
        m0.color.b = 1.0
        m0.color.a = 0.5
        m0.points = self._traj_to_marker_points(nominal)

        # Marker 1: via-points (red spheres).
        m1 = Marker()
        m1.header.frame_id = "base_link"
        m1.ns = "planning_vis"
        m1.id = 1
        m1.type = Marker.SPHERE_LIST
        m1.action = Marker.ADD
        m1.pose.orientation.w = 1.0
        m1.scale.x = 0.06
        m1.scale.y = 0.06
        m1.scale.z = 0.06
        m1.color.r = 1.0
        m1.color.g = 0.0
        m1.color.b = 0.0
        m1.color.a = 1.0
        if via is not None:
            m1.points = self._traj_to_marker_points(via)

        # Marker 2: modulated trajectory (green line).
        m2 = Marker()
        m2.header.frame_id = "base_link"
        m2.ns = "planning_vis"
        m2.id = 2
        m2.type = Marker.LINE_STRIP
        m2.action = Marker.ADD
        m2.pose.orientation.w = 1.0
        m2.scale.x = 0.015
        m2.color.r = 0.0
        m2.color.g = 1.0
        m2.color.b = 0.0
        m2.color.a = 0.8
        m2.points = self._traj_to_marker_points(modulated)

        extra_markers: List[Marker] = []
        if self._nominal_ee_points is not None and self._nominal_ee_points.shape[0] == 3:
            m3 = Marker()
            m3.header.frame_id = "base_link"
            m3.ns = "planning_vis"
            m3.id = 3
            m3.type = Marker.LINE_STRIP
            m3.action = Marker.ADD
            m3.pose.orientation.w = 1.0
            m3.scale.x = 0.01
            m3.color.r = 1.0
            m3.color.g = 0.6
            m3.color.b = 0.0
            m3.color.a = 0.8
            for i in range(self._nominal_ee_points.shape[1]):
                p = Point()
                p.x = float(self._nominal_ee_points[0, i])
                p.y = float(self._nominal_ee_points[1, i])
                p.z = float(self._nominal_ee_points[2, i])
                m3.points.append(p)
            extra_markers.append(m3)

        obstacle_centers: List[np.ndarray] = []
        obstacle_radii: List[float] = []
        if self._plane_obstacles:
            for obs in self._plane_obstacles:
                p_xyz = self._map_uvz_to_xyz(obs["u"], obs["v"], obs.get("z", 0.0))
                obstacle_centers.append(np.asarray(p_xyz, dtype=float).reshape(3))
                obstacle_radii.append(float(obs["radius"]))
        elif self._analytic_obstacles:
            for obs in self._analytic_obstacles:
                obs_arr = np.asarray(obs, dtype=float).reshape(-1)
                if obs_arr.size != 4:
                    continue
                obstacle_centers.append(obs_arr[:3].copy())
                obstacle_radii.append(float(max(obs_arr[3], 1e-3)))

        if obstacle_centers:
            m4 = Marker()
            m4.header.frame_id = "base_link"
            m4.ns = "planning_vis"
            m4.id = 4
            m4.type = Marker.SPHERE_LIST
            m4.action = Marker.ADD
            m4.pose.orientation.w = 1.0
            marker_radius = max(obstacle_radii) if obstacle_radii else 0.04
            marker_d = float(max(2.0 * marker_radius, 0.02))
            m4.scale.x = marker_d
            m4.scale.y = marker_d
            m4.scale.z = marker_d
            m4.color.r = 1.0
            m4.color.g = 1.0
            m4.color.b = 0.0
            m4.color.a = 0.6
            for p_xyz in obstacle_centers:
                p = Point()
                p.x = float(p_xyz[0])
                p.y = float(p_xyz[1])
                p.z = float(p_xyz[2])
                m4.points.append(p)
            extra_markers.append(m4)

        arr.markers = [m0, m1, m2] + extra_markers
        self.vis_pub.publish(arr)

    def _publish_debug_markers_with_backend(
        self,
        nominal_traj: np.ndarray,
        via_points: Optional[np.ndarray],
        modulated_traj: np.ndarray,
    ) -> None:
        if self._publish_debug_markers_cpp(nominal_traj, via_points, modulated_traj):
            return
        self._publish_debug_markers(nominal_traj, via_points, modulated_traj)

    def _collect_collision_indices_for_traj(self, traj: np.ndarray, where: str) -> List[int]:
        traj = np.asarray(traj, dtype=float)
        if traj.ndim != 2 or traj.shape[1] <= 0:
            return []

        cpp_required = self._cpp_collision_required_for_offline()
        if self.execution_mode == "offline" and self._cpp_runtime_requested():
            if not self._ensure_cpp_collision_runtime_available(where):
                if cpp_required:
                    raise RuntimeError(
                        f"cpp_bridge collision is required for {where} but runtime is unavailable: "
                        f"{self._last_cpp_collision_error or self._describe_cpp_collision_unavailable()}"
                    )
            else:
                motion = self._check_motion_batch_cpp(
                    traj.T,
                    check_edges=True,
                    where=where,
                )
                if motion is not None:
                    bad = set(
                        int(i)
                        for i, v in enumerate(motion["state_valid"].tolist())
                        if (not bool(v))
                    )
                    for i, v in enumerate(motion["edge_valid"].tolist()):
                        if not bool(v):
                            bad.add(int(i))
                            bad.add(int(i + 1))
                    return sorted(bad)
                batch = self._check_states_batch_cpp(traj.T)
                if batch is not None and batch.size == traj.shape[1]:
                    return [int(i) for i, v in enumerate(batch.tolist()) if (not bool(v))]
                reason = self._last_cpp_collision_error or "unknown batch failure"
                if cpp_required:
                    raise RuntimeError(
                        f"cpp_bridge collision batch failed in {where}: "
                        f"{reason}"
                    )
                self.get_logger().warn(
                    f"cpp_bridge collision batch is unavailable in {where}; "
                    f"fallback to python backend (reason={reason})."
                )
        elif cpp_required:
            raise RuntimeError(
                f"cpp_bridge collision is required for {where} but runtime_backend is not cpp_bridge."
            )

        if self._moveit_backend_active():
            try:
                danger_idx: List[int] = []
                with self._moveit_read_only_scene() as locked_scene:
                    test_state = self._create_moveit_robot_state()
                    checker = self._make_moveit_batch_checker(locked_scene, test_state)
                    checker(traj[:, 0])
                    for i in range(traj.shape[1]):
                        if not checker(traj[:, i]):
                            danger_idx.append(i)
                return danger_idx
            except Exception as exc:  # pylint: disable=broad-except
                if self.moveit_py_strict:
                    raise
                self._moveit_py_available = False
                self.get_logger().warn(
                    f"MoveItPy collision scan failed in {where}: {exc}. Fallback to analytic."
                )

        danger_idx: List[int] = []
        self._rrt_collision_checker = self.analytic_checker
        for i in range(traj.shape[1]):
            if not self.analytic_checker(traj[:, i]):
                danger_idx.append(i)
        return danger_idx

    def _collect_danger_indices_from_nominal(self) -> List[int]:
        traj = np.asarray(self.fmp_core.nominal_intent, dtype=float)
        return self._collect_collision_indices_for_traj(traj, "offline Step1 scan")

    def _build_time_aligned_via_trajectory(
        self,
        nominal_traj: np.ndarray,
        time_axis: np.ndarray,
        via_points: Optional[np.ndarray],
        via_times: Optional[np.ndarray],
    ) -> Optional[np.ndarray]:
        traj = np.asarray(nominal_traj, dtype=float)
        t_axis = np.asarray(time_axis, dtype=float).reshape(-1)
        if traj.ndim != 2 or traj.shape[1] != t_axis.size or t_axis.size < 2:
            return None
        if via_points is None or via_times is None:
            return None
        via = np.asarray(via_points, dtype=float)
        vt = np.asarray(via_times, dtype=float).reshape(-1)
        if via.ndim != 2 or via.shape[0] != traj.shape[0] or via.shape[1] != vt.size or vt.size <= 0:
            return None

        order = np.argsort(vt)
        vt = vt[order]
        via = via[:, order]

        t_min = float(t_axis[0])
        t_max = float(t_axis[-1])
        mask = (vt >= t_min) & (vt <= t_max)
        if not np.any(mask):
            return None
        vt = vt[mask]
        via = via[:, mask]
        if vt.size <= 0:
            return None

        t_knots = np.concatenate(([t_min], vt, [t_max]))
        q_knots = np.hstack((traj[:, [0]], via, traj[:, [-1]]))

        keep = np.ones(t_knots.size, dtype=bool)
        keep[1:] = np.diff(t_knots) > 1e-9
        t_knots = t_knots[keep]
        q_knots = q_knots[:, keep]
        if t_knots.size < 2:
            return None

        out = np.zeros_like(traj)
        for j in range(out.shape[0]):
            out[j, :] = np.interp(t_axis, t_knots, q_knots[j, :])
        return out

    def _path_to_vias(
        self,
        path_points: np.ndarray,
        t_start: float,
        t_end: float,
    ) -> Tuple[np.ndarray, np.ndarray, str]:
        if self.via_densify_enable:
            via, via_time = hm_compat.densify_path_to_vias(
                path_points,
                t_start,
                t_end,
                interp_dist=self.via_interp_dist,
                via_trim_sec=self.via_trim_sec,
            )
            return via, via_time, "dense"

        arr = np.asarray(path_points, dtype=float)
        if arr.ndim != 2 or arr.size == 0:
            return np.empty((0, 0), dtype=float), np.empty((0,), dtype=float), "raw"
        if arr.shape[1] < 2 and arr.shape[0] >= 2:
            arr = arr.T
        dim, count = arr.shape
        if count < 2:
            return np.empty((dim, 0), dtype=float), np.empty((0,), dtype=float), "raw"

        diff = np.diff(arr, axis=1)
        seg_len = np.linalg.norm(diff, axis=0)
        arc = np.concatenate(([0.0], np.cumsum(seg_len)))
        total = float(arc[-1])
        if total <= 1e-12:
            via_time = np.linspace(float(t_start), float(t_end), count, dtype=float)
        else:
            via_time = float(t_start) + (arc / total) * (float(t_end) - float(t_start))
        return arr, via_time, "raw"

    def _repair_modulated_with_via_projection(
        self,
        modulated_traj: np.ndarray,
        nominal_traj: np.ndarray,
        time_axis: np.ndarray,
        via_points: Optional[np.ndarray],
        via_times: Optional[np.ndarray],
        colliding_idx: List[int],
    ) -> Optional[np.ndarray]:
        if not self.offline_postcheck_repair_with_via:
            return None
        if not colliding_idx:
            return None
        via_full = self._build_time_aligned_via_trajectory(
            nominal_traj=nominal_traj,
            time_axis=time_axis,
            via_points=via_points,
            via_times=via_times,
        )
        if via_full is None:
            return None

        mod = np.asarray(modulated_traj, dtype=float)
        if mod.shape != via_full.shape:
            return None

        n = mod.shape[1]
        if n <= 0:
            return None

        margin = int(max(self.offline_postcheck_repair_margin_points, 0))
        mask = np.zeros((n,), dtype=bool)
        for idx in colliding_idx:
            i = int(idx)
            if i < 0 or i >= n:
                continue
            lo = max(0, i - margin)
            hi = min(n - 1, i + margin)
            mask[lo : hi + 1] = True
        if not np.any(mask):
            return None

        repaired = mod.copy()
        repaired[:, mask] = via_full[:, mask]
        return repaired

    def _refresh_nominal_trajectory_for_offline(self) -> bool:
        if self.nominal_source != "ee_plane":
            self._nominal_ee_points = None
            self._ee_metrics_last = {
                "ee_plane_deviation_mean": 0.0,
                "ee_plane_deviation_max": 0.0,
                "obstacle_clearance_min": 0.0,
                "obstacle_clearance_p95": 0.0,
            }
            return True

        if not self.ik_client.service_is_ready():
            self.get_logger().error("IK service /compute_ik is not ready.")
            return False
        solved = self._build_joint_nominal_from_ee_path()
        if solved is None:
            self.get_logger().error("Failed to build joint nominal trajectory from ee_plane path.")
            return False
        q_nominal, t_axis = solved
        if not self.fmp_core.set_nominal_trajectory(q_nominal, t_axis):
            self.get_logger().error("Failed to update FMP model with ee_plane nominal trajectory.")
            return False
        self.get_logger().info(
            f"ee_plane nominal trajectory prepared: points={q_nominal.shape[1]}, ik_fail_ratio={self._ik_fail_ratio_last:.3f}"
        )
        return True

    def _run_matlab_compat_replan(
        self,
        nominal_state: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
        _ = nominal_state
        nominal_traj = np.asarray(self.fmp_core.nominal_intent, dtype=float)
        time_axis = np.asarray(self.fmp_core.time_axis, dtype=float)
        n_points = int(nominal_traj.shape[1])

        danger_indices = self._collect_danger_indices_from_nominal()
        segments = hm_compat.extract_danger_segments(
            danger_indices,
            n_points,
            gap=self.segment_gap,
            pad=self.segment_pad,
        )
        if not segments:
            empty = np.empty((nominal_traj.shape[0], 0), dtype=float)
            return empty, np.empty((0,), dtype=float), {
                "segment_count": 0,
                "chosen_budget": 0,
                "rrt_stop_reasons": [],
                "rrt_iter_used": 0.0,
                "rrt_collision_queries": 0.0,
                "rrt_time_ms": 0.0,
                "timeout_hit": False,
            }

        if self.refine_fixed_budget >= 0:
            chosen_budget = int(self.refine_fixed_budget)
        else:
            idx_start0, idx_goal0 = segments[0]
            local_start0 = nominal_traj[:, idx_start0]
            local_goal0 = nominal_traj[:, idx_goal0]
            intent_local0 = nominal_traj[:, idx_start0 : idx_goal0 + 1]

            def _evaluate_budget(budget: int) -> Dict[str, Any]:
                return self._run_rrt_with_collision_backend(
                    detailed=True,
                    start=local_start0,
                    goal=local_goal0,
                    intent_path=intent_local0.T,
                    t_start=float(time_axis[idx_start0]),
                    t_end=float(time_axis[idx_goal0]),
                    refine_budget=int(budget),
                    **self._rrt_detail_sampling,
                )

            chosen_budget, _ = hm_compat.auto_select_refine_budget(
                self.refine_budget_candidates,
                _evaluate_budget,
            )

        via_points_all: List[np.ndarray] = []
        via_times_all: List[np.ndarray] = []
        stop_reasons: List[str] = []
        iter_values: List[float] = []
        query_values: List[float] = []
        time_values: List[float] = []
        timeout_hit = False

        for idx_start, idx_goal in segments:
            local_start = nominal_traj[:, idx_start]
            local_goal = nominal_traj[:, idx_goal]
            intent_local = nominal_traj[:, idx_start : idx_goal + 1]
            t_start = float(time_axis[idx_start])
            t_end = float(time_axis[idx_goal])

            detailed = self._run_rrt_with_collision_backend(
                detailed=True,
                start=local_start,
                goal=local_goal,
                intent_path=intent_local.T,
                t_start=t_start,
                t_end=t_end,
                refine_budget=chosen_budget,
                **self._rrt_detail_sampling,
            )
            stop_reasons.append(str(detailed.get("stop_reason", "unknown")))

            meta = detailed.get("meta", {})
            iter_values.append(float(meta.get("iter_used", 0.0)))
            query_values.append(float(meta.get("collision_queries", 0.0)))
            time_values.append(float(meta.get("time_ms", 0.0)))
            timeout_hit = timeout_hit or bool(meta.get("timeout_hit", False))

            path_refine = np.asarray(detailed.get("path_refine", np.empty((0, 0))), dtype=float)
            path_first = np.asarray(detailed.get("path_first", np.empty((0, 0))), dtype=float)
            path_for_via = path_refine if path_refine.size > 0 else path_first
            if path_for_via.size == 0:
                continue

            dense_via, dense_time, via_mode = self._path_to_vias(path_for_via, t_start, t_end)
            if dense_via.size == 0 or dense_time.size == 0:
                continue
            self.get_logger().debug(
                f"Phase B local path converted to {via_mode} via points: "
                f"{dense_via.shape[1]} (raw_path_points={path_for_via.shape[1]})"
            )
            via_points_all.append(dense_via)
            via_times_all.append(dense_time)

        if not via_points_all:
            empty = np.empty((nominal_traj.shape[0], 0), dtype=float)
            return empty, np.empty((0,), dtype=float), {
                "segment_count": len(segments),
                "chosen_budget": int(chosen_budget),
                "rrt_stop_reasons": stop_reasons,
                "rrt_iter_used": float(np.mean(iter_values)) if iter_values else 0.0,
                "rrt_collision_queries": float(np.mean(query_values)) if query_values else 0.0,
                "rrt_time_ms": float(np.mean(time_values)) if time_values else 0.0,
                "timeout_hit": bool(timeout_hit),
            }

        via_points = np.hstack(via_points_all)
        via_times = np.concatenate(via_times_all)
        order = np.argsort(via_times)
        via_times = via_times[order]
        via_points = via_points[:, order]
        if self.via_global_dedup_enable and via_times.size > 0:
            keep = np.ones(via_times.size, dtype=bool)
            keep[1:] = np.diff(via_times) > 1e-9
            via_times = via_times[keep]
            via_points = via_points[:, keep]

        return via_points, via_times, {
            "segment_count": len(segments),
            "chosen_budget": int(chosen_budget),
            "rrt_stop_reasons": stop_reasons,
            "rrt_iter_used": float(np.mean(iter_values)) if iter_values else 0.0,
            "rrt_collision_queries": float(np.mean(query_values)) if query_values else 0.0,
            "rrt_time_ms": float(np.mean(time_values)) if time_values else 0.0,
            "timeout_hit": bool(timeout_hit),
        }

    def _offline_once_timer_cb(self) -> None:
        if self._offline_once_timer is not None:
            self._offline_once_timer.cancel()
            self._offline_once_timer = None
        ok = self.plan_and_execute_offline()
        try:
            self._write_benchmark_results()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Write offline benchmark results failed: {exc}")
        if ok:
            self.get_logger().info("Offline planning pipeline finished.")
        else:
            self.get_logger().error("Offline planning pipeline failed.")

    def _compute_vel_acc_offline(self, q: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        n = q.shape[1]
        v = np.zeros_like(q)
        a = np.zeros_like(q)

        for k in range(1, n):
            dt = max(t[k] - t[k - 1], 1e-6)
            v[:, k] = (q[:, k] - q[:, k - 1]) / dt
        v[:, 0] = 0.0

        for k in range(1, n):
            dt = max(t[k] - t[k - 1], 1e-6)
            a[:, k] = (v[:, k] - v[:, k - 1]) / dt
        a[:, 0] = 0.0

        v = np.clip(v, -self.vel_limits[:, None], self.vel_limits[:, None])
        a = np.clip(a, -self.acc_limits[:, None], self.acc_limits[:, None])
        return v, a

    def _execute_trajectory_offline_python(
        self,
        trajectory_matrix: np.ndarray,
        *,
        nominal_dt_override: Optional[float] = None,
    ) -> str:
        traj = np.asarray(trajectory_matrix, dtype=float)
        if traj.ndim == 1:
            traj = traj.reshape(1, -1)
        if traj.shape[0] != len(self.joint_names):
            self.get_logger().error(
                f"execute_trajectory_offline rejected: expected {len(self.joint_names)}xN, got {traj.shape}"
            )
            return "rejected_bad_shape"
        self._update_dispatch_trajectory_diag(traj)
        if not self._validate_offline_dispatch_start(traj, where="python offline dispatch"):
            return "failed_start_state_mismatch"
        if self.offline_stitch_start_from_current:
            traj = np.hstack([self._q_now.reshape(-1, 1), traj])
            self._update_dispatch_trajectory_diag(traj)
            if traj.shape[1] >= 2:
                start_delta_max = float(np.max(np.abs(traj[:, 1] - traj[:, 0])))
                self.get_logger().warn(
                    "Offline start stitching is explicitly enabled: "
                    f"stitched start delta max_abs={start_delta_max:.4f} rad"
                )

        n_points = traj.shape[1]
        if n_points < 2:
            traj = np.hstack([traj, traj])
            n_points = 2

        dt_nominal = float(self.nominal_dt if nominal_dt_override is None else nominal_dt_override)
        t = self._build_time_array(n_points, dt_nominal)
        t = self._scale_time_for_limits(traj, t)
        self._update_dispatch_trajectory_diag(traj, duration_sec=float(t[-1]) if t.size else 0.0)
        if self.time_param_backend == "ruckig" and ruckig_lib is not None:
            vel, acc = self._time_parameterize(traj, t)
        else:
            vel, acc = self._compute_vel_acc_offline(traj, t)

        jerk_proxy = self._compute_jerk_proxy(acc, t)
        self._trajectory_jerk_proxy_samples.append(jerk_proxy)
        if (not self._jerk_warned) and jerk_proxy > float(max(self.jerk_warn_threshold, 0.0)):
            self.get_logger().warn(
                f"Offline trajectory jerk proxy is high: {jerk_proxy:.3f} > threshold {self.jerk_warn_threshold:.3f}"
            )
            self._jerk_warned = True

        traj_msg = JointTrajectory()
        traj_msg.joint_names = list(self.joint_names)
        for k in range(n_points):
            pt = JointTrajectoryPoint()
            pt.positions = traj[:, k].tolist()
            pt.velocities = vel[:, k].tolist()
            pt.accelerations = acc[:, k].tolist()
            sec = int(t[k])
            nanosec = int((t[k] - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
            traj_msg.points.append(pt)

        if not self.trajectory_action_client.server_is_ready():
            self.get_logger().error("Action server is not ready for offline execution.")
            return "failed_server_offline"

        self._send_goal_async(traj_msg)
        self._last_send_monotonic = time.monotonic()
        self._last_sent_traj = traj.copy()
        self._last_sent_signature = self._trajectory_signature(traj)
        self._last_sent_meta = {"danger": True, "offline_mode": True}
        return "sent_success"

    def execute_trajectory_offline(self, trajectory_matrix: np.ndarray) -> str:
        self._record_dispatch_failure(status=-1, error_code=0, error_string="", aborted=False)
        self._trajectory_estimated_max_velocity = 0.0
        self._trajectory_estimated_max_acceleration = 0.0
        self._trajectory_time_scaling_factor = 1.0
        self._start_state_error_max = 0.0
        self._start_state_error_norm = 0.0
        traj_for_dispatch = np.asarray(trajectory_matrix, dtype=float)
        if traj_for_dispatch.ndim == 1:
            traj_for_dispatch = traj_for_dispatch.reshape(1, -1)
        if traj_for_dispatch.ndim != 2 or traj_for_dispatch.shape[0] != len(self.joint_names):
            self._dispatch_error_string = f"invalid dispatch trajectory shape {traj_for_dispatch.shape}"
            return "rejected_bad_shape"
        ok_dyn, nominal_dt_override = self._apply_dispatch_dynamics_safety(traj_for_dispatch)
        self._update_dispatch_trajectory_diag(
            traj_for_dispatch,
            duration_sec=float(max(traj_for_dispatch.shape[1] - 1, 0) * max(nominal_dt_override, 1e-6)),
        )
        if not ok_dyn:
            return "failed_dispatch_safety_gate"
        if self._use_cpp_runtime_for_offline():
            decision_cpp = self._dispatch_trajectory_cpp_offline(
                traj_for_dispatch,
                nominal_dt_override=nominal_dt_override,
            )
            if decision_cpp == "sent_success":
                traj = np.asarray(traj_for_dispatch, dtype=float)
                self._last_send_monotonic = time.monotonic()
                self._last_sent_traj = traj.copy()
                self._last_sent_signature = self._trajectory_signature(traj)
                self._last_sent_meta = {"danger": True, "offline_mode": True, "runtime_backend": "cpp_bridge"}
                return "sent_success"
            hard_fail_codes = {
                "failed_stale_state",
                "failed_action_aborted",
                "failed_action_canceled",
                "failed_action_error_code",
                "failed_action_result_timeout",
                "failed_send_goal_timeout",
                "rejected_by_action_server",
                "failed_action_unknown",
                "failed_start_state_mismatch",
                "failed_dispatch_safety_gate",
            }
            if decision_cpp in hard_fail_codes:
                self.get_logger().error(
                    f"cpp_bridge dispatch returned {decision_cpp}; aborting instead of unsafe fallback."
                )
                return decision_cpp
            if self.offline_wait_action_result:
                self.get_logger().error(
                    f"cpp_bridge dispatch returned {decision_cpp}; offline_wait_action_result is enabled, no fallback."
                )
                return decision_cpp
            self.get_logger().warn(
                f"cpp_bridge dispatch failed ({decision_cpp}), fallback to python trajectory dispatch."
            )
        return self._execute_trajectory_offline_python(
            traj_for_dispatch,
            nominal_dt_override=nominal_dt_override,
        )

    def plan_and_execute_offline(self) -> bool:
        self.get_logger().info("========== Offline Planning Pipeline ==========")
        if not self._refresh_nominal_trajectory_for_offline():
            return False
        nominal_traj = np.asarray(self.fmp_core.nominal_intent, dtype=float)
        time_axis = np.asarray(self.fmp_core.time_axis, dtype=float)
        if nominal_traj.ndim != 2 or nominal_traj.shape[1] <= 1:
            self.get_logger().error("Nominal trajectory is empty.")
            return False

        if not self._assert_cpp_collision_bridge_ready(
            "offline pipeline preflight",
            nominal_traj[:, 0],
        ):
            return False

        if not self._sync_obstacle_config_to_planning_scene("offline pre-scan"):
            self.get_logger().error(
                "Offline planning aborted because obstacle_config_file could not be synced "
                "to MoveIt PlanningScene."
            )
            return False

        n_points = int(nominal_traj.shape[1])
        self.get_logger().info("Step 1/4: global collision scan.")
        try:
            danger_indices = self._collect_danger_indices_from_nominal()
        except RuntimeError as exc:
            self.get_logger().error(f"Step 1/4 failed: {exc}")
            return False

        danger_index_count = int(len(danger_indices))
        nominal_index_count = max(n_points - danger_index_count, 0)
        self._danger_count += danger_index_count
        self._nominal_count += nominal_index_count
        if danger_index_count > 0:
            self._danger_event_count += 1
        self.get_logger().info(
            "Offline scan counters: "
            f"nominal_count_added={nominal_index_count}, "
            f"danger_count_added={danger_index_count}, "
            f"danger_event_added={1 if danger_index_count > 0 else 0}"
        )

        via_points_all: List[np.ndarray] = []
        via_times_all: List[np.ndarray] = []
        if danger_indices:
            self.get_logger().warn(
                f"Collision points detected on nominal trajectory: {len(danger_indices)}"
            )
            segments = hm_compat.extract_danger_segments(
                danger_indices,
                n_points,
                gap=self.segment_gap,
                pad=self.segment_pad,
            )
            self.get_logger().info(f"Step 2/4: planning {len(segments)} collision segments.")
            if not segments:
                self.get_logger().error(
                    "Nominal trajectory has collision indices but no danger segments were extracted; "
                    "this run is not a valid local-planner comparison."
                )
                return False
            self._rrt_call_count += int(len(segments))
            if self._rrt_call_count <= 0:
                self.get_logger().error(
                    "rrt_call_count is zero after danger segment extraction; "
                    "this run is not a valid OMPL/local-planner comparison."
                )
                return False
            chosen_budget = int(self.refine_fixed_budget)
            if chosen_budget < 0:
                chosen_budget = int(self.refine_budget_candidates[0]) if self.refine_budget_candidates else 100
            self._segment_count_samples.append(float(len(segments)))
            self._refine_budget_samples.append(float(chosen_budget))

            for seg_idx, (idx_start, idx_goal) in enumerate(segments):
                local_start = nominal_traj[:, idx_start]
                local_goal = nominal_traj[:, idx_goal]
                intent_local = nominal_traj[:, idx_start : idx_goal + 1]
                t_start = float(time_axis[idx_start])
                t_end = float(time_axis[idx_goal])

                self.get_logger().info(
                    f"Segment {seg_idx + 1}/{len(segments)}: idx [{idx_start}, {idx_goal}]"
                )
                detailed = None
                if self.use_cpp_local_planner:
                    detailed = self._plan_local_segment_cpp(
                        start=local_start,
                        goal=local_goal,
                        intent_path=intent_local.T,
                        t_start=t_start,
                        t_end=t_end,
                        segment_label=f"segment {seg_idx + 1}/{len(segments)}",
                    )
                    if detailed is None and not self.allow_cpp_local_planner_fallback:
                        partial_via = np.hstack(via_points_all) if via_points_all else None
                        self._publish_debug_markers_with_backend(nominal_traj, partial_via, nominal_traj)
                        self.get_logger().error(
                            "C++ local planner unavailable and fallback is disabled: "
                            f"{self._last_cpp_collision_error or 'unknown error'}"
                        )
                        return False
                    if detailed is not None:
                        maybe_path = np.asarray(detailed.get("path_refine", np.empty((0, 0))), dtype=float)
                        if maybe_path.size == 0:
                            if not self.allow_cpp_local_planner_fallback:
                                partial_via = np.hstack(via_points_all) if via_points_all else None
                                self._publish_debug_markers_with_backend(nominal_traj, partial_via, nominal_traj)
                                self.get_logger().error(
                                    f"Segment {seg_idx + 1} failed in C++ local planner and fallback is disabled."
                                )
                                return False
                            self.get_logger().warn(
                                f"Segment {seg_idx + 1}: C++ local planner failed, fallback to Python RRT."
                            )
                            detailed = None

                if detailed is None:
                    try:
                        detailed = self._run_rrt_with_collision_backend(
                            detailed=True,
                            start=local_start,
                            goal=local_goal,
                            intent_path=intent_local.T,
                            t_start=t_start,
                            t_end=t_end,
                            refine_budget=chosen_budget,
                            **self._rrt_detail_sampling,
                        )
                    except RuntimeError as exc:
                        partial_via = np.hstack(via_points_all) if via_points_all else None
                        self._publish_debug_markers_with_backend(nominal_traj, partial_via, nominal_traj)
                        self.get_logger().error(
                            f"Segment {seg_idx + 1} failed before RRT output: {exc}"
                        )
                        return False

                stop_reason = str(detailed.get("stop_reason", "unknown"))
                self._rrt_stop_reason_counts[stop_reason] = (
                    self._rrt_stop_reason_counts.get(stop_reason, 0) + 1
                )
                meta = detailed.get("meta", {})
                iter_used = float(meta.get("iter_used", 0.0))
                time_ms = float(meta.get("time_ms", 0.0))
                collision_queries = float(meta.get("collision_queries", 0.0))
                timeout_hit = bool(meta.get("timeout_hit", False))
                self._rrt_iter_samples.append(iter_used)
                self._rrt_time_ms_samples.append(time_ms)
                self._rrt_collision_queries_samples.append(collision_queries)
                self._timing_samples["rrt_plan_ms"].append(time_ms)
                if timeout_hit:
                    self._rrt_timeout_count += 1

                path_refine = np.asarray(detailed.get("path_refine", np.empty((0, 0))), dtype=float)
                path_first = np.asarray(detailed.get("path_first", np.empty((0, 0))), dtype=float)
                path_for_via = path_refine if path_refine.size > 0 else path_first
                if path_for_via.size == 0:
                    # Keep visualization available even when this offline segment fails.
                    partial_via = np.hstack(via_points_all) if via_points_all else None
                    self._publish_debug_markers_with_backend(nominal_traj, partial_via, nominal_traj)
                    self.get_logger().error(
                        "Segment "
                        f"{seg_idx + 1} failed: no valid RRT path "
                        f"(stop_reason={stop_reason}, "
                        f"iter_used={iter_used}, "
                        f"time_ms={time_ms:.1f}, "
                        f"collision_queries={collision_queries}, "
                        f"timeout_hit={timeout_hit})."
                    )
                    return False
                if not self._validate_local_path_with_cpp_motion(
                    path_for_via,
                    f"Segment {seg_idx + 1} local path post-check",
                ):
                    partial_via = np.hstack(via_points_all) if via_points_all else None
                    self._publish_debug_markers_with_backend(nominal_traj, partial_via, nominal_traj)
                    self.get_logger().error(
                        f"Segment {seg_idx + 1} failed: local path did not pass CheckMotionBatch."
                    )
                    return False

                dense_via, dense_time, via_mode = self._path_to_vias(path_for_via, t_start, t_end)
                if dense_via.size <= 0 or dense_time.size <= 0:
                    partial_via = np.hstack(via_points_all) if via_points_all else None
                    self._publish_debug_markers_with_backend(nominal_traj, partial_via, nominal_traj)
                    self.get_logger().error(f"Segment {seg_idx + 1} failed: via conversion returned empty.")
                    return False
                self.get_logger().info(
                    f"Segment {seg_idx + 1}: {via_mode} via points={dense_via.shape[1]} "
                    f"(raw_path_points={path_for_via.shape[1]})"
                )
                via_points_all.append(dense_via)
                via_times_all.append(dense_time)
        else:
            self.get_logger().info("Nominal trajectory is collision-free.")

        if via_points_all:
            global_via_points = np.hstack(via_points_all)
            global_via_times = np.concatenate(via_times_all)
            via_count_pre_sort = int(global_via_times.size)
            order = np.argsort(global_via_times)
            global_via_times = global_via_times[order]
            global_via_points = global_via_points[:, order]
            if self.via_global_dedup_enable and global_via_times.size > 1:
                via_count_pre_dedup = int(global_via_times.size)
                keep = np.ones(global_via_times.size, dtype=bool)
                keep[1:] = np.diff(global_via_times) > 1e-9
                global_via_times = global_via_times[keep]
                global_via_points = global_via_points[:, keep]
                self.get_logger().info(
                    f"Global via merged: {via_count_pre_sort} -> {via_count_pre_dedup} -> "
                    f"{int(global_via_times.size)} (sort + dedup)"
                )
            else:
                self.get_logger().info(
                    f"Global via merged: {via_count_pre_sort} points (dedup disabled)"
                )
        else:
            global_via_points = None
            global_via_times = None
            self.get_logger().info("Global via merged: 0 points")

        self.get_logger().info("Step 3/4: global FMP modulation.")
        t_mod0 = time.monotonic()
        modulated = self.fmp_core.modulate(
            nominal_point=nominal_traj[:, 0].tolist(),
            via_points=global_via_points,
            via_times=global_via_times,
        )
        self._ee_metrics_last = self._compute_ee_metrics(modulated)
        mod_ms = (time.monotonic() - t_mod0) * 1000.0
        self._timing_samples["fmp_modulate_ms"].append(float(mod_ms))
        self.get_logger().info(f"FMP modulation finished in {mod_ms:.1f} ms.")
        if int(modulated.shape[1]) < 2:
            self.get_logger().error(
                "Offline post-check rejected modulated trajectory: "
                f"trajectory_point_count={int(modulated.shape[1])} < 2."
            )
            self._record_postcheck_metrics(None, passed=False)
            self._publish_debug_markers_with_backend(nominal_traj, global_via_points, modulated)
            self._export_offline_eval_input(
                nominal_traj,
                global_via_points,
                global_via_times,
                modulated,
                time_axis,
                dispatch_result="failed_postcheck_too_short",
            )
            return False
        if self.nominal_source == "ee_plane":
            self.get_logger().info(
                "EE metrics: "
                f"dev_mean={self._ee_metrics_last['ee_plane_deviation_mean']:.4f}, "
                f"dev_max={self._ee_metrics_last['ee_plane_deviation_max']:.4f}, "
                f"clearance_min={self._ee_metrics_last['obstacle_clearance_min']:.4f}"
            )

        if not self.offline_postcheck_collision_enable:
            self.get_logger().warn(
                "offline_postcheck_collision_enable=false is ignored in offline strict safety mode."
            )
        post_motion = self._check_motion_batch_cpp(
            modulated.T,
            check_edges=True,
            where="offline post-modulation strict check",
            edge_resolution=self.postcheck_edge_resolution,
        )
        postcheck_ok = self._strict_motion_check_passed(
            post_motion,
            expected_states=int(modulated.shape[1]),
            require_edges=True,
        )
        self._record_postcheck_metrics(post_motion, passed=postcheck_ok)
        if not postcheck_ok:
            self._log_motion_check_failure(
                "Offline post-check rejected modulated trajectory",
                post_motion,
                expected_states=int(modulated.shape[1]),
            )
            invalid_idx = self._motion_invalid_indices(post_motion)
            repaired = self._repair_modulated_with_via_projection(
                modulated,
                nominal_traj,
                time_axis,
                global_via_points,
                global_via_times,
                invalid_idx,
            )
            if repaired is not None:
                self.get_logger().warn(
                    "Offline post-check repair: replacing "
                    f"{len(invalid_idx)} invalid index neighborhoods with time-aligned via projection."
                )
                repair_motion = self._check_motion_batch_cpp(
                    repaired.T,
                    check_edges=True,
                    where="offline postcheck repair strict check",
                    edge_resolution=self.postcheck_edge_resolution,
                )
                repair_ok = self._strict_motion_check_passed(
                    repair_motion,
                    expected_states=int(repaired.shape[1]),
                    require_edges=True,
                )
                self._record_postcheck_metrics(repair_motion, passed=repair_ok)
                if repair_ok:
                    modulated = repaired
                    post_motion = repair_motion
                    postcheck_ok = True
                    self.get_logger().info(
                        "Offline post-check repair succeeded; repaired trajectory will be dispatched."
                    )
                else:
                    self._log_motion_check_failure(
                        "Offline post-check repair rejected trajectory",
                        repair_motion,
                        expected_states=int(repaired.shape[1]),
                    )
            if self.execute_only_if_postcheck_passed:
                if not postcheck_ok:
                    self._publish_debug_markers_with_backend(nominal_traj, global_via_points, modulated)
                    self._export_offline_eval_input(
                        nominal_traj,
                        global_via_points,
                        global_via_times,
                        modulated,
                        time_axis,
                        dispatch_result="failed_postcheck",
                    )
                    return False
            if not postcheck_ok:
                self.get_logger().warn(
                    "Unsafe experimental mode: dispatching trajectory even though post-check failed."
                )

        self._publish_debug_markers_with_backend(nominal_traj, global_via_points, modulated)

        self.get_logger().info("Step 4/4: execute full offline trajectory.")
        decision = self.execute_trajectory_offline(modulated)
        self.get_logger().info(f"Offline execution dispatch result: {decision}")
        self._export_offline_eval_input(
            nominal_traj,
            global_via_points,
            global_via_times,
            modulated,
            time_axis,
            dispatch_result=decision,
        )
        if (
            decision == "sent_success"
            and self.offline_wait_action_result
            and (not self._use_cpp_runtime_for_offline())
        ):
            action_wait = self._wait_offline_action_result_python(self.offline_action_result_timeout_sec)
            if action_wait != "succeeded":
                self.get_logger().error(
                    "Offline execution failed after dispatch: "
                    f"final_action_result={action_wait}"
                )
                try:
                    self._export_offline_plot(nominal_traj, global_via_points, modulated)
                except Exception as exc:  # pylint: disable=broad-except
                    self.get_logger().warn(f"Offline trajectory plot export failed: {exc}")
                return False
        try:
            self._export_offline_plot(nominal_traj, global_via_points, modulated)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f"Offline trajectory plot export failed: {exc}")
        return decision == "sent_success"

    def orchestrator_loop(self) -> None:
        """
        Two-phase non-blocking orchestrator:
        Phase A: send collision request and return immediately.
        Phase B: poll collision result; when ready, run planning/modulation/execution.
        """
        loop_t0 = time.monotonic()
        nominal = list(self.current_nominal_point)
        via_points = None

        collision_free: Optional[bool] = None
        if self._moveit_backend_active():
            t_col0 = time.monotonic()
            try:
                collision_free = self._sync_moveit_collision_free(np.asarray(nominal, dtype=float))
                self._timing_samples["collision_check_ms"].append(
                    float((time.monotonic() - t_col0) * 1000.0)
                )
                self._orchestrator_phase_b_count += 1
                self._waiting_collision_result = False
                self._pending_collision_request_ts = None
            except Exception as exc:  # pylint: disable=broad-except
                if self.moveit_py_strict:
                    raise
                self._moveit_py_available = False
                self.get_logger().warn(
                    f"MoveItPy sync collision check failed: {exc}. Fallback to service checker."
                )

        if collision_free is None:
            # Phase A: no pending request -> send async check and return.
            if not self._waiting_collision_result:
                sent = self.scene_monitor.request_collision_check(nominal)
                if sent:
                    self._waiting_collision_result = True
                    self._pending_collision_request_ts = time.monotonic()
                    self.get_logger().debug("Phase A: collision check request sent.")
                else:
                    self.get_logger().debug("Phase A: collision request not sent this cycle.")
                return

            # Phase B: pending request -> poll once, keep non-blocking behavior.
            collision_free = self.scene_monitor.poll_collision_result()
            if collision_free is None:
                self.get_logger().debug("Phase B: collision result pending, skip this tick.")
                return

            self._waiting_collision_result = False
            self._orchestrator_phase_b_count += 1

            now_ts = time.monotonic()
            if self._pending_collision_request_ts is not None:
                collision_check_ms = (now_ts - self._pending_collision_request_ts) * 1000.0
                self._timing_samples["collision_check_ms"].append(float(collision_check_ms))
            self._pending_collision_request_ts = None

        now_ts = time.monotonic()
        # Danger hysteresis to avoid oscillatory triggering.
        now = now_ts
        raw_danger = not collision_free
        if raw_danger:
            self._danger_latched_until = now + self.danger_hold_sec
        if self.danger_replan_hysteresis:
            danger = raw_danger or (now < self._danger_latched_until)
        else:
            danger = raw_danger

        danger_event = danger and (not self._last_danger_flag)
        self._last_danger_flag = danger
        if danger:
            self._danger_count += 1
        else:
            self._nominal_count += 1
        if danger_event:
            self._danger_event_count += 1

        if not danger:
            self.get_logger().info("Phase B: nominal state collision-free. Skip local replan.")
            t_mod0 = time.monotonic()
            modulated = self.fmp_core.modulate(nominal_point=nominal, via_points=None)
            mod_ms = (time.monotonic() - t_mod0) * 1000.0
            self._timing_samples["fmp_modulate_ms"].append(float(mod_ms))
            meta = {
                "danger": False,
                "danger_event": False,
                "min_obstacle_distance": 1.0,
                "risk_window_sec": 0.0,
            }
        else:
            self.get_logger().warn("Phase B: danger detected. Trigger IntentBiasedRRT -> FMP modulation.")
            self._rrt_call_count += 1

            t_rrt0 = time.monotonic()
            if self.hybrid_mode == "matlab_compat":
                via_points, via_times, compat_meta = self._run_matlab_compat_replan(
                    nominal_state=np.array(nominal, dtype=float)
                )
                rrt_meta = {
                    "iter_used": float(compat_meta.get("rrt_iter_used", 0.0)),
                    "time_ms": float(compat_meta.get("rrt_time_ms", 0.0)),
                    "collision_queries": float(compat_meta.get("rrt_collision_queries", 0.0)),
                    "timeout_hit": bool(compat_meta.get("timeout_hit", False)),
                }
                segment_count = int(compat_meta.get("segment_count", 0))
                chosen_budget = int(compat_meta.get("chosen_budget", 0))
                stop_reasons = [str(x) for x in compat_meta.get("rrt_stop_reasons", [])]
                self._segment_count_samples.append(float(segment_count))
                self._refine_budget_samples.append(float(chosen_budget))
                for reason in stop_reasons:
                    self._rrt_stop_reason_counts[reason] = self._rrt_stop_reason_counts.get(reason, 0) + 1
            else:
                via_points, via_times, rrt_meta = self._run_rrt_with_collision_backend(
                    detailed=False,
                    start=np.array(nominal, dtype=float),
                    goal=self.fmp_core.nominal_intent[:, -1],
                    intent_path=self.fmp_core.nominal_intent.T,
                    t_start=0.0,
                    t_end=10.0,
                )
                self._segment_count_samples.append(1.0)
                self._refine_budget_samples.append(0.0)

            rrt_ms = (time.monotonic() - t_rrt0) * 1000.0
            self._timing_samples["rrt_plan_ms"].append(float(rrt_ms))
            self._rrt_iter_samples.append(float(rrt_meta.get("iter_used", 0)))
            self._rrt_time_ms_samples.append(float(rrt_meta.get("time_ms", 0.0)))
            self._rrt_collision_queries_samples.append(float(rrt_meta.get("collision_queries", 0)))
            if bool(rrt_meta.get("timeout_hit", False)):
                self._rrt_timeout_count += 1
            self.get_logger().info(
                f"RRT Iterations: {rrt_meta['iter_used']}, Time: {rrt_meta['time_ms']:.1f}ms"
            )
            t_mod0 = time.monotonic()
            modulated = self.fmp_core.modulate(
                nominal_point=nominal,
                via_points=via_points,
                via_times=via_times,
            )
            mod_ms = (time.monotonic() - t_mod0) * 1000.0
            self._timing_samples["fmp_modulate_ms"].append(float(mod_ms))
            meta = {
                "danger": True,
                "danger_event": danger_event,
                "min_obstacle_distance": 0.0,
                "risk_window_sec": self.danger_hold_sec,
            }

        self._publish_debug_markers_with_backend(
            self.fmp_core.nominal_intent,
            via_points if danger else None,
            modulated,
        )

        t_exec0 = time.monotonic()
        decision = self.execute_trajectory(modulated, meta=meta)
        exec_ms = (time.monotonic() - t_exec0) * 1000.0
        self._timing_samples["execute_trajectory_ms"].append(float(exec_ms))
        total_ms = (time.monotonic() - loop_t0) * 1000.0
        self._timing_samples["orchestrator_total_ms"].append(float(total_ms))
        self.get_logger().debug(f"Dispatch decision: {decision}")

        if (not self._stats_written) and ((time.monotonic() - self._stats_start_monotonic) >= self._stats_window_sec):
            try:
                self._write_benchmark_results()
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"Write benchmark results failed: {exc}")
            self._stats_written = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntentHybridPlannerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Avoid extra rosout errors when context is already being torn down.
        pass
    finally:
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
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
