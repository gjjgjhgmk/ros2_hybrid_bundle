#!/usr/bin/env python3
import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from intent_hybrid_interfaces.srv import CheckMotionBatch
except Exception:  # pragma: no cover
    CheckMotionBatch = None

try:
    from moveit_msgs.msg import RobotState
    from moveit_msgs.srv import GetPositionFK
except Exception:  # pragma: no cover
    RobotState = None
    GetPositionFK = None

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


DEFAULT_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

EXPECTED_CHECK_MOTION_TYPE = "intent_hybrid_interfaces/srv/CheckMotionBatch"


def _bool_arg(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid bool value: {value}")


def _as_array(data: Any, name: str) -> np.ndarray:
    arr = np.asarray(data, dtype=float)
    if arr.ndim != 2:
        raise ValueError(f"{name} must be 2D, got shape {arr.shape}")
    return arr


def normalize_traj_dofxn(data: Any, dof: int, name: str) -> np.ndarray:
    arr = _as_array(data, name)
    if arr.shape[0] == dof:
        return arr
    if arr.shape[1] == dof:
        return arr.T
    raise ValueError(f"{name} shape mismatch: got {arr.shape}, dof={dof}")


def normalize_xyz_nx3(data: Any, name: str) -> np.ndarray:
    arr = _as_array(data, name)
    if arr.shape[1] == 3:
        return arr
    if arr.shape[0] == 3:
        return arr.T
    raise ValueError(f"{name} must be Nx3 or 3xN, got {arr.shape}")


def compute_joint_metrics(traj_dofxn: np.ndarray, dt: float) -> Dict[str, float]:
    traj = np.asarray(traj_dofxn, dtype=float)
    if traj.ndim != 2 or traj.shape[1] < 1:
        return {
            "joint_path_length": 0.0,
            "max_joint_delta": 0.0,
            "joint_jerk_integral": 0.0,
            "estimated_max_velocity": 0.0,
            "estimated_max_acceleration": 0.0,
        }
    d = np.diff(traj, axis=1)
    seg_norm = np.linalg.norm(d, axis=0) if d.size > 0 else np.zeros((0,), dtype=float)
    path_length = float(np.sum(seg_norm)) if seg_norm.size > 0 else 0.0
    max_joint_delta = float(np.max(np.abs(d))) if d.size > 0 else 0.0

    dt_safe = max(float(dt), 1e-6)
    vel = d / dt_safe if d.size > 0 else np.zeros((traj.shape[0], 0), dtype=float)
    acc = np.diff(vel, axis=1) / dt_safe if vel.shape[1] >= 2 else np.zeros((traj.shape[0], 0), dtype=float)
    jerk = np.diff(acc, axis=1) / dt_safe if acc.shape[1] >= 2 else np.zeros((traj.shape[0], 0), dtype=float)

    est_max_vel = float(np.max(np.abs(vel))) if vel.size > 0 else 0.0
    est_max_acc = float(np.max(np.abs(acc))) if acc.size > 0 else 0.0
    jerk_integral = float(np.sum(np.abs(jerk)) * dt_safe) if jerk.size > 0 else 0.0
    return {
        "joint_path_length": path_length,
        "max_joint_delta": max_joint_delta,
        "joint_jerk_integral": jerk_integral,
        "estimated_max_velocity": est_max_vel,
        "estimated_max_acceleration": est_max_acc,
    }


def _pick_rrt_meta(scenario: Dict[str, Any], benchmark: Dict[str, Any]) -> Dict[str, Any]:
    out = {
        "rrt_stop_reason": str(scenario.get("rrt_stop_reason", "")),
        "rrt_elapsed_ms": float(scenario.get("rrt_elapsed_ms", 0.0)),
        "rrt_collision_queries": int(scenario.get("rrt_collision_queries", 0)),
    }
    if out["rrt_stop_reason"]:
        return out
    rrt = benchmark.get("rrt", {})
    stop_counts = rrt.get("stop_reason_counts", {}) if isinstance(rrt, dict) else {}
    if isinstance(stop_counts, dict) and stop_counts:
        out["rrt_stop_reason"] = max(stop_counts.items(), key=lambda kv: kv[1])[0]
    out["rrt_elapsed_ms"] = float(rrt.get("time_ms", {}).get("mean", out["rrt_elapsed_ms"]) or 0.0)
    out["rrt_collision_queries"] = int(rrt.get("collision_queries", {}).get("mean", out["rrt_collision_queries"]) or 0)
    return out


def _pick_dispatch_meta(scenario: Dict[str, Any], benchmark: Dict[str, Any]) -> Dict[str, Any]:
    dispatch = benchmark.get("dispatch_safety", {}) if isinstance(benchmark, dict) else {}
    status = scenario.get("dispatch_action_status", dispatch.get("dispatch_action_status", -1))
    err_code = scenario.get("dispatch_error_code", dispatch.get("dispatch_error_code", 0))
    aborted = scenario.get("execution_aborted", dispatch.get("execution_aborted", False))
    return {
        "dispatch_action_status": int(status),
        "dispatch_error_code": int(err_code),
        "execution_aborted": bool(aborted),
    }


def _parse_obstacles(scene: Dict[str, Any]) -> List[Dict[str, float]]:
    out: List[Dict[str, float]] = []
    for ob in scene.get("obstacles", []):
        if isinstance(ob, dict):
            x = float(ob.get("x", 0.0))
            y = float(ob.get("y", 0.0))
            z = float(ob.get("z", 0.0))
            r = float(ob.get("radius", ob.get("r", 0.0)))
        else:
            arr = np.asarray(ob, dtype=float).reshape(-1)
            if arr.size < 4:
                continue
            x, y, z, r = float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3])
        out.append({"x": x, "y": y, "z": z, "radius": max(r, 0.0)})
    return out


def _clearance_stats(ee_xyz: np.ndarray, obstacles: List[Dict[str, float]]) -> Dict[str, float]:
    if ee_xyz.size == 0 or not obstacles:
        return {"min_clearance": 0.0, "mean_clearance": 0.0}
    vals: List[float] = []
    for p in ee_xyz:
        d = float("inf")
        for ob in obstacles:
            c = math.sqrt(
                (float(p[0]) - ob["x"]) ** 2
                + (float(p[1]) - ob["y"]) ** 2
                + (float(p[2]) - ob["z"]) ** 2
            ) - ob["radius"]
            d = min(d, c)
        vals.append(d)
    arr = np.asarray(vals, dtype=float)
    return {
        "min_clearance": float(np.min(arr)) if arr.size > 0 else 0.0,
        "mean_clearance": float(np.mean(arr)) if arr.size > 0 else 0.0,
    }


class IntentHybridEvaluator(Node):
    def __init__(self, args_ns: argparse.Namespace) -> None:
        super().__init__("intent_hybrid_evaluator")
        self.args = args_ns
        self.backend_status: Dict[str, Any] = {
            "collision_backend_available": False,
            "collision_backend_checked": False,
            "collision_backend_error": "preflight not run",
            "dryrun_mode": False,
            "result_interpretable": False,
            "failure_reason": "collision_backend_not_checked",
            "service_name": str(self.args.motion_service),
            "service_type": "",
            "preflight_ok": False,
            "preflight_collision_queries": 0,
        }
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 50
        )
        self.motion_client = None
        if CheckMotionBatch is not None:
            self.motion_client = self.create_client(
                CheckMotionBatch,
                str(self.args.motion_service),
            )
        self.fk_client = None
        if bool(self.args.enable_fk) and GetPositionFK is not None:
            self.fk_client = self.create_client(GetPositionFK, str(self.args.fk_service))
        self.get_logger().info(
            "Evaluator started: "
            f"motion_service={self.args.motion_service}, motion_client_ready={bool(self.motion_client is not None)}, "
            f"fk_enabled={bool(self.fk_client is not None)}"
        )

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_joint_state = msg

    def _wait_motion_service(self, timeout: float) -> bool:
        if self.motion_client is None:
            return False
        return bool(self.motion_client.wait_for_service(timeout_sec=max(timeout, 0.1)))

    def _service_type_status(self) -> Tuple[bool, str, str]:
        service_name = str(self.args.motion_service)
        for name, types in self.get_service_names_and_types():
            if name != service_name:
                continue
            type_text = ",".join(types)
            return EXPECTED_CHECK_MOTION_TYPE in types, type_text, ""
        return False, "", f"service not found: {service_name}"

    def collision_backend_preflight(
        self,
        sample_traj: np.ndarray,
        joint_names: Sequence[str],
        group_name: str,
        edge_resolution: float,
    ) -> Dict[str, Any]:
        status = {
            "collision_backend_available": False,
            "collision_backend_checked": True,
            "collision_backend_error": "",
            "dryrun_mode": False,
            "result_interpretable": False,
            "failure_reason": "",
            "service_name": str(self.args.motion_service),
            "service_type": "",
            "preflight_ok": False,
            "preflight_collision_queries": 0,
        }
        if CheckMotionBatch is None:
            status["collision_backend_error"] = "intent_hybrid_interfaces/CheckMotionBatch import failed"
            status["failure_reason"] = "collision_backend_unavailable"
            self.backend_status = status
            self._log_backend_preflight(status)
            return status
        deadline = time.monotonic() + max(float(self.args.collision_service_timeout_sec), 0.1)
        type_ok = False
        type_text = ""
        type_error = ""
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            type_ok, type_text, type_error = self._service_type_status()
            if type_ok or type_text:
                break
        status["service_type"] = type_text
        if not type_ok:
            status["collision_backend_error"] = type_error or f"service type mismatch: {type_text}"
            status["failure_reason"] = "collision_backend_unavailable"
            self.backend_status = status
            self._log_backend_preflight(status)
            return status
        if self.motion_client is None:
            status["collision_backend_error"] = "CheckMotionBatch client is not initialized"
            status["failure_reason"] = "collision_backend_unavailable"
            self.backend_status = status
            self._log_backend_preflight(status)
            return status
        if not self.motion_client.wait_for_service(timeout_sec=max(float(self.args.collision_service_timeout_sec), 0.1)):
            status["collision_backend_error"] = f"service not ready: {self.args.motion_service}"
            status["failure_reason"] = "collision_backend_unavailable"
            self.backend_status = status
            self._log_backend_preflight(status)
            return status

        traj = np.asarray(sample_traj, dtype=float)
        if traj.ndim != 2 or traj.shape[0] != len(joint_names):
            status["collision_backend_error"] = f"invalid preflight trajectory shape {traj.shape}"
            status["failure_reason"] = "collision_backend_unavailable"
            self.backend_status = status
            self._log_backend_preflight(status)
            return status
        if traj.shape[1] < 2:
            traj = np.repeat(traj[:, :1], 2, axis=1)
        motion = self._check_motion(
            traj[:, :2],
            joint_names,
            group_name,
            edge_resolution,
            check_edges=True,
            service_timeout_sec=max(float(self.args.collision_service_timeout_sec), 0.1),
        )
        state_len_ok = int(np.asarray(motion.get("state_valid", []), dtype=bool).size) == 2
        edge_len_ok = int(np.asarray(motion.get("edge_valid", []), dtype=bool).size) == 1
        response_ok = bool(motion.get("ok", False))
        queries = int(motion.get("collision_queries", 0))
        status["preflight_collision_queries"] = queries
        if response_ok and (queries > 0 or (state_len_ok and edge_len_ok)):
            status["collision_backend_available"] = True
            status["preflight_ok"] = True
            status["result_interpretable"] = True
        else:
            status["collision_backend_error"] = (
                str(motion.get("error_message", ""))
                or f"preflight response invalid: ok={response_ok}, state_len_ok={state_len_ok}, edge_len_ok={edge_len_ok}, queries={queries}"
            )
            status["failure_reason"] = "collision_backend_unavailable"
        self.backend_status = status
        self._log_backend_preflight(status)
        return status

    def _log_backend_preflight(self, status: Dict[str, Any]) -> None:
        available = "available" if bool(status.get("collision_backend_available", False)) else "not available"
        self.get_logger().info(f"[Evaluator] check_motion_batch service: {available}")
        self.get_logger().info(f"[Evaluator] service type: {status.get('service_type', '')}")
        self.get_logger().info(f"[Evaluator] preflight ok: {bool(status.get('preflight_ok', False))}")
        self.get_logger().info(f"[Evaluator] collision backend error: {status.get('collision_backend_error', '')}")

    def _check_motion(
        self,
        traj_dofxn: np.ndarray,
        joint_names: Sequence[str],
        group_name: str,
        edge_resolution: float,
        *,
        check_edges: bool = True,
        service_timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        traj = np.asarray(traj_dofxn, dtype=float)
        n = int(traj.shape[1])
        if CheckMotionBatch is None or self.motion_client is None:
            return {
                "ok": False,
                "error_message": "intent_hybrid_interfaces/CheckMotionBatch is unavailable",
                "state_valid": np.zeros((n,), dtype=bool),
                "edge_valid": np.zeros((max(n - 1, 0),), dtype=bool),
                "first_invalid_state": -1,
                "first_invalid_edge": -1,
                "elapsed_ms": 0.0,
                "collision_queries": 0,
            }
        req = CheckMotionBatch.Request()
        req.group_name = str(group_name)
        req.joint_names = list(joint_names)
        req.dof = int(len(joint_names))
        req.states_flat = traj.T.reshape(-1).tolist()
        req.check_edges = bool(check_edges)
        req.edge_resolution = float(edge_resolution)
        if not self._wait_motion_service(self.args.service_wait_sec):
            return {
                "ok": False,
                "error_message": f"service not ready: {self.args.motion_service}",
                "state_valid": np.zeros((n,), dtype=bool),
                "edge_valid": np.zeros((max(n - 1, 0),), dtype=bool),
                "first_invalid_state": -1,
                "first_invalid_edge": -1,
                "elapsed_ms": 0.0,
                "collision_queries": 0,
            }
        fut = self.motion_client.call_async(req)
        timeout_base = self.args.service_timeout_sec if service_timeout_sec is None else service_timeout_sec
        timeout_sec = max(float(timeout_base), 0.5) + 0.01 * float(n)
        if not self._spin_until_future(fut, timeout_sec):
            return {
                "ok": False,
                "error_message": "CheckMotionBatch timeout/no-response",
                "state_valid": np.zeros((n,), dtype=bool),
                "edge_valid": np.zeros((max(n - 1, 0),), dtype=bool),
                "first_invalid_state": -1,
                "first_invalid_edge": -1,
                "elapsed_ms": 0.0,
                "collision_queries": 0,
            }
        resp = fut.result()
        if resp is None:
            return {
                "ok": False,
                "error_message": "CheckMotionBatch result is None",
                "state_valid": np.zeros((n,), dtype=bool),
                "edge_valid": np.zeros((max(n - 1, 0),), dtype=bool),
                "first_invalid_state": -1,
                "first_invalid_edge": -1,
                "elapsed_ms": 0.0,
                "collision_queries": 0,
            }
        state_valid = np.asarray(list(resp.state_valid), dtype=bool).reshape(-1)
        edge_valid = np.asarray(list(resp.edge_valid), dtype=bool).reshape(-1)
        expected_edges = max(n - 1, 0)
        if state_valid.size != n:
            state_valid = np.zeros((n,), dtype=bool)
        if check_edges and edge_valid.size != expected_edges:
            edge_valid = np.zeros((expected_edges,), dtype=bool)
        return {
            "ok": bool(resp.ok),
            "error_message": str(resp.error_message),
            "state_valid": state_valid,
            "edge_valid": edge_valid,
            "first_invalid_state": int(resp.first_invalid_state),
            "first_invalid_edge": int(resp.first_invalid_edge),
            "elapsed_ms": float(resp.elapsed_ms),
            "collision_queries": int(resp.collision_queries),
        }

    @staticmethod
    def _motion_invalid_counts(motion: Dict[str, Any]) -> Tuple[int, int]:
        s = np.asarray(motion.get("state_valid", []), dtype=bool).reshape(-1)
        e = np.asarray(motion.get("edge_valid", []), dtype=bool).reshape(-1)
        return int(np.count_nonzero(~s)), int(np.count_nonzero(~e))

    @staticmethod
    def _strict_motion_pass(motion: Dict[str, Any], expected_states: int) -> bool:
        s = np.asarray(motion.get("state_valid", []), dtype=bool).reshape(-1)
        e = np.asarray(motion.get("edge_valid", []), dtype=bool).reshape(-1)
        expected_edges = max(expected_states - 1, 0)
        if expected_states < 2:
            return False
        if not bool(motion.get("ok", False)):
            return False
        if s.size != expected_states or e.size != expected_edges:
            return False
        if not bool(np.all(s)):
            return False
        if not bool(np.all(e)):
            return False
        if int(motion.get("first_invalid_state", -2)) != -1:
            return False
        if int(motion.get("first_invalid_edge", -2)) != -1:
            return False
        return True

    def _spin_until_future(self, fut, timeout_sec: float) -> bool:
        deadline = time.monotonic() + max(timeout_sec, 0.01)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if fut.done():
                return True
        return fut.done()

    def _current_joint_state_map(self) -> Dict[str, float]:
        msg = self._latest_joint_state
        if msg is None:
            for _ in range(int(max(self.args.current_state_wait_sec, 0.1) / 0.05)):
                rclpy.spin_once(self, timeout_sec=0.05)
                msg = self._latest_joint_state
                if msg is not None:
                    break
        if msg is None or not msg.name or not msg.position:
            return {}
        return {str(n): float(v) for n, v in zip(msg.name, msg.position)}

    def _compute_fk_xyz(
        self,
        traj_dofxn: np.ndarray,
        joint_names: Sequence[str],
        ee_link: str,
        base_frame: str,
    ) -> np.ndarray:
        if self.fk_client is None or GetPositionFK is None or RobotState is None:
            return np.zeros((0, 3), dtype=float)
        if not self.fk_client.wait_for_service(timeout_sec=max(self.args.service_wait_sec, 0.1)):
            self.get_logger().warn("FK service is not ready, skip EE plot from FK.")
            return np.zeros((0, 3), dtype=float)
        traj = np.asarray(traj_dofxn, dtype=float)
        n = int(traj.shape[1])
        out = np.zeros((n, 3), dtype=float)
        valid_mask = np.zeros((n,), dtype=bool)
        for i in range(n):
            req = GetPositionFK.Request()
            req.header.frame_id = str(base_frame)
            req.fk_link_names = [str(ee_link)]
            rs = RobotState()
            rs.joint_state.name = list(joint_names)
            rs.joint_state.position = traj[:, i].tolist()
            req.robot_state = rs
            fut = self.fk_client.call_async(req)
            if not self._spin_until_future(fut, self.args.fk_timeout_sec):
                continue
            resp = fut.result()
            if resp is None:
                continue
            if not resp.pose_stamped:
                continue
            p = resp.pose_stamped[0].pose.position
            out[i, :] = [float(p.x), float(p.y), float(p.z)]
            valid_mask[i] = True
        return out[valid_mask, :] if np.any(valid_mask) else np.zeros((0, 3), dtype=float)

    def _make_ee_plot(
        self,
        out_png: Path,
        ee_nominal: np.ndarray,
        ee_modulated: np.ndarray,
        obstacles: List[Dict[str, float]],
    ) -> None:
        fig, ax = plt.subplots(figsize=(7, 6), dpi=140)
        if ee_nominal.size > 0:
            ax.plot(ee_nominal[:, 0], ee_nominal[:, 1], "--", color="#2b5de5", lw=2.0, label="nominal_ee")
        if ee_modulated.size > 0:
            ax.plot(ee_modulated[:, 0], ee_modulated[:, 1], "-", color="#1a9a2a", lw=2.2, label="modulated_ee")
        for ob in obstacles:
            circ = plt.Circle((ob["x"], ob["y"]), max(ob["radius"], 1e-6), color="#ff7a7a", alpha=0.35)
            ax.add_patch(circ)
            ax.plot(ob["x"], ob["y"], "rx", ms=5)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, ls="--", alpha=0.4)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("EE Plane: nominal vs modulated")
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(str(out_png))
        plt.close(fig)

    def _make_joint_plot(
        self,
        out_png: Path,
        nominal: np.ndarray,
        modulated: np.ndarray,
        dt: float,
    ) -> None:
        dof = nominal.shape[0]
        cols = 2
        rows = int(math.ceil(dof / cols))
        fig, axes = plt.subplots(rows, cols, figsize=(10, 2.5 * rows), dpi=140, sharex=True)
        axes_arr = np.atleast_1d(axes).reshape(rows, cols)
        t_nom = np.arange(nominal.shape[1], dtype=float) * dt
        t_mod = np.arange(modulated.shape[1], dtype=float) * dt
        for j in range(rows * cols):
            ax = axes_arr[j // cols, j % cols]
            if j < dof:
                ax.plot(t_nom, nominal[j, :], "--", lw=1.8, color="#2b5de5", label="nominal")
                ax.plot(t_mod, modulated[j, :], "-", lw=2.0, color="#1a9a2a", label="modulated")
                ax.set_title(f"q{j + 1}")
                ax.grid(True, ls="--", alpha=0.35)
            else:
                ax.axis("off")
        handles, labels = axes_arr[0, 0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labels, loc="upper right")
        fig.tight_layout()
        fig.savefig(str(out_png))
        plt.close(fig)

    def _make_collision_timeline(
        self,
        out_png: Path,
        nom_motion: Dict[str, Any],
        mod_motion: Dict[str, Any],
    ) -> None:
        nom_s = (~np.asarray(nom_motion.get("state_valid", []), dtype=bool)).astype(int)
        nom_e = (~np.asarray(nom_motion.get("edge_valid", []), dtype=bool)).astype(int)
        mod_s = (~np.asarray(mod_motion.get("state_valid", []), dtype=bool)).astype(int)
        mod_e = (~np.asarray(mod_motion.get("edge_valid", []), dtype=bool)).astype(int)
        fig, axes = plt.subplots(2, 1, figsize=(9, 5), dpi=140, sharex=False)
        axes[0].plot(nom_s, label="nominal_state_invalid", color="#d62728")
        axes[0].plot(mod_s, label="modulated_state_invalid", color="#2ca02c")
        axes[0].set_ylim(-0.1, 1.1)
        axes[0].grid(True, ls="--", alpha=0.35)
        axes[0].legend(loc="upper right")
        axes[0].set_title("Collision Timeline (State)")
        axes[1].plot(nom_e, label="nominal_edge_invalid", color="#ff7f0e")
        axes[1].plot(mod_e, label="modulated_edge_invalid", color="#1f77b4")
        axes[1].set_ylim(-0.1, 1.1)
        axes[1].grid(True, ls="--", alpha=0.35)
        axes[1].legend(loc="upper right")
        axes[1].set_title("Collision Timeline (Edge)")
        axes[1].set_xlabel("Index")
        fig.tight_layout()
        fig.savefig(str(out_png))
        plt.close(fig)

    def evaluate_scenario(self, scenario: Dict[str, Any], global_cfg: Dict[str, Any], out_root: Path) -> Dict[str, Any]:
        name = str(scenario.get("name", f"scenario_{int(time.time())}"))
        dof = int(global_cfg.get("dof", len(global_cfg.get("joint_names", DEFAULT_JOINT_NAMES))))
        joint_names = list(scenario.get("joint_names", global_cfg.get("joint_names", DEFAULT_JOINT_NAMES)))
        dof = int(len(joint_names)) if dof <= 0 else dof
        if len(joint_names) != dof:
            raise ValueError(f"joint_names count mismatch in scenario {name}: dof={dof}, names={len(joint_names)}")

        nominal = normalize_traj_dofxn(scenario["nominal_trajectory"], dof, f"{name}.nominal_trajectory")
        modulated = normalize_traj_dofxn(scenario["modulated_trajectory"], dof, f"{name}.modulated_trajectory")
        dt = float(scenario.get("nominal_dt", global_cfg.get("nominal_dt", 0.1)))
        group_name = str(scenario.get("group_name", global_cfg.get("group_name", "ur_manipulator")))
        edge_resolution = float(scenario.get("edge_resolution", global_cfg.get("edge_resolution", 0.02)))
        ee_link = str(scenario.get("ee_link", global_cfg.get("ee_link", "tool0")))
        base_frame = str(scenario.get("base_frame", global_cfg.get("base_frame", "base_link")))
        scene = scenario.get("scene", global_cfg.get("scene", {}))
        obstacles = _parse_obstacles(scene if isinstance(scene, dict) else {})

        nominal_motion = self._check_motion(nominal, joint_names, group_name, edge_resolution, check_edges=True)
        modulated_motion = self._check_motion(modulated, joint_names, group_name, edge_resolution, check_edges=True)
        nom_bad_s, nom_bad_e = self._motion_invalid_counts(nominal_motion)
        mod_bad_s, mod_bad_e = self._motion_invalid_counts(modulated_motion)
        mod_pass = self._strict_motion_pass(modulated_motion, expected_states=int(modulated.shape[1]))
        nominal_total = nom_bad_s + nom_bad_e
        backend_available = bool(self.backend_status.get("collision_backend_available", False))
        dryrun_mode = bool(self.backend_status.get("dryrun_mode", False))
        result_interpretable = bool(self.backend_status.get("result_interpretable", False))
        failure_reason = str(self.backend_status.get("failure_reason", ""))
        if not backend_available:
            result_interpretable = False
            failure_reason = "collision_backend_unavailable"
        avoidance_success = bool(
            result_interpretable
            and mod_pass
            and (mod_bad_s == 0)
            and (mod_bad_e == 0)
            and (nominal_total >= 0)
        )

        smooth = compute_joint_metrics(modulated, dt)
        current_map = self._current_joint_state_map()
        start_err_max = 0.0
        start_err_norm = 0.0
        if current_map:
            q0 = nominal[:, 0]
            diffs: List[float] = []
            for j, name_j in enumerate(joint_names):
                if name_j in current_map:
                    diffs.append(float(q0[j] - current_map[name_j]))
            if diffs:
                arr = np.asarray(diffs, dtype=float)
                start_err_max = float(np.max(np.abs(arr)))
                start_err_norm = float(np.linalg.norm(arr))

        bench = {}
        bench_path = scenario.get("benchmark_json", "")
        if bench_path:
            p = Path(str(bench_path)).expanduser()
            if p.exists():
                bench = json.loads(p.read_text(encoding="utf-8"))
        dispatch = _pick_dispatch_meta(scenario, bench)
        rrt_meta = _pick_rrt_meta(scenario, bench)

        local_reviews: List[Dict[str, Any]] = []
        for idx, lp in enumerate(scenario.get("local_paths", [])):
            if isinstance(lp, dict):
                lp_data = lp.get("path", [])
                lp_stop = str(lp.get("stop_reason", ""))
                lp_elapsed = float(lp.get("elapsed_ms", 0.0))
                lp_queries = int(lp.get("collision_queries", 0))
            else:
                lp_data = lp
                lp_stop = ""
                lp_elapsed = 0.0
                lp_queries = 0
            lp_arr = normalize_traj_dofxn(lp_data, dof, f"{name}.local_paths[{idx}]")
            lp_motion = self._check_motion(lp_arr, joint_names, group_name, edge_resolution, check_edges=True)
            lp_pass = self._strict_motion_pass(lp_motion, expected_states=int(lp_arr.shape[1]))
            lp_bad_s, lp_bad_e = self._motion_invalid_counts(lp_motion)
            local_reviews.append(
                {
                    "index": idx,
                    "postcheck_passed": bool(lp_pass),
                    "invalid_state_count": int(lp_bad_s),
                    "invalid_edge_count": int(lp_bad_e),
                    "first_invalid_state": int(lp_motion.get("first_invalid_state", -1)),
                    "first_invalid_edge": int(lp_motion.get("first_invalid_edge", -1)),
                    "stop_reason": lp_stop,
                    "elapsed_ms": lp_elapsed,
                    "collision_queries": lp_queries,
                }
            )

        ee_nom = np.zeros((0, 3), dtype=float)
        ee_mod = np.zeros((0, 3), dtype=float)
        if "ee_nominal_xyz" in scenario and "ee_modulated_xyz" in scenario:
            ee_nom = normalize_xyz_nx3(scenario["ee_nominal_xyz"], f"{name}.ee_nominal_xyz")
            ee_mod = normalize_xyz_nx3(scenario["ee_modulated_xyz"], f"{name}.ee_modulated_xyz")
        elif self.fk_client is not None:
            ee_nom = self._compute_fk_xyz(nominal, joint_names, ee_link, base_frame)
            ee_mod = self._compute_fk_xyz(modulated, joint_names, ee_link, base_frame)

        clr = _clearance_stats(ee_mod, obstacles)
        length_nom = compute_joint_metrics(nominal, dt)["joint_path_length"]
        length_ratio = (smooth["joint_path_length"] / length_nom) if length_nom > 1e-12 else 0.0

        scenario_out = out_root / name
        scenario_out.mkdir(parents=True, exist_ok=True)
        if bool(self.args.enable_plot):
            self._make_joint_plot(scenario_out / "joint_traj_compare.png", nominal, modulated, dt)
            self._make_collision_timeline(
                scenario_out / "collision_timeline.png", nominal_motion, modulated_motion
            )
            if ee_nom.size > 0 and ee_mod.size > 0:
                self._make_ee_plot(
                    scenario_out / "ee_plane_compare.png",
                    ee_nom,
                    ee_mod,
                    obstacles,
                )

        result = {
            "scenario_name": name,
            "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "collision_backend_available": backend_available,
            "collision_backend_checked": bool(self.backend_status.get("collision_backend_checked", False)),
            "collision_backend_error": str(self.backend_status.get("collision_backend_error", "")),
            "dryrun_mode": dryrun_mode,
            "result_interpretable": result_interpretable,
            "failure_reason": failure_reason,
            "inputs": {
                "group_name": group_name,
                "joint_names": joint_names,
                "nominal_points": int(nominal.shape[1]),
                "modulated_points": int(modulated.shape[1]),
                "edge_resolution": edge_resolution,
                "scene_obstacle_count": int(len(obstacles)),
            },
            "avoidance": {
                "nominal_invalid_state_count": int(nom_bad_s),
                "nominal_invalid_edge_count": int(nom_bad_e),
                "modulated_invalid_state_count": int(mod_bad_s),
                "modulated_invalid_edge_count": int(mod_bad_e),
                "avoidance_success": bool(avoidance_success),
                "modulated_postcheck_passed": bool(mod_pass),
            },
            "smoothness": {
                **smooth,
                "trajectory_length_ratio": float(length_ratio),
            },
            "executability": {
                **dispatch,
                "start_state_error_max": float(start_err_max),
                "start_state_error_norm": float(start_err_norm),
            },
            "local_path_review": {
                **rrt_meta,
                "local_paths_checked": int(len(local_reviews)),
                "local_paths_passed": int(sum(1 for x in local_reviews if bool(x["postcheck_passed"]))),
                "items": local_reviews,
            },
            "clearance": clr,
            "motion_raw": {
                "nominal": {
                    "ok": bool(nominal_motion.get("ok", False)),
                    "first_invalid_state": int(nominal_motion.get("first_invalid_state", -1)),
                    "first_invalid_edge": int(nominal_motion.get("first_invalid_edge", -1)),
                    "elapsed_ms": float(nominal_motion.get("elapsed_ms", 0.0)),
                    "collision_queries": int(nominal_motion.get("collision_queries", 0)),
                },
                "modulated": {
                    "ok": bool(modulated_motion.get("ok", False)),
                    "first_invalid_state": int(modulated_motion.get("first_invalid_state", -1)),
                    "first_invalid_edge": int(modulated_motion.get("first_invalid_edge", -1)),
                    "elapsed_ms": float(modulated_motion.get("elapsed_ms", 0.0)),
                    "collision_queries": int(modulated_motion.get("collision_queries", 0)),
                },
            },
        }
        (scenario_out / "evaluation_result.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.get_logger().info(
            f"[{name}] avoidance_success={result['avoidance']['avoidance_success']}, "
            f"interpretable={result['result_interpretable']}, "
            f"modulated_invalid=({mod_bad_s},{mod_bad_e}), "
            f"dispatch_status={result['executability']['dispatch_action_status']}, "
            f"aborted={result['executability']['execution_aborted']}"
        )
        return result


def _write_summary(out_dir: Path, results: List[Dict[str, Any]]) -> None:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    summary = {
        "timestamp": ts,
        "scenario_count": len(results),
        "success_count": int(
            sum(1 for r in results if bool(r.get("result_interpretable", False)) and bool(r["avoidance"]["avoidance_success"]))
        ),
        "interpretable_count": int(sum(1 for r in results if bool(r.get("result_interpretable", False)))),
        "backend_available_count": int(sum(1 for r in results if bool(r.get("collision_backend_available", False)))),
        "backend_unavailable_count": int(sum(1 for r in results if not bool(r.get("collision_backend_available", False)))),
        "results": results,
    }
    (out_dir / f"evaluation_summary_{ts}.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    csv_path = out_dir / f"evaluation_summary_{ts}.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        fieldnames = [
            "scenario_name",
            "collision_backend_available",
            "collision_backend_checked",
            "collision_backend_error",
            "dryrun_mode",
            "result_interpretable",
            "failure_reason",
            "avoidance_success",
            "nominal_invalid_state_count",
            "nominal_invalid_edge_count",
            "modulated_invalid_state_count",
            "modulated_invalid_edge_count",
            "joint_path_length",
            "max_joint_delta",
            "joint_jerk_integral",
            "estimated_max_velocity",
            "estimated_max_acceleration",
            "trajectory_length_ratio",
            "dispatch_action_status",
            "dispatch_error_code",
            "execution_aborted",
            "rrt_stop_reason",
            "rrt_elapsed_ms",
            "rrt_collision_queries",
            "local_paths_checked",
            "local_paths_passed",
            "min_clearance",
            "mean_clearance",
        ]
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in results:
            w.writerow(
                {
                    "scenario_name": r["scenario_name"],
                    "collision_backend_available": int(r.get("collision_backend_available", False)),
                    "collision_backend_checked": int(r.get("collision_backend_checked", False)),
                    "collision_backend_error": r.get("collision_backend_error", ""),
                    "dryrun_mode": int(r.get("dryrun_mode", False)),
                    "result_interpretable": int(r.get("result_interpretable", False)),
                    "failure_reason": r.get("failure_reason", ""),
                    "avoidance_success": int(r["avoidance"]["avoidance_success"]),
                    "nominal_invalid_state_count": r["avoidance"]["nominal_invalid_state_count"],
                    "nominal_invalid_edge_count": r["avoidance"]["nominal_invalid_edge_count"],
                    "modulated_invalid_state_count": r["avoidance"]["modulated_invalid_state_count"],
                    "modulated_invalid_edge_count": r["avoidance"]["modulated_invalid_edge_count"],
                    "joint_path_length": r["smoothness"]["joint_path_length"],
                    "max_joint_delta": r["smoothness"]["max_joint_delta"],
                    "joint_jerk_integral": r["smoothness"]["joint_jerk_integral"],
                    "estimated_max_velocity": r["smoothness"]["estimated_max_velocity"],
                    "estimated_max_acceleration": r["smoothness"]["estimated_max_acceleration"],
                    "trajectory_length_ratio": r["smoothness"]["trajectory_length_ratio"],
                    "dispatch_action_status": r["executability"]["dispatch_action_status"],
                    "dispatch_error_code": r["executability"]["dispatch_error_code"],
                    "execution_aborted": int(r["executability"]["execution_aborted"]),
                    "rrt_stop_reason": r["local_path_review"]["rrt_stop_reason"],
                    "rrt_elapsed_ms": r["local_path_review"]["rrt_elapsed_ms"],
                    "rrt_collision_queries": r["local_path_review"]["rrt_collision_queries"],
                    "local_paths_checked": r["local_path_review"]["local_paths_checked"],
                    "local_paths_passed": r["local_path_review"]["local_paths_passed"],
                    "min_clearance": r["clearance"]["min_clearance"],
                    "mean_clearance": r["clearance"]["mean_clearance"],
                }
            )


def _load_eval_config(path: Path) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    global_cfg = data.get("global", {}) if isinstance(data, dict) else {}
    scenarios = data.get("scenarios", []) if isinstance(data, dict) else []
    if not isinstance(scenarios, list) or len(scenarios) == 0:
        raise ValueError("config.scenarios is empty")
    return global_cfg, scenarios


def _load_planner_output(path: Path) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    joint_names = list(data.get("joint_names", DEFAULT_JOINT_NAMES))
    time_axis = data.get("time_axis", [])
    nominal_dt = 0.1
    if isinstance(time_axis, list) and len(time_axis) >= 2:
        try:
            nominal_dt = float(time_axis[1]) - float(time_axis[0])
        except Exception:
            nominal_dt = 0.1
    scenario = {
        "name": str(data.get("scenario_name", path.stem)),
        "joint_names": joint_names,
        "nominal_dt": float(data.get("nominal_dt", nominal_dt)),
        "nominal_trajectory": data.get("nominal_trajectory", data.get("nominal_traj", [])),
        "modulated_trajectory": data.get("modulated_trajectory", data.get("modulated_traj", [])),
        "local_paths": data.get("local_paths", []),
        "rrt_stop_reason": data.get("rrt_stop_reason", ""),
        "rrt_elapsed_ms": data.get("rrt_elapsed_ms", 0.0),
        "rrt_collision_queries": data.get("rrt_collision_queries", 0),
        "dispatch_action_status": data.get("dispatch_action_status", -1),
        "dispatch_error_code": data.get("dispatch_error_code", 0),
        "execution_aborted": data.get("execution_aborted", False),
    }
    if data.get("via_points"):
        scenario["local_paths"] = scenario["local_paths"] or [
            {
                "path": data.get("via_points", []),
                "stop_reason": data.get("rrt_stop_reason", "via_points"),
                "elapsed_ms": data.get("rrt_elapsed_ms", 0.0),
                "collision_queries": data.get("rrt_collision_queries", 0),
            }
        ]
    if data.get("scene"):
        scenario["scene"] = data["scene"]
    global_cfg = {
        "joint_names": joint_names,
        "nominal_dt": float(scenario["nominal_dt"]),
        "group_name": str(data.get("group_name", "ur_manipulator")),
        "edge_resolution": float(data.get("edge_resolution", 0.02)),
        "ee_link": str(data.get("ee_link", "tool0")),
        "base_frame": str(data.get("base_frame", "base_link")),
    }
    return global_cfg, [scenario]


def _sample_for_preflight(global_cfg: Dict[str, Any], scenarios: List[Dict[str, Any]]) -> Tuple[np.ndarray, List[str], str, float]:
    joint_names = list(global_cfg.get("joint_names", DEFAULT_JOINT_NAMES))
    dof = int(len(joint_names))
    group_name = str(global_cfg.get("group_name", "ur_manipulator"))
    edge_resolution = float(global_cfg.get("edge_resolution", 0.02))
    if scenarios:
        sc = scenarios[0]
        joint_names = list(sc.get("joint_names", joint_names))
        dof = int(len(joint_names))
        group_name = str(sc.get("group_name", group_name))
        edge_resolution = float(sc.get("edge_resolution", edge_resolution))
        if sc.get("nominal_trajectory") is not None:
            traj = normalize_traj_dofxn(sc["nominal_trajectory"], dof, "preflight.nominal_trajectory")
            if traj.shape[1] >= 2:
                return traj[:, :2], joint_names, group_name, edge_resolution
            if traj.shape[1] == 1:
                return np.repeat(traj, 2, axis=1), joint_names, group_name, edge_resolution
    return np.zeros((dof, 2), dtype=float), joint_names, group_name, edge_resolution


def main(args: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="", help="Evaluation config JSON with global/scenarios.")
    parser.add_argument("--planner-output", default="", help="Minimal planner output JSON: nominal_traj/modulated_traj/via_points.")
    parser.add_argument("--out-dir", default=str(Path.cwd() / "evaluation"))
    parser.add_argument("--motion-service", default="/intent_runtime/check_motion_batch")
    parser.add_argument("--fk-service", default="/compute_fk")
    parser.add_argument("--enable-fk", action="store_true")
    parser.add_argument("--enable-plot", action="store_true")
    parser.add_argument("--preflight-only", action="store_true")
    parser.add_argument("--require-collision-backend", type=_bool_arg, default=True)
    parser.add_argument("--collision-service-timeout-sec", type=float, default=3.0)
    parser.add_argument("--dryrun-allow-missing-backend", type=_bool_arg, default=False)
    parser.add_argument("--service-timeout-sec", type=float, default=2.0)
    parser.add_argument("--service-wait-sec", type=float, default=2.0)
    parser.add_argument("--fk-timeout-sec", type=float, default=1.0)
    parser.add_argument("--current-state-wait-sec", type=float, default=0.5)
    ns, ros_args = parser.parse_known_args(args=args)

    out_dir = Path(ns.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    if ns.planner_output:
        global_cfg, scenarios = _load_planner_output(Path(ns.planner_output).expanduser())
    elif ns.config:
        global_cfg, scenarios = _load_eval_config(Path(ns.config).expanduser())
    elif ns.preflight_only:
        global_cfg, scenarios = {"joint_names": DEFAULT_JOINT_NAMES, "group_name": "ur_manipulator", "edge_resolution": 0.02}, []
    else:
        raise SystemExit("--config or --planner-output is required unless --preflight-only is used")

    rclpy.init(args=ros_args)
    node = IntentHybridEvaluator(ns)
    try:
        sample, sample_joints, sample_group, sample_edge_resolution = _sample_for_preflight(global_cfg, scenarios)
        backend = node.collision_backend_preflight(
            sample,
            sample_joints,
            sample_group,
            sample_edge_resolution,
        )
        if not bool(backend.get("collision_backend_available", False)):
            if bool(ns.dryrun_allow_missing_backend):
                backend["dryrun_mode"] = True
                backend["result_interpretable"] = False
                backend["failure_reason"] = "collision_backend_unavailable"
                node.backend_status = backend
                node.get_logger().warn(
                    "Collision backend is unavailable; continuing only because dryrun_allow_missing_backend=true. "
                    "Results will be marked result_interpretable=false."
                )
            elif bool(ns.require_collision_backend):
                node.get_logger().error(
                    "Collision backend preflight failed; stop evaluation. "
                    f"reason={backend.get('collision_backend_error', '')}"
                )
                raise SystemExit(2)
        if ns.preflight_only:
            if bool(backend.get("collision_backend_available", False)):
                node.get_logger().info("Preflight-only finished successfully.")
                return
            raise SystemExit(2)
        results: List[Dict[str, Any]] = []
        for sc in scenarios:
            results.append(node.evaluate_scenario(sc, global_cfg, out_dir))
        _write_summary(out_dir, results)
        node.get_logger().info(f"Evaluation finished. Outputs written to: {out_dir}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
