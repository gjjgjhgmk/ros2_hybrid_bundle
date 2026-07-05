"""Metrics and JSON/CSV export for normalized two-dimensional paths."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Sequence

import numpy as np

from .planner_2d import path_collision_details, path_min_clearance


def path_length(path: Iterable[Sequence[float]]) -> float:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[0] < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))


def jerk_integral(
    path: Iterable[Sequence[float]], time_axis: Optional[Sequence[float]] = None
) -> float:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[0] < 4:
        return 0.0
    times = (
        np.linspace(0.0, 1.0, arr.shape[0])
        if time_axis is None
        else np.asarray(time_axis, dtype=float).reshape(-1)
    )
    if times.size != arr.shape[0] or np.any(np.diff(times) <= 0.0):
        raise ValueError("time_axis must be strictly increasing and match path length")
    velocity = np.gradient(arr, times, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, times, axis=0, edge_order=2)
    jerk = np.gradient(acceleration, times, axis=0, edge_order=2)
    return float(np.trapz(np.linalg.norm(jerk, axis=1), times))


def _json_clearance(value: float) -> Optional[float]:
    return float(value) if math.isfinite(value) else None


def build_evaluation(
    *,
    scenario_name: str,
    group_name: str,
    nominal_path: Iterable[Sequence[float]],
    modulated_path: Iterable[Sequence[float]],
    obstacles: Iterable[Dict[str, Any]],
    safety_margin: float,
    rrt_result: Dict[str, Any],
    cart_waypoint_count: int,
    ur_move_result: Dict[str, Any],
    failure_reason: str = "",
) -> Dict[str, Any]:
    nominal = np.asarray(nominal_path, dtype=float)
    modulated = np.asarray(modulated_path, dtype=float)
    nominal_collision = path_collision_details(nominal, obstacles, margin=0.0)
    modulated_collision = path_collision_details(modulated, obstacles, margin=0.0)
    nominal_clearance = path_min_clearance(nominal, obstacles)
    modulated_clearance = path_min_clearance(modulated, obstacles)
    nominal_length = path_length(nominal)
    modulated_length = path_length(modulated)
    nominal_jerk = jerk_integral(nominal)
    modulated_jerk = jerk_integral(modulated)

    clearance_ok = not math.isfinite(modulated_clearance) or modulated_clearance >= float(safety_margin)
    avoidance_success = modulated_collision["collision_count"] == 0 and clearance_ok
    moveit_success = bool(ur_move_result.get("planning_success", False))
    execution_requested = bool(ur_move_result.get("execution_requested", False))
    execution_success = ur_move_result.get("execution_success")
    execution_id = str(ur_move_result.get("execution_id", ""))

    if not failure_reason:
        if modulated_collision["collision_count"] > 0:
            failure_reason = "modulated_path_in_collision"
        elif not clearance_ok:
            failure_reason = "modulated_clearance_below_margin"
        elif not ur_move_result.get("available", False):
            failure_reason = "ur_move_unavailable"
        elif not moveit_success:
            failure_reason = "moveit_planning_failed"
        elif execution_requested and execution_success is False:
            failure_reason = "moveit_execution_failed"

    return {
        "scenario_name": str(scenario_name),
        "group_name": str(group_name),
        "nominal_collision_count": int(nominal_collision["collision_count"]),
        "nominal_invalid_point_count": int(nominal_collision["invalid_point_count"]),
        "nominal_invalid_edge_count": int(nominal_collision["invalid_edge_count"]),
        "nominal_min_clearance": _json_clearance(nominal_clearance),
        "modulated_collision_count": int(modulated_collision["collision_count"]),
        "modulated_invalid_point_count": int(modulated_collision["invalid_point_count"]),
        "modulated_invalid_edge_count": int(modulated_collision["invalid_edge_count"]),
        "modulated_min_clearance": _json_clearance(modulated_clearance),
        "safety_margin": float(safety_margin),
        "avoidance_success": bool(avoidance_success),
        "path_length_nominal": float(nominal_length),
        "path_length_modulated": float(modulated_length),
        "path_length_ratio": float(modulated_length / max(nominal_length, 1e-12)),
        "jerk_nominal": float(nominal_jerk),
        "jerk_modulated": float(modulated_jerk),
        "smoothness_gain": float(nominal_jerk - modulated_jerk),
        "rrt_stop_reason": str(rrt_result.get("stop_reason", "not_required")),
        "rrt_iter_used": int(rrt_result.get("iter_used", 0)),
        "rrt_collision_queries": int(rrt_result.get("collision_queries", 0)),
        "rrt_elapsed_ms": float(rrt_result.get("elapsed_ms", 0.0)),
        "cart_waypoint_count": int(cart_waypoint_count),
        "ur_move_available": bool(ur_move_result.get("available", False)),
        "ur_move_transport": str(ur_move_result.get("transport", "")),
        "moveit_plan_success": moveit_success,
        "execution_requested": execution_requested,
        "execution_success": execution_success,
        "execution_id": execution_id,
        # 2D metrics remain interpretable even when the external MoveIt service is offline.
        "result_interpretable": True,
        "failure_reason": str(failure_reason),
    }


def write_evaluation(result: Dict[str, Any], out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    with (out_dir / "result.json").open("w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False, allow_nan=False)
    with (out_dir / "result.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(result.keys()))
        writer.writeheader()
        writer.writerow(result)
