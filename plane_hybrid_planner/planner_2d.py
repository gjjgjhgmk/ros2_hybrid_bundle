"""Nominal path generation, circle collision checking, and 2D RRT-Connect."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .path_resample import resample_path_by_arclength


def _point2(value: Sequence[float], name: str = "point") -> np.ndarray:
    point = np.asarray(value, dtype=float).reshape(-1)
    if point.size != 2 or not np.all(np.isfinite(point)):
        raise ValueError(f"{name} must contain two finite values")
    return point


def _affine_fit_to_box(
    path: np.ndarray,
    *,
    u_range: Sequence[float],
    v_range: Sequence[float],
) -> np.ndarray:
    u_min, u_max = (float(u_range[0]), float(u_range[1]))
    v_min, v_max = (float(v_range[0]), float(v_range[1]))
    if not u_max > u_min or not v_max > v_min:
        raise ValueError("u_range and v_range must satisfy max > min")
    fitted = path.copy()
    x_values = fitted[:, 0]
    y_values = fitted[:, 1]
    x_span = float(np.max(x_values) - np.min(x_values))
    y_span = float(np.max(y_values) - np.min(y_values))
    if x_span <= 1e-12 or y_span <= 1e-12:
        raise ValueError("path must span both x and y to be auto-fitted")
    fitted[:, 0] = u_min + (x_values - np.min(x_values)) / x_span * (u_max - u_min)
    fitted[:, 1] = v_min + (y_values - np.min(y_values)) / y_span * (v_max - v_min)
    return fitted


def normalize_obstacles(obstacles: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    normalized: List[Dict[str, Any]] = []
    for index, obstacle in enumerate(obstacles or []):
        if str(obstacle.get("type", "circle")).lower() != "circle":
            raise ValueError(f"obstacle {index}: only circle is supported in phase 1")
        center = _point2(obstacle["center"], f"obstacle {index} center")
        radius = float(obstacle["radius"])
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError(f"obstacle {index}: radius must be positive")
        normalized.append(
            {"type": "circle", "center": center.tolist(), "radius": radius}
        )
    return normalized


def generate_nominal_path(spec: Dict[str, Any]) -> np.ndarray:
    path_type = str(spec.get("type", "line")).lower()
    num_points = int(spec.get("num_points", 120))
    if num_points < 2:
        raise ValueError("nominal num_points must be at least 2")

    if path_type == "polyline":
        points = np.asarray(spec.get("points", []), dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] < 2:
            raise ValueError("polyline nominal path requires at least two 2D points")
        path = resample_path_by_arclength(points, num_points)
    elif path_type == "matlab_sine":
        x_line = np.linspace(0.0, 100.0, num_points)
        amplitude_raw = float(spec.get("amplitude_raw", 30.0))
        center_raw = float(spec.get("center_raw", 50.0))
        cycles = float(spec.get("cycles", 1.0))
        y_line = center_raw + amplitude_raw * np.sin(cycles * x_line * math.pi / 50.0)
        raw_path = np.column_stack((x_line, y_line))
        path = _affine_fit_to_box(
            raw_path,
            u_range=spec.get("u_range", [0.12, 0.88]),
            v_range=spec.get("v_range", [0.22, 0.78]),
        )
    else:
        start = _point2(spec["start"], "nominal start")
        goal = _point2(spec["goal"], "nominal goal")
        alpha = np.linspace(0.0, 1.0, num_points)
        path = start[None, :] + alpha[:, None] * (goal - start)[None, :]
        if path_type == "sine":
            delta = goal - start
            length = float(np.linalg.norm(delta))
            if length <= 1e-12:
                raise ValueError("sine nominal path requires distinct start and goal")
            normal = np.asarray([-delta[1], delta[0]], dtype=float) / length
            amplitude = float(spec.get("amplitude", 0.08))
            cycles = float(spec.get("cycles", 1.0))
            path += (
                amplitude * np.sin(2.0 * math.pi * cycles * alpha)
            )[:, None] * normal[None, :]
        elif path_type != "line":
            raise ValueError(f"unsupported nominal path type: {path_type}")

    if np.any(path < 0.0) or np.any(path > 1.0):
        raise ValueError("nominal path leaves normalized [0, 1] plane")
    return path


def point_clearance(
    point: Sequence[float], obstacles: Iterable[Dict[str, Any]]
) -> float:
    obs = normalize_obstacles(obstacles)
    if not obs:
        return float("inf")
    p = _point2(point)
    return min(
        float(np.linalg.norm(p - np.asarray(item["center"], dtype=float)) - item["radius"])
        for item in obs
    )


def point_in_collision(
    point: Sequence[float], obstacles: Iterable[Dict[str, Any]], margin: float = 0.0
) -> bool:
    return point_clearance(point, obstacles) <= float(margin)


def _point_to_segment_distance(point: np.ndarray, start: np.ndarray, goal: np.ndarray) -> float:
    delta = goal - start
    denom = float(np.dot(delta, delta))
    if denom <= 1e-18:
        return float(np.linalg.norm(point - start))
    alpha = float(np.clip(np.dot(point - start, delta) / denom, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + alpha * delta)))


def segment_clearance(
    start: Sequence[float],
    goal: Sequence[float],
    obstacles: Iterable[Dict[str, Any]],
) -> float:
    obs = normalize_obstacles(obstacles)
    if not obs:
        return float("inf")
    p0 = _point2(start, "segment start")
    p1 = _point2(goal, "segment goal")
    return min(
        _point_to_segment_distance(np.asarray(item["center"], dtype=float), p0, p1)
        - float(item["radius"])
        for item in obs
    )


def segment_in_collision(
    start: Sequence[float],
    goal: Sequence[float],
    obstacles: Iterable[Dict[str, Any]],
    margin: float = 0.0,
) -> bool:
    return segment_clearance(start, goal, obstacles) <= float(margin)


def path_collision_details(
    path: Iterable[Sequence[float]],
    obstacles: Iterable[Dict[str, Any]],
    margin: float = 0.0,
) -> Dict[str, Any]:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] == 0:
        raise ValueError("path must be a non-empty array shaped (N, 2)")
    point_invalid = [
        index
        for index, point in enumerate(arr)
        if point_in_collision(point, obstacles, margin=margin)
    ]
    edge_invalid = [
        index
        for index in range(arr.shape[0] - 1)
        if segment_in_collision(arr[index], arr[index + 1], obstacles, margin=margin)
    ]
    return {
        "collision_count": len(point_invalid) + len(edge_invalid),
        "invalid_point_count": len(point_invalid),
        "invalid_edge_count": len(edge_invalid),
        "invalid_point_indices": point_invalid,
        "invalid_edge_indices": edge_invalid,
    }


def path_min_clearance(
    path: Iterable[Sequence[float]], obstacles: Iterable[Dict[str, Any]]
) -> float:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] == 0:
        raise ValueError("path must be a non-empty array shaped (N, 2)")
    obs = normalize_obstacles(obstacles)
    if not obs:
        return float("inf")
    values = [point_clearance(point, obs) for point in arr]
    values.extend(segment_clearance(arr[i], arr[i + 1], obs) for i in range(arr.shape[0] - 1))
    return float(min(values))


@dataclass
class _Tree:
    nodes: List[np.ndarray]
    parents: List[int]

    @classmethod
    def rooted_at(cls, root: np.ndarray) -> "_Tree":
        return cls(nodes=[root.copy()], parents=[-1])

    def add(self, point: np.ndarray, parent: int) -> int:
        self.nodes.append(point.copy())
        self.parents.append(int(parent))
        return len(self.nodes) - 1

    def nearest(self, target: np.ndarray) -> int:
        distances = [float(np.dot(node - target, node - target)) for node in self.nodes]
        return int(np.argmin(distances))

    def trace_root_to(self, index: int) -> List[np.ndarray]:
        path: List[np.ndarray] = []
        while index >= 0:
            path.append(self.nodes[index])
            index = self.parents[index]
        return list(reversed(path))


class _CollisionCounter:
    def __init__(self, obstacles: List[Dict[str, Any]], margin: float):
        self.obstacles = obstacles
        self.margin = float(margin)
        self.queries = 0

    def point_valid(self, point: np.ndarray) -> bool:
        self.queries += 1
        in_bounds = bool(np.all(point >= 0.0) and np.all(point <= 1.0))
        return in_bounds and not point_in_collision(point, self.obstacles, self.margin)

    def edge_valid(self, start: np.ndarray, goal: np.ndarray) -> bool:
        self.queries += 1
        return not segment_in_collision(start, goal, self.obstacles, self.margin)


def rrt_connect(
    start: Sequence[float],
    goal: Sequence[float],
    obstacles: Iterable[Dict[str, Any]],
    step_size: float = 0.04,
    max_iter: int = 1000,
    goal_tolerance: float = 0.04,
    rng_seed: int = 42,
    margin: float = 0.0,
    timeout_sec: float = 0.0,
) -> Dict[str, Any]:
    start_arr = _point2(start, "start")
    goal_arr = _point2(goal, "goal")
    obstacle_list = normalize_obstacles(obstacles)
    step_size = float(step_size)
    goal_tolerance = float(goal_tolerance)
    max_iter = int(max_iter)
    if step_size <= 0.0 or goal_tolerance < 0.0 or max_iter <= 0:
        raise ValueError("step_size/max_iter must be positive and goal_tolerance non-negative")

    checks = _CollisionCounter(obstacle_list, margin=margin)
    started = time.monotonic()

    def result(ok: bool, path: List[np.ndarray], reason: str, iterations: int) -> Dict[str, Any]:
        return {
            "ok": bool(ok),
            "path": [point.tolist() for point in path] if ok else [],
            "stop_reason": reason,
            "iter_used": int(iterations),
            "collision_queries": int(checks.queries),
            "elapsed_ms": float((time.monotonic() - started) * 1000.0),
        }

    if not checks.point_valid(start_arr):
        return result(False, [], "collision_start", 0)
    if not checks.point_valid(goal_arr):
        return result(False, [], "collision_goal", 0)
    if checks.edge_valid(start_arr, goal_arr):
        return result(True, [start_arr, goal_arr], "direct", 0)

    rng = np.random.default_rng(int(rng_seed))
    start_tree = _Tree.rooted_at(start_arr)
    goal_tree = _Tree.rooted_at(goal_arr)

    def steer(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, bool]:
        delta = target - source
        distance = float(np.linalg.norm(delta))
        if distance <= step_size:
            return target.copy(), True
        return source + delta * (step_size / max(distance, 1e-12)), False

    def extend(tree: _Tree, target: np.ndarray) -> Tuple[str, int]:
        near_index = tree.nearest(target)
        candidate, reached = steer(tree.nodes[near_index], target)
        if not checks.point_valid(candidate) or not checks.edge_valid(tree.nodes[near_index], candidate):
            return "trapped", near_index
        new_index = tree.add(candidate, near_index)
        if reached or float(np.linalg.norm(candidate - target)) <= goal_tolerance:
            if not np.allclose(candidate, target) and checks.point_valid(target) and checks.edge_valid(candidate, target):
                new_index = tree.add(target, new_index)
            return "reached", new_index
        return "advanced", new_index

    def connect(tree: _Tree, target: np.ndarray) -> Tuple[str, int]:
        near = tree.nodes[tree.nearest(target)]
        max_steps = int(math.ceil(float(np.linalg.norm(target - near)) / step_size)) + 2
        last_index = tree.nearest(target)
        for _ in range(max_steps):
            status, last_index = extend(tree, target)
            if status != "advanced":
                return status, last_index
        return "trapped", last_index

    for iteration in range(1, max_iter + 1):
        if timeout_sec > 0.0 and time.monotonic() - started >= float(timeout_sec):
            return result(False, [], "timeout", iteration - 1)

        active_is_start = iteration % 2 == 1
        active = start_tree if active_is_start else goal_tree
        other = goal_tree if active_is_start else start_tree
        sample = rng.uniform(0.0, 1.0, size=2)
        status, active_index = extend(active, sample)
        if status == "trapped":
            continue

        target = active.nodes[active_index]
        connect_status, other_index = connect(other, target)
        if connect_status != "reached":
            continue

        if active_is_start:
            start_part = active.trace_root_to(active_index)
            goal_part = other.trace_root_to(other_index)
        else:
            start_part = other.trace_root_to(other_index)
            goal_part = active.trace_root_to(active_index)
        tail = list(reversed(goal_part))
        if start_part and tail and np.allclose(start_part[-1], tail[0]):
            tail = tail[1:]
        path = start_part + tail
        if not np.allclose(path[0], start_arr) or not np.allclose(path[-1], goal_arr):
            return result(False, [], "failed_connect", iteration)
        if any(not checks.edge_valid(path[i], path[i + 1]) for i in range(len(path) - 1)):
            return result(False, [], "failed_connect", iteration)
        return result(True, path, "success", iteration)

    return result(False, [], "max_iter", max_iter)
