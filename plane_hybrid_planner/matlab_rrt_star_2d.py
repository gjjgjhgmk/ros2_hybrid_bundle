"""MATLAB-compatible intent-biased Informed RRT* for normalized 2D paths.

The public interface uses UV coordinates. Internally all geometry is scaled to
the MATLAB script's [0, 100] workspace so its numeric parameters remain
unchanged.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np


MATLAB_SCALE = 100.0


@dataclass(frozen=True)
class MatlabRRTStarConfig:
    step_size: float = 1.5
    r_neighbor: float = 4.0
    max_iter: int = 6000
    edge_sample_step: float = 0.5
    rrt_inflation: float = 0.5
    convergence_window: int = 200
    convergence_epsilon_rel: float = 1e-3
    convergence_hits: int = 3
    lambda_k: float = 0.15
    enable_refine: bool = True
    refine_budget: int = 100
    enable_intent_bias: bool = True
    p_intent_pre: float = 0.55
    p_goal_pre: float = 0.15
    p_uniform_pre: float = 0.30
    p_intent_post: float = 0.65
    p_informed_post: float = 0.30
    p_goal_post: float = 0.05
    sigma_intent: float = 1.2
    post_intent_max_retry: int = 10

    @classmethod
    def from_mapping(cls, values: Dict[str, Any] | None) -> "MatlabRRTStarConfig":
        values = values or {}
        known = {field.name for field in cls.__dataclass_fields__.values()}
        return cls(**{key: values[key] for key in values if key in known})


class _CollisionCounter:
    def __init__(self, obstacles: Iterable[Dict[str, Any]], inflation: float, sample_step: float):
        self.obstacles = [
            {
                "center": np.asarray(obstacle["center"], dtype=float) * MATLAB_SCALE,
                "radius": float(obstacle["radius"]) * MATLAB_SCALE,
            }
            for obstacle in obstacles
        ]
        self.inflation = float(inflation)
        self.sample_step = float(sample_step)
        self.queries = 0

    def point_valid(self, point: np.ndarray) -> bool:
        self.queries += 1
        for obstacle in self.obstacles:
            if np.linalg.norm(point - obstacle["center"]) <= obstacle["radius"] + self.inflation:
                return False
        return True

    def edge_valid(self, first: np.ndarray, second: np.ndarray) -> bool:
        edge_len = float(np.linalg.norm(second - first))
        sample_count = max(2, int(math.ceil(edge_len / self.sample_step)))
        for sample in range(sample_count + 1):
            ratio = sample / sample_count
            if not self.point_valid(first + ratio * (second - first)):
                return False
        return True


def _path_objective(path: np.ndarray, lambda_k: float) -> float:
    if path.shape[0] < 2:
        return math.inf
    segments = np.diff(path, axis=0)
    length = float(np.sum(np.linalg.norm(segments, axis=1)))
    if segments.shape[0] < 2:
        return length
    headings = np.arctan2(segments[:, 1], segments[:, 0])
    delta = np.diff(headings)
    delta = np.arctan2(np.sin(delta), np.cos(delta))
    return length + float(lambda_k) * float(np.sum(np.abs(delta)))


def _backtrack(tree: List[List[float]], goal_index: int, goal: np.ndarray) -> np.ndarray:
    path = [goal.copy()]
    current = int(goal_index)
    while current >= 0:
        path.append(np.asarray(tree[current][0:2], dtype=float))
        current = int(tree[current][2])
    path.reverse()
    return np.asarray(path, dtype=float)


def _sample_informed(
    rng: np.random.RandomState,
    center: np.ndarray,
    rotation: np.ndarray,
    c_best: float,
    c_min: float,
) -> np.ndarray:
    if not math.isfinite(c_best):
        return center.copy()
    radius_major = c_best / 2.0
    radius_minor = math.sqrt(max(c_best * c_best - c_min * c_min, np.finfo(float).eps)) / 2.0
    radial = math.sqrt(float(rng.rand()))
    angle = 2.0 * math.pi * float(rng.rand())
    unit_ball = np.asarray([radial * math.cos(angle), radial * math.sin(angle)])
    return rotation @ np.diag([radius_major, radius_minor]) @ unit_ball + center


def _inside_informed(point: np.ndarray, start: np.ndarray, goal: np.ndarray, c_best: float) -> bool:
    return float(np.linalg.norm(point - start) + np.linalg.norm(point - goal)) <= c_best + 1e-9


def _sample_intent_biased(
    rng: np.random.RandomState,
    found_first: bool,
    c_best: float,
    start: np.ndarray,
    goal: np.ndarray,
    intent: np.ndarray,
    center: np.ndarray,
    rotation: np.ndarray,
    c_min: float,
    bounds: Tuple[float, float, float, float],
    cfg: MatlabRRTStarConfig,
) -> Tuple[np.ndarray, bool]:
    x_min, x_max, y_min, y_max = bounds
    use_intent = cfg.enable_intent_bias and intent.size > 0
    sampled_informed = False
    random_value = float(rng.rand())

    if not found_first:
        if use_intent and random_value < cfg.p_intent_pre:
            index = int(rng.randint(intent.shape[0]))
            node = intent[index] + cfg.sigma_intent * rng.randn(2)
        elif random_value < cfg.p_intent_pre + cfg.p_goal_pre:
            node = goal.copy()
        else:
            node = np.asarray([rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)])
        return node, sampled_informed

    if use_intent and random_value < cfg.p_intent_post:
        node = None
        for _ in range(cfg.post_intent_max_retry):
            index = int(rng.randint(intent.shape[0]))
            candidate = intent[index] + cfg.sigma_intent * rng.randn(2)
            if not math.isfinite(c_best) or _inside_informed(candidate, start, goal, c_best):
                node = candidate
                break
        if node is None:
            node = _sample_informed(rng, center, rotation, c_best, c_min)
            sampled_informed = True
    elif random_value < cfg.p_intent_post + cfg.p_informed_post:
        node = _sample_informed(rng, center, rotation, c_best, c_min)
        sampled_informed = True
    elif random_value < cfg.p_intent_post + cfg.p_informed_post + cfg.p_goal_post:
        node = goal.copy()
    else:
        node = np.asarray([rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)])
    return node, sampled_informed


def plan_intent_biased_informed_rrt_star(
    start_uv: Sequence[float],
    goal_uv: Sequence[float],
    intent_uv: Iterable[Sequence[float]],
    obstacles: Iterable[Dict[str, Any]],
    *,
    config: MatlabRRTStarConfig | None = None,
    rng_seed: int = 2027,
) -> Dict[str, Any]:
    """Port of ``get_intent_biased_rrt_paths`` from the MATLAB script."""
    cfg = config or MatlabRRTStarConfig()
    start = np.asarray(start_uv, dtype=float) * MATLAB_SCALE
    goal = np.asarray(goal_uv, dtype=float) * MATLAB_SCALE
    intent = np.asarray(intent_uv, dtype=float) * MATLAB_SCALE
    if start.shape != (2,) or goal.shape != (2,) or intent.ndim != 2 or intent.shape[1] != 2:
        raise ValueError("start/goal must be 2D and intent_uv must be shaped (N, 2)")

    collision = _CollisionCounter(obstacles, cfg.rrt_inflation, cfg.edge_sample_step)
    started = time.perf_counter()
    if not collision.point_valid(start):
        return _failure("collision_start", collision.queries, started)
    if not collision.point_valid(goal):
        return _failure("collision_goal", collision.queries, started)

    rng = np.random.RandomState(int(rng_seed))
    c_best = math.inf
    c_min = float(np.linalg.norm(goal - start))
    center = (start + goal) / 2.0
    direction = (goal - start) / max(c_min, np.finfo(float).eps)
    angle = math.atan2(direction[1], direction[0])
    rotation = np.asarray([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    bounds = (
        min(start[0], goal[0]) - 20.0,
        max(start[0], goal[0]) + 20.0,
        min(start[1], goal[1]) - 20.0,
        max(start[1], goal[1]) + 20.0,
    )

    # [x, y, parent_index], where -1 is the MATLAB parent 0 sentinel.
    tree: List[List[float]] = [[float(start[0]), float(start[1]), -1, 0.0]]
    goal_index = -1
    found_first = False
    path_first = np.empty((0, 2), dtype=float)
    path_refine = np.empty((0, 2), dtype=float)
    t_first = math.nan
    t_refine = math.nan
    refine_hit = False
    refine_expand_count = 0
    best_history = np.full(cfg.max_iter, math.inf, dtype=float)
    convergence_count = 0
    stop_reason = "max_iter_guard"
    iter_used = cfg.max_iter

    for iteration in range(1, cfg.max_iter + 1):
        random_node, sampled_informed = _sample_intent_biased(
            rng, found_first, c_best, start, goal, intent, center, rotation, c_min, bounds, cfg
        )
        if (
            not sampled_informed
            and found_first
            and math.isfinite(c_best)
            and np.linalg.norm(random_node - start) + np.linalg.norm(random_node - goal) > c_best
        ):
            random_node = _sample_informed(rng, center, rotation, c_best, c_min)

        coordinates = np.asarray([row[0:2] for row in tree], dtype=float)
        distances = np.linalg.norm(coordinates - random_node, axis=1)
        nearest_index = int(np.argmin(distances))
        nearest = coordinates[nearest_index]
        theta = math.atan2(random_node[1] - nearest[1], random_node[0] - nearest[0])
        new_node = nearest + cfg.step_size * np.asarray([math.cos(theta), math.sin(theta)])

        if not collision.point_valid(new_node) or not collision.edge_valid(nearest, new_node):
            if iteration > 1:
                best_history[iteration - 1] = best_history[iteration - 2]
            continue

        coordinates = np.asarray([row[0:2] for row in tree], dtype=float)
        distances = np.linalg.norm(coordinates - new_node, axis=1)
        neighbors = np.flatnonzero(distances <= cfg.r_neighbor)
        best_parent = nearest_index
        min_cost = float(tree[nearest_index][3] + distances[nearest_index])
        for index in neighbors:
            candidate_cost = float(tree[index][3] + distances[index])
            if candidate_cost < min_cost and collision.edge_valid(coordinates[index], new_node):
                best_parent = int(index)
                min_cost = candidate_cost

        new_index = len(tree)
        tree.append([float(new_node[0]), float(new_node[1]), best_parent, min_cost])
        for index in neighbors:
            if int(index) == best_parent:
                continue
            cost_via_new = min_cost + float(np.linalg.norm(coordinates[index] - new_node))
            if cost_via_new < tree[index][3] and collision.edge_valid(new_node, coordinates[index]):
                tree[index][2] = new_index
                tree[index][3] = cost_via_new

        if found_first:
            refine_expand_count += 1
            if cfg.enable_refine and not refine_hit and refine_expand_count >= max(cfg.refine_budget, 0):
                path_refine = _backtrack(tree, goal_index, goal)
                t_refine = time.perf_counter() - started
                refine_hit = True
                stop_reason = "budget_hit"

        if np.linalg.norm(new_node - goal) <= cfg.step_size and collision.edge_valid(new_node, goal):
            cost_to_goal = min_cost + float(np.linalg.norm(new_node - goal))
            if cost_to_goal < c_best:
                c_best = cost_to_goal
                goal_index = new_index
                if not found_first:
                    found_first = True
                    t_first = time.perf_counter() - started
                    path_first = _backtrack(tree, goal_index, goal)
                    if not cfg.enable_refine or cfg.refine_budget == 0:
                        path_refine = path_first.copy()
                        t_refine = t_first
                        refine_hit = True

        if found_first:
            best_history[iteration - 1] = _path_objective(_backtrack(tree, goal_index, goal), cfg.lambda_k)
            window = cfg.convergence_window
            if iteration % window == 0 and iteration >= 2 * window:
                previous = best_history[iteration - window - 1]
                current = best_history[iteration - 1]
                relative = (
                    (previous - current) / max(previous, np.finfo(float).eps)
                    if math.isfinite(previous) and math.isfinite(current)
                    else math.inf
                )
                convergence_count = convergence_count + 1 if relative < cfg.convergence_epsilon_rel else 0
                if convergence_count >= cfg.convergence_hits:
                    stop_reason = "converged"
                    iter_used = iteration
                    break
        elif iteration > 1:
            best_history[iteration - 1] = best_history[iteration - 2]

    elapsed = time.perf_counter() - started
    if not found_first:
        return _failure("failed", collision.queries, started, iter_used=cfg.max_iter)

    path_converged = _backtrack(tree, goal_index, goal)
    if path_refine.size == 0:
        path_refine = path_converged.copy()
        t_refine = elapsed
    selected = path_refine
    return {
        "ok": True,
        "path": (selected / MATLAB_SCALE).tolist(),
        "path_first": (path_first / MATLAB_SCALE).tolist(),
        "path_refine": (path_refine / MATLAB_SCALE).tolist(),
        "path_converged": (path_converged / MATLAB_SCALE).tolist(),
        "stop_reason": stop_reason,
        "iter_used": int(iter_used),
        "collision_queries": int(collision.queries),
        "elapsed_ms": float(elapsed * 1000.0),
        "t_first_ms": float(t_first * 1000.0),
        "t_refine_ms": float(t_refine * 1000.0),
        "refine_expand_count": int(refine_expand_count),
        "internal_scale": MATLAB_SCALE,
    }


def _failure(
    reason: str,
    queries: int,
    started: float,
    *,
    iter_used: int = 0,
) -> Dict[str, Any]:
    return {
        "ok": False,
        "path": [],
        "path_first": [],
        "path_refine": [],
        "path_converged": [],
        "stop_reason": reason,
        "iter_used": int(iter_used),
        "collision_queries": int(queries),
        "elapsed_ms": float((time.perf_counter() - started) * 1000.0),
        "t_first_ms": math.nan,
        "t_refine_ms": math.nan,
        "refine_expand_count": 0,
        "internal_scale": MATLAB_SCALE,
    }
