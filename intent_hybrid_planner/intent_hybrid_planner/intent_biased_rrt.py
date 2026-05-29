#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Intent-biased informed RRT* planner (pure NumPy module).

Design goals:
- N-dimensional planning (works for 6-DoF joint vectors).
- Intent-biased mixture sampling.
- Informed hyperellipsoid sampling with SVD-based alignment.
- Decoupled collision checking via callback.
- Real-time friendly loop with explicit timeout break.
"""

from __future__ import annotations

import time
import warnings
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np


class IntentBiasedRRT:
    """
    Intent-biased informed RRT* planner.

    Collision callback convention:
    - collision_checker_fn(state) returns True when `state` is valid/collision-free.
    - returns False when in collision/invalid.
    """

    def __init__(
        self,
        collision_checker_fn: Callable[[np.ndarray], bool],
        edge_checker_fn: Optional[Callable[[np.ndarray, np.ndarray, int], bool]] = None,
        *,
        step_size: float = 0.35,
        r_neighbor: float = 0.9,
        max_iter: int = 80,
        timeout_sec: float = 0.04,
        max_edge_samples: int = 6,
        edge_sample_step: float = 0.0,
        p_intent: float = 0.60,
        p_goal: float = 0.10,
        p_uniform: float = 0.30,
        sigma_intent: float = 0.08,
        post_intent_max_retry: int = 10,
        min_sampling_span: float = 2.0,
        state_min: Optional[np.ndarray] = None,
        state_max: Optional[np.ndarray] = None,
        rng_seed: Optional[int] = None,
    ) -> None:
        if not callable(collision_checker_fn):
            raise TypeError("collision_checker_fn must be callable.")

        self.collision_checker_fn = collision_checker_fn
        self.edge_checker_fn = edge_checker_fn
        self.step_size = float(max(step_size, 1e-6))
        self.r_neighbor = float(max(r_neighbor, self.step_size))
        self.max_iter = int(max(max_iter, 1))
        # timeout_sec <= 0 means "no wall-time cutoff", only max_iter limits the loop.
        self.timeout_sec = float(timeout_sec)
        self.max_edge_samples = int(max(max_edge_samples, 2))
        # <= 0 means auto from step_size (MATLAB-style ratio ~= 1/3 step).
        if float(edge_sample_step) > 0.0:
            self.edge_sample_step = float(edge_sample_step)
        else:
            self.edge_sample_step = float(max(self.step_size / 3.0, 1e-6))
        self.sigma_intent = float(max(sigma_intent, 0.0))
        self.post_intent_max_retry = int(max(post_intent_max_retry, 1))
        self.min_sampling_span = float(max(min_sampling_span, 1e-3))

        # Normalize mixture probabilities.
        probs = np.array([p_intent, p_goal, p_uniform], dtype=float)
        probs = np.maximum(probs, 0.0)
        denom = float(np.sum(probs))
        if denom <= 0.0:
            probs = np.array([0.6, 0.1, 0.3], dtype=float)
            denom = 1.0
        probs /= denom
        self.p_intent, self.p_goal, self.p_uniform = probs.tolist()

        self.state_min = None if state_min is None else np.asarray(state_min, dtype=float).reshape(-1)
        self.state_max = None if state_max is None else np.asarray(state_max, dtype=float).reshape(-1)
        self.rng = np.random.default_rng(rng_seed)

        self._collision_queries = 0
        self._eps = 1e-12
        self._time_eps = 1e-3

    def plan(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        intent_path: np.ndarray,
        t_start: float,
        t_end: float,
    ) -> Tuple[np.ndarray, np.ndarray, dict]:
        """
        Plan via points and via times.

        Returns:
        - via_points: (D, M), D is state dimension (6 for 6-DoF use case).
        - via_times:  (M,), strictly monotonic.
        - meta: {'iter_used', 'time_ms', 'collision_queries', 'timeout_hit'}
        """
        wall_t0 = time.time()
        self._collision_queries = 0

        s = np.asarray(start, dtype=float).reshape(-1)
        g = np.asarray(goal, dtype=float).reshape(-1)
        if s.size == 0 or g.size == 0:
            raise ValueError("start/goal cannot be empty.")
        if s.shape != g.shape:
            raise ValueError(f"start and goal shape mismatch: {s.shape} vs {g.shape}")
        dim = s.size

        intent_pts = self._normalize_intent_path(intent_path, dim)  # (K, D)
        low, high = self._compute_sampling_bounds(s, g, intent_pts)

        # Degenerate case: identical start and goal.
        if np.linalg.norm(g - s) <= self._eps:
            path = np.vstack([s, g])
            via_times = self._allocate_via_times(path, float(t_start), float(t_end))
            meta = {
                "iter_used": 0,
                "time_ms": (time.time() - wall_t0) * 1000.0,
                "collision_queries": int(self._collision_queries),
                "timeout_hit": False,
            }
            return path.T, via_times, meta

        # RRT* tree buffers (fixed-size for real-time predictability).
        max_nodes = self.max_iter + 2
        states = np.empty((max_nodes, dim), dtype=float)
        parents = np.full(max_nodes, -1, dtype=int)
        costs = np.full(max_nodes, np.inf, dtype=float)

        states[0] = s
        costs[0] = 0.0
        node_count = 1

        c_min = float(np.linalg.norm(g - s))
        c_best = float("inf")
        center = 0.5 * (s + g)
        c_mat = self._compute_alignment_matrix(s, g)  # SVD-based
        found_first = False
        best_goal_parent = -1
        timeout_hit = False
        iter_used = 0

        for it in range(self.max_iter):
            # Hard real-time budget guard.
            if self.timeout_sec > 0.0 and (time.time() - wall_t0) > self.timeout_sec:
                timeout_hit = True
                warnings.warn(
                    f"IntentBiasedRRT timeout: {self.timeout_sec * 1000:.1f}ms budget hit.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                break
            iter_used = it + 1

            rand_node = self._sample_mixture(
                start=s,
                goal=g,
                intent_pts=intent_pts,
                found_first=found_first,
                c_best=c_best,
                c_min=c_min,
                center=center,
                c_mat=c_mat,
                low=low,
                high=high,
            )

            # Nearest neighbor (vectorized).
            diff = states[:node_count] - rand_node[None, :]
            dist = np.linalg.norm(diff, axis=1)
            nearest_idx = int(np.argmin(dist))
            nearest = states[nearest_idx]

            # Steer.
            delta = rand_node - nearest
            d = float(np.linalg.norm(delta))
            if d <= self.step_size:
                new_state = rand_node
            else:
                new_state = nearest + (self.step_size / max(d, self._eps)) * delta

            if not self._is_state_valid(new_state):
                continue
            if not self._is_edge_valid(nearest, new_state):
                continue

            # Find neighbors for best-parent and rewiring.
            d_all = np.linalg.norm(states[:node_count] - new_state[None, :], axis=1)
            neighbors = np.flatnonzero(d_all <= self.r_neighbor)

            best_parent = nearest_idx
            min_cost = float(costs[nearest_idx] + d_all[nearest_idx])

            for idx in neighbors:
                cost_try = float(costs[idx] + d_all[idx])
                if cost_try + 1e-12 < min_cost and self._is_edge_valid(states[idx], new_state):
                    best_parent = int(idx)
                    min_cost = cost_try

            new_idx = node_count
            states[new_idx] = new_state
            parents[new_idx] = best_parent
            costs[new_idx] = min_cost
            node_count += 1

            for idx in neighbors:
                idx = int(idx)
                if idx == best_parent:
                    continue
                cost_rewire = float(min_cost + np.linalg.norm(states[idx] - new_state))
                if cost_rewire + 1e-12 < costs[idx] and self._is_edge_valid(new_state, states[idx]):
                    parents[idx] = new_idx
                    costs[idx] = cost_rewire

            # Goal connect test.
            d_goal = float(np.linalg.norm(new_state - g))
            if d_goal <= self.step_size and self._is_edge_valid(new_state, g):
                cost_goal = float(min_cost + d_goal)
                if cost_goal + 1e-12 < c_best:
                    c_best = cost_goal
                    best_goal_parent = new_idx
                    found_first = True

        if best_goal_parent >= 0:
            path = self._backtrack_path(states, parents, best_goal_parent)
            if np.linalg.norm(path[-1] - g) > self._eps:
                path = np.vstack([path, g])
        else:
            path = self._fallback_path(states[:node_count], parents[:node_count], s, g)

        via_times = self._allocate_via_times(path, float(t_start), float(t_end))
        meta = {
            "iter_used": int(iter_used),
            "time_ms": (time.time() - wall_t0) * 1000.0,
            "collision_queries": int(self._collision_queries),
            "timeout_hit": bool(timeout_hit),
        }
        return path.T, via_times, meta

    def plan_detailed(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        intent_path: np.ndarray,
        t_start: float,
        t_end: float,
        *,
        refine_budget: int = 100,
        p_intent_pre: float = 0.55,
        p_goal_pre: float = 0.15,
        p_uniform_pre: float = 0.30,
        p_intent_post: float = 0.65,
        p_informed_post: float = 0.30,
        p_goal_post: float = 0.05,
    ) -> Dict[str, Any]:
        """
        MATLAB-compatible detailed planning result.

        Returns a dictionary containing:
        - path_first/path_refine/path_conv: (D, M*) via candidates
        - t_first/t_refine/t_conv: seconds
        - stop_reason: converged / budget_hit / max_iter_guard / timeout / failed
        - via_points/via_times: chosen path converted to via format
        - meta: planner statistics
        """
        wall_t0 = time.time()
        self._collision_queries = 0

        s = np.asarray(start, dtype=float).reshape(-1)
        g = np.asarray(goal, dtype=float).reshape(-1)
        if s.size == 0 or g.size == 0:
            raise ValueError("start/goal cannot be empty.")
        if s.shape != g.shape:
            raise ValueError(f"start and goal shape mismatch: {s.shape} vs {g.shape}")
        dim = s.size

        probs_pre = self._normalize_prob_triplet(p_intent_pre, p_goal_pre, p_uniform_pre)
        probs_post = self._normalize_prob_triplet(p_intent_post, p_goal_post, p_informed_post)

        intent_pts = self._normalize_intent_path(intent_path, dim)
        low, high = self._compute_sampling_bounds(s, g, intent_pts)

        if np.linalg.norm(g - s) <= self._eps:
            path = np.vstack([s, g])
            via_times = self._allocate_via_times(path, float(t_start), float(t_end))
            t_used = (time.time() - wall_t0)
            return {
                "path_first": path.T,
                "path_refine": path.T,
                "path_conv": path.T,
                "t_first": float(t_used),
                "t_refine": float(t_used),
                "t_conv": float(t_used),
                "stop_reason": "converged",
                "via_points": path.T,
                "via_times": via_times,
                "meta": {
                    "iter_used": 0,
                    "time_ms": t_used * 1000.0,
                    "collision_queries": int(self._collision_queries),
                    "timeout_hit": False,
                    "refine_budget": int(max(refine_budget, 0)),
                    "refine_expand_count": 0,
                },
            }

        max_nodes = self.max_iter + 2
        states = np.empty((max_nodes, dim), dtype=float)
        parents = np.full(max_nodes, -1, dtype=int)
        costs = np.full(max_nodes, np.inf, dtype=float)

        states[0] = s
        costs[0] = 0.0
        node_count = 1

        c_min = float(np.linalg.norm(g - s))
        c_best = float("inf")
        center = 0.5 * (s + g)
        c_mat = self._compute_alignment_matrix(s, g)

        found_first = False
        first_goal_parent = -1
        best_goal_parent = -1
        t_first = float("nan")
        t_refine = float("nan")
        timeout_hit = False
        iter_used = 0
        refine_expand_count = 0
        stop_reason = "max_iter_guard"
        budget_limit = int(max(refine_budget, 0))

        for it in range(self.max_iter):
            now_elapsed = time.time() - wall_t0
            if self.timeout_sec > 0.0 and now_elapsed > self.timeout_sec:
                timeout_hit = True
                stop_reason = "timeout"
                warnings.warn(
                    f"IntentBiasedRRT timeout: {self.timeout_sec * 1000:.1f}ms budget hit.",
                    RuntimeWarning,
                    stacklevel=2,
                )
                break
            iter_used = it + 1

            if found_first and np.isfinite(c_best):
                p_intent, p_goal, p_uniform = probs_post
            else:
                p_intent, p_goal, p_uniform = probs_pre

            rand_node = self._sample_mixture_with_probs(
                start=s,
                goal=g,
                intent_pts=intent_pts,
                found_first=found_first,
                c_best=c_best,
                c_min=c_min,
                center=center,
                c_mat=c_mat,
                low=low,
                high=high,
                p_intent=p_intent,
                p_goal=p_goal,
                p_uniform=p_uniform,
            )

            diff = states[:node_count] - rand_node[None, :]
            dist = np.linalg.norm(diff, axis=1)
            nearest_idx = int(np.argmin(dist))
            nearest = states[nearest_idx]

            delta = rand_node - nearest
            d = float(np.linalg.norm(delta))
            if d <= self.step_size:
                new_state = rand_node
            else:
                new_state = nearest + (self.step_size / max(d, self._eps)) * delta

            if not self._is_state_valid(new_state):
                continue
            if not self._is_edge_valid(nearest, new_state):
                continue

            d_all = np.linalg.norm(states[:node_count] - new_state[None, :], axis=1)
            neighbors = np.flatnonzero(d_all <= self.r_neighbor)

            best_parent = nearest_idx
            min_cost = float(costs[nearest_idx] + d_all[nearest_idx])
            for idx in neighbors:
                cost_try = float(costs[idx] + d_all[idx])
                if cost_try + 1e-12 < min_cost and self._is_edge_valid(states[idx], new_state):
                    best_parent = int(idx)
                    min_cost = cost_try

            new_idx = node_count
            states[new_idx] = new_state
            parents[new_idx] = best_parent
            costs[new_idx] = min_cost
            node_count += 1

            for idx in neighbors:
                idx = int(idx)
                if idx == best_parent:
                    continue
                cost_rewire = float(min_cost + np.linalg.norm(states[idx] - new_state))
                if cost_rewire + 1e-12 < costs[idx] and self._is_edge_valid(new_state, states[idx]):
                    parents[idx] = new_idx
                    costs[idx] = cost_rewire

            d_goal = float(np.linalg.norm(new_state - g))
            if d_goal <= self.step_size and self._is_edge_valid(new_state, g):
                cost_goal = float(min_cost + d_goal)
                if cost_goal + 1e-12 < c_best:
                    c_best = cost_goal
                    best_goal_parent = new_idx
                    if not found_first:
                        found_first = True
                        first_goal_parent = new_idx
                        t_first = float(time.time() - wall_t0)
                        if budget_limit <= 0:
                            t_refine = t_first
                            stop_reason = "budget_hit"
                            break

            if found_first:
                refine_expand_count += 1
                if budget_limit > 0 and refine_expand_count >= budget_limit:
                    t_refine = float(time.time() - wall_t0)
                    stop_reason = "budget_hit"
                    break

        t_conv = float(time.time() - wall_t0)

        if best_goal_parent < 0:
            empty_path = np.empty((dim, 0), dtype=float)
            return {
                "path_first": empty_path,
                "path_refine": empty_path,
                "path_conv": empty_path,
                "t_first": float("nan"),
                "t_refine": float("nan"),
                "t_conv": t_conv,
                "stop_reason": "failed" if stop_reason not in ("timeout",) else stop_reason,
                "via_points": empty_path,
                "via_times": np.empty((0,), dtype=float),
                "meta": {
                    "iter_used": int(iter_used),
                    "time_ms": t_conv * 1000.0,
                    "collision_queries": int(self._collision_queries),
                    "timeout_hit": bool(timeout_hit),
                    "refine_budget": int(budget_limit),
                    "refine_expand_count": int(refine_expand_count),
                },
            }

        path_conv = self._backtrack_path(states, parents, best_goal_parent)
        if np.linalg.norm(path_conv[-1] - g) > self._eps:
            path_conv = np.vstack([path_conv, g])

        if first_goal_parent >= 0:
            path_first = self._backtrack_path(states, parents, first_goal_parent)
            if np.linalg.norm(path_first[-1] - g) > self._eps:
                path_first = np.vstack([path_first, g])
        else:
            path_first = path_conv.copy()
            t_first = t_conv

        if np.isnan(t_first):
            t_first = t_conv
        if np.isnan(t_refine):
            t_refine = t_conv
        t_refine = max(t_refine, t_first)
        t_refine = min(t_refine, t_conv)

        if stop_reason == "max_iter_guard" and not timeout_hit:
            stop_reason = "converged"

        if stop_reason == "budget_hit":
            path_refine = path_conv.copy()
        else:
            path_refine = path_conv.copy()

        via_times = self._allocate_via_times(path_refine, float(t_start), float(t_end))
        meta = {
            "iter_used": int(iter_used),
            "time_ms": t_conv * 1000.0,
            "collision_queries": int(self._collision_queries),
            "timeout_hit": bool(timeout_hit),
            "refine_budget": int(budget_limit),
            "refine_expand_count": int(refine_expand_count),
        }
        return {
            "path_first": path_first.T,
            "path_refine": path_refine.T,
            "path_conv": path_conv.T,
            "t_first": float(t_first),
            "t_refine": float(t_refine),
            "t_conv": float(t_conv),
            "stop_reason": stop_reason,
            "via_points": path_refine.T,
            "via_times": via_times,
            "meta": meta,
        }

    def _normalize_intent_path(self, intent_path: np.ndarray, dim: int) -> np.ndarray:
        arr = np.asarray(intent_path, dtype=float)
        if arr.size == 0:
            return np.empty((0, dim), dtype=float)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)

        # Accept both (D, K) and (K, D).
        if arr.shape[0] == dim and arr.shape[1] != dim:
            arr = arr.T
        elif arr.shape[1] != dim and arr.shape[0] != dim:
            raise ValueError(
                f"intent_path shape {arr.shape} incompatible with state dimension {dim}."
            )
        elif arr.shape[1] != dim and arr.shape[0] == dim:
            arr = arr.T

        return np.asarray(arr, dtype=float)

    def _compute_sampling_bounds(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        intent_pts: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        pts = [start, goal]
        if intent_pts.size > 0:
            pts.append(intent_pts)
        stack = np.vstack(pts)
        lo = np.min(stack, axis=0)
        hi = np.max(stack, axis=0)

        # Ensure near-constant joints still have enough exploration bandwidth
        # to绕开障碍（否则会退化为几乎1维搜索，极易超时）。
        span = np.maximum(hi - lo, self.min_sampling_span)
        margin = 0.20 * span + 0.05
        lo = lo - margin
        hi = hi + margin

        if self.state_min is not None:
            if self.state_min.size != start.size:
                raise ValueError("state_min dimension mismatch with start/goal.")
            lo = np.maximum(lo, self.state_min)
        if self.state_max is not None:
            if self.state_max.size != start.size:
                raise ValueError("state_max dimension mismatch with start/goal.")
            hi = np.minimum(hi, self.state_max)

        # Ensure box is valid.
        lo = np.minimum(lo, hi - 1e-6)
        return lo, hi

    def _sample_mixture(
        self,
        *,
        start: np.ndarray,
        goal: np.ndarray,
        intent_pts: np.ndarray,
        found_first: bool,
        c_best: float,
        c_min: float,
        center: np.ndarray,
        c_mat: np.ndarray,
        low: np.ndarray,
        high: np.ndarray,
    ) -> np.ndarray:
        return self._sample_mixture_with_probs(
            start=start,
            goal=goal,
            intent_pts=intent_pts,
            found_first=found_first,
            c_best=c_best,
            c_min=c_min,
            center=center,
            c_mat=c_mat,
            low=low,
            high=high,
            p_intent=self.p_intent,
            p_goal=self.p_goal,
            p_uniform=self.p_uniform,
        )

    def _sample_mixture_with_probs(
        self,
        *,
        start: np.ndarray,
        goal: np.ndarray,
        intent_pts: np.ndarray,
        found_first: bool,
        c_best: float,
        c_min: float,
        center: np.ndarray,
        c_mat: np.ndarray,
        low: np.ndarray,
        high: np.ndarray,
        p_intent: float,
        p_goal: float,
        p_uniform: float,
    ) -> np.ndarray:
        r = float(self.rng.random())
        use_intent = intent_pts.shape[0] > 0

        if use_intent and r < p_intent:
            if found_first and np.isfinite(c_best):
                for _ in range(self.post_intent_max_retry):
                    cand = self._sample_intent(intent_pts)
                    if self._is_inside_informed(cand, start, goal, c_best):
                        return cand
                return self._sample_informed(center, c_mat, c_best, c_min, low, high)
            return self._sample_intent(intent_pts)

        if r < (p_intent + p_goal):
            return goal.copy()

        # Uniform branch: informed hyperellipsoid preferred; box fallback before first solution.
        if found_first and np.isfinite(c_best):
            return self._sample_informed(center, c_mat, c_best, c_min, low, high)
        return self._sample_uniform(low, high)

    def _normalize_prob_triplet(self, p_intent: float, p_goal: float, p_uniform: float) -> Tuple[float, float, float]:
        probs = np.array([p_intent, p_goal, p_uniform], dtype=float)
        probs = np.maximum(probs, 0.0)
        s = float(np.sum(probs))
        if s <= self._eps:
            return 0.6, 0.1, 0.3
        probs = probs / s
        return float(probs[0]), float(probs[1]), float(probs[2])

    def _sample_intent(self, intent_pts: np.ndarray) -> np.ndarray:
        idx = int(self.rng.integers(0, intent_pts.shape[0]))
        base = intent_pts[idx]
        if self.sigma_intent <= 0.0:
            return base.copy()
        return base + self.sigma_intent * self.rng.normal(size=base.shape[0])

    def _sample_uniform(self, low: np.ndarray, high: np.ndarray) -> np.ndarray:
        return low + self.rng.random(low.shape[0]) * (high - low)

    def _sample_informed(
        self,
        center: np.ndarray,
        c_mat: np.ndarray,
        c_best: float,
        c_min: float,
        low: np.ndarray,
        high: np.ndarray,
    ) -> np.ndarray:
        dim = center.size
        if (not np.isfinite(c_best)) or c_best <= 0.0:
            return self._sample_uniform(low, high)

        # Hyperellipsoid radii.
        c_best = max(float(c_best), float(c_min))
        r1 = c_best / 2.0
        if c_best <= c_min + 1e-12:
            r_other = 0.0
        else:
            r_other = np.sqrt(max(c_best * c_best - c_min * c_min, 0.0)) / 2.0
        radii = np.full(dim, r_other, dtype=float)
        radii[0] = r1

        x_ball = self._sample_unit_n_ball(dim)
        sample = center + c_mat @ (radii * x_ball)
        return np.clip(sample, low, high)

    def _sample_unit_n_ball(self, dim: int) -> np.ndarray:
        v = self.rng.normal(size=dim)
        n = float(np.linalg.norm(v))
        if n <= self._eps:
            v = np.zeros(dim, dtype=float)
            v[0] = 1.0
            n = 1.0
        direction = v / n
        radius = float(self.rng.random()) ** (1.0 / dim)
        return radius * direction

    def _compute_alignment_matrix(self, start: np.ndarray, goal: np.ndarray) -> np.ndarray:
        """
        Compute C_mat that aligns e1 with (goal-start) direction (N-D).
        Uses SVD first, Householder fallback for numerical robustness.
        """
        direction = goal - start
        dim = direction.size
        c_min = float(np.linalg.norm(direction))
        if c_min <= self._eps:
            return np.eye(dim, dtype=float)

        a1 = direction / c_min
        e1 = np.zeros(dim, dtype=float)
        e1[0] = 1.0

        # SVD-based proper rotation.
        m = np.outer(a1, e1)
        u, _, vt = np.linalg.svd(m, full_matrices=True)
        d = np.eye(dim, dtype=float)
        det_uv = float(np.linalg.det(u @ vt))
        d[-1, -1] = 1.0 if det_uv >= 0.0 else -1.0
        c_mat = u @ d @ vt

        if not np.all(np.isfinite(c_mat)) or np.linalg.norm(c_mat @ e1 - a1) > 1e-6:
            # Householder fallback: map e1 -> a1
            v = e1 - a1
            nv = float(np.linalg.norm(v))
            if nv <= self._eps:
                return np.eye(dim, dtype=float)
            v = v / nv
            c_mat = np.eye(dim, dtype=float) - 2.0 * np.outer(v, v)

        return c_mat

    def _is_inside_informed(
        self,
        point: np.ndarray,
        start: np.ndarray,
        goal: np.ndarray,
        c_best: float,
    ) -> bool:
        return (
            np.linalg.norm(point - start) + np.linalg.norm(point - goal)
            <= float(c_best) + 1e-9
        )

    def _is_state_valid(self, state: np.ndarray) -> bool:
        self._collision_queries += 1
        try:
            return bool(self.collision_checker_fn(np.asarray(state, dtype=float)))
        except Exception:
            return False

    def _is_edge_valid(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        # Adaptive edge sampling: n ~= ceil(edge_len / edge_sample_step), capped by max_edge_samples.
        edge_len = float(np.linalg.norm(np.asarray(p2, dtype=float) - np.asarray(p1, dtype=float)))
        if edge_len <= self._eps:
            n = 2
        else:
            n = int(np.ceil(edge_len / max(self.edge_sample_step, self._eps)))
            n = max(n, 2)
        n = min(int(max(n, 2)), int(max(self.max_edge_samples, 2)))
        if self.edge_checker_fn is not None:
            # Keep query accounting comparable with point-sampled path checks.
            self._collision_queries += int(max(n, 2))
            try:
                return bool(self.edge_checker_fn(np.asarray(p1, dtype=float), np.asarray(p2, dtype=float), n))
            except Exception:
                return False
        for a in np.linspace(0.0, 1.0, n):
            p = p1 + a * (p2 - p1)
            if not self._is_state_valid(p):
                return False
        return True

    def _backtrack_path(
        self,
        states: np.ndarray,
        parents: np.ndarray,
        leaf_idx: int,
    ) -> np.ndarray:
        idx = int(leaf_idx)
        chain = []
        while idx >= 0:
            chain.append(states[idx].copy())
            idx = int(parents[idx]) if idx < parents.shape[0] else -1
        chain.reverse()
        return np.vstack(chain)

    def _fallback_path(
        self,
        states: np.ndarray,
        parents: np.ndarray,
        start: np.ndarray,
        goal: np.ndarray,
    ) -> np.ndarray:
        if states.shape[0] == 0:
            warnings.warn("No tree nodes available, fallback to [start, goal].", RuntimeWarning, stacklevel=2)
            return np.vstack([start, goal])

        nearest_idx = int(np.argmin(np.linalg.norm(states - goal[None, :], axis=1)))
        best_prefix = self._backtrack_path(states, parents, nearest_idx)

        if self._is_edge_valid(best_prefix[-1], goal):
            if np.linalg.norm(best_prefix[-1] - goal) > self._eps:
                best_prefix = np.vstack([best_prefix, goal])
            return best_prefix

        warnings.warn(
            "No feasible goal-reaching path found, fallback to [start, goal].",
            RuntimeWarning,
            stacklevel=2,
        )
        return np.vstack([start, goal])

    def _allocate_via_times(
        self,
        path_points: np.ndarray,  # (M, D)
        t_start: float,
        t_end: float,
    ) -> np.ndarray:
        m = path_points.shape[0]
        if m <= 0:
            return np.array([t_start], dtype=float)
        if m == 1:
            return np.array([t_start], dtype=float)

        seg = np.diff(path_points, axis=0)
        seg_len = np.linalg.norm(seg, axis=1)
        s = np.concatenate(([0.0], np.cumsum(seg_len)))
        s_end = float(s[-1])

        if s_end <= self._eps:
            t = np.full(m, float(t_start), dtype=float)
        else:
            t = float(t_start) + (s / s_end) * (float(t_end) - float(t_start))

        # Strict monotonicity guard for downstream FMP stability.
        for k in range(1, m):
            t[k] = max(float(t[k]), float(t[k - 1]) + self._time_eps)

        return t


__all__ = ["IntentBiasedRRT"]
