#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal MATLAB-compatible helpers for segmented hybrid replanning flow.
"""

from __future__ import annotations

from typing import Callable, Dict, Iterable, List, Sequence, Tuple

import numpy as np


def extract_danger_segments(
    danger_indices: Sequence[int],
    n_points: int,
    *,
    gap: int = 10,
    pad: int = 4,
) -> List[Tuple[int, int]]:
    """Convert sparse danger indices into merged local planning segments."""
    if n_points <= 0:
        return []
    if not danger_indices:
        return []

    idx = sorted({int(i) for i in danger_indices if 0 <= int(i) < n_points})
    if not idx:
        return []

    raw_segments: List[List[int]] = []
    current = [idx[0]]
    for value in idx[1:]:
        if value - current[-1] > int(gap):
            raw_segments.append(current)
            current = [value]
        else:
            current.append(value)
    raw_segments.append(current)

    padded: List[Tuple[int, int]] = []
    for seg in raw_segments:
        start = max(0, seg[0] - int(pad))
        end = min(n_points - 1, seg[-1] + int(pad))
        padded.append((start, end))

    merged: List[Tuple[int, int]] = []
    for start, end in padded:
        if not merged or start > merged[-1][1] + 1:
            merged.append((start, end))
            continue
        merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    return merged


def densify_path_to_vias(
    path_points: np.ndarray,
    t_start: float,
    t_end: float,
    *,
    interp_dist: float = 0.5,
    via_trim_sec: float = 0.05,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Densify piecewise path by arclength and return trimmed via points and times.
    """
    arr = np.asarray(path_points, dtype=float)
    if arr.ndim != 2 or arr.size == 0:
        return np.empty((0, 0), dtype=float), np.empty((0,), dtype=float)

    if arr.shape[1] < 2 and arr.shape[0] >= 2:
        arr = arr.T

    dim, count = arr.shape
    if count < 2:
        return np.empty((dim, 0), dtype=float), np.empty((0,), dtype=float)

    dist_step = max(float(interp_dist), 1e-6)
    dense_chunks: List[np.ndarray] = []
    for i in range(count - 1):
        p1 = arr[:, i]
        p2 = arr[:, i + 1]
        seg_len = float(np.linalg.norm(p2 - p1))
        num_interp = max(2, int(np.ceil(seg_len / dist_step)))
        alpha = np.linspace(0.0, 1.0, num_interp, dtype=float)
        chunk = p1[:, None] + (p2 - p1)[:, None] * alpha[None, :]
        if i < count - 2:
            chunk = chunk[:, :-1]
        dense_chunks.append(chunk)
    dense = np.hstack(dense_chunks) if dense_chunks else arr

    diff = np.diff(dense, axis=1)
    seg_len = np.linalg.norm(diff, axis=0)
    arc = np.concatenate(([0.0], np.cumsum(seg_len)))
    total = float(arc[-1])
    if total <= 1e-12:
        t_local = np.linspace(float(t_start), float(t_end), dense.shape[1], dtype=float)
    else:
        ratio = arc / total
        t_local = float(t_start) + ratio * (float(t_end) - float(t_start))

    trim = max(float(via_trim_sec), 0.0)
    mask = (t_local > float(t_start) + trim) & (t_local < float(t_end) - trim)
    if not np.any(mask):
        return np.empty((dim, 0), dtype=float), np.empty((0,), dtype=float)
    return dense[:, mask], t_local[mask]


def choose_refine_budget_pareto(
    budgets: Sequence[int],
    gains: Sequence[float],
    deltas: Sequence[float],
) -> int:
    """Pick budget from Pareto set by gain-per-time preference."""
    b = np.asarray(budgets, dtype=int).reshape(-1)
    g = np.asarray(gains, dtype=float).reshape(-1)
    t = np.asarray(deltas, dtype=float).reshape(-1)
    if b.size == 0:
        return 100

    valid = np.isfinite(g) & np.isfinite(t)
    if not np.any(valid):
        return int(b[min(np.argmin(np.abs(b - 100)), b.size - 1)])

    b = b[valid]
    g = g[valid]
    t = t[valid]
    n = b.size

    pareto = np.ones(n, dtype=bool)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            dominates = (g[j] >= g[i]) and (t[j] <= t[i]) and ((g[j] > g[i]) or (t[j] < t[i]))
            if dominates:
                pareto[i] = False
                break

    pb = b[pareto]
    pg = g[pareto]
    pt = t[pareto]
    if pb.size == 0:
        return int(b[np.argmax(g)])

    score = pg / np.maximum(pt, 1e-9)
    best = int(np.argmax(score))
    return int(pb[best])


def auto_select_refine_budget(
    budget_candidates: Sequence[int],
    evaluate_plan_fn: Callable[[int], Dict[str, np.ndarray]],
) -> Tuple[int, List[Dict[str, float]]]:
    """
    Sweep candidate budgets and choose one by Pareto rule.
    evaluate_plan_fn must return path_first/path_refine/t_first/t_refine keys.
    """
    budgets = [int(x) for x in budget_candidates]
    gains: List[float] = []
    deltas: List[float] = []
    logs: List[Dict[str, float]] = []

    for budget in budgets:
        plan = evaluate_plan_fn(budget)
        path_first = np.asarray(plan.get("path_first", np.empty((0, 0))), dtype=float)
        path_refine = np.asarray(plan.get("path_refine", np.empty((0, 0))), dtype=float)
        t_first = float(plan.get("t_first", np.nan))
        t_refine = float(plan.get("t_refine", np.nan))

        if path_first.size == 0 or path_refine.size == 0:
            gain = np.nan
            delta_t = np.nan
        else:
            j_first = _path_objective(path_first)
            j_refine = _path_objective(path_refine)
            gain = float((j_first - j_refine) / max(j_first, 1e-12))
            delta_t = float(max(0.0, t_refine - t_first))

        gains.append(gain)
        deltas.append(delta_t)
        logs.append({"budget": float(budget), "gain": float(gain), "delta_t": float(delta_t)})

    best = choose_refine_budget_pareto(budgets, gains, deltas)
    return int(best), logs


def _path_objective(path: np.ndarray) -> float:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2:
        return float("inf")
    if arr.shape[1] < 2:
        return 0.0
    diff = np.diff(arr, axis=1)
    seg = np.linalg.norm(diff, axis=0)
    return float(np.sum(seg))


__all__ = [
    "extract_danger_segments",
    "densify_path_to_vias",
    "auto_select_refine_budget",
    "choose_refine_budget_pareto",
]

