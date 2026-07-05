"""Path simplification and arc-length resampling utilities."""

from typing import Iterable, Sequence

import numpy as np


def _as_path(path: Iterable[Sequence[float]]) -> np.ndarray:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[1] < 2 or arr.shape[0] == 0:
        raise ValueError("path must be a non-empty array shaped (N, D), D >= 2")
    if not np.all(np.isfinite(arr)):
        raise ValueError("path contains non-finite values")
    return arr


def _remove_consecutive_duplicates(path: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    if path.shape[0] <= 1:
        return path.copy()
    keep = np.ones(path.shape[0], dtype=bool)
    keep[1:] = np.linalg.norm(np.diff(path, axis=0), axis=1) > eps
    return path[keep]


def resample_path_by_arclength(
    path: Iterable[Sequence[float]], num_points: int
) -> np.ndarray:
    arr = _remove_consecutive_duplicates(_as_path(path))
    num_points = int(num_points)
    if num_points < 2:
        raise ValueError("num_points must be at least 2")
    if arr.shape[0] == 1:
        return np.repeat(arr, num_points, axis=0)

    segment_lengths = np.linalg.norm(np.diff(arr, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])
    if total <= 1e-12:
        return np.repeat(arr[:1], num_points, axis=0)

    targets = np.linspace(0.0, total, num_points)
    result = np.column_stack(
        [np.interp(targets, cumulative, arr[:, dim]) for dim in range(arr.shape[1])]
    )
    result[0] = arr[0]
    result[-1] = arr[-1]
    return result


def simplify_path_optional(
    path: Iterable[Sequence[float]], min_dist: float = 0.0
) -> np.ndarray:
    arr = _remove_consecutive_duplicates(_as_path(path))
    min_dist = float(min_dist)
    if min_dist <= 0.0 or arr.shape[0] <= 2:
        return arr.copy()

    kept = [arr[0]]
    for point in arr[1:-1]:
        if float(np.linalg.norm(point - kept[-1])) >= min_dist:
            kept.append(point)
    kept.append(arr[-1])
    return np.asarray(kept, dtype=float)

