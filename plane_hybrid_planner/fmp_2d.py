"""A small, replaceable 2D FMP-style smooth modulation layer.

Phase 1 aligns a collision-free coarse path with the nominal path by arc
length, smooths the resulting displacement field, and applies it to the
nominal trajectory.  The public interface is intentionally independent of
the approximation so the full learned MATLAB FMP can replace it later.
"""

from typing import Iterable, Optional, Sequence

import numpy as np

from .path_resample import resample_path_by_arclength


def _validate_path(path: Iterable[Sequence[float]], name: str) -> np.ndarray:
    arr = np.asarray(path, dtype=float)
    if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] < 2:
        raise ValueError(f"{name} must be shaped (N, 2), N >= 2")
    if not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} contains non-finite values")
    return arr


def _smooth_signal(signal: np.ndarray, window: int, iterations: int) -> np.ndarray:
    window = max(int(window), 1)
    if window % 2 == 0:
        window += 1
    if window <= 1 or iterations <= 0:
        return signal.copy()

    radius = window // 2
    axis = np.arange(-radius, radius + 1, dtype=float)
    sigma = max(window / 4.0, 1.0)
    kernel = np.exp(-0.5 * (axis / sigma) ** 2)
    kernel /= np.sum(kernel)

    result = signal.copy()
    for _ in range(int(iterations)):
        padded = np.pad(result, ((radius, radius), (0, 0)), mode="edge")
        result = np.column_stack(
            [np.convolve(padded[:, dim], kernel, mode="valid") for dim in range(result.shape[1])]
        )
    return result


def modulate_path(
    nominal_uv_path: Iterable[Sequence[float]],
    via_uv_path: Optional[Iterable[Sequence[float]]] = None,
    time_axis: Optional[Sequence[float]] = None,
    *,
    smoothing: bool = True,
    smoothing_window: int = 11,
    smoothing_iterations: int = 2,
    blend_strength: float = 1.0,
) -> np.ndarray:
    nominal = _validate_path(nominal_uv_path, "nominal_uv_path")
    if time_axis is not None:
        times = np.asarray(time_axis, dtype=float).reshape(-1)
        if times.size != nominal.shape[0] or np.any(np.diff(times) <= 0.0):
            raise ValueError("time_axis must be strictly increasing and match nominal path length")

    if via_uv_path is None:
        return nominal.copy()
    via = np.asarray(via_uv_path, dtype=float)
    if via.size == 0:
        return nominal.copy()
    via = _validate_path(via, "via_uv_path")

    aligned_via = resample_path_by_arclength(via, nominal.shape[0])
    displacement = aligned_via - nominal
    displacement[0] = 0.0
    displacement[-1] = 0.0
    if smoothing:
        displacement = _smooth_signal(
            displacement,
            window=int(smoothing_window),
            iterations=int(smoothing_iterations),
        )
        displacement[0] = 0.0
        displacement[-1] = 0.0

    strength = float(np.clip(blend_strength, 0.0, 1.5))
    modulated = nominal + strength * displacement
    modulated[0] = nominal[0]
    modulated[-1] = nominal[-1]
    return np.clip(modulated, 0.0, 1.0)

