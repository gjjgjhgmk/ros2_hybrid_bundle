"""MATLAB-compatible danger segmentation, local RRT*, and FMP orchestration."""

from __future__ import annotations

from dataclasses import asdict
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]
for _candidate in (_REPO_ROOT, _REPO_ROOT / "intent_hybrid_planner"):
    if _candidate.exists() and str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))

from intent_hybrid_planner import fmp_core

from .matlab_rrt_star_2d import MATLAB_SCALE, MatlabRRTStarConfig, plan_intent_biased_informed_rrt_star


MATLAB_DEFAULTS: Dict[str, Any] = {
    "demo_len": 150,
    "demo_dt": 0.1,
    "alpha": 0.1,
    "n_clusters": 20,
    "online_dt": 0.05,
    "interp_dist": 0.5,
    "via_trim": 0.05,
    "transition_ratio": 0.10,
    "transition_gamma": 1.0,
    "safe_margin": 0.2,
    "danger_segment_gap": 10,
    "segment_padding": 4,
    "corner_smoothing_enabled": True,
    "corner_angle_threshold_rad": 0.01,
    "corner_window_mode": "time",
    "corner_window_value": 0.1,
}


def matlab_parameters(overrides: Dict[str, Any] | None = None) -> Dict[str, Any]:
    result = dict(MATLAB_DEFAULTS)
    result.update(overrides or {})
    return result


def danger_indices_matlab(
    nominal_uv: np.ndarray,
    obstacles: Iterable[Dict[str, Any]],
    safe_margin: float = 0.2,
) -> List[int]:
    """Return zero-based equivalents of MATLAB's danger indices."""
    points = np.asarray(nominal_uv, dtype=float) * MATLAB_SCALE
    danger = set()
    for obstacle in obstacles:
        center = np.asarray(obstacle["center"], dtype=float) * MATLAB_SCALE
        radius = float(obstacle["radius"]) * MATLAB_SCALE + float(safe_margin)
        for index, point in enumerate(points):
            if np.linalg.norm(point - center) < radius:
                danger.add(index)
    return sorted(danger)


def split_danger_segments(indices: Sequence[int], gap: int = 10) -> List[List[int]]:
    if not indices:
        return []
    segments = [[int(indices[0])]]
    for index in indices[1:]:
        if int(index) - segments[-1][-1] > int(gap):
            segments.append([int(index)])
        else:
            segments[-1].append(int(index))
    return segments


def _densify_matlab(path_uv: np.ndarray, interp_dist: float) -> np.ndarray:
    path = np.asarray(path_uv, dtype=float) * MATLAB_SCALE
    parts = []
    for index in range(path.shape[0] - 1):
        first, second = path[index], path[index + 1]
        count = max(2, int(np.ceil(np.linalg.norm(second - first) / float(interp_dist))))
        segment = np.linspace(first, second, count)
        parts.append(segment[:-1] if index < path.shape[0] - 2 else segment)
    return np.vstack(parts) / MATLAB_SCALE


def _train_matlab_fmp_dimension(
    nominal_values: np.ndarray, time_axis: np.ndarray, params: Dict[str, Any]
):
    """Train one coordinate exactly as MATLAB's separate x/y FMP models."""
    trajectory = np.asarray(nominal_values, dtype=float).reshape(1, -1)
    demo_dt = float(params["demo_dt"])
    demo_dura = 1.0 / demo_dt
    scaled_time = demo_dura * time_axis
    phase = np.exp(float(params["alpha"]) * scaled_time / scaled_time[-1])
    base = np.vstack((scaled_time, phase, trajectory))
    demo_scale = 1.0 / MATLAB_SCALE
    offset = np.asarray([[0.0], [0.0], [demo_scale]])
    training = np.hstack((base, base + offset, base - offset))
    cluster_count = int(params["n_clusters"])
    centers, inv_covs, membership = fmp_core.gk_clustering(
        training,
        cluster_count,
        max_iter_fcm=30,
        max_iter_gk=30,
        init_length=trajectory.shape[1],
    )
    regression = fmp_core._train_local_regression(  # pylint: disable=protected-access
        training,
        membership,
        cluster_count,
        np.asarray([0, 1], dtype=int),
        np.asarray([2], dtype=int),
    )
    return {
        "C": centers,
        "inv_covs": inv_covs,
        "p1_u": regression,
        "alpha": float(params["alpha"]),
        "N_C": cluster_count,
        "location_x": np.asarray([0, 1], dtype=int),
        "location_y": np.asarray([2], dtype=int),
        "m": 2.0,
        "demo_dura": demo_dura,
    }


def resolve_corner_window_samples(
    mode: str,
    value: float,
    time_axis: np.ndarray,
    path_point_count: int,
) -> Tuple[int, float]:
    """Resolve the ambiguous MATLAB local_window into an integer radius."""
    normalized_mode = str(mode).strip().lower()
    numeric_value = float(value)
    if numeric_value < 0.0:
        raise ValueError("corner_window_value must be non-negative")
    times = np.asarray(time_axis, dtype=float).reshape(-1)
    sample_interval = float(np.median(np.diff(times))) if times.size >= 2 else 0.0
    if normalized_mode == "time":
        if sample_interval <= 0.0:
            raise ValueError("time window mode requires a strictly increasing time axis")
        samples = int(round(numeric_value / sample_interval))
    elif normalized_mode == "samples":
        samples = int(round(numeric_value))
    elif normalized_mode == "ratio":
        if numeric_value > 1.0:
            raise ValueError("ratio corner window must be in [0, 1]")
        samples = int(round(numeric_value * max(int(path_point_count) - 1, 1)))
    else:
        raise ValueError("corner_window_mode must be time, samples, or ratio")
    return max(samples, 0), sample_interval


def _gaussian_smooth_region(values: np.ndarray) -> np.ndarray:
    length = int(values.shape[0])
    if length <= 1:
        return values.copy()
    kernel_length = length if length % 2 == 1 else max(length - 1, 1)
    radius = kernel_length // 2
    axis = np.arange(-radius, radius + 1, dtype=float)
    sigma = max(kernel_length / 6.0, 1.0)
    kernel = np.exp(-0.5 * (axis / sigma) ** 2)
    kernel /= np.sum(kernel)
    smoothed = np.empty_like(values)
    for dimension in range(values.shape[1]):
        padded = np.pad(values[:, dimension], (radius, radius), mode="edge")
        smoothed[:, dimension] = np.convolve(padded, kernel, mode="valid")
    return smoothed


def apply_local_corner_smoothing(
    path_uv: np.ndarray,
    time_axis: np.ndarray,
    *,
    enabled: bool = True,
    angle_threshold_rad: float = 0.01,
    window_mode: str = "time",
    window_value: float = 0.1,
) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Apply MATLAB's local heading-change smoothing with explicit window units."""
    path = np.asarray(path_uv, dtype=float)
    if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] < 2:
        raise ValueError("path_uv must be shaped (N, 2), N >= 2")
    window_samples, sample_interval = resolve_corner_window_samples(
        window_mode, window_value, time_axis, path.shape[0]
    )
    metadata: Dict[str, Any] = {
        "enabled": bool(enabled),
        "window_mode": str(window_mode).lower(),
        "configured_window_value": float(window_value),
        "sample_interval_sec": float(sample_interval),
        "resolved_window_samples": int(window_samples),
        "angle_threshold_rad": float(angle_threshold_rad),
        "sharp_point_count": 0,
        "region_count": 0,
        "applied": False,
        "max_displacement_uv": 0.0,
        "mean_displacement_uv": 0.0,
        "interpretation": (
            f"{float(window_value):g} seconds converted using trajectory sample interval"
            if str(window_mode).lower() == "time"
            else f"{float(window_value):g} interpreted as {str(window_mode).lower()} window"
        ),
    }
    if not enabled or window_samples <= 0 or path.shape[0] < 3:
        return path.copy(), metadata

    dx = np.gradient(path[:, 0])
    dy = np.gradient(path[:, 1])
    headings = np.arctan2(dy, dx)
    heading_changes = np.abs(np.diff(headings))
    heading_changes = np.minimum(heading_changes, 2.0 * np.pi - heading_changes)
    heading_changes = np.concatenate(([0.0], heading_changes))
    sharp_indices = np.flatnonzero(heading_changes > float(angle_threshold_rad))
    metadata["sharp_point_count"] = int(sharp_indices.size)
    if sharp_indices.size == 0:
        return path.copy(), metadata

    mask = np.zeros(path.shape[0], dtype=bool)
    for index in sharp_indices:
        first = max(1, int(index) - window_samples)
        last = min(path.shape[0] - 2, int(index) + window_samples)
        if first <= last:
            mask[first : last + 1] = True

    transitions = np.diff(np.concatenate(([False], mask, [False])).astype(np.int8))
    starts = np.flatnonzero(transitions == 1)
    ends = np.flatnonzero(transitions == -1)
    smoothed = path.copy()
    for first, end_exclusive in zip(starts, ends):
        smoothed[first:end_exclusive] = _gaussian_smooth_region(path[first:end_exclusive])
    smoothed[0] = path[0]
    smoothed[-1] = path[-1]
    metadata["region_count"] = int(starts.size)
    metadata["applied"] = bool(starts.size)
    displacement = np.linalg.norm(smoothed - path, axis=1)
    metadata["max_displacement_uv"] = float(np.max(displacement))
    metadata["mean_displacement_uv"] = float(np.mean(displacement))
    return smoothed, metadata


def run_matlab_hybrid(
    nominal_uv: Iterable[Sequence[float]],
    obstacles: Iterable[Dict[str, Any]],
    *,
    parameter_overrides: Dict[str, Any] | None = None,
    rrt_overrides: Dict[str, Any] | None = None,
    environment_seed: int = 2025,
) -> Dict[str, Any]:
    params = matlab_parameters(parameter_overrides)
    nominal = np.asarray(nominal_uv, dtype=float)
    obstacle_list = list(obstacles)
    if nominal.ndim != 2 or nominal.shape[1] != 2 or nominal.shape[0] < 2:
        raise ValueError("nominal_uv must be shaped (N, 2)")

    # MATLAB always uses demoLen=150. Resampling at this boundary preserves the
    # same danger indices, time axis, clustering density, and FMP output count.
    if nominal.shape[0] != int(params["demo_len"]):
        from .path_resample import resample_path_by_arclength

        nominal = resample_path_by_arclength(nominal, int(params["demo_len"]))

    danger = danger_indices_matlab(nominal, obstacle_list, float(params["safe_margin"]))
    segments = split_danger_segments(danger, int(params["danger_segment_gap"]))
    all_via_points: List[np.ndarray] = []
    all_via_times: List[np.ndarray] = []
    segment_results = []
    plotted_paths = []
    demo_dt = float(params["demo_dt"])
    rrt_cfg = MatlabRRTStarConfig.from_mapping(rrt_overrides)

    for segment_number, segment in enumerate(segments, start=1):
        start_index = max(0, segment[0] - int(params["segment_padding"]))
        goal_index = min(nominal.shape[0] - 1, segment[-1] + int(params["segment_padding"]))
        local_intent = nominal[start_index : goal_index + 1]
        seed = int(environment_seed) + segment_number * 1000 + 2
        result = plan_intent_biased_informed_rrt_star(
            nominal[start_index],
            nominal[goal_index],
            local_intent,
            obstacle_list,
            config=rrt_cfg,
            rng_seed=seed,
        )
        result.update(
            {
                "segment_number": segment_number,
                "start_index": start_index,
                "goal_index": goal_index,
            }
        )
        segment_results.append(result)
        if not result["ok"]:
            continue

        selected = np.asarray(result["path_refine"] or result["path"], dtype=float)
        plotted_paths.append(selected)
        dense = _densify_matlab(selected, float(params["interp_dist"]))
        cumulative = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1))))
        t_start = (start_index + 1) * demo_dt
        t_goal = (goal_index + 1) * demo_dt
        local_times = t_start + cumulative / max(cumulative[-1], np.finfo(float).eps) * (t_goal - t_start)
        mask = (local_times > t_start + float(params["via_trim"])) & (
            local_times < t_goal - float(params["via_trim"])
        )
        all_via_points.append(dense[mask])
        all_via_times.append(local_times[mask])

    failed_segments = [result for result in segment_results if not result["ok"]]
    if failed_segments:
        result = _result(nominal, danger, segments, segment_results, plotted_paths, ok=False)
        result.update(
            {
                "matlab_parameters": params,
                "matlab_rrt_parameters": asdict(rrt_cfg),
                "corner_smoothing_metadata": {
                    "enabled": bool(params["corner_smoothing_enabled"]),
                    "applied": False,
                    "failure_reason": "rrt_failed_before_smoothing",
                },
            }
        )
        return result

    demo_time_axis = np.arange(1, nominal.shape[0] + 1, dtype=float) * demo_dt
    if not all_via_points:
        modulated = nominal.copy()
        via_points = np.empty((0, 2), dtype=float)
        via_times = np.empty(0, dtype=float)
        time_axis = demo_time_axis
    else:
        via_points = np.vstack(all_via_points)
        via_times = np.concatenate(all_via_times)
        unique_times, unique_indices = np.unique(via_times, return_index=True)
        via_times = unique_times
        via_points = via_points[unique_indices]
        model_x = _train_matlab_fmp_dimension(nominal[:, 0], demo_time_axis, params)
        model_y = _train_matlab_fmp_dimension(nominal[:, 1], demo_time_axis, params)
        online_dt = float(params["online_dt"])
        time_axis = np.arange(
            online_dt,
            nominal.shape[0] * demo_dt + online_dt * 0.5,
            online_dt,
            dtype=float,
        )
        progress_demo = np.linspace(0.0, 1.0, nominal.shape[0])
        progress_online = np.linspace(0.0, 1.0, time_axis.size)
        nominal_online = np.column_stack(
            (
                np.interp(progress_online, progress_demo, nominal[:, 0]),
                np.interp(progress_online, progress_demo, nominal[:, 1]),
            )
        )
        common = {
            "via_times": via_times,
            "transition_ratio": float(params["transition_ratio"]),
            "transition_gamma": float(params["transition_gamma"]),
        }
        modulated_x = fmp_core.modulate_trajectory(
            model_x,
            nominal_online[:, 0].reshape(1, -1),
            time_axis,
            via_points[:, 0].reshape(1, -1),
            **common,
        ).reshape(-1)
        modulated_y = fmp_core.modulate_trajectory(
            model_y,
            nominal_online[:, 1].reshape(1, -1),
            time_axis,
            via_points[:, 1].reshape(1, -1),
            **common,
        ).reshape(-1)
        modulated = np.column_stack((modulated_x, modulated_y))

    modulated, corner_metadata = apply_local_corner_smoothing(
        modulated,
        time_axis,
        enabled=bool(params["corner_smoothing_enabled"]),
        angle_threshold_rad=float(params["corner_angle_threshold_rad"]),
        window_mode=str(params["corner_window_mode"]),
        window_value=float(params["corner_window_value"]),
    )

    result = _result(nominal, danger, segments, segment_results, plotted_paths, ok=True)
    result.update(
        {
            "modulated_path": modulated,
            "via_points": via_points,
            "via_times": via_times,
            "time_axis": time_axis,
            "matlab_parameters": params,
            "matlab_rrt_parameters": asdict(rrt_cfg),
            "corner_smoothing_metadata": corner_metadata,
        }
    )
    return result


def _result(nominal, danger, segments, segment_results, plotted_paths, *, ok: bool) -> Dict[str, Any]:
    return {
        "ok": bool(ok),
        "nominal_path": nominal,
        "modulated_path": nominal.copy(),
        "danger_indices": list(danger),
        "danger_count": len(danger),
        "segments": [list(segment) for segment in segments],
        "segment_results": segment_results,
        "rrt_paths": plotted_paths,
        "via_points": np.empty((0, 2), dtype=float),
        "via_times": np.empty(0, dtype=float),
        "time_axis": np.arange(1, nominal.shape[0] + 1, dtype=float) * 0.1,
        "rrt_stop_reason": "not_required" if not segments else ("success" if ok else "failed"),
        "rrt_iter_used": int(sum(item.get("iter_used", 0) for item in segment_results)),
        "rrt_collision_queries": int(sum(item.get("collision_queries", 0) for item in segment_results)),
        "rrt_elapsed_ms": float(sum(item.get("elapsed_ms", 0.0) for item in segment_results)),
    }
