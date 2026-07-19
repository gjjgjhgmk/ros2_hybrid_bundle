"""Compare the current MATLAB-compatible 2D hybrid pipeline with OMPL planners."""

from __future__ import annotations

import argparse
import csv
import json
import logging
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence

import numpy as np
import yaml

from .evaluate_2d import jerk_integral, path_length
from .matlab_hybrid_2d import run_matlab_hybrid
from .obstacle_coordinates import normalize_obstacles_to_uv, obstacle_input_config
from .plane_mapping import PlaneMapper
from .planner_2d import (
    generate_nominal_path,
    path_collision_details,
    path_min_clearance,
    point_clearance,
)


LOGGER = logging.getLogger("plane_hybrid_planner")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def _write_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False, allow_nan=False)


def _write_csv(path: Path, rows: Iterable[Dict[str, Any]]) -> None:
    rows = list(rows)
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _clearance_series(path: np.ndarray, obstacles: List[Dict[str, Any]]) -> np.ndarray:
    if path.size == 0:
        return np.empty(0, dtype=float)
    return np.asarray([point_clearance(point, obstacles) for point in path], dtype=float)


def _plot_compare(
    out_dir: Path,
    nominal: np.ndarray,
    obstacles: List[Dict[str, Any]],
    records: List[Dict[str, Any]],
    safety_margin: float,
) -> None:
    import matplotlib  # pylint: disable=import-outside-toplevel

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    figure, axis = plt.subplots(figsize=(8.8, 6.8))
    axis.plot(nominal[:, 0], nominal[:, 1], "--", color="#2563eb", label="nominal", linewidth=2.0)
    for record in records:
        path = np.asarray(record.get("path", []), dtype=float)
        if path.ndim != 2 or path.shape[0] < 2:
            continue
        label = f"{record['planner']}:{record['mode']}"
        axis.plot(path[:, 0], path[:, 1], linewidth=2.0, label=label)
    for index, obstacle in enumerate(obstacles):
        center = obstacle["center"]
        circle = plt.Circle(center, obstacle["radius"], color="#dc2626", alpha=0.24)
        margin_circle = plt.Circle(
            center,
            obstacle["radius"] + safety_margin,
            fill=False,
            linestyle=":",
            color="#991b1b",
            linewidth=1.2,
        )
        axis.add_patch(circle)
        axis.add_patch(margin_circle)
        axis.text(center[0], center[1], f"O{index + 1}", ha="center", va="center", fontsize=8)
    axis.scatter(*nominal[0], marker="o", color="#111827", s=55, label="start", zorder=5)
    axis.scatter(*nominal[-1], marker="*", color="#111827", s=95, label="goal", zorder=5)
    axis.set(xlabel="u", ylabel="v", title="Planner comparison in normalized 2D space")
    axis.set_xlim(0.0, 1.0)
    axis.set_ylim(0.0, 1.0)
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(out_dir / "comparison_paths.png", dpi=180)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(9.0, 5.0))
    axis.axhline(0.0, color="#dc2626", linewidth=1.1, label="collision boundary")
    axis.axhline(safety_margin, color="#f59e0b", linestyle=":", label="safety margin")
    for record in records:
        path = np.asarray(record.get("path", []), dtype=float)
        if path.ndim != 2 or path.shape[0] < 2:
            continue
        series = _clearance_series(path, obstacles)
        if series.size == 0:
            continue
        progress = np.linspace(0.0, 1.0, series.size)
        axis.plot(progress, series, label=f"{record['planner']}:{record['mode']}")
    axis.set(xlabel="normalized progress", ylabel="point clearance in UV", title="Clearance comparison")
    axis.grid(alpha=0.25)
    axis.legend(loc="best", fontsize=8)
    figure.tight_layout()
    figure.savefig(out_dir / "comparison_clearance.png", dpi=180)
    plt.close(figure)


def _scenario_from_file(document: Dict[str, Any], name: str) -> Dict[str, Any]:
    scenarios = document.get("scenarios", document)
    if not isinstance(scenarios, dict) or name not in scenarios:
        available = ", ".join(sorted(scenarios.keys())) if isinstance(scenarios, dict) else ""
        raise KeyError(f"scenario '{name}' not found; available: {available}")
    scenario = scenarios[name]
    if not isinstance(scenario, dict):
        raise ValueError(f"scenario '{name}' must be a mapping")
    return dict(scenario)


def _planner_modes(value: str) -> List[str]:
    normalized = str(value).strip().lower()
    if normalized == "raw":
        return ["raw"]
    if normalized == "simplify":
        return ["simplify"]
    if normalized == "both":
        return ["raw", "simplify"]
    raise ValueError("--ompl-mode must be raw, simplify, or both")


def _run_ompl_backend(
    request: Dict[str, Any],
    *,
    out_dir: Path,
    request_name: str,
    executable: Sequence[str] | None,
) -> Dict[str, Any]:
    request_path = out_dir / f"{request_name}.json"
    _write_json(request_path, request)
    if executable:
        cmd = [*executable, str(request_path)]
    else:
        cmd = [
            "ros2",
            "run",
            "intent_hybrid_runtime_cpp",
            "ompl_2d_benchmark",
            "--request-file",
            str(request_path),
        ]
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    (out_dir / f"{request_name}.stdout.log").write_text(proc.stdout, encoding="utf-8")
    (out_dir / f"{request_name}.stderr.log").write_text(proc.stderr, encoding="utf-8")
    if proc.returncode != 0:
        return {
            "ok": False,
            "planner": request.get("planner_type", ""),
            "mode": "simplify" if request.get("simplify_enable", False) else "raw",
            "error_reason": f"ompl_backend_returncode_{proc.returncode}",
            "error_message": proc.stderr.strip() or proc.stdout.strip(),
            "path": [],
            "path_points": 0,
            "solve_time_ms": 0.0,
            "simplify_time_ms": 0.0,
            "total_time_ms": 0.0,
            "collision_queries": 0,
            "planner_vertices": 0,
            "raw_state_count": 0,
            "raw_path_length": 0.0,
            "path_length": 0.0,
            "simplify_changed": False,
            "request": request,
        }
    try:
        stdout = proc.stdout.strip()
        json_start = stdout.find("{")
        if json_start > 0:
            stdout = stdout[json_start:]
        result = json.loads(stdout)
    except json.JSONDecodeError as exc:
        return {
            "ok": False,
            "planner": request.get("planner_type", ""),
            "mode": "simplify" if request.get("simplify_enable", False) else "raw",
            "error_reason": "ompl_backend_invalid_json",
            "error_message": str(exc),
            "path": [],
            "path_points": 0,
            "solve_time_ms": 0.0,
            "simplify_time_ms": 0.0,
            "total_time_ms": 0.0,
            "collision_queries": 0,
            "planner_vertices": 0,
            "raw_state_count": 0,
            "raw_path_length": 0.0,
            "path_length": 0.0,
            "simplify_changed": False,
            "request": request,
        }
    result["planner"] = request.get("planner_type", "")
    result["mode"] = "simplify" if request.get("simplify_enable", False) else "raw"
    return result


def _metric_row(
    *,
    planner: str,
    mode: str,
    path: np.ndarray,
    obstacles: List[Dict[str, Any]],
    safety_margin: float,
    elapsed_ms: float,
    planner_result: Dict[str, Any],
    success: bool,
    kind: str,
    failure_reason: str = "",
) -> Dict[str, Any]:
    collision = path_collision_details(path, obstacles, margin=0.0) if path.size else {
        "collision_count": 0,
        "invalid_point_count": 0,
        "invalid_edge_count": 0,
    }
    clearance = path_min_clearance(path, obstacles) if path.size else float("nan")
    safe = bool(np.isfinite(clearance) and clearance >= safety_margin)
    return {
        "planner": planner,
        "mode": mode,
        "kind": kind,
        "success": bool(success),
        "failure_reason": failure_reason,
        "planning_time_ms": float(elapsed_ms),
        "path_points": int(path.shape[0]) if path.ndim == 2 else 0,
        "path_length": float(path_length(path)) if path.size else 0.0,
        "jerk_integral": float(jerk_integral(path)) if path.size and path.shape[0] >= 4 else 0.0,
        "collision_count": int(collision["collision_count"]),
        "min_clearance": None if not np.isfinite(clearance) else float(clearance),
        "safety_margin": float(safety_margin),
        "clearance_ok": bool(safe),
        "solver_stop_reason": str(planner_result.get("stop_reason", "")),
        "solver_error_message": str(planner_result.get("error_message", "")),
        "solver_collision_queries": int(planner_result.get("collision_queries", 0)),
        "solver_planner_vertices": int(planner_result.get("planner_vertices", 0)),
        "solver_raw_state_count": int(planner_result.get("raw_state_count", 0)),
        "solver_raw_path_length": float(planner_result.get("raw_path_length", 0.0)),
        "solver_simplify_changed": bool(planner_result.get("simplify_changed", False)),
    }


def run(args: argparse.Namespace) -> Dict[str, Any]:
    config_path = Path(args.config).expanduser().resolve()
    scenario_path = Path(args.scenario).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    config = _load_yaml(config_path)
    scenario = _scenario_from_file(_load_yaml(scenario_path), args.scenario_name)
    nominal = generate_nominal_path(scenario["nominal"])
    mapper = PlaneMapper.from_config(config)
    input_config = obstacle_input_config(scenario, config)
    obstacles = normalize_obstacles_to_uv(
        scenario.get("obstacles", []),
        mapper,
        input_mode=str(input_config.get("coordinate_mode", "normalized")),
        input_frame=str(input_config.get("frame_id", mapper.frame_id)),
        radius_scale_mode=str(input_config.get("radius_scale_mode", "min")),
    )
    safety_margin = float(scenario.get("safety_margin", 0.005))

    baseline_started = time.perf_counter()
    hybrid_result = run_matlab_hybrid(
        nominal,
        obstacles,
        parameter_overrides=scenario.get("matlab_fmp", {}),
        rrt_overrides=scenario.get("matlab_rrt", {}),
        environment_seed=int(scenario.get("environment_seed", 2025)),
    )
    baseline_elapsed_ms = (time.perf_counter() - baseline_started) * 1000.0
    hybrid_path = np.asarray(hybrid_result.get("modulated_path", nominal), dtype=float)
    hybrid_row = _metric_row(
        planner="current_hybrid",
        mode="baseline",
        kind="hybrid",
        path=hybrid_path,
        obstacles=obstacles,
        safety_margin=safety_margin,
        elapsed_ms=baseline_elapsed_ms,
        planner_result={
            "stop_reason": hybrid_result.get("rrt_stop_reason", ""),
            "error_message": "MATLAB-compatible hybrid pipeline",
            "collision_queries": hybrid_result.get("rrt_collision_queries", 0),
            "planner_vertices": hybrid_result.get("rrt_iter_used", 0),
            "raw_state_count": hybrid_result.get("nominal_path", nominal).shape[0]
            if isinstance(hybrid_result.get("nominal_path"), np.ndarray)
            else int(np.asarray(hybrid_result.get("nominal_path", nominal)).shape[0]),
            "raw_path_length": path_length(hybrid_path),
            "simplify_changed": bool(hybrid_result.get("corner_smoothing_metadata", {}).get("applied", False)),
        },
        success=bool(hybrid_result.get("ok", False)),
        failure_reason="" if bool(hybrid_result.get("ok", False)) else "current_hybrid_failed",
    )
    plot_records: List[Dict[str, Any]] = [
        {**hybrid_row, "path": hybrid_path.tolist()}
    ]

    ompl_config = config.get("ompl", {})
    request_base = {
        "start": nominal[0].tolist(),
        "goal": nominal[-1].tolist(),
        "obstacles": [{"center": obstacle["center"], "radius": obstacle["radius"]} for obstacle in obstacles],
        "timeout_sec": float(args.timeout_sec or ompl_config.get("timeout_sec", 1.0)),
        "step_size": float(args.step_size or ompl_config.get("step_size", 0.04)),
        "goal_tolerance": float(args.goal_tolerance or ompl_config.get("goal_tolerance", 0.04)),
        "edge_resolution": float(args.edge_resolution or ompl_config.get("edge_resolution", 0.01)),
        "goal_bias": float(args.goal_bias or ompl_config.get("goal_bias", 0.05)),
        "rng_seed": int(args.rng_seed or ompl_config.get("rng_seed", 42)),
        "simplify_timeout_sec": float(args.simplify_timeout_sec or ompl_config.get("simplify_timeout_sec", 0.05)),
        "simplify_at_least_once": bool(
            args.simplify_at_least_once
            if args.simplify_at_least_once is not None
            else ompl_config.get("simplify_at_least_once", True)
        ),
    }

    planners = [planner.strip().lower() for planner in args.planners.split(",") if planner.strip()]
    if not planners:
        planners = ["rrt", "rrt_star", "informed_rrt_star"]
    modes = _planner_modes(args.ompl_mode)

    records: List[Dict[str, Any]] = [hybrid_row]
    executable = None
    if args.ompl_executable:
        executable = [str(Path(args.ompl_executable).expanduser().resolve())]

    for planner in planners:
        for mode in modes:
            request = dict(request_base)
            request["planner_type"] = planner
            request["simplify_enable"] = mode == "simplify"
            request_name = f"ompl_{planner}_{mode}"
            started = time.perf_counter()
            planner_result = _run_ompl_backend(
                request,
                out_dir=out_dir,
                request_name=request_name,
                executable=executable,
            )
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            path = np.asarray(planner_result.get("path", []), dtype=float)
            if path.ndim != 2:
                path = np.empty((0, 2), dtype=float)
            records.append(
                _metric_row(
                    planner=planner,
                    mode=mode,
                    kind="ompl",
                    path=path,
                    obstacles=obstacles,
                    safety_margin=safety_margin,
                    elapsed_ms=elapsed_ms,
                    planner_result=planner_result,
                    success=bool(planner_result.get("ok", False)),
                    failure_reason=str(planner_result.get("error_reason", ""))
                    if not planner_result.get("ok", False)
                    else "",
                )
            )
            plot_records.append({**records[-1], "path": path.tolist()})
            _write_json(out_dir / f"{request_name}.result.json", planner_result)

    summary = {
        "scenario_name": args.scenario_name,
        "config_path": str(config_path),
        "scenario_path": str(scenario_path),
        "nominal_count": int(nominal.shape[0]),
        "obstacle_count": len(obstacles),
        "safety_margin": float(safety_margin),
        "planners": records,
        "result_interpretable": True,
    }
    _write_json(out_dir / "comparison_result.json", summary)
    _write_csv(out_dir / "comparison_result.csv", records)
    _plot_compare(out_dir, nominal, obstacles, plot_records, safety_margin)

    LOGGER.info("Comparison completed: scenario=%s out=%s", args.scenario_name, out_dir)
    for record in records:
        LOGGER.info(
            "[comparison] planner=%s mode=%s success=%s time_ms=%.2f path_length=%.4f jerk=%.4f clearance=%s",
            record["planner"],
            record["mode"],
            record["success"],
            record["planning_time_ms"],
            record["path_length"],
            record["jerk_integral"],
            "inf" if record["min_clearance"] is None else f"{record['min_clearance']:.4f}",
        )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="table plane YAML")
    parser.add_argument("--scenario", required=True, help="scenario YAML")
    parser.add_argument("--scenario-name", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument(
        "--planners",
        default="rrt,rrt_star,informed_rrt_star",
        help="comma-separated OMPL planners to compare",
    )
    parser.add_argument("--ompl-mode", default="raw", choices=["raw", "simplify", "both"])
    parser.add_argument("--timeout-sec", type=float, default=0.0)
    parser.add_argument("--step-size", type=float, default=0.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.0)
    parser.add_argument("--edge-resolution", type=float, default=0.0)
    parser.add_argument("--goal-bias", type=float, default=0.0)
    parser.add_argument("--rng-seed", type=int, default=0)
    parser.add_argument("--simplify-timeout-sec", type=float, default=0.0)
    parser.add_argument("--simplify-at-least-once", action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument(
        "--ompl-executable",
        default="",
        help="optional absolute path to the ompl_2d_benchmark executable",
    )
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="[%(levelname)s] [PlaneHybrid] %(message)s",
    )
    try:
        run(args)
        return 0
    except Exception as exc:  # CLI should preserve a useful summary instead of raw crash output.
        LOGGER.exception("Comparison failed: %s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
