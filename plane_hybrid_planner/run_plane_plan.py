"""Command-line entry point for 2D planning and ur_move plan/execute requests."""

from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence

import numpy as np
import yaml

from .evaluate_2d import build_evaluation, write_evaluation
from .matlab_hybrid_2d import run_matlab_hybrid
from .path_resample import resample_path_by_arclength, simplify_path_optional
from .plane_mapping import PlaneMapper
from .planner_2d import (
    generate_nominal_path,
    normalize_obstacles,
    path_collision_details,
    path_min_clearance,
    point_clearance,
)
from .scene_sync import sync_circular_obstacles_to_moveit
from .ur_move_zmq_client import UrMoveZmqClient


LOGGER = logging.getLogger("plane_hybrid_planner")


def _load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a mapping: {path}")
    return data


def _resolve_path(value: str, relative_to: Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (relative_to / path).resolve()


def _scenario_from_file(document: Dict[str, Any], name: str) -> Dict[str, Any]:
    scenarios = document.get("scenarios", document)
    if not isinstance(scenarios, dict) or name not in scenarios:
        available = ", ".join(sorted(scenarios.keys())) if isinstance(scenarios, dict) else ""
        raise KeyError(f"scenario '{name}' not found; available: {available}")
    scenario = scenarios[name]
    if not isinstance(scenario, dict):
        raise ValueError(f"scenario '{name}' must be a mapping")
    merged = dict(scenario)
    defaults = document.get("matlab_defaults", {})
    if isinstance(defaults, dict):
        if "environment_seed" not in merged and "environment_seed" in defaults:
            merged["environment_seed"] = defaults["environment_seed"]

        default_fmp = defaults.get("matlab_fmp", {})
        scenario_fmp = merged.get("matlab_fmp", merged.get("fmp", {}))
        if isinstance(default_fmp, dict) or isinstance(scenario_fmp, dict):
            merged["matlab_fmp"] = {
                **(default_fmp if isinstance(default_fmp, dict) else {}),
                **(scenario_fmp if isinstance(scenario_fmp, dict) else {}),
            }

        default_rrt = defaults.get("matlab_rrt", {})
        scenario_rrt = merged.get("matlab_rrt", merged.get("rrt", {}))
        if isinstance(default_rrt, dict) or isinstance(scenario_rrt, dict):
            merged["matlab_rrt"] = {
                **(default_rrt if isinstance(default_rrt, dict) else {}),
                **(scenario_rrt if isinstance(scenario_rrt, dict) else {}),
            }
    return merged


def _write_json(path: Path, data: Any) -> None:
    with path.open("w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, ensure_ascii=False, allow_nan=False)


def _clearance_profile(path: np.ndarray, obstacles: List[Dict[str, Any]]) -> np.ndarray:
    if not obstacles:
        return np.full(path.shape[0], np.nan, dtype=float)
    return np.asarray([point_clearance(point, obstacles) for point in path], dtype=float)


def _plot_results(
    out_dir: Path,
    nominal: np.ndarray,
    rrt_path: np.ndarray,
    modulated: np.ndarray,
    simplified: np.ndarray,
    resampled: np.ndarray,
    cart_waypoints: List[Dict[str, Any]],
    obstacles: List[Dict[str, Any]],
    safety_margin: float,
) -> None:
    import matplotlib  # pylint: disable=import-outside-toplevel

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    figure, axis = plt.subplots(figsize=(8.0, 6.5))
    axis.plot(nominal[:, 0], nominal[:, 1], "--", color="#2563eb", label="nominal", linewidth=2)
    if rrt_path.size:
        axis.plot(rrt_path[:, 0], rrt_path[:, 1], "o-", color="#f97316", label="RRT coarse", markersize=3)
    axis.plot(modulated[:, 0], modulated[:, 1], color="#15803d", label="FMP modulated", linewidth=2.5)
    if simplified.size:
        axis.plot(
            simplified[:, 0],
            simplified[:, 1],
            "-.",
            color="#7c3aed",
            label="simplified",
            linewidth=1.6,
        )
    if resampled.size:
        axis.plot(
            resampled[:, 0],
            resampled[:, 1],
            "x-",
            color="#0f766e",
            label="resampled",
            markersize=4,
            linewidth=1.2,
        )
    for index, obstacle in enumerate(obstacles):
        center = obstacle["center"]
        circle = plt.Circle(center, obstacle["radius"], color="#dc2626", alpha=0.28)
        margin_circle = plt.Circle(
            center,
            obstacle["radius"] + safety_margin,
            fill=False,
            linestyle=":",
            color="#991b1b",
            linewidth=1.3,
        )
        axis.add_patch(circle)
        axis.add_patch(margin_circle)
        axis.text(center[0], center[1], f"O{index + 1}", ha="center", va="center", fontsize=8)
    axis.scatter(*nominal[0], marker="o", color="#111827", s=55, label="start", zorder=5)
    axis.scatter(*nominal[-1], marker="*", color="#111827", s=90, label="goal", zorder=5)
    axis.set(xlim=(0.0, 1.0), ylim=(0.0, 1.0), xlabel="u", ylabel="v", title="2D table-plane path")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(out_dir / "uv_path_compare.png", dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(8.8, 6.4))
    axis.set_title("2D planning trajectory comparison")
    if nominal.size:
        axis.plot(nominal[:, 0], nominal[:, 1], "--", color="#2563eb", label="nominal", linewidth=2)
    if rrt_path.size:
        axis.plot(rrt_path[:, 0], rrt_path[:, 1], "o-", color="#f97316", label="RRT coarse", markersize=3)
    if simplified.size:
        axis.plot(simplified[:, 0], simplified[:, 1], "-.", color="#7c3aed", label="simplified", linewidth=1.6)
    if resampled.size:
        axis.plot(resampled[:, 0], resampled[:, 1], "x-", color="#0f766e", label="resampled", markersize=4)
    if modulated.size:
        axis.plot(modulated[:, 0], modulated[:, 1], color="#15803d", label="FMP modulated", linewidth=2.5)
    for index, obstacle in enumerate(obstacles):
        center = obstacle["center"]
        circle = plt.Circle(center, obstacle["radius"], color="#dc2626", alpha=0.22)
        margin_circle = plt.Circle(
            center,
            obstacle["radius"] + safety_margin,
            fill=False,
            linestyle=":",
            color="#991b1b",
            linewidth=1.0,
        )
        axis.add_patch(circle)
        axis.add_patch(margin_circle)
        axis.text(center[0], center[1], f"O{index + 1}", ha="center", va="center", fontsize=8)
    if nominal.size:
        axis.scatter(*nominal[0], marker="o", color="#111827", s=50, label="start", zorder=5)
        axis.scatter(*nominal[-1], marker="*", color="#111827", s=95, label="goal", zorder=5)
    axis.set(xlim=(0.0, 1.0), ylim=(0.0, 1.0), xlabel="u", ylabel="v")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(out_dir / "trajectory_compare.png", dpi=170)
    plt.close(figure)

    nominal_clearance = _clearance_profile(nominal, obstacles)
    modulated_clearance = _clearance_profile(modulated, obstacles)
    figure, axis = plt.subplots(figsize=(9.0, 4.8))
    if obstacles:
        axis.plot(np.linspace(0.0, 1.0, nominal.shape[0]), nominal_clearance, "--", label="nominal")
        axis.plot(np.linspace(0.0, 1.0, modulated.shape[0]), modulated_clearance, label="modulated")
    else:
        axis.text(0.5, 0.5, "No obstacles: clearance is unbounded", ha="center", va="center")
    axis.axhline(0.0, color="#dc2626", linewidth=1.2, label="collision boundary")
    axis.axhline(safety_margin, color="#f59e0b", linestyle=":", label="safety margin")
    axis.set(xlabel="normalized path progress", ylabel="clearance in UV", title="Obstacle clearance")
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(out_dir / "clearance_plot.png", dpi=160)
    plt.close(figure)

    if cart_waypoints:
        xyz = np.asarray([waypoint["position"] for waypoint in cart_waypoints], dtype=float)
        figure, axis = plt.subplots(figsize=(7.0, 6.0))
        axis.plot(xyz[:, 0], xyz[:, 1], "o-", color="#0f766e", markersize=3)
        axis.scatter(xyz[0, 0], xyz[0, 1], color="#111827", label="start")
        axis.scatter(xyz[-1, 0], xyz[-1, 1], marker="*", s=80, color="#111827", label="goal")
        axis.set(xlabel="x [m]", ylabel="y [m]", title="Cartesian XY waypoints")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
        figure.tight_layout()
        figure.savefig(out_dir / "cart_xy_path.png", dpi=160)
        plt.close(figure)


def _path_debug_payload(
    *,
    nominal: np.ndarray,
    rrt_path: np.ndarray,
    modulated: np.ndarray,
    simplified: np.ndarray,
    resampled: np.ndarray,
    cart_waypoints: List[Dict[str, Any]],
    dispatch_summary: Dict[str, Any],
) -> Dict[str, Any]:
    return {
        "nominal_path": nominal.tolist(),
        "rrt_path": rrt_path.tolist(),
        "modulated_path": modulated.tolist(),
        "simplified_path": simplified.tolist(),
        "resampled_path": resampled.tolist(),
        "cart_waypoints": cart_waypoints,
        "nominal_count": int(nominal.shape[0]),
        "rrt_count": int(rrt_path.shape[0]),
        "modulated_count": int(modulated.shape[0]),
        "simplified_count": int(simplified.shape[0]),
        "resampled_count": int(resampled.shape[0]),
        "cart_waypoint_count": int(len(cart_waypoints)),
        "dispatch_summary": dispatch_summary,
    }


def _log_dispatch_summary(summary: Dict[str, Any]) -> None:
    for key in (
        "nominal_count",
        "modulated_count",
        "algorithm_failure",
        "safety_pass",
        "simplified_count",
        "resampled_count",
        "cart_waypoint_count",
        "dispatch",
        "group",
        "planner",
        "frame_id",
        "ik_frame",
        "preposition_requested",
        "scene_sync_success",
    ):
        LOGGER.info("[plane_hybrid] %s=%s", key, summary.get(key))
    if not summary.get("dispatch", False):
        LOGGER.info("[plane_hybrid] blocked_stage=%s", summary.get("blocked_stage", ""))
        LOGGER.info("[plane_hybrid] blocked_reason=%s", summary.get("blocked_reason", ""))


def _format_clearance(value: float) -> str:
    return "inf" if not math.isfinite(value) else f"{value:.4f}"


def run(args: argparse.Namespace) -> Dict[str, Any]:
    config_path = Path(args.config).expanduser().resolve()
    scenario_path = Path(args.scenario).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    config = _load_yaml(config_path)
    scenario = _scenario_from_file(_load_yaml(scenario_path), args.scenario_name)
    group_name = str(args.group or config.get("group_name", "left_arm"))
    if group_name not in {"left_arm", "right_arm"}:
        raise ValueError("group must be left_arm or right_arm")

    nominal = generate_nominal_path(scenario["nominal"])
    obstacles = normalize_obstacles(scenario.get("obstacles", []))
    safety_margin = float(scenario.get("safety_margin", 0.015))
    nominal_details = path_collision_details(nominal, obstacles, margin=0.0)
    nominal_clearance = path_min_clearance(nominal, obstacles)
    LOGGER.info(
        "Scenario=%s group=%s nominal_collisions=%d min_clearance=%s",
        args.scenario_name,
        group_name,
        nominal_details["collision_count"],
        "inf" if not math.isfinite(nominal_clearance) else f"{nominal_clearance:.4f}",
    )

    rrt_result: Dict[str, Any] = {
        "ok": True,
        "path": [],
        "stop_reason": "not_required",
        "iter_used": 0,
        "collision_queries": 0,
        "elapsed_ms": 0.0,
    }
    rrt_path = np.empty((0, 2), dtype=float)
    modulated = nominal.copy()
    algorithm_failure = ""
    fmp_fallback_used = False
    blocked_stage = ""
    blocked_reason = ""
    dispatch_failure_reason = ""
    simplified_count = 0
    resampled_count = 0
    cart_waypoint_count = 0
    simplified_path = np.empty((0, 2), dtype=float)
    resampled_path = np.empty((0, 2), dtype=float)
    planner_name = str(config.get("planner", "lin"))
    ik_frame = str(config.get("ik_frame", f"{group_name.split('_')[0]}_ee_link"))
    mapper = PlaneMapper.from_config(config)
    scene_sync_result = sync_circular_obstacles_to_moveit(
        obstacles,
        mapper,
        scene_config=config.get("scene_obstacles", {}),
    )
    if scene_sync_result["attempted"]:
        LOGGER.info(
            "Scene obstacle sync: success=%s count=%d frame=%s reason=%s",
            scene_sync_result["success"],
            scene_sync_result["obstacle_count"],
            scene_sync_result["frame_id"],
            scene_sync_result["reason"] or "none",
        )

    matlab_result = run_matlab_hybrid(
        nominal,
        obstacles,
        parameter_overrides=scenario.get("matlab_fmp", {}),
        rrt_overrides=scenario.get("matlab_rrt", {}),
        environment_seed=int(scenario.get("environment_seed", 2025)),
    )
    nominal = np.asarray(matlab_result["nominal_path"], dtype=float)
    modulated = np.asarray(matlab_result["modulated_path"], dtype=float)
    plotted_rrt_paths = matlab_result.get("rrt_paths", [])
    if plotted_rrt_paths:
        rrt_path = np.vstack(plotted_rrt_paths)
    rrt_result = {
        "ok": bool(matlab_result["ok"]),
        "path": rrt_path.tolist(),
        "stop_reason": str(matlab_result["rrt_stop_reason"]),
        "iter_used": int(matlab_result["rrt_iter_used"]),
        "collision_queries": int(matlab_result["rrt_collision_queries"]),
        "elapsed_ms": float(matlab_result["rrt_elapsed_ms"]),
    }
    if not matlab_result["ok"]:
        algorithm_failure = "matlab_rrt_failed"
        blocked_stage = "algorithm"
        blocked_reason = (
            f"rrt_stop_reason={rrt_result['stop_reason']}, "
            f"iter_used={rrt_result['iter_used']}, "
            f"collision_queries={rrt_result['collision_queries']}"
        )
        LOGGER.error("MATLAB-compatible local Informed RRT* failed")
    else:
        strict_collision = path_collision_details(modulated, obstacles, margin=0.0)
        strict_margin = path_collision_details(modulated, obstacles, margin=safety_margin)
        if strict_collision["collision_count"] > 0:
            algorithm_failure = "modulated_path_in_collision"
            first_invalid_idx = (
                strict_collision["invalid_point_indices"][0]
                if strict_collision["invalid_point_indices"]
                else (
                    strict_collision["invalid_edge_indices"][0]
                    if strict_collision["invalid_edge_indices"]
                    else -1
                )
            )
            blocked_stage = "safety_check"
            blocked_reason = (
                f"first_invalid_idx={first_invalid_idx}, "
                f"clearance={_format_clearance(path_min_clearance(modulated, obstacles))}, "
                f"collision_count={strict_collision['collision_count']}"
            )
            LOGGER.error("MATLAB-compatible FMP output failed strict 2D collision post-check")
        elif strict_margin["collision_count"] > 0:
            algorithm_failure = "modulated_clearance_below_margin"
            first_invalid_idx = (
                strict_margin["invalid_point_indices"][0]
                if strict_margin["invalid_point_indices"]
                else (
                    strict_margin["invalid_edge_indices"][0]
                    if strict_margin["invalid_edge_indices"]
                    else -1
                )
            )
            blocked_stage = "safety_check"
            blocked_reason = (
                f"first_invalid_idx={first_invalid_idx}, "
                f"clearance={_format_clearance(path_min_clearance(modulated, obstacles))}, "
                f"safety_margin={safety_margin:.4f}"
            )
            LOGGER.error("MATLAB-compatible FMP output is collision-free but below the safety margin")

    waypoint_config = config.get("waypoints", {})
    cart_waypoints: List[Dict[str, Any]] = []
    if not algorithm_failure:
        min_dist = float(waypoint_config.get("simplify_min_dist", 0.0))
        simplified_path = simplify_path_optional(modulated, min_dist=min_dist)
        simplified_count = int(simplified_path.shape[0])
        resampled_path = resample_path_by_arclength(
            simplified_path, int(waypoint_config.get("resample_count", 30))
        )
        resampled_count = int(resampled_path.shape[0])
        cart_waypoints = mapper.uv_path_to_cart_waypoints(resampled_path)
        cart_waypoint_count = len(cart_waypoints)

    _write_json(
        out_dir / "cart_waypoints.json",
        {
            "scenario_name": args.scenario_name,
            "group_name": group_name,
            "frame_id": mapper.frame_id,
            "start_waypoint": cart_waypoints[0] if cart_waypoints else None,
            "waypoints": cart_waypoints,
        },
    )
    _write_json(out_dir / "scene_obstacles.json", scene_sync_result)
    _write_json(
        out_dir / "planning_path_debug.json",
        _path_debug_payload(
            nominal=nominal,
            rrt_path=rrt_path,
            modulated=modulated,
            simplified=simplified_path,
            resampled=resampled_path,
            cart_waypoints=cart_waypoints,
            dispatch_summary=dispatch_summary,
        ),
    )

    ur_move_result: Dict[str, Any] = {
        "available": False,
        "planning_success": False,
        "execution_requested": False,
        "execution_success": None,
        "execution_id": "",
        "error": "2D path not safe; request not sent" if algorithm_failure else "",
        "error_kind": "algorithm_failure" if algorithm_failure else "",
        "transport": "",
        "request": {},
        "response": {},
    }
    preposition_result: Dict[str, Any] = {
        "requested": False,
        "planning_success": False,
        "execution_success": None,
        "error": "",
        "error_kind": "",
        "planner": "",
        "response": {},
    }
    execution_config = config.get("execution", {})
    if args.execute:
        plan_only = False
        execute = True
    elif args.plan_only:
        plan_only = True
        execute = False
    else:
        plan_only = bool(execution_config.get("plan_only", True))
        execute = bool(execution_config.get("execute", False)) and not plan_only

    if not algorithm_failure and cart_waypoints:
        ur_config = config.get("ur_move", {})
        workspace_root = config.get("workspace_root", "")
        resolved_workspace = (
            str(_resolve_path(str(workspace_root), config_path.parent)) if workspace_root else None
        )
        client = UrMoveZmqClient(
            host=str(ur_config.get("host", "127.0.0.1")),
            port=int(ur_config.get("port", 5605)),
            timeout_sec=float(ur_config.get("timeout_sec", 60.0)),
            workspace_root=resolved_workspace,
        )
        move_to_start_first = bool(execution_config.get("move_to_start_first", True))
        if move_to_start_first and cart_waypoints:
            preposition_planner = str(execution_config.get("preposition_planner", "ompl"))
            preposition_result = client.send_cart_waypoints(
                [cart_waypoints[0]],
                group_name=group_name,
                ik_frame=ik_frame,
                planner=preposition_planner,
                plan_only=plan_only,
                execute=execute,
                velocity_scale=float(ur_config.get("velocity_scale", 0.1)),
                acceleration_scale=float(ur_config.get("acceleration_scale", 0.1)),
            )
            preposition_result["requested"] = True
            preposition_result["planner"] = preposition_planner
            LOGGER.info(
                "Preposition response: %s",
                json.dumps(preposition_result["response"], ensure_ascii=False),
            )
            preposition_ok = bool(preposition_result.get("planning_success", False))
            if execute and preposition_result.get("execution_success") is False:
                preposition_ok = False
            if not preposition_ok:
                blocked_stage = "preposition"
                blocked_reason = str(preposition_result.get("error", "preposition failed"))
                dispatch_failure_reason = "preposition_failed"
                ur_move_result = preposition_result

        if not dispatch_failure_reason:
            ur_move_result = client.send_cart_waypoints(
                cart_waypoints,
                group_name=group_name,
                ik_frame=ik_frame,
                planner=planner_name,
                plan_only=plan_only,
                execute=execute,
                velocity_scale=float(ur_config.get("velocity_scale", 0.1)),
                acceleration_scale=float(ur_config.get("acceleration_scale", 0.1)),
            )
            LOGGER.info("ur_move raw response: %s", json.dumps(ur_move_result["response"], ensure_ascii=False))

    dispatch = bool(
        not algorithm_failure
        and not dispatch_failure_reason
        and cart_waypoint_count > 0
    )
    dispatch_summary = {
        "nominal_count": int(nominal.shape[0]),
        "modulated_count": int(modulated.shape[0]),
        "algorithm_failure": algorithm_failure or dispatch_failure_reason or None,
        "safety_pass": bool(not algorithm_failure),
        "simplified_count": int(simplified_count),
        "resampled_count": int(resampled_count),
        "cart_waypoint_count": int(cart_waypoint_count),
        "dispatch": bool(dispatch),
        "group": group_name,
        "planner": planner_name,
        "frame_id": mapper.frame_id,
        "ik_frame": ik_frame,
        "blocked_stage": blocked_stage,
        "blocked_reason": blocked_reason,
        "preposition_requested": bool(preposition_result.get("requested", False)),
        "scene_sync_success": bool(scene_sync_result.get("success", False)),
    }
    _log_dispatch_summary(dispatch_summary)

    _write_json(out_dir / "ur_move_response.json", ur_move_result)
    _write_json(out_dir / "preposition_response.json", preposition_result)
    result = build_evaluation(
        scenario_name=args.scenario_name,
        group_name=group_name,
        nominal_path=nominal,
        modulated_path=modulated,
        obstacles=obstacles,
        safety_margin=safety_margin,
        rrt_result=rrt_result,
        cart_waypoint_count=len(cart_waypoints),
        ur_move_result=ur_move_result,
        failure_reason=algorithm_failure or dispatch_failure_reason,
    )
    result.update(
        {
            "nominal_path_points": int(nominal.shape[0]),
            "rrt_path_points": int(rrt_path.shape[0]),
            "modulated_path_points": int(modulated.shape[0]),
            "fmp_fallback_used": bool(fmp_fallback_used),
            "algorithm": "matlab_intent_informed_rrt_star_ta_hdi_fmp",
            "danger_count": int(matlab_result["danger_count"]),
            "danger_segment_count": int(len(matlab_result["segments"])),
            "via_point_count": int(np.asarray(matlab_result["via_points"]).shape[0]),
            "matlab_parameters": matlab_result.get("matlab_parameters", {}),
            "matlab_rrt_parameters": matlab_result.get("matlab_rrt_parameters", {}),
            "corner_smoothing_metadata": matlab_result.get("corner_smoothing_metadata", {}),
            "plane_frame_id": mapper.frame_id,
            "dispatch_summary": dispatch_summary,
            "dispatch_requested": bool(dispatch),
            "scene_sync": scene_sync_result,
            "preposition_result": preposition_result,
            "start_cart_waypoint": cart_waypoints[0] if cart_waypoints else None,
        }
    )
    write_evaluation(result, out_dir)
    _plot_results(
        out_dir,
        nominal=nominal,
        rrt_path=rrt_path,
        modulated=modulated,
        simplified=simplified_path,
        resampled=resampled_path,
        cart_waypoints=cart_waypoints,
        obstacles=obstacles,
        safety_margin=safety_margin,
    )

    LOGGER.info(
        "Completed: avoidance_success=%s moveit_plan_success=%s failure_reason=%s out=%s",
        result["avoidance_success"],
        result["moveit_plan_success"],
        result["failure_reason"] or "none",
        out_dir,
    )
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="table plane YAML")
    parser.add_argument("--scenario", required=True, help="scenario YAML")
    parser.add_argument("--scenario-name", required=True)
    parser.add_argument("--group", choices=["left_arm", "right_arm"], default="")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--plan-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--out-dir", required=True)
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
    except Exception as exc:  # CLI must leave a useful error instead of a traceback-only failure.
        LOGGER.exception("Plane planning failed: %s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
