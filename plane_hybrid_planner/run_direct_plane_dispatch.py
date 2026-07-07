"""Direct planar waypoint dispatch for isolating the ur_move path.

This utility bypasses the 2D obstacle pipeline entirely and sends a clean
table-plane trajectory to ur_move. It is meant to answer one narrow question:
does the downstream plane mapping + ur_move + MoveIt path remain smooth when
the upstream planner is not involved?
"""

from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
from typing import Any, Dict, Sequence

import numpy as np
import yaml

from .path_resample import resample_path_by_arclength
from .plane_mapping import PlaneMapper
from .ur_move_zmq_client import UrMoveZmqClient


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


def _plot_direct_compare(out_dir: Path, uv_path: np.ndarray, cart_waypoints: list[Dict[str, Any]]) -> None:
    import matplotlib  # pylint: disable=import-outside-toplevel

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # pylint: disable=import-outside-toplevel

    figure, axis = plt.subplots(figsize=(8.2, 6.2))
    axis.plot(uv_path[:, 0], uv_path[:, 1], "o-", color="#2563eb", markersize=3, label="UV path")
    axis.scatter(uv_path[0, 0], uv_path[0, 1], color="#111827", s=50, label="start")
    axis.scatter(uv_path[-1, 0], uv_path[-1, 1], marker="*", s=95, color="#111827", label="goal")
    axis.set(xlabel="u", ylabel="v", title="Direct plane path comparison")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(out_dir / "trajectory_compare.png", dpi=170)
    plt.close(figure)

    if cart_waypoints:
        xyz = np.asarray([waypoint["position"] for waypoint in cart_waypoints], dtype=float)
        figure, axis = plt.subplots(figsize=(7.2, 5.8))
        axis.plot(xyz[:, 0], xyz[:, 1], "o-", color="#0f766e", markersize=3, label="Cartesian XY")
        axis.scatter(xyz[0, 0], xyz[0, 1], color="#111827", s=50, label="start")
        axis.scatter(xyz[-1, 0], xyz[-1, 1], marker="*", s=95, color="#111827", label="goal")
        axis.set(xlabel="x [m]", ylabel="y [m]", title="Direct Cartesian waypoints")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.25)
        axis.legend(loc="best")
        figure.tight_layout()
        figure.savefig(out_dir / "cart_xy_path.png", dpi=170)
        plt.close(figure)


def _resolve_path(value: str, relative_to: Path) -> Path:
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (relative_to / path).resolve()


def _demo_uv_path(mode: str) -> np.ndarray:
    mode = str(mode).lower()
    if mode == "polyline":
        anchors = np.asarray(
            [
                [0.08, 0.50],
                [0.28, 0.56],
                [0.52, 0.44],
                [0.74, 0.55],
                [0.92, 0.50],
            ],
            dtype=float,
        )
    elif mode == "line":
        anchors = np.asarray([[0.08, 0.50], [0.92, 0.50]], dtype=float)
    else:
        raise ValueError("path mode must be 'line' or 'polyline'")
    return anchors


def run(args: argparse.Namespace) -> Dict[str, Any]:
    config_path = Path(args.config).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    config = _load_yaml(config_path)
    group_name = str(args.group or config.get("group_name", "left_arm"))
    if group_name not in {"left_arm", "right_arm"}:
        raise ValueError("group must be left_arm or right_arm")

    mapper = PlaneMapper.from_config(config)
    planner_name = str(args.planner or config.get("planner", "lin"))
    ik_frame = str(args.ik_frame or config.get("ik_frame", f"{group_name.split('_')[0]}_ee_link"))

    demo_mode = str(args.path_mode)
    uv_path = _demo_uv_path(demo_mode)
    uv_path = resample_path_by_arclength(uv_path, int(args.num_points))
    cart_waypoints = mapper.uv_path_to_cart_waypoints(uv_path)

    _write_json(
        out_dir / "direct_uv_path.json",
        {
            "path_mode": demo_mode,
            "uv_path": uv_path.tolist(),
            "count": int(uv_path.shape[0]),
        },
    )
    _write_json(
        out_dir / "cart_waypoints.json",
        {
            "group_name": group_name,
            "frame_id": mapper.frame_id,
            "coordinate_mode": mapper.coordinate_mode,
            "ik_frame": ik_frame,
            "waypoints": cart_waypoints,
        },
    )
    _write_json(
        out_dir / "direct_path_debug.json",
        {
            "path_mode": demo_mode,
            "uv_path": uv_path.tolist(),
            "cart_waypoints": cart_waypoints,
            "group_name": group_name,
            "planner": planner_name,
            "frame_id": mapper.frame_id,
            "coordinate_mode": mapper.coordinate_mode,
            "ik_frame": ik_frame,
        },
    )

    ur_config = config.get("ur_move", {})
    workspace_root = config.get("workspace_root", "")
    resolved_workspace = (
        _resolve_path(str(workspace_root), config_path.parent) if workspace_root else None
    )
    client = UrMoveZmqClient(
        host=str(args.host or ur_config.get("host", "127.0.0.1")),
        port=int(args.port or ur_config.get("port", 5605)),
        timeout_sec=float(args.timeout_sec or ur_config.get("timeout_sec", 10.0)),
        workspace_root=str(resolved_workspace) if resolved_workspace else None,
    )
    plan_only = not bool(args.execute)
    response = client.send_cart_waypoints(
        cart_waypoints,
        group_name=group_name,
        ik_frame=ik_frame,
        planner=planner_name,
        plan_only=plan_only,
        execute=bool(args.execute),
        velocity_scale=float(ur_config.get("velocity_scale", 0.1)),
        acceleration_scale=float(ur_config.get("acceleration_scale", 0.1)),
    )
    LOGGER.info("ur_move raw response: %s", json.dumps(response["response"], ensure_ascii=False))
    _write_json(out_dir / "ur_move_response.json", response)

    summary = {
        "dispatch": True,
        "group": group_name,
        "planner": planner_name,
        "frame_id": mapper.frame_id,
        "coordinate_mode": mapper.coordinate_mode,
        "ik_frame": ik_frame,
        "path_mode": demo_mode,
        "uv_point_count": int(uv_path.shape[0]),
        "cart_waypoint_count": len(cart_waypoints),
        "ur_move_available": bool(response.get("available", False)),
        "moveit_plan_success": bool(response.get("planning_success", False)),
        "execution_requested": bool(response.get("execution_requested", False)),
        "execution_success": response.get("execution_success"),
        "execution_id": response.get("execution_id", ""),
        "error_kind": str(response.get("error_kind", "")),
        "error": str(response.get("error", "")),
        "transport": str(response.get("transport", "")),
    }
    _write_json(out_dir / "direct_dispatch_summary.json", summary)

    LOGGER.info("[plane_hybrid] dispatch=True")
    LOGGER.info("[plane_hybrid] group=%s", group_name)
    LOGGER.info("[plane_hybrid] planner=%s", planner_name)
    LOGGER.info("[plane_hybrid] frame_id=%s", mapper.frame_id)
    LOGGER.info("[plane_hybrid] coordinate_mode=%s", mapper.coordinate_mode)
    LOGGER.info("[plane_hybrid] ik_frame=%s", ik_frame)
    LOGGER.info("[plane_hybrid] uv_point_count=%d", int(uv_path.shape[0]))
    LOGGER.info("[plane_hybrid] cart_waypoint_count=%d", len(cart_waypoints))
    LOGGER.info("[plane_hybrid] ur_move_available=%s", summary["ur_move_available"])
    LOGGER.info("[plane_hybrid] moveit_plan_success=%s", summary["moveit_plan_success"])
    _plot_direct_compare(out_dir, uv_path, cart_waypoints)
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="table plane YAML")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--group", choices=["left_arm", "right_arm"], default="")
    parser.add_argument("--planner", default="")
    parser.add_argument("--ik-frame", default="")
    parser.add_argument("--host", default="")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--timeout-sec", type=float, default=0.0)
    parser.add_argument("--path-mode", choices=["line", "polyline"], default="line")
    parser.add_argument("--num-points", type=int, default=30)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--plan-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.set_defaults(plan_only=True, execute=False)
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
    except Exception as exc:  # Keep CLI failures actionable instead of traceback-only.
        LOGGER.exception("Direct plane dispatch failed: %s", exc)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
