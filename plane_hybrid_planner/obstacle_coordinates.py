"""Normalize configured table obstacles into canonical UV coordinates."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

from .plane_mapping import PlaneMapper
from .planner_2d import normalize_obstacles


def _center2(value: Any, label: str) -> Tuple[float, float]:
    center = np.asarray(value, dtype=float).reshape(-1)
    if center.size != 2 or not np.all(np.isfinite(center)):
        raise ValueError(f"{label} must contain two finite values")
    return float(center[0]), float(center[1])


def _input_config(
    scenario: Dict[str, Any],
    config: Dict[str, Any],
) -> Dict[str, Any]:
    merged = dict(config.get("obstacle_input", {}) or {})
    merged.update(scenario.get("obstacle_input", {}) or {})
    if "coordinate_mode" not in merged:
        merged["coordinate_mode"] = "normalized"
    if "frame_id" not in merged:
        merged["frame_id"] = config.get("plane", {}).get("frame_id", "")
    if "radius_scale_mode" not in merged:
        merged["radius_scale_mode"] = "min"
    return merged


def obstacle_input_config(scenario: Dict[str, Any], config: Dict[str, Any]) -> Dict[str, Any]:
    """Return the effective obstacle-input coordinate contract."""
    return _input_config(scenario, config)


def normalize_obstacles_to_uv(
    obstacles: Iterable[Dict[str, Any]],
    mapper: PlaneMapper,
    *,
    input_mode: str = "normalized",
    input_frame: str = "",
    radius_scale_mode: str = "min",
    strict_metric_bounds: bool = True,
) -> List[Dict[str, Any]]:
    """Convert configured circle obstacles to the canonical UV algorithm format."""
    mode = str(input_mode).strip().lower()
    frame = str(input_frame or mapper.frame_id)
    scale_mode = str(radius_scale_mode).strip().lower()

    if mode not in {"normalized", "metric"}:
        raise ValueError("obstacle_input.coordinate_mode must be 'normalized' or 'metric'")
    if mode == "metric" and frame != mapper.frame_id:
        raise ValueError(
            "obstacle_input.frame_id must match plane.frame_id unless a real TF2 "
            "transform is applied. Changing a frame_id label is not a coordinate "
            f"transform. input_frame={frame}, plane_frame={mapper.frame_id}"
        )

    canonical: List[Dict[str, Any]] = []
    for index, obstacle in enumerate(obstacles or []):
        if str(obstacle.get("type", "circle")).lower() != "circle":
            raise ValueError(f"obstacle {index}: only circle is supported")
        input_center = _center2(obstacle["center"], f"obstacle {index} center")
        radius = float(obstacle["radius"])
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError(f"obstacle {index}: radius must be positive")

        if mode == "metric":
            center_uv = mapper.xy_to_uv(
                input_center[0],
                input_center[1],
                strict=bool(strict_metric_bounds),
            )
            radius_uv = mapper.metric_radius_to_uv(radius, mode=scale_mode)
        else:
            center_uv = mapper._validate_uv(  # pylint: disable=protected-access
                input_center[0],
                input_center[1],
                clamp=False,
            )
            radius_uv = radius

        item = {
            "type": "circle",
            "id": str(obstacle.get("id", f"obstacle_{index + 1:02d}")),
            "center": [float(center_uv[0]), float(center_uv[1])],
            "radius": float(radius_uv),
            "coordinate_mode": "normalized",
            "input_mode": mode,
            "input_frame": frame,
            "input_center": [float(input_center[0]), float(input_center[1])],
            "input_radius": float(radius),
            "radius_scale_mode": scale_mode,
        }
        canonical.append(item)

    return normalize_obstacles(canonical)


def coordinate_debug_payload(
    obstacles_uv: Iterable[Dict[str, Any]],
    mapper: PlaneMapper,
) -> Dict[str, Any]:
    """Build a roundtrip debug payload for canonical UV obstacles."""
    obstacle_payload = []
    roundtrip_pass = True
    for obstacle in obstacles_uv:
        center_uv = [float(obstacle["center"][0]), float(obstacle["center"][1])]
        roundtrip_xy = list(mapper.uv_to_xy(center_uv[0], center_uv[1], clamp=False))
        radius_uv = float(obstacle["radius"])
        mode = str(obstacle.get("radius_scale_mode", "min"))
        roundtrip_radius_m = mapper.uv_radius_to_metric(radius_uv, mode=mode)
        item: Dict[str, Any] = {
            "id": str(obstacle.get("id", "")),
            "input_mode": str(obstacle.get("input_mode", "normalized")),
            "input_center": list(obstacle.get("input_center", center_uv)),
            "center_uv": center_uv,
            "roundtrip_center_m": roundtrip_xy,
            "radius_uv": radius_uv,
            "roundtrip_radius_m": roundtrip_radius_m,
        }
        if item["input_mode"] == "metric":
            item["radius_input_m"] = float(obstacle.get("input_radius", float("nan")))
            input_center = np.asarray(item["input_center"], dtype=float)
            roundtrip_pass = roundtrip_pass and bool(
                np.allclose(input_center, np.asarray(roundtrip_xy), rtol=0.0, atol=1e-9)
            )
            roundtrip_pass = roundtrip_pass and bool(
                np.isclose(
                    float(item["radius_input_m"]),
                    float(roundtrip_radius_m),
                    rtol=0.0,
                    atol=1e-9,
                )
            )
        obstacle_payload.append(item)

    return {
        "algorithm_coordinate_mode": mapper.algorithm_coordinate_mode,
        "algorithm_frame": "normalized_uv",
        "metric_frame": mapper.frame_id,
        "plane_bounds_m": {
            "x": [mapper.x_min, mapper.x_max],
            "y": [mapper.y_min, mapper.y_max],
        },
        "obstacles": obstacle_payload,
        "coordinate_roundtrip_pass": bool(roundtrip_pass),
    }
