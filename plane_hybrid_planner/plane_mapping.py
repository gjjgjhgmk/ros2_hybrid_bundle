"""Map normalized table-plane coordinates to Cartesian poses and scene objects."""

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class PlaneMapper:
    frame_id: str
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    fixed_z: float
    orientation_xyzw: Tuple[float, float, float, float]
    clamp_uv: bool = True

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "PlaneMapper":
        plane = config.get("plane", {})
        orientation = np.asarray(
            config.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=float
        ).reshape(-1)
        if orientation.size != 4 or not np.all(np.isfinite(orientation)):
            raise ValueError("orientation_xyzw must contain four finite values")
        norm = float(np.linalg.norm(orientation))
        if norm <= 1e-12:
            raise ValueError("orientation_xyzw must be a non-zero quaternion")
        orientation = orientation / norm

        x_min = float(plane["x_min"])
        x_max = float(plane["x_max"])
        y_min = float(plane["y_min"])
        y_max = float(plane["y_max"])
        if not x_max > x_min or not y_max > y_min:
            raise ValueError("plane x/y ranges must have max > min")

        return cls(
            frame_id=str(plane.get("frame_id", "world")),
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            fixed_z=float(plane["z"]),
            orientation_xyzw=tuple(float(x) for x in orientation),
            clamp_uv=bool(config.get("waypoints", {}).get("clamp_uv", True)),
        )

    def _validate_uv(self, u: float, v: float) -> Tuple[float, float]:
        uv = np.asarray([u, v], dtype=float)
        if not np.all(np.isfinite(uv)):
            raise ValueError("u and v must be finite")
        if self.clamp_uv:
            uv = np.clip(uv, 0.0, 1.0)
        elif np.any(uv < 0.0) or np.any(uv > 1.0):
            raise ValueError(f"uv coordinate outside [0, 1]: {uv.tolist()}")
        return float(uv[0]), float(uv[1])

    def uv_to_xyz(self, u: float, v: float) -> Tuple[float, float, float]:
        u, v = self._validate_uv(u, v)
        x = self.x_min + u * (self.x_max - self.x_min)
        y = self.y_min + v * (self.y_max - self.y_min)
        return float(x), float(y), float(self.fixed_z)

    @property
    def x_span(self) -> float:
        return float(self.x_max - self.x_min)

    @property
    def y_span(self) -> float:
        return float(self.y_max - self.y_min)

    def uv_radius_to_metric(self, radius_uv: float, mode: str = "min") -> float:
        radius = float(radius_uv)
        if not np.isfinite(radius) or radius < 0.0:
            raise ValueError("radius_uv must be a finite non-negative value")
        normalized_mode = str(mode).strip().lower()
        if normalized_mode == "min":
            scale = min(self.x_span, self.y_span)
        elif normalized_mode == "max":
            scale = max(self.x_span, self.y_span)
        elif normalized_mode == "mean":
            scale = 0.5 * (self.x_span + self.y_span)
        elif normalized_mode == "x":
            scale = self.x_span
        elif normalized_mode == "y":
            scale = self.y_span
        else:
            raise ValueError("radius scale mode must be one of min/max/mean/x/y")
        return float(radius * scale)

    def uv_circle_to_cylinder(
        self,
        obstacle: Dict[str, Any],
        *,
        height: float,
        center_z: float | None = None,
        radius_scale_mode: str = "min",
        frame_id: str | None = None,
        obstacle_id: str = "",
    ) -> Dict[str, Any]:
        center = np.asarray(obstacle["center"], dtype=float).reshape(-1)
        if center.size != 2 or not np.all(np.isfinite(center)):
            raise ValueError("obstacle center must contain two finite values")
        x, y, z_default = self.uv_to_xyz(float(center[0]), float(center[1]))
        z = float(z_default if center_z is None else center_z)
        return {
            "id": str(obstacle_id or obstacle.get("id", "")),
            "frame_id": str(frame_id or self.frame_id),
            "position": [x, y, z],
            "radius": self.uv_radius_to_metric(
                float(obstacle["radius"]), mode=radius_scale_mode
            ),
            "height": float(height),
        }

    def uv_path_to_cart_waypoints(
        self, path_uv: Iterable[Sequence[float]]
    ) -> List[Dict[str, Any]]:
        path = np.asarray(list(path_uv), dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] == 0:
            raise ValueError("path_uv must be a non-empty array shaped (N, 2)")
        waypoints: List[Dict[str, Any]] = []
        for u, v in path:
            waypoints.append(
                {
                    "frame_id": self.frame_id,
                    "position": list(self.uv_to_xyz(float(u), float(v))),
                    "orientation": list(self.orientation_xyzw),
                }
            )
        return waypoints
