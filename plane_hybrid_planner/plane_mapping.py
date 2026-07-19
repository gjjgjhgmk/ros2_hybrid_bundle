"""Map normalized table-plane coordinates to metric Cartesian poses."""

from dataclasses import dataclass
import warnings
from typing import Any, Dict, Iterable, List, Literal, Sequence, Tuple

import numpy as np

RadiusScaleMode = Literal["min", "max", "mean", "x", "y"]


@dataclass(frozen=True)
class PlaneMapper:
    frame_id: str
    algorithm_coordinate_mode: str
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
        algorithm_coordinate_mode = str(
            plane.get("algorithm_coordinate_mode", "normalized")
        ).strip().lower()
        if algorithm_coordinate_mode != "normalized":
            raise ValueError("plane.algorithm_coordinate_mode must be 'normalized'")
        if "coordinate_mode" in plane or "coordinate_mode" in config:
            legacy_mode = str(
                plane.get("coordinate_mode", config.get("coordinate_mode"))
            ).strip().lower()
            if legacy_mode not in {"normalized", "metric"}:
                raise ValueError("legacy coordinate_mode must be 'normalized' or 'metric'")
            warnings.warn(
                "coordinate_mode is deprecated; the 2D algorithm always uses "
                "normalized UV. Use plane.algorithm_coordinate_mode and "
                "obstacle_input.coordinate_mode instead.",
                FutureWarning,
                stacklevel=2,
            )

        return cls(
            frame_id=str(plane.get("frame_id", "world")),
            algorithm_coordinate_mode=algorithm_coordinate_mode,
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            fixed_z=float(plane["z"]),
            orientation_xyzw=tuple(float(x) for x in orientation),
            clamp_uv=bool(config.get("waypoints", {}).get("clamp_uv", True)),
        )

    @property
    def coordinate_mode(self) -> str:
        """Backward-compatible alias for the fixed algorithm coordinate mode."""
        return self.algorithm_coordinate_mode

    def _validate_uv(
        self,
        u: float,
        v: float,
        *,
        clamp: bool | None = None,
    ) -> Tuple[float, float]:
        uv = np.asarray([u, v], dtype=float)
        if not np.all(np.isfinite(uv)):
            raise ValueError("u and v must be finite")
        should_clamp = self.clamp_uv if clamp is None else bool(clamp)
        if should_clamp:
            uv = np.clip(uv, 0.0, 1.0)
        elif np.any(uv < 0.0) or np.any(uv > 1.0):
            raise ValueError(f"uv coordinate outside [0, 1]: {uv.tolist()}")
        return float(uv[0]), float(uv[1])

    def uv_to_xy(self, u: float, v: float, *, clamp: bool | None = None) -> Tuple[float, float]:
        u, v = self._validate_uv(u, v, clamp=clamp)
        x = self.x_min + u * (self.x_max - self.x_min)
        y = self.y_min + v * (self.y_max - self.y_min)
        return float(x), float(y)

    def xy_to_uv(self, x: float, y: float, *, strict: bool = True) -> Tuple[float, float]:
        xy = np.asarray([x, y], dtype=float)
        if not np.all(np.isfinite(xy)):
            raise ValueError("x and y must be finite")
        u = (float(xy[0]) - self.x_min) / self.x_span
        v = (float(xy[1]) - self.y_min) / self.y_span
        if strict and (u < 0.0 or u > 1.0 or v < 0.0 or v > 1.0):
            raise ValueError(
                "metric XY outside configured plane: "
                f"input_xy={[float(xy[0]), float(xy[1])]}, "
                f"x_range={[self.x_min, self.x_max]}, "
                f"y_range={[self.y_min, self.y_max]}, "
                f"computed_uv={[u, v]}, frame_id={self.frame_id}"
            )
        if not strict:
            u, v = self._validate_uv(u, v, clamp=True)
        return float(u), float(v)

    def uv_to_xyz(
        self,
        u: float,
        v: float,
        z: float | None = None,
        *,
        clamp: bool | None = None,
    ) -> Tuple[float, float, float]:
        x, y = self.uv_to_xy(u, v, clamp=clamp)
        return float(x), float(y), float(self.fixed_z if z is None else z)

    @property
    def x_span(self) -> float:
        return float(self.x_max - self.x_min)

    @property
    def y_span(self) -> float:
        return float(self.y_max - self.y_min)

    def _radius_scale(self, mode: str) -> float:
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
        if not np.isclose(self.x_span, self.y_span, rtol=0.0, atol=1e-12):
            warnings.warn(
                "x_span and y_span differ; a circular UV obstacle maps to an "
                "ellipse in metric XY. radius_scale_mode selects an approximation.",
                RuntimeWarning,
                stacklevel=3,
            )
        return float(scale)

    def uv_radius_to_metric(self, radius_uv: float, mode: str = "min") -> float:
        radius = float(radius_uv)
        if not np.isfinite(radius) or radius < 0.0:
            raise ValueError("radius_uv must be a finite non-negative value")
        scale = self._radius_scale(mode)
        return float(radius * scale)

    def metric_radius_to_uv(self, radius_m: float, mode: str = "min") -> float:
        radius = float(radius_m)
        if not np.isfinite(radius) or radius < 0.0:
            raise ValueError("radius_m must be a finite non-negative value")
        scale = self._radius_scale(mode)
        return float(radius / scale)

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
        x, y, z_default = self.uv_to_xyz(float(center[0]), float(center[1]), clamp=False)
        z = float(z_default if center_z is None else center_z)
        height_value = float(height)
        return {
            "id": str(obstacle_id or obstacle.get("id", "")),
            "frame_id": str(frame_id or self.frame_id),
            "position": [x, y, z],
            "radius": self.uv_radius_to_metric(
                float(obstacle["radius"]), mode=radius_scale_mode
            ),
            "height": height_value,
            "bottom_z": float(z - height_value / 2.0),
            "top_z": float(z + height_value / 2.0),
        }

    def uv_path_to_cart_waypoints(
        self,
        path_uv: Iterable[Sequence[float]],
        *,
        z: float | None = None,
        orientation_xyzw: Sequence[float] | None = None,
    ) -> List[Dict[str, Any]]:
        path = np.asarray(list(path_uv), dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] == 0:
            raise ValueError("path_uv must be a non-empty array shaped (N, 2)")
        orientation = self.orientation_xyzw
        if orientation_xyzw is not None:
            orientation_arr = np.asarray(orientation_xyzw, dtype=float).reshape(-1)
            if orientation_arr.size != 4 or not np.all(np.isfinite(orientation_arr)):
                raise ValueError("orientation_xyzw must contain four finite values")
            norm = float(np.linalg.norm(orientation_arr))
            if norm <= 1e-12:
                raise ValueError("orientation_xyzw must be a non-zero quaternion")
            orientation = tuple(float(value) for value in orientation_arr / norm)
        waypoints: List[Dict[str, Any]] = []
        for u, v in path:
            waypoints.append(
                {
                    "frame_id": self.frame_id,
                    "position": list(self.uv_to_xyz(float(u), float(v), z=z)),
                    "orientation": list(orientation),
                }
            )
        return waypoints
