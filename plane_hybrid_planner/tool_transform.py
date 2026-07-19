"""Rigid transforms between wrist and drawing-tool tip task poses."""

from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Sequence, Tuple

import numpy as np


def _vec3(value: Sequence[float], name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != 3 or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain three finite values")
    return arr


def normalize_quaternion_xyzw(value: Sequence[float]) -> np.ndarray:
    quat = np.asarray(value, dtype=float).reshape(-1)
    if quat.size != 4 or not np.all(np.isfinite(quat)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError("quaternion must be non-zero")
    return quat / norm


def quaternion_xyzw_to_matrix(value: Sequence[float]) -> np.ndarray:
    x, y, z, w = normalize_quaternion_xyzw(value)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion_xyzw(matrix: np.ndarray) -> np.ndarray:
    rot = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = np.asarray(
            [
                (rot[2, 1] - rot[1, 2]) / s,
                (rot[0, 2] - rot[2, 0]) / s,
                (rot[1, 0] - rot[0, 1]) / s,
                0.25 * s,
            ],
            dtype=float,
        )
    else:
        index = int(np.argmax(np.diag(rot)))
        if index == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            quat = np.asarray(
                [
                    0.25 * s,
                    (rot[0, 1] + rot[1, 0]) / s,
                    (rot[0, 2] + rot[2, 0]) / s,
                    (rot[2, 1] - rot[1, 2]) / s,
                ],
                dtype=float,
            )
        elif index == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            quat = np.asarray(
                [
                    (rot[0, 1] + rot[1, 0]) / s,
                    0.25 * s,
                    (rot[1, 2] + rot[2, 1]) / s,
                    (rot[0, 2] - rot[2, 0]) / s,
                ],
                dtype=float,
            )
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            quat = np.asarray(
                [
                    (rot[0, 2] + rot[2, 0]) / s,
                    (rot[1, 2] + rot[2, 1]) / s,
                    0.25 * s,
                    (rot[1, 0] - rot[0, 1]) / s,
                ],
                dtype=float,
            )
    return normalize_quaternion_xyzw(quat)


def transform_matrix(
    translation: Sequence[float],
    rotation_xyzw: Sequence[float],
) -> np.ndarray:
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = quaternion_xyzw_to_matrix(rotation_xyzw)
    mat[:3, 3] = _vec3(translation, "translation")
    return mat


def pose_to_matrix(pose: Dict[str, Any]) -> np.ndarray:
    return transform_matrix(pose["position"], pose["orientation"])


def matrix_to_pose(matrix: np.ndarray, *, frame_id: str) -> Dict[str, Any]:
    mat = np.asarray(matrix, dtype=float).reshape(4, 4)
    return {
        "frame_id": str(frame_id),
        "position": [float(value) for value in mat[:3, 3]],
        "orientation": [float(value) for value in matrix_to_quaternion_xyzw(mat[:3, :3])],
    }


def compose_transform(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    return np.asarray(first, dtype=float).reshape(4, 4) @ np.asarray(second, dtype=float).reshape(4, 4)


def invert_transform(transform: np.ndarray) -> np.ndarray:
    mat = np.asarray(transform, dtype=float).reshape(4, 4)
    inv = np.eye(4, dtype=float)
    inv[:3, :3] = mat[:3, :3].T
    inv[:3, 3] = -inv[:3, :3] @ mat[:3, 3]
    return inv


def tip_pose_to_ee_pose(
    tip_pose: Dict[str, Any],
    ee_to_tip: np.ndarray,
) -> Dict[str, Any]:
    frame_id = str(tip_pose["frame_id"])
    frame_to_tip = pose_to_matrix(tip_pose)
    frame_to_ee = compose_transform(frame_to_tip, invert_transform(ee_to_tip))
    return matrix_to_pose(frame_to_ee, frame_id=frame_id)


def ee_pose_to_tip_pose(
    ee_pose: Dict[str, Any],
    ee_to_tip: np.ndarray,
) -> Dict[str, Any]:
    frame_id = str(ee_pose["frame_id"])
    frame_to_ee = pose_to_matrix(ee_pose)
    frame_to_tip = compose_transform(frame_to_ee, ee_to_tip)
    return matrix_to_pose(frame_to_tip, frame_id=frame_id)


def transform_waypoints_tip_to_ee(
    tip_waypoints: Iterable[Dict[str, Any]],
    ee_to_tip: np.ndarray,
) -> List[Dict[str, Any]]:
    return [tip_pose_to_ee_pose(waypoint, ee_to_tip) for waypoint in tip_waypoints]


def transform_waypoints_ee_to_tip(
    ee_waypoints: Iterable[Dict[str, Any]],
    ee_to_tip: np.ndarray,
) -> List[Dict[str, Any]]:
    return [ee_pose_to_tip_pose(waypoint, ee_to_tip) for waypoint in ee_waypoints]


def ee_to_tip_from_config(config: Dict[str, Any]) -> Tuple[np.ndarray, Dict[str, Any]]:
    tool = dict(config.get("tool", {}) or {})
    ee_to_tip = dict(tool.get("ee_to_tip", {}) or {})
    translation = ee_to_tip.get("translation", [0.0, -0.10606601717798213, -0.10606601717798213])
    rotation = ee_to_tip.get("rotation_xyzw", [-0.3826834323650898, 0.0, 0.0, 0.9238795325112867])
    matrix = transform_matrix(translation, rotation)
    payload = {
        "parent_link": str(tool.get("parent_link", "left_ee_link")),
        "tip_link": str(tool.get("tip_link", "left_pen_tip_link")),
        "translation": [float(value) for value in translation],
        "rotation_xyzw": [float(value) for value in normalize_quaternion_xyzw(rotation)],
        "source": "config",
        "validated": False,
    }
    return matrix, payload
