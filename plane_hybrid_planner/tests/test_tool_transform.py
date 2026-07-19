import numpy as np
import pytest

from plane_hybrid_planner.tool_transform import (
    ee_pose_to_tip_pose,
    tip_pose_to_ee_pose,
    transform_matrix,
)


def _assert_pose_close(actual, expected, tol=1e-9):
    assert actual["frame_id"] == expected["frame_id"]
    assert actual["position"] == pytest.approx(expected["position"], abs=tol)
    a = np.asarray(actual["orientation"], dtype=float)
    e = np.asarray(expected["orientation"], dtype=float)
    assert np.allclose(a, e, atol=tol) or np.allclose(a, -e, atol=tol)


def test_tip_pose_to_ee_pose_roundtrip_with_rotation_and_lateral_offset():
    ee_to_tip = transform_matrix(
        [0.012, -0.105, -0.109],
        [-0.3826834323650898, 0.0, 0.0, 0.9238795325112867],
    )
    tip_pose = {
        "frame_id": "left_interface_link",
        "position": [0.5504, 0.1806, -0.0082],
        "orientation": [0.3826834323650898, 0.0, 0.0, 0.9238795325112867],
    }

    ee_pose = tip_pose_to_ee_pose(tip_pose, ee_to_tip)
    recovered_tip_pose = ee_pose_to_tip_pose(ee_pose, ee_to_tip)

    _assert_pose_close(recovered_tip_pose, tip_pose)
