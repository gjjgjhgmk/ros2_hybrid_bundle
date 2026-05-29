import numpy as np
import pytest


rclpy = pytest.importorskip("rclpy")
_ = rclpy

from intent_hybrid_planner.intent_hybrid_evaluator import (
    compute_joint_metrics,
    normalize_traj_dofxn,
    normalize_xyz_nx3,
)


def test_normalize_traj_dofxn_accepts_both_layouts():
    dof = 3
    arr_dofxn = np.array(
        [
            [0.0, 1.0, 2.0, 3.0],
            [4.0, 5.0, 6.0, 7.0],
            [8.0, 9.0, 10.0, 11.0],
        ],
        dtype=float,
    )
    arr_nxdof = arr_dofxn.T

    out1 = normalize_traj_dofxn(arr_dofxn, dof, "a")
    out2 = normalize_traj_dofxn(arr_nxdof, dof, "b")

    assert out1.shape == (3, 4)
    assert out2.shape == (3, 4)
    assert np.allclose(out1, arr_dofxn)
    assert np.allclose(out2, arr_dofxn)


def test_normalize_xyz_nx3_accepts_both_layouts():
    xyz_nx3 = np.array([[0.0, 1.0, 2.0], [3.0, 4.0, 5.0]], dtype=float)
    xyz_3xn = xyz_nx3.T

    out1 = normalize_xyz_nx3(xyz_nx3, "x")
    out2 = normalize_xyz_nx3(xyz_3xn, "y")

    assert out1.shape == (2, 3)
    assert out2.shape == (2, 3)
    assert np.allclose(out1, xyz_nx3)
    assert np.allclose(out2, xyz_nx3)


def test_compute_joint_metrics_basic():
    traj = np.array(
        [
            [0.0, 0.1, 0.2, 0.3],
            [0.0, -0.1, -0.2, -0.3],
        ],
        dtype=float,
    )
    m = compute_joint_metrics(traj, dt=0.1)
    assert m["joint_path_length"] > 0.0
    assert m["max_joint_delta"] > 0.0
    assert m["estimated_max_velocity"] > 0.0
    assert m["estimated_max_acceleration"] >= 0.0
    assert m["joint_jerk_integral"] >= 0.0
