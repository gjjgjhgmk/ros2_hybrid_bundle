import numpy as np

from intent_hybrid_planner import fmp_core


def _nominal_trajectory():
    n = 150
    t = np.linspace(0.0, 10.0, n)
    start_q = np.array([-0.8, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=float)
    end_q = np.array([0.8, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=float)
    blend = np.linspace(0.0, 1.0, n, dtype=float)
    traj = start_q[:, None] + (end_q - start_q)[:, None] * blend[None, :]
    return traj, t


def test_no_via_shape_and_finite():
    traj, t = _nominal_trajectory()
    model = fmp_core.train_fmp_model(traj, t, N_C=20, alpha=0.1)

    out = fmp_core.modulate_trajectory(
        fmp_model=model,
        demo_traj=traj,
        time_axis=t,
        via_points=np.empty((traj.shape[0], 0), dtype=float),
        via_times=np.empty((0,), dtype=float),
    )

    assert out.shape == traj.shape
    assert np.isfinite(out).all()


def test_single_via_has_local_effect():
    traj, t = _nominal_trajectory()
    model = fmp_core.train_fmp_model(traj, t, N_C=20, alpha=0.1)

    idx = 75
    via_points = traj[:, [idx]].copy()
    via_points[0, 0] += 0.15
    via_times = np.array([t[idx]], dtype=float)

    out = fmp_core.modulate_trajectory(
        fmp_model=model,
        demo_traj=traj,
        time_axis=t,
        via_points=via_points,
        via_times=via_times,
    )

    assert out.shape == traj.shape
    assert np.isfinite(out).all()
    assert abs(float(out[0, idx] - traj[0, idx])) > 1e-4


def test_multi_via_repeat_cluster_path_stable():
    traj, t = _nominal_trajectory()
    model = fmp_core.train_fmp_model(traj, t, N_C=20, alpha=0.1)

    idxs = [60, 61, 62]
    via_points = traj[:, idxs].copy()
    via_points[0, :] += np.array([0.12, 0.10, 0.08], dtype=float)
    via_points[2, :] += np.array([0.05, -0.04, 0.03], dtype=float)
    via_times = t[idxs]

    out = fmp_core.modulate_trajectory(
        fmp_model=model,
        demo_traj=traj,
        time_axis=t,
        via_points=via_points,
        via_times=via_times,
    )

    assert out.shape == traj.shape
    assert np.isfinite(out).all()


def test_via_time_raw_and_scaled_consistency():
    traj, t = _nominal_trajectory()
    model = fmp_core.train_fmp_model(traj, t, N_C=20, alpha=0.1)

    idxs = [45, 95]
    via_points = traj[:, idxs].copy()
    via_points[0, 0] += 0.14
    via_points[2, 1] -= 0.09

    via_times_raw = t[idxs]
    via_times_scaled = via_times_raw * float(model["demo_dura"])

    out_raw = fmp_core.modulate_trajectory(
        fmp_model=model,
        demo_traj=traj,
        time_axis=t,
        via_points=via_points,
        via_times=via_times_raw,
    )
    out_scaled = fmp_core.modulate_trajectory(
        fmp_model=model,
        demo_traj=traj,
        time_axis=t,
        via_points=via_points,
        via_times=via_times_scaled,
    )

    assert out_raw.shape == traj.shape
    assert out_scaled.shape == traj.shape
    assert np.isfinite(out_raw).all()
    assert np.isfinite(out_scaled).all()
    assert np.max(np.abs(out_raw - out_scaled)) <= 1e-8
