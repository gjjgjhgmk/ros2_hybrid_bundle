import numpy as np

from plane_hybrid_planner.matlab_hybrid_2d import (
    MATLAB_DEFAULTS,
    apply_local_corner_smoothing,
    danger_indices_matlab,
    resolve_corner_window_samples,
    run_matlab_hybrid,
    split_danger_segments,
)
from plane_hybrid_planner.matlab_rrt_star_2d import MatlabRRTStarConfig
from plane_hybrid_planner.planner_2d import generate_nominal_path


OBSTACLE = [{"type": "circle", "center": [0.5, 0.5], "radius": 0.12}]


def _nominal():
    return generate_nominal_path(
        {"type": "line", "start": [0.1, 0.5], "goal": [0.9, 0.5], "num_points": 150}
    )


def test_matlab_numeric_defaults_are_preserved():
    rrt = MatlabRRTStarConfig()
    assert MATLAB_DEFAULTS["demo_len"] == 150
    assert MATLAB_DEFAULTS["demo_dt"] == 0.1
    assert MATLAB_DEFAULTS["online_dt"] == 0.05
    assert MATLAB_DEFAULTS["interp_dist"] == 0.5
    assert rrt.step_size == 1.5
    assert rrt.r_neighbor == 4.0
    assert rrt.max_iter == 6000
    assert rrt.refine_budget == 100
    assert rrt.p_intent_pre == 0.55


def test_matlab_danger_segmentation_uses_gap_greater_than_ten():
    assert split_danger_segments([1, 3, 13, 24], gap=10) == [[1, 3, 13], [24]]
    danger = danger_indices_matlab(_nominal(), OBSTACLE, safe_margin=0.2)
    assert danger
    assert max(danger) - min(danger) > 10


def test_corner_window_modes_are_explicit_and_integer():
    time_axis = np.arange(0.05, 0.55, 0.05)
    assert resolve_corner_window_samples("time", 0.1, time_axis, 10)[0] == 2
    assert resolve_corner_window_samples("samples", 3, time_axis, 10)[0] == 3
    assert resolve_corner_window_samples("ratio", 0.2, time_axis, 11)[0] == 2


def test_corner_smoothing_records_default_time_interpretation():
    path = np.asarray([[0.0, 0.0], [0.3, 0.0], [0.3, 0.3], [0.6, 0.3], [1.0, 0.3]])
    smoothed, metadata = apply_local_corner_smoothing(
        path, np.arange(0.05, 0.30, 0.05), window_mode="time", window_value=0.1
    )
    assert metadata["window_mode"] == "time"
    assert metadata["resolved_window_samples"] == 2
    assert metadata["sharp_point_count"] > 0
    assert metadata["applied"]
    assert np.allclose(smoothed[[0, -1]], path[[0, -1]])


def test_matlab_pipeline_returns_safe_300_point_fmp_path():
    result = run_matlab_hybrid(_nominal(), OBSTACLE, environment_seed=2025)
    assert result["ok"]
    assert result["danger_count"] > 0
    assert result["segment_results"]
    segment = result["segment_results"][0]
    assert segment["refine_expand_count"] >= 100
    assert len(segment["path_refine"]) >= 2
    assert result["modulated_path"].shape == (300, 2)
    assert result["via_points"].shape[0] > 0
    metadata = result["corner_smoothing_metadata"]
    assert metadata["window_mode"] == "time"
    assert metadata["configured_window_value"] == 0.1
    assert metadata["resolved_window_samples"] == 2
    assert metadata["max_displacement_uv"] > 0.0
