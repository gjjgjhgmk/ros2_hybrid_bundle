import numpy as np

from plane_hybrid_planner.planner_2d import (
    generate_nominal_path,
    path_collision_details,
    path_min_clearance,
    rrt_connect,
)


MIDDLE_OBSTACLE = [{"type": "circle", "center": [0.5, 0.5], "radius": 0.12}]


def test_nominal_types_preserve_endpoints():
    for spec in (
        {"type": "line", "start": [0.1, 0.5], "goal": [0.9, 0.5], "num_points": 25},
        {
            "type": "sine",
            "start": [0.1, 0.5],
            "goal": [0.9, 0.5],
            "amplitude": 0.05,
            "num_points": 25,
        },
        {"type": "polyline", "points": [[0.1, 0.5], [0.5, 0.7], [0.9, 0.5]], "num_points": 25},
    ):
        path = generate_nominal_path(spec)
        assert path.shape == (25, 2)
        assert np.allclose(path[0], [0.1, 0.5])
        assert np.allclose(path[-1], [0.9, 0.5])


def test_rrt_direct_and_invalid_start():
    direct = rrt_connect([0.1, 0.1], [0.9, 0.1], MIDDLE_OBSTACLE)
    assert direct["ok"] and direct["stop_reason"] == "direct"
    invalid = rrt_connect([0.5, 0.5], [0.9, 0.1], MIDDLE_OBSTACLE)
    assert not invalid["ok"]
    assert invalid["path"] == []
    assert invalid["stop_reason"] == "collision_start"


def test_rrt_middle_obstacle_has_valid_states_and_edges():
    nominal = generate_nominal_path(
        {"type": "line", "start": [0.1, 0.5], "goal": [0.9, 0.5], "num_points": 120}
    )
    assert path_collision_details(nominal, MIDDLE_OBSTACLE)["collision_count"] > 0
    result = rrt_connect(
        nominal[0], nominal[-1], MIDDLE_OBSTACLE, step_size=0.04, max_iter=1000, rng_seed=42, margin=0.02
    )
    assert result["ok"]
    path = np.asarray(result["path"])
    assert np.allclose(path[0], nominal[0])
    assert np.allclose(path[-1], nominal[-1])
    assert path_collision_details(path, MIDDLE_OBSTACLE, margin=0.02)["collision_count"] == 0
    assert path_min_clearance(path, MIDDLE_OBSTACLE) >= 0.02

