import numpy as np

from intent_hybrid_planner.intent_biased_rrt import IntentBiasedRRT


def test_edge_checker_callback_invoked_and_blocks_edges():
    calls = {"edge": 0}

    def state_checker(_state):
        return True

    def edge_checker(_p1, _p2, _samples):
        calls["edge"] += 1
        return False

    planner = IntentBiasedRRT(
        collision_checker_fn=state_checker,
        edge_checker_fn=edge_checker,
        step_size=0.2,
        max_iter=20,
        timeout_sec=0.02,
        max_edge_samples=4,
        rng_seed=1,
    )

    start = np.array([0.0, 0.0], dtype=float)
    goal = np.array([1.0, 0.0], dtype=float)
    intent = np.array([[0.0, 0.0], [0.5, 0.0], [1.0, 0.0]], dtype=float)

    via_points, via_times, _meta = planner.plan(start, goal, intent, 0.0, 1.0)

    assert calls["edge"] > 0
    assert via_points.shape[0] == 2
    assert via_times.shape[0] == via_points.shape[1]


def test_edge_checker_callback_can_allow_edges():
    calls = {"edge": 0}

    def state_checker(_state):
        return True

    def edge_checker(_p1, _p2, _samples):
        calls["edge"] += 1
        return True

    planner = IntentBiasedRRT(
        collision_checker_fn=state_checker,
        edge_checker_fn=edge_checker,
        step_size=0.3,
        max_iter=40,
        timeout_sec=0.03,
        max_edge_samples=5,
        rng_seed=2,
    )

    start = np.array([0.0, 0.0], dtype=float)
    goal = np.array([1.0, 0.2], dtype=float)
    intent = np.array([[0.0, 0.0], [0.5, 0.1], [1.0, 0.2]], dtype=float)

    via_points, via_times, _meta = planner.plan(start, goal, intent, 0.0, 2.0)

    assert calls["edge"] > 0
    assert via_points.shape[0] == 2
    assert via_points.shape[1] >= 2
    assert np.all(np.diff(via_times) >= 0.0)
