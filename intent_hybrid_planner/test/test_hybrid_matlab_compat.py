import numpy as np

from intent_hybrid_planner.hybrid_matlab_compat import (
    auto_select_refine_budget,
    choose_refine_budget_pareto,
    densify_path_to_vias,
    extract_danger_segments,
)
from intent_hybrid_planner.intent_biased_rrt import IntentBiasedRRT


def test_extract_danger_segments_gap_and_pad():
    danger = [10, 11, 12, 30, 42, 43]
    segs = extract_danger_segments(danger, 100, gap=10, pad=4)
    assert segs == [(6, 16), (26, 34), (38, 47)]


def test_densify_path_to_vias_shapes_and_trim():
    path = np.array(
        [
            [0.0, 1.0, 2.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    via_points, via_times = densify_path_to_vias(
        path,
        t_start=0.0,
        t_end=2.0,
        interp_dist=0.2,
        via_trim_sec=0.1,
    )
    assert via_points.shape[0] == 3
    assert via_points.shape[1] == via_times.size
    assert np.all(np.diff(via_times) > 0.0)
    assert float(via_times[0]) > 0.0
    assert float(via_times[-1]) < 2.0


def test_choose_refine_budget_pareto_prefers_gain_over_delta():
    budgets = [25, 50, 100, 150]
    gains = [0.1, 0.15, 0.16, 0.17]
    deltas = [1.0, 1.2, 5.0, 10.0]
    chosen = choose_refine_budget_pareto(budgets, gains, deltas)
    assert chosen in budgets
    assert chosen == 50


def test_auto_select_refine_budget_with_mock_plans():
    base = np.array(
        [
            [0.0, 1.0, 2.0],
            [0.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    def eval_plan(budget: int):
        scale = 1.0 - min(budget, 150) / 500.0
        refine = base.copy()
        refine[0, 1] = refine[0, 1] * scale
        return {
            "path_first": base,
            "path_refine": refine,
            "t_first": 0.05,
            "t_refine": 0.05 + budget / 1000.0,
        }

    best, logs = auto_select_refine_budget([25, 50, 100, 150], eval_plan)
    assert best in [25, 50, 100, 150]
    assert len(logs) == 4


def test_plan_detailed_monotonic_times_and_reason():
    planner = IntentBiasedRRT(
        collision_checker_fn=lambda _: True,
        step_size=0.2,
        r_neighbor=0.4,
        max_iter=200,
        timeout_sec=0.2,
        rng_seed=7,
    )
    start = np.array([0.0, 0.0], dtype=float)
    goal = np.array([1.0, 1.0], dtype=float)
    intent = np.vstack(
        [
            np.linspace(start[0], goal[0], 20),
            np.linspace(start[1], goal[1], 20),
        ]
    ).T

    out = planner.plan_detailed(
        start=start,
        goal=goal,
        intent_path=intent,
        t_start=0.0,
        t_end=1.0,
        refine_budget=20,
    )
    assert out["stop_reason"] in {"converged", "budget_hit", "max_iter_guard", "timeout", "failed"}
    if np.isfinite(out["t_first"]):
        assert out["t_first"] <= out["t_refine"] <= out["t_conv"]
    via_t = np.asarray(out["via_times"], dtype=float)
    if via_t.size > 1:
        assert np.all(np.diff(via_t) > 0.0)
