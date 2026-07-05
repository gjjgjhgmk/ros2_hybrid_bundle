import csv
import json

import numpy as np

from plane_hybrid_planner.evaluate_2d import build_evaluation, write_evaluation


def test_evaluation_and_export(tmp_path):
    nominal = np.column_stack((np.linspace(0.1, 0.9, 60), np.full(60, 0.5)))
    modulated = nominal.copy()
    modulated[:, 1] += 0.22 * np.sin(np.linspace(0.0, np.pi, 60))
    obstacle = [{"type": "circle", "center": [0.5, 0.5], "radius": 0.1}]
    result = build_evaluation(
        scenario_name="unit",
        group_name="left_arm",
        nominal_path=nominal,
        modulated_path=modulated,
        obstacles=obstacle,
        safety_margin=0.01,
        rrt_result={"stop_reason": "success", "iter_used": 7, "collision_queries": 22},
        cart_waypoint_count=30,
        ur_move_result={
            "available": False,
            "planning_success": False,
            "execution_id": "",
            "transport": "compatible_zmq",
        },
    )
    assert result["nominal_collision_count"] > 0
    assert result["modulated_collision_count"] == 0
    assert result["avoidance_success"]
    assert result["failure_reason"] == "ur_move_unavailable"
    assert result["result_interpretable"]

    write_evaluation(result, tmp_path)
    assert json.loads((tmp_path / "result.json").read_text())["scenario_name"] == "unit"
    with (tmp_path / "result.csv").open(newline="") as handle:
        assert next(csv.DictReader(handle))["group_name"] == "left_arm"


def test_execution_failure_is_not_reported_as_planning_failure():
    nominal = np.column_stack((np.linspace(0.1, 0.9, 30), np.full(30, 0.5)))
    result = build_evaluation(
        scenario_name="execute_fail",
        group_name="left_arm",
        nominal_path=nominal,
        modulated_path=nominal,
        obstacles=[],
        safety_margin=0.0,
        rrt_result={"stop_reason": "direct", "iter_used": 0, "collision_queries": 0},
        cart_waypoint_count=30,
        ur_move_result={
            "available": True,
            "planning_success": True,
            "execution_requested": True,
            "execution_success": False,
            "execution_id": "",
            "transport": "workspace_client",
            "error": "Some trajectories failed to execute",
            "error_kind": "moveit_execution_failed",
        },
    )
    assert result["moveit_plan_success"] is True
    assert result["execution_success"] is False
    assert result["failure_reason"] == "moveit_execution_failed"
