import numpy as np
import pytest


rclpy = pytest.importorskip("rclpy")
_ = rclpy

from intent_hybrid_planner.intent_hybrid_planner_node import IntentHybridPlannerNode


def _bare_node() -> IntentHybridPlannerNode:
    node = object.__new__(IntentHybridPlannerNode)
    node.moveit_group_name = "manipulator"
    node._moveit_update_call_style = None
    node.runtime_backend = "python"
    node.execution_mode = "offline"
    node._cpp_runtime_clients_ready = False
    node.cpp_check_states_client = None
    node.cpp_dispatch_client = None
    node.cpp_publish_markers_client = None
    node._cpp_rrt_fallback_checker = None
    node._cpp_rrt_fallback_edge_checker = None
    node._cpp_rrt_fallback_warned = False
    return node


def test_parse_analytic_obstacles_json_valid():
    node = _bare_node()
    node.analytic_obstacles_json = "[[0.1, 0.2, 0.3, 0.05], [0.4, 0.5, 0.6, 0.08]]"

    out = node._parse_analytic_obstacles_param()
    assert len(out) == 2
    assert np.allclose(out[0], np.array([0.1, 0.2, 0.3, 0.05], dtype=float))
    assert np.allclose(out[1], np.array([0.4, 0.5, 0.6, 0.08], dtype=float))


def test_parse_analytic_obstacles_json_invalid_fallback():
    node = _bare_node()
    node.analytic_obstacles_json = "not-json"
    node.get_logger = lambda: type("L", (), {"warn": lambda self, msg: None})()

    out = node._parse_analytic_obstacles_param()
    assert len(out) == 2
    assert out[0].shape == (4,)
    assert out[1].shape == (4,)


def test_set_moveit_state_positions_update_fallback():
    node = _bare_node()

    calls = {"set": 0, "update_true": 0, "update": 0}

    class FakeState:
        def set_joint_group_positions(self, group, values):
            calls["set"] += 1
            assert group == "manipulator"
            assert values == [1.0, 2.0, 3.0]

        def update(self, force=None):
            if force is True:
                calls["update_true"] += 1
                raise TypeError("update(True) not supported")
            calls["update"] += 1

    state = FakeState()
    node._set_moveit_state_positions(state, np.array([1.0, 2.0, 3.0], dtype=float))
    node._set_moveit_state_positions(state, np.array([1.0, 2.0, 3.0], dtype=float))

    assert calls["set"] == 2
    assert calls["update_true"] == 1
    assert calls["update"] == 2
    assert node._moveit_update_call_style == "update"


def test_cpp_runtime_offline_gate():
    node = _bare_node()
    node.runtime_backend = "cpp_bridge"
    node.execution_mode = "offline"
    node._cpp_runtime_clients_ready = True
    node.cpp_check_states_client = object()
    node.cpp_dispatch_client = object()
    node.cpp_publish_markers_client = object()

    assert node._cpp_runtime_requested() is True
    assert node._cpp_runtime_ready() is True
    assert node._use_cpp_runtime_for_offline() is True

    node.execution_mode = "online"
    assert node._use_cpp_runtime_for_offline() is False

    node.execution_mode = "offline"
    node.cpp_dispatch_client = None
    assert node._cpp_runtime_ready() is False
    assert node._use_cpp_runtime_for_offline() is False


def test_cpp_collision_checker_single_state_fallback_checker():
    node = _bare_node()
    node._check_states_batch_cpp = lambda _q: None
    node.get_logger = lambda: type("L", (), {"warn": lambda self, _msg: None})()

    calls = {"n": 0}

    def fallback_checker(state):
        calls["n"] += 1
        assert state.shape == (3,)
        return True

    node._cpp_rrt_fallback_checker = fallback_checker
    out = node._cpp_collision_checker_single_state(np.array([1.0, 2.0, 3.0], dtype=float))
    assert out is True
    assert calls["n"] == 1
    assert node._cpp_rrt_fallback_warned is True


def test_cpp_collision_checker_edge_fallback_checker():
    node = _bare_node()
    node._check_states_batch_cpp = lambda _q: None
    node.get_logger = lambda: type("L", (), {"warn": lambda self, _msg: None})()

    calls = {"n": 0}

    def fallback_checker(state):
        calls["n"] += 1
        assert state.shape == (2,)
        return True

    node._cpp_rrt_fallback_checker = fallback_checker
    out = node._cpp_collision_checker_edge(
        np.array([0.0, 0.0], dtype=float),
        np.array([1.0, 0.0], dtype=float),
        5,
    )
    assert out is True
    assert calls["n"] == 5
