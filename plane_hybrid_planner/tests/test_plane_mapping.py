import numpy as np
import pytest

from plane_hybrid_planner.obstacle_coordinates import normalize_obstacles_to_uv
from plane_hybrid_planner.plane_mapping import PlaneMapper


def _config(clamp=True):
    return {
        "plane": {
            "frame_id": "left_interface_link",
            "algorithm_coordinate_mode": "normalized",
            "x_min": 0.50,
            "x_max": 0.68,
            "y_min": 0.06,
            "y_max": 0.24,
            "z": 0.2,
        },
        "orientation_xyzw": [0.0, 2.0, 0.0, 0.0],
        "waypoints": {"clamp_uv": clamp},
    }


def test_uv_mapping_and_quaternion_normalization():
    mapper = PlaneMapper.from_config(_config())
    assert mapper.coordinate_mode == "normalized"
    assert mapper.algorithm_coordinate_mode == "normalized"
    assert np.allclose(mapper.uv_to_xyz(0.5, 0.25), [0.59, 0.105, 0.2])
    waypoint = mapper.uv_path_to_cart_waypoints([[0.0, 0.0], [1.0, 1.0]])[0]
    assert waypoint["frame_id"] == "left_interface_link"
    assert waypoint["orientation"] == [0.0, 1.0, 0.0, 0.0]


def test_uv_clamp_and_strict_mode():
    assert PlaneMapper.from_config(_config(True)).uv_to_xyz(-1.0, 2.0) == (0.5, 0.24, 0.2)
    with pytest.raises(ValueError):
        PlaneMapper.from_config(_config(False)).uv_to_xyz(-0.01, 0.5)


def test_metric_xy_to_uv_exact_example():
    mapper = PlaneMapper.from_config(_config())
    assert mapper.xy_to_uv(0.5504, 0.1806) == pytest.approx((0.28, 0.67), abs=1e-9)


def test_uv_to_metric_xy_exact_example():
    mapper = PlaneMapper.from_config(_config())
    assert mapper.uv_to_xy(0.28, 0.67) == pytest.approx((0.5504, 0.1806), abs=1e-9)


def test_xy_uv_roundtrip():
    mapper = PlaneMapper.from_config(_config())
    xy = (0.5900, 0.1500)
    uv = mapper.xy_to_uv(*xy)
    assert mapper.uv_to_xy(*uv) == pytest.approx(xy, abs=1e-12)


def test_uv_xy_roundtrip():
    mapper = PlaneMapper.from_config(_config())
    uv = (0.72, 0.33)
    xy = mapper.uv_to_xy(*uv)
    assert mapper.xy_to_uv(*xy) == pytest.approx(uv, abs=1e-12)


def test_metric_radius_uv_roundtrip():
    mapper = PlaneMapper.from_config(_config())
    for mode in ("min", "max", "mean", "x", "y"):
        metric = mapper.uv_radius_to_metric(0.08, mode)
        assert mapper.metric_radius_to_uv(metric, mode) == pytest.approx(0.08, abs=1e-12)


def test_metric_obstacle_is_not_double_mapped():
    mapper = PlaneMapper.from_config(_config())
    obstacles = normalize_obstacles_to_uv(
        [{"id": "measured", "center": [0.5504, 0.1806], "radius": 0.0144}],
        mapper,
        input_mode="metric",
        input_frame="left_interface_link",
        radius_scale_mode="min",
    )
    assert obstacles[0]["center"] == pytest.approx([0.28, 0.67], abs=1e-9)
    assert obstacles[0]["radius"] == pytest.approx(0.08, abs=1e-9)
    assert mapper.uv_to_xy(*obstacles[0]["center"]) == pytest.approx([0.5504, 0.1806], abs=1e-9)


def test_old_normalized_scenario_remains_supported():
    mapper = PlaneMapper.from_config(_config())
    obstacles = normalize_obstacles_to_uv(
        [{"center": [0.28, 0.67], "radius": 0.08}],
        mapper,
        input_mode="normalized",
        input_frame="left_interface_link",
        radius_scale_mode="min",
    )
    assert obstacles[0]["center"] == pytest.approx([0.28, 0.67], abs=1e-12)
    assert obstacles[0]["radius"] == pytest.approx(0.08, abs=1e-12)


def test_frame_mismatch_is_rejected():
    mapper = PlaneMapper.from_config(_config())
    with pytest.raises(ValueError, match="Changing a frame_id label is not a coordinate transform"):
        normalize_obstacles_to_uv(
            [{"center": [0.5504, 0.1806], "radius": 0.0144}],
            mapper,
            input_mode="metric",
            input_frame="world",
            radius_scale_mode="min",
        )


def test_uv_circle_to_cylinder_uses_same_plane_frame():
    mapper = PlaneMapper.from_config(_config())
    cylinder = mapper.uv_circle_to_cylinder(
        {"center": [0.5, 0.25], "radius": 0.1},
        height=0.18,
        center_z=0.22,
        radius_scale_mode="min",
        obstacle_id="demo_01",
    )
    assert cylinder["id"] == "demo_01"
    assert cylinder["frame_id"] == "left_interface_link"
    assert np.allclose(cylinder["position"], [0.59, 0.105, 0.22])
    assert pytest.approx(cylinder["radius"], rel=1e-6) == 0.018
    assert cylinder["bottom_z"] == pytest.approx(0.13)
    assert cylinder["top_z"] == pytest.approx(0.31)
