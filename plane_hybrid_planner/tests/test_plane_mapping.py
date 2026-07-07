import numpy as np
import pytest

from plane_hybrid_planner.plane_mapping import PlaneMapper


def _config(clamp=True):
    return {
        "plane": {
            "frame_id": "world",
            "coordinate_mode": "metric",
            "x_min": 0.3,
            "x_max": 0.7,
            "y_min": 0.1,
            "y_max": 0.5,
            "z": 0.2,
        },
        "orientation_xyzw": [0.0, 2.0, 0.0, 0.0],
        "waypoints": {"clamp_uv": clamp},
    }


def test_uv_mapping_and_quaternion_normalization():
    mapper = PlaneMapper.from_config(_config())
    assert mapper.coordinate_mode == "metric"
    assert np.allclose(mapper.uv_to_xyz(0.5, 0.25), [0.5, 0.2, 0.2])
    waypoint = mapper.uv_path_to_cart_waypoints([[0.0, 0.0], [1.0, 1.0]])[0]
    assert waypoint["frame_id"] == "world"
    assert waypoint["orientation"] == [0.0, 1.0, 0.0, 0.0]


def test_uv_clamp_and_strict_mode():
    assert PlaneMapper.from_config(_config(True)).uv_to_xyz(-1.0, 2.0) == (0.3, 0.5, 0.2)
    with pytest.raises(ValueError):
        PlaneMapper.from_config(_config(False)).uv_to_xyz(-0.01, 0.5)


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
    assert cylinder["frame_id"] == "world"
    assert np.allclose(cylinder["position"], [0.5, 0.2, 0.22])
    assert pytest.approx(cylinder["radius"], rel=1e-6) == 0.04
