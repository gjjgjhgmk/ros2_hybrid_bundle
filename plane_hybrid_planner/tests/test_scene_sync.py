import pytest

from plane_hybrid_planner.obstacle_coordinates import normalize_obstacles_to_uv
from plane_hybrid_planner.plane_mapping import PlaneMapper
from plane_hybrid_planner.scene_sync import scene_obstacle_geometry


def _mapper():
    return PlaneMapper.from_config(
        {
            "plane": {
                "frame_id": "left_interface_link",
                "algorithm_coordinate_mode": "normalized",
                "x_min": 0.50,
                "x_max": 0.68,
                "y_min": 0.06,
                "y_max": 0.24,
                "z": 0.35,
            }
        }
    )


def test_scene_sync_geometry_uses_one_metric_pose_for_collision_and_marker():
    mapper = _mapper()
    obstacles = normalize_obstacles_to_uv(
        [{"id": "measured_obstacle_01", "center": [0.5504, 0.1806], "radius": 0.0144}],
        mapper,
        input_mode="metric",
        input_frame="left_interface_link",
        radius_scale_mode="min",
    )
    geometry = scene_obstacle_geometry(
        obstacles,
        mapper,
        scene_config={
            "frame_id": "left_interface_link",
            "cylinder_height": 0.18,
            "center_z_mode": "from_table_surface",
            "table_surface_z": -0.0102,
            "radius_scale_mode": "min",
        },
    )

    assert geometry["frame_id"] == "left_interface_link"
    assert geometry["table_surface_z"] == pytest.approx(-0.0102)
    assert geometry["cylinder_bottom_z"] == pytest.approx(-0.0102)
    assert geometry["cylinder_top_z"] == pytest.approx(0.1698)
    assert geometry["center_z"] == pytest.approx(0.0798)
    assert geometry["objects"][0]["frame_id"] == "left_interface_link"
    assert geometry["objects"][0]["position"] == pytest.approx([0.5504, 0.1806, 0.0798])
    assert geometry["objects"][0]["radius"] == pytest.approx(0.0144)


def test_scene_sync_frame_mismatch_is_rejected():
    mapper = _mapper()
    with pytest.raises(ValueError, match="Changing a frame_id label is not a coordinate transform"):
        scene_obstacle_geometry(
            [{"id": "obs", "center": [0.28, 0.67], "radius": 0.08}],
            mapper,
            scene_config={"frame_id": "world"},
        )
