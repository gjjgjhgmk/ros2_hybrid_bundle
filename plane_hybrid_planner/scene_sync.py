"""Sync canonical UV circular obstacles into the active MoveIt planning scene."""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, Iterable, List

from .plane_mapping import PlaneMapper

LOGGER = logging.getLogger(__name__)


def scene_obstacle_geometry(
    obstacles_uv: Iterable[Dict[str, Any]],
    mapper: PlaneMapper,
    *,
    scene_config: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    """Map canonical UV obstacles to metric cylinders in mapper.frame_id."""
    config = dict(scene_config or {})
    frame_id = str(config.get("frame_id", mapper.frame_id))
    if frame_id != mapper.frame_id:
        raise ValueError(
            "scene_obstacles.frame_id must match plane.frame_id unless a real TF2 "
            "transform is applied. Changing a frame_id label is not a coordinate "
            f"transform. scene_frame={frame_id}, plane_frame={mapper.frame_id}"
        )

    height = float(config.get("cylinder_height", 0.20))
    if height <= 0.0:
        raise ValueError("scene_obstacles.cylinder_height must be positive")
    table_surface_z = float(
        config.get("table_surface_z", config.get("drawing_plane", {}).get("table_surface_z", -0.0102))
    )
    center_z_mode = str(config.get("center_z_mode", "explicit" if "center_z" in config else "from_table_surface"))
    formula_center_z = table_surface_z + height / 2.0
    if center_z_mode == "from_table_surface":
        center_z_value = formula_center_z
    elif center_z_mode == "explicit":
        if "center_z" not in config:
            raise ValueError("scene_obstacles.center_z is required when center_z_mode=explicit")
        center_z_value = float(config["center_z"])
    else:
        raise ValueError("scene_obstacles.center_z_mode must be from_table_surface or explicit")
    if "center_z" in config:
        explicit_center_z = float(config["center_z"])
        if abs(explicit_center_z - formula_center_z) > float(config.get("center_z_warning_tolerance", 0.002)):
            LOGGER.warning(
                "scene_obstacles.center_z differs from table_surface_z + cylinder_height / 2: "
                "explicit=%.6f formula=%.6f",
                explicit_center_z,
                formula_center_z,
            )
        if center_z_mode == "explicit":
            center_z_value = explicit_center_z

    radius_scale_mode = str(config.get("radius_scale_mode", "min"))
    id_prefix = str(config.get("id_prefix", "plane_obstacle"))
    objects = []
    for index, obstacle in enumerate(list(obstacles_uv or []), start=1):
        mapped = mapper.uv_circle_to_cylinder(
            obstacle,
            height=height,
            center_z=center_z_value,
            radius_scale_mode=radius_scale_mode,
            frame_id=frame_id,
            obstacle_id=str(obstacle.get("id") or f"{id_prefix}_{index:02d}"),
        )
        objects.append(mapped)

    return {
        "frame_id": frame_id,
        "objects": objects,
        "table_surface_z": table_surface_z,
        "cylinder_height": height,
        "center_z": float(center_z_value),
        "cylinder_bottom_z": float(center_z_value - height / 2.0),
        "cylinder_top_z": float(center_z_value + height / 2.0),
        "radius_scale_mode": radius_scale_mode,
    }


def sync_circular_obstacles_to_moveit(
    obstacles_uv: Iterable[Dict[str, Any]],
    mapper: PlaneMapper,
    *,
    scene_config: Dict[str, Any] | None = None,
    timeout_sec: float = 5.0,
) -> Dict[str, Any]:
    config = dict(scene_config or {})
    geometry = scene_obstacle_geometry(obstacles_uv, mapper, scene_config=config)
    result: Dict[str, Any] = {
        "attempted": False,
        "success": False,
        "obstacle_count": len(geometry["objects"]),
        "frame_id": geometry["frame_id"],
        "objects": geometry["objects"],
        "rviz_marker_topic": "/visualization_marker_array",
        "rviz_markers_published": False,
        "reason": "",
        "table_surface_z": geometry["table_surface_z"],
        "cylinder_height": geometry["cylinder_height"],
        "center_z": geometry["center_z"],
        "cylinder_bottom_z": geometry["cylinder_bottom_z"],
        "cylinder_top_z": geometry["cylinder_top_z"],
    }
    if not bool(config.get("sync_to_moveit", False)):
        result["reason"] = "sync_disabled"
        return result

    try:
        import rclpy  # pylint: disable=import-outside-toplevel
        from geometry_msgs.msg import Pose  # pylint: disable=import-outside-toplevel
        from moveit_msgs.msg import CollisionObject, PlanningScene  # pylint: disable=import-outside-toplevel
        from moveit_msgs.srv import ApplyPlanningScene  # pylint: disable=import-outside-toplevel
        from rclpy.qos import (  # pylint: disable=import-outside-toplevel
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from shape_msgs.msg import SolidPrimitive  # pylint: disable=import-outside-toplevel
        from std_msgs.msg import Header  # pylint: disable=import-outside-toplevel
        from visualization_msgs.msg import Marker, MarkerArray  # pylint: disable=import-outside-toplevel
    except Exception as exc:  # pragma: no cover - depends on ROS runtime.
        result["reason"] = f"ros_import_failed: {exc}"
        return result

    obstacles = geometry["objects"]
    frame_id = geometry["frame_id"]
    publish_repeats = max(int(config.get("publish_repeats", 3)), 1)
    clear_slots = max(int(config.get("clear_slots", 8)), len(obstacles))
    id_prefix = str(config.get("id_prefix", "plane_obstacle"))

    result["attempted"] = True

    shutdown_needed = False
    if not rclpy.ok():
        rclpy.init(args=None)
        shutdown_needed = True

    node = rclpy.create_node("plane_hybrid_scene_sync")
    planning_scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    collision_object_pub = node.create_publisher(CollisionObject, "/collision_object", 10)
    marker_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    marker_pub = node.create_publisher(MarkerArray, result["rviz_marker_topic"], marker_qos)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    try:
        collision_objects: List[CollisionObject] = []
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.ns = id_prefix
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for index in range(clear_slots):
            remove_obj = CollisionObject()
            remove_obj.header = Header(frame_id=frame_id)
            remove_obj.id = f"{id_prefix}_{index + 1:02d}"
            remove_obj.operation = CollisionObject.REMOVE
            collision_objects.append(remove_obj)

        for index, mapped in enumerate(obstacles, start=1):
            obj = CollisionObject()
            obj.header = Header(frame_id=frame_id)
            obj.id = mapped["id"]
            obj.operation = CollisionObject.ADD

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [float(mapped["height"]), float(mapped["radius"])]

            pose = Pose()
            pose.position.x = float(mapped["position"][0])
            pose.position.y = float(mapped["position"][1])
            pose.position.z = float(mapped["position"][2])
            pose.orientation.w = 1.0

            obj.primitives.append(primitive)
            obj.primitive_poses.append(pose)
            collision_objects.append(obj)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.ns = id_prefix
            marker.id = index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 2.0 * float(mapped["radius"])
            marker.scale.y = 2.0 * float(mapped["radius"])
            marker.scale.z = float(mapped["height"])
            marker.color.r = 0.92
            marker.color.g = 0.25
            marker.color.b = 0.18
            marker.color.a = 0.55
            marker.frame_locked = False
            marker_array.markers.append(marker)

        if client.wait_for_service(timeout_sec=float(timeout_sec)):
            request = ApplyPlanningScene.Request()
            request.scene = PlanningScene()
            request.scene.is_diff = True
            request.scene.world.collision_objects = collision_objects
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=float(timeout_sec))
            if future.done():
                response = future.result()
                if response is not None and bool(response.success):
                    result["success"] = True
                else:
                    result["reason"] = "apply_planning_scene_returned_false"
            else:
                result["reason"] = "apply_planning_scene_timeout"
        else:
            result["reason"] = "apply_planning_scene_unavailable"

        for _ in range(publish_repeats):
            scene_msg = PlanningScene()
            scene_msg.is_diff = True
            scene_msg.world.collision_objects = collision_objects
            planning_scene_pub.publish(scene_msg)
            for obj in collision_objects:
                collision_object_pub.publish(obj)
            marker_pub.publish(marker_array)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.10)

        result["rviz_markers_published"] = True

        if not result["success"] and not result["reason"]:
            result["reason"] = "published_only"
    finally:
        node.destroy_node()
        if shutdown_needed:
            rclpy.shutdown()

    return result
