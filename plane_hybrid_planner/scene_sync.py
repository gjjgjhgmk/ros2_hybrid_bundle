"""Sync 2D circular obstacles into the active MoveIt planning scene."""

from __future__ import annotations

import time
from typing import Any, Dict, Iterable, List

from .plane_mapping import PlaneMapper


def sync_circular_obstacles_to_moveit(
    obstacles_uv: Iterable[Dict[str, Any]],
    mapper: PlaneMapper,
    *,
    scene_config: Dict[str, Any] | None = None,
    timeout_sec: float = 5.0,
) -> Dict[str, Any]:
    config = dict(scene_config or {})
    result: Dict[str, Any] = {
        "attempted": False,
        "success": False,
        "obstacle_count": 0,
        "frame_id": str(config.get("frame_id", mapper.frame_id)),
        "objects": [],
        "reason": "",
    }
    if not bool(config.get("sync_to_moveit", False)):
        result["reason"] = "sync_disabled"
        return result

    try:
        import rclpy  # pylint: disable=import-outside-toplevel
        from geometry_msgs.msg import Pose  # pylint: disable=import-outside-toplevel
        from moveit_msgs.msg import CollisionObject, PlanningScene  # pylint: disable=import-outside-toplevel
        from moveit_msgs.srv import ApplyPlanningScene  # pylint: disable=import-outside-toplevel
        from shape_msgs.msg import SolidPrimitive  # pylint: disable=import-outside-toplevel
        from std_msgs.msg import Header  # pylint: disable=import-outside-toplevel
    except Exception as exc:  # pragma: no cover - depends on ROS runtime.
        result["reason"] = f"ros_import_failed: {exc}"
        return result

    obstacles = list(obstacles_uv or [])
    frame_id = str(config.get("frame_id", mapper.frame_id))
    height = float(config.get("cylinder_height", 0.20))
    center_z = config.get("center_z")
    center_z_value = None if center_z is None else float(center_z)
    radius_scale_mode = str(config.get("radius_scale_mode", "min"))
    publish_repeats = max(int(config.get("publish_repeats", 3)), 1)
    clear_slots = max(int(config.get("clear_slots", 8)), len(obstacles))
    id_prefix = str(config.get("id_prefix", "plane_obstacle"))

    result["attempted"] = True
    result["obstacle_count"] = len(obstacles)

    shutdown_needed = False
    if not rclpy.ok():
        rclpy.init(args=None)
        shutdown_needed = True

    node = rclpy.create_node("plane_hybrid_scene_sync")
    planning_scene_pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    collision_object_pub = node.create_publisher(CollisionObject, "/collision_object", 10)
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    result["objects"] = []

    try:
        collision_objects: List[CollisionObject] = []
        for index in range(clear_slots):
            remove_obj = CollisionObject()
            remove_obj.header = Header(frame_id=frame_id)
            remove_obj.id = f"{id_prefix}_{index + 1:02d}"
            remove_obj.operation = CollisionObject.REMOVE
            collision_objects.append(remove_obj)

        for index, obstacle in enumerate(obstacles, start=1):
            mapped = mapper.uv_circle_to_cylinder(
                obstacle,
                height=height,
                center_z=center_z_value,
                radius_scale_mode=radius_scale_mode,
                frame_id=frame_id,
                obstacle_id=f"{id_prefix}_{index:02d}",
            )
            result["objects"].append(mapped)

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
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.10)

        if not result["success"] and not result["reason"]:
            result["reason"] = "published_only"
    finally:
        node.destroy_node()
        if shutdown_needed:
            rclpy.shutdown()

    return result
