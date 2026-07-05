#!/usr/bin/env python3
"""Spawn obstacles to both MoveIt planning scene and Gazebo (GZ)."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

PACKAGE_NAME = "intent_hybrid_planner"


@dataclass
class CylinderObstacle:
    obs_id: str
    x: float
    y: float
    z: float
    radius: float
    height: float
    color: Tuple[float, float, float, float]


def _package_share_dir() -> Path:
    try:
        out = subprocess.run(
            ["ros2", "pkg", "prefix", PACKAGE_NAME],
            check=True,
            text=True,
            capture_output=True,
        )
        prefix = out.stdout.strip()
        if not prefix:
            raise RuntimeError("empty package prefix")
        return Path(prefix) / "share" / PACKAGE_NAME
    except Exception as exc:  # pylint: disable=broad-except
        raise RuntimeError(f"Cannot locate package share for {PACKAGE_NAME}: {exc}") from exc


def _default_config_path() -> Path:
    return _package_share_dir() / "config" / "obstacles_default.json"


def _default_sdf_path() -> Path:
    return _package_share_dir() / "sdf" / "obstacles.sdf"


def _as_float(v: Any, fallback: float) -> float:
    try:
        return float(v)
    except Exception:  # pylint: disable=broad-except
        return fallback


def _parse_color(raw: Any) -> Tuple[float, float, float, float]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 4:
        return (1.0, 0.0, 0.0, 1.0)
    color = []
    for i, v in enumerate(raw):
        default = 1.0 if i == 3 else 0.0
        color.append(max(0.0, min(1.0, _as_float(v, default))))
    return (float(color[0]), float(color[1]), float(color[2]), float(color[3]))


def _load_obstacle_config(config_path: Path) -> Tuple[str, str, List[CylinderObstacle]]:
    raw = json.loads(config_path.read_text(encoding="utf-8"))
    world_name = str(raw.get("world_name", "empty")).strip() or "empty"
    frame_id = str(raw.get("frame_id", "base_link")).strip() or "base_link"
    if frame_id != "base_link":
        raise RuntimeError(f"Only frame_id=base_link is supported, got {frame_id}")

    obstacles: List[CylinderObstacle] = []
    for i, item in enumerate(raw.get("obstacles", []), start=1):
        if not isinstance(item, dict):
            continue
        if str(item.get("type", "cylinder")).strip().lower() != "cylinder":
            continue
        obs_id = str(item.get("id", f"pillar_{i:02d}")).strip() or f"pillar_{i:02d}"
        obstacles.append(
            CylinderObstacle(
                obs_id=obs_id,
                x=_as_float(item.get("x"), 0.45),
                y=_as_float(item.get("y"), 0.0),
                z=_as_float(item.get("z"), 0.40),
                radius=max(1e-4, _as_float(item.get("radius"), 0.08)),
                height=max(1e-4, _as_float(item.get("height"), 0.8)),
                color=_parse_color(item.get("color")),
            )
        )
    if not obstacles:
        raise RuntimeError("No valid cylinder obstacles found in config.")
    return (world_name, frame_id, obstacles)


def _build_sdf_text(obstacles: Sequence[CylinderObstacle]) -> str:
    parts: List[str] = []
    parts.append('<?xml version="1.0" ?>')
    parts.append('<sdf version="1.7">')
    parts.append('  <model name="hybrid_obstacles">')
    parts.append("    <static>true</static>")
    parts.append('    <link name="obstacles_link">')
    for obs in obstacles:
        color = f"{obs.color[0]:.3f} {obs.color[1]:.3f} {obs.color[2]:.3f} {obs.color[3]:.3f}"
        parts.append(f'      <collision name="{obs.obs_id}_collision">')
        parts.append(f"        <pose>{obs.x:.6f} {obs.y:.6f} {obs.z:.6f} 0 0 0</pose>")
        parts.append("        <geometry>")
        parts.append("          <cylinder>")
        parts.append(f"            <radius>{obs.radius:.6f}</radius>")
        parts.append(f"            <length>{obs.height:.6f}</length>")
        parts.append("          </cylinder>")
        parts.append("        </geometry>")
        parts.append("      </collision>")
        parts.append(f'      <visual name="{obs.obs_id}_visual">')
        parts.append(f"        <pose>{obs.x:.6f} {obs.y:.6f} {obs.z:.6f} 0 0 0</pose>")
        parts.append("        <geometry>")
        parts.append("          <cylinder>")
        parts.append(f"            <radius>{obs.radius:.6f}</radius>")
        parts.append(f"            <length>{obs.height:.6f}</length>")
        parts.append("          </cylinder>")
        parts.append("        </geometry>")
        parts.append("        <material>")
        parts.append(f"          <ambient>{color}</ambient>")
        parts.append(f"          <diffuse>{color}</diffuse>")
        parts.append("        </material>")
        parts.append("      </visual>")
    parts.append("    </link>")
    parts.append("  </model>")
    parts.append("</sdf>")
    parts.append("")
    return "\n".join(parts)


def _spawn_to_gazebo(world_name: str, sdf_path: Path, logger: Node) -> bool:
    cmd = [
        "ros2",
        "run",
        "ros_gz_sim",
        "create",
        "-world",
        world_name,
        "-file",
        str(sdf_path),
        "-name",
        "hybrid_obstacles",
        "-allow_renaming",
        "true",
    ]
    logger.get_logger().info(f"Spawning Gazebo obstacles: {' '.join(cmd)}")
    try:
        out = subprocess.run(cmd, text=True, capture_output=True, check=False)
    except Exception as exc:  # pylint: disable=broad-except
        logger.get_logger().error(f"Gazebo spawn command failed: {exc}")
        return False
    if out.stdout.strip():
        logger.get_logger().info(out.stdout.strip())
    if out.returncode != 0:
        logger.get_logger().error(
            f"Gazebo obstacle spawn failed (rc={out.returncode}): {out.stderr.strip()}"
        )
        return False
    logger.get_logger().info("Gazebo obstacle spawn succeeded.")
    return True


def _build_collision_objects(
    frame_id: str, obstacles: Sequence[CylinderObstacle]
) -> List[CollisionObject]:
    out: List[CollisionObject] = []
    for obs in obstacles:
        obj = CollisionObject()
        obj.header = Header(frame_id=frame_id)
        obj.id = obs.obs_id
        obj.operation = CollisionObject.ADD

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [float(obs.height), float(obs.radius)]

        pose = Pose()
        pose.position.x = float(obs.x)
        pose.position.y = float(obs.y)
        pose.position.z = float(obs.z)
        pose.orientation.w = 1.0

        obj.primitives.append(prim)
        obj.primitive_poses.append(pose)
        out.append(obj)
    return out


def _apply_planning_scene(node: Node, objs: Sequence[CollisionObject], timeout_sec: float = 5.0) -> bool:
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().error("/apply_planning_scene service is not available.")
        return False

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects = list(objs)

    req = ApplyPlanningScene.Request()
    req.scene = scene
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error("/apply_planning_scene timed out.")
        return False
    try:
        resp = future.result()
    except Exception as exc:  # pylint: disable=broad-except
        node.get_logger().error(f"/apply_planning_scene failed: {exc}")
        return False
    if not bool(resp.success):
        node.get_logger().error("/apply_planning_scene returned success=false.")
        return False
    node.get_logger().info("Applied planning_scene obstacles via /apply_planning_scene.")
    return True


def _publish_planning_scene(
    frame_id: str, obstacles: Sequence[CylinderObstacle], publish_count: int
) -> bool:
    node = Node("spawn_obstacles")
    pub = node.create_publisher(PlanningScene, "/planning_scene", 10)
    collision_pub = node.create_publisher(CollisionObject, "/collision_object", 10)
    objs = _build_collision_objects(frame_id, obstacles)
    count = max(1, int(publish_count))
    try:
        # Authoritative MoveIt update. Topic-only publication is volatile and can be
        # missed by move_group or by a late-initializing PlanningSceneMonitor.
        if not _apply_planning_scene(node, objs):
            node.destroy_node()
            return False

        for i in range(count):
            scene = PlanningScene()
            scene.is_diff = True
            scene.world.collision_objects = objs
            pub.publish(scene)
            for obj in objs:
                collision_pub.publish(obj)
            node.get_logger().info(
                f"Published planning_scene/collision_object obstacles ({i + 1}/{count})."
            )
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.3)
    except Exception as exc:  # pylint: disable=broad-except
        node.get_logger().error(f"PlanningScene publish failed: {exc}")
        node.destroy_node()
        return False
    node.destroy_node()
    return True


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="both", choices=["planning_scene", "gazebo", "both"])
    parser.add_argument("--config", default="", help="Obstacle JSON config path.")
    parser.add_argument("--world-name", default="", help="Override world name.")
    parser.add_argument("--publish-count", type=int, default=3)
    parser.add_argument("--sdf-output", default="", help="SDF output path.")
    ns = parser.parse_args(args=args)

    config_path = Path(ns.config).expanduser() if ns.config else _default_config_path()
    if not config_path.exists():
        print(f"[spawn_obstacles] ERROR: config file not found: {config_path}", file=sys.stderr)
        sys.exit(1)

    try:
        cfg_world, frame_id, obstacles = _load_obstacle_config(config_path)
    except Exception as exc:  # pylint: disable=broad-except
        print(f"[spawn_obstacles] ERROR: invalid obstacle config: {exc}", file=sys.stderr)
        sys.exit(1)

    world_name = (ns.world_name or cfg_world or "empty").strip()
    sdf_output = (
        Path(ns.sdf_output).expanduser() if ns.sdf_output else _default_sdf_path()
    ).resolve()
    sdf_output.parent.mkdir(parents=True, exist_ok=True)
    sdf_output.write_text(_build_sdf_text(obstacles), encoding="utf-8")

    needs_gazebo = ns.mode in ("gazebo", "both")
    needs_scene = ns.mode in ("planning_scene", "both")

    gazebo_ok = True
    scene_ok = True

    rclpy.init(args=args)
    log_node = Node("spawn_obstacles_log")
    log_node.get_logger().info(
        f"Obstacle config loaded: {config_path}, mode={ns.mode}, world={world_name}, "
        f"count={len(obstacles)}, frame={frame_id}, sdf={sdf_output}"
    )

    if needs_gazebo:
        gazebo_ok = _spawn_to_gazebo(world_name, sdf_output, log_node)
    if needs_scene:
        scene_ok = _publish_planning_scene(frame_id, obstacles, ns.publish_count)

    log_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    if gazebo_ok and scene_ok:
        print("[spawn_obstacles] success")
        sys.exit(0)
    print(
        f"[spawn_obstacles] failed: gazebo_ok={gazebo_ok}, planning_scene_ok={scene_ok}",
        file=sys.stderr,
    )
    sys.exit(2)


if __name__ == "__main__":
    main()
