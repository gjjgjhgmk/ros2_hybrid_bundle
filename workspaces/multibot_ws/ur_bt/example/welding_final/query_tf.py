#!/usr/bin/env python3
"""Query TF poses and convert them to ur_move cart waypoints.

Edit the constants in the CONFIG section, start ur_move's TF ZMQ server, then run:

    python3 query_tf.py

The generated waypoints mean: drive `ik_frame` to the pose of `goal_frame`
expressed in `reference_frame`. This matches ur_move's MoveIt call:
setPoseTarget(PoseStamped(frame_id=reference_frame), ik_frame).
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping


# ----------------------------- CONFIG ---------------------------------

# ur_move TF ZMQ server. This is the host running tf_zmq_server.py, not the
# trajectory planner port. The TF port is 5609 by default in ur_move.
TF_SERVER_HOST = "127.0.0.1"
TF_SERVER_PORT = 5609
TF_TIMEOUT_SEC = 5

# One seed waypoint is created for each arm. `goal_frame` should represent the
# desired pose of the arm's ik_frame, expressed by TF relative to reference_frame.
ARM_TF_QUERIES: Mapping[str, Dict[str, str]] = {
    "left_arm": {
        "waypoint_name": "left_weld_seed",
        "reference_frame": "left_interface_link",
        "goal_frame": "left_ee_link",
        "ik_frame": "left_ee_link",
    },
    "right_arm": {
        "waypoint_name": "right_weld_seed",
        "reference_frame": "right_interface_link",
        "goal_frame": "right_ee_link",
        "ik_frame": "right_ee_link",
    },
}

WELD_PLANNER = "lin"
MAX_VELOCITY_SCALING = 0.05
MAX_ACCELERATION_SCALING = 0.05

# Optional offset applied in each arm's reference frame. Keep zero when you want
# the waypoint exactly at the queried TF pose.
POSITION_OFFSET = [0.0, 0.0, 0.0]

OUTPUT_WAYPOINTS = Path(__file__).resolve().parent / "queried_waypoints.json"

# --------------------------- END CONFIG -------------------------------


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
UR_BT_CLIENTS_DIR = UR_BT_DIR / "src" / "ur_bt" / "clients"
sys.path.insert(0, str(UR_BT_CLIENTS_DIR))

from tf_client import TFClient  # noqa: E402


def normalize_quaternion(q: Dict[str, float]) -> Dict[str, float]:
    norm = math.sqrt(q["x"] ** 2 + q["y"] ** 2 + q["z"] ** 2 + q["w"] ** 2)
    if norm <= 0.0:
        raise ValueError("TF returned a zero-length quaternion")
    return {key: value / norm for key, value in q.items()}


def query_transform(client: TFClient, reference_frame: str, goal_frame: str) -> Dict[str, Any]:
    """Return goal_frame pose expressed in reference_frame."""
    response = client.lookup_transform(reference_frame, goal_frame)
    if response is None:
        raise RuntimeError(f"No response while querying {reference_frame} <- {goal_frame}")
    if not response.get("success", False):
        raise RuntimeError(response.get("message", f"TF lookup failed: {reference_frame} <- {goal_frame}"))
    return response["data"]


def transform_to_waypoint(
    group: str,
    query_config: Dict[str, str],
    transform: Dict[str, Any],
    position_offset: List[float],
) -> Dict[str, Any]:
    translation = transform["translation"]
    rotation = normalize_quaternion(transform["rotation"])

    position = [
        translation["x"] + position_offset[0],
        translation["y"] + position_offset[1],
        translation["z"] + position_offset[2],
    ]

    return {
        "group": group,
        "planner": WELD_PLANNER,
        "description": query_config["waypoint_name"],
        "type": "cart",
        "max_velocity_scaling_factor": MAX_VELOCITY_SCALING,
        "max_acceleration_scaling_factor": MAX_ACCELERATION_SCALING,
        "ik_frame": query_config["ik_frame"],
        "frame_id": query_config["reference_frame"],
        "position": position,
        "orientation": [rotation["x"], rotation["y"], rotation["z"], rotation["w"]],
    }


def save_waypoints(path: Path, waypoints: Dict[str, Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(waypoints, f, ensure_ascii=False, indent=2)
        f.write("\n")


def main() -> int:
    waypoints: Dict[str, Dict[str, Any]] = {}

    with TFClient(
        server_ip=TF_SERVER_HOST,
        server_port=TF_SERVER_PORT,
        timeout=TF_TIMEOUT_SEC,
    ) as tf_client:
        for group, query_config in ARM_TF_QUERIES.items():
            reference_frame = query_config["reference_frame"]
            goal_frame = query_config["goal_frame"]

            transform = query_transform(tf_client, reference_frame, goal_frame)
            waypoint = transform_to_waypoint(group, query_config, transform, POSITION_OFFSET)
            waypoints[query_config["waypoint_name"]] = waypoint

            print(f"{group}: {reference_frame} <- {goal_frame}")
            print(f"  position: {waypoint['position']}")
            print(f"  orientation: {waypoint['orientation']}")

    save_waypoints(OUTPUT_WAYPOINTS, waypoints)
    print(f"Saved {len(waypoints)} waypoint(s) to {OUTPUT_WAYPOINTS}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
