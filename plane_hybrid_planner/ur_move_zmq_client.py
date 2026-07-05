"""Adapter for the existing ur_move TrajectoryPlannerServer ZMQ protocol."""

from __future__ import annotations

import copy
import importlib.util
import json
import logging
from pathlib import Path
from typing import Any, Dict, Iterable, Optional


LOGGER = logging.getLogger(__name__)


class UrMoveZmqClient:
    """Use the Workspace client when possible, with a protocol-compatible fallback."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5605,
        timeout_sec: float = 10.0,
        workspace_root: Optional[str] = None,
        prefer_workspace_client: bool = True,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.timeout_sec = max(float(timeout_sec), 0.05)
        self.workspace_root = Path(workspace_root).resolve() if workspace_root else None
        self.prefer_workspace_client = bool(prefer_workspace_client)
        self._workspace_client = None
        self._workspace_error = ""

    def _load_workspace_client(self) -> Any:
        if self._workspace_client is not None:
            return self._workspace_client
        if not self.prefer_workspace_client or self.port != 5605 or self.workspace_root is None:
            return None
        module_path = self.workspace_root / "ur_move" / "client" / "zmq_ur_move_client.py"
        if not module_path.exists():
            self._workspace_error = f"Workspace client not found: {module_path}"
            return None
        try:
            spec = importlib.util.spec_from_file_location("workspace_ur_move_client", module_path)
            if spec is None or spec.loader is None:
                raise ImportError(f"cannot load module spec: {module_path}")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self._workspace_client = module.UrMoveClient(
                server_host=self.host,
                timeout_ms=int(self.timeout_sec * 1000.0),
            )
            return self._workspace_client
        except Exception as exc:  # Import failure should not break offline evaluation.
            self._workspace_error = str(exc)
            LOGGER.debug("Workspace UrMoveClient unavailable: %s", exc)
            return None

    @staticmethod
    def build_waypoint_dict(
        cart_waypoints: Iterable[Dict[str, Any]],
        *,
        group_name: str,
        ik_frame: str,
        planner: str = "lin",
        velocity_scale: float = 0.1,
        acceleration_scale: float = 0.1,
    ) -> Dict[str, Dict[str, Any]]:
        if group_name not in {"left_arm", "right_arm"}:
            raise ValueError("group_name must be left_arm or right_arm")
        result: Dict[str, Dict[str, Any]] = {}
        for index, waypoint in enumerate(cart_waypoints):
            position = list(waypoint["position"])
            orientation = list(waypoint["orientation"])
            if len(position) != 3 or len(orientation) != 4:
                raise ValueError("Cartesian waypoint requires position[3] and orientation[4]")
            result[f"plane_wp_{index:03d}"] = {
                "group": group_name,
                "planner": str(planner),
                "type": "cart",
                "ik_frame": str(ik_frame),
                "frame_id": str(waypoint["frame_id"]),
                "position": [float(value) for value in position],
                "orientation": [float(value) for value in orientation],
                "max_velocity_scaling_factor": float(velocity_scale),
                "max_acceleration_scaling_factor": float(acceleration_scale),
            }
        if not result:
            raise ValueError("at least one Cartesian waypoint is required")
        return result

    def _direct_request(self, request: Dict[str, Any]) -> Dict[str, Any]:
        try:
            import zmq  # pylint: disable=import-outside-toplevel
        except ImportError as exc:
            return {
                "success": False,
                "error": f"pyzmq unavailable: {exc}",
                "error_kind": "ur_move_unavailable",
            }

        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        timeout_ms = int(self.timeout_sec * 1000.0)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, min(timeout_ms, 5000))
        try:
            socket.connect(f"tcp://{self.host}:{self.port}")
            socket.send_string(json.dumps(request))
            raw = socket.recv_string()
            response = json.loads(raw)
            response["_raw_response"] = raw
            return response
        except (zmq.Again, OSError) as exc:
            return {
                "success": False,
                "error": f"ur_move request timed out/unavailable: {exc}",
                "error_kind": "ur_move_unavailable",
            }
        except Exception as exc:  # Malformed server responses remain visible to the caller.
            return {
                "success": False,
                "error": f"ur_move request failed: {exc}",
                "error_kind": "ur_move_protocol_error",
            }
        finally:
            socket.close(linger=0)
            context.term()

    def send_cart_waypoints(
        self,
        cart_waypoints: Iterable[Dict[str, Any]],
        *,
        group_name: str,
        ik_frame: str,
        planner: str = "lin",
        plan_only: bool = True,
        execute: bool = False,
        velocity_scale: float = 0.1,
        acceleration_scale: float = 0.1,
    ) -> Dict[str, Any]:
        if plan_only and execute:
            raise ValueError("plan_only and execute cannot both be true")
        waypoint_dict = self.build_waypoint_dict(
            cart_waypoints,
            group_name=group_name,
            ik_frame=ik_frame,
            planner=planner,
            velocity_scale=velocity_scale,
            acceleration_scale=acceleration_scale,
        )
        request_waypoints = []
        for name, waypoint in waypoint_dict.items():
            item = copy.deepcopy(waypoint)
            item["name"] = name
            request_waypoints.append(item)
        request = {"waypoints": request_waypoints, "execute": bool(execute)}

        workspace_client = self._load_workspace_client()
        transport = "workspace_client" if workspace_client is not None else "compatible_zmq"
        if workspace_client is not None:
            try:
                response = workspace_client.plan_trajectory(
                    copy.deepcopy(waypoint_dict), execute=bool(execute)
                )
            except Exception as exc:
                response = {
                    "success": False,
                    "error": f"Workspace UrMoveClient failed: {exc}",
                    "error_kind": "ur_move_unavailable",
                }
        else:
            response = self._direct_request(request)

        success = bool(response.get("success", False))
        has_trajectory = bool(response.get("trajectories"))
        has_execution_id = bool(response.get("execution_id"))
        error_text = str(response.get("error", ""))
        execution_failed = bool(execute and not success and has_trajectory)
        planning_success = bool(success or has_trajectory or has_execution_id)
        error_kind = str(response.get("error_kind", ""))
        unavailable = error_kind == "ur_move_unavailable"
        return {
            "available": not unavailable,
            "planning_success": planning_success,
            "execution_requested": bool(execute),
            "execution_success": (success if execute else None) if not execution_failed else False,
            "execution_id": response.get("execution_id", ""),
            "error": error_text,
            "error_kind": (
                "moveit_execution_failed"
                if execution_failed and not error_kind
                else error_kind
            ),
            "transport": transport,
            "workspace_client_error": self._workspace_error,
            "request": request,
            "response": response,
        }
