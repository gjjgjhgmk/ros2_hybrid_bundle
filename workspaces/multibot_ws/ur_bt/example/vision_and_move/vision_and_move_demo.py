#!/usr/bin/env python3
"""Move the right arm to a point 10 cm above the detected ChArUco board."""

import argparse
import logging
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import py_trees
import yaml
import zmq


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
sys.path.insert(0, str(UR_BT_DIR / "src"))

from ur_bt import BehaviorTreeManager  # noqa: E402
from ur_bt.blackboard_manager import AccessType, BlackboardManager  # noqa: E402
from ur_bt.clients.tf_client import TFClient  # noqa: E402


LOGGER = logging.getLogger(__name__)


@dataclass
class VisionAndMoveSettings:
    """Runtime settings for the vision-and-move demo."""

    source_frame: str
    waypoint_frame: str
    end_effector_frame: str
    orientation_reference_frame: str
    camera_optical_frame: str
    camera_link_to_optical_xyz: List[float]
    camera_link_to_optical_rpy: List[float]
    handeye_calibration_file: Path
    waypoint_name: str
    approach_height_m: float
    group: str
    planner: str
    ik_frame: str
    max_velocity_scaling_factor: float
    max_acceleration_scaling_factor: float
    orientation_mode: str
    fixed_orientation: List[float]

    @classmethod
    def from_config(cls, config: Dict[str, Any]) -> "VisionAndMoveSettings":
        """Create settings from the top-level demo YAML dictionary."""
        demo_config = config.get("vision_and_move", {})
        handeye_file = Path(demo_config.get("handeye_calibration_file", "right_handeye_calibration.yaml"))
        if not handeye_file.is_absolute():
            handeye_file = THIS_DIR / handeye_file
        return cls(
            source_frame=demo_config.get("source_frame", "right_base_link"),
            waypoint_frame=demo_config.get("waypoint_frame", "right_interface_link"),
            end_effector_frame=demo_config.get("end_effector_frame", "right_tool0"),
            orientation_reference_frame=demo_config.get("orientation_reference_frame", "right_interface_link"),
            camera_optical_frame=demo_config.get("camera_optical_frame", "right_camera_opencv_optical_frame"),
            camera_link_to_optical_xyz=HandEyeCalibrationLoader._parse_vector(
                demo_config.get("camera_link_to_optical_xyz", [0.0, 0.0, 0.0]),
                "camera_link_to_optical_xyz",
            ),
            camera_link_to_optical_rpy=HandEyeCalibrationLoader._parse_vector(
                demo_config.get("camera_link_to_optical_rpy", [-math.pi / 2.0, 0.0, -math.pi / 2.0]),
                "camera_link_to_optical_rpy",
            ),
            handeye_calibration_file=handeye_file,
            waypoint_name=demo_config.get("waypoint_name", "right_above_charuco_board"),
            approach_height_m=float(demo_config.get("approach_height_m", 0.10)),
            group=demo_config.get("group", "right_arm"),
            planner=demo_config.get("planner", "lin"),
            ik_frame=demo_config.get("ik_frame", "right_ee_link"),
            max_velocity_scaling_factor=float(demo_config.get("max_velocity_scaling_factor", 0.05)),
            max_acceleration_scaling_factor=float(demo_config.get("max_acceleration_scaling_factor", 0.05)),
            orientation_mode=demo_config.get("orientation_mode", "ik_frame_in_reference_frame"),
            fixed_orientation=demo_config.get("fixed_orientation", [0.0, 0.0, 0.0, 1.0]),
        )


class CalibrationBoardPoseClient:
    """ZeroMQ client for querying board pose from the calibration service."""

    def __init__(self, host: str, port: int, timeout: int):
        """Connect to the calibration ZeroMQ service."""
        self.address = f"tcp://{host}:{port}"
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout * 1000)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout * 1000)
        self.socket.connect(self.address)
        LOGGER.info("Connected to calibration ZMQ service: %s", self.address)

    def get_board_pose(self) -> Dict[str, Any]:
        """Return the board pose detected from the current camera image."""
        request = {
            "operation": "get_board_pose",
            "params": {
                "image_path": None,
                "ignore_end_pose": True,
            },
        }
        self.socket.send_json(request)
        response = self.socket.recv_json()
        if response.get("status") != "success":
            raise RuntimeError(response.get("message", response))

        result = response.get("result", {})
        if not result.get("success", False):
            raise RuntimeError(result.get("message", result))
        return result

    def close(self) -> None:
        """Close the ZMQ socket and context."""
        self.socket.close()
        self.context.term()


class HandEyeCalibrationLoader:
    """Load the fixed right_tool0<-right_camera_link transform from YAML."""

    def __init__(self, calibration_path: Path):
        """Store the hand-eye calibration file path."""
        self.calibration_path = calibration_path

    def load_transform(self) -> Dict[str, Any]:
        """Return the hand-eye transform in the common transform dictionary format."""
        if not self.calibration_path.exists():
            raise FileNotFoundError(f"Hand-eye calibration file not found: {self.calibration_path}")

        with self.calibration_path.open("r", encoding="utf-8") as file_obj:
            data = yaml.safe_load(file_obj) or {}

        xyz = self._parse_vector(data.get("camera_link_xyz"), "camera_link_xyz")
        rpy = self._parse_vector(data.get("camera_link_rpy"), "camera_link_rpy")
        quaternion = self.rpy_to_quaternion(rpy[0], rpy[1], rpy[2])

        return {
            "translation": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
            "rotation": {
                key: value
                for key, value in zip(
                    ["x", "y", "z", "w"],
                    BoardTransformWaypointBuilder.normalize_quaternion(quaternion),
                )
            },
        }

    @staticmethod
    def _parse_vector(value: Any, key: str) -> List[float]:
        """Parse a three-element YAML vector stored as either a list or space-separated string."""
        if isinstance(value, str):
            parts = value.split()
        elif isinstance(value, list):
            parts = value
        else:
            raise ValueError(f"{key} must be a list or space-separated string")

        if len(parts) != 3:
            raise ValueError(f"{key} must contain 3 values")
        return [float(part) for part in parts]

    @staticmethod
    def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert ROS URDF roll-pitch-yaw radians to a quaternion in [x, y, z, w] order."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]


class TransformMath:
    """Small quaternion transform helpers for composing poses."""

    @staticmethod
    def vector3(value: Any, field_name: str) -> List[float]:
        """Convert a list or {'x', 'y', 'z'} dictionary into [x, y, z]."""
        if isinstance(value, dict):
            return [float(value["x"]), float(value["y"]), float(value["z"])]
        if isinstance(value, (list, tuple)) and len(value) == 3:
            return [float(value[0]), float(value[1]), float(value[2])]
        raise ValueError(f"{field_name} must be [x, y, z] or a dict with x/y/z")

    @staticmethod
    def quaternion(value: Any, field_name: str) -> List[float]:
        """Convert a list or {'x', 'y', 'z', 'w'} dictionary into [x, y, z, w]."""
        if isinstance(value, dict):
            return [float(value["x"]), float(value["y"]), float(value["z"]), float(value["w"])]
        if isinstance(value, (list, tuple)) and len(value) == 4:
            return [float(value[0]), float(value[1]), float(value[2]), float(value[3])]
        raise ValueError(f"{field_name} must be [x, y, z, w] or a dict with x/y/z/w")

    @staticmethod
    def multiply(q1: List[float], q2: List[float]) -> List[float]:
        """Multiply two quaternions in [x, y, z, w] order."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]

    @staticmethod
    def rotate(point: List[float], quaternion: List[float]) -> List[float]:
        """Rotate a point by a quaternion."""
        q_conj = [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]
        rotated = TransformMath.multiply(
            TransformMath.multiply(quaternion, [point[0], point[1], point[2], 0.0]),
            q_conj,
        )
        return rotated[:3]

    @staticmethod
    def compose(parent_child: Dict[str, Any], child_target: Dict[str, Any]) -> Dict[str, Any]:
        """Compose parent<-child and child<-target transforms."""
        p_t = TransformMath.vector3(parent_child["translation"], "parent_child.translation")
        p_q = TransformMath.quaternion(parent_child["rotation"], "parent_child.rotation")
        c_t = TransformMath.vector3(child_target["translation"], "child_target.translation")
        c_q = TransformMath.quaternion(child_target["rotation"], "child_target.rotation")

        rotated_child_t = TransformMath.rotate(c_t, p_q)
        return {
            "translation": {
                "x": p_t[0] + rotated_child_t[0],
                "y": p_t[1] + rotated_child_t[1],
                "z": p_t[2] + rotated_child_t[2],
            },
            "rotation": {
                key: value
                for key, value in zip(
                    ["x", "y", "z", "w"],
                    BoardTransformWaypointBuilder.normalize_quaternion(TransformMath.multiply(p_q, c_q)),
                )
            },
        }


class TransformLogger:
    """Log transforms involved in generating the target waypoint."""

    @staticmethod
    def info(name: str, transform: Dict[str, Any]) -> None:
        """Log transform translation and quaternion in a compact, consistent format."""
        translation = TransformMath.vector3(transform["translation"], f"{name}.translation")
        rotation = TransformMath.quaternion(transform["rotation"], f"{name}.rotation")
        LOGGER.info(
            "%s: xyz=(%.4f, %.4f, %.4f), quat=(%.5f, %.5f, %.5f, %.5f)",
            name,
            translation[0],
            translation[1],
            translation[2],
            rotation[0],
            rotation[1],
            rotation[2],
            rotation[3],
        )


class CameraOpticalTransformBuilder:
    """Build the fixed right_camera_link<-OpenCV optical camera transform."""

    def __init__(self, settings: VisionAndMoveSettings):
        """Store settings used to build the camera optical transform."""
        self.settings = settings

    def build(self) -> Dict[str, Any]:
        """Return the fixed camera link to optical camera transform."""
        roll, pitch, yaw = self.settings.camera_link_to_optical_rpy
        quaternion = HandEyeCalibrationLoader.rpy_to_quaternion(roll, pitch, yaw)
        return {
            "translation": {
                "x": self.settings.camera_link_to_optical_xyz[0],
                "y": self.settings.camera_link_to_optical_xyz[1],
                "z": self.settings.camera_link_to_optical_xyz[2],
            },
            "rotation": {
                key: value
                for key, value in zip(
                    ["x", "y", "z", "w"],
                    BoardTransformWaypointBuilder.normalize_quaternion(quaternion),
                )
            },
        }


class BoardTransformWaypointBuilder:
    """Build a right-arm cartesian waypoint from a board transform."""

    def __init__(self, settings: VisionAndMoveSettings):
        """Store immutable settings used to generate the waypoint."""
        self.settings = settings

    def build(self, transform: Dict[str, Any], orientation_transform: Dict[str, Any] | None = None) -> Dict[str, Any]:
        """Return a cartesian waypoint above the detected board position."""
        translation = transform["translation"]
        rotation = transform["rotation"]
        orientation = self._select_orientation(rotation, orientation_transform)

        return {
            "group": self.settings.group,
            "planner": self.settings.planner,
            "description": (
                f"{self.settings.waypoint_name}: "
                f"{self.settings.waypoint_frame} <- board, "
                f"z_offset={self.settings.approach_height_m:.3f}m"
            ),
            "type": "cart",
            "max_velocity_scaling_factor": self.settings.max_velocity_scaling_factor,
            "max_acceleration_scaling_factor": self.settings.max_acceleration_scaling_factor,
            "ik_frame": self.settings.ik_frame,
            "frame_id": self.settings.waypoint_frame,
            "position": [
                float(translation["x"]),
                float(translation["y"]),
                float(translation["z"]) + self.settings.approach_height_m,
            ],
            "orientation": orientation,
        }

    def _select_orientation(
        self,
        rotation: Dict[str, float],
        orientation_transform: Dict[str, Any] | None,
    ) -> List[float]:
        """Select and normalize the target orientation quaternion."""
        if self.settings.orientation_mode == "fixed":
            return self.normalize_quaternion(self.settings.fixed_orientation)
        if self.settings.orientation_mode == "ik_frame_in_reference_frame":
            if orientation_transform is None:
                raise ValueError("orientation_transform is required when orientation_mode is ik_frame_in_reference_frame")
            return self.normalize_quaternion(
                TransformMath.quaternion(orientation_transform["rotation"], "orientation_transform.rotation")
            )
        if self.settings.orientation_mode != "board_frame":
            raise ValueError(f"Unsupported orientation_mode: {self.settings.orientation_mode}")

        return self.normalize_quaternion(
            [
                float(rotation["x"]),
                float(rotation["y"]),
                float(rotation["z"]),
                float(rotation["w"]),
            ]
        )

    @staticmethod
    def normalize_quaternion(quaternion: List[float]) -> List[float]:
        """Normalize a quaternion and reject invalid values."""
        if len(quaternion) != 4:
            raise ValueError("Quaternion must have 4 elements: [x, y, z, w]")

        norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
        if norm <= 1e-9:
            raise ValueError("Quaternion norm is zero")
        return [float(value) / norm for value in quaternion]


class QueryBoardPoseAndWriteWaypoint(py_trees.behaviour.Behaviour):
    """Query board pose from calibration ZMQ and write a generated waypoint."""

    def __init__(
        self,
        blackboard_manager: BlackboardManager,
        tf_client: TFClient,
        calibration_client: CalibrationBoardPoseClient,
        handeye_loader: HandEyeCalibrationLoader,
        settings: VisionAndMoveSettings,
        name: str = "QueryBoardPoseAndWriteWaypoint",
    ):
        """Initialize the one-shot board pose query behavior."""
        super().__init__(name=name)
        self.blackboard_manager = blackboard_manager
        self.tf_client = tf_client
        self.calibration_client = calibration_client
        self.handeye_loader = handeye_loader
        self.settings = settings
        self.waypoint_builder = BoardTransformWaypointBuilder(settings)
        self.camera_optical_builder = CameraOpticalTransformBuilder(settings)
        self.generated_waypoint: Dict[str, Any] = {}

    def setup(self, **kwargs: Any) -> None:
        """Register write access for the waypoint blackboard key."""
        if "arm_waypoints_data" not in self.blackboard_manager.registered_keys:
            self.blackboard_manager.register_key("arm_waypoints_data", AccessType.WRITE)

    def initialise(self) -> None:
        """Clear any previous generated waypoint."""
        self.generated_waypoint = {}

    def update(self) -> py_trees.common.Status:
        """Query board pose, transform it to arm base, and write the waypoint."""
        try:
            camera_board_pose = self.calibration_client.get_board_pose()
            LOGGER.debug("Board pose response from calibration ZMQ: %s", camera_board_pose)
            base_tool = self.tf_client.lookup_transform(
                source_frame=self.settings.source_frame,
                target_frame=self.settings.end_effector_frame,
            )
            if not base_tool or not base_tool.get("success", False):
                message = base_tool.get("message", "No response") if base_tool else "No response"
                raise RuntimeError(f"Failed to query end-effector transform: {message}")
            waypoint_source = self.tf_client.lookup_transform(
                source_frame=self.settings.waypoint_frame,
                target_frame=self.settings.source_frame,
            )
            if not waypoint_source or not waypoint_source.get("success", False):
                message = waypoint_source.get("message", "No response") if waypoint_source else "No response"
                raise RuntimeError(f"Failed to query waypoint frame transform: {message}")

            reference_ik = self.tf_client.lookup_transform(
                source_frame=self.settings.orientation_reference_frame,
                target_frame=self.settings.ik_frame,
            )
            if not reference_ik or not reference_ik.get("success", False):
                message = reference_ik.get("message", "No response") if reference_ik else "No response"
                raise RuntimeError(f"Failed to query target orientation transform: {message}")

            position = TransformMath.vector3(camera_board_pose["position"], "board_pose.position")
            quaternion = TransformMath.quaternion(camera_board_pose["quaternion"], "board_pose.quaternion")
            optical_board = {
                "translation": {
                    "x": position[0],
                    "y": position[1],
                    "z": position[2],
                },
                "rotation": {
                    "x": quaternion[0],
                    "y": quaternion[1],
                    "z": quaternion[2],
                    "w": quaternion[3],
                },
            }
            tool_camera = self.handeye_loader.load_transform()
            camera_optical = self.camera_optical_builder.build()
            tool_optical = TransformMath.compose(tool_camera, camera_optical)
            base_camera = TransformMath.compose(base_tool["data"], tool_camera)
            base_optical = TransformMath.compose(base_tool["data"], tool_optical)
            base_board = TransformMath.compose(base_optical, optical_board)
            waypoint_board = TransformMath.compose(waypoint_source["data"], base_board)
            TransformLogger.info(f"{self.settings.source_frame} <- {self.settings.end_effector_frame}", base_tool["data"])
            TransformLogger.info("right_tool0 <- right_camera_link", tool_camera)
            TransformLogger.info(f"right_camera_link <- {self.settings.camera_optical_frame}", camera_optical)
            TransformLogger.info(f"{self.settings.camera_optical_frame} <- charuco", optical_board)
            TransformLogger.info(f"{self.settings.source_frame} <- right_camera_link", base_camera)
            TransformLogger.info(f"{self.settings.source_frame} <- {self.settings.camera_optical_frame}", base_optical)
            TransformLogger.info(f"{self.settings.source_frame} <- charuco", base_board)
            TransformLogger.info(f"{self.settings.waypoint_frame} <- {self.settings.source_frame}", waypoint_source["data"])
            TransformLogger.info(f"{self.settings.waypoint_frame} <- charuco", waypoint_board)
            TransformLogger.info(
                f"{self.settings.orientation_reference_frame} <- {self.settings.ik_frame}",
                reference_ik["data"],
            )

            self.generated_waypoint = self.waypoint_builder.build(waypoint_board, reference_ik["data"])
            waypoints_data = self.blackboard_manager.get("arm_waypoints_data", {})
            updated_waypoints = dict(waypoints_data)
            updated_waypoints[self.settings.waypoint_name] = self.generated_waypoint
            self.blackboard_manager.set("arm_waypoints_data", updated_waypoints)
        except Exception as exc:
            LOGGER.exception("Failed to build or store generated waypoint: %s", exc)
            return py_trees.common.Status.FAILURE

        position = self.generated_waypoint["position"]
        LOGGER.info(
            "Generated waypoint %s in %s: position=(%.4f, %.4f, %.4f)",
            self.settings.waypoint_name,
            self.settings.waypoint_frame,
            position[0],
            position[1],
            position[2],
        )
        return py_trees.common.Status.SUCCESS


class VisionAndMoveDemo:
    """Application object for the right-arm vision-and-move demo."""

    def __init__(self, config_path: Path, waypoints_path: Path, show_tree: bool, dry_run: bool):
        """Create the demo with file paths and execution flags."""
        self.config_path = config_path
        self.waypoints_path = waypoints_path
        self.show_tree = show_tree
        self.dry_run = dry_run
        self.manager: BehaviorTreeManager | None = None
        self.tf_client: TFClient | None = None
        self.calibration_client: CalibrationBoardPoseClient | None = None
        self.handeye_loader: HandEyeCalibrationLoader | None = None

    def run(self) -> bool:
        """Initialize clients, build behaviors, and execute the demo."""
        self.manager = BehaviorTreeManager(
            config_path=str(self.config_path),
            waypoints_path=str(self.waypoints_path),
            show_progress=True,
            show_tree=self.show_tree,
        )

        try:
            settings = VisionAndMoveSettings.from_config(self.manager.config)
            tf_config = self.manager.config.get("zmq", {}).get("tf", {})
            self.tf_client = TFClient(
                server_ip=tf_config.get("host", "192.168.56.122"),
                server_port=int(tf_config.get("port", 5609)),
                timeout=int(tf_config.get("timeout", 5)),
            )
            calibration_config = self.manager.config.get("zmq", {}).get("calibration", {})
            self.calibration_client = CalibrationBoardPoseClient(
                host=calibration_config.get("host", "192.168.56.122"),
                port=int(calibration_config.get("port", 7021)),
                timeout=int(calibration_config.get("timeout", 10)),
            )

            behaviors = self._build_behaviors(settings)
            return self.manager.execute(behaviors, wait=True)
        finally:
            self._cleanup()

    def _build_behaviors(self, settings: VisionAndMoveSettings) -> List[py_trees.behaviour.Behaviour]:
        """Build the behavior sequence for querying vision and moving the arm."""
        if self.manager is None or self.tf_client is None or self.calibration_client is None:
            raise RuntimeError("Demo is not initialized")

        self.handeye_loader = HandEyeCalibrationLoader(settings.handeye_calibration_file)
        query_board = QueryBoardPoseAndWriteWaypoint(
            blackboard_manager=self.manager.blackboard_manager,
            tf_client=self.tf_client,
            calibration_client=self.calibration_client,
            handeye_loader=self.handeye_loader,
            settings=settings,
        )

        if self.dry_run:
            return [query_board]

        move_right_arm = self.manager.arm_move_behavior.move_to_waypoints(
            [(settings.waypoint_name, settings.max_velocity_scaling_factor, settings.max_acceleration_scaling_factor)],
            name="MoveRightArmAboveCharucoBoard",
            use_remote_execution=True,
            concurrent_remote_execution=False,
        )
        return [query_board, move_right_arm]

    def _cleanup(self) -> None:
        """Close clients owned by this demo."""
        if self.tf_client is not None:
            self.tf_client.close()
            self.tf_client = None
        if self.calibration_client is not None:
            self.calibration_client.close()
            self.calibration_client = None
        if self.manager is not None:
            self.manager.cleanup()
            self.manager = None


def parse_args() -> argparse.Namespace:
    """Parse command line arguments for the demo."""
    parser = argparse.ArgumentParser(
        description="Query the ChArUco board frame and move the right arm above it.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml")
    parser.add_argument("--waypoints", type=Path, default=THIS_DIR / "waypoints.json")
    parser.add_argument("--show-tree", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="query TF and generate the waypoint without moving")
    return parser.parse_args()


def configure_logging(config_path: Path) -> None:
    """Configure console logging before the BehaviorTreeManager starts."""
    level_name = "INFO"
    try:
        with config_path.open("r", encoding="utf-8") as file_obj:
            config = yaml.safe_load(file_obj) or {}
        level_name = config.get("logging", {}).get("level", "INFO")
    except Exception:
        pass

    logging.basicConfig(
        level=getattr(logging, level_name.upper(), logging.INFO),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )


def main() -> int:
    """Run the demo and return a process exit code."""
    args = parse_args()
    args.config = args.config.resolve()
    args.waypoints = args.waypoints.resolve()
    configure_logging(args.config)

    demo = VisionAndMoveDemo(
        config_path=args.config,
        waypoints_path=args.waypoints,
        show_tree=args.show_tree,
        dry_run=args.dry_run,
    )
    return 0 if demo.run() else 1


if __name__ == "__main__":
    raise SystemExit(main())
