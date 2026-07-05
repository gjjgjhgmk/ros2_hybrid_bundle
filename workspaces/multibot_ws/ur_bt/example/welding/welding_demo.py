#!/usr/bin/env python3
"""Run a toy dual-arm welding-point tracking demo.

This demo intentionally ignores target.csv. It builds a small, conservative
dual-arm welding sequence in each arm's own interface frame and sends both arm
groups in the same motion request for every stage.
"""

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
sys.path.insert(0, str(UR_BT_DIR / "src"))


ARM_INFO = {
    "left_arm": {
        "home": "左臂-home",
        "label": "left",
        "ik_frame": "left_ee_link",
        "interface_frame": "left_interface_link",
    },
    "right_arm": {
        "home": "右臂-home",
        "label": "right",
        "ik_frame": "right_ee_link",
        "interface_frame": "right_interface_link",
    },
}


@dataclass(frozen=True)
class WeldingPoint:
    arm: str
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class WeldingStage:
    index: int
    left: WeldingPoint
    right: WeldingPoint


# Conservative built-in welding sequence. The two arms stay in separate local
# work patches: left uses positive y and higher z, right uses negative y and
# lower z. This avoids relying on dual-arm collision avoidance in the server.
DEFAULT_WELDING_SEQUENCE = [
    WeldingStage(
        index=1,
        left=WeldingPoint("left_arm", 0.36, 0.22, 0.34),
        right=WeldingPoint("right_arm", 0.36, -0.22, 0.22),
    ),
    WeldingStage(
        index=2,
        left=WeldingPoint("left_arm", 0.40, 0.22, 0.34),
        right=WeldingPoint("right_arm", 0.40, -0.22, 0.22),
    ),
    WeldingStage(
        index=3,
        left=WeldingPoint("left_arm", 0.44, 0.22, 0.36),
        right=WeldingPoint("right_arm", 0.44, -0.22, 0.24),
    ),
    WeldingStage(
        index=4,
        left=WeldingPoint("left_arm", 0.48, 0.22, 0.36),
        right=WeldingPoint("right_arm", 0.48, -0.22, 0.24),
    ),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Track a built-in dual-arm welding-point sequence with ur_bt.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml", help="ur_bt config path")
    parser.add_argument(
        "--base-waypoints",
        type=Path,
        default=THIS_DIR.parent / "waypoints.json",
        help="base waypoints containing home poses",
    )
    parser.add_argument(
        "--generated-waypoints",
        type=Path,
        default=THIS_DIR / "generated_waypoints.json",
        help="generated waypoint json path",
    )
    parser.add_argument("--planner", choices=["ptp", "lin", "ompl"], default="lin", help="planner for weld points")
    parser.add_argument("--approach-planner", choices=["ptp", "lin", "ompl"], default="ptp", help="planner for approach points")
    parser.add_argument("--approach-z", type=float, default=0.08, help="z offset for approach points, meters")
    parser.add_argument("--dwell", type=float, default=0.2, help="simulated welding dwell time, seconds")
    parser.add_argument("--vel", type=float, default=0.05, help="velocity scaling")
    parser.add_argument("--acc", type=float, default=0.05, help="acceleration scaling")
    parser.add_argument("--home-vel", type=float, default=0.2, help="home velocity scaling")
    parser.add_argument("--home-acc", type=float, default=0.2, help="home acceleration scaling")
    parser.add_argument(
        "--orientation",
        default="0,0,0,1",
        help="ee orientation quaternion as x,y,z,w",
    )
    parser.add_argument("--dry-run", action="store_true", help="generate waypoints and print summary only")
    parser.add_argument("--step", action="store_true", help="wait for Enter before every dual-arm stage")
    parser.add_argument("--skip-home", action="store_true", help="do not move both arms to home before/after welding")
    return parser.parse_args()


def parse_orientation(value: str) -> List[float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 4:
        raise ValueError("--orientation must contain four comma-separated values: x,y,z,w")
    return [float(part) for part in parts]


def waypoint_name(stage: WeldingStage, point: WeldingPoint, suffix: str = "") -> str:
    label = ARM_INFO[point.arm]["label"]
    return f"weld_stage_{stage.index:02d}_{label}{suffix}"


def make_cart_waypoint(
    name: str,
    point: WeldingPoint,
    planner: str,
    position: Iterable[float],
    orientation: List[float],
    vel: float,
    acc: float,
) -> Dict[str, object]:
    return {
        "group": point.arm,
        "planner": planner,
        "description": name,
        "type": "cart",
        "max_velocity_scaling_factor": vel,
        "max_acceleration_scaling_factor": acc,
        "ik_frame": ARM_INFO[point.arm]["ik_frame"],
        "frame_id": ARM_INFO[point.arm]["interface_frame"],
        "position": list(position),
        "orientation": orientation,
    }


def generate_waypoints(args: argparse.Namespace, stages: List[WeldingStage]) -> Dict[str, Dict[str, object]]:
    with args.base_waypoints.open("r", encoding="utf-8") as f:
        waypoints = json.load(f)

    orientation = parse_orientation(args.orientation)
    for stage in stages:
        for point in (stage.left, stage.right):
            approach_name = waypoint_name(stage, point, "_approach")
            weld_name = waypoint_name(stage, point)
            waypoints[approach_name] = make_cart_waypoint(
                name=approach_name,
                point=point,
                planner=args.approach_planner,
                position=(point.x, point.y, point.z + args.approach_z),
                orientation=orientation,
                vel=args.vel,
                acc=args.acc,
            )
            waypoints[weld_name] = make_cart_waypoint(
                name=weld_name,
                point=point,
                planner=args.planner,
                position=(point.x, point.y, point.z),
                orientation=orientation,
                vel=args.vel,
                acc=args.acc,
            )
    return waypoints


def save_waypoints(path: Path, waypoints: Dict[str, Dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(waypoints, f, ensure_ascii=False, indent=2)
        f.write("\n")


def print_summary(stages: List[WeldingStage], generated_path: Path) -> None:
    print(f"Loaded {len(stages)} built-in dual-arm welding stages")
    print(f"Generated waypoints: {generated_path}")
    print("Execution order:")
    for stage in stages:
        left_frame = ARM_INFO[stage.left.arm]["interface_frame"]
        right_frame = ARM_INFO[stage.right.arm]["interface_frame"]
        print(
            f"  stage {stage.index:02d}: "
            f"left {left_frame}:({stage.left.x:.3f}, {stage.left.y:.3f}, {stage.left.z:.3f}) | "
            f"right {right_frame}:({stage.right.x:.3f}, {stage.right.y:.3f}, {stage.right.z:.3f})"
        )


def move_both_home(manager, args: argparse.Namespace, name: str):
    return manager.arm_move_behavior.move_to_waypoints(
        [
            (ARM_INFO["left_arm"]["home"], args.home_vel, args.home_acc),
            (ARM_INFO["right_arm"]["home"], args.home_vel, args.home_acc),
        ],
        name=name,
        concurrent_remote_execution=True,
    )


def execute_demo(args: argparse.Namespace, stages: List[WeldingStage]) -> None:
    from ur_bt import BehaviorTreeManager

    manager = BehaviorTreeManager(
        config_path=str(args.config),
        waypoints_path=str(args.generated_waypoints),
        show_progress=True,
        show_tree=False,
    )

    try:
        if not args.skip_home:
            if not manager.execute([move_both_home(manager, args, "welding_demo_move_both_arms_home")], wait=True):
                raise RuntimeError("Failed to move arms to home")

        behaviors = []
        for stage in stages:
            if args.step:
                behaviors.append(manager.utility_behavior.wait_for_input(f"Press Enter to execute welding stage {stage.index:02d}"))

            behaviors.append(
                manager.arm_move_behavior.move_to_waypoints(
                    [
                        (waypoint_name(stage, stage.left, "_approach"), args.vel, args.acc),
                        (waypoint_name(stage, stage.right, "_approach"), args.vel, args.acc),
                    ],
                    name=f"welding_demo_stage_{stage.index:02d}_approach_both",
                    concurrent_remote_execution=True,
                )
            )
            behaviors.append(
                manager.arm_move_behavior.move_to_waypoints(
                    [
                        (waypoint_name(stage, stage.left), args.vel, args.acc),
                        (waypoint_name(stage, stage.right), args.vel, args.acc),
                    ],
                    name=f"welding_demo_stage_{stage.index:02d}_weld_both",
                    concurrent_remote_execution=True,
                )
            )
            behaviors.append(manager.utility_behavior.sleep(args.dwell, name=f"welding_demo_stage_{stage.index:02d}_dwell"))

        if not manager.execute(behaviors, wait=True):
            raise RuntimeError("Welding target tracking failed")

        if not args.skip_home:
            if not manager.execute([move_both_home(manager, args, "welding_demo_return_both_arms_home")], wait=True):
                raise RuntimeError("Failed to return arms to home")
    finally:
        manager.cleanup()


def main() -> int:
    args = parse_args()
    args.config = args.config.resolve()
    args.base_waypoints = args.base_waypoints.resolve()
    args.generated_waypoints = args.generated_waypoints.resolve()

    stages = DEFAULT_WELDING_SEQUENCE
    waypoints = generate_waypoints(args, stages)
    save_waypoints(args.generated_waypoints, waypoints)
    print_summary(stages, args.generated_waypoints)

    if args.dry_run:
        return 0

    execute_demo(args, stages)
    print("Welding demo completed successfully")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
