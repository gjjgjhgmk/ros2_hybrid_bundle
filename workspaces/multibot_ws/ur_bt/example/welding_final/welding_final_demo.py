#!/usr/bin/env python3
"""Run the final dual-arm welding waypoint sequence.

The demo loads final_waypoints.json and executes paired left/right welding
points as one planning request per stage. In remote mode, ur_bt then sends the
left/right trajectories to their executor servers concurrently.
"""

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import List, Tuple


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
sys.path.insert(0, str(UR_BT_DIR / "src"))


HOME_PAIR = ("left_home", "right_home")
WELD_PAIRS: List[Tuple[str, str]] = [
    ("left_weld_03", "right_weld_05"),
    ("left_weld_seed", "right_weld_seed"),
    ("left_weld_05", "right_weld_02"),
    ("left_weld_02", "right_weld_04"),
    ("left_weld_04", "right_weld_03"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Execute final dual-arm welding waypoint sequence",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml")
    parser.add_argument("--waypoints", type=Path, default=THIS_DIR / "final_waypoints.json")
    parser.add_argument("--vel", type=float, default=0.05, help="welding velocity scaling")
    parser.add_argument("--acc", type=float, default=0.05, help="welding acceleration scaling")
    parser.add_argument("--home-vel", type=float, default=0.2, help="home velocity scaling")
    parser.add_argument("--home-acc", type=float, default=0.2, help="home acceleration scaling")
    parser.add_argument("--dwell", type=float, default=0.2, help="pause after each weld stage, seconds")
    parser.add_argument("--skip-home", action="store_true", help="do not move to home before/after welding")
    parser.add_argument(
        "--step",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="wait for Enter before each target trajectory",
    )
    parser.add_argument("--dry-run", action="store_true", help="validate and print the sequence without executing")
    parser.add_argument("--show-tree", action="store_true", help="print behavior tree status while running")
    return parser.parse_args()


def load_waypoints(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def validate_waypoints(waypoints: dict) -> None:
    required_names = [*HOME_PAIR]
    for left_name, right_name in WELD_PAIRS:
        required_names.extend([left_name, right_name])

    missing = [name for name in required_names if name not in waypoints]
    if missing:
        raise ValueError(f"Missing waypoint(s): {', '.join(missing)}")


def print_sequence() -> None:
    print("Execution sequence:")
    print(f"  home: {HOME_PAIR[0]} + {HOME_PAIR[1]}")
    for index, (left_name, right_name) in enumerate(WELD_PAIRS, start=1):
        print(f"  weld stage {index:02d}: {left_name} + {right_name}")
    print(f"  return: {HOME_PAIR[0]} + {HOME_PAIR[1]}")


class WeldingFinalDemo:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.manager = None

    def run(self) -> bool:
        from ur_bt import BehaviorTreeManager

        self.manager = BehaviorTreeManager(
            config_path=str(self.args.config),
            waypoints_path=str(self.args.waypoints),
            show_progress=True,
            show_tree=self.args.show_tree,
        )

        try:
            behaviors = self._build_behaviors()
            return self.manager.execute(behaviors, wait=True)
        finally:
            self.manager.cleanup()

    def _build_behaviors(self):
        behaviors = []
        arm_move = self.manager.arm_move_behavior
        utility = self.manager.utility_behavior

        if not self.args.skip_home:
            self._append_target_move(
                behaviors,
                utility,
                arm_move,
                "Move to home",
                HOME_PAIR[0],
                HOME_PAIR[1],
                "welding_final_move_home",
                self.args.home_vel,
                self.args.home_acc,
            )
            behaviors.append(utility.sleep(0.5, name="welding_final_settle_home"))

        for index, (left_name, right_name) in enumerate(WELD_PAIRS, start=1):
            self._append_target_move(
                behaviors,
                utility,
                arm_move,
                f"Execute weld stage {index:02d}",
                left_name,
                right_name,
                f"welding_final_stage_{index:02d}",
                self.args.vel,
                self.args.acc,
            )
            if self.args.dwell > 0:
                behaviors.append(utility.sleep(self.args.dwell, name=f"welding_final_stage_{index:02d}_dwell"))

        if not self.args.skip_home:
            self._append_target_move(
                behaviors,
                utility,
                arm_move,
                "Return to home",
                HOME_PAIR[0],
                HOME_PAIR[1],
                "welding_final_return_home",
                self.args.home_vel,
                self.args.home_acc,
            )

        return behaviors

    def _append_target_move(
        self,
        behaviors,
        utility,
        arm_move,
        prompt: str,
        left_waypoint: str,
        right_waypoint: str,
        name: str,
        vel: float,
        acc: float,
    ) -> None:
        if self.args.step:
            behaviors.append(
                utility.wait_for_input(
                    f"{prompt}: {left_waypoint} + {right_waypoint}. Press Enter to execute trajectory."
                )
            )
        behaviors.append(self._move_both(arm_move, left_waypoint, right_waypoint, name, vel, acc))

    @staticmethod
    def _move_both(arm_move, left_waypoint: str, right_waypoint: str, name: str, vel: float, acc: float):
        return arm_move.move_to_waypoints(
            [
                (left_waypoint, vel, acc),
                (right_waypoint, vel, acc),
            ],
            name=name,
            concurrent_remote_execution=True,
        )


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()
    args.config = args.config.resolve()
    args.waypoints = args.waypoints.resolve()

    validate_waypoints(load_waypoints(args.waypoints))
    print_sequence()

    if args.dry_run:
        return 0

    demo = WeldingFinalDemo(args)
    return 0 if demo.run() else 1


if __name__ == "__main__":
    raise SystemExit(main())
