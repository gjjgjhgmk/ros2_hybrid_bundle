#!/usr/bin/env python3
"""
Real-hardware dual-arm concurrent remote execution demo.

This demo keeps planning as one ur_move request per stage, with both arm groups
in the same request. In remote execution mode, the planned left/right
trajectories are then sent concurrently to the two executor servers.
"""

import argparse
import logging
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
sys.path.insert(0, str(UR_BT_DIR / "src"))

from ur_bt import BehaviorTreeManager  # noqa: E402


class ConcurrentRemoteDualArmDemo:
    """Runs a small dual-arm motion sequence using concurrent remote execution."""

    def __init__(self, config_path: Path, waypoints_path: Path, show_tree: bool = False):
        self.config_path = config_path
        self.waypoints_path = waypoints_path
        self.show_tree = show_tree
        self.velocity = 0.1
        self.acceleration = 0.1
        self.manager = None

    def run(self) -> bool:
        self.manager = BehaviorTreeManager(
            config_path=str(self.config_path),
            waypoints_path=str(self.waypoints_path),
            show_progress=True,
            show_tree=self.show_tree,
        )

        try:
            logging.info("Starting concurrent remote dual-arm demo")
            return self.manager.execute(self._build_behaviors(), wait=True)
        finally:
            self.manager.cleanup()

    def _build_behaviors(self):
        arm_move = self.manager.arm_move_behavior
        sleep = self.manager.utility_behavior.sleep

        return [
            self._move_both(arm_move, "left_home", "right_home", "real_both_home"),
            sleep(1.0, name="real_settle_home"),
            self._move_both(arm_move, "left_goal1", "right_goal1", "real_both_goal1"),
            sleep(1.0, name="real_settle_goal1"),
            self._move_both(arm_move, "left_goal2", "right_goal2", "real_both_goal2"),
            sleep(1.0, name="real_settle_goal2"),
            self._move_both(arm_move, "left_home", "right_home", "real_both_return_home"),
        ]

    def _move_both(self, arm_move, left_waypoint: str, right_waypoint: str, name: str):
        # Keep both groups in one planning request. The config controls whether
        # the resulting trajectories are sent concurrently to 5660/5661.
        return arm_move.move_to_waypoints(
            [
                (left_waypoint, self.velocity, self.acceleration),
                (right_waypoint, self.velocity, self.acceleration),
            ],
            name=name,
            use_remote_execution=True,
            concurrent_remote_execution=True,
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Run real dual-arm concurrent remote execution demo")
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml")
    parser.add_argument("--waypoints", type=Path, default=THIS_DIR / "waypoints.json")
    parser.add_argument("--show-tree", action="store_true")
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()
    demo = ConcurrentRemoteDualArmDemo(args.config.resolve(), args.waypoints.resolve(), args.show_tree)
    return 0 if demo.run() else 1


if __name__ == "__main__":
    raise SystemExit(main())
