#!/usr/bin/env python3
"""
Compare against the single-request demo by sending multiple waypoint requests.

Each request still contains both arm groups. The difference is that goal1 and
goal2 are sent as separate ur_move requests:
  request 1: left_goal1 + right_goal1
  request 2: left_goal2 + right_goal2
"""

import argparse
import logging
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
sys.path.insert(0, str(UR_BT_DIR / "src"))

from ur_bt import BehaviorTreeManager  # noqa: E402


class MultiRequestDualArmDemo:
    """Runs two dual-arm stages as two separate ur_move waypoint requests."""

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
            behaviors = self._build_behaviors()
            logging.info("Starting multi-request dual-arm simulation demo")
            return self.manager.execute(behaviors, wait=True)
        finally:
            self.manager.cleanup()

    def _build_behaviors(self):
        # Each move_to_waypoints call below sends one request containing both
        # groups. The stages are separate, so ur_move completes goal1 before it
        # receives goal2.
        arm_move = self.manager.arm_move_behavior
        sleep = self.manager.utility_behavior.sleep

        return [
            arm_move.move_to_waypoints(
                [
                    ("left_goal1", self.velocity, self.acceleration),
                    ("right_goal1", self.velocity, self.acceleration),
                ],
                name="multi_request_both_goal1",
                use_remote_execution=False,
            ),
            sleep(1.0, name="multi_request_settle_goal1"),
            arm_move.move_to_waypoints(
                [
                    ("left_goal2", self.velocity, self.acceleration),
                    ("right_goal2", self.velocity, self.acceleration),
                ],
                name="multi_request_both_goal2",
                use_remote_execution=False,
            ),
            sleep(1.0, name="multi_request_settle_goal2"),
            arm_move.move_to_waypoints(
                [
                    ("left_home", self.velocity, self.acceleration),
                    ("right_home", self.velocity, self.acceleration),
                ],
                name="multi_request_both_return_home",
                use_remote_execution=False,
            ),
        ]


def parse_args():
    parser = argparse.ArgumentParser(description="Run sequential multi-request dual-arm demo")
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml")
    parser.add_argument("--waypoints", type=Path, default=THIS_DIR / "waypoints.json")
    parser.add_argument("--show-tree", action="store_true")
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()
    demo = MultiRequestDualArmDemo(args.config.resolve(), args.waypoints.resolve(), args.show_tree)
    return 0 if demo.run() else 1


if __name__ == "__main__":
    raise SystemExit(main())
