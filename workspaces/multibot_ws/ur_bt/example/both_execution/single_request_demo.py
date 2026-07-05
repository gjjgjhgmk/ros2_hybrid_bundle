#!/usr/bin/env python3
"""
Send one waypoint request containing both arm groups and both target stages.

The main request contains left_goal1, right_goal1, left_goal2 and right_goal2.
ur_move groups them by arm, concatenates each arm's two waypoints, then executes
the two group trajectories in parallel in local simulation mode.
"""

import argparse
import logging
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
sys.path.insert(0, str(UR_BT_DIR / "src"))

from ur_bt import BehaviorTreeManager  # noqa: E402


class SingleRequestDualArmDemo:
    """Runs two dual-arm stages as one combined ur_move waypoint request."""

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
            logging.info("Starting single-request dual-arm simulation demo")
            return self.manager.execute(behaviors, wait=True)
        finally:
            self.manager.cleanup()

    def _build_behaviors(self):
        # The four waypoints below are sent in one request. ur_move turns them
        # into left_goal1->left_goal2 and right_goal1->right_goal2 trajectories.
        arm_move = self.manager.arm_move_behavior
        sleep = self.manager.utility_behavior.sleep

        return [
            arm_move.move_to_waypoints(
                [
                    ("left_home", self.velocity, self.acceleration),
                    ("right_home", self.velocity, self.acceleration),
                ],
                name="single_request_both_home",
                use_remote_execution=False,
            ),
            sleep(1.0, name="single_request_settle_home"),
            arm_move.move_to_waypoints(
                [
                    ("left_goal1", self.velocity, self.acceleration),
                    ("right_goal1", self.velocity, self.acceleration),
                    ("left_goal2", self.velocity, self.acceleration),
                    ("right_goal2", self.velocity, self.acceleration),
                ],
                name="single_request_both_goal1_goal2",
                use_remote_execution=False,
            ),
            sleep(1.0, name="single_request_settle_goals"),
            arm_move.move_to_waypoints(
                [
                    ("left_home", self.velocity, self.acceleration),
                    ("right_home", self.velocity, self.acceleration),
                ],
                name="single_request_both_return_home",
                use_remote_execution=False,
            ),
        ]


def parse_args():
    parser = argparse.ArgumentParser(description="Run single-request dual-arm demo")
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml")
    parser.add_argument("--waypoints", type=Path, default=THIS_DIR / "waypoints.json")
    parser.add_argument("--show-tree", action="store_true")
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()
    demo = SingleRequestDualArmDemo(args.config.resolve(), args.waypoints.resolve(), args.show_tree)
    return 0 if demo.run() else 1


if __name__ == "__main__":
    raise SystemExit(main())
