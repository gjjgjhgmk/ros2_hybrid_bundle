#!/usr/bin/env python3
"""Open or close the left/right grippers from the welding_final config."""

import argparse
import logging
import sys
from pathlib import Path
from typing import Dict

import yaml


THIS_DIR = Path(__file__).resolve().parent
UR_BT_DIR = THIS_DIR.parents[1]
UR_BT_GRIPPER_CLIENT_DIR = UR_BT_DIR / "src" / "ur_bt" / "clients" / "gripper"
sys.path.insert(0, str(UR_BT_GRIPPER_CLIENT_DIR))

from gripper_zmq_client import GripperZMQClient  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control left/right gripper open and close actions",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("action", choices=("open", "close"), help="gripper action")
    parser.add_argument("--gripper", choices=("left", "right", "both"), default="both")
    parser.add_argument("--config", type=Path, default=THIS_DIR / "config.yaml")
    parser.add_argument("--max-effort", type=float, default=50.0, help="maximum gripper effort in N")
    return parser.parse_args()


def load_gripper_config(config_path: Path) -> Dict[str, Dict[str, object]]:
    with config_path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}
    gripper_config = config.get("zmq", {}).get("gripper", {})
    if not gripper_config:
        raise RuntimeError(f"No zmq.gripper config found in {config_path}")
    return gripper_config


def make_client(gripper_config: Dict[str, Dict[str, object]], gripper: str) -> GripperZMQClient:
    config = gripper_config.get(gripper)
    if not config:
        raise RuntimeError(f"No config found for {gripper} gripper")

    return GripperZMQClient(
        server_host=str(config.get("host", "127.0.0.1")),
        port=int(config.get("port", 5630 if gripper == "left" else 5640)),
        gripper_name=gripper,
        timeout_ms=int(config.get("timeout_ms", 20000)),
    )


def run_action(client: GripperZMQClient, action: str, max_effort: float) -> bool:
    try:
        if action == "open":
            return client.open(max_effort=max_effort)
        if action == "close":
            return client.close(max_effort=max_effort)
        raise ValueError(f"Unsupported action: {action}")
    finally:
        client.close_connection()


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    args = parse_args()
    gripper_config = load_gripper_config(args.config.resolve())
    grippers = ("left", "right") if args.gripper == "both" else (args.gripper,)

    results = []
    for gripper in grippers:
        client = make_client(gripper_config, gripper)
        success = run_action(client, args.action, args.max_effort)
        results.append(success)
        print(f"{gripper} gripper {args.action}: {'success' if success else 'failed'}")

    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
