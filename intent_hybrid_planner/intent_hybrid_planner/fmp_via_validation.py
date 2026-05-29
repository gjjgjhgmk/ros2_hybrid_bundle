#!/usr/bin/env python3
"""
Layered validation runner for FMP via-point test flow.

Levels:
- l1: pure algorithm sanity check (no ROS2 runtime dependency).
- l2: ROS2 node-level smoke check (action server may be absent).
- l3: empty-sim execution check (requires controller/action server, expects PASS).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from . import fmp_core


def _nominal():
    n = 150
    t = np.linspace(0.0, 10.0, n)
    start_q = np.array([-0.8, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=float)
    end_q = np.array([0.8, -1.57, 1.57, -1.57, -1.57, 0.0], dtype=float)
    blend = np.linspace(0.0, 1.0, n, dtype=float)
    traj = start_q[:, None] + (end_q - start_q)[:, None] * blend[None, :]
    return traj, t


def _run_l1() -> int:
    traj, t = _nominal()
    model = fmp_core.train_fmp_model(traj, t, N_C=20, alpha=0.1)

    out0 = fmp_core.modulate_trajectory(
        fmp_model=model,
        demo_traj=traj,
        time_axis=t,
        via_points=np.empty((traj.shape[0], 0)),
        via_times=np.empty((0,)),
    )
    idxs = [45, 95]
    via_points = traj[:, idxs].copy()
    via_points[0, 0] += 0.14
    via_points[2, 1] -= 0.09
    raw = t[idxs]
    scaled = raw * float(model["demo_dura"])
    out1 = fmp_core.modulate_trajectory(model, traj, t, via_points, raw)
    out2 = fmp_core.modulate_trajectory(model, traj, t, via_points, scaled)

    ok = (
        out0.shape == traj.shape
        and out1.shape == traj.shape
        and out2.shape == traj.shape
        and np.isfinite(out0).all()
        and np.isfinite(out1).all()
        and np.isfinite(out2).all()
        and np.max(np.abs(out1 - out2)) <= 1e-8
    )
    if ok:
        print("[L1][PASS] FMP algorithm quick checks passed.")
        return 0
    print("[L1][FAIL] FMP algorithm quick checks failed.")
    return 1


def _read_latest_result(result_dir: Path) -> dict:
    files = sorted(result_dir.glob("fmp_via_test_*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not files:
        return {}
    return json.loads(files[0].read_text(encoding="utf-8"))


def _run_ros2_node(timeout_sec: float) -> int:
    cmd = ["ros2", "run", "intent_hybrid_planner", "fmp_via_test_node"]
    try:
        subprocess.run(cmd, check=False, timeout=timeout_sec)
    except FileNotFoundError:
        print("[ROS2][FAIL] ros2 command not found.")
        return 2
    except subprocess.TimeoutExpired:
        print(f"[ROS2][FAIL] ros2 run timed out at {timeout_sec:.1f}s.")
        return 3
    return 0


def _run_l2(result_dir: Path, timeout_sec: float) -> int:
    rc = _run_ros2_node(timeout_sec)
    if rc != 0:
        return rc
    time.sleep(0.5)
    result = _read_latest_result(result_dir)
    if not result:
        print("[L2][FAIL] No result json generated.")
        return 4
    status = str(result.get("status", ""))
    if status in {"PASS", "FAIL_ACTION_SERVER_NOT_READY", "FAIL_NO_JOINT_STATE"}:
        print(f"[L2][PASS] Node smoke completed with status={status}.")
        return 0
    print(f"[L2][FAIL] Unexpected status={status}.")
    return 5


def _run_l3(result_dir: Path, timeout_sec: float) -> int:
    rc = _run_ros2_node(timeout_sec)
    if rc != 0:
        return rc
    time.sleep(0.5)
    result = _read_latest_result(result_dir)
    if not result:
        print("[L3][FAIL] No result json generated.")
        return 6
    status = str(result.get("status", ""))
    if status == "PASS":
        print("[L3][PASS] Empty-sim execution passed.")
        return 0
    print(f"[L3][FAIL] Expected PASS, got status={status}.")
    return 7


def main() -> None:
    parser = argparse.ArgumentParser(description="FMP via validation runner.")
    parser.add_argument("--level", choices=["l1", "l2", "l3"], required=True)
    parser.add_argument("--result-dir", default="result")
    parser.add_argument("--timeout-sec", type=float, default=30.0)
    args = parser.parse_args()

    result_dir = Path.cwd() / args.result_dir
    result_dir.mkdir(parents=True, exist_ok=True)

    if args.level == "l1":
        sys.exit(_run_l1())
    if args.level == "l2":
        sys.exit(_run_l2(result_dir=result_dir, timeout_sec=args.timeout_sec))
    sys.exit(_run_l3(result_dir=result_dir, timeout_sec=args.timeout_sec))


if __name__ == "__main__":
    main()
