#!/usr/bin/env python3
import argparse
import shlex
import subprocess
import sys
from pathlib import Path
from typing import List, Optional

from intent_hybrid_planner.intent_hybrid_evaluator import main as evaluator_main


def main(args: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-planner", action="store_true")
    parser.add_argument(
        "--planner-output",
        default=str(Path.cwd() / "evaluation_inputs" / "offline_eval_input_latest.json"),
    )
    parser.add_argument("--config", default="")
    parser.add_argument("--out-dir", default=str(Path.cwd() / "evaluation" / "plan_and_eval_latest"))
    parser.add_argument("--enable-plot", action="store_true")
    parser.add_argument("--enable-fk", action="store_true")
    parser.add_argument("--require-collision-backend", default="true")
    parser.add_argument("--dryrun-allow-missing-backend", default="false")
    parser.add_argument("--collision-service-timeout-sec", default="3.0")
    parser.add_argument("--planner-timeout-sec", type=float, default=180.0)
    parser.add_argument(
        "--planner-extra",
        default="",
        help="Extra planner args, for example: \"--ros-args -p execution_mode:=offline\"",
    )
    ns = parser.parse_args(args=args)

    if ns.run_planner:
        cmd = ["ros2", "run", "intent_hybrid_planner", "intent_hybrid_planner_node"]
        # Keep a demo-friendly offline profile unless the caller overrides it.
        default_planner_args = [
            "--ros-args",
            "-p",
            "runtime_backend:=cpp_bridge",
            "-p",
            "execution_mode:=offline",
            "-p",
            "hybrid_mode:=matlab_compat",
            "-p",
            "offline_export_eval_input_enable:=true",
            "-p",
            "action_path_tolerance_rad:=0.5",
            "-p",
            "action_goal_tolerance_rad:=0.2",
            "-p",
            "action_goal_time_tolerance_sec:=5.0",
            "-p",
            "nominal_dt:=0.12",
        ]
        cmd.extend(default_planner_args)
        if ns.planner_extra.strip():
            extra_tokens = shlex.split(ns.planner_extra)
            if extra_tokens and extra_tokens[0] == "--ros-args":
                extra_tokens = extra_tokens[1:]
            cmd.extend(extra_tokens)
        print(f"[run_plan_and_eval] running planner: {' '.join(cmd)}", flush=True)
        subprocess.run(cmd, check=True, timeout=max(float(ns.planner_timeout_sec), 1.0))

    eval_args = [
        "--out-dir",
        ns.out_dir,
        "--require-collision-backend",
        str(ns.require_collision_backend),
        "--dryrun-allow-missing-backend",
        str(ns.dryrun_allow_missing_backend),
        "--collision-service-timeout-sec",
        str(ns.collision_service_timeout_sec),
    ]
    if ns.config:
        eval_args.extend(["--config", ns.config])
    else:
        eval_args.extend(["--planner-output", ns.planner_output])
    if ns.enable_plot:
        eval_args.append("--enable-plot")
    if ns.enable_fk:
        eval_args.append("--enable-fk")

    print(f"[run_plan_and_eval] running evaluator: intent_hybrid_evaluator {' '.join(eval_args)}", flush=True)
    evaluator_main(eval_args)


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f"[run_plan_and_eval] planner failed: {exc}", file=sys.stderr)
        raise SystemExit(exc.returncode)
