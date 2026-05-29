#!/usr/bin/env python3
import argparse
from datetime import datetime
from pathlib import Path
import time
from typing import Dict, Optional

import matplotlib
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import MarkerArray

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def _points_to_xyz(marker) -> np.ndarray:
    if not marker.points:
        return np.zeros((0, 3), dtype=float)
    out = np.zeros((len(marker.points), 3), dtype=float)
    for i, p in enumerate(marker.points):
        out[i, 0] = float(p.x)
        out[i, 1] = float(p.y)
        out[i, 2] = float(p.z)
    return out


class PlanningVisSnapshot(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("planning_vis_snapshot")
        self._markers: Dict[int, object] = {}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MarkerArray, topic, self._cb, qos)

    def _cb(self, msg: MarkerArray) -> None:
        for m in msg.markers:
            self._markers[int(m.id)] = m

    def wait_until_ready(self, timeout_sec: float, require_planner: bool) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if require_planner:
                if (0 in self._markers) and (2 in self._markers):
                    return True
            elif len(self._markers) > 0:
                return True
        return False

    def marker_xyz(self, marker_id: int) -> np.ndarray:
        m = self._markers.get(marker_id)
        if m is None:
            return np.zeros((0, 3), dtype=float)
        return _points_to_xyz(m)


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/planning_vis")
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument(
        "--capture-sec",
        type=float,
        default=10.0,
        help="Keep collecting markers for this duration after first valid markers.",
    )
    parser.add_argument("--output", default="/home/woody/simple_fmp_v1/png")
    parser.add_argument("--allow-fallback", action="store_true")
    parser.add_argument("--min-span", type=float, default=0.25)
    ns, ros_args = parser.parse_known_args(args=args)

    out_path = Path(ns.output).expanduser()
    if out_path.suffix.lower() != ".png":
        out_path.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = out_path / f"planning_vis_snapshot_{stamp}.png"
    else:
        out_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = PlanningVisSnapshot(topic=ns.topic)
    try:
        if not node.wait_until_ready(
            timeout_sec=max(ns.timeout_sec, 0.5),
            require_planner=(not ns.allow_fallback),
        ):
            node.get_logger().error("No planning_vis markers received in time.")
            return
        capture_sec = max(float(ns.capture_sec), 0.0)
        if capture_sec > 0.0:
            deadline = time.monotonic() + capture_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)

        nominal = node.marker_xyz(0)
        via = node.marker_xyz(1)
        modulated = node.marker_xyz(2)
        nominal_ee = node.marker_xyz(3)
        obstacles = node.marker_xyz(4)
        actual_ee = node.marker_xyz(5)
        node.get_logger().info(
            "Marker points count: "
            f"nominal={nominal.shape[0]}, via={via.shape[0]}, modulated={modulated.shape[0]}, "
            f"nominal_ee={nominal_ee.shape[0]}, obstacles={obstacles.shape[0]}, actual_ee={actual_ee.shape[0]}"
        )
        if nominal.shape == modulated.shape and nominal.shape[0] > 0:
            max_delta = float(np.max(np.linalg.norm(modulated - nominal, axis=1)))
            node.get_logger().info(f"Nominal vs modulated max point delta: {max_delta:.6f} m")
        if nominal.shape[0] == 0 or modulated.shape[0] == 0:
            if not ns.allow_fallback:
                node.get_logger().error(
                    "Planner markers (id=0/2) not found. Re-run with snapshot process already running before offline planner, "
                    "or use --allow-fallback."
                )
                return
            node.get_logger().warn(
                "Nominal/modulated markers not found in this snapshot; plotting available marker traces only."
            )

        fig, ax = plt.subplots(figsize=(8, 6), dpi=140)
        if modulated.shape[0] > 0:
            ax.plot(
                modulated[:, 0],
                modulated[:, 1],
                color="#169c22",
                linewidth=2.5,
                alpha=0.9,
                linestyle="-",
                label="modulated",
                zorder=2,
            )
            ax.scatter([modulated[0, 0]], [modulated[0, 1]], c="g", s=30)
            ax.scatter([modulated[-1, 0]], [modulated[-1, 1]], c="g", s=30, marker="s")
        if nominal.shape[0] > 0:
            ax.plot(
                nominal[:, 0],
                nominal[:, 1],
                color="#1f56d6",
                linewidth=2.1,
                alpha=0.95,
                linestyle="--",
                label="nominal",
                zorder=3,
            )
            ax.scatter([nominal[0, 0]], [nominal[0, 1]], c="b", s=30)
            ax.scatter([nominal[-1, 0]], [nominal[-1, 1]], c="b", s=30, marker="s")
        if nominal_ee.shape[0] > 0:
            ax.plot(
                nominal_ee[:, 0],
                nominal_ee[:, 1],
                color="#ff9900",
                linewidth=2.0,
                alpha=0.85,
                label="nominal_ee",
            )
        if via.shape[0] > 0:
            ax.scatter(via[:, 0], via[:, 1], c="red", s=24, label="via_points")
        if obstacles.shape[0] > 0:
            ax.scatter(obstacles[:, 0], obstacles[:, 1], c="gold", s=60, marker="x", label="plane_obstacles")
        if actual_ee.shape[0] > 0:
            ax.plot(actual_ee[:, 0], actual_ee[:, 1], "m-", linewidth=1.6, alpha=0.8, label="actual_ee_trace")

        xy_blocks = []
        for arr in (nominal, modulated, nominal_ee, via, obstacles, actual_ee):
            if arr.shape[0] > 0:
                xy_blocks.append(arr[:, :2])
        if xy_blocks:
            all_xy = np.vstack(xy_blocks)
            x_min, y_min = np.min(all_xy, axis=0)
            x_max, y_max = np.max(all_xy, axis=0)
            cx = 0.5 * (x_min + x_max)
            cy = 0.5 * (y_min + y_max)
            span_x = max(float(x_max - x_min), float(ns.min_span))
            span_y = max(float(y_max - y_min), float(ns.min_span))
            ax.set_xlim(cx - 0.5 * span_x, cx + 0.5 * span_x)
            ax.set_ylim(cy - 0.5 * span_y, cy + 0.5 * span_y)

        ax.set_aspect("equal", adjustable="box")
        ax.ticklabel_format(style="plain", useOffset=False)
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Hybrid Planning Trajectory Snapshot")
        ax.legend(loc="best")
        fig.tight_layout()
        fig.savefig(str(out_path))
        node.get_logger().info(f"Saved snapshot figure: {out_path}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
