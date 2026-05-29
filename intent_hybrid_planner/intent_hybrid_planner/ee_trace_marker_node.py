#!/usr/bin/env python3
import argparse
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class EETraceMarkerNode(Node):
    def __init__(self, max_points: int, frame_id: str, ee_link: str, use_tf: bool) -> None:
        super().__init__("ee_trace_marker")
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self._name_to_idx = {}
        self._points: List[Point] = []
        self._max_points = max(10, int(max_points))
        self._frame_id = frame_id
        self._ee_link = ee_link
        self._use_tf = bool(use_tf)
        self._tf_warned = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        vis_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, "/planning_vis", vis_qos)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.get_logger().info(
            "EE trace marker started: "
            f"frame={self._frame_id}, ee_link={self._ee_link}, use_tf={self._use_tf}, "
            f"max_points={self._max_points}, topic=/planning_vis"
        )

    @staticmethod
    def _fk_wrist_fallback(q: np.ndarray) -> Point:
        # Fallback-only approximate FK (kept for degraded mode when TF is unavailable).
        L1, L2, L3 = 0.163, 0.479, 0.392
        q1, q2, q3 = q[0], q[1], q[2]
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c23, s23 = np.cos(q2 + q3), np.sin(q2 + q3)
        p = Point()
        p.x = float(c1 * (L2 * c2 + L3 * c23))
        p.y = float(s1 * (L2 * c2 + L3 * c23))
        p.z = float(L1 + L2 * s2 + L3 * s23)
        return p

    def _lookup_ee_tf(self) -> Optional[Point]:
        if not self._use_tf:
            return None
        try:
            tfm = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._ee_link,
                Time(),
            )
            p = Point()
            p.x = float(tfm.transform.translation.x)
            p.y = float(tfm.transform.translation.y)
            p.z = float(tfm.transform.translation.z)
            return p
        except (LookupException, ConnectivityException, ExtrapolationException):
            if not self._tf_warned:
                self.get_logger().warn(
                    f"TF lookup failed for {self._frame_id}->{self._ee_link}, using FK fallback."
                )
                self._tf_warned = True
            return None

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        if not self._name_to_idx:
            self._name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in self._name_to_idx for n in self.joint_names):
            return
        idx = [self._name_to_idx[n] for n in self.joint_names]
        if len(msg.position) < max(idx) + 1:
            return
        q = np.array([msg.position[i] for i in idx], dtype=float)
        p = self._lookup_ee_tf()
        if p is None:
            p = self._fk_wrist_fallback(q)
        self._points.append(p)
        if len(self._points) > self._max_points:
            self._points = self._points[-self._max_points :]
        self._publish()

    def _publish(self) -> None:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "planning_vis"
        marker.id = 5
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.008
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.9
        marker.points = list(self._points)
        msg = MarkerArray()
        msg.markers.append(marker)
        self._pub.publish(msg)


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--max-points", type=int, default=1200)
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--ee-link", default="tool0")
    parser.add_argument("--disable-tf", action="store_true")
    ns, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = EETraceMarkerNode(
        max_points=ns.max_points,
        frame_id=ns.frame_id,
        ee_link=ns.ee_link,
        use_tf=(not ns.disable_tf),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
