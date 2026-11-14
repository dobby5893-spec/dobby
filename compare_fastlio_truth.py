#!/usr/bin/env python3
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Comparison(Node):
    def __init__(self):
        super().__init__("codex_fastlio_truth_comparison")
        self.data = {"fastlio": [], "truth": []}
        self.fastlio_sub = self.create_subscription(
            Odometry, "/Odometry", self.on_fastlio, 50
        )
        self.truth_sub = self.create_subscription(
            Odometry, "/odom/ground_truth", self.on_truth, 50
        )

    def add(self, key, msg):
        p = msg.pose.pose.position
        self.data[key].append((p.x, p.y, p.z, yaw(msg.pose.pose.orientation)))

    def on_fastlio(self, msg):
        self.add("fastlio", msg)

    def on_truth(self, msg):
        self.add("truth", msg)


def summarize(name, values):
    if not values:
        print(f"{name}: no samples")
        return
    first, last = values[0], values[-1]
    dxy = math.hypot(last[0] - first[0], last[1] - first[1])
    dz = last[2] - first[2]
    dyaw = math.atan2(math.sin(last[3] - first[3]), math.cos(last[3] - first[3]))
    print(
        f"{name}: n={len(values)} first=({first[0]:.3f},{first[1]:.3f},{first[2]:.3f},{first[3]:.3f}) "
        f"last=({last[0]:.3f},{last[1]:.3f},{last[2]:.3f},{last[3]:.3f}) "
        f"delta_xy={dxy:.4f} delta_z={dz:.4f} delta_yaw={dyaw:.4f}"
    )


rclpy.init()
node = Comparison()
end = time.monotonic() + float(os.environ.get("PROBE_SECONDS", "30"))
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

summarize("FAST_LIO raw", node.data["fastlio"])
summarize("Gazebo truth", node.data["truth"])
node.destroy_node()
rclpy.shutdown()
