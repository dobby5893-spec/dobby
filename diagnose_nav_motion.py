#!/usr/bin/env python3
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class MotionProbe(Node):
    def __init__(self):
        super().__init__("codex_motion_probe")
        self.samples = {"nav": [], "smooth": [], "odom": [], "gazebo": []}
        self.nav_sub = self.create_subscription(Twist, "/cmd_vel_nav", self.on_nav, 50)
        self.smooth_sub = self.create_subscription(Twist, "/cmd_vel", self.on_smooth, 50)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.truth_sub = self.create_subscription(
            Odometry, "/odom/ground_truth", self.on_truth, 50
        )

    def on_nav(self, msg):
        self.samples["nav"].append((msg.linear.x, msg.angular.z))

    def on_smooth(self, msg):
        self.samples["smooth"].append((msg.linear.x, msg.angular.z))

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.samples["odom"].append((p.x, p.y, yaw(msg.pose.pose.orientation)))

    def on_truth(self, msg):
        p = msg.pose.pose.position
        self.samples["gazebo"].append((p.x, p.y, yaw(msg.pose.pose.orientation)))


def pose_summary(label, values):
    if not values:
        print(f"{label}: no samples")
        return
    first, last = values[0], values[-1]
    distance = math.hypot(last[0] - first[0], last[1] - first[1])
    angle = math.atan2(math.sin(last[2] - first[2]), math.cos(last[2] - first[2]))
    print(
        f"{label}: count={len(values)} first=({first[0]:.3f},{first[1]:.3f},{first[2]:.3f}) "
        f"last=({last[0]:.3f},{last[1]:.3f},{last[2]:.3f}) "
        f"delta_distance={distance:.3f} delta_yaw={angle:.3f}"
    )


def velocity_summary(label, values):
    if not values:
        print(f"{label}: no samples")
        return
    nonzero = sum(abs(x) > 1e-4 or abs(z) > 1e-4 for x, z in values)
    print(
        f"{label}: count={len(values)} nonzero={nonzero} "
        f"linear=[{min(x for x, _ in values):.3f},{max(x for x, _ in values):.3f}] "
        f"angular=[{min(z for _, z in values):.3f},{max(z for _, z in values):.3f}]"
    )


rclpy.init()
node = MotionProbe()
end = time.monotonic() + float(os.environ.get("PROBE_SECONDS", "15"))
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

velocity_summary("cmd_vel_nav", node.samples["nav"])
velocity_summary("cmd_vel", node.samples["smooth"])
pose_summary("odom", node.samples["odom"])
pose_summary("gazebo", node.samples["gazebo"])
node.destroy_node()
rclpy.shutdown()
