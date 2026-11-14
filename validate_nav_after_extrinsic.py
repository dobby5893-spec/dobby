#!/usr/bin/env python3
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Validation(Node):
    def __init__(self):
        super().__init__("codex_nav_extrinsic_validation")
        self.fastlio = []
        self.truth = []
        self.fastlio_sub = self.create_subscription(
            Odometry, "/fastlio/odom", self.on_fastlio, 50
        )
        self.truth_sub = self.create_subscription(
            Odometry, "/odom/ground_truth", self.on_truth, 50
        )
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    @staticmethod
    def sample(msg):
        p = msg.pose.pose.position
        return (p.x, p.y, p.z, yaw(msg.pose.pose.orientation))

    def on_fastlio(self, msg):
        self.fastlio.append(self.sample(msg))

    def on_truth(self, msg):
        self.truth.append(self.sample(msg))

    def stop(self):
        msg = Twist()
        for _ in range(20):
            self.stop_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)


def delta(values):
    first, last = values[0], values[-1]
    return (
        math.hypot(last[0] - first[0], last[1] - first[1]),
        last[2] - first[2],
        math.atan2(math.sin(last[3] - first[3]), math.cos(last[3] - first[3])),
    )


rclpy.init()
node = Validation()
wait_end = time.monotonic() + 8.0
while (not node.fastlio or not node.truth) and time.monotonic() < wait_end:
    rclpy.spin_once(node, timeout_sec=0.1)
if not node.fastlio or not node.truth:
    raise RuntimeError("Missing /fastlio/odom or /odom/ground_truth")

start = node.fastlio[-1]
target_x = start[0] + 0.8 * math.cos(start[3])
target_y = start[1] + 0.8 * math.sin(start[3])

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = target_x
goal.pose.pose.position.y = target_y
goal.pose.pose.orientation.z = math.sin(start[3] / 2.0)
goal.pose.pose.orientation.w = math.cos(start[3] / 2.0)

if not node.client.wait_for_server(timeout_sec=8.0):
    raise RuntimeError("NavigateToPose server unavailable")
sent = node.client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, sent, timeout_sec=8.0)
handle = sent.result()
if handle is None or not handle.accepted:
    raise RuntimeError("NavigateToPose goal rejected or response timed out")

result_future = handle.get_result_async()
end = time.monotonic() + 50.0
while not result_future.done() and time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

if not result_future.done():
    cancel = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel, timeout_sec=5.0)
    status = "TIMEOUT_CANCELED"
else:
    wrapped = result_future.result()
    status = f"{wrapped.status} ({'SUCCEEDED' if wrapped.status == GoalStatus.STATUS_SUCCEEDED else 'NOT_SUCCEEDED'})"

node.stop()
fast_d = delta(node.fastlio)
truth_d = delta(node.truth)
print(
    f"goal=({target_x:.3f},{target_y:.3f}) status={status}\n"
    f"FAST_LIO delta_xy={fast_d[0]:.3f} delta_z={fast_d[1]:.3f} delta_yaw={fast_d[2]:.3f}\n"
    f"Gazebo  delta_xy={truth_d[0]:.3f} delta_z={truth_d[1]:.3f} delta_yaw={truth_d[2]:.3f}\n"
    f"translation_delta_error={abs(fast_d[0] - truth_d[0]):.3f}"
)
node.destroy_node()
rclpy.shutdown()
