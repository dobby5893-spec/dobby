#!/usr/bin/env python3
import math

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class GoalToleranceProbe(Node):
    def __init__(self):
        super().__init__("codex_goal_tolerance_probe")
        self.pose = None
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def on_odom(self, msg):
        self.pose = msg.pose.pose


rclpy.init()
node = GoalToleranceProbe()
deadline = node.get_clock().now().nanoseconds + int(5e9)
while node.pose is None and node.get_clock().now().nanoseconds < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if node.pose is None:
    raise RuntimeError("No /odom pose received")

x = node.pose.position.x
y = node.pose.position.y
current_yaw = yaw_from_quaternion(node.pose.orientation)
target_yaw = current_yaw + 0.35

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = x
goal.pose.pose.position.y = y
goal.pose.pose.orientation.z = math.sin(target_yaw / 2.0)
goal.pose.pose.orientation.w = math.cos(target_yaw / 2.0)

if not node.client.wait_for_server(timeout_sec=5.0):
    raise RuntimeError("NavigateToPose action server unavailable")

sent = node.client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, sent, timeout_sec=5.0)
handle = sent.result()
if handle is None or not handle.accepted:
    raise RuntimeError("Goal rejected")

result_future = handle.get_result_async()
rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
wrapped = result_future.result()

if wrapped is None:
    cancel_future = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
    print(
        f"timeout: start=({x:.3f},{y:.3f},{current_yaw:.3f}) "
        f"target_yaw={target_yaw:.3f}; goal canceled"
    )
else:
    print(
        f"status={wrapped.status} succeeded={wrapped.status == GoalStatus.STATUS_SUCCEEDED} "
        f"start=({x:.3f},{y:.3f},{current_yaw:.3f}) target_yaw={target_yaw:.3f} "
        f"yaw_delta=0.350"
    )

node.destroy_node()
rclpy.shutdown()
