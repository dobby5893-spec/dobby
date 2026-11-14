#!/usr/bin/env python3
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


CANDIDATES = [
    ("near_corridor_a", -3.04, -1.19),
    ("near_corridor_b", -3.13, -2.06),
    ("corridor_y3", -3.20, -3.00),
    ("corridor_y4", -3.20, -4.00),
    ("corridor_y5", -3.20, -5.00),
    ("corridor_y6", -3.20, -6.00),
    ("corridor_y7", -3.20, -7.00),
    ("corridor_y8", -3.20, -8.00),
    ("corridor_inside_edge", -3.20, -8.80),
    ("deep_corridor", -3.25, -9.35),
]


class PathProbe(Node):
    def __init__(self):
        super().__init__("codex_corridor_path_probe")
        self.client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def test(self, name, x, y):
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = "GridBased"
        goal.use_start = False

        if not self.client.wait_for_server(timeout_sec=5.0):
            print(f"{name}: action server unavailable")
            return
        sent = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=10.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            print(f"{name}: rejected")
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        wrapped = result_future.result()
        if wrapped is None:
            print(f"{name}: timed out")
            return
        path = wrapped.result.path
        print(
            f"{name}: status={wrapped.status} succeeded={wrapped.status == GoalStatus.STATUS_SUCCEEDED} "
            f"poses={len(path.poses)} target=({x:.2f},{y:.2f})"
        )


rclpy.init()
node = PathProbe()
for candidate in CANDIDATES:
    node.test(*candidate)
node.destroy_node()
rclpy.shutdown()
