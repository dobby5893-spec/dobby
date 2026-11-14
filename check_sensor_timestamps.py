#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class TimestampProbe(Node):
    def __init__(self):
        super().__init__("codex_timestamp_probe")
        self.records = {"lidar": [], "imu": []}
        self.frames = {"lidar": set(), "imu": set()}
        self.point_counts = []
        self.lidar_sub = self.create_subscription(
            PointCloud2, "/livox/lidar", self.on_lidar, qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.on_imu, qos_profile_sensor_data
        )

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_lidar(self, msg):
        self.records["lidar"].append(self.stamp(msg))
        self.frames["lidar"].add(msg.header.frame_id)
        self.point_counts.append(msg.width * msg.height)

    def on_imu(self, msg):
        self.records["imu"].append(self.stamp(msg))
        self.frames["imu"].add(msg.header.frame_id)


def summarize(name, stamps, frames):
    if len(stamps) < 2:
        print(f"{name}: only {len(stamps)} samples frames={sorted(frames)}")
        return
    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    backwards = [(i, d) for i, d in enumerate(deltas) if d < 0.0]
    duplicates = sum(abs(d) < 1e-12 for d in deltas)
    print(
        f"{name}: samples={len(stamps)} frames={sorted(frames)} "
        f"first={stamps[0]:.9f} last={stamps[-1]:.9f} "
        f"min_dt={min(deltas):.9f} max_dt={max(deltas):.9f} "
        f"backwards={len(backwards)} duplicates={duplicates}"
    )
    for index, delta in backwards[:10]:
        print(
            f"  backwards[{index}]: {stamps[index]:.9f} -> "
            f"{stamps[index + 1]:.9f}, dt={delta:.9f}"
        )


rclpy.init()
node = TimestampProbe()
end = time.monotonic() + 20.0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)

summarize("lidar", node.records["lidar"], node.frames["lidar"])
summarize("imu", node.records["imu"], node.frames["imu"])
if node.point_counts:
    print(
        f"lidar points: min={min(node.point_counts)} max={max(node.point_counts)} "
        f"average={sum(node.point_counts) / len(node.point_counts):.1f}"
    )
node.destroy_node()
rclpy.shutdown()
