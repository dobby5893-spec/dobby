"""Resolve a fresh map/world alignment before launching floor navigation."""
from collections import deque
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

from .checkpoint_store import world_fingerprint
from .continuous_ground import continuous_ground_region, verified_slab_world
from .fastlio_tf_bridge import (compose_transform, inverse_transform,
                               interpolate_world_pose, rotate_vector)
from .resume_alignment import StableAlignment


def transform_tuple(transform):
    p, q = transform.translation, transform.rotation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class GroundPriorResolver(Node):
    def __init__(self):
        super().__init__('ground_prior_resolver')
        self.world = self.declare_parameter('world', '').value
        self.output = Path(self.declare_parameter('output', '').value)
        if not verified_slab_world(self.world) or not str(self.output):
            raise ValueError('Fresh slab alignment requires the verified world and output path')
        self.timeout = self.declare_parameter('timeout_s', 60.).value
        self.started = time.monotonic()
        self.done = False
        self.failed = False
        self.last_log = 0.
        self.history = deque(maxlen=3000)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.gate = StableAlignment(minimum_age=2., window=1., distance=.015, angle=.02)
        self.create_subscription(Odometry, '/odom/ground_truth',
                                 self.history.append, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=rclpy.clock.Clock())

    def tick(self):
        if self.done or self.failed:
            return
        wall_now = time.monotonic()
        if wall_now-self.started > self.timeout:
            self.failed = True
            self.get_logger().error('Ground alignment timed out; navigation remains unstarted')
            return
        try:
            body = self.tf.lookup_transform('map', 'base_link', Time())
            stamp = body.header.stamp.sec*1000000000+body.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)*1e-9
            if not -.1 <= age <= .5:
                raise ValueError('Waiting for a fresh mapping TF')
            world = interpolate_world_pose(self.history, stamp)
            if world is None:
                raise ValueError('Waiting for synchronized Gazebo odometry')
            stance = transform_tuple(self.tf.lookup_transform(
                'base_footprint', 'base_link', Time.from_msg(body.header.stamp)).transform)
            world_feet = compose_transform(world, inverse_transform(stance))
            if abs(world_feet[0][2]) > .10 or rotate_vector(world[1], (0, 0, 1))[2] < math.cos(.12):
                raise ValueError('Robot is not standing on the verified first-floor slab')
            previous = next((m for m in reversed(self.history)
                             if m.header.stamp.sec*1000000000+m.header.stamp.nanosec <= stamp-500000000), None)
            if previous is None:
                raise ValueError('Waiting for stationary world odometry history')
            p = previous.pose.pose.position
            if math.dist(world[0], (p.x, p.y, p.z)) > .025:
                self.gate = StableAlignment(minimum_age=2., window=1., distance=.015, angle=.02)
                raise ValueError('Waiting for stationary startup')
            mapped = transform_tuple(body.transform)
            alignment = compose_transform(mapped, inverse_transform(world))
            if not self.gate.add(stamp*1e-9, alignment):
                raise ValueError('Waiting for stable map/world alignment')
            state = dict(map_body=mapped, world_body=world,
                         world_sha256=world_fingerprint(self.world),
                         stamp_ns=stamp, source='fresh-synchronized-alignment')
            region = continuous_ground_region(self.world, state)
            if not region:
                raise ValueError('Verified first-floor geometry is unavailable')
            temporary = self.output.with_suffix('.tmp')
            temporary.write_text(json.dumps(state, allow_nan=False))
            os.replace(temporary, self.output)
            self.done = True
            self.get_logger().info('Fresh first-floor alignment verified; navigation can load the slab prior')
        except Exception as error:
            if wall_now-self.last_log > 5.:
                self.get_logger().info(str(error))
                self.last_log = wall_now


def main():
    rclpy.init()
    node = GroundPriorResolver()
    try:
        while rclpy.ok() and not node.done and not node.failed:
            rclpy.spin_once(node, timeout_sec=.1)
        success = node.done
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)
