import copy
import math
import json
from pathlib import Path
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, DurabilityPolicy
from .resume_alignment import StableAlignment
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster


def quaternion_product(a, b):
    x, y, z, w = a
    u, v, t, s = b
    return (w*u+x*s+y*t-z*v, w*v-x*t+y*s+z*u,
            w*t+x*v-y*u+z*s, w*s-x*u-y*v-z*t)


def rotate_vector(q, vector):
    inverse = (-q[0], -q[1], -q[2], q[3])
    return quaternion_product(
        quaternion_product(q, (*vector, 0.0)), inverse)[:3]


def compose_transform(a, b):
    shifted = rotate_vector(a[1], b[0])
    return (tuple(a[0][i]+shifted[i] for i in range(3)),
            quaternion_product(a[1], b[1]))


def inverse_transform(transform):
    p, q = transform
    inverse_q = (-q[0], -q[1], -q[2], q[3])
    return (rotate_vector(inverse_q, tuple(-v for v in p)), inverse_q)


def pose_transform(pose):
    p, q = pose.position, pose.orientation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


def interpolate_world_pose(history, stamp_ns):
    ns = lambda m: m.header.stamp.sec*1000000000+m.header.stamp.nanosec
    before = next((m for m in reversed(history) if ns(m) <= stamp_ns), None)
    after = next((m for m in history if ns(m) >= stamp_ns), None)
    if before is None or after is None:
        return None
    t0, t1 = ns(before), ns(after)
    if max(stamp_ns-t0, t1-stamp_ns) > 150000000:
        return None
    a, b = pose_transform(before.pose.pose), pose_transform(after.pose.pose)
    fraction = (stamp_ns-t0)/(t1-t0) if t1 != t0 else 0.
    sign = 1 if sum(x*y for x,y in zip(a[1],b[1])) >= 0 else -1
    q = [(1-fraction)*x+fraction*sign*y for x,y in zip(a[1],b[1])]
    norm = math.sqrt(sum(v*v for v in q))
    return (tuple((1-fraction)*x+fraction*y for x,y in zip(a[0],b[0])), tuple(v/norm for v in q))


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def quaternion_from_yaw(yaw):
    half_yaw = yaw / 2.0

    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


def world_twist_to_body(world_twist, yaw):
    """Convert P3D world-frame twist to the planar base_footprint frame."""
    result = copy.deepcopy(world_twist)
    c, s = math.cos(yaw), math.sin(yaw)
    for vector in (result.twist.linear, result.twist.angular):
        x, y = vector.x, vector.y
        vector.x = c * x + s * y
        vector.y = -s * x + c * y
    rotation = [
        [c, s, 0.0, 0.0, 0.0, 0.0],
        [-s, c, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, c, s, 0.0],
        [0.0, 0.0, 0.0, -s, c, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    ]
    covariance = world_twist.covariance
    result.covariance = [
        sum(rotation[i][k] * covariance[6 * k + l] * rotation[j][l]
            for k in range(6) for l in range(6))
        for i in range(6) for j in range(6)
    ]
    return result


class FastLioTfBridge(Node):
    def __init__(self):
        super().__init__("fastlio_tf_bridge")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.latest_local_odom = None
        self.initial_local_z = None
        self.latest_local_foot = None
        self.local_pending = deque(maxlen=300)
        self.fast_pending = deque(maxlen=40)
        self.world_history = deque(maxlen=3000)
        resume_path = self.declare_parameter('resume_state', '').value
        self.resume = json.loads(Path(resume_path).read_text()) if resume_path else None
        self.alignment_gate = StableAlignment()
        self.last_alignment_log = -1e9
        self.alignment_publisher = self.create_publisher(Header, '/fastlio/map_alignment_ready',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.map_anchor = None if self.resume else ((0., 0., 0.), (0., 0., 0., 1.))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if self.map_anchor is not None:
            self.publish_map_anchor()
        self.retry_timer = self.create_timer(0.01, self.retry_pending, clock=rclpy.clock.Clock())

        self.local_odom_subscription = self.create_subscription(
            Odometry,
            "/odom/ground_truth",
            self.local_odom_callback,
            10,
        )

        self.fastlio_odom_subscription = self.create_subscription(
            Odometry,
            "/Odometry",
            self.fastlio_odom_callback,
            10,
        )
        self.nav_odom_publisher = self.create_publisher(
            Odometry,
            "/odom",
            10,
        )

        self.fastlio_odom_publisher = self.create_publisher(
            Odometry,
            "/fastlio/odom",
            10,
        )

    def local_odom_callback(self, message):
        self.local_pending.append(message)
        self.world_history.append(message)

    def fastlio_odom_callback(self, message):
        self.fast_pending.append(message)

    def publish_map_anchor(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'camera_init'
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.map_anchor[0]
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = self.map_anchor[1]
        self.static_tf_broadcaster.sendTransform(transform)
        ready = Header()
        ready.frame_id = 'map'
        ready.stamp = self.get_clock().now().to_msg()
        self.alignment_publisher.publish(ready)

    def retry_pending(self):
        for queue, callback in ((self.local_pending, self.process_local), (self.fast_pending, self.process_fastlio)):
            for _ in range(50):
                if not queue:
                    break
                message = queue[0]
                age = (self.get_clock().now()-rclpy.time.Time.from_msg(message.header.stamp)).nanoseconds*1e-9
                if age > 2.0 or age < -0.1:
                    queue.popleft()
                    continue
                if not callback(message):
                    break
                queue.popleft()

    def process_local(self, message):
        try:
            stance = self.tf_buffer.lookup_transform(
                'base_footprint', 'base_link', rclpy.time.Time.from_msg(message.header.stamp)).transform
        except Exception:
            return False
        foot_to_body = ((stance.translation.x, stance.translation.y, stance.translation.z),
                        (stance.rotation.x, stance.rotation.y, stance.rotation.z, stance.rotation.w))
        world_foot = compose_transform(
            pose_transform(message.pose.pose), inverse_transform(foot_to_body))
        local_position = copy.deepcopy(message.pose.pose.position)
        local_position.x, local_position.y, local_position.z = world_foot[0]
        local_orientation = copy.deepcopy(message.pose.pose.orientation)
        local_orientation.x, local_orientation.y, local_orientation.z, local_orientation.w = world_foot[1]
        self.latest_local_odom = message
        if self.initial_local_z is None:
            self.initial_local_z = local_position.z
        local_z = local_position.z - self.initial_local_z
        local_yaw = yaw_from_quaternion(local_orientation)

        rotation_x, rotation_y, rotation_z, rotation_w = (
            quaternion_from_yaw(local_yaw)
        )
        self.latest_local_foot = (
            (local_position.x, local_position.y, local_z),
            (rotation_x, rotation_y, rotation_z, rotation_w))

        odom_transform = TransformStamped()
        odom_transform.header.stamp = message.header.stamp
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_footprint"
        odom_transform.transform.translation.x = local_position.x
        odom_transform.transform.translation.y = local_position.y
        odom_transform.transform.translation.z = local_z
        odom_transform.transform.rotation.x = rotation_x
        odom_transform.transform.rotation.y = rotation_y
        odom_transform.transform.rotation.z = rotation_z
        odom_transform.transform.rotation.w = rotation_w
        self.tf_broadcaster.sendTransform(odom_transform)

        nav_odom = Odometry()
        nav_odom.header.stamp = message.header.stamp
        nav_odom.header.frame_id = "odom"
        nav_odom.child_frame_id = "base_footprint"
        nav_odom.pose.pose.position.x = local_position.x
        nav_odom.pose.pose.position.y = local_position.y
        nav_odom.pose.pose.position.z = local_z
        nav_odom.pose.pose.orientation.x = rotation_x
        nav_odom.pose.pose.orientation.y = rotation_y
        nav_odom.pose.pose.orientation.z = rotation_z
        nav_odom.pose.pose.orientation.w = rotation_w
        # gazebo_ros_p3d with frame_name=world publishes world-frame rates.
        # Odometry consumers expect twist in child_frame_id (base_footprint).
        nav_odom.twist = world_twist_to_body(message.twist, local_yaw)
        self.nav_odom_publisher.publish(nav_odom)
        return True

    def process_fastlio(self, message):
        if not self.world_history:
            return False
        try:
            stance = self.tf_buffer.lookup_transform(
                'base_footprint', 'base_link', rclpy.time.Time.from_msg(message.header.stamp)).transform
        except Exception:
            return False
        try:
            local = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time.from_msg(message.header.stamp)).transform
        except Exception:
            return False
        if self.map_anchor is None:
            ns = lambda m: m.header.stamp.sec*1000000000+m.header.stamp.nanosec
            aligned = interpolate_world_pose(self.world_history, ns(message))
            if aligned is None:
                return False
            world_to_map = compose_transform(self.resume['map_body'], inverse_transform(self.resume['world_body']))
            desired = compose_transform(world_to_map, aligned)
            candidate = compose_transform(desired, inverse_transform(pose_transform(message.pose.pose)))
            if not self.alignment_gate.add(ns(message)*1e-9, candidate):
                seconds = ns(message)*1e-9
                if seconds-self.last_alignment_log > 5.0:
                    self.get_logger().info('Resume initialization: age=%.1fs; waiting for stable alignment before map fusion' % (seconds-self.alignment_gate.first))
                    self.last_alignment_log = seconds
                return True  # Consume warmup scans without publishing a map transform.
            self.map_anchor = candidate
            self.publish_map_anchor()
            self.get_logger().info('Restored map alignment from checkpoint and current simulation spawn')
        foot_to_body = ((stance.translation.x, stance.translation.y, stance.translation.z),
                        (stance.rotation.x, stance.rotation.y, stance.rotation.z, stance.rotation.w))
        map_body = compose_transform(self.map_anchor, pose_transform(message.pose.pose))
        map_foot = compose_transform(map_body, inverse_transform(foot_to_body))
        matched_local_foot = ((local.translation.x, local.translation.y, local.translation.z),
                              (local.rotation.x, local.rotation.y, local.rotation.z, local.rotation.w))
        map_odom = compose_transform(map_foot, inverse_transform(matched_local_foot))

        local_orientation = self.latest_local_odom.pose.pose.orientation
        yaw_odom_base = yaw_from_quaternion(local_orientation)

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"

        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map_odom[0]
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = map_odom[1]

        self.tf_broadcaster.sendTransform(transform)

        nav_odometry = Odometry()
        nav_odometry.header.stamp = message.header.stamp
        nav_odometry.header.frame_id = "map"
        nav_odometry.child_frame_id = "base_footprint"
        nav_odometry.pose = copy.deepcopy(message.pose)
        nav_odometry.pose.pose.position.x, nav_odometry.pose.pose.position.y, nav_odometry.pose.pose.position.z = map_foot[0]
        nav_odometry.pose.pose.orientation.x, nav_odometry.pose.pose.orientation.y, nav_odometry.pose.pose.orientation.z, nav_odometry.pose.pose.orientation.w = map_foot[1]
        nav_odometry.twist = world_twist_to_body(
            self.latest_local_odom.twist, yaw_odom_base
        )

        self.fastlio_odom_publisher.publish(nav_odometry)
        return True


def main():
    rclpy.init()
    node = FastLioTfBridge()

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
