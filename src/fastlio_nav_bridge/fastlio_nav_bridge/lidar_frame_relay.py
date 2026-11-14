"""Relay raw LiDAR scans through a TF branch rooted in FAST-LIO's map."""

import math
import numpy as np

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import StaticTransformBroadcaster


def quaternion_from_matrix(values):
    """Convert a row-major 3x3 rotation matrix to an xyzw quaternion."""
    if len(values) != 9:
        raise ValueError('mapping.extrinsic_R must contain 9 values')

    m00, m01, m02, m10, m11, m12, m20, m21, m22 = map(float, values)
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            scale / 4.0,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            scale / 4.0,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            scale / 4.0,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            scale / 4.0,
            (m10 - m01) / scale,
        )

    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-12:
        raise ValueError('mapping.extrinsic_R is not a valid rotation')
    return tuple(value / norm for value in quaternion)


class LidarFrameRelay(Node):
    """Keep points sensor-relative and attach them to FAST-LIO's moving body."""

    def __init__(self):
        super().__init__('fastlio_lidar_frame_relay')

        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/fastlio/lidar_octomap')
        self.declare_parameter('input_frame', 'livox_frame')
        self.declare_parameter('output_frame', 'fastlio_livox')
        self.declare_parameter('body_frame', 'body')
        self.declare_parameter('publish_every_n', 5)
        self.declare_parameter('wait_for_alignment', False)
        self.declare_parameter('minimum_valid_range', 0.01)
        self.declare_parameter('mapping.extrinsic_T', [0.0, 0.0, 0.105])
        self.declare_parameter('mapping.extrinsic_R', [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ])

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.input_frame = self.get_parameter('input_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        body_frame = self.get_parameter('body_frame').value
        self.publish_every_n = max(
            1, int(self.get_parameter('publish_every_n').value))
        self.minimum_valid_range = float(self.get_parameter('minimum_valid_range').value)
        offset = self.get_parameter('mapping.extrinsic_T').value
        rotation = self.get_parameter('mapping.extrinsic_R').value

        if len(offset) != 3:
            raise ValueError('mapping.extrinsic_T must contain 3 values')
        qx, qy, qz, qw = quaternion_from_matrix(rotation)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = body_frame
        transform.child_frame_id = self.output_frame
        transform.transform.translation.x = float(offset[0])
        transform.transform.translation.y = float(offset[1])
        transform.transform.translation.z = float(offset[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcaster.sendTransform(transform)

        self.publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, input_topic, self.relay, qos_profile_sensor_data)
        self.warned_frame_mismatch = False
        self.scan_count = 0
        self.alignment_stamp = None if self.get_parameter('wait_for_alignment').value else 0
        self.create_subscription(Header, '/fastlio/map_alignment_ready', self.on_alignment,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_alignment(self, message):
        self.alignment_stamp = message.stamp.sec*1000000000 + message.stamp.nanosec

    def relay(self, cloud):
        stamp = cloud.header.stamp.sec*1000000000 + cloud.header.stamp.nanosec
        if self.alignment_stamp is None or stamp < self.alignment_stamp:
            return
        if cloud.header.frame_id != self.input_frame:
            if not self.warned_frame_mismatch:
                self.get_logger().error(
                    'Expected input frame %s, received %s; dropping cloud'
                    % (self.input_frame, cloud.header.frame_id))
                self.warned_frame_mismatch = True
            return

        self.scan_count += 1
        if self.scan_count % self.publish_every_n != 0:
            return

        # Gazebo emits zero vectors for rays without a valid hit. Treating those
        # as occupied creates a false obstacle at the moving sensor origin.
        points = point_cloud2.read_points(cloud, skip_nans=False)
        x, y, z = points['x'], points['y'], points['z']
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        valid &= x*x + y*y + z*z >= self.minimum_valid_range**2
        if not np.all(valid):
            cloud = point_cloud2.create_cloud(cloud.header, cloud.fields, points[valid])
        cloud.header.frame_id = self.output_frame
        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = LidarFrameRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
