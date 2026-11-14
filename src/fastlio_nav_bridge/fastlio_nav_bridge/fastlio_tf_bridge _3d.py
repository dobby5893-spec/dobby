import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def normalized_quaternion(orientation):
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("收到长度为零的姿态四元数")

    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b

    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quaternion_inverse(q):
    x, y, z, w = q
    return (-x, -y, -z, w)  # q 已归一化


def rotate_vector(q, vector):
    x, y, z, w = q
    vx, vy, vz = vector

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def set_rotation(target, q):
    target.x, target.y, target.z, target.w = q


class FastLioTfBridge(Node):
    def __init__(self):
        super().__init__("fastlio_tf_bridge")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_local_odom = None
        self.initial_local_z = None

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
            Odometry, "/odom", 10
        )
        self.fastlio_odom_publisher = self.create_publisher(
            Odometry, "/fastlio/odom", 10
        )

    def local_odom_callback(self, message):
        local_position = message.pose.pose.position
        local_orientation = message.pose.pose.orientation

        # 以启动时机器狗的高度作为零点，使楼层高度体现为 z 的变化。
        if self.initial_local_z is None:
            self.initial_local_z = local_position.z

        local_z = local_position.z - self.initial_local_z
        local_q = normalized_quaternion(local_orientation)
        self.latest_local_odom = message

        odom_transform = TransformStamped()
        odom_transform.header.stamp = message.header.stamp
        odom_transform.header.frame_id = "odom"
        odom_transform.child_frame_id = "base_footprint"
        odom_transform.transform.translation.x = local_position.x
        odom_transform.transform.translation.y = local_position.y
        odom_transform.transform.translation.z = local_z
        set_rotation(odom_transform.transform.rotation, local_q)
        self.tf_broadcaster.sendTransform(odom_transform)

        nav_odom = Odometry()
        nav_odom.header.stamp = message.header.stamp
        nav_odom.header.frame_id = "odom"
        nav_odom.child_frame_id = "base_footprint"
        nav_odom.pose = message.pose
        nav_odom.pose.pose.position.z = local_z
        nav_odom.twist = message.twist
        self.nav_odom_publisher.publish(nav_odom)

    def fastlio_odom_callback(self, message):
        if self.latest_local_odom is None:
            return

        fastlio_position = message.pose.pose.position
        fastlio_q = normalized_quaternion(
            message.pose.pose.orientation
        )

        local_pose = self.latest_local_odom.pose.pose
        local_position = local_pose.position
        local_q = normalized_quaternion(local_pose.orientation)

        # T_map_odom = T_map_base × inverse(T_odom_base)
        map_to_odom_q = quaternion_multiply(
            fastlio_q, quaternion_inverse(local_q)
        )

        odom_to_base_position = (
            local_position.x,
            local_position.y,
            local_position.z - self.initial_local_z,
        )
        rotated_position = rotate_vector(
            map_to_odom_q, odom_to_base_position
        )

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = (
            fastlio_position.x - rotated_position[0]
        )
        transform.transform.translation.y = (
            fastlio_position.y - rotated_position[1]
        )
        transform.transform.translation.z = (
            fastlio_position.z - rotated_position[2]
        )
        set_rotation(transform.transform.rotation, map_to_odom_q)
        self.tf_broadcaster.sendTransform(transform)

        nav_odometry = Odometry()
        nav_odometry.header.stamp = message.header.stamp
        nav_odometry.header.frame_id = "map"
        nav_odometry.child_frame_id = "base_footprint"
        nav_odometry.pose = message.pose
        nav_odometry.twist = self.latest_local_odom.twist
        self.fastlio_odom_publisher.publish(nav_odometry)


def main():
    rclpy.init()
    node = FastLioTfBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
