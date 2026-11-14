import math

from geometry_msgs.msg import TwistWithCovariance
from fastlio_nav_bridge.fastlio_tf_bridge import world_twist_to_body


def test_northward_motion_when_body_faces_north_is_forward():
    source = TwistWithCovariance()
    source.twist.linear.y = 0.2
    result = world_twist_to_body(source, math.pi / 2)
    assert abs(result.twist.linear.x - 0.2) < 1e-10
    assert abs(result.twist.linear.y) < 1e-10
    assert source.twist.linear.y == 0.2
    assert source.twist.linear.x == 0.0


def test_covariance_axes_rotate_with_velocity():
    source = TwistWithCovariance()
    source.covariance[0] = 1.0
    source.covariance[7] = 4.0
    source.twist.angular.z = 0.3
    result = world_twist_to_body(source, math.pi / 2)
    assert abs(result.covariance[0] - 4.0) < 1e-10
    assert abs(result.covariance[7] - 1.0) < 1e-10
    assert result.twist.angular.z == 0.3


def test_body_to_map_rotation_uses_map_heading_once():
    # World velocity (0,.2), world body yaw 90deg, map body yaw 120deg.
    # Correct map velocity is world velocity rotated by the frame offset 30deg.
    source = TwistWithCovariance()
    source.twist.linear.y = 0.2
    body = world_twist_to_body(source, math.pi / 2).twist.linear
    map_yaw = 2 * math.pi / 3
    map_x = math.cos(map_yaw) * body.x - math.sin(map_yaw) * body.y
    map_y = math.sin(map_yaw) * body.x + math.cos(map_yaw) * body.y
    assert abs(map_x + 0.1) < 1e-10
    assert abs(map_y - math.sqrt(3) * 0.1) < 1e-10
