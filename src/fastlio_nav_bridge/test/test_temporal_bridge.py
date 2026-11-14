from collections import deque
from types import SimpleNamespace

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from fastlio_nav_bridge.fastlio_tf_bridge import FastLioTfBridge


def test_delayed_lidar_uses_matching_odom_instead_of_latest_pose():
    node = object.__new__(FastLioTfBridge)
    message = Odometry()
    message.header.stamp.sec = 10
    message.pose.pose.position.x = 10.2
    message.pose.pose.position.z = .2
    message.pose.pose.orientation.w = 1.
    node.world_history = deque([message])
    node.map_anchor = ((0.,0.,0.),(0.,0.,0.,1.))
    node.latest_local_odom = message
    node.latest_local_foot = ((99.,0.,0.),(0.,0.,0.,1.))
    transforms = []
    def lookup(parent, child, stamp):
        assert stamp.nanoseconds == 10000000000
        result = TransformStamped()
        result.transform.rotation.w = 1.
        if parent == 'odom':
            result.transform.translation.x = 4.
        else:
            result.transform.translation.z = .2
        return result
    node.tf_buffer = SimpleNamespace(lookup_transform=lookup)
    node.tf_broadcaster = SimpleNamespace(sendTransform=transforms.append)
    node.fastlio_odom_publisher = SimpleNamespace(publish=lambda msg: None)
    assert node.process_fastlio(message)
    assert abs(transforms[0].transform.translation.x - 6.2) < 1e-10
    assert transforms[0].header.stamp.sec == 10


def test_resume_interpolates_ten_hz_odometry_with_a_32_ms_scan_offset():
    from fastlio_nav_bridge.fastlio_tf_bridge import interpolate_world_pose
    messages=[]
    for nanosecond, x in ((0, 1.0),(100000000, 2.0)):
        message=Odometry();message.header.stamp.sec=10;message.header.stamp.nanosec=nanosecond
        message.pose.pose.position.x=x;message.pose.pose.orientation.w=1.
        messages.append(message)
    pose=interpolate_world_pose(messages,10032000000)
    assert abs(pose[0][0]-1.32)<1e-10
    assert interpolate_world_pose(messages,10101000000) is None
