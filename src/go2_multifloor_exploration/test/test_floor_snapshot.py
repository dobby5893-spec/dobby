"""Exercise the real FloorSliceNode transport, isolated from simulation."""
import array
import os
import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from octomap_msgs.msg import Octomap
from std_msgs.msg import Float64
from frontier_exploration_ros2.msg import FloorMapSnapshot


def test_floor_slice_transports_exact_source_and_independent_generations(tmp_path):
    assert os.environ['ROS_DOMAIN_ID'] == '188'
    fixture = tmp_path / 'stairs.ot'
    subprocess.run([os.environ['STAIR_TEST_FIXTURE'], str(fixture)], check=True)
    data = array.array('b', fixture.read_bytes().split(b'\ndata\n', 1)[1])
    rclpy.init()
    node = Node('floor_snapshot_fixture')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    source = node.create_publisher(Octomap, '/snapshot_test/octomap', qos)
    height = node.create_publisher(Float64, '/snapshot_test/height', qos)
    received = []
    node.create_subscription(FloorMapSnapshot, '/snapshot_test/floor/snapshot', received.append, qos)
    log = (tmp_path / 'slice.log').open('w')
    process = subprocess.Popen([os.environ['FLOOR_SLICE_NODE'], '--ros-args',
        '-p', 'octomap_topic:=/snapshot_test/octomap', '-p', 'output_topic:=/snapshot_test/floor',
        '-p', 'floor_height_topic:=/snapshot_test/height', '-p', 'publish_period_s:=0.1',
        '-p', 'min_x:=-1.0', '-p', 'max_x:=1.0', '-p', 'min_y:=-1.0', '-p', 'max_y:=1.0'],
        stdout=log, stderr=subprocess.STDOUT)

    def wait(condition):
        deadline = time.monotonic() + 8
        while not condition() and time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'slice.log').read_text()
            rclpy.spin_once(node, timeout_sec=.05)
        assert condition(), (tmp_path / 'slice.log').read_text()

    try:
        wait(lambda: source.get_subscription_count() > 0 and height.get_subscription_count() > 0)
        first = Octomap()
        first.header.frame_id = 'map'
        first.header.stamp.sec = 10
        first.id, first.binary, first.resolution, first.data = 'OcTree', False, .05, data
        source.publish(first)
        wait(lambda: len(received) >= 1)
        a = received[-1]
        assert a.octomap == first
        assert a.source_octomap_generation == a.octomap_generation > 0
        height.publish(Float64(data=.3))
        wait(lambda: received[-1].floor_map.info.origin.position.z == .3)
        b = received[-1]
        assert b.octomap == first
        assert b.octomap_generation == a.octomap_generation
        assert b.floor_map_generation > a.floor_map_generation
        second = Octomap()
        second.header.frame_id = 'map'
        second.header.stamp.sec = 20
        second.id, second.binary, second.resolution, second.data = 'OcTree', False, .05, data
        source.publish(second)
        wait(lambda: received[-1].octomap_generation > b.octomap_generation)
        c = received[-1]
        assert c.octomap == second
        assert c.source_octomap_generation == c.octomap_generation
        assert c.floor_map_generation > b.floor_map_generation
    finally:
        process.terminate()
        process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
