"""Isolated ROS test of timestamp/stability gating without Gazebo or navigation."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from fastlio_nav_bridge.continuous_ground import read_alignment_region
from fastlio_nav_bridge.fastlio_tf_bridge import compose_transform


def test_resolver_waits_for_fresh_stable_measured_alignment(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '185')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    world = Path(__file__).resolve().parents[3]/'worlds/research_two_floor.world'
    output = tmp_path/'ground.json'
    params = tmp_path/'resolver.yaml'
    params.write_text(json.dumps({'/**': {'ros__parameters': {
        'use_sim_time': True, 'world': str(world), 'output': str(output), 'timeout_s': 20.}}}))
    rclpy.init()
    node = Node('ground_prior_protocol_fixture')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    odom_pub = node.create_publisher(Odometry, '/odom/ground_truth', 10)
    tf_pub = node.create_publisher(TFMessage, '/tf', 20)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    static_pub = node.create_publisher(TFMessage, '/tf_static', durable)
    foot = TransformStamped()
    foot.header.frame_id, foot.child_frame_id = 'base_link', 'base_footprint'
    foot.transform.translation.z = -.21
    foot.transform.rotation.w = 1.
    static_pub.publish(TFMessage(transforms=[foot]))
    log = (tmp_path/'resolver.log').open('w')
    process = subprocess.Popen([
        sys.executable, '-c',
        'from fastlio_nav_bridge.ground_prior_resolver import main; main()',
        '--ros-args', '--params-file', str(params)],
        stdout=log, stderr=subprocess.STDOUT)
    started = time.monotonic()
    transform = ((1., -2., .04), (0., 0., math.sin(.15), math.cos(.15)))
    body_world = ((0., 0., .22), (0., 0., math.sin(.1), math.cos(.1)))

    def stamp(value):
        return Time(sec=int(value), nanosec=int(value % 1 * 1e9))

    def pump(duration, stale=False, jitter=False):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline and process.poll() is None:
            elapsed = time.monotonic()-started+.1
            clock_pub.publish(Clock(clock=stamp(elapsed)))
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'world', 'base_link'
            odom.header.stamp = stamp(elapsed)
            p, q = body_world
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = p
            odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
            odom_pub.publish(odom)
            mapped = compose_transform(transform, body_world)
            message = TransformStamped()
            message.header.frame_id, message.child_frame_id = 'map', 'base_link'
            message.header.stamp = stamp(max(.001, elapsed-2.) if stale else elapsed)
            p, q = mapped
            message.transform.translation.x = p[0]+(.06 if jitter and int(elapsed*10)%2 else 0.)
            message.transform.translation.y, message.transform.translation.z = p[1:]
            message.transform.rotation.x, message.transform.rotation.y, message.transform.rotation.z, message.transform.rotation.w = q
            tf_pub.publish(TFMessage(transforms=[message]))
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.02)

    try:
        pump(2.5, stale=True)
        assert process.poll() is None and not output.exists(), 'Stale TF opened the navigation gate'
        pump(2., jitter=True)
        assert process.poll() is None and not output.exists(), 'Unstable alignment opened the gate'
        pump(10.)
        assert process.poll() == 0, (tmp_path/'resolver.log').read_text()
        region = read_alignment_region(output, world)
        assert all(abs(a-b)<1e-6 for a, b in zip(region[:7], (*transform[0], *transform[1])))
        assert region[7:] == [9.5, 6.5, .1]
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
