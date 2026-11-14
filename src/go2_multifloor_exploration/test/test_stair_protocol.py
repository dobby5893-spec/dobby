"""Isolated fake-interface test; never contacts Gazebo or the user's ROS domain."""
import array
import copy
import json
import os
from pathlib import Path
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid
from octomap_msgs.msg import Octomap
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty, Float64, String, Bool
from tf2_msgs.msg import TFMessage
from frontier_exploration_ros2.srv import ControlExploration


@pytest.mark.parametrize("handoff_kind", ["complete", "residual", "bounded_residual", "region_audit", "region_audit_v3", "region_audit_v4"])
def test_stair_handoff_requires_idle_gait_ack_and_measured_height(tmp_path, handoff_kind):
    assert os.environ.get('ROS_DOMAIN_ID') == '179', 'This test requires its isolated ROS domain'
    fixture = tmp_path / 'stairs.ot'
    subprocess.run([os.environ['STAIR_TEST_FIXTURE'], str(fixture)], check=True)
    data = fixture.read_bytes().split(b'\ndata\n', 1)[1]
    rclpy.init()
    node = Node('stair_protocol_fixture')
    node.declare_parameter('gait.swing_height', .04)
    node.declare_parameter('gait.stance_duration', .25)
    state = dict(pose=(-.8, 0., 0.), idle=False, allow_gait=False, rejections=0,
                 first_ascent_failed=False, ascent_follows=0, compute=[], follows=[], floors=[], modes=[], phases=[], starts=0, slice_height=0., completed=0, deferred=0, stamp=0.)

    def parameters(values):
        stairs = any(p.name == 'gait.swing_height' and p.value > .05 for p in values)
        ok = not stairs or state['allow_gait']
        if not ok:
            state['rejections'] += 1
        return SetParametersResult(successful=ok, reason='' if ok else 'test rejection')

    node.add_on_set_parameters_callback(parameters)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    map_pub = node.create_publisher(Octomap, '/octomap_full', qos)
    tf_pub = node.create_publisher(TFMessage, '/tf', 20)
    slice_pub = node.create_publisher(OccupancyGrid, '/map_floor', qos)
    completion = node.create_publisher(Empty, '/exploration_complete', qos)
    verified = node.create_publisher(String, '/explore/floor_completion', qos)

    def formal(floor_z, residual=False):
        area = '.1' if residual else '0'
        return String(data=f"floor_completion;authority=joint_residual_audit;"
            f"status={'VERIFIED_WITH_RESIDUALS' if residual else 'VERIFIED_COMPLETE'};"
            f"floor_z={floor_z};stamp={state['stamp']};remaining={area};residual={area};transition=0;"
            f"region_count={1 if residual else 0};regions={'11:.1:occluded' if residual else ''};"
            "safe_start=1;complete_views=1;complete_routes=1;stable_samples=3;stable_seconds=10")

    exhausted = node.create_publisher(String, '/explore/floor_exhausted', qos)
    node.create_subscription(Empty, '/multifloor/exploration_complete',
                             lambda m: state.update(completed=state['completed']+1), qos)
    node.create_subscription(String, '/multifloor/exploration_deferred',
                             lambda m: state.update(deferred=state['deferred']+1), qos)
    node.create_subscription(Float64, '/multifloor/active_floor_height', lambda m: state['floors'].append(m.data), qos)
    node.create_subscription(Bool, '/multifloor/stair_mode', lambda m: state['modes'].append(m.data), qos)
    node.create_subscription(String, '/multifloor/checkpoint_state', lambda m: state['phases'].append(json.loads(m.data)), qos)

    def control(request, response):
        response.accepted = True
        if request.action == request.ACTION_STOP:
            response.state = request.STATE_IDLE if state['idle'] else request.STATE_STOPPING
        else:
            state['starts'] += 1
            response.state = request.STATE_RUNNING
        return response

    node.create_service(ControlExploration, '/control_exploration', control)
    group = ReentrantCallbackGroup()

    def compute(handle):
        request = handle.request
        state['compute'].append(request.planner_id)
        result = ComputePathToPose.Result()
        if request.planner_id == 'Stair3D' and not state['first_ascent_failed']:
            state['first_ascent_failed'] = True
            handle.abort()
            return result
        result.path.header = copy.deepcopy(request.goal.header)
        begin = state['pose']
        for i in range(9):
            p = PoseStamped()
            p.header = copy.deepcopy(request.goal.header)
            p.pose.orientation = copy.deepcopy(request.goal.pose.orientation)
            for index, axis in enumerate(('x', 'y', 'z')):
                target = getattr(request.goal.pose.position, axis)
                setattr(p.pose.position, axis, begin[index] + (target - begin[index]) * i / 8.)
            result.path.poses.append(p)
        handle.succeed()
        return result

    def follow(handle):
        target = handle.request.path.poses[-1].pose.position
        state['follows'].append((target.x, target.y, target.z))
        if state['compute'][-1] == 'Stair3D':
            state['ascent_follows'] += 1
        # Deliberately report success without climbing: the manager must reject it.
        state['pose'] = (target.x, target.y, .15 if state['compute'][-1] == 'Stair3D' else 0.)
        # Let the independently published measured TF arrive before the result.
        time.sleep(.15)
        handle.succeed()
        return FollowPath.Result()

    servers = [ActionServer(node, ComputePathToPose, '/compute_path_to_pose', compute, callback_group=group),
               ActionServer(node, FollowPath, '/follow_path', follow, callback_group=group)]
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    params = tmp_path / 'manager.yaml'
    params.write_text(json.dumps({'/**': {'ros__parameters': {
        'use_sim_time': True, 'timer_period_s': .10,
        'gait_controller_node': '/stair_protocol_fixture',
        'expected_floor_count': 2, 'floor_height_separation': .20,
        'visited_floor_tolerance': .10, 'floor_slice_settle_s': .30,
        'stair_scan_interval_s': .25, 'stair_search_budget_s': 1.5,
        'stair_observation_wait_s': .10, 'stair_observation_attempts': 0,
        'suppression_timeout_s': .40, 'transition_rpc_timeout_s': 2.,
    }}}))
    log = (tmp_path / 'manager.log').open('w')
    manager = subprocess.Popen([os.environ['STAIR_TEST_MANAGER'], '--ros-args', '--params-file', str(params)],
                               stdout=log, stderr=subprocess.STDOUT)
    start = time.monotonic()

    def pump_until(condition, timeout=15.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            assert manager.poll() is None, (tmp_path / 'manager.log').read_text()
            elapsed = time.monotonic() - start + .1
            state['stamp'] = elapsed
            stamp = Time(sec=int(elapsed), nanosec=int(elapsed % 1 * 1e9))
            clock_pub.publish(Clock(clock=stamp))
            transforms = []
            for child, offset in [('base_link', .21), ('base_footprint', 0.)]:
                t = TransformStamped()
                t.header.frame_id = 'map'
                t.header.stamp = stamp
                t.child_frame_id = child
                t.transform.translation.x, t.transform.translation.y = state['pose'][:2]
                t.transform.translation.z = state['pose'][2] + offset
                t.transform.rotation.w = 1.
                transforms.append(t)
            tf_pub.publish(TFMessage(transforms=transforms))
            octree = Octomap()
            octree.header.frame_id = 'map'
            octree.header.stamp = stamp
            octree.id, octree.binary, octree.resolution = 'OcTree', False, .05
            octree.data = array.array('b', data)
            map_pub.publish(octree)
            floor = OccupancyGrid()
            floor.header = octree.header
            floor.info.resolution = .1
            floor.info.width = floor.info.height = 1
            floor.info.origin.orientation.w = 1.
            floor.info.origin.position.z = state['slice_height']
            floor.data = [0]
            slice_pub.publish(floor)
            time.sleep(.05)
        assert condition(), (state, (tmp_path / 'manager.log').read_text()[-5000:])

    try:
        pump_until(lambda: any(x['phase'] == 1 for x in state['phases']))
        if handoff_kind != 'complete':
            # A retained report from another floor must never stop this floor.
            exhausted.publish(String(data=f"v1;floor_z=3;stamp={state['stamp']};"
                "reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4"))
            until = time.monotonic() + .4
            pump_until(lambda: time.monotonic() >= until)
            assert state['phases'][-1]['phase'] == 1
            if handoff_kind == 'region_audit_v4':
                exhausted.publish(String(data=f"v4;floor_z=0;stamp={state['stamp']};"
                    "reason=region_observability_audit;remaining_unknown_area=3;actionable_unknown_area=0;"
                    "excluded_unknown_area=1;deferred_unknown_area=2;transition_unknown_area=0;unresolved_unknown_area=0;"
                    "scope=per_region_current_floor;region_count=2;stable_samples=3;stable_seconds=10;"
                    "safe_start=1;complete_views=1;complete_routes=0;runtime_evidence=1;"
                    "runtime_min_positions=3;runtime_min_healthy_s=60;attempts=0;"
                    "regions=11:1:observed_no_gain,22:1:verification_deferred"))
            elif handoff_kind == 'region_audit_v3':
                exhausted.publish(String(data=f"v3;floor_z=0;stamp={state['stamp']};"
                    "reason=region_observability_audit;remaining_unknown_area=3;actionable_unknown_area=0;"
                    "excluded_unknown_area=1;deferred_unknown_area=1;transition_unknown_area=1;unresolved_unknown_area=0;"
                    "scope=per_region_current_floor;region_count=2;stable_samples=3;stable_seconds=10;"
                    "safe_start=1;complete_views=1;complete_routes=1;attempts=0;regions=11:1:occluded,22:1:transition"))
            elif handoff_kind == 'region_audit':
                exhausted.publish(String(data=f"v2;floor_z=0;stamp={state['stamp']};"
                    "reason=region_observability_audit;remaining_unknown_area=3;actionable_unknown_area=0;"
                    "excluded_unknown_area=1;deferred_unknown_area=2;unresolved_unknown_area=0;"
                    "scope=per_region_current_floor;region_count=2;stable_samples=3;stable_seconds=10;"
                    "safe_start=1;complete_views=1;attempts=0;regions=11:1:occluded,22:1:no_known_access"))
            elif handoff_kind == 'bounded_residual':
                exhausted.publish(String(data=f"v1;floor_z=0;stamp={state['stamp']};"
                    "reason=bounded_residuals_deferred;remaining_unknown_area=10.59;"
                    "actionable_unknown_area=9.38;excluded_unknown_area=1.21;deferred_unknown_area=9.38;"
                    "evidence_rounds=6;raw_empty_rounds=0;failed_groups=3;failed_options=3;"
                    "validation_scope=all_candidates;observation_positions=0;expanded_stage=3;healthy_no_gain_s=180;attempts=50"))
                # Global failed fans cannot authorize a floor transition.
                until = time.monotonic() + .8
                pump_until(lambda: time.monotonic() >= until)
                assert state['phases'][-1]['phase'] == 1
                assert not state['compute']
                exhausted.publish(String(data=f"v1;floor_z=0;stamp={state['stamp']};"
                    "reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4"))
            else:
                exhausted.publish(String(data=f"v1;floor_z=0;stamp={state['stamp']};"
                    "reason=closed_regions_excluded;remaining_unknown_area=.42;actionable_unknown_area=0;excluded_unknown_area=.42;attempts=4"))
        else:
            completion.publish(Empty())
        # Every legacy report and retained Empty must leave exploration active.
        until = time.monotonic() + .8
        pump_until(lambda: time.monotonic() >= until)
        assert state['phases'][-1]['phase'] == 1
        assert not state['compute'], 'Legacy completion authorized stair handoff'
        verified.publish(formal(0., handoff_kind != 'complete'))
        pump_until(lambda: any(x['phase'] == 2 for x in state['phases']))
        until = time.monotonic() + .8
        pump_until(lambda: time.monotonic() >= until)
        assert not state['compute'], 'Stair handoff started before frontier became idle'
        state['idle'] = True
        pump_until(lambda: state['rejections'] > 0)
        assert 'Stair3D' not in state['compute'], 'Climbed before gait update was accepted'
        state['allow_gait'] = True
        pump_until(lambda: state['first_ascent_failed'])
        pump_until(lambda: state['modes'] and state['modes'][-1] is False)
        assert all(len(x['visited_floors']) <= 1 for x in state['phases']), 'Failed first plan advanced the floor'
        pump_until(lambda: state['ascent_follows'] > 0)
        until = time.monotonic() + 1.0
        pump_until(lambda: time.monotonic() >= until)
        assert state['floors'] and all(abs(z) < .08 for z in state['floors'])
        assert all(len(x['visited_floors']) == 1 for x in state['phases'] if x['phase'] != 0)
        assert state['modes'][-1] is True, 'Failure on stairs restored flat gait prematurely'
        assert state['starts'] == 0, 'Restarted floor exploration before actual landing'
        # Now provide actual supported landing feedback; the old floor slice must still block restart.
        state['pose'] = (2.5, 0., .30)
        pump_until(lambda: any(z > .20 for z in state['floors']))
        until = time.monotonic() + .8
        pump_until(lambda: time.monotonic() >= until)
        assert state['starts'] == 0, 'Restarted before a matching floor slice arrived'
        state['slice_height'] = .30
        pump_until(lambda: state['starts'] == 1)
        assert state['modes'][-1] is False
        assert any(len(x['visited_floors']) == 2 for x in state['phases'])
        # Completion is explicit and only after the configured second floor is explored.
        pump_until(lambda: state['phases'][-1]['phase'] == 1)
        completion.publish(Empty())
        until = time.monotonic() + .5
        pump_until(lambda: time.monotonic() >= until)
        assert state['phases'][-1]['phase'] == 1, 'Empty bypassed second-floor audit'
        verified.publish(formal(.30))
        pump_until(lambda: state['phases'][-1]['phase'] == 8)
        if handoff_kind != 'complete':
            pump_until(lambda: state['deferred'] > 0)
            assert state['completed'] == 0, 'Residual unknowns were reported as full coverage'
        else:
            pump_until(lambda: state['completed'] > 0)
            assert state['deferred'] == 0
        (tmp_path / 'protocol_result.json').write_text(json.dumps(state, indent=2))
    finally:
        manager.terminate()
        try:
            manager.wait(timeout=6)
        except subprocess.TimeoutExpired:
            manager.kill()
            manager.wait(timeout=3)
        log.close()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()
