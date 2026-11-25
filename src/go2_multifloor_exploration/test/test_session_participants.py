"""Exercise the real managed Explorer and FloorSlice over ROS services and topics."""
import array
import copy
import os
import subprocess
import time
import uuid

import rclpy
from builtin_interfaces.msg import Time
from frontier_exploration_ros2.msg import FloorMapSnapshot, FloorParticipant, FloorSession
from frontier_exploration_ros2.srv import ControlExploration
from octomap_msgs.msg import Octomap
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Header


def test_managed_nodes_pin_context_and_gate_start_stop(tmp_path):
    fixture = tmp_path / 'source.ot'
    subprocess.run([os.environ['STAIR_TEST_FIXTURE'], str(fixture)], check=True)
    data = array.array('b', fixture.read_bytes().split(b'\ndata\n', 1)[1])
    rclpy.init()
    node = rclpy.create_node('floor_session_fixture')
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    snapshots, acks = [], {}
    node.create_subscription(FloorMapSnapshot, '/map_floor/snapshot', snapshots.append, durable)
    node.create_subscription(FloorParticipant, '/multifloor/session_participant',
                             lambda m: acks.__setitem__(m.role, m), 5)
    owner = node.create_publisher(FloorSession, '/multifloor/floor_session', 5)
    source = node.create_publisher(Octomap, '/octomap_full', durable)
    client = node.create_client(ControlExploration, '/control_exploration')
    args = ['--ros-args', '-p', 'managed_floor_sessions:=true', '-p', 'use_sim_time:=false']
    pulse = FloorSession()
    processes = []
    logs = []

    def run(executable, *parameters):
        output = (tmp_path / f'node-{len(processes)}.log').open('w+')
        logs.append(output)
        process = subprocess.Popen([executable, *args, *parameters], stdout=output, stderr=subprocess.STDOUT)
        processes.append(process)
        return process

    def pump(condition, timeout=8.):
        deadline = time.monotonic() + timeout
        while not condition() and time.monotonic() < deadline:
            stamp = node.get_clock().now().to_msg()
            if pulse.generation:
                pulse.heartbeat = stamp
                owner.publish(pulse)
            source.publish(Octomap(header=Header(stamp=stamp, frame_id='map'), id='OcTree',
                                   binary=False, resolution=.05, data=data))
            rclpy.spin_once(node, timeout_sec=.06)
            time.sleep(.03)
        diagnostics = []
        for logfile in logs:
            logfile.seek(0)
            diagnostics.append(logfile.read()[-6000:])
        assert condition(), diagnostics

    def command(action, context=None, sequence=1):
        request = ControlExploration.Request(action=action)
        if context is not None:
            request.task_id = context.task_id
            request.floor_session_id = context.floor_session_id
            request.session_generation = context.generation
            request.explorer_instance = context.explorer_instance
            request.command_sequence = sequence
        future = client.call_async(request)
        pump(future.done)
        return future.result()

    def matching(role, phase):
        p = acks.get(role)
        return p is not None and (p.task_id, p.floor_session_id, p.generation, p.phase) == (
            pulse.task_id, pulse.floor_session_id, pulse.generation, phase)

    try:
        run(os.environ['FLOOR_SLICE_NODE'], '-p', 'publish_period_s:=0.2')
        explorer = run(os.environ['FRONTIER_EXPLORER'], '-p', 'autostart:=true',
                       '-p', 'map_topic:=/map_floor', '-p', 'geometry_scoring_enabled:=false')
        pump(lambda: len(acks) == 2 and client.service_is_ready())
        assert not command(ControlExploration.Request.ACTION_START).accepted
        assert not snapshots
        pulse = FloorSession(
            task_id=str(uuid.uuid4()), floor_session_id=str(uuid.uuid4()), generation=1,
            started_at=node.get_clock().now().to_msg(), heartbeat=Time(),
            phase=FloorSession.PREPARING,
            explorer_instance=acks[FloorParticipant.EXPLORER].instance_id,
            map_producer=acks[FloorParticipant.SLICE].instance_id)
        pump(lambda: matching(FloorParticipant.EXPLORER, FloorSession.PREPARING) and
             matching(FloorParticipant.SLICE, FloorSession.PREPARING) and bool(snapshots))
        first = copy.deepcopy(pulse)
        assert snapshots[-1].map_producer == first.map_producer
        assert not command(ControlExploration.Request.ACTION_START, first).accepted

        pulse.phase = FloorSession.ACTIVE
        pump(lambda: matching(FloorParticipant.EXPLORER, FloorSession.ACTIVE))
        started = command(ControlExploration.Request.ACTION_START, first)
        assert started.accepted and started.state == ControlExploration.Request.STATE_RUNNING
        assert started.floor_session_id == first.floor_session_id
        assert not command(ControlExploration.Request.ACTION_STOP, first, 2).accepted

        pulse.phase = FloorSession.CLOSING
        pump(lambda: matching(FloorParticipant.EXPLORER, FloorSession.CLOSING))
        stopped = command(ControlExploration.Request.ACTION_STOP, first, 3)
        assert stopped.accepted and stopped.state == ControlExploration.Request.STATE_IDLE
        pulse.phase = FloorSession.CLOSED
        pump(lambda: matching(FloorParticipant.EXPLORER, FloorSession.CLOSED) and
             matching(FloorParticipant.SLICE, FloorSession.CLOSED))

        pulse.floor_session_id = str(uuid.uuid4())
        pulse.generation += 1
        pulse.started_at = node.get_clock().now().to_msg()
        pulse.phase = FloorSession.PREPARING
        pump(lambda: bool(snapshots) and snapshots[-1].floor_session_id == pulse.floor_session_id and
             matching(FloorParticipant.EXPLORER, FloorSession.PREPARING))
        assert snapshots[-1].task_id == first.task_id
        assert snapshots[-1].octomap.data == data
        assert not command(ControlExploration.Request.ACTION_START, first, 99).accepted

        explorer.terminate()
        explorer.wait(timeout=5)
        run(os.environ['FRONTIER_EXPLORER'], '-p', 'map_topic:=/map_floor')
        pump(lambda: acks[FloorParticipant.EXPLORER].instance_id != first.explorer_instance and
             client.service_is_ready())
        assert acks[FloorParticipant.EXPLORER].generation == 0
        assert not command(ControlExploration.Request.ACTION_START, pulse, 100).accepted
    finally:
        for process in processes:
            if process.poll() is None:
                process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        for logfile in logs:
            logfile.close()
        node.destroy_node()
        rclpy.shutdown()
