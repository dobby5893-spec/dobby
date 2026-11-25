"""Save stationary simulation progress and gate cold-start continuation."""
from collections import deque
import fcntl
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from rclpy.time import Time
from nav_msgs.msg import Odometry
from octomap_msgs.msg import Octomap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from frontier_exploration_ros2.msg import FloorSession
from frontier_exploration_ros2.srv import ControlExploration

from .checkpoint_store import save_checkpoint, load_checkpoint, full_octomap_file, world_fingerprint
from .fastlio_tf_bridge import pose_transform, compose_transform, inverse_transform, rotate_vector


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def transform_tuple(transform):
    p, q = transform.translation, transform.rotation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class ExplorationCheckpoint(Node):
    def __init__(self):
        super().__init__('exploration_checkpoint')
        self.root = Path(self.declare_parameter('checkpoint_dir', '/home/dobby/scan/checkpoints/current').value)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = (self.root / '.writer.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.world = self.declare_parameter('world', '/home/dobby/scan/worlds/research_two_floor.world').value
        self.fingerprint = world_fingerprint(self.world)
        self.resume_path = self.declare_parameter('resume_state', '').value
        self.managed = self.declare_parameter('managed_floor_sessions', False).value
        self.session = None
        self.session_seen = 0.
        self.start_enabled = self.declare_parameter('start_exploration', True).value
        self.interval = self.declare_parameter('save_interval_s', 30.0).value
        self.map = None
        self.cloud = None
        self.previous_cloud = None
        self.odom = deque(maxlen=3000)
        self.floor = None
        self.last_save = 0.0
        self.last_log = 0.0
        self.started = not self.start_enabled
        self.pending = None
        self.recovery_ready_since = None
        self.resume = json.loads(Path(self.resume_path).read_text()) if self.resume_path else None
        self.resume_validated = self.resume is None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Octomap, '/octomap_full', self.on_map, durable)
        self.create_subscription(Odometry, '/odom/ground_truth', self.odom.append, qos_profile_sensor_data)
        self.create_subscription(String, '/multifloor/checkpoint_state', self.on_floor, durable)
        self.create_subscription(PointCloud2, '/Laser_map', self.on_cloud, durable)
        self.history_publisher = self.create_publisher(PointCloud2, '/checkpoint/previous_cloud_map', durable)
        self.status = self.create_publisher(String, '/exploration_checkpoint/status', durable)
        self.create_service(Trigger, '/exploration_checkpoint/save', self.manual_save)
        if self.managed:
            self.create_subscription(FloorSession, '/multifloor/floor_session', self.on_session, 5)
            self.session_ready = self.create_publisher(
                FloorSession, '/multifloor/startup_ready', 5)
        else:
            self.control = self.create_client(ControlExploration, '/control_exploration')
        self.timer = self.create_timer(1.0, self.tick, clock=rclpy.clock.Clock())
        if self.resume_path:
            cloud_path = Path(self.resume_path).parent / 'cloud_map.cdr'
            if cloud_path.exists():
                previous = deserialize_message(cloud_path.read_bytes(), PointCloud2)
                previous.header.stamp.sec = 0
                previous.header.stamp.nanosec = 0
                self.previous_cloud = previous
                self.history_publisher.publish(previous)
        self.get_logger().info('Checkpoint writer ready: ' + str(self.root))

    def on_session(self, message):
        """Update the floor context for the startup readiness check."""
        if (self.session is None or
                message.task_id != self.session.task_id or
                message.floor_session_id != self.session.floor_session_id):
            self.recovery_ready_since = None
        self.session = message
        self.session_seen = time.monotonic()

    def on_map(self, message):
        if not message.binary and message.id == 'OcTree' and message.header.frame_id == 'map':
            self.map = message

    def on_cloud(self, message):
        self.cloud = message

    def on_floor(self, message):
        self.floor = json.loads(message.data)

    def state_message(self, message):
        self.status.publish(String(data=message))

    def aligned_world_pose(self, stamp):
        target = stamp_ns(stamp)
        before = next((m for m in reversed(self.odom) if stamp_ns(m.header.stamp) <= target), None)
        after = next((m for m in self.odom if stamp_ns(m.header.stamp) >= target), None)
        if before is None or after is None:
            raise ValueError('Waiting for world odometry bracketing map timestamp')
        t0, t1 = stamp_ns(before.header.stamp), stamp_ns(after.header.stamp)
        if max(target-t0, t1-target) > 150000000:
            raise ValueError('World odometry is not synchronized with map')
        a, b = pose_transform(before.pose.pose), pose_transform(after.pose.pose)
        fraction = (target-t0)/(t1-t0) if t1 != t0 else 0.0
        sign = 1 if sum(x*y for x,y in zip(a[1],b[1])) >= 0 else -1
        q = [(1-fraction)*x+fraction*sign*y for x,y in zip(a[1],b[1])]
        norm = math.sqrt(sum(x*x for x in q))
        return (tuple((1-fraction)*x+fraction*y for x,y in zip(a[0],b[0])), tuple(x/norm for x in q))

    def snapshot(self):
        if self.resume and not self.resume_validated:
            raise ValueError('Checkpoint deferred until resume alignment is verified')
        if not self.map or not self.odom or not self.floor:
            raise ValueError('Waiting for full map, floor state and world odometry')
        if self.floor['phase'] not in (1, 3, 8):
            raise ValueError('Checkpoint deferred during floor transition')
        age = (self.get_clock().now().nanoseconds-stamp_ns(self.map.header.stamp))*1e-9
        if not -0.1 <= age <= 2.0:
            raise ValueError('Map is not fresh')
        world = self.aligned_world_pose(self.map.header.stamp)
        old = next((m for m in reversed(self.odom)
                    if stamp_ns(m.header.stamp) <= stamp_ns(self.map.header.stamp)-500000000), None)
        if old is None:
            raise ValueError('Waiting for stationary history')
        previous = pose_transform(old.pose.pose)
        if math.dist(world[0], previous[0]) > 0.025 or abs(sum(x*y for x,y in zip(world[1],previous[1]))) < 0.999:
            raise ValueError('Checkpoint deferred until robot stands still')
        # A tilted robot on stairs is not a safe Gazebo spawn checkpoint.
        if abs(rotate_vector(world[1], (0, 0, 1))[2]) < math.cos(0.12):
            raise ValueError('Checkpoint deferred on inclined terrain')
        stamp = Time.from_msg(self.map.header.stamp)
        body = transform_tuple(self.tf.lookup_transform('map', 'base_link', stamp).transform)
        feet = transform_tuple(self.tf.lookup_transform('map', 'base_footprint', stamp).transform)
        state = dict(world=self.world, world_sha256=self.fingerprint, map_body=body,
                     world_body=world, map_feet=feet, floor=self.floor,
                     resolution=self.map.resolution, node_count=len(self.map.data)//5, saved_unix=time.time(),
                     original_sim_stamp=[self.map.header.stamp.sec, self.map.header.stamp.nanosec])
        raw = np.asarray(self.map.data, dtype=np.int8).tobytes()
        files = {'map.ot': full_octomap_file(raw, self.map.resolution)}
        if self.cloud is not None:
            import copy
            cloud = copy.deepcopy(self.cloud)
            anchor = transform_tuple(self.tf.lookup_transform('map', cloud.header.frame_id, Time()).transform)
            # PCL cloud fields can include padding; preserve layout and other fields.
            fields = {f.name:f.offset for f in cloud.fields}
            data = bytearray(cloud.data)
            xyz = [np.ndarray((cloud.height, cloud.width), dtype='<f4', buffer=data,
                              offset=fields[k], strides=(cloud.row_step, cloud.point_step)) for k in ('x','y','z')]
            q = anchor[1]
            rotation = np.array([rotate_vector(q, axis) for axis in ((1,0,0),(0,1,0),(0,0,1))]).T
            points = np.stack(xyz,axis=-1) @ rotation.T + np.array(anchor[0])
            for index in range(3):
                xyz[index][:] = points[...,index]
            cloud.data = bytes(data)
            cloud.header.frame_id = 'map'
            if self.previous_cloud is not None and self.previous_cloud.point_step == cloud.point_step:
                previous = self.previous_cloud
                # Display history is bounded independently of the exact navigation map.
                def rows(message):
                    return b''.join(bytes(message.data)[row*message.row_step:row*message.row_step+message.width*message.point_step]
                                    for row in range(message.height))
                combined = rows(previous) + rows(cloud)
                points = np.frombuffer(combined, dtype=np.dtype('V%d' % cloud.point_step))
                if len(points) > 500000:
                    points = points[np.linspace(0,len(points)-1,500000,dtype=np.int64)]
                cloud.height, cloud.width = 1, len(points)
                cloud.row_step = cloud.width*cloud.point_step
                cloud.data = points.tobytes()
            files['cloud_map.cdr'] = serialize_message(cloud)
        return save_checkpoint(self.root, state, files)

    def manual_save(self, request, response):
        try:
            path = self.snapshot()
            self.last_save = time.monotonic()
            response.success, response.message = True, str(path)
            self.state_message('saved ' + str(path))
        except Exception as error:
            response.success, response.message = False, str(error)
        return response

    def tick(self):
        now = time.monotonic()
        if self.managed:
            ready = (
                self.start_enabled and self.session is not None and
                self.session.phase == FloorSession.PREPARING and now - self.session_seen < 3.0)
        else:
            awaiting_validation = not self.started or not self.resume_validated
            ready = awaiting_validation and self.control.service_is_ready()
        if ready and self.map and self.floor:
            try:
                if self.managed:
                    floor_context = self.floor.get('task_id'), self.floor.get('floor_session_id')
                    if floor_context != (self.session.task_id, self.session.floor_session_id):
                        raise ValueError('Waiting for checkpoint state matching the floor session')
                if self.floor['phase'] not in ((7,) if self.managed else (1, 3, 8)):
                    raise ValueError('Waiting for floor initialization')
                checking_resume = self.resume is not None and not self.resume_validated
                if checking_resume:
                    minimum_nodes = self.resume.get('node_count', 0) * 0.9
                    if len(self.map.data) // 5 < minimum_nodes:
                        raise ValueError('Restored map is incomplete; exploration remains stopped')
                body = self.tf.lookup_transform('map', 'base_link', Time())
                age = (self.get_clock().now().nanoseconds-stamp_ns(body.header.stamp))*1e-9
                if not -0.1 <= age <= 0.5:
                    raise ValueError('Waiting for fresh localization')
                if checking_resume:
                    if not self.odom:
                        raise ValueError("Waiting for independent world odometry")
                    saved = self.resume['map_body']
                    current = transform_tuple(body.transform)
                    # Verify current mapping frame against the independent Gazebo spawn.
                    actual_world = pose_transform(self.odom[-1].pose.pose)
                    if math.dist(actual_world[0], self.resume['world_body'][0]) > 0.2:
                        raise ValueError('Simulation did not spawn at saved position; exploration remains stopped')
                    world_to_map = compose_transform(saved, inverse_transform(self.resume['world_body']))
                    expected = compose_transform(world_to_map, pose_transform(self.odom[-1].pose.pose))
                    angle = 2*math.acos(min(1., abs(sum(a*b for a,b in zip(current[1],expected[1])))))
                    if math.dist(current[0], expected[0]) > 0.12 or angle > 0.12:
                        raise ValueError('Resume alignment error exceeds 12 cm; exploration remains stopped')
                if self.recovery_ready_since is None:
                    self.recovery_ready_since = now
                if now-self.recovery_ready_since >= 3.0 and self.pending is None:
                    self.resume_validated = True
                    if self.managed:
                        self.session_ready.publish(self.session)
                    elif self.floor['phase'] != 1 or not self.start_enabled:
                        self.started = True  # Floor manager owns transition/completion.
                    else:
                        request = ControlExploration.Request()
                        request.action = ControlExploration.Request.ACTION_START
                        self.pending = self.control.call_async(request)
                if self.pending and self.pending.done():
                    reply = self.pending.result()
                    self.started = bool(reply.accepted)
                    self.pending = None
                    self.state_message('exploration started' if self.started else 'start rejected: ' + reply.message)
            except Exception as error:
                self.recovery_ready_since = None
                if now-self.last_log > 15:
                    self.state_message('startup blocked: ' + str(error))
                    self.get_logger().warn(str(error))
                    self.last_log = now
        if now-self.last_save >= self.interval:
            try:
                path = self.snapshot()
                self.last_save = now
                self.state_message('saved ' + str(path))
                self.get_logger().info('Saved exploration checkpoint: ' + str(path))
            except Exception as error:
                if now-self.last_log > 30:
                    self.get_logger().info(str(error))
                    self.last_log = now


def main():
    rclpy.init()
    node = ExplorationCheckpoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.get_logger().info('Final checkpoint: ' + str(node.snapshot()))
        except Exception as error:
            node.get_logger().info('Keeping previous checkpoint: ' + str(error))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
