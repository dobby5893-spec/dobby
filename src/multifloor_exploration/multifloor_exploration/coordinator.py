"""Coordinate per-floor frontier exploration and closed-loop stair traversal."""

import math
from enum import Enum

import rclpy
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def euler_from_quaternion(quaternion):
    roll = math.atan2(
        2.0 * (quaternion.w * quaternion.x
               + quaternion.y * quaternion.z),
        1.0 - 2.0 * (quaternion.x * quaternion.x
                     + quaternion.y * quaternion.y),
    )
    pitch_term = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.asin(clamp(pitch_term, -1.0, 1.0))
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )
    return roll, pitch, yaw


def double_parameter(name, value):
    return ParameterMessage(
        name=name,
        value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE,
            double_value=float(value),
        ),
    )


class State(Enum):
    WAITING = 'waiting_for_stack'
    FLOOR_1 = 'exploring_floor_1'
    NAVIGATING_TO_STAIRS = 'navigating_to_stairs'
    PREPARING_STAIRS = 'preparing_stair_gait'
    CLIMBING = 'climbing_stairs'
    SWITCHING_FLOOR = 'switching_to_floor_2'
    FLOOR_2 = 'exploring_floor_2'
    COMPLETE = 'complete'
    FAILED = 'failed'


class MultifloorCoordinator(Node):
    def __init__(self):
        super().__init__('multifloor_coordinator')
        self.declare_parameter('startup_delay_s', 8.0)
        self.declare_parameter('stair_test_only', False)
        self.declare_parameter('stair_test_approach', [
            0.0, -3.0,
            4.20, -3.0,
            4.20, -1.40,
        ])
        self.declare_parameter('stair_entry', [4.20, -1.40, 0.0])
        self.declare_parameter('stair_waypoints', [
            8.95, -1.40,
            9.40, -1.40,
            9.40, 1.40,
            8.90, 1.40,
            4.55, 1.40,
        ])
        self.declare_parameter('approach_linear_speed', 0.08)
        self.declare_parameter('stair_linear_speed', 0.04)
        self.declare_parameter('stair_angular_speed', 0.15)
        self.declare_parameter('stair_waypoint_tolerance', 0.16)
        self.declare_parameter('stair_timeout_s', 900.0)
        self.declare_parameter('stair_required_height_gain', 1.80)
        self.declare_parameter('maximum_tilt', 0.75)
        self.declare_parameter('odometry_timeout_s', 2.0)
        self.declare_parameter('floor_1_min_z', 0.20)
        self.declare_parameter('floor_1_max_z', 1.60)
        self.declare_parameter('floor_2_min_z', 2.60)
        self.declare_parameter('floor_2_max_z', 4.00)
        self.declare_parameter('flat_swing_height', 0.04)
        self.declare_parameter('stair_swing_height', 0.10)
        self.declare_parameter('flat_stance_duration', 0.25)
        self.declare_parameter('stair_stance_duration', 0.36)

        self.state = State.WAITING
        self.pose = None
        self.map_received = False
        self.start_time = self.get_clock().now()
        self.stair_start_time = None
        self.stair_start_z = None
        self.last_pose_time = None
        self.waypoint_index = 0
        self.stair_segment_start = 0
        self.pending_request = False
        self.stair_test_only = bool(
            self.get_parameter('stair_test_only').value)

        waypoint_values = list(
            self.get_parameter('stair_waypoints').value)
        if len(waypoint_values) % 2:
            raise ValueError('stair_waypoints must contain x/y pairs')
        self.stair_waypoints = list(zip(
            waypoint_values[0::2], waypoint_values[1::2]))
        if self.stair_test_only:
            approach_values = list(
                self.get_parameter('stair_test_approach').value)
            if len(approach_values) % 2:
                raise ValueError('stair_test_approach must contain x/y pairs')
            approach = list(zip(
                approach_values[0::2], approach_values[1::2]))
            self.stair_segment_start = len(approach)
            self.stair_waypoints = approach + self.stair_waypoints

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Empty, '/exploration_complete', self.exploration_complete,
            transient_qos)
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, transient_qos)
        self.create_subscription(
            Odometry, '/Odometry', self.odometry_callback, 20)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(
            String, '/multifloor/status', transient_qos)
        self.frontier_client = self.create_client(
            ControlExploration, '/control_exploration')
        self.octomap_parameters = self.create_client(
            SetParameters, '/octomap_server_3d/set_parameters')
        self.gait_parameters = self.create_client(
            SetParameters,
            '/quadruped_controller_node/set_parameters')
        self.navigate_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')
        self.timer = self.create_timer(0.05, self.tick)
        self.publish_state('Waiting for mapping, Nav2, and frontier services')

    def publish_state(self, detail=''):
        message = self.state.value
        if detail:
            message += ': ' + detail
        self.status_publisher.publish(String(data=message))
        self.get_logger().info(message)

    def fail(self, reason):
        self.stop_robot()
        self.state = State.FAILED
        self.publish_state(reason)

    def map_callback(self, _message):
        self.map_received = True

    def odometry_callback(self, message):
        position = message.pose.pose.position
        roll, pitch, yaw = euler_from_quaternion(
            message.pose.pose.orientation)
        self.pose = (
            position.x,
            position.y,
            position.z,
            roll,
            pitch,
            yaw,
        )
        self.last_pose_time = self.get_clock().now()

    def exploration_complete(self, _message):
        # This legacy coordinator has no joint-audit consumer. Keep stair-test
        # mode usable, but fail closed for automatic floor handoff.
        self.get_logger().warning(
            'Legacy Empty completion ignored. Automatic multifloor completion '
            'requires go2_multifloor_exploration/full_multifloor_demo.launch.py.')

    def tick(self):
        if self.state == State.WAITING:
            delay = self.get_parameter('startup_delay_s').value
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if self.stair_test_only:
                ready = (
                    elapsed >= delay
                    and self.pose is not None
                    and self.gait_parameters.service_is_ready()
                )
            else:
                ready = (
                    elapsed >= delay
                    and self.pose is not None
                    and self.map_received
                    and self.frontier_client.service_is_ready()
                    and self.octomap_parameters.service_is_ready()
                    and self.gait_parameters.service_is_ready()
                    and self.navigate_client.server_is_ready()
                )
            if ready and not self.pending_request:
                self.pending_request = True
                if self.stair_test_only:
                    self.set_gait(True, self.begin_climb)
                else:
                    self.set_floor_projection(1, self.start_floor_1)
        elif self.state == State.CLIMBING:
            self.drive_stairs()

    def call_frontier(self, action, callback=None):
        request = ControlExploration.Request()
        request.action = action
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        future = self.frontier_client.call_async(request)

        def finished(result_future):
            try:
                response = result_future.result()
                if not response.accepted:
                    self.fail('Frontier control rejected: ' + response.message)
                    return
                if callback:
                    callback()
            except Exception as error:
                self.fail('Frontier control failed: %s' % error)

        future.add_done_callback(finished)

    def set_floor_projection(self, floor, callback):
        self.pending_request = True
        request = SetParameters.Request()
        if floor == 1:
            minimum = self.get_parameter('floor_1_min_z').value
            maximum = self.get_parameter('floor_1_max_z').value
        else:
            minimum = self.get_parameter('floor_2_min_z').value
            maximum = self.get_parameter('floor_2_max_z').value
        request.parameters = [
            double_parameter('occupancy_min_z', minimum),
            double_parameter('occupancy_max_z', maximum),
        ]
        future = self.octomap_parameters.call_async(request)

        def finished(result_future):
            self.pending_request = False
            try:
                response = result_future.result()
                if not all(result.successful for result in response.results):
                    reasons = ', '.join(
                        result.reason for result in response.results
                        if not result.successful)
                    self.fail('OctoMap floor switch failed: ' + reasons)
                    return
                callback()
            except Exception as error:
                self.fail('OctoMap parameter request failed: %s' % error)

        future.add_done_callback(finished)

    def set_gait(self, stair_mode, callback):
        request = SetParameters.Request()
        if stair_mode:
            swing = self.get_parameter('stair_swing_height').value
            stance = self.get_parameter('stair_stance_duration').value
        else:
            swing = self.get_parameter('flat_swing_height').value
            stance = self.get_parameter('flat_stance_duration').value
        request.parameters = [
            double_parameter('gait.swing_height', swing),
            double_parameter('gait.stance_duration', stance),
        ]
        future = self.gait_parameters.call_async(request)

        def finished(result_future):
            try:
                response = result_future.result()
                if not all(result.successful for result in response.results):
                    reasons = ', '.join(
                        result.reason for result in response.results
                        if not result.successful)
                    self.fail('Gait update failed: ' + reasons)
                    return
                callback()
            except Exception as error:
                self.fail('Gait parameter request failed: %s' % error)

        future.add_done_callback(finished)

    def start_floor_1(self):
        self.state = State.FLOOR_1
        self.publish_state('Starting floor 1 frontier exploration')
        self.call_frontier(ControlExploration.Request.ACTION_START)

    def send_stair_entry_goal(self):
        entry = self.get_parameter('stair_entry').value
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(entry[0])
        goal.pose.pose.position.y = float(entry[1])
        half_yaw = float(entry[2]) / 2.0
        goal.pose.pose.orientation.z = math.sin(half_yaw)
        goal.pose.pose.orientation.w = math.cos(half_yaw)
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self.stair_entry_goal_response)

    def stair_entry_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.fail('Nav2 rejected the stair-entry goal')
                return
            handle.get_result_async().add_done_callback(
                self.stair_entry_result)
        except Exception as error:
            self.fail('Failed to send stair-entry goal: %s' % error)

    def stair_entry_result(self, future):
        try:
            status = future.result().status
            if status != 4:  # action_msgs/GoalStatus STATUS_SUCCEEDED
                self.fail('Nav2 could not reach stair entry; status=%d' % status)
                return
        except Exception as error:
            self.fail('Stair-entry navigation failed: %s' % error)
            return
        self.state = State.PREPARING_STAIRS
        self.publish_state('Enabling stair gait')
        self.set_gait(True, self.begin_climb)

    def begin_climb(self):
        self.waypoint_index = 0
        self.stair_start_time = self.get_clock().now()
        self.stair_start_z = self.pose[2]
        self.state = State.CLIMBING
        self.publish_state('Closed-loop stair traversal started')

    def drive_stairs(self):
        if self.pose is None:
            return
        pose_age = (
            self.get_clock().now() - self.last_pose_time
        ).nanoseconds / 1e9
        if pose_age > self.get_parameter('odometry_timeout_s').value:
            self.fail('FAST-LIO odometry is stale (%.2f s)' % pose_age)
            return
        roll = self.pose[3]
        pitch = self.pose[4]
        maximum_tilt = self.get_parameter('maximum_tilt').value
        if abs(roll) > maximum_tilt or abs(pitch) > maximum_tilt:
            self.fail(
                'Unsafe body tilt: roll=%.2f rad, pitch=%.2f rad'
                % (roll, pitch))
            return
        elapsed = (
            self.get_clock().now() - self.stair_start_time
        ).nanoseconds / 1e9
        if elapsed > self.get_parameter('stair_timeout_s').value:
            self.fail('Stair traversal timed out')
            return
        if self.waypoint_index >= len(self.stair_waypoints):
            self.stop_robot()
            height_gain = self.pose[2] - self.stair_start_z
            required_gain = self.get_parameter(
                'stair_required_height_gain').value
            if height_gain < required_gain:
                self.fail(
                    'Reached final stair waypoint but height gain is only %.2f m'
                    % height_gain)
                return
            self.state = State.SWITCHING_FLOOR
            self.publish_state('Upper floor reached at z=%.2f m' % self.pose[2])
            if self.stair_test_only:
                self.set_gait(False, self.finish_stair_test)
            else:
                self.set_gait(False, self.switch_to_floor_2)
            return

        target_x, target_y = self.stair_waypoints[self.waypoint_index]
        x, y, _z, _roll, _pitch, yaw = self.pose
        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)
        tolerance = self.get_parameter('stair_waypoint_tolerance').value
        if distance <= tolerance:
            self.stop_robot()
            self.waypoint_index += 1
            self.publish_state(
                'Reached stair waypoint %d/%d at z=%.2f m'
                % (self.waypoint_index, len(self.stair_waypoints), self.pose[2]))
            return

        target_yaw = math.atan2(dy, dx)
        error = normalize_angle(target_yaw - yaw)
        command = Twist()
        angular_limit = self.get_parameter('stair_angular_speed').value
        if abs(error) > 0.22:
            command.angular.z = clamp(1.2 * error, -angular_limit, angular_limit)
        else:
            if self.waypoint_index < self.stair_segment_start:
                linear_limit = self.get_parameter(
                    'approach_linear_speed').value
            else:
                linear_limit = self.get_parameter(
                    'stair_linear_speed').value
            command.linear.x = clamp(0.6 * distance, 0.04, linear_limit)
            command.angular.z = clamp(0.8 * error, -angular_limit, angular_limit)
        self.cmd_vel_publisher.publish(command)

    def switch_to_floor_2(self):
        self.set_floor_projection(2, self.start_floor_2)

    def finish_stair_test(self):
        self.pending_request = False
        self.state = State.COMPLETE
        self.publish_state('Stair traversal test succeeded')

    def start_floor_2(self):
        self.state = State.FLOOR_2
        self.publish_state('Starting floor 2 frontier exploration')
        self.call_frontier(ControlExploration.Request.ACTION_START)

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())


def main():
    rclpy.init()
    node = MultifloorCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
