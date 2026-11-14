#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== active thresholds ==='
ros2 param get /controller_server progress_checker.required_movement_radius
ros2 param get /controller_server progress_checker.movement_time_allowance
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance

echo '=== same corridor goal ==='
set +e
timeout 25 ros2 action send_goal \
  /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -3.20, y: -3.00, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.7071068, w: 0.7071068}}}}'
RC=$?
set -e

timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
echo "action command rc=$RC"
tail -40 /home/dobby/scan/log/codex_stack/nav2.log
