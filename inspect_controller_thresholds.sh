#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

for parameter in \
  progress_checker.plugin \
  progress_checker.required_movement_radius \
  progress_checker.movement_time_allowance \
  general_goal_checker.plugin \
  general_goal_checker.xy_goal_tolerance \
  general_goal_checker.yaw_goal_tolerance \
  general_goal_checker.stateful \
  FollowPath.use_rotate_to_heading \
  FollowPath.rotate_to_heading_angular_vel \
  FollowPath.rotate_to_heading_min_angle \
  FollowPath.max_angular_accel \
  FollowPath.goal_dist_tol; do
  printf '%s = ' "$parameter"
  ros2 param get /controller_server "$parameter" || true
done
