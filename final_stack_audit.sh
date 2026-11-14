#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== runtime parameters ==='
ros2 param get /controller_server controller_frequency
ros2 param get /controller_server progress_checker.required_movement_radius
ros2 param get /controller_server progress_checker.movement_time_allowance
ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance
ros2 param get /laser_mapping mapping.extrinsic_est_en
ros2 param get /laser_mapping mapping.extrinsic_T

echo '=== Nav2 warning counts since last restart ==='
printf 'missed control loops: '
grep -c 'Control loop missed' /home/dobby/scan/log/codex_stack/nav2.log || true
printf 'robot out of bounds: '
grep -c 'Robot is out of bounds' /home/dobby/scan/log/codex_stack/nav2.log || true
printf 'goal succeeded: '
grep -c 'Goal succeeded' /home/dobby/scan/log/codex_stack/nav2.log || true

echo '=== lifecycle ==='
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
