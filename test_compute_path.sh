#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

timeout 20 ros2 action send_goal \
  /compute_path_to_pose \
  nav2_msgs/action/ComputePathToPose \
  '{goal: {header: {frame_id: map}, pose: {position: {x: 1.2, y: 0.02, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, planner_id: GridBased, use_start: false}' \
  --feedback
