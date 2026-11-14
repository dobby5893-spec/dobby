#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== base_link -> livox_frame ==='
timeout 5 ros2 run tf2_ros tf2_echo base_link livox_frame --ros-args -p use_sim_time:=true || true
echo '=== livox_frame -> base_link ==='
timeout 5 ros2 run tf2_ros tf2_echo livox_frame base_link --ros-args -p use_sim_time:=true || true
echo '=== URDF/config references ==='
grep -RniE 'livox_frame|0\.011|0\.02329|0\.04412|extrinsic_(T|R|est)' \
  /home/dobby/scan/src/go2_config \
  /home/dobby/scan/src/FAST_LIO_ROS2 \
  /home/dobby/scan/src 2>/dev/null | head -200 || true
