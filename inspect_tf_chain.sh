#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== /odom header and pose ==='
ros2 topic echo /odom --once | sed -n '1,40p'
echo '=== map -> base_link ==='
timeout 4 ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true || true
echo '=== map -> odom ==='
timeout 4 ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true || true
echo '=== odom -> base_link ==='
timeout 4 ros2 run tf2_ros tf2_echo odom base_link --ros-args -p use_sim_time:=true || true
echo '=== TF publishers ==='
ros2 topic info /tf -v
