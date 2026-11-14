#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== pose-like topics ==='
ros2 topic list -t | grep -Ei 'odom|pose|gazebo|ground|model' || true
echo '=== odom publishers ==='
ros2 topic info /odom -v || true
echo '=== map to base_link ==='
timeout 4 ros2 run tf2_ros tf2_echo map base_link --ros-args -p use_sim_time:=true || true
echo '=== current map metadata ==='
ros2 topic echo /map --once --qos-durability transient_local --qos-reliability reliable \
  | sed -n '1,24p'
