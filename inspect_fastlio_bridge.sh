#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

for topic in /Odometry /fastlio/odom /odom/ground_truth; do
  echo "=== $topic info ==="
  ros2 topic info "$topic" -v || true
  echo "=== $topic sample ==="
  ros2 topic echo "$topic" --once | sed -n '1,42p' || true
done

echo '=== camera_init -> body ==='
timeout 4 ros2 run tf2_ros tf2_echo camera_init body --ros-args -p use_sim_time:=true || true
echo '=== bridge package ==='
ros2 pkg prefix fastlio_nav_bridge
echo '=== bridge source matches ==='
grep -RniE 'map.*odom|odom.*map|TransformBroadcaster|create_publisher|create_subscription' \
  /home/dobby/scan/src/fastlio_nav_bridge 2>/dev/null | head -160 || true
