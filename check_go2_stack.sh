#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== nodes ==='
ros2 node list | sort
echo '=== lifecycle ==='
ros2 lifecycle get /controller_server || true
ros2 lifecycle get /planner_server || true
ros2 lifecycle get /bt_navigator || true
echo '=== lidar rate ==='
timeout 4 ros2 topic hz /livox/lidar || true
echo '=== registered cloud rate ==='
timeout 4 ros2 topic hz /cloud_registered || true
echo '=== recent errors ==='
grep -Eai 'error|failed|exception|smaller than' /home/dobby/scan/log/codex_stack/*.log | tail -80 || true
