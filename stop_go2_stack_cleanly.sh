#!/usr/bin/env bash
set -eo pipefail

stop_match() {
  local pattern="$1"
  pkill -INT -f "$pattern" 2>/dev/null || true
}

stop_match '/opt/ros/humble/bin/ros2 launch go2_config gazebo.launch.py'
stop_match '/opt/ros/humble/bin/ros2 launch fast_lio mapping.launch.py'
stop_match '/opt/ros/humble/bin/ros2 launch nav2_bringup navigation_launch.py'
stop_match '/opt/ros/humble/bin/ros2 run fastlio_nav_bridge fastlio_tf_bridge'
stop_match '/opt/ros/humble/bin/ros2 run octomap_server octomap_server_node'
sleep 8

for pattern in \
  '/opt/ros/humble/bin/ros2 launch go2_config gazebo.launch.py' \
  '/opt/ros/humble/bin/ros2 launch fast_lio mapping.launch.py' \
  '/opt/ros/humble/bin/ros2 launch nav2_bringup navigation_launch.py' \
  '/opt/ros/humble/bin/ros2 run fastlio_nav_bridge fastlio_tf_bridge' \
  '/opt/ros/humble/bin/ros2 run octomap_server octomap_server_node' \
  '/home/dobby/scan/install/champ_base/lib/champ_base/quadruped_controller_node' \
  '/home/dobby/scan/install/champ_base/lib/champ_base/state_estimation_node' \
  '/home/dobby/scan/install/champ_gazebo/lib/champ_gazebo/contact_sensor' \
  '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher' \
  '/home/dobby/scan/install/fast_lio/lib/fast_lio/fastlio_mapping' \
  '/home/dobby/scan/install/fastlio_nav_bridge/lib/fastlio_nav_bridge/fastlio_tf_bridge' \
  '/opt/ros/humble/lib/octomap_server/octomap_server_node' \
  '/opt/ros/humble/lib/nav2_'; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done
pkill -KILL gzserver 2>/dev/null || true
pkill -KILL gzclient 2>/dev/null || true
sleep 3

echo '=== remaining matching processes ==='
ps -eo pid,ppid,args | grep -E \
  '[g]zserver|[g]zclient|[q]uadruped_controller_node|[f]astlio_mapping|[f]astlio_tf_bridge|[o]ctomap_server_node|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[v]elocity_smoother' || true
