#!/usr/bin/env bash
set -eo pipefail

SCAN_ROOT=/home/dobby/scan
LOG_ROOT="$SCAN_ROOT/log/codex_stack"
source /opt/ros/humble/setup.bash
source "$SCAN_ROOT/install/setup.bash"

# Stop any active motion before replacing localization and navigation.
timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true

for pattern in \
  '/opt/ros/humble/bin/ros2 launch fast_lio mapping.launch.py' \
  '/opt/ros/humble/bin/ros2 launch nav2_bringup navigation_launch.py' \
  '/opt/ros/humble/bin/ros2 run fastlio_nav_bridge fastlio_tf_bridge' \
  '/opt/ros/humble/bin/ros2 run octomap_server octomap_server_node'; do
  pkill -INT -f "$pattern" 2>/dev/null || true
done
sleep 7

for pattern in \
  '/opt/ros/humble/bin/ros2 launch fast_lio mapping.launch.py' \
  '/opt/ros/humble/bin/ros2 launch nav2_bringup navigation_launch.py' \
  '/opt/ros/humble/bin/ros2 run fastlio_nav_bridge fastlio_tf_bridge' \
  '/opt/ros/humble/bin/ros2 run octomap_server octomap_server_node' \
  '/home/dobby/scan/install/fast_lio/lib/fast_lio/fastlio_mapping' \
  '/home/dobby/scan/install/fastlio_nav_bridge/lib/fastlio_nav_bridge/fastlio_tf_bridge' \
  '/opt/ros/humble/lib/octomap_server/octomap_server_node' \
  '/opt/ros/humble/lib/nav2_'; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done
sleep 3

nohup ros2 launch fast_lio mapping.launch.py \
  use_sim_time:=true rviz:=false config_file:=mid360.yaml \
  >"$LOG_ROOT/fast_lio.log" 2>&1 &
echo "$!" >"$LOG_ROOT/fast_lio.pid"

for _ in $(seq 1 30); do
  if timeout 2 ros2 topic echo /cloud_registered --once >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

nohup ros2 run fastlio_nav_bridge fastlio_tf_bridge \
  --ros-args -p use_sim_time:=true \
  >"$LOG_ROOT/tf_bridge.log" 2>&1 &
echo "$!" >"$LOG_ROOT/tf_bridge.pid"
sleep 2

nohup ros2 run octomap_server octomap_server_node --ros-args \
  -p use_sim_time:=true \
  -p frame_id:=map \
  -p base_frame_id:=base_link \
  -p resolution:=0.10 \
  -p sensor_model.max_range:=8.0 \
  -p point_cloud_min_z:=0.35 \
  -p point_cloud_max_z:=1.20 \
  -p occupancy_min_z:=0.35 \
  -p occupancy_max_z:=1.00 \
  -p filter_ground_plane:=false \
  -p filter_speckles:=true \
  -p incremental_2D_projection:=false \
  -p latch:=true \
  -r cloud_in:=/livox/lidar \
  -r projected_map:=/map \
  >"$LOG_ROOT/octomap.log" 2>&1 &
echo "$!" >"$LOG_ROOT/octomap.pid"
sleep 3

nohup ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true \
  params_file:="$SCAN_ROOT/config/nav2_go2_fastlio.yaml" \
  >"$LOG_ROOT/nav2.log" 2>&1 &
echo "$!" >"$LOG_ROOT/nav2.pid"

for _ in $(seq 1 30); do
  state=$(ros2 lifecycle get /controller_server 2>/dev/null || true)
  if [[ "$state" == active* ]]; then
    echo "controller_server $state"
    ros2 lifecycle get /planner_server
    ros2 lifecycle get /bt_navigator
    exit 0
  fi
  sleep 1
done

echo 'Localization/Nav2 stack did not become active in time' >&2
exit 1
