#!/usr/bin/env bash
set -eo pipefail

SCAN_ROOT=/home/dobby/scan
LOG_ROOT="$SCAN_ROOT/log/codex_stack"
mkdir -p "$LOG_ROOT"

source /opt/ros/humble/setup.bash
source "$SCAN_ROOT/install/setup.bash"
export DISPLAY=:0
export XAUTHORITY=/home/dobby/.Xauthority

start_component() {
  local name="$1"
  shift
  nohup "$@" >"$LOG_ROOT/$name.log" 2>&1 &
  echo "$!" >"$LOG_ROOT/$name.pid"
  echo "$name pid=$! log=$LOG_ROOT/$name.log"
}

start_component gazebo ros2 launch go2_config gazebo.launch.py

for _ in $(seq 1 40); do
  if timeout 2 ros2 topic echo /clock --once >/dev/null 2>&1; then
    echo "clock is active"
    break
  fi
  sleep 1
done

start_component fast_lio ros2 launch fast_lio mapping.launch.py \
  use_sim_time:=true rviz:=false config_file:=mid360.yaml
sleep 4

start_component tf_bridge ros2 run fastlio_nav_bridge fastlio_tf_bridge \
  --ros-args -p use_sim_time:=true
sleep 2

start_component octomap ros2 run octomap_server octomap_server_node --ros-args \
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
  -r projected_map:=/map
sleep 3

start_component nav2 ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  params_file:="$SCAN_ROOT/config/nav2_go2_fastlio.yaml"

echo "stack launch commands submitted"
