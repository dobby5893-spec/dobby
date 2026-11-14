#!/usr/bin/env bash
set -eo pipefail

SCAN_ROOT=/home/dobby/scan
LOG_ROOT="$SCAN_ROOT/log/codex_stack"

if [[ -d "$LOG_ROOT" ]]; then
  for pid_file in "$LOG_ROOT"/*.pid; do
    [[ -e "$pid_file" ]] || continue
    pid=$(cat "$pid_file")
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
fi

sleep 8

# Only processes belonging to this stack are matched here. This removes children
# left behind if a ros2 launch parent exited before completing its shutdown.
pkill -INT -f '/home/dobby/scan/install/fast_lio/lib/fast_lio/fastlio_mapping' || true
pkill -INT -f '/home/dobby/scan/install/fastlio_nav_bridge/lib/fastlio_nav_bridge/fastlio_tf_bridge' || true
pkill -INT -f '/opt/ros/humble/lib/octomap_server/octomap_server_node' || true
pkill -INT -f '/opt/ros/humble/lib/nav2_' || true
pkill -INT -f '/opt/ros/humble/lib/gazebo_ros/gzserver' || true
pkill -INT -f '/opt/ros/humble/lib/gazebo_ros/gzclient' || true
sleep 5

rm -f "$LOG_ROOT"/*.pid
exec "$SCAN_ROOT/start_go2_diagnostic_stack.sh"
