#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

OUT_DIR=/home/dobby/scan/log/codex_stack
PROBE_SECONDS=40 python3 /home/dobby/scan/diagnose_nav_motion.py \
  >"$OUT_DIR/corridor_motion_probe.txt" 2>&1 &
PROBE_PID=$!
sleep 1

set +e
timeout 55 ros2 action send_goal \
  /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: -3.20, y: -3.00, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.7071068, w: 0.7071068}}}}' \
  >"$OUT_DIR/corridor_action_result.txt" 2>&1
ACTION_RC=$?
set -e

timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
wait "$PROBE_PID" || true

echo "=== action command rc=$ACTION_RC ==="
cat "$OUT_DIR/corridor_action_result.txt"
echo '=== synchronized motion probe ==='
cat "$OUT_DIR/corridor_motion_probe.txt"
echo '=== relevant Nav2 log ==='
grep -Ea 'Begin navigating|Received a goal|Reached the goal|Goal succeeded|Goal failed|Failed to make progress|Aborting|failed to create plan|collision|patience' \
  "$OUT_DIR/nav2.log" | tail -80 || true
