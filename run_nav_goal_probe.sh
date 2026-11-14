#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

OUT_DIR=/home/dobby/scan/log/codex_stack
python3 /home/dobby/scan/diagnose_nav_motion.py >"$OUT_DIR/nav_motion_probe.txt" 2>&1 &
PROBE_PID=$!
sleep 1

set +e
timeout 30 ros2 action send_goal \
  /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 1.2, y: 0.02, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' \
  --feedback >"$OUT_DIR/nav_action_result.txt" 2>&1
ACTION_RC=$?
set -e

timeout 2 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
wait "$PROBE_PID" || true

echo "=== action command rc=$ACTION_RC ==="
cat "$OUT_DIR/nav_action_result.txt"
echo '=== synchronized motion probe ==='
cat "$OUT_DIR/nav_motion_probe.txt"
echo '=== recent Nav2 log ==='
tail -100 "$OUT_DIR/nav2.log"
