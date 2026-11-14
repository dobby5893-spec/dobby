#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

OUT=/home/dobby/scan/log/codex_stack/direct_motion_probe.txt
python3 /home/dobby/scan/diagnose_nav_motion.py >"$OUT" 2>&1 &
PROBE_PID=$!
sleep 2

timeout 3 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null

wait "$PROBE_PID"
cat "$OUT"
