#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

pkill -INT -f '/opt/ros/humble/bin/ros2 launch nav2_bringup navigation_launch.py' 2>/dev/null || true
sleep 7
pkill -KILL -f '/opt/ros/humble/bin/ros2 launch nav2_bringup navigation_launch.py' 2>/dev/null || true
pkill -KILL -f '/opt/ros/humble/lib/nav2_' 2>/dev/null || true
sleep 3

LOG=/home/dobby/scan/log/codex_stack/nav2.log
nohup ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  params_file:=/home/dobby/scan/config/nav2_go2_fastlio.yaml \
  >"$LOG" 2>&1 &
echo "$!" >/home/dobby/scan/log/codex_stack/nav2.pid
echo "nav2 pid=$! log=$LOG"

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

echo 'Nav2 did not become active in time' >&2
tail -100 "$LOG" >&2
exit 1
