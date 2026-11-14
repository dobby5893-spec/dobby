#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

echo '=== FAST-LIO warnings and resets ==='
grep -Eai 'warn|error|loop|back|reset|time|sync|imu|point|nan|fail' \
  /home/dobby/scan/log/codex_stack/fast_lio.log | tail -160 || true

echo '=== input topic rates ==='
timeout 5 ros2 topic hz /livox/lidar || true
timeout 5 ros2 topic hz /imu/data || true

echo '=== FAST-LIO runtime parameters ==='
ros2 param dump /laser_mapping | sed -n '1,120p'

echo '=== recent process resource use ==='
ps -eo pid,pcpu,pmem,etime,args --sort=-pcpu | head -18
