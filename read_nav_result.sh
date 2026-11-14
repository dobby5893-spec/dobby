#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash
echo '=== latest action status ==='
ros2 topic echo /navigate_to_pose/_action/status --once
echo '=== recent Nav2 log ==='
tail -80 /home/dobby/scan/log/codex_stack/nav2.log
