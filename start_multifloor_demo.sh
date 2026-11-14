#!/usr/bin/env bash
set -eo pipefail

cd /home/dobby/scan
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash
set -u

exec ros2 launch go2_multifloor_exploration full_multifloor_demo.launch.py "$@"
