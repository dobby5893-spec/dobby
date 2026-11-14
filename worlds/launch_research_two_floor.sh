#!/usr/bin/env bash
set -eo pipefail

cd /home/dobby/scan
source /opt/ros/humble/setup.bash
source /home/dobby/scan/install/setup.bash

# The VM desktop uses Wayland with an Xwayland display for Gazebo Classic.
export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
if [[ -z "${XAUTHORITY:-}" ]]; then
  auth_file=$(find /run/user/1000 -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print -quit)
  if [[ -n "$auth_file" ]]; then
    export XAUTHORITY="$auth_file"
  fi
fi

exec ros2 launch go2_config gazebo.launch.py \
  world:=/home/dobby/scan/worlds/research_two_floor.world \
  world_init_x:=0.0 \
  world_init_y:=0.0 \
  world_init_z:=0.275
