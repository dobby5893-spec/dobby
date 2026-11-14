#!/usr/bin/env bash
# Fresh simulation entry point: isolate ROS overlays inherited from the terminal.
set -e
set -o pipefail
script_path="$(readlink -f "${BASH_SOURCE[0]}")"
if [[ "${1:-}" != "--clean-environment" ]]; then
  exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PACKAGE_PATH -u COLCON_CURRENT_PREFIX \
    bash --noprofile --norc "$script_path" --clean-environment "$@"
fi
shift
if [[ $# -gt 1 || ($# -eq 1 && "$1" != "--check") ]]; then
  echo "Usage: $script_path [--check]" >&2
  exit 2
fi
workspace=/home/dobby/scan
cd "$workspace"
source /opt/ros/humble/setup.bash
source "$workspace/install/local_setup.bash"
source /usr/share/gazebo/setup.sh

python3 - <<'CHECK'
import runpy
from pathlib import Path
launch = '/home/dobby/scan/install/go2_multifloor_exploration/share/go2_multifloor_exploration/launch/full_multifloor_demo.launch.py'
resolved = runpy.run_path(launch)['validate_workspace_packages']()
for name, prefix in resolved.items():
    print(f'CHECK {name}: {prefix}', flush=True)
binary = (Path(resolved['frontier_exploration_ros2']) /
          'lib/frontier_exploration_ros2/frontier_explorer').resolve(strict=True)
if not binary.is_relative_to(Path('/home/dobby/scan')):
    raise RuntimeError(f'Explorer executable points outside workspace: {binary}')
print(f'EXPLORER: {binary}', flush=True)
CHECK
if [[ "${1:-}" == "--check" ]]; then
  exit 0
fi
if pgrep -u "$(id -u)" -x gzserver >/dev/null; then
  echo "Gazebo is already running. Stop the current simulation with Ctrl+C in its terminal before starting a fresh task." >&2
  exit 3
fi
checkpoint="$workspace/checkpoints/fresh_$(date +%Y%m%d_%H%M%S)_$$"
echo "Starting fresh exploration: resume=false, checkpoint=$checkpoint"
exec ros2 launch go2_multifloor_exploration full_multifloor_demo.launch.py \
  resume:=false checkpoint_dir:="$checkpoint" \
  gazebo_gui:=true navigation_rviz:=true mapping_rviz:=true start_exploration:=true
