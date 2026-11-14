# Go2 autonomous exploration workspace

Source snapshot exported from the Ubuntu workspace /home/dobby/scan.
Target environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.

## Contents

- src/: exploration, 3D surface planning, floor/stair management, FAST-LIO, LiDAR simulation, Go2/CHAMP and supporting packages.
- config/, worlds/, tools/: navigation configuration, simulation scenes and launch helpers.
- maps/: existing demonstration/map resources.
- SOURCE_SNAPSHOT.json: file hashes verified against the VM export.
- UPSTREAM_SOURCES.json: available upstream commit IDs and modified files. Third-party repositories are vendored snapshots; their original licenses remain in place.

Build/install outputs, checkpoints, trial recordings, caches and FAST-LIO runtime logs are excluded.
LiDAR scan-pattern CSVs and robot meshes are retained because runtime packages reference them.

## Build and run

Install the package/system dependencies on Ubuntu, then build with colcon:

    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    source install/local_setup.bash

The existing launch helpers contain absolute paths to /home/dobby/scan.
Use that workspace location, or update those paths before launching elsewhere.
The source repository is stored on Windows for version control; the simulation runs on Ubuntu.

Fresh exploration, including Gazebo and both RViz windows:

    /home/dobby/scan/tools/start_fresh_multifloor.sh

Checkpoint resume uses full_multifloor_demo.launch.py with resume:=true and an explicitly selected valid checkpoint_dir.
Runtime checkpoints themselves are not included in this repository.

## Validation status

Automated checks and frozen-map replays exist under package test directories.
End-to-end physical stair ascent in the two-floor simulation has not yet been demonstrated.
Exploration completion, residual-region classification and stair handoff remain under development.

## Linux executable permissions after cloning

The Windows Git import stored files with mode 100644. The original VM executable paths (317 files) are preserved in `LINUX_EXECUTABLE_PATHS.json`. After cloning on Ubuntu, run this once from the repository root, before building or launching:

```bash
python3 - <<'PY'
import json
from pathlib import Path
root = Path.cwd().resolve()
paths = json.loads((root / "LINUX_EXECUTABLE_PATHS.json").read_text())
for name in paths:
    path = (root / name).resolve()
    path.relative_to(root)
    if not path.is_file():
        raise FileNotFoundError(name)
for name in paths:
    path = root / name
    path.chmod(path.stat().st_mode | 0o111)
print(f"Restored executable permissions for {len(paths)} files")
PY
```

Git normalized CRLF text files to LF during import. Original byte hashes in `SOURCE_SNAPSHOT.json` describe the VM export; text hashes can therefore differ only because of line endings. Original export bytes and file modes are also retained in the local `.transfer/go2-source.tar.gz` archive, which is excluded from Git.
