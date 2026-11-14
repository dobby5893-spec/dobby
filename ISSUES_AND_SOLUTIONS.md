# Go2 Mapping and Navigation Issue Log

This file records diagnosed problems, evidence, fixes, verification, and rollback points. Add a new dated entry whenever a new problem is investigated.

## 2026-09-17 — Go2 remains stationary and Nav2 reports collision ahead

### Symptom

- Nav2 accepts a goal but publishes no sustained motion.
- The controller repeatedly reports `RegulatedPurePursuitController detected collision ahead!` and eventually exceeds controller patience.
- The robot is physically stationary, but its FAST-LIO pose moves and the occupancy map places the robot inside a wall.
- Purple cost cells near the robot are inflation/obstacle costs derived from that inconsistent pose and map; they are a consequence, not the primary fault.

### Evidence

- In a 10-second stationary sample, ground truth moved about 0.004 m while FAST-LIO displaced about 0.135 m and accumulated about 1.501 m of path.
- `/livox/lidar` published `x`, `y`, `z`, `intensity`, `tag`, and `line`, but no per-point timestamp.
- The simulated `line` value was only `point_index % 4` and did not represent a real rotating scan ring.
- FAST-LIO's Mid360 handler therefore used its spinning-lidar fallback and inferred point times from yaw. That assumption is invalid for the Mid360 non-repetitive scan pattern.
- The Gazebo Mid360 sensor runs at 10 Hz while `mid360.yaml` specified 8 Hz.

### Root cause

The simulator/FAST-LIO PointCloud2 contract was incomplete. With no explicit point-time field, FAST-LIO applied the wrong timing model and performed erroneous motion compensation even while the robot was stationary. The resulting localization drift contaminated the map and made Nav2's collision checks reject movement.

### Fix

1. `livox_points_plugin.cpp` now publishes a FLOAT32 `time` field. It is zero for every point because Gazebo evaluates all rays in one callback at one simulation instant.
2. `preprocess.h` registers that `time` field in the custom PCL point type.
3. `preprocess.cpp` detects whether the PointCloud2 message contains `time`. When present, it uses the supplied seconds converted to FAST-LIO's millisecond `curvature` convention. A valid all-zero synchronized snapshot no longer falls into the yaw-based fallback.
4. The fallback remains available for other PointCloud2 sources without a `time` field and now rejects invalid line indices before indexing line arrays.
5. `mid360.yaml` now matches the Gazebo sensor rate: 10 Hz.

### Verification

- `ros2_livox_simulation` and `fast_lio` both rebuilt successfully.
- The live `/livox/lidar` schema contains FLOAT32 `time` at byte offset 16.
- FAST-LIO completed IMU and map initialization with the rebuilt packages.
- Before the fix, a 10-second stationary sample showed about 0.135 m FAST-LIO displacement and 1.501 m accumulated path while ground truth moved about 0.004 m.
- After the fix, a 10-second stationary sample showed 0.0012 m FAST-LIO displacement and 0.0768 m accumulated path while ground truth moved 0.0040 m.
- Result: stationary displacement improved by about 99.1%, and accumulated jitter path improved by about 94.9%.
- A one-metre-class Nav2 goal was then sent in verified free space. `/cmd_vel_nav` produced 84 non-zero samples, with maximum linear speed 0.100 m/s.
- During that test, ground truth displaced 0.712 m and FAST-LIO estimated 0.707 m, a difference of about 0.005 m.
- Nav2 reported `Reached the goal!` followed by `Goal succeeded`, with zero recoveries.
- Result: both localization stability and end-to-end navigation motion are verified.

### Rollback

Restore `/home/dobby/scan/mid360_timing_before_fix_20260917.tar.gz` from the workspace root, then rebuild the two affected packages.

### Test-suite note

The two packages compile and the end-to-end runtime checks above pass. Running all package tests also invokes the simulator repository's existing lint suite. That suite reports many repository-wide historical issues such as missing copyright headers, old header-guard style, formatting differences, and CMake whitespace. These occur in both modified and untouched files and are not runtime failures from this fix. They were intentionally not mass-formatted because doing so would create a large unrelated change.

## Earlier relevant incidents

### 2026-09-17 — A second Nav2 launch failed while configuring `bt_navigator`

The terminal ended with `Failed to change state for node: bt_navigator` and `Failed to bring up all requested nodes`. Process and ROS graph inspection showed two complete Nav2 instances: a background instance left by the automated navigation test at 05:01 and a terminal instance started at 05:23. Both published nodes with identical names, including `/bt_navigator`, `/controller_server`, `/planner_server`, and `/lifecycle_manager_navigation`. The duplicate names and lifecycle services caused the newer lifecycle manager to abort configuration. The old background launch and its orphaned child processes were stopped. Because the newer launch had already aborted and left its nodes inactive, it must be stopped once with Ctrl+C and relaunched. This was a process-management conflict, not a Mid360 or Nav2 parameter failure.

### Frontier exploration reported completion too early

Frontier candidate filtering was too aggressive around wall-adjacent cells. The search logic was changed so a wall-adjacent frontier is discarded only when its local four-neighbor topology proves it unusable, instead of rejecting it merely because one nearby costmap cell crosses the threshold. Unit tests were expanded and passed after rebuilding in a clean diagnostic build tree.

### Tests crashed only in the normal build tree

The normal build/install artifacts were inconsistent: the test binary loaded libraries that did not match its compiled layout and crashed inside runtime-state reset. A clean Debug build tree ran the same test successfully, and the full clean test suite passed. The lesson is to use a clean build/install pair when ABI/layout mismatch is suspected, rather than diagnosing the crash as application logic immediately.
