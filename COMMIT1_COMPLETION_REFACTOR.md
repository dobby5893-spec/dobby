# Commit 1: unified floor completion authorization

Implementation workspace: /home/dobby/scan
Feature branch: feature/floor-completion-commit1
Baseline: 702934056e6f36ebadc3e28c516bd16dc84b496d
Rollback reference: baseline/completion-commit1-20261114
No main branch overwrite or remote push.

## Behavior and call chain

FloorCoveragePolicy::completionCandidate is only a 2D audit request (including zero unknown).
Core::try_send_next_goal and Core::handle_exploration_complete request the same node audit,
without sending v1 authority or latching return-to-start completion.
Node::serviceResidualAudit keeps the original asynchronous 3D visibility/occlusion,
SurfaceGraph path/support checks, transition mask, map/epoch/pose checks and stability
(3 fresh samples and 10 seconds). It issues authority only after measured clearance passes.

Historical /explore/floor_exhausted v1-v4 and /exploration_complete Empty remain diagnostic
subscriptions in MultifloorManager. Only /explore/floor_completion (std_msgs/String) is
accepted for authority via parseVerifiedFloorCompletion/parseAuthorizedFloorCompletion.
This dedicated Commit 1 notification is NOT the future v5 map/session protocol.

The completion parser validates finite fields, nonnegative areas, per-region accounting,
unique region IDs, exact allowed classifications, stability and safety flags.
Only VERIFIED_COMPLETE (zero residual) and VERIFIED_WITH_RESIDUALS (independently
audited occluded/transition records) authorize handoff. BLOCKED is used for completed
no-route/no-known-access audits, INCOMPLETE for actionable/uncertain work, and ERROR
for inconsistent evidence or audit exceptions. Nonverified states cannot authorize.

2D excluded cells no longer bypass 3D audit. When there is no reachable 2D boundary,
the unknown component itself is checked against the known-free observation lattice.
Runtime observed_no_gain and verification_deferred evidence no longer skips validation.
Unknown raw map values are preserved. Without independent evidence, no completion.

Manager still verifies the complete stair-entry route before an actual transition.
The last configured floor may stop after formal completion without a nonexistent next
stair route. Existing idle acknowledgement, gait acknowledgement, measured motion,
landing support/stability and matching new-floor slice gates are preserved.

## Legacy Python compatibility

multifloor_exploration/coordinator.py ignores Empty completion without changing state
or calling STOP. Its old automatic floor-handoff path is intentionally fail-closed,
not adapted to duplicate the new validator. Use
go2_multifloor_exploration/full_multifloor_demo.launch.py for automatic multifloor work.
The separate stair-test motion mode is retained. No motion parameters were changed.

## Files/functions

frontier_exploration_ros2/include/frontier_exploration_ros2/floor_completion.hpp:
CompletionStatus, VerifiedFloorCompletion, parseVerifiedFloorCompletion.
floor_coverage_policy.hpp: shouldHandoff renamed completionCandidate.
frontier_explorer_core.hpp: request_floor_completion_audit callback.
frontier_explorer_node.hpp: formal completion publisher.
residual_region_audit.hpp: ResidualAudit::status/handoff, auditResidualRegions.
src/frontier_explorer_core_dispatch.cpp: try_send_next_goal, send_frontier_goal,
handle_exploration_complete callers.
src/frontier_explorer_node.cpp: callback wiring and serviceResidualAudit.
test/test_connected_dispatch.cpp and test/test_floor_coverage_policy.cpp:
strict audit and zero-residual regressions, historical expectations replaced by the
explicit stronger contract (not lower thresholds).

go2_multifloor_exploration/include/go2_multifloor_exploration/floor_handoff.hpp:
formal authorization adapter, legacy parsing retained.
src/multifloor_manager.cpp: historical/Empty diagnostic callbacks and formal subscriber.
CMakeLists.txt: test dependency on shared completion header.
test/test_floor_handoff.cpp: formal parser safety and rejection tests.
test/test_stair_protocol.py: old formats/Empty must not stop exploration before
formal audit; all pre-existing idle/gait/landing/height assertions retained.

multifloor_exploration/multifloor_exploration/coordinator.py:
exploration_complete diagnostic-only behavior.
test/test_completion_compat.py: both legacy floor states reject Empty transitions.

## Validation on Ubuntu / ROS 2 Humble

Three-package colcon build: exit 0.
frontier_exploration_ros2, go2_multifloor_exploration, multifloor_exploration built.
Existing LTO serial compilation warning remains, no compile error.

Final test cases (all passed):
test_floor_coverage_policy: 34
test_connected_dispatch: 44
test_floor_handoff: 16
test_stair_protocol: 6
test_control_service_and_idle: 16
test_region_observation_task: 30
test_floor_slice: 13
test_stair_transition: 8
test_transition_footprint: 4
test_preemption_flow: 59
Total C++/ROS cases: 230; failures: 0.
Legacy Python compatibility pytest: 1 passed.
git diff --check passed.

First regression run exposed old positive expectations for 2D exclusions and legacy
completion. Those were replaced by checks that 3D validation actually runs and
uncertain/actionable evidence refuses handoff. Final regression above passed.

Logs: /tmp/commit1-build-verified.log, /tmp/commit1-test-final.log,
/tmp/commit1-legacy-test.log. CTest XML under build/<package>/test_results.

## Limits / next stages (NOT implemented)

No Gazebo or hardware validation in this stage.
No 2D/3D generation/source synchronization (Commit 2).
No task/floor session IDs, v5 protocol, restart isolation or formal-report deduplication
(Commit 3). Existing floor-height/time checks cannot solve those races.
No claim of full integration validation (Commit 4).
Regional 3D checks can consume more existing audit budget than the former 2D shortcut;
timeout remains INCOMPLETE, never completion; budgets and all algorithm parameters unchanged.
The existing 2D navigation filters can still reject an audit-discovered opportunity;
this stage prevents false completion, it does not rewrite frontier dispatch.
FloorCompletion status is logged; only verified statuses are transported as authority.

## Rollback

Use git revert <this Commit 1 hash> on the feature branch for a reversible rollback,
or inspect baseline/completion-commit1-20261114 without changing main.
