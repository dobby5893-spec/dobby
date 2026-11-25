#include <gtest/gtest.h>
#include "frontier_exploration_ros2/floor_completion_v5.hpp"
#include "frontier_exploration_ros2/active_residual.hpp"

namespace frontier_exploration_ros2
{
namespace
{
msg::FloorSession session()
{
  msg::FloorSession s;
  s.task_id = newSessionIdentity();
  s.floor_session_id = newSessionIdentity();
  s.explorer_instance = newSessionIdentity();
  s.map_producer = newSessionIdentity();
  s.generation = 1;
  s.phase = s.PREPARING;
  s.started_at.sec = s.heartbeat.sec = 10;
  return s;
}

msg::FloorCompletion completion(const msg::FloorSession & s)
{
  msg::FloorCompletion m;
  m.protocol_version = 5;
  m.task_id = s.task_id;
  m.floor_session_id = s.floor_session_id;
  m.session_generation = s.generation;
  m.explorer_instance = s.explorer_instance;
  m.map_producer = s.map_producer;
  m.report_id = newSessionIdentity();
  m.sequence = 1;
  m.generated_at.sec = 20;
  m.completion_status = m.VERIFIED_COMPLETE;
  m.safe_start = m.complete_routes = m.complete_views = true;
  m.stable_samples = 3;
  m.stable_seconds = 10.;
  return m;
}

TEST(FloorSession, PinnedTaskAndMonotonicFloorLifecycle)
{
  auto first = session();
  EXPECT_TRUE(validSession(first, sessionStamp(first.heartbeat)));
  EXPECT_FALSE(validSession(first, 5000000000));
  EXPECT_TRUE(canApplySession({}, first, true));
  EXPECT_FALSE(canApplySession({}, first, false));
  EXPECT_NE(first.task_id, session().task_id);

  auto active = first;
  active.phase = active.ACTIVE;
  EXPECT_TRUE(canApplySession(first, active, true));
  EXPECT_TRUE(canApplySession(active, active, false));  // Paused ROS time.
  EXPECT_FALSE(canApplySession(active, first, true));
  EXPECT_FALSE(canApplySession({}, active, true));      // Participant restart.
  EXPECT_FALSE(canApplySession(active, session(), true)); // Manager restart.

  auto next = first;
  ++next.generation;
  next.floor_session_id = newSessionIdentity();
  EXPECT_FALSE(canApplySession(active, next, true));
  active.phase = active.CLOSED;
  EXPECT_FALSE(canApplySession(active, next, false));
  EXPECT_TRUE(canApplySession(active, next, true));
  EXPECT_EQ(first.task_id, next.task_id);
  EXPECT_NE(first.floor_session_id, next.floor_session_id); // Even at the same height.
}

TEST(FloorSession, RejectsRetainedSnapshotAndProducerCounterCollision)
{
  const auto s = session();
  msg::FloorMapSnapshot map;
  map.task_id = s.task_id;
  map.floor_session_id = s.floor_session_id;
  map.session_generation = s.generation;
  map.map_producer = s.map_producer;
  map.octomap.header.stamp.sec = map.floor_map.header.stamp.sec = 11;
  EXPECT_TRUE(sessionSnapshot(s, map));
  EXPECT_FALSE(sessionSnapshot({}, map));
  auto other = s;
  other.floor_session_id = newSessionIdentity();
  EXPECT_FALSE(sessionSnapshot(other, map));
  other = s;
  other.task_id = newSessionIdentity();
  EXPECT_FALSE(sessionSnapshot(other, map));
  other = s;
  other.map_producer = newSessionIdentity();
  EXPECT_FALSE(sessionSnapshot(other, map));
  map.octomap.header.stamp.sec = 9;
  EXPECT_FALSE(sessionSnapshot(s, map));
}

TEST(FloorSession, TypedCompletionRetainsAllAuthorizationPredicates)
{
  auto s = session();
  const auto report = completion(s);
  EXPECT_TRUE(completionIdentity(report, s, 20000000000));
  EXPECT_TRUE(verifyFloorCompletion(report));
  EXPECT_FALSE(completionIdentity(report, session(), 20000000000));
  EXPECT_FALSE(completionIdentity(report, s, 19000000000));
  auto candidate = report;
  candidate.report_id.clear();
  EXPECT_FALSE(completionIdentity(candidate, s, 20000000000));
  candidate = report;
  candidate.stable_samples = 2;
  EXPECT_FALSE(verifyFloorCompletion(candidate));
  candidate = report;
  candidate.safe_start = false;
  EXPECT_FALSE(verifyFloorCompletion(candidate));
  candidate = report;
  candidate.complete_routes = false;
  EXPECT_FALSE(verifyFloorCompletion(candidate));
  for (auto status : {report.INCOMPLETE, report.BLOCKED, report.ERROR}) {
    candidate = report;
    candidate.completion_status = status;
    EXPECT_FALSE(verifyFloorCompletion(candidate));
  }

  candidate = report;
  candidate.completion_status = report.VERIFIED_WITH_RESIDUALS;
  candidate.remaining_area = candidate.residual_area = .1;
  candidate.region_count = 1;
  msg::FloorRegionEvidence region;
  region.region_id = region.dependency_id = 42;
  region.classification = region.VERIFIED_ENCLOSED;
  region.area = .1;
  candidate.regions.push_back(region);
  EXPECT_TRUE(verifyFloorCompletion(candidate));
  candidate.regions.front().classification = 0;
  EXPECT_FALSE(verifyFloorCompletion(candidate));
  candidate.regions.front() = region;
  candidate.residual_area = .2;
  EXPECT_FALSE(verifyFloorCompletion(candidate));
  candidate.residual_area = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(verifyFloorCompletion(candidate));
}

TEST(FloorSession, ProducerScopeAppliesToBothPlanningAndFormalEvidence)
{
  auto s = session();
  auto floor = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  floor->header.frame_id = "map";
  floor->info.width = floor->info.height = 2;
  floor->info.resolution = .1;
  floor->data.assign(4, 0);
  auto tree = std::make_shared<octomap::OcTree>(.1);
  tree->updateNode(octomap::point3d(0, 0, -.05), true);
  PairedFloorMap pair;
  pair.tree = tree;
  pair.floor = floor;
  pair.version = {1, 1, 23, floorDigest(*floor), {s.task_id, s.floor_session_id, s.map_producer, s.generation}};
  auto evidence = beginMapEvidence(pair, .05, .05);
  sealDependency(evidence, {0, -1, -1, 1, 1, 0, 0}, pair);
  auto report = completion(s);
  setCompletionMapEvidence(report, evidence);
  auto decoded = completionMapEvidence(report);
  ASSERT_TRUE(decoded);
  EXPECT_TRUE(mapEvidenceCurrent(*decoded, pair));
  EXPECT_TRUE(spatialEvidenceCurrent(*decoded, pair, tree.get()));
  pair.version.scope.producer = newSessionIdentity();
  EXPECT_FALSE(mapEvidenceCurrent(*decoded, pair));
  EXPECT_FALSE(spatialEvidenceCurrent(*decoded, pair, tree.get()));
}
}  // namespace
}  // namespace frontier_exploration_ros2
