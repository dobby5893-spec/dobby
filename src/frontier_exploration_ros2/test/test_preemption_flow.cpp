/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <gtest/gtest.h>

#include <action_msgs/msg/goal_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{
namespace
{

// Covers goal-update/cancel/reselection/result lifecycle edges in FrontierExplorerCore.

class FakeGoalHandle : public GoalHandleInterface
{
public:
  void cancel_goal_async(
    std::function<void(bool accepted, const std::string & error_message)> callback) override
  {
    cancel_calls += 1;
    pending_callback = std::move(callback);
  }

  void resolve_cancel(bool accepted = true, const std::string & error_message = "")
  {
    if (pending_callback) {
      pending_callback(accepted, error_message);
      pending_callback = nullptr;
    }
  }

  int cancel_calls{0};

private:
  std::function<void(bool accepted, const std::string & error_message)> pending_callback;
};

geometry_msgs::msg::Pose make_pose(double x = 0.0, double y = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int default_value)
{
  // Shared synthetic grid builder for map/costmap-only unit scenarios.
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = "map";
  msg.info.width = static_cast<uint32_t>(width);
  msg.info.height = static_cast<uint32_t>(height);
  msg.info.resolution = 1.0;
  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(static_cast<std::size_t>(width * height), static_cast<int8_t>(default_value));
  return msg;
}

FrontierCandidate make_frontier(double x, double y, int size = 1)
{
  return FrontierCandidate{{x, y}, {x, y}, size};
}

void set_cell(nav_msgs::msg::OccupancyGrid & msg, int x, int y, int value)
{
  const int width = static_cast<int>(msg.info.width);
  msg.data[static_cast<std::size_t>(y * width + x)] = static_cast<int8_t>(value);
}

std::unique_ptr<FrontierExplorerCore> make_preemption_core(std::vector<std::string> * info_logs = nullptr)
{
  // Creates an ACTIVE frontier-goal core so tests can exercise goal-update entrypoints directly.
  FrontierExplorerCoreParams params;
  params.goal_preemption_enabled = true;
  params.goal_skip_on_blocked_goal = true;
  params.goal_preemption_min_interval_s = 0.0;
  params.post_goal_settle_enabled = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose());
    };
  callbacks.log_info = [info_logs](const std::string & message) {
      if (info_logs != nullptr) {
        info_logs->push_back(message);
      }
    };
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  auto core = std::make_unique<FrontierExplorerCore>(params, callbacks);

  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);

  core->map = OccupancyGrid2d(map_msg);
  core->costmap = OccupancyGrid2d(costmap_msg);
  core->local_costmap.reset();
  core->map_generation = 1;
  core->costmap_generation = 1;
  core->local_costmap_generation = 1;

  core->set_goal_state(GoalLifecycleState::ACTIVE);
  core->active_goal_frontier = make_frontier(1.0, 1.0);
  core->active_goal_frontiers = {make_frontier(1.0, 1.0)};
  core->active_goal_kind = "frontier";
  core->active_goal_sent_time_ns = 0;
  core->current_dispatch_id = 1;

  return core;
}

// Replacement and cancellation policy behavior.
TEST(PreemptionFlowTests, ReselectionReplacementDispatchesWithoutCancel)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };

  FrontierSequence frontier_sequence{make_frontier(2.0, 2.0)};
  core->request_frontier_reselection(frontier_sequence, make_pose(), "mrtsp", "replacement");
  core->request_frontier_reselection(frontier_sequence, make_pose(), "mrtsp", "replacement");

  EXPECT_EQ(dispatch_calls, 1);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
}

TEST(PreemptionFlowTests, ThrottledMapProcessingCoalescesMultipleRawUpdatesIntoSingleProcessedRefresh)
{
  FrontierExplorerCoreParams params;
  params.map_processing_rate_hz = 5.0;
  params.post_goal_settle_enabled = false;
  params.return_to_start_on_complete = false;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.wait_for_action_server = [](double) {return true;};

  int dispatch_calls = 0;
  int frontier_search_calls = 0;
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 10}};
      result.robot_map_cell = {1, 1};
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto costmap_msg = build_grid(20, 20, 0);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.costmap_generation = 1;

  core.ingestRawMapUpdate(OccupancyGrid2d(build_grid(20, 20, 0)));
  core.ingestRawMapUpdate(OccupancyGrid2d(build_grid(20, 20, 0)));
  ASSERT_TRUE(core.decision_map_dirty);

  core.processPendingMapUpdate();

  EXPECT_FALSE(core.decision_map_dirty);
  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_EQ(dispatch_calls, 1);

  core.processPendingMapUpdate();
  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_EQ(dispatch_calls, 1);
}

TEST(PreemptionFlowTests, UrgentRawMapUpdateDoesNotRefreshDecisionMapWhenPreemptionExitsEarly)
{
  auto core = make_preemption_core();
  core->params.goal_preemption_enabled = false;
  core->decision_map = OccupancyGrid2d(build_grid(20, 20, 0));
  core->decision_map_generation = 7;

  core->ingestRawMapUpdate(OccupancyGrid2d(build_grid(20, 20, 0)));
  ASSERT_TRUE(core->decision_map_dirty);

  core->handleUrgentRawMapUpdateForActiveGoal();

  EXPECT_TRUE(core->decision_map_dirty);
  EXPECT_EQ(core->decision_map_generation, 7);
}

TEST(PreemptionFlowTests, ReplacementDebounceTracksSelectedFrontierOnly)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };

  const FrontierCandidate selected_frontier{
    {2.0, 2.0},
    {2.0, 2.0},
    {2, 2},
    {2, 2},
    {2.0, 2.0},
    std::nullopt,
    6};
  const FrontierCandidate look_ahead_a{
    {4.0, 4.0},
    {4.0, 4.0},
    {4, 4},
    {4, 4},
    {4.0, 4.0},
    std::nullopt,
    6};
  const FrontierCandidate look_ahead_b{
    {6.0, 6.0},
    {6.0, 6.0},
    {6, 6},
    {6, 6},
    {6.0, 6.0},
    std::nullopt,
    6};

  const FrontierSequence first_sequence{selected_frontier, look_ahead_a};
  const FrontierSequence second_sequence{selected_frontier, look_ahead_b};

  core->request_frontier_reselection(first_sequence, make_pose(), "mrtsp", "replacement");
  core->request_frontier_reselection(second_sequence, make_pose(), "mrtsp", "replacement");

  EXPECT_EQ(dispatch_calls, 1);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
}

TEST(PreemptionFlowTests, BlockedGoalWithoutReplacementUsesExplicitCancel)
{
  std::vector<std::string> info_logs;
  std::vector<std::string> debug_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->callbacks.log_debug = [&debug_logs](const std::string & message) {
      debug_logs.push_back(message);
    };

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers.clear();
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 1);
  ASSERT_FALSE(debug_logs.empty());
  EXPECT_NE(debug_logs.back().find("no replacement frontier is available"), std::string::npos);
}

TEST(PreemptionFlowTests, BlockedGoalReplacementLogsSkipVerb)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->replacement_required_hits = 1;

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(dispatch_calls, 1);
  ASSERT_FALSE(info_logs.empty());
  bool found_skip_log = false;
  for (const auto & message : info_logs) {
    if (message.find("Skipping blocked frontier goal:") != std::string::npos) {
      found_skip_log = true;
      break;
    }
  }
  EXPECT_TRUE(found_skip_log);
}

TEST(PreemptionFlowTests, DispatchSkipsBlockedFrontierAndUsesNextSequenceTarget)
{
  FrontierExplorerCoreParams params;
  params.goal_skip_on_blocked_goal = true;
  params.occ_threshold = 60;

  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  set_cell(costmap_msg, 1, 1, 60);

  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);

  int dispatch_calls = 0;
  std::optional<GoalDispatchRequest> dispatched_request;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_request](
    const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_request = request;
    };

  const FrontierSequence frontier_sequence{
    make_frontier(1.0, 1.0),
    make_frontier(2.0, 2.0),
    make_frontier(3.0, 3.0),
  };

  EXPECT_TRUE(core.send_frontier_goal(frontier_sequence, make_pose(), "Sending frontier goal"));

  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_TRUE(dispatched_request.has_value());
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.x, 2.0, 1e-9);
	EXPECT_NEAR(dispatched_request->goal_pose.pose.position.y, 2.0, 1e-9);
	ASSERT_EQ(dispatched_request->frontier_sequence.size(), 2U);

	auto expected_frontier = make_frontier(2.0, 2.0);
	expected_frontier.goal_point = std::pair<double, double>{2.0, 2.0};

	EXPECT_TRUE(core.are_frontiers_equivalent(
	  dispatched_request->frontier_sequence.front(),
	  expected_frontier));
}

TEST(PreemptionFlowTests, DispatchSkipsUnsupported3DGoalAndPreservesVerifiedPose)
{
  FrontierExplorerCore core(FrontierExplorerCoreParams{},FrontierExplorerCoreCallbacks{});
  core.map=OccupancyGrid2d(build_grid(20,20,0));
  core.costmap=OccupancyGrid2d(build_grid(20,20,0));
  std::optional<GoalDispatchRequest> sent;
  core.callbacks.wait_for_action_server=[](double){return true;};
  core.callbacks.dispatch_goal_request=[&](const GoalDispatchRequest & request){sent=request;};
  int checks=0;
  core.callbacks.validate_frontier_goal=[&](const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::PoseStamped & goal)->std::optional<geometry_msgs::msg::PoseStamped> {
    if (++checks==1) {return std::nullopt;}
    auto result=goal;
    result.pose.position.x=2.4; result.pose.position.y=2.2; result.pose.position.z=-0.20;
    result.pose.orientation.z=0.0; result.pose.orientation.w=1.0;
    return result;
  };
  ASSERT_TRUE(core.send_frontier_goal({make_frontier(1,1),make_frontier(3,3)},make_pose(),"test"));
  ASSERT_TRUE(sent); EXPECT_EQ(checks,2);
  EXPECT_DOUBLE_EQ(sent->goal_pose.pose.position.x,2.4);
  EXPECT_DOUBLE_EQ(sent->goal_pose.pose.position.z,-0.20);
  EXPECT_DOUBLE_EQ(sent->goal_pose.pose.orientation.z,0.0);
  ASSERT_TRUE(sent->frontier); EXPECT_DOUBLE_EQ(*sent->frontier->goal_z,-0.20);
}

TEST(PreemptionFlowTests, Unsupported3DGoalsNeverReachNavigationAction)
{
  FrontierExplorerCore core(FrontierExplorerCoreParams{},FrontierExplorerCoreCallbacks{});
  core.map=OccupancyGrid2d(build_grid(20,20,0));
  core.costmap=OccupancyGrid2d(build_grid(20,20,0));
  int sends=0;
  core.callbacks.dispatch_goal_request=[&](const GoalDispatchRequest &){++sends;};
  core.callbacks.validate_frontier_goal=[](const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::PoseStamped &)->std::optional<geometry_msgs::msg::PoseStamped>{return std::nullopt;};
  EXPECT_FALSE(core.send_frontier_goal({make_frontier(1,1)},make_pose(),"test"));
  EXPECT_EQ(sends,0);
}

TEST(PreemptionFlowTests, MrtspDispatchAdjustsCloseTargetToClosestFarEnoughFreePoint)
{
  FrontierExplorerCoreParams params;
  params.frontier_selection_min_distance = 2.0;

  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 100);
  set_cell(costmap_msg, 5, 5, 0);
  set_cell(costmap_msg, 6, 5, 0);
  set_cell(costmap_msg, 7, 5, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);

  int dispatch_calls = 0;
  std::optional<GoalDispatchRequest> dispatched_request;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_request](
    const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_request = request;
    };

  const FrontierSequence frontier_sequence{
    FrontierCandidate{
      {5.0, 5.0},
      {5.0, 5.0},
      {5, 5},
      {5, 5},
      {5.0, 5.0},
      std::nullopt,
      8},
  };

  EXPECT_TRUE(core.send_frontier_goal(frontier_sequence, make_pose(5.1, 5.5), "Sending frontier goal"));

  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_TRUE(dispatched_request.has_value());
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.x, 7.5, 1e-9);
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.y, 5.5, 1e-9);
}

TEST(PreemptionFlowTests, FarEnoughFrontierGoalIsNotRelocated)
{
  FrontierExplorerCoreParams params;
  params.frontier_selection_min_distance = 0.8;

  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);

  int dispatch_calls = 0;
  std::optional<GoalDispatchRequest> dispatched_request;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_request](
    const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_request = request;
    };

  const FrontierSequence frontier_sequence{
    make_frontier(5.0, 5.0),
  };

  EXPECT_TRUE(core.send_frontier_goal(
      frontier_sequence,
      make_pose(1.0, 1.0),
      "Sending frontier goal"));

  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_TRUE(dispatched_request.has_value());
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.x, 5.0, 1e-9);
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.y, 5.0, 1e-9);
}

TEST(PreemptionFlowTests, MrtspDispatchSkipsReplacementPointsThatRemainTooCloseToTarget)
{
  FrontierExplorerCoreParams params;
  params.frontier_selection_min_distance = 2.0;

  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 100);
  set_cell(costmap_msg, 4, 3, 0);
  set_cell(costmap_msg, 5, 5, 0);
  set_cell(costmap_msg, 6, 5, 0);
  set_cell(costmap_msg, 7, 5, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);

  int dispatch_calls = 0;
  std::optional<GoalDispatchRequest> dispatched_request;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_request](
    const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_request = request;
    };

  const FrontierSequence frontier_sequence{
    FrontierCandidate{
      {5.0, 5.0},
      {5.0, 5.0},
      {5, 5},
      {5, 5},
      {5.0, 5.0},
      std::nullopt,
      8},
  };

  EXPECT_TRUE(core.send_frontier_goal(frontier_sequence, make_pose(5.1, 5.5), "Sending frontier goal"));

  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_TRUE(dispatched_request.has_value());
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.x, 7.5, 1e-9);
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.y, 5.5, 1e-9);
}

TEST(PreemptionFlowTests, EscapeModeRetriesWithoutMinDistanceFiltersAndBypassesDispatchAdjustment)
{
  FrontierExplorerCoreParams params;
  params.escape_enabled = true;
  params.frontier_candidate_min_goal_distance_m = 2.0;
  params.frontier_selection_min_distance = 2.0;
  params.return_to_start_on_complete = false;

  std::vector<double> observed_min_goal_distances;
  int dispatch_calls = 0;
  std::optional<GoalDispatchRequest> dispatched_request;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(5.1, 5.5));
    };
  callbacks.wait_for_action_server = [](double) {return true;};
  callbacks.dispatch_goal_request = [&dispatch_calls, &dispatched_request](
    const GoalDispatchRequest & request) {
      dispatch_calls += 1;
      dispatched_request = request;
    };
  callbacks.frontier_search = [&observed_min_goal_distances](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double min_goal_distance,
    bool)
    {
      observed_min_goal_distances.push_back(min_goal_distance);
      FrontierSearchResult result;
      result.robot_map_cell = {5, 5};
      if (min_goal_distance > 0.0) {
        return result;
      }
      result.frontiers = {make_frontier(5.0, 5.0, 8)};
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 100);
  set_cell(costmap_msg, 5, 5, 0);
  set_cell(costmap_msg, 7, 5, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.map_generation = 1;
  core.costmap_generation = 1;

  core.try_send_next_goal();

  ASSERT_EQ(observed_min_goal_distances.size(), 2U);
  EXPECT_DOUBLE_EQ(observed_min_goal_distances[0], 2.0);
  EXPECT_DOUBLE_EQ(observed_min_goal_distances[1], 0.0);
  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_TRUE(dispatched_request.has_value());
  EXPECT_NE(dispatched_request->description.find("escape"), std::string::npos);
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.x, 5.0, 1e-9);
  EXPECT_NEAR(dispatched_request->goal_pose.pose.position.y, 5.0, 1e-9);
}

TEST(PreemptionFlowTests, EscapeModeDisabledDoesNotRetryWithoutMinDistanceFilters)
{
  FrontierExplorerCoreParams params;
  params.escape_enabled = false;
  params.frontier_candidate_min_goal_distance_m = 2.0;
  params.return_to_start_on_complete = false;

  std::vector<double> observed_min_goal_distances;
  int dispatch_calls = 0;

  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = []() {return int64_t{5'000'000'000};};
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(5.1, 5.5));
    };
  callbacks.wait_for_action_server = [](double) {return true;};
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.frontier_search = [&observed_min_goal_distances](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double min_goal_distance,
    bool)
    {
      observed_min_goal_distances.push_back(min_goal_distance);
      FrontierSearchResult result;
      result.robot_map_cell = {5, 5};
      if (min_goal_distance <= 0.0) {
        result.frontiers = {make_frontier(5.0, 5.0, 8)};
      }
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.map_generation = 1;
  core.costmap_generation = 1;

  core.try_send_next_goal();

  ASSERT_EQ(observed_min_goal_distances.size(), 1U);
  EXPECT_DOUBLE_EQ(observed_min_goal_distances[0], 2.0);
  EXPECT_EQ(dispatch_calls, 0);
}

TEST(PreemptionFlowTests, FrontierCostStatusUsesConfiguredOccupancyThreshold)
{
  FrontierExplorerCoreParams params;
  params.goal_skip_on_blocked_goal = true;
  params.occ_threshold = 60;

  FrontierExplorerCore core(params, FrontierExplorerCoreCallbacks{});
  auto costmap_msg = build_grid(20, 20, 0);
  set_cell(costmap_msg, 1, 1, 55);
  core.costmap = OccupancyGrid2d(costmap_msg);

  const auto target = std::optional<FrontierLike>{make_frontier(1.0, 1.0)};
  EXPECT_FALSE(core.frontier_cost_status(target).has_value());

  set_cell(costmap_msg, 1, 1, 60);
  core.costmap = OccupancyGrid2d(costmap_msg);
  EXPECT_TRUE(core.frontier_cost_status(target).has_value());
}

TEST(PreemptionFlowTests, PreemptionCancelsActiveGoalAndDefersReselectionWhenSettleEnabled)
{
  std::vector<std::string> info_logs;
  FrontierExplorerCoreParams params;
  params.goal_preemption_enabled = true;
  params.goal_skip_on_blocked_goal = true;
  params.goal_preemption_min_interval_s = 0.0;
  params.post_goal_settle_enabled = true;
  params.post_goal_min_settle = 0.0;

  int64_t now_ns = 5'000'000'000;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {
      now_ns += 100'000'000;
      return now_ns;
    };
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose());
    };
  callbacks.log_info = [&info_logs](const std::string & message) {
      info_logs.push_back(message);
    };
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.local_costmap.reset();
  core.map_generation = 1;
  core.costmap_generation = 1;
  core.local_costmap_generation = 1;
  core.set_goal_state(GoalLifecycleState::ACTIVE);
  core.active_goal_frontier = make_frontier(1.0, 1.0);
  core.active_goal_frontiers = {make_frontier(1.0, 1.0)};
  core.active_goal_kind = "frontier";
  core.active_goal_sent_time_ns = 0;
  core.current_dispatch_id = 1;

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_handle = fake_handle;

  int dispatch_calls = 0;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core.callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };
  core.replacement_required_hits = 1;

  core.consider_preempt_active_goal("map");

  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core.pending_frontier_sequence.empty());
  EXPECT_FALSE(core.awaiting_map_refresh);
  EXPECT_FALSE(core.post_goal_settle_active);
}

TEST(PreemptionFlowTests, SupersededResultCallbackDoesNotClearActiveState)
{
  auto core = make_preemption_core();
  core->current_dispatch_id = 2;
  core->set_goal_state(GoalLifecycleState::ACTIVE);

  core->get_result_callback(
    1,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
    0,
    "");

  EXPECT_TRUE(core->goal_in_progress);
}

TEST(PreemptionFlowTests, LateRejectedCancelResponseDoesNotResurrectCompletedGoal)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  core->request_active_goal_cancel("cancel for completed goal race");
  ASSERT_EQ(fake_handle->cancel_calls, 1);
  ASSERT_TRUE(core->cancel_request_in_progress);
  ASSERT_EQ(core->goal_state, GoalLifecycleState::CANCELING);

  core->get_result_callback(
    core->current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_CANCELED,
    0,
    "");
  ASSERT_FALSE(core->goal_in_progress);
  ASSERT_FALSE(core->cancel_request_in_progress);

  fake_handle->resolve_cancel(false, "");

  EXPECT_FALSE(core->goal_in_progress);
  EXPECT_FALSE(core->cancel_request_in_progress);
  EXPECT_EQ(core->goal_state, GoalLifecycleState::IDLE);
}

TEST(PreemptionFlowTests, CostmapTriggerOnlyChecksBlockingAndSkipsReselection)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  int frontier_search_calls = 0;
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      return result;
    };

  core->consider_preempt_active_goal("costmap");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_TRUE(core->active_goal_blocked_reason.has_value());
}

TEST(PreemptionFlowTests, BlockedGoalSkipCanBeDisabledViaParameter)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_enabled = false;

  auto blocked_costmap = build_grid(20, 20, 0);
  set_cell(blocked_costmap, 1, 1, 100);
  core->costmap = OccupancyGrid2d(blocked_costmap);

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_FALSE(core->active_goal_blocked_reason.has_value());
}

TEST(PreemptionFlowTests, VisibleGainPreemptionCanBeDisabledViaParameter)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;

  int frontier_search_calls = 0;
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
}

TEST(PreemptionFlowTests, VisibleGainGateSkipsSnapshotSearchWhenGoalStillHasVisibleReveal)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_TRUE(core->pending_frontier_sequence.empty());
}

TEST(PreemptionFlowTests, VisibleGainGateSkipsSmallClusterEvenBelowMinRevealThreshold)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 2.0;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 1};
  core->active_goal_frontiers = {*core->active_goal_frontier};

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_TRUE(core->pending_frontier_sequence.empty());
}

TEST(PreemptionFlowTests, VisibleGainExhaustionCancelsAndDefersSnapshotReselection)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 4.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->replacement_required_hits = 1;
  const FrontierCandidate active_frontier{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontier = active_frontier;
  core->active_goal_frontiers = {*core->active_goal_frontier};

  auto map_msg = build_grid(12, 12, 0);
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->pending_frontier_sequence.empty());
}

TEST(PreemptionFlowTests, LowGainPreemptionCancelsWithoutSnapshotReselection)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_min_interval_s = 0.5;
  core->params.goal_preemption_lidar_range_m = 4.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->replacement_required_hits = 1;

  const FrontierCandidate active_frontier{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontier = active_frontier;
  core->active_goal_frontiers = {*core->active_goal_frontier};

  auto map_msg = build_grid(12, 12, 0);
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  core->callbacks.frontier_search = [&frontier_search_calls, active_frontier](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {active_frontier};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->pending_frontier_sequence.empty());
}

TEST(PreemptionFlowTests, CompletionDistanceTreatsNearFrontierAsCompleteWithVisibleGainGate)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->replacement_required_hits = 1;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.1, 5.0));
    };

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {
        FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8},
        FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceIsDisabledAtZero)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = true;
  core->params.goal_preemption_lidar_range_m = 6.0;
  core->params.goal_preemption_lidar_fov_deg = 90.0;
  core->params.goal_preemption_lidar_ray_step_deg = 1.0;
  core->params.goal_preemption_complete_if_within_m = 0.0;
  core->params.goal_preemption_lidar_min_reveal_length_m = 0.5;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.1, 5.0));
    };

  auto map_msg = build_grid(12, 12, -1);
  for (int x = 0; x <= 4; ++x) {
    set_cell(map_msg, x, 5, 0);
  }
  core->map = OccupancyGrid2d(map_msg);

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
}

TEST(PreemptionFlowTests, CompletionDistanceTriggersWhenVisibleGainPreemptionDisabled)
{
  std::vector<std::string> info_logs;
  auto core = make_preemption_core(&info_logs);
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->replacement_required_hits = 1;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.1, 5.0));
    };

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {
        FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8},
        FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceUsesDispatchedGoalPointRatherThanCentroid)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->active_goal_frontier = FrontierCandidate{
    {5.0, 5.0},
    {4.0, 5.0},
    {4, 5},
    {4, 5},
    {4.0, 5.0},
    std::pair<double, double>{4.0, 5.0},
    8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(4.1, 5.0));
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_TRUE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceUsesAdjustedDispatchPoseRatherThanOriginalFrontierPoint)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {4.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->active_goal_pose.emplace();
  core->active_goal_pose->header.frame_id = "map";
  core->active_goal_pose->pose = make_pose(6.0, 5.0);
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(4.1, 5.0));
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 0);
  EXPECT_FALSE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceDoesNotReselectWhenOutsideThresholdAndPreemptionDisabled)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(2.0, 5.0));
    };

  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{8.0, 8.0}, {8.0, 8.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(frontier_search_calls, 0);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(fake_handle->cancel_calls, 0);
}

TEST(PreemptionFlowTests, CompletionDistanceRedispatchGuardUsesCompletedGoalPointRatherThanCentroid)
{
  auto core = make_preemption_core();
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->params.frontier_candidate_min_goal_distance_m = 0.0;
  core->set_goal_state(GoalLifecycleState::IDLE);
  core->distance_completed_frontier = FrontierCandidate{
    {5.0, 5.0},
    {4.0, 5.0},
    {4, 5},
    {4, 5},
    {4.0, 5.0},
    std::pair<double, double>{4.0, 5.0},
    8};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(4.1, 5.0));
    };

  double observed_min_goal_distance = -1.0;
  core->callbacks.frontier_search = [&observed_min_goal_distance](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double min_goal_distance,
    bool)
    {
      observed_min_goal_distance = min_goal_distance;
      FrontierSearchResult result;
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->try_send_next_goal();

  EXPECT_DOUBLE_EQ(observed_min_goal_distance, 0.25);
  EXPECT_TRUE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, CompletionDistanceCancelsWithoutReplacementAndAvoidsRedispatch)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.25;
  core->replacement_required_hits = 1;
  core->active_goal_frontier = FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8};
  core->active_goal_frontiers = {*core->active_goal_frontier};
  core->callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(3.1, 5.0));
    };

  int completion_calls = 0;
  int frontier_search_calls = 0;
  int dispatch_calls = 0;
  core->callbacks.on_exploration_complete = [&completion_calls]() {
      completion_calls += 1;
    };
  core->callbacks.wait_for_action_server = [](double) {return true;};
  core->callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double min_goal_distance,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      if (min_goal_distance >= 0.25) {
        result.robot_map_cell = {0, 0};
        return result;
      }
      result.frontiers = {FrontierCandidate{{4.0, 5.0}, {3.0, 5.0}, 8}};
      result.robot_map_cell = {0, 0};
      return result;
    };

  core->consider_preempt_active_goal("map");

  EXPECT_EQ(fake_handle->cancel_calls, 1);
  EXPECT_EQ(dispatch_calls, 0);
  ASSERT_TRUE(core->distance_completed_frontier.has_value());

  core->get_result_callback(
    core->current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_CANCELED,
    0,
    "");
  core->try_send_next_goal();

  EXPECT_EQ(frontier_search_calls, 1);
  EXPECT_EQ(dispatch_calls, 0);
  EXPECT_EQ(completion_calls, 1);
  EXPECT_TRUE(core->return_to_start_completed);
}

// Queue/lifecycle and completion gating behavior.
TEST(PreemptionFlowTests, ReplacementQueueStaysSingleWithLatestSequence)
{
  auto core = make_preemption_core();
  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = fake_handle;

  core->replacement_required_hits = 1;
  core->callbacks.wait_for_action_server = [](double) {return false;};

  FrontierSequence first{make_frontier(2.0, 2.0)};
  FrontierSequence second{make_frontier(3.0, 3.0)};

  core->request_frontier_reselection(first, make_pose(), "mrtsp", "first");
  core->request_frontier_reselection(second, make_pose(), "mrtsp", "second");

  EXPECT_TRUE(core->are_frontier_sequences_equivalent(core->pending_frontier_sequence, second));
}

TEST(PreemptionFlowTests, ReturnToStartCompletedStopsFurtherFrontierSearch)
{
  auto core = make_preemption_core();
  core->set_goal_state(GoalLifecycleState::IDLE);
  core->return_to_start_completed = true;

  int frontier_search_calls = 0;
  core->callbacks.frontier_search = [&frontier_search_calls](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      frontier_search_calls += 1;
      FrontierSearchResult result;
      return result;
    };

  core->try_send_next_goal();

  EXPECT_EQ(frontier_search_calls, 0);
}

TEST(PreemptionFlowTests, GoalSucceededRedispatchesAfterCooldownWithoutNewMap)
{
  FrontierExplorerCoreParams params;
  params.post_goal_settle_enabled = true;
  params.post_goal_min_settle = 0.25;
  params.return_to_start_on_complete = false;

  int64_t now_ns = 1'000'000'000;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {
      return now_ns;
    };
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.wait_for_action_server = [](double) {return true;};

  int dispatch_calls = 0;
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 10}};
      result.robot_map_cell = {1, 1};
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.map_generation = 1;
  core.costmap_generation = 1;
  core.local_costmap_generation = 0;

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_EQ(core.current_dispatch_id, 1);

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_response_callback(core.current_dispatch_id, fake_handle, true, "");
  core.get_result_callback(
    core.current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
    0,
    "");

  EXPECT_TRUE(core.awaiting_map_refresh);
  EXPECT_TRUE(core.post_goal_settle_active);

  core.costmapCallback(OccupancyGrid2d(costmap_msg));
  EXPECT_EQ(dispatch_calls, 1);

  core.decision_map_dirty = false;
  now_ns += 100'000'000;
  core.processPendingMapUpdate();
  EXPECT_EQ(dispatch_calls, 1);

  now_ns += 200'000'000;
  core.processPendingMapUpdate();
  EXPECT_EQ(dispatch_calls, 2);
}

TEST(PreemptionFlowTests, GoalSucceededRedispatchesImmediatelyWhenPostGoalSettleDisabled)
{
  FrontierExplorerCoreParams params;
  params.post_goal_settle_enabled = false;
  params.post_goal_min_settle = 99.0;
  params.return_to_start_on_complete = false;

  int64_t now_ns = 1'000'000'000;
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [&now_ns]() {
      now_ns += 100'000'000;
      return now_ns;
    };
  callbacks.get_current_pose = []() {
      return std::optional<geometry_msgs::msg::Pose>(make_pose(1.0, 1.0));
    };
  callbacks.log_info = [](const std::string &) {};
  callbacks.log_warn = [](const std::string &) {};
  callbacks.log_debug = [](const std::string &) {};
  callbacks.log_error = [](const std::string &) {};
  callbacks.wait_for_action_server = [](double) {return true;};

  int dispatch_calls = 0;
  callbacks.dispatch_goal_request = [&dispatch_calls](const GoalDispatchRequest &) {
      dispatch_calls += 1;
    };
  callbacks.frontier_search = [](
    const geometry_msgs::msg::Pose &,
    const OccupancyGrid2d &,
    const OccupancyGrid2d &,
    const std::optional<OccupancyGrid2d> &,
    double,
    bool)
    {
      FrontierSearchResult result;
      result.frontiers = {FrontierCandidate{{2.0, 2.0}, {2.0, 2.0}, 10}};
      result.robot_map_cell = {1, 1};
      return result;
    };

  FrontierExplorerCore core(params, callbacks);
  auto map_msg = build_grid(20, 20, 0);
  auto costmap_msg = build_grid(20, 20, 0);
  core.map = OccupancyGrid2d(map_msg);
  core.costmap = OccupancyGrid2d(costmap_msg);
  core.map_generation = 1;
  core.costmap_generation = 1;
  core.local_costmap_generation = 0;

  core.try_send_next_goal();
  ASSERT_EQ(dispatch_calls, 1);
  ASSERT_EQ(core.current_dispatch_id, 1);

  auto fake_handle = std::make_shared<FakeGoalHandle>();
  core.goal_response_callback(core.current_dispatch_id, fake_handle, true, "");
  core.get_result_callback(
    core.current_dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
    0,
    "");

  EXPECT_FALSE(core.awaiting_map_refresh);
  EXPECT_FALSE(core.post_goal_settle_active);
  EXPECT_EQ(dispatch_calls, 2);
}

}  // namespace

TEST(PreemptionFlowTests, ObservationPointNearRobotDoesNotCompleteUnderlyingFrontier)
{
  auto core = make_preemption_core();
  auto handle = std::make_shared<FakeGoalHandle>();
  core->goal_handle = handle;
  core->params.goal_preemption_enabled = false;
  core->params.goal_skip_on_blocked_goal = false;
  core->params.goal_preemption_complete_if_within_m = 0.50;
  core->active_goal_frontier = make_frontier(1.5, 0.0);
  core->active_goal_frontier->observation_goal = true;
  core->callbacks.get_current_pose = []() {
    return std::optional<geometry_msgs::msg::Pose>(make_pose(1.1, 0.0));
  };
  core->consider_preempt_active_goal("map");
  EXPECT_EQ(handle->cancel_calls, 0);
  EXPECT_FALSE(core->distance_completed_frontier.has_value());
}

TEST(PreemptionFlowTests, Relocated3DObservationBlockedLocallyNeverReachesAction)
{
  FrontierExplorerCore core(FrontierExplorerCoreParams{}, FrontierExplorerCoreCallbacks{});
  core.map = OccupancyGrid2d(build_grid(20,20,0));
  core.costmap = OccupancyGrid2d(build_grid(20,20,0));
  auto local = build_grid(20,20,0);
  set_cell(local,1,1,99);
  core.local_costmap = OccupancyGrid2d(local);
  int sends = 0;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&](const GoalDispatchRequest &) {++sends;};
  core.callbacks.validate_frontier_goal = [](const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::PoseStamped & input) {
      auto result=input;
      result.pose.position.x=1.5;result.pose.position.y=1.5;
      return std::optional<geometry_msgs::msg::PoseStamped>(result);
    };
  EXPECT_FALSE(core.send_frontier_goal({make_frontier(4,4)}, make_pose(), "test"));
  EXPECT_EQ(sends,0);
}

TEST(PreemptionFlowTests, Relocated3DObservationRetainsFrontierReferenceAndMarker)
{
  FrontierExplorerCore core(FrontierExplorerCoreParams{}, FrontierExplorerCoreCallbacks{});
  core.map = OccupancyGrid2d(build_grid(20,20,0));
  core.costmap = OccupancyGrid2d(build_grid(20,20,0));
  std::optional<GoalDispatchRequest> sent;
  core.callbacks.wait_for_action_server = [](double) {return true;};
  core.callbacks.dispatch_goal_request = [&](const GoalDispatchRequest & request) {sent=request;};
  core.callbacks.validate_frontier_goal = [](const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::PoseStamped & input) {
      auto result=input;
      result.pose.position.x=1.5;result.pose.position.y=1.5;
      return std::optional<geometry_msgs::msg::PoseStamped>(result);
    };
  ASSERT_TRUE(core.send_frontier_goal({make_frontier(4,4)}, make_pose(), "test"));
  ASSERT_TRUE(sent);
  ASSERT_TRUE(sent->frontier);
  EXPECT_TRUE(sent->frontier->observation_goal);
  EXPECT_DOUBLE_EQ(sent->frontier->center_point.first,4.0);
  EXPECT_DOUBLE_EQ(sent->goal_pose.pose.position.x,1.5);
}

}  // namespace frontier_exploration_ros2

namespace frontier_exploration_ros2
{
TEST(PreemptionFlowTests, Pending3DValidationDoesNotDispatchOrClaimBlockedTargets)
{
  FrontierExplorerCore core(FrontierExplorerCoreParams{},FrontierExplorerCoreCallbacks{});
  core.map=OccupancyGrid2d(build_grid(20,20,0));
  core.costmap=OccupancyGrid2d(build_grid(20,20,0));
  int sends=0;
  std::vector<std::string> logs;
  core.callbacks.dispatch_goal_request=[&](const GoalDispatchRequest &){++sends;};
  core.callbacks.validate_frontier_goal=[](const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::PoseStamped &)->std::optional<geometry_msgs::msg::PoseStamped>{return std::nullopt;};
  core.callbacks.frontier_goal_validation_pending=[](){return true;};
  core.callbacks.log_info=[&](const std::string & message){logs.push_back(message);};
  EXPECT_FALSE(core.send_frontier_goal({make_frontier(1,1)},make_pose(),"test"));
  EXPECT_EQ(sends,0);
  ASSERT_EQ(logs.size(),1U);
  EXPECT_NE(logs[0].find("pending"),std::string::npos);
  EXPECT_EQ(logs[0].find("blocked"),std::string::npos);
}
}


namespace frontier_exploration_ros2 {
TEST(PreemptionFlowTests, RepeatedSuccessfulObservationsWithoutRevealSelectOtherFrontier)
{
  FrontierExplorerCoreParams params;
  params.post_goal_settle_enabled=true; params.post_goal_min_settle=0;
  params.return_to_start_on_complete=false;
  int64_t now=1'000'000'000;
  FrontierSequence markers;
  FrontierExplorerCoreCallbacks cb;
  cb.now_ns=[&](){return now;};
  cb.get_current_pose=[](){return std::optional<geometry_msgs::msg::Pose>(make_pose(0,0));};
  cb.log_info=[](const std::string&){}; cb.log_warn=cb.log_info;
  cb.log_debug=cb.log_info; cb.log_error=cb.log_info;
  cb.wait_for_action_server=[](double){return false;};
  cb.dispatch_goal_request=[](const GoalDispatchRequest&){};
  cb.publish_frontier_markers=[&](const FrontierSequence & f){markers=f;};
  cb.frontier_search=[](const geometry_msgs::msg::Pose&,const OccupancyGrid2d&,
    const OccupancyGrid2d&,const std::optional<OccupancyGrid2d>&,double,bool) {
      FrontierSearchResult r; r.frontiers={make_frontier(2,2,10),make_frontier(7,7,10)};
      r.robot_map_cell={0,0};return r;
    };
  FrontierExplorerCore core(params,cb);
  auto m=build_grid(20,20,0);core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
  auto f=make_frontier(2,2,10); f.observation_goal=true;
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.pose=make_pose(1,1);
  for(int i=0;i<2;++i) {
    core.dispatch_goal_request("navigate_to_pose",pose,"frontier",f,{f},"test observation");
    core.goal_response_callback(core.current_dispatch_id,std::make_shared<FakeGoalHandle>(),true,"");
    core.get_result_callback(core.current_dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"");
    now+=1'000'000'000;
    core.try_send_next_goal();
  }
  ASSERT_EQ(markers.size(),1U);
  EXPECT_EQ(core.frontier_reference_point(markers.front()),std::make_pair(7.0,7.0));
  // An older pending sequence must not bypass the newly cooled reference,
  // even if the validator could relocate it to a different observation.
  int validations=0;
  core.callbacks.wait_for_action_server=[](double){return true;};
  core.callbacks.validate_frontier_goal=[&](const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::PoseStamped & input) {
      ++validations; auto result=input;result.pose=make_pose(6,6);
      return std::optional<geometry_msgs::msg::PoseStamped>(result);
    };
  EXPECT_FALSE(core.send_frontier_goal({f},make_pose(),"stale pending sequence"));
  EXPECT_EQ(validations,0);
  core.pending_frontier_sequence={f};
  EXPECT_FALSE(core.dispatch_pending_frontier_goal(make_pose()));
  EXPECT_TRUE(core.pending_frontier_sequence.empty());
  EXPECT_TRUE(core.send_frontier_goal({make_frontier(7,7)},make_pose(),"other region"));
  EXPECT_EQ(validations,1);
}
}

namespace frontier_exploration_ros2 {
TEST(PreemptionFlowTests, ProximityCompletionAlsoEvaluatesInformationGain)
{
  std::vector<std::string> logs;
  auto core=make_preemption_core(&logs);
  core->set_goal_state(GoalLifecycleState::IDLE);
  core->params.goal_preemption_enabled=false;
  core->params.goal_skip_on_blocked_goal=false;
  core->params.goal_preemption_complete_if_within_m=.25;
  core->params.post_goal_settle_enabled=true;
  core->params.post_goal_min_settle=0;
  core->params.return_to_start_on_complete=false;
  core->callbacks.get_current_pose=[](){return std::optional<geometry_msgs::msg::Pose>(make_pose(1.9,2));};
  core->callbacks.dispatch_goal_request=[](const GoalDispatchRequest&){};
  core->callbacks.wait_for_action_server=[](double){return false;};
  core->callbacks.on_exploration_complete=[](){};
  core->callbacks.frontier_search=[](const geometry_msgs::msg::Pose&,const OccupancyGrid2d&,
    const OccupancyGrid2d&,const std::optional<OccupancyGrid2d>&,double,bool) {
      FrontierSearchResult r;r.frontiers={make_frontier(7,7,10)};r.robot_map_cell={1,2};return r;
    };
  geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.pose=make_pose(2,2);
  auto frontier=make_frontier(2,2,10);  // a full goal, not a relocated observation
  core->dispatch_goal_request("navigate_to_pose",pose,"frontier",frontier,{frontier},"test full goal");
  auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(core->current_dispatch_id,handle,true,"");
  core->consider_preempt_active_goal("map");
  ASSERT_EQ(handle->cancel_calls,1);
  core->get_result_callback(core->current_dispatch_id,action_msgs::msg::GoalStatus::STATUS_CANCELED,0,"");
  core->try_send_next_goal();
  EXPECT_TRUE(std::any_of(logs.begin(),logs.end(),[](const auto & s){return s.find("completed by proximity")!=std::string::npos;}));
  EXPECT_TRUE(std::any_of(logs.begin(),logs.end(),[](const auto & s){return s.find("Observation information gain:")!=std::string::npos;}));
}
}

namespace frontier_exploration_ros2 {
TEST(PreemptionFlowTests, ObservationApproachWaitsForSameFrontierAndBoundsPendingWait)
{
  for (bool expire : {false,true}) {
    FrontierExplorerCoreParams params;
    params.post_goal_settle_enabled=true;params.post_goal_min_settle=0;
    params.return_to_start_on_complete=false;
    params.goal_skip_on_blocked_goal=false;
    int64_t now=1'000'000'000LL;
    bool pending=true,waiting=true;std::vector<GoalDispatchRequest> sent;
    FrontierExplorerCoreCallbacks cb;
    cb.now_ns=[&](){return now;};
    cb.get_current_pose=[](){return std::optional<geometry_msgs::msg::Pose>(make_pose(2,2));};
    cb.log_info=[](const std::string&){};cb.log_warn=cb.log_info;cb.log_debug=cb.log_info;cb.log_error=cb.log_info;
    cb.wait_for_action_server=[](double){return true;};
    cb.dispatch_goal_request=[&](const GoalDispatchRequest & request){sent.push_back(request);};
    cb.frontier_search=[](const geometry_msgs::msg::Pose&,const OccupancyGrid2d&,
      const OccupancyGrid2d&,const std::optional<OccupancyGrid2d>&,double,bool) {
        FrontierSearchResult result;result.frontiers={make_frontier(1,8,100),make_frontier(8,2,10)};
        result.robot_map_cell={2,2};return result;
      };
    cb.frontier_goal_validation_pending=[&](){return pending;};
    cb.validate_frontier_goal=[&](const geometry_msgs::msg::Pose&,
      const geometry_msgs::msg::PoseStamped& input)->std::optional<geometry_msgs::msg::PoseStamped>{
        if(input.pose.position.x>4 && waiting)return std::nullopt;
        auto output=input;output.pose=make_pose(3,2);return output;
      };
    FrontierExplorerCore core(params,cb);
    auto map=build_grid(20,20,0);core.map=OccupancyGrid2d(map);core.costmap=OccupancyGrid2d(map);
    auto frontier=make_frontier(8,2,10);frontier.observation_goal=true;
    geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="map";pose.pose=make_pose(2,2);
    core.dispatch_goal_request("navigate_to_pose",pose,"frontier",frontier,{frontier},"initial observation");
    core.goal_response_callback(core.current_dispatch_id,std::make_shared<FakeGoalHandle>(),true,"");
    core.get_result_callback(core.current_dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"");
    sent.clear();now+=1'000'000'000LL;
    core.try_send_next_goal();
    EXPECT_TRUE(sent.empty()) << "Other cached candidate must not steal the ongoing approach";
    pending=false;now+=1'000'000'000LL;
    core.try_send_next_goal();
    EXPECT_TRUE(sent.empty()) << "Retry backoff must not be mistaken for a completed validation";
    if(expire)now+=20'000'000'000LL;
    else waiting=false;
    core.try_send_next_goal();
    ASSERT_EQ(sent.size(),1U);
    ASSERT_TRUE(sent.front().frontier);
    EXPECT_EQ(core.frontier_reference_point(*sent.front().frontier),
      expire?std::make_pair(1.0,8.0):std::make_pair(8.0,2.0));
  }
}

namespace {
std::unique_ptr<FrontierExplorerCore> make_random_recovery_core()
{
  auto core=make_preemption_core();
  core->clear_active_goal_state();
  core->callbacks.get_current_pose=[](){return std::optional<geometry_msgs::msg::Pose>(make_pose(1.5,1.5));};
  core->callbacks.wait_for_action_server=[](double){return true;};
  core->callbacks.publish_frontier_markers=[](const auto &){};
  core->callbacks.publish_selected_frontier_pose=[](const auto &){};
  core->callbacks.publish_optimized_map=[](const auto &){};
  core->callbacks.debug_outputs_enabled=[](){return false;};
  core->callbacks.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool) {
    FrontierSearchResult result;result.frontiers={make_frontier(8.5,8.5,10)};
    result.robot_map_cell={1,1};return result;
  };
  core->callbacks.validate_frontier_goal=[](const auto &,const auto &)
    ->std::optional<geometry_msgs::msg::PoseStamped>{return std::nullopt;};
  core->callbacks.frontier_goal_validation_pending=[](){return false;};
  return core;
}
geometry_msgs::msg::PoseStamped recovery_pose(double x=3.5,double y=1.5)
{
  geometry_msgs::msg::PoseStamped p;p.header.frame_id="map";
  p.pose=make_pose(x,y);return p;
}
}
TEST(RandomRecoveryFlow, PendingValidationDoesNotSendAnUncheckedGoal)
{
  auto core=make_random_recovery_core();int sent=0;
  core->callbacks.request_random_recovery=[](const auto &,bool)
    ->std::optional<geometry_msgs::msg::PoseStamped>{return std::nullopt;};
  core->callbacks.random_recovery_pending=[](){return true;};
  core->callbacks.dispatch_goal_request=[&](const auto &){++sent;};
  core->try_send_next_goal();
  EXPECT_EQ(sent,0);EXPECT_FALSE(core->goal_in_progress);
}
TEST(RandomRecoveryFlow, UsesOwnedActionAndSettlesBeforeNormalExploration)
{
  auto core=make_random_recovery_core();std::vector<GoalDispatchRequest> sent;
  core->params.post_goal_settle_enabled=true;
  core->callbacks.request_random_recovery=[](const auto &,bool) {
    return std::optional<geometry_msgs::msg::PoseStamped>(recovery_pose());
  };
  core->callbacks.random_recovery_pending=[](){return false;};
  core->callbacks.dispatch_goal_request=[&](const auto & request){sent.push_back(request);};
  core->try_send_next_goal();
  ASSERT_EQ(sent.size(),1U);EXPECT_EQ(sent.front().goal_kind,"random_recovery");
  EXPECT_FALSE(sent.front().frontier);
  auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(sent.front().dispatch_id,handle,true,"");
  core->get_result_callback(sent.front().dispatch_id,
    action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
  EXPECT_FALSE(core->return_to_start_completed);
  EXPECT_FALSE(core->goal_in_progress);
  EXPECT_TRUE(core->awaiting_map_refresh);
}
TEST(RandomRecoveryFlow, LatestCostmapCanRejectAnOtherwiseVerifiedEndpoint)
{
  auto core=make_random_recovery_core();int sent=0;
  auto cost=build_grid(20,20,0);set_cell(cost,3,1,100);
  core->costmap=OccupancyGrid2d(cost);
  core->callbacks.request_random_recovery=[](const auto &,bool) {
    return std::optional<geometry_msgs::msg::PoseStamped>(recovery_pose());
  };
  core->callbacks.random_recovery_pending=[](){return false;};
  core->callbacks.dispatch_goal_request=[&](const auto &){++sent;};
  core->try_send_next_goal();EXPECT_EQ(sent,0);
}
TEST(RandomRecoveryFlow, ExistingActiveNavigationIsNeverPreemptedByRandomSampling)
{
  auto core=make_preemption_core();int requested=0;
  core->callbacks.request_random_recovery=[&](const auto &,bool)
    ->std::optional<geometry_msgs::msg::PoseStamped>{++requested;return std::nullopt;};
  core->try_send_next_goal();EXPECT_EQ(requested,0);
}
TEST(RandomRecoveryFlow, ExpiredRecoveryCancelsItsOwnGoalWithoutCompletingExploration)
{
  auto core=make_preemption_core();auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_handle=handle;core->active_goal_kind="random_recovery";
  core->callbacks.now_ns=[](){return int64_t{91'000'000'000};};
  core->processPendingMapUpdate();
  EXPECT_EQ(handle->cancel_calls,1);
  EXPECT_FALSE(core->return_to_start_completed);
}
TEST(RandomRecoveryFlow, FinalDispatchRejectsAnEndpointCloserThanHalfAMeter)
{
  auto core=make_random_recovery_core();int sent=0;
  core->callbacks.request_random_recovery=[](const auto &,bool) {
    return std::optional<geometry_msgs::msg::PoseStamped>(recovery_pose(1.7,1.5));
  };
  core->callbacks.random_recovery_pending=[](){return false;};
  core->callbacks.dispatch_goal_request=[&](const auto &){++sent;};
  core->try_send_next_goal();EXPECT_EQ(sent,0);
}
}

namespace frontier_exploration_ros2 {
namespace {
std::unique_ptr<FrontierExplorerCore> clearance_core(std::vector<GoalDispatchRequest> & sent) {
  auto core=make_random_recovery_core();
  core->callbacks.request_random_recovery=[](const auto &,bool)
    ->std::optional<geometry_msgs::msg::PoseStamped>{return std::nullopt;};
  core->callbacks.take_clearance_recovery_request=[ready=true]() mutable {return std::exchange(ready,false);};
  core->callbacks.wait_for_clearance_server=[](double){return true;};
  core->callbacks.dispatch_goal_request=[&sent](const auto & r){sent.push_back(r);};
  return core;
}
}
TEST(ClearanceHandoff, UsesSpecializedActionAndExclusivelyOwnsExecution) {
  std::vector<GoalDispatchRequest> sent;auto core=clearance_core(sent);
  core->try_send_next_goal();ASSERT_EQ(sent.size(),1U);
  EXPECT_EQ(sent[0].goal_kind,"clearance_recovery");EXPECT_EQ(sent[0].action_name,"escape_move");
  EXPECT_FALSE(sent[0].frontier);
  auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(sent[0].dispatch_id,handle,true,"");
  core->try_send_next_goal();core->processPendingMapUpdate();
  EXPECT_EQ(sent.size(),1U);EXPECT_EQ(handle->cancel_calls,0);
}
TEST(ClearanceHandoff, NoSpecializedServerNeverFallsBackToNavigateToPose) {
  std::vector<GoalDispatchRequest> sent;auto core=clearance_core(sent);
  core->callbacks.wait_for_clearance_server=[](double){return false;};
  core->try_send_next_goal();EXPECT_TRUE(sent.empty());EXPECT_FALSE(core->goal_in_progress);
}
TEST(ClearanceHandoff, SuccessWaitsForMapWithoutCompletingExploration) {
  std::vector<GoalDispatchRequest> sent;auto core=clearance_core(sent);
  core->params.post_goal_settle_enabled=true;core->try_send_next_goal();
  ASSERT_EQ(sent.size(),1U);
  core->goal_response_callback(sent[0].dispatch_id,std::make_shared<FakeGoalHandle>(),true,"");
  core->get_result_callback(sent[0].dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"");
  EXPECT_TRUE(core->awaiting_map_refresh);EXPECT_FALSE(core->goal_in_progress);
  EXPECT_FALSE(core->return_to_start_completed);
}
TEST(ClearanceHandoff, StopWhileAcceptanceIsPendingCancelsOnlyOwnedGoal) {
  std::vector<GoalDispatchRequest> sent;auto core=clearance_core(sent);
  core->try_send_next_goal();ASSERT_EQ(sent.size(),1U);
  core->stop_exploration_session("user stop");
  auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(sent[0].dispatch_id,handle,true,"");
  EXPECT_EQ(handle->cancel_calls,1);
  EXPECT_FALSE(core->exploration_enabled);
}
TEST(ClearanceHandoff, ExecutionTimeoutCancelsItsOwnHandle) {
  auto core=make_preemption_core();auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_handle=handle;core->active_goal_kind="clearance_recovery";
  core->callbacks.now_ns=[](){return int64_t{36'000'000'000};};
  core->processPendingMapUpdate();EXPECT_EQ(handle->cancel_calls,1);
  core->processPendingMapUpdate();EXPECT_EQ(handle->cancel_calls,1);
  EXPECT_FALSE(core->return_to_start_completed);
}
TEST(ClearanceHandoff, FailureReleasesExecutionWithoutClaimingCompletion) {
  std::vector<GoalDispatchRequest> sent;auto core=clearance_core(sent);
  core->try_send_next_goal();ASSERT_EQ(sent.size(),1U);
  core->goal_response_callback(sent[0].dispatch_id,std::make_shared<FakeGoalHandle>(),true,"");
  core->get_result_callback(sent[0].dispatch_id,action_msgs::msg::GoalStatus::STATUS_ABORTED,0,"unsafe");
  EXPECT_FALSE(core->goal_in_progress);EXPECT_FALSE(core->return_to_start_completed);
}
}

namespace frontier_exploration_ros2 {
TEST(CrossGoalStagnationFlow, ProgressTimeoutOutcomeNotifiesRecoveryBeforeReselection) {
  auto core=make_random_recovery_core();
  core->params.frontier_suppression_enabled=true;
  core->params.frontier_suppression_startup_grace_period_s=0;
  core->params.frontier_suppression_no_progress_timeout_s=2;
  int64_t now=1'000'000'000;int stalls=0;
  core->callbacks.now_ns=[&](){return now;};
  core->callbacks.on_navigation_stall=[&](const auto &){++stalls;};
  core->callbacks.dispatch_goal_request=[](const auto &){};
  auto f=make_frontier(3.5,1.5);
  core->dispatch_goal_request("navigate_to_pose",recovery_pose(),"frontier",f,{f},"test");
  auto id=core->current_dispatch_id;auto handle=std::make_shared<FakeGoalHandle>();
  core->goal_response_callback(id,handle,true,"");
  core->evaluate_active_goal_progress_timeout();now+=3'000'000'000;
  EXPECT_TRUE(core->evaluate_active_goal_progress_timeout());
  EXPECT_EQ(handle->cancel_calls,1);
  core->exploration_enabled=false;
  core->get_result_callback(id,action_msgs::msg::GoalStatus::STATUS_CANCELED,0,"");
  EXPECT_EQ(stalls,1);
  core->get_result_callback(id-1,action_msgs::msg::GoalStatus::STATUS_CANCELED,0,"");
  EXPECT_EQ(stalls,1);
}
}

namespace frontier_exploration_ros2 {
TEST(RecoveryLifecycle, RecordedInsetPoseDispatchesCompletesAndResumesExploration) {
 auto core=make_random_recovery_core();
 auto grid=build_grid(240,180,0);grid.info.resolution=.1;
 grid.info.origin.position.x=-12;grid.info.origin.position.y=-9;
 core->map=OccupancyGrid2d(grid);core->costmap=OccupancyGrid2d(grid);core->local_costmap.reset();
 core->params.exploration_boundary_polygon={{-9.65,-6.65},{9.65,-6.65},{9.65,6.65},{-9.65,6.65}};
 core->params.exploration_boundary_margin_m=.15;
 core->params.post_goal_settle_enabled=false;
 auto measured=make_pose(-4.3665,6.5384);
 core->callbacks.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(measured);};
 bool recovery_needed=true;
 core->callbacks.request_random_recovery=[&](const auto &,bool)
   ->std::optional<geometry_msgs::msg::PoseStamped>{
   if(!recovery_needed)return std::nullopt;
   return recovery_pose(-5.1,6.3);
 };
 core->callbacks.random_recovery_pending=[](){return false;};
 std::vector<GoalDispatchRequest> sent;
 core->callbacks.dispatch_goal_request=[&](const auto & req){sent.push_back(req);};
 core->try_send_next_goal();
 ASSERT_EQ(sent.size(),1U);EXPECT_EQ(sent[0].goal_kind,"random_recovery");
 core->goal_response_callback(sent[0].dispatch_id,std::make_shared<FakeGoalHandle>(),true,"");
 // Map updates while recovering cannot replace the owned action.
 core->try_send_next_goal();EXPECT_EQ(sent.size(),1U);
 measured=make_pose(-5.1,6.3);recovery_needed=false;
 core->get_result_callback(sent[0].dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
 EXPECT_FALSE(core->return_to_start_completed);EXPECT_FALSE(core->goal_in_progress);
 core->callbacks.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool) {
   FrontierSearchResult result;result.frontiers={make_frontier(-3.,5.,10)};
   result.robot_map_cell={69,153};return result;
 };
 core->callbacks.validate_frontier_goal=[](const auto &,const auto &)
   ->std::optional<geometry_msgs::msg::PoseStamped>{return recovery_pose(-3.,5.);};
 core->frontier_snapshot.reset();core->decision_map_dirty=true;
 core->try_send_next_goal();
 ASSERT_EQ(sent.size(),2U);EXPECT_EQ(sent[1].goal_kind,"frontier");
 EXPECT_FALSE(core->return_to_start_completed);
}
}

namespace frontier_exploration_ros2 {
TEST(ClearanceHandoff, ContactRecoveryPrecedesFrontierSelectionWithoutIdleTimeout) {
 std::vector<GoalDispatchRequest> sent;auto core=clearance_core(sent);int searched=0;
 core->callbacks.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){
   ++searched;return FrontierSearchResult{};
 };
 core->try_send_next_goal();
 ASSERT_EQ(sent.size(),1U);EXPECT_EQ(sent[0].goal_kind,"clearance_recovery");
 EXPECT_EQ(searched,0);
}
}
