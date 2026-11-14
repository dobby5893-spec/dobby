#include <gtest/gtest.h>
#include "frontier_exploration_ros2/viewpoint_sampling.hpp"
#include "frontier_exploration_ros2/frontier_search.hpp"
#include "frontier_exploration_ros2/frontier_evaluator.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
using namespace frontier_exploration_ros2;

static nav_msgs::msg::OccupancyGrid fanMap() {
  nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";
  m.info.width=100;m.info.height=100;m.info.resolution=.1;
  m.info.origin.orientation.w=1;m.data.assign(10000,0);return m;
}
static geometry_msgs::msg::Pose fanRobot(double x,double y) {
  geometry_msgs::msg::Pose p;p.position.x=x;p.position.y=y;p.orientation.w=1;return p;
}
TEST(ViewpointFan, BoundedTwoRoundsRemainOnFreeSide) {
  auto first=sampleViewpointFan({4,5},{-1,0},{.35,.5,.8},15);
  auto second=sampleViewpointFan({4,5},{-1,0},{1.1,1.4},10);
  ASSERT_EQ(first.size(),15u);ASSERT_EQ(second.size(),10u);
  for(const auto & p:first)EXPECT_LT(p.first,4.);
  EXPECT_NEAR(first[0].first,3.65,1e-9);
  EXPECT_NEAR(std::hypot(second.back().first-4,second.back().second-5),1.4,1e-9);
  EXPECT_EQ(sampleViewpointFan({4,5},{-1,0},{.1,.2,.3,.4,.5,.6},15).size(),15u);
}
TEST(ViewpointFan, WallOccludesUnknownGain) {
  auto m=fanMap();
  for(int y=0;y<100;++y)for(int x=60;x<100;++x)m.data[y*100+x]=-1;
  for(int y=0;y<80;++y)m.data[y*100+40]=100;
  OccupancyGrid2d map(m);
  EXPECT_EQ(viewpointVisibleGain(map,{3.5,5.},{6.05,5.}),0);
  EXPECT_GT(viewpointVisibleGain(map,{4.5,5.},{6.05,5.}),0);
}
TEST(ViewpointFan, ReachableViewWithGainBeatsCloserOccludedPoint) {
  auto m=fanMap();auto cm=fanMap();
  for(int y=0;y<100;++y)for(int x=60;x<100;++x)m.data[y*100+x]=-1;
  for(int y=0;y<80;++y)m.data[y*100+40]=100;
  FrontierCandidate candidate{{6.05,5.05},{3.5,5.},20};
  candidate.viewpoint_candidates={{3.5,5.},{4.5,5.}};
  const auto result=FrontierEvaluator({}).filter_reachable(
    fanRobot(3.5,8.5),OccupancyGrid2d(m),OccupancyGrid2d(cm),{candidate});
  ASSERT_EQ(result.size(),1u);
  EXPECT_EQ(result.front().goal_point,std::make_optional(Viewpoint{4.5,5.}));
  EXPECT_EQ(result.front().viewpoint_candidates.size(),2u);
}
TEST(ViewpointFan, ExpandedRoundUsedWhenPrimaryDisconnected) {
  auto m=fanMap();auto cm=fanMap();
  for(int y=0;y<100;++y)m.data[y*100+40]=100;
  FrontierCandidate candidate{{6,5},{5,5},20};
  candidate.viewpoint_candidates={{5,5}};
  candidate.fallback_viewpoint_candidates={{2,5}};
  const auto result=FrontierEvaluator({}).filter_reachable(
    fanRobot(1,5),OccupancyGrid2d(m),OccupancyGrid2d(cm),{candidate});
  ASSERT_EQ(result.size(),1u);
  EXPECT_TRUE(result.front().viewpoint_candidates.empty());
  EXPECT_EQ(result.front().goal_point,std::make_optional(Viewpoint{2,5}));
}
TEST(ViewpointFan, PrimaryRoundKeepsPriorityOverExpandedShortcut) {
  auto m=fanMap();
  FrontierCandidate candidate{{6,5},{4,5},20};
  candidate.viewpoint_candidates={{4,5}};
  candidate.fallback_viewpoint_candidates={{2,5}};
  const auto result=FrontierEvaluator({}).filter_reachable(
    fanRobot(1,5),OccupancyGrid2d(m),OccupancyGrid2d(m),{candidate});
  ASSERT_EQ(result.size(),1u);
  EXPECT_EQ(result.front().goal_point,std::make_optional(Viewpoint{4,5}));
}
TEST(ViewpointFan, SideApproachSurvivesBlockedStraightApproaches) {
  auto m=fanMap();auto cm=fanMap();
  for(int y=0;y<100;++y)for(int x=40;x<100;++x)m.data[y*100+x]=-1;
  for(int x=30;x<40;++x)m.data[50*100+x]=100;
  FrontierCache cache(100,100);std::vector<FrontierPoint *> frontier;
  for(int y=45;y<=55;++y)frontier.push_back(cache.getPoint(40,y));
  FrontierSearchOptions options;options.viewpoint_clearance_m=.10;
  auto candidate=build_frontier_candidate(frontier,{10,50},OccupancyGrid2d(m),
    OccupancyGrid2d(cm),std::nullopt,cache,fanRobot(1,5),.5,options);
  ASSERT_TRUE(candidate);
  ASSERT_FALSE(candidate->viewpoint_candidates.empty());
  EXPECT_LE(candidate->viewpoint_candidates.size(),15u);
  EXPECT_LE(candidate->fallback_viewpoint_candidates.size(),10u);
  for(const auto & p:candidate->viewpoint_candidates)
    EXPECT_GT(std::abs(p.second-5.05),.10);
}

TEST(ViewpointFan, RetainsBoundaryButNeverDispatchesUncheckedNeighbor) {
  auto m=fanMap();
  for(int y=0;y<100;++y)for(int x=40;x<100;++x)m.data[y*100+x]=-1;
  FrontierCache cache(100,100);std::vector<FrontierPoint *> frontier;
  for(int y=45;y<=55;++y)frontier.push_back(cache.getPoint(40,y));
  FrontierSearchOptions options;options.viewpoint_clearance_m=10.;
  auto candidate=build_frontier_candidate(frontier,{10,50},OccupancyGrid2d(m),
    OccupancyGrid2d(fanMap()),std::nullopt,cache,fanRobot(1,5),.5,options);
  ASSERT_TRUE(candidate);
  EXPECT_TRUE(candidate->viewpoint_search_exhausted);
  EXPECT_TRUE(FrontierEvaluator({}).filter_reachable(fanRobot(1,5),
    OccupancyGrid2d(m),OccupancyGrid2d(fanMap()),{*candidate}).empty());
}


TEST(ViewpointFan, DispatchChecksAlternativesWhenPreferredPointBecomesBlocked) {
  FrontierExplorerCore core(FrontierExplorerCoreParams{},FrontierExplorerCoreCallbacks{});
  auto m=fanMap();auto cm=fanMap();cm.data[50*100+30]=100;
  core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(cm);
  core.callbacks.wait_for_action_server=[](double){return true;};
  std::optional<GoalDispatchRequest> sent;
  core.callbacks.dispatch_goal_request=[&](const auto & request){sent=request;};
  core.callbacks.validate_frontier_goal=[](const auto &,const auto &)
    ->std::optional<geometry_msgs::msg::PoseStamped>{return std::nullopt;};
  core.callbacks.validate_frontier_goal_set=[&](const auto &,const auto & options)
    ->std::optional<geometry_msgs::msg::PoseStamped> {
    EXPECT_EQ(options.size(),2u);
    EXPECT_NEAR(options.front().pose.position.y,4.05,1e-6);
    EXPECT_NEAR(options.back().pose.position.x,2.05,1e-6);
    return options.front();
  };
  FrontierCandidate candidate{{4.05,5.05},{3.05,5.05},20};
  candidate.viewpoint_candidates={{3.05,5.05},{3.05,4.05}};
  candidate.fallback_viewpoint_candidates={{2.05,5.05}};
  ASSERT_TRUE(core.send_frontier_goal({candidate},fanRobot(1.05,5.05),"fan test"));
  ASSERT_TRUE(sent);EXPECT_NEAR(sent->goal_pose.pose.position.y,4.05,1e-6);
}

TEST(ViewpointFan, HiddenTerminalViewIsRejectedEvenWhenReachable) {
 auto m=fanMap();auto cm=fanMap();
 for(int y=0;y<100;++y)for(int x=60;x<100;++x)m.data[y*100+x]=-1;
 for(int y=0;y<80;++y)m.data[y*100+40]=100;
 FrontierCandidate candidate{{6.05,5.05},{3.5,5.},20};
 candidate.requires_visible_viewpoint=true;candidate.viewpoint_candidates={{3.5,5.}};
 EXPECT_TRUE(FrontierEvaluator({}).filter_reachable(
   fanRobot(3.5,8.5),OccupancyGrid2d(m),OccupancyGrid2d(cm),{candidate}).empty());
}
TEST(ViewpointFan, OneCellResidualBetweenScanRaysIsStillVisible) {
 auto m=fanMap();m.data[55*100+75]=-1;
 EXPECT_EQ(viewpointVisibleGain(OccupancyGrid2d(m),{4.05,4.05},{7.55,5.55}),1);
}

TEST(ViewpointFan, VisibleViewpointRequiresArrivalInsteadOfHalfMeterCompletion) {
 FrontierExplorerCore core(FrontierExplorerCoreParams{},FrontierExplorerCoreCallbacks{});
 core.map=OccupancyGrid2d(fanMap());core.costmap=OccupancyGrid2d(fanMap());
 core.callbacks.wait_for_action_server=[](double){return true;};
 std::optional<GoalDispatchRequest> sent;
 core.callbacks.dispatch_goal_request=[&](const auto & request){sent=request;};
 core.callbacks.validate_frontier_goal=[](const auto &,const auto & goal)
   ->std::optional<geometry_msgs::msg::PoseStamped>{return goal;};
 FrontierCandidate candidate{{4.5,5},{3.5,5},20};candidate.requires_visible_viewpoint=true;
 ASSERT_TRUE(core.send_frontier_goal({candidate},fanRobot(1,5),"visible arrival"));
 ASSERT_TRUE(sent);ASSERT_TRUE(sent->frontier);
 EXPECT_TRUE(sent->frontier->observation_goal);
}

TEST(ViewpointFan, ExpansionAddsOppositeAnglesAndLongerViewsWithinSensorRange) {
 auto first=sampleExpandedViewpoints({0,0},{1,0},1);
 auto second=sampleExpandedViewpoints({0,0},{1,0},2);
 auto third=sampleExpandedViewpoints({0,0},{1,0},3);
 EXPECT_GT(first.size(),0u);EXPECT_GT(second.size(),first.size());EXPECT_GT(third.size(),second.size());
 bool opposite=false,far=false;
 for(const auto & p:third) {
   EXPECT_LE(std::hypot(p.first,p.second),3.200001);
   opposite|=p.first<-.3;far|=std::hypot(p.first,p.second)>3.;
 }
 EXPECT_TRUE(opposite);EXPECT_TRUE(far);
 EXPECT_TRUE(sampleExpandedViewpoints({0,0},{1,0},0).empty());
}

TEST(ViewpointFan, ExpandedAlternativesAreNotTruncatedBefore3DValidation) {
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 FrontierExplorerCore core(params,FrontierExplorerCoreCallbacks{});
 auto m=fanMap();m.data[90*100+90]=-1;
 core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(fanMap());
 core.callbacks.get_current_pose=[](){return std::optional<geometry_msgs::msg::Pose>(fanRobot(1.05,5.05));};
 core.callbacks.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){return FrontierSearchResult{};};
 core.try_send_next_goal();
 ASSERT_GT(core.frontier_search_options().viewpoint_expansion_level,0u);
 core.callbacks.wait_for_action_server=[](double){return true;};
 core.callbacks.dispatch_goal_request=[](const auto &){};
 size_t checked=0;
 core.callbacks.validate_frontier_goal_set=[&](const auto &,const auto & options)
   ->std::optional<geometry_msgs::msg::PoseStamped>{checked=options.size();return options.back();};
 FrontierCandidate candidate{{9.05,9.05},{2.05,5.05},20};
 for(int i=0;i<40;++i)candidate.fallback_viewpoint_candidates.push_back({2.05+i*.1,5.05});
 EXPECT_TRUE(core.send_frontier_goal({candidate},fanRobot(1.05,5.05),"expanded validator"));
 EXPECT_EQ(checked,40u);
}
