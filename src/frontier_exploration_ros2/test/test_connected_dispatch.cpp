#include <gtest/gtest.h>
#include <action_msgs/msg/goal_status.hpp>
#include "frontier_exploration_ros2/connected_dispatch.hpp"
using namespace frontier_exploration_ros2;

static nav_msgs::msg::OccupancyGrid room()
{
  nav_msgs::msg::OccupancyGrid m;
  m.header.frame_id = "map";
  m.info.resolution = 0.1;
  m.info.width = 60;
  m.info.height = 40;
  m.info.origin.orientation.w = 1.0;
  m.data.assign(2400, 0);
  return m;
}
static geometry_msgs::msg::Pose robot()
{
  geometry_msgs::msg::Pose p;
  p.position.x = 1.05;
  p.position.y = 2.05;
  p.orientation.w = 1.0;
  return p;
}
TEST(ConnectedDispatch, OpenRoom)
{
  OccupancyGrid2d g(room());
  ConnectedDispatch c(g, g, std::nullopt, robot(), "map", 0.35, 65);
  EXPECT_TRUE(c.failure.empty());
  EXPECT_TRUE(c.choose({4.05, 2.05}, 0.2, 0.8));
}
TEST(ConnectedDispatch, SolidWallAndUnknownStripDisconnect)
{
  for (int value : {100, -1}) {
    auto m = room();
    for (int y = 0; y < 40; ++y) {m.data[y * 60 + 30] = value;}
    OccupancyGrid2d g(m);
    ConnectedDispatch c(g, g, std::nullopt, robot(), "map", 0.35, 65);
    EXPECT_FALSE(c.choose({4.05, 2.05}, 0.2, 0.0));
  }
}
TEST(ConnectedDispatch, DoorMustFitClearance)
{
  for (int half : {2, 7}) {
    auto m = room();
    for (int y = 0; y < 40; ++y) {
      if (y < 20-half || y > 20+half) {m.data[y * 60 + 30] = 100;}
    }
    OccupancyGrid2d g(m);
    ConnectedDispatch c(g, g, std::nullopt, robot(), "map", 0.35, 65);
    EXPECT_EQ(c.choose({4.05, 2.05}, 0.2, 0.0).has_value(), half == 7);
  }
}
TEST(ConnectedDispatch, BlockedStartDoesNotSnapThroughWall)
{
  auto m = room();
  m.data[20 * 60 + 10] = 100;
  OccupancyGrid2d g(m);
  ConnectedDispatch c(g, g, std::nullopt, robot(), "map", 0.35, 65);
  EXPECT_FALSE(c.failure.empty());
  EXPECT_FALSE(c.choose({4.05, 2.05}, 1.5, 0.0));
}
TEST(ConnectedDispatch, RejectFrameMismatch)
{
  auto m = room();
  OccupancyGrid2d g(m);
  m.header.frame_id = "odom";
  ConnectedDispatch c(g, g, OccupancyGrid2d(m), robot(), "map", 0.35, 65);
  EXPECT_FALSE(c.failure.empty());
}
TEST(ConnectedDispatch, RelocatesAwayFromObstacle)
{
  auto m = room();
  m.data[20 * 60 + 40] = 100;
  OccupancyGrid2d g(m);
  ConnectedDispatch c(g, g, std::nullopt, robot(), "map", 0.35, 65);
  auto p = c.choose({4.05, 2.05}, 0.8, 0.0);
  ASSERT_TRUE(p);
  EXPECT_GT(std::hypot(p->first - 4.05, p->second - 2.05), 0.35);
}

TEST(ConnectedDispatch, GeofenceExcludesOtherwiseReachableTarget)
{
  OccupancyGrid2d g(room());
  const ExplorationBoundaryPolygon boundary{{0.0, 0.0}, {3.5, 0.0}, {3.5, 4.0}, {0.0, 4.0}};
  ConnectedDispatch c(g, g, std::nullopt, robot(), "map", 0.0, 65, boundary, 0.0);
  EXPECT_TRUE(c.failure.empty());
  EXPECT_FALSE(c.choose({4.05, 2.05}, 0.2, 0.0));
  EXPECT_TRUE(c.choose({3.05, 2.05}, 0.2, 0.0));
}

TEST(ExplorationBoundary, AppliesInwardMargin)
{
  const ExplorationBoundaryPolygon boundary{{0.0, 0.0}, {5.0, 0.0}, {5.0, 5.0}, {0.0, 5.0}};
  EXPECT_TRUE(is_inside_exploration_boundary(2.5, 2.5, boundary, 0.5));
  EXPECT_FALSE(is_inside_exploration_boundary(0.25, 2.5, boundary, 0.5));
  EXPECT_FALSE(is_inside_exploration_boundary(5.25, 2.5, boundary, 0.0));
  EXPECT_TRUE(is_inside_exploration_boundary(100.0, 100.0, {}, 10.0));
}


TEST(ConnectedDispatch, RecordedNorthEdgePoseCanReturnInsideMargin) {
 auto m=room();m.info.width=240;m.info.height=180;
 m.info.origin.position.x=-12;m.info.origin.position.y=-9;m.data.assign(240*180,0);
 OccupancyGrid2d g(m);auto pose=robot();pose.position.x=-4.3665;pose.position.y=6.5384;
 const ExplorationBoundaryPolygon boundary{{-9.65,-6.65},{9.65,-6.65},{9.65,6.65},{-9.65,6.65}};
 ConnectedDispatch c(g,g,std::nullopt,pose,"map",0.,90,boundary,.15);
 EXPECT_TRUE(c.failure.empty())<<c.failure;
 EXPECT_TRUE(c.choose({-5.1,6.3},1e-6,.5));
 EXPECT_FALSE(c.choose({-5.1,6.6},1e-6,.5));
}


TEST(ConnectedDispatch, InsetReturnCannotUseUnknownOrOccupiedStart) {
 for(int value:{-1,100}) {
  auto m=room();auto pose=robot();pose.position.x=.15;pose.position.y=2.05;
  m.data[20*60+1]=value;OccupancyGrid2d g(m);
  const ExplorationBoundaryPolygon b{{0,0},{6,0},{6,4},{0,4}};
  ConnectedDispatch c(g,g,std::nullopt,pose,"map",0.,90,b,.4);
  EXPECT_FALSE(c.failure.empty());EXPECT_FALSE(c.choose({1.05,2.05},.01,.5));
 }
}
TEST(ConnectedDispatch, InsetReturnCannotCrossAClosedWall) {
 auto m=room();auto pose=robot();pose.position.x=.15;
 for(int y=0;y<40;++y)m.data[y*60+3]=100;
 OccupancyGrid2d g(m);const ExplorationBoundaryPolygon b{{0,0},{6,0},{6,4},{0,4}};
 ConnectedDispatch c(g,g,std::nullopt,pose,"map",0.,90,b,.4);
 EXPECT_TRUE(c.failure.empty());EXPECT_FALSE(c.choose({1.05,2.05},.01,.5));
}
TEST(ConnectedDispatch, OutsideOuterBoundaryIsDistinctFromObstacleFailure) {
 auto m=room();OccupancyGrid2d g(m);auto pose=robot();pose.position.x=.15;
 const ExplorationBoundaryPolygon b{{.2,0},{6,0},{6,4},{.2,4}};
 ConnectedDispatch c(g,g,std::nullopt,pose,"map",0.,90,b,.1);
 EXPECT_EQ(c.failure,"robot outside outer exploration boundary");
}
TEST(ExplorationBoundary, TransitUsesInsetBandButNeverLeavesOuterPolygon) {
 const ExplorationBoundaryPolygon b{{0,0},{5,0},{5,5},{0,5}};
 EXPECT_TRUE(boundary_return_step({.1,2.},{.5,2.},b,.4));
 EXPECT_TRUE(boundary_return_step({.1,2.},{.05,2.},b,.4));
 EXPECT_TRUE(boundary_return_step({.5,2.},{.1,2.},b,.4));
 EXPECT_FALSE(boundary_return_step({.1,2.},{-.05,2.},b,.4));
 const ExplorationBoundaryPolygon concave{{0,0},{3,0},{3,3},{2,3},{2,1},{1,1},{1,3},{0,3}};
 EXPECT_FALSE(boundary_return_step({.5,2.},{2.5,2.},concave,0.));
}

TEST(BoundaryReentry, OutsideDistanceCannotGrowOrLeaveAfterEntering) {
 ExplorationBoundaryPolygon b{{0,0},{5,0},{5,3},{0,3}};
 EXPECT_NEAR(boundary_outside_distance({2,3.42},b),.42,1e-6);
 EXPECT_TRUE(boundary_reentry_step({2,3.42},{2,2.5},b));
 EXPECT_FALSE(boundary_reentry_step({2,3.42},{2,3.5},b));
 EXPECT_FALSE(boundary_reentry_step({2,2.5},{2,3.1},b));
 EXPECT_FALSE(boundary_reentry_step({2,3.8},{2,2.5},b));
}
TEST(ConnectedDispatch, OutsideStartCanOnlyReturnWithExplicitReentryMode) {
 auto m=room();OccupancyGrid2d g(m);auto p=robot();p.position.y=3.42;
 ExplorationBoundaryPolygon b{{0,0},{5,0},{5,3},{0,3}};
 ConnectedDispatch ordinary(g,g,std::nullopt,p,"map",0,90,b,.15);
 EXPECT_FALSE(ordinary.failure.empty());
 ConnectedDispatch recovery(g,g,std::nullopt,p,"map",0,90,b,.15,true);
 ASSERT_TRUE(recovery.failure.empty())<<recovery.failure;
 EXPECT_TRUE(recovery.choose({1.05,2.45},.01,.15));
 EXPECT_FALSE(recovery.choose({1.05,3.55},.01,.15));
}
TEST(ConnectedDispatch, ReentryCannotCrossObstacleUnknownOrLargeExcursion) {
 ExplorationBoundaryPolygon b{{0,0},{5,0},{5,3},{0,3}};
 for(int cost:{100,-1}) {
  auto m=room();for(int x=0;x<60;++x)m.data[31*60+x]=cost;
  OccupancyGrid2d g(m);auto p=robot();p.position.y=3.42;
  ConnectedDispatch recovery(g,g,std::nullopt,p,"map",0,90,b,.15,true);
  EXPECT_FALSE(recovery.choose({1.05,2.45},.01,.15));
 }
 auto m=room();OccupancyGrid2d g(m);auto p=robot();p.position.y=3.8;
 ConnectedDispatch far(g,g,std::nullopt,p,"map",0,90,b,.15,true);
 EXPECT_FALSE(far.failure.empty());
}

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
TEST(BoundaryReentry, SchedulerRequestsRecoveryBeforeFrontierConnectivity) {
 FrontierExplorerCoreParams params;params.exploration_boundary_polygon={{0,0},{5,0},{5,3},{0,3}};
 geometry_msgs::msg::Pose actual=robot();actual.position.y=3.42;
 int requests=0,searches=0;
 FrontierExplorerCoreCallbacks cb;cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
 cb.request_random_recovery=[&](const auto &,bool blocked)->std::optional<geometry_msgs::msg::PoseStamped>{
  EXPECT_TRUE(blocked);++requests;return std::nullopt;};
 cb.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){
  ++searches;return FrontierSearchResult{};};
 FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(room());core.costmap=OccupancyGrid2d(room());
 core.try_send_next_goal();EXPECT_EQ(requests,1);EXPECT_EQ(searches,0);
}

TEST(BoundaryReentry, ReportedSuccessNeedsMeasuredInteriorPosition) {
 for(bool returned:{false,true}) {
  FrontierExplorerCoreParams params;params.exploration_boundary_polygon={{0,0},{5,0},{5,3},{0,3}};
  params.exploration_boundary_margin_m=.15;params.post_goal_settle_enabled=false;
  auto actual=robot();actual.position.y=3.42;
  std::vector<GoalDispatchRequest> sent;std::vector<std::string> messages;bool requested=false;
  FrontierExplorerCoreCallbacks cb;
  cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
  cb.wait_for_action_server=[](double){return true;};
  cb.request_random_recovery=[&](const auto &,bool)->std::optional<geometry_msgs::msg::PoseStamped>{
   if(requested)return std::nullopt;
   requested=true;
   geometry_msgs::msg::PoseStamped g;g.header.frame_id="map";g.pose=actual;g.pose.position.y=2.45;return g;};
  cb.dispatch_goal_request=[&](const auto & request){sent.push_back(request);};
  cb.log_warn=[&](const std::string & m){messages.push_back(m);};cb.log_info=cb.log_warn;
  FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(room());core.costmap=OccupancyGrid2d(room());
  core.try_send_next_goal();ASSERT_EQ(sent.size(),1U);
  if(returned)actual.position.y=2.45;
  core.get_result_callback(sent.front().dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
  bool accepted=false,rejected=false;
  for(auto m:messages) {
   accepted|=m.find("Random recovery point reached")!=std::string::npos;
   rejected|=m.find("Recovery success rejected")!=std::string::npos;
  }
  EXPECT_EQ(accepted,returned);EXPECT_EQ(rejected,!returned);
 }
}

TEST(NoNextGoalWatchdog, ExactlyFiveSimulatedMinutesTriggersNotWallTimeOrMapUpdates) {
 NoNextGoalWatchdog w;w.observe(0,0);
 for(int t=1;t<300;++t){w.observe(t,0);EXPECT_FALSE(w.due(t));}
 w.observe(300,0);EXPECT_TRUE(w.due(300));EXPECT_DOUBLE_EQ(w.seconds(),300);
 w.attempted(300);EXPECT_FALSE(w.due(329));w.observe(330,0);EXPECT_TRUE(w.due(330));
}
TEST(NoNextGoalWatchdog, ActiveMovementAndMissingDataPauseWithoutForgivingFailure) {
 NoNextGoalWatchdog w;w.observe(0,0);w.observe(200,0);w.pause(200);
 w.pause(1000);w.observe(1000,0);EXPECT_DOUBLE_EQ(w.seconds(),200);
 w.observe(1099,0);EXPECT_FALSE(w.due(1099));w.observe(1100,0);EXPECT_TRUE(w.due(1100));
 w.position(1100,0,0);w.position(1100,.25,0);w.accepted(1100);w.pause(2000);w.observe(2000,0);w.observe(2299,0);
 EXPECT_FALSE(w.due(2299));w.observe(2300,0);EXPECT_TRUE(w.due(2300));
}
TEST(NoNextGoalWatchdog, FloorClockAndSessionResetDiscardOldDeadline) {
 NoNextGoalWatchdog w;w.observe(0,0);w.observe(299,0);w.observe(300,3);
 EXPECT_FALSE(w.due(300));EXPECT_DOUBLE_EQ(w.seconds(),0);
 w.observe(599,3);w.observe(0,3);EXPECT_DOUBLE_EQ(w.seconds(),0);
 w.observe(300,3);EXPECT_TRUE(w.due(300));w.clear();EXPECT_FALSE(w.due(1000));
}
TEST(NoNextGoalWatchdog, SchedulerTriggersEvenWhenEveryCandidateRemainsUnusable) {
 int64_t now=0;int fired=0,forced_calls=0;bool forced=false;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 params.no_next_goal_timeout_s=300;params.post_goal_settle_enabled=false;
 params.reachability_filter_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[](){return std::optional<geometry_msgs::msg::Pose>(robot());};
 cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){return FrontierSearchResult{};};
 cb.on_no_next_goal_timeout=[&](){++fired;forced=true;};
 cb.request_random_recovery=[&](const auto &,bool)->std::optional<geometry_msgs::msg::PoseStamped>{
  if(forced){++forced_calls;forced=false;}return std::nullopt;};
 FrontierExplorerCore core(params,cb);
 auto m=room();m.data[30*60+30]=-1;
 for(int t:{0,100,200,299}) {
  now=int64_t(t)*1000000000LL;m.header.stamp.sec=t;
  core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);core.try_send_next_goal();
 }
 EXPECT_EQ(fired,0);EXPECT_EQ(forced_calls,0);
 now=300000000000LL;m.header.stamp.sec=300;core.map=OccupancyGrid2d(m);core.try_send_next_goal();
 EXPECT_EQ(fired,1);EXPECT_EQ(forced_calls,1);
 // Continued fresh maps and no candidate must not reset the idle duration.
 now=330000000000LL;m.header.stamp.sec=330;core.map=OccupancyGrid2d(m);core.try_send_next_goal();
 EXPECT_EQ(fired,2);EXPECT_EQ(forced_calls,2);
}

#include "frontier_exploration_ros2/residual_region_audit.hpp"
static ResidualAudit auditFixture(nav_msgs::msg::OccupancyGrid m,
 const std::function<int(std::pair<double,double>,std::pair<double,double>)> & view) {
 OccupancyGrid2d grid(m);
 return auditResidualRegions(grid,{1.05,2.05},{},0.,[](int,int){return false;},view,
   std::chrono::steady_clock::now()+std::chrono::seconds(3));
}
TEST(ResidualRegions, OneUnvisitedRegionBlocksHandoffEvenWhenAnotherIsOccluded) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;
 auto result=auditFixture(m,[](auto,auto target){return target.second>2?1:0;});
 EXPECT_TRUE(result.complete);EXPECT_EQ(result.regions.size(),2U);
 EXPECT_GT(result.pending,0);EXPECT_GT(result.deferred,0);EXPECT_FALSE(result.handoff());
}
TEST(ResidualRegions, EachOccludedRegionIsListedAndRawUnknownsArePreserved) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;auto original=m.data;
 auto result=auditFixture(m,[](auto,auto){return 0;});
 ASSERT_TRUE(result.handoff());EXPECT_EQ(result.regions.size(),2U);
 EXPECT_NEAR(result.remaining,.02,1e-6);EXPECT_NEAR(result.deferred,.02,1e-6);
 EXPECT_NE(result.regions[0].id,result.regions[1].id);EXPECT_EQ(m.data,original);
}
TEST(ResidualRegions, TimeoutAndUncertainGeometryNeverCertifyCompletion) {
 auto m=room();m.data[10*60+30]=-1;
 auto result=auditFixture(m,[](auto,auto){return -1;});
 EXPECT_FALSE(result.complete);EXPECT_FALSE(result.handoff());
 OccupancyGrid2d grid(m);
 result=auditResidualRegions(grid,{1.05,2.05},{},0.,[](int,int){return false;},
   [](auto,auto){return 0;},std::chrono::steady_clock::now());
 EXPECT_FALSE(result.complete);EXPECT_FALSE(result.handoff());
}
TEST(ResidualRegions, UnsafeOrUnknownRobotSeedCannotCertifyAnything) {
 auto m=room();m.data[20*60+10]=-1;
 auto result=auditFixture(m,[](auto,auto){return 0;});
 EXPECT_FALSE(result.complete);EXPECT_FALSE(result.handoff());
}
TEST(ResidualRegions, FreshIndependentStableMapsRequiredAndOpeningRevokes) {
 auto m=room();m.data[10*60+30]=-1;
 auto result=auditFixture(m,[](auto,auto){return 0;});ASSERT_TRUE(result.handoff());
 ResidualAuditStability stable;
 EXPECT_FALSE(stable.observe(result,0,1));EXPECT_FALSE(stable.observe(result,20,1));
 EXPECT_FALSE(stable.observe(result,21,2));EXPECT_TRUE(stable.observe(result,22,3));
 auto opened=auditFixture(m,[](auto,auto){return 1;});
 EXPECT_FALSE(stable.observe(opened,23,4));
 EXPECT_FALSE(stable.observe(result,24,5));
}
TEST(ResidualRegions, TwoDimensionalClosedAreaMustBeIndependentlyAudited) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;OccupancyGrid2d grid(m);
 auto result=auditResidualRegions(grid,{1.05,2.05},{},0.,
  [](int x,int y){return x==30 && y==10;},[](auto,auto){return 0;},
  std::chrono::steady_clock::now()+std::chrono::seconds(3));
 EXPECT_EQ(result.closed,0);EXPECT_NEAR(result.deferred,.02,1e-6);
 EXPECT_EQ(result.regions.size(),2U);EXPECT_GT(result.sites,0U);
 EXPECT_TRUE(result.handoff());
}
TEST(ResidualRegions, TwoDimensionalDisconnectionDoesNotProveThreeDimensionalOcclusion) {
 auto m=room();
 for(int y=0;y<40;++y)m.data[y*60+25]=100;
 m.data[20*60+40]=-1;
 auto result=auditFixture(m,[](auto,auto){return 1;});
 EXPECT_FALSE(result.handoff());ASSERT_EQ(result.regions.size(),1U);
 EXPECT_GT(result.sites,0U);EXPECT_EQ(result.regions[0].state,"actionable");
}
TEST(ResidualRegions, ThreeDimensionalWallOcclusionIsNotPlannerFailure) {
 octomap::OcTree tree(.1);VisibilitySensor sensor;sensor.z=.35;
 EXPECT_TRUE(auditCellVisible(tree,{0,0,0,0},{2.05,0.05},0,sensor));
 for(int y=-10;y<=10;++y)for(int z=0;z<=15;++z)
   tree.updateNode(octomap::point3d(1.05,y*.1,z*.1),true);
 tree.updateInnerOccupancy();
 EXPECT_FALSE(auditCellVisible(tree,{0,0,0,0},{2.05,0.05},0,sensor));
}

TEST(NoNextGoalWatchdog, AcceptedButStationaryFailedGoalsDoNotResetDeadline) {
 NoNextGoalWatchdog w;w.observe(0,0);w.position(0,1,1);
 for(int t:{100,200,299}) {
  w.observe(t,0);w.position(t,1.01,1.01);w.accepted(t);
  w.observe(t+1,0);
 }
 w.observe(303,0);w.position(303,1,1);
 EXPECT_TRUE(w.due(303));
}
TEST(NoNextGoalWatchdog, ActualMovementResetsFailedGoalHistory) {
 NoNextGoalWatchdog w;w.observe(0,0);w.position(0,1,1);w.observe(299,0);
 w.position(299,1.25,1);EXPECT_DOUBLE_EQ(w.seconds(),0);
 w.observe(598,0);EXPECT_FALSE(w.due(598));w.observe(599,0);EXPECT_TRUE(w.due(599));
}

TEST(ResidualRegions, StairCellsDoNotHideAnAttachedUnexploredRoom) {
 auto m=room();m.data[20*60+20]=-1;m.data[20*60+21]=-1;
 OccupancyGrid2d grid(m);
 auto result=auditResidualRegions(grid,{1.05,2.05},{},0.,[](int,int){return false;},
  [](auto,auto){return 1;},std::chrono::steady_clock::now()+std::chrono::seconds(2),
  [](int x,int y){return x==20 && y==20;});
 EXPECT_GT(result.transition,0);EXPECT_GT(result.pending,0);EXPECT_FALSE(result.handoff());
}
TEST(ResidualRegions, OnlyConfirmedStairCellsCanBeHandedToTransitionTask) {
 auto m=room();m.data[20*60+20]=-1;OccupancyGrid2d grid(m);
 auto result=auditResidualRegions(grid,{1.05,2.05},{},0.,[](int,int){return false;},
  [](auto,auto){return -1;},std::chrono::steady_clock::now()+std::chrono::seconds(2),
  [](int x,int y){return x==20 && y==20;});
 EXPECT_TRUE(result.handoff());EXPECT_EQ(result.regions[0].state,"transition");
 EXPECT_NEAR(result.remaining,result.transition,1e-6);
}
TEST(ResidualRoutes, VisibleSafeEndpointOnAnotherIslandIsNotActionable) {
 using namespace go2_surface_3d_planner;
 octomap::OcTree tree(.1);
 for(double cx:{0.,1.2})for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)
  tree.updateNode(octomap::point3d(cx+(x+.5)*.1,(y+.5)*.1,-.05),true);
 tree.updateInnerOccupancy();
 SurfaceSearchConfig cfg;cfg.collision_radius=.04;cfg.cylinder_offset=0;
 cfg.snap_radius=0;cfg.yaw_bins=1;cfg.max_iterations=100000;
 auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
 VisibilitySensor sensor;
 EXPECT_EQ(auditAdmissibleView(tree,cfg,{1.25,.05},{1.6,.05},0,sensor,deadline),1);
 EXPECT_EQ(auditAdmissibleView(tree,cfg,{1.25,.05},{1.6,.05},0,sensor,deadline,
   SurfacePoint{.05,.05,0,0},nullptr,3.),-2);
 EXPECT_EQ(auditAdmissibleView(tree,cfg,{1.25,.05},{1.6,.05},0,sensor,
   std::chrono::steady_clock::now()-std::chrono::seconds(1),SurfacePoint{.05,.05,0,0}),-1);
}

TEST(ResidualRegions, LargeUnvisitedAreaIsInspectedBeforeTinyUncertainSliver) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;m.data[30*60+31]=-1;
 bool first=true;double first_y=0;
 auto result=auditFixture(m,[&](auto,auto target){
   if(first){first_y=target.second;first=false;}
   return target.second>2?1:-1;
 });
 EXPECT_GT(first_y,2);EXPECT_FALSE(result.complete);EXPECT_FALSE(result.handoff());
 ASSERT_TRUE(result.reference);EXPECT_GT(result.reference->second,2);
 EXPECT_NEAR(result.remaining,.03,1e-6);
}
TEST(ResidualRegions, UncertainRegionCannotPreventFindingAnotherReachableTarget) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;
 auto result=auditFixture(m,[](auto,auto target){return target.second>2?1:-1;});
 EXPECT_FALSE(result.complete);ASSERT_TRUE(result.reference);
 EXPECT_GT(result.reference->second,2);EXPECT_EQ(result.regions.size(),2U);
 EXPECT_FALSE(result.handoff());
}

TEST(ResidualRegions, SmallLaterTargetCannotReplaceLargeReachableRegion) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;m.data[30*60+31]=-1;
 auto result=auditFixture(m,[](auto,auto){return 1;});
 ASSERT_TRUE(result.reference);EXPECT_GT(result.reference->second,2);
}

TEST(ResidualRegions, RuntimeEvidenceCannotHideAnAdjoiningUnvisitedCell) {
 auto m=room();m.data[10*60+30]=-1;m.data[10*60+31]=-1;
 OccupancyGrid2d grid(m);
 auto result=auditResidualRegions(grid,{1.05,2.05},{},0,[](int,int){return false;},
  [](auto,auto){return 1;},std::chrono::steady_clock::now()+std::chrono::seconds(2),
  {},[](int x,int y){return x==30 && y==10?1:0;});
 EXPECT_NEAR(result.pending,.02,1e-6);EXPECT_EQ(result.deferred,0);
 EXPECT_FALSE(result.handoff());EXPECT_EQ(m.data[10*60+30],-1);
}
TEST(ResidualRegions, RuntimeDeferralCannotReplaceIndependentVerification) {
 auto m=room();m.data[10*60+30]=-1;m.data[30*60+30]=-1;
 OccupancyGrid2d grid(m);int calls=0;
 auto result=auditResidualRegions(grid,{1.05,2.05},{},0,[](int,int){return false;},
  [&](auto,auto){++calls;return -1;},std::chrono::steady_clock::now()+std::chrono::seconds(2),
  {},[](int,int y){return y<20?1:2;});
 EXPECT_GT(calls,0);EXPECT_FALSE(result.handoff());
 EXPECT_EQ(result.status(),CompletionStatus::INCOMPLETE);
 EXPECT_GT(result.pending,0);
}

TEST(ResidualRegions, TwoDimensionalExclusionCannotHideThreeDimensionalOpportunity) {
 auto m=room();m.data[10*60+30]=-1;OccupancyGrid2d grid(m);int calls=0;
 auto result=auditResidualRegions(grid,{1.05,2.05},{},0,[](int,int){return true;},
  [&](auto,auto){++calls;return 1;},std::chrono::steady_clock::now()+std::chrono::seconds(2));
 EXPECT_GT(calls,0);EXPECT_EQ(result.closed,0);EXPECT_GT(result.pending,0);
 EXPECT_FALSE(result.handoff());EXPECT_EQ(m.data[10*60+30],-1);
}
TEST(ResidualRegions, ZeroResidualRequiresStableFormalCompletion) {
 auto result=auditFixture(room(),[](auto,auto){return 1;});
 EXPECT_EQ(result.status(),CompletionStatus::VERIFIED_COMPLETE);
 ResidualAuditStability stable;
 EXPECT_FALSE(stable.observe(result,0,1));
 EXPECT_FALSE(stable.observe(result,5,2));
 EXPECT_TRUE(stable.observe(result,10,3));
}
TEST(ResidualRegions, RuntimeHistoryStillAllowsIndependentOcclusionProof) {
 auto m=room();m.data[10*60+30]=-1;OccupancyGrid2d grid(m);
 for(int kind:{1,2}) {
  int calls=0;
  auto result=auditResidualRegions(grid,{1.05,2.05},{},0,[](int,int){return true;},
    [&](auto,auto){++calls;return 0;},std::chrono::steady_clock::now()+std::chrono::seconds(2),
    {},[&](int,int){return kind;});
  EXPECT_GT(calls,0);EXPECT_TRUE(result.handoff());
  EXPECT_EQ(result.status(),CompletionStatus::VERIFIED_WITH_RESIDUALS);
  EXPECT_EQ(result.regions.front().state,"occluded");
 }
}
TEST(ResidualRegions, BlockedAndInvalidEvidenceAreSeparateFromCompletion) {
 auto m=room();m.data[10*60+30]=-1;
 auto blocked=auditFixture(m,[](auto,auto){return -2;});
 EXPECT_EQ(blocked.status(),CompletionStatus::BLOCKED);EXPECT_FALSE(blocked.handoff());
 auto uncertain=auditFixture(m,[](auto,auto){return -1;});
 EXPECT_EQ(uncertain.status(),CompletionStatus::INCOMPLETE);EXPECT_FALSE(uncertain.handoff());
 ResidualAudit invalid;invalid.remaining=1;
 EXPECT_EQ(invalid.status(),CompletionStatus::ERROR);EXPECT_FALSE(invalid.handoff());
}
