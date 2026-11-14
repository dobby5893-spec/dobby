#include <gtest/gtest.h>
#include "frontier_exploration_ros2/mapped_observation.hpp"
#include "frontier_exploration_ros2/symmetric_relocation.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
#include <action_msgs/msg/goal_status.hpp>
using namespace frontier_exploration_ros2;
static ObservationGrid openGrid() {
 ObservationGrid g;g.width=120;g.height=80;g.resolution=.1;g.x=-6;g.y=-4;
 g.cells.assign(9600,0);return g;
}
TEST(KnownSymmetry, UnknownCanvasExcludedAndObstaclesIncluded) {
 auto g=openGrid();g.cells.assign(g.cells.size(),-1);
 g.cells[20*g.width+30]=0;g.cells[60*g.width+100]=100;
 auto b=knownBounds(g);ASSERT_TRUE(b);
 EXPECT_NEAR(b->center().first,.55,1e-9);EXPECT_NEAR(b->center().second,.05,1e-9);
 auto p=b->reflect({-2.,-1.});EXPECT_NEAR(p.first,3.1,1e-9);EXPECT_NEAR(p.second,1.1,1e-9);
}
TEST(KnownSymmetry, RotatedGridCenterUsesWorldCoordinates) {
 auto g=openGrid();g.yaw=M_PI/2;
 auto b=knownBounds(g);ASSERT_TRUE(b);
 EXPECT_NEAR(b->center().first,-10,1e-9);EXPECT_NEAR(b->center().second,2,1e-9);
 auto p=b->reflect({-9,1});EXPECT_NEAR(p.first,-11,1e-9);EXPECT_NEAR(p.second,3,1e-9);
}
TEST(KnownSymmetry, EmptyOrMalformedMapHasNoDestination) {
 auto g=openGrid();g.cells.assign(g.cells.size(),-1);EXPECT_FALSE(knownBounds(g));
 g.cells.clear();EXPECT_FALSE(symmetricTarget(g,{-4,-2},.22,{}));
}
TEST(SymmetricTarget, BothCoordinatesReflectAboutKnownBounds) {
 auto g=openGrid();auto t=symmetricTarget(g,{-4,-2},.22,{});
 ASSERT_TRUE(t);EXPECT_NEAR(t->center.first,0,1e-9);EXPECT_NEAR(t->center.second,0,1e-9);
 EXPECT_NEAR(t->mirror.first,4,1e-9);EXPECT_NEAR(t->mirror.second,2,1e-9);
 EXPECT_LT(std::hypot(t->destination.first-4,t->destination.second-2),.1);
}
TEST(SymmetricTarget, OccupiedMirrorProjectsToReachableFreeCell) {
 auto g=openGrid();
 for(int y=55;y<=65;++y)for(int x=95;x<=105;++x)g.cells[y*g.width+x]=100;
 auto t=symmetricTarget(g,{-4,-2},.22,{});ASSERT_TRUE(t);
 int x=std::floor((t->destination.first-g.x)/g.resolution);
 int y=std::floor((t->destination.second-g.y)/g.resolution);
 EXPECT_EQ(g.cells[y*g.width+x],0);
 auto path=mappedObservationRoute(g,{-4,-2,0,0},
   {t->destination.first,t->destination.second,0,0},.22,
   std::chrono::steady_clock::now()+std::chrono::seconds(1),true);
 EXPECT_FALSE(path.empty());
}
TEST(SymmetricTarget, DisconnectedOppositeRoomIsRejected) {
 auto g=openGrid();for(int y=0;y<g.height;++y)g.cells[y*g.width+60]=100;
 EXPECT_FALSE(symmetricTarget(g,{-4,-2},.22,{}));
}
TEST(SymmetricTarget, NearCenterChoosesDistantReachablePoint) {
 auto g=openGrid();auto t=symmetricTarget(g,{0,0},.22,{});ASSERT_TRUE(t);
 EXPECT_GE(std::hypot(t->destination.first,t->destination.second),2.);
}
TEST(SymmetricTarget, MissionBoundaryIsRespected) {
 auto g=openGrid();ExplorationBoundaryPolygon boundary{{-5,-3},{1,-3},{1,3},{-5,3}};
 auto t=symmetricTarget(g,{-4,-2},.22,{},boundary,.15);
 if(t)EXPECT_TRUE(is_inside_exploration_boundary(t->destination,boundary,.15));
}
TEST(SymmetricRelocation, DestinationRemainsFixedWhileMapExpands) {
 auto g=openGrid();SymmetricRelocation p;p.observe(g,0,0);
 ASSERT_TRUE(p.begin(g,{-4,-2},0,{},0));auto destination=p.target->destination;
 g.width=140;g.cells.assign(140*80,0);p.observe(g,0,1);
 ASSERT_TRUE(p.target);EXPECT_EQ(p.target->destination,destination);
 EXPECT_FALSE(p.begin(g,{0,0},2,{},0));
}
TEST(SymmetricRelocation, HistoryExcludesImmediateMirrorReturn) {
 auto g=openGrid();SymmetricRelocation p;p.observe(g,0,0);
 ASSERT_TRUE(p.begin(g,{-4,-2},0,{},0));auto first=p.target->destination;
 p.finish(1);ASSERT_TRUE(p.begin(g,first,62,{},0));
 auto next=p.target->destination;
 EXPECT_GE(std::hypot(next.first+4,next.second+2),1.5);
}
TEST(SymmetricRelocation, WaitDoesNotReopenTwoTripBudgetAndFlickerIsNotGain) {
 auto g=openGrid();SymmetricRelocation p;p.observe(g,0,0);
 ASSERT_TRUE(p.begin(g,{-4,-2},0,{},0));p.finish(1);
 p.begin(g,{-2,-2},62,{},0);p.finish(63);
 EXPECT_FALSE(p.begin(g,{1,1},10000,{},0));EXPECT_EQ(p.attempts,2u);
 g.cells[0]=-1;EXPECT_FALSE(p.observe(g,0,10001));
 g.cells[0]=0;EXPECT_FALSE(p.observe(g,0,10002));EXPECT_EQ(p.attempts,2u);
}
TEST(SymmetricRelocation, NewKnownWorldAreaReopensBudgetButKeepsVisitHistory) {
 auto g=openGrid();g.cells[0]=-1;for(int i=1;i<100;++i)g.cells[i]=-1;
 SymmetricRelocation p;p.observe(g,0,0);p.attempts=2;p.history.push_back({-4,-2});
 for(int i=0;i<100;++i)g.cells[i]=0;
 EXPECT_TRUE(p.observe(g,0,1));EXPECT_EQ(p.attempts,0u);ASSERT_EQ(p.history.size(),1u);
}
TEST(SymmetricRelocation, PlanningBudgetRequiresBothTimeAndCompletedFailures) {
 auto g=openGrid();SymmetricRelocation p;p.observe(g,0,0);ASSERT_TRUE(p.begin(g,{-4,-2},0,{},0));
 p.rejected();p.rejected();
 for(int t=1;t<=30;++t)EXPECT_FALSE(p.planningExpired(t));
 p.rejected();EXPECT_TRUE(p.planningExpired(31));
}
TEST(SymmetricRelocation, TravelPauseAndMeasuredStagesPreventFalsePlanningTimeout) {
 auto g=openGrid();SymmetricRelocation p;p.observe(g,0,0);ASSERT_TRUE(p.begin(g,{-4,-2},0,{},0));
 p.rejected();p.rejected();p.rejected();p.pause(1000);
 EXPECT_FALSE(p.planningExpired(1001));EXPECT_DOUBLE_EQ(p.planning_seconds,1);
 ASSERT_TRUE(p.stage({-3,-2}));EXPECT_DOUBLE_EQ(p.planning_seconds,0);EXPECT_EQ(p.failures,0u);
 EXPECT_FALSE(p.stage({-3,-2}));
}
TEST(SymmetricRelocation, FloorChangeOrBackwardClockClearsOldTrip) {
 auto g=openGrid();SymmetricRelocation p;p.observe(g,0,10);ASSERT_TRUE(p.begin(g,{-4,-2},10,{},0));
 p.observe(g,3,11);EXPECT_FALSE(p.active());EXPECT_TRUE(p.history.empty());
 p.begin(g,{-4,-2},12,{},0);p.observe(g,3,0);EXPECT_FALSE(p.active());
}
TEST(SymmetricRelocationCore, FailedRegionStartsFixedTripAndOrdinaryQueueCannotInterruptIt) {
 nav_msgs::msg::OccupancyGrid m;m.info.width=120;m.info.height=80;m.info.resolution=.1;
 m.header.frame_id="map";m.info.origin.orientation.w=1;m.data.assign(9600,0);
 for(int y=60;y<64;++y)for(int x=80;x<84;++x)m.data[y*120+x]=-1;
 int64_t now=0;geometry_msgs::msg::Pose actual;actual.position.x=1.55;actual.position.y=1.55;actual.orientation.w=1;
 FrontierExplorerCoreParams params;params.region_observation_tasks_enabled=true;
 params.reachability_filter_enabled=false;params.post_goal_settle_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
 cb.wait_for_action_server=[](double){return true;};cb.floor_cleanup_data_ready=[](){return true;};
 cb.validate_frontier_goal=[](const auto &,const auto &)->std::optional<geometry_msgs::msg::PoseStamped>{return {};};
 std::vector<RelocationPoint> targets;
 cb.request_symmetric_relocation_step=[&](const auto & pose,RelocationPoint destination) {
   targets.push_back(destination);RelocationStep step;geometry_msgs::msg::PoseStamped g;
   g.header.frame_id="map";g.pose=pose;g.pose.position.x+=1.;step.goal=g;return step;};
 FrontierCandidate first{{8.05,6.05},{7.05,6.05},4};
 cb.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){
   FrontierSearchResult result;result.frontiers={first};result.robot_map_cell={15,15};return result;};
 std::vector<GoalDispatchRequest> sent;cb.dispatch_goal_request=[&](const auto & goal){sent.push_back(goal);};
 FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
 EXPECT_FALSE(core.send_frontier_goal({first},actual,"failed first route"));
 for(int t=1;t<=61 && sent.empty();++t) {
   now=t*1000000000LL;m.header.stamp.sec=t;core.map=OccupancyGrid2d(m);
   if(t==1 || t==20 || t==40)core.report_frontier_validation({8.05,6.05},false);
   core.try_send_next_goal();
 }
 ASSERT_EQ(sent.size(),1u);EXPECT_EQ(sent.back().goal_kind,"symmetry_relocation");
 ASSERT_EQ(targets.size(),1u);auto fixed=targets.front();
 core.pending_frontier_sequence={first};
 actual=sent.back().goal_pose.pose;now+=1000000000LL;
 m.header.stamp.sec=now/1000000000LL;core.map=OccupancyGrid2d(m);
 core.get_result_callback(sent.back().dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
 ASSERT_EQ(sent.size(),2u);EXPECT_EQ(sent.back().goal_kind,"symmetry_relocation");
 ASSERT_EQ(targets.size(),2u);EXPECT_EQ(targets.back(),fixed);
 EXPECT_TRUE(core.pending_frontier_sequence.empty());EXPECT_TRUE(core.exploration_enabled);
}
