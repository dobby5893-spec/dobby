#include <gtest/gtest.h>
#include "frontier_exploration_ros2/region_observation_task.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
#include <action_msgs/msg/goal_status.hpp>
#include <iostream>
using namespace frontier_exploration_ros2;
static nav_msgs::msg::OccupancyGrid rooms() {
 nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=100;m.info.height=60;m.info.resolution=.1;
 m.info.origin.orientation.w=1;m.data.assign(6000,0);
 for(int y=10;y<50;++y)for(int x=20;x<60;++x)
   m.data[y*100+x]=(x==20 || x==59 || y==10 || y==49)?100:-1;
 for(int y=10;y<50;++y)for(int x=65;x<95;++x)
   m.data[y*100+x]=(x==65 || x==94 || y==10 || y==49)?100:-1;
 return m;
}
static void stamp(nav_msgs::msg::OccupancyGrid & m,int t) {m.header.stamp.sec=t;}
TEST(RegionObservationTask, CentroidDriftAndSplitRetainOriginalUnknownFootprint) {
 auto m=rooms();RegionObservationTask task;
 ASSERT_TRUE(task.start(OccupancyGrid2d(m),{3.05,3.05},0));
 EXPECT_TRUE(task.matches(OccupancyGrid2d(m),{4.85,3.05}));
 for(int y=11;y<49;++y)m.data[y*100+39]=100;
 EXPECT_TRUE(task.matches(OccupancyGrid2d(m),{4.85,3.05}));
 EXPECT_FALSE(task.matches(OccupancyGrid2d(m),{7.05,3.05}));
}
TEST(RegionObservationTask, NearbyRegionsAcrossObservedWallDoNotMerge) {
 auto m=rooms();
 for(int y=11;y<49;++y)for(int x=60;x<65;++x)m.data[y*100+x]=100;
 RegionObservationTask task;ASSERT_TRUE(task.start(OccupancyGrid2d(m),{5.75,3.05},0));
 EXPECT_FALSE(task.matches(OccupancyGrid2d(m),{6.75,3.05}));
}
TEST(RegionObservationTask, FiveFreshStationarySecondsChangeViewButNotRegion) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.5,3.},0);
 for(int t=1;t<=5;++t) {
   stamp(m,t);auto d=task.poll(OccupancyGrid2d(m),{1.5,3.},t,false,false);
   EXPECT_EQ(d,t<5?RegionObservationTask::Decision::Waiting:RegionObservationTask::Decision::ChangeView);
 }
 EXPECT_TRUE(task.active());EXPECT_EQ(task.triedPositions(),1u);EXPECT_DOUBLE_EQ(task.idleSeconds(),5.);
}
TEST(RegionObservationTask, TravelAndRepeatedMapDoNotConsumeObservationTime) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.5,3.},1000);
 for(int t=1001;t<1010;++t) {
   EXPECT_EQ(task.poll(OccupancyGrid2d(m),{1.5,3.},t,false,false),RegionObservationTask::Decision::Waiting);
 }
 EXPECT_DOUBLE_EQ(task.idleSeconds(),0.);
 stamp(m,1010);task.poll(OccupancyGrid2d(m),{1.5,3.},1010,false,false);
 EXPECT_DOUBLE_EQ(task.idleSeconds(),0.);
}
TEST(RegionObservationTask, MissingDataPauseCannotBeChargedRetroactively) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.5,3.},0);
 stamp(m,1);task.poll(OccupancyGrid2d(m),{1.5,3.},1,false,false);
 task.pause(100);stamp(m,101);task.poll(OccupancyGrid2d(m),{1.5,3.},101,false,false);
 EXPECT_DOUBLE_EQ(task.idleSeconds(),2.);
}
TEST(RegionObservationTask, RequiresBothSixtySecondsAndThreeDistinctReachedPositions) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 int now=0;
 for(int attempt=0;attempt<12;++attempt) {
   RegionObservationTask::Point p{1.5,2.+(attempt%3)*.5};
   task.arrived(OccupancyGrid2d(m),p,now);
   for(int i=0;i<5;++i) {
     ++now;stamp(m,now);
     auto d=task.poll(OccupancyGrid2d(m),p,now,false,false);
     if(attempt==11 && i==4)EXPECT_EQ(d,RegionObservationTask::Decision::Deferred);
     else EXPECT_NE(d,RegionObservationTask::Decision::Deferred);
   }
 }
 EXPECT_FALSE(task.active());EXPECT_TRUE(task.deferred(OccupancyGrid2d(m),{4.8,3.},61));
 EXPECT_FALSE(task.deferred(OccupancyGrid2d(m),{4.8,3.},121)); // A larger adjoining footprint is not wholly exhausted.
 EXPECT_LT(m.data[30*100+30],0);
}
TEST(RegionObservationTask, SixtySecondsAtOnePositionCannotDeferWholeRegion) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 int now=0;
 for(int attempt=0;attempt<12;++attempt) {
   task.arrived(OccupancyGrid2d(m),{1.5,3.},now);
   for(int i=0;i<5;++i) {stamp(m,++now);task.poll(OccupancyGrid2d(m),{1.5,3.},now,false,false);}
 }
 EXPECT_TRUE(task.active());EXPECT_EQ(task.triedPositions(),1u);EXPECT_DOUBLE_EQ(task.idleSeconds(),60.);
}
TEST(RegionObservationTask, SingleCellInformationResetsBudgetAndNeverCountsTwice) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.5,3.},0);
 for(int t=1;t<=5;++t) {stamp(m,t);task.poll(OccupancyGrid2d(m),{1.5,3.},t,false,false);}
 ASSERT_EQ(task.triedPositions(),1u);
 m.data[30*100+30]=0;EXPECT_TRUE(task.update(OccupancyGrid2d(m),6));
 EXPECT_EQ(task.triedPositions(),0u);EXPECT_DOUBLE_EQ(task.idleSeconds(),0.);
 m.data[30*100+30]=-1;task.update(OccupancyGrid2d(m),7);
 m.data[30*100+30]=0;EXPECT_FALSE(task.update(OccupancyGrid2d(m),8));
}
TEST(RegionObservationTask, RouteAdvanceWaitsForFreshObservationAndPreservesPriorEvidence) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.5,3.},0);
 for(int t=1;t<=5;++t) {stamp(m,t);task.poll(OccupancyGrid2d(m),{1.5,3.},t,false,false);}
 task.arrived(OccupancyGrid2d(m),{1.5,3.5},5);stamp(m,6);
 EXPECT_EQ(task.poll(OccupancyGrid2d(m),{1.5,3.5},6,false,true),RegionObservationTask::Decision::Waiting);
 for(int t=7;t<10;++t){stamp(m,t);task.poll(OccupancyGrid2d(m),{1.5,3.5},t,false,true);}
 stamp(m,10);
 EXPECT_EQ(task.poll(OccupancyGrid2d(m),{1.5,3.5},10,false,true),RegionObservationTask::Decision::Advance);
 EXPECT_TRUE(task.active());EXPECT_EQ(task.triedPositions(),2u);EXPECT_DOUBLE_EQ(task.idleSeconds(),10.);
}
TEST(RegionObservationTask, CompleteFootprintAndFloorResetAreExplicit) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 EXPECT_FALSE(task.finished(OccupancyGrid2d(m)));
 for(int y=11;y<49;++y)for(int x=21;x<59;++x)m.data[y*100+x]=0;
 task.update(OccupancyGrid2d(m),1);EXPECT_TRUE(task.finished(OccupancyGrid2d(m)));
 m.info.origin.position.z=3;task.update(OccupancyGrid2d(m),2);EXPECT_FALSE(task.active());
}
TEST(RegionObservationTask, CoreRetainsRegionAfterGainAndCentroidMovesPastOldTolerance) {
 auto m=rooms();int64_t now=0;geometry_msgs::msg::Pose actual;actual.position.x=.85;actual.position.y=3.05;actual.orientation.w=1;
 FrontierExplorerCoreParams params;params.region_observation_tasks_enabled=true;
 params.reachability_filter_enabled=false;params.post_goal_min_settle=2.;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
 cb.wait_for_action_server=[](double){return true;};
 cb.log_info=[](const auto & msg){std::cerr<<msg<<"\n";};
 cb.log_warn=[](const auto & msg){std::cerr<<msg<<"\n";};
 std::vector<GoalDispatchRequest> sent;cb.dispatch_goal_request=[&](const auto & g){sent.push_back(g);};
 cb.validate_frontier_goal=[](const auto &,const auto & g)->std::optional<geometry_msgs::msg::PoseStamped>{return g;};
 FrontierCandidate shifted{{4.85,3.05},{1.55,2.05},30};shifted.path_cost=10;
 FrontierCandidate other{{7.05,3.05},{1.55,4.05},30};other.path_cost=1;
 cb.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){
   FrontierSearchResult r;r.robot_map_cell={15,30};r.frontiers={other,shifted};return r;};
 FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
 FrontierCandidate first{{3.05,3.05},{1.55,3.05},20};
 ASSERT_TRUE(core.send_frontier_goal({first},actual,"region begin"));ASSERT_EQ(sent.size(),1u);
 actual.position.x=1.55;
 core.get_result_callback(sent.back().dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
 m.data[30*100+30]=0;stamp(m,2);core.map=OccupancyGrid2d(m);now=2'000'000'000LL;
 core.try_send_next_goal();
 ASSERT_EQ(sent.size(),2u);ASSERT_TRUE(sent.back().frontier);
 EXPECT_NEAR(frontier_reference_point(*sent.back().frontier).first,4.85,1e-6);
}

TEST(RegionObservationTask, QueuedReplacementCannotSkipFiveSecondArrivalObservation) {
 auto m=rooms();int64_t now=0;
 geometry_msgs::msg::Pose actual;actual.position.x=1.55;actual.position.y=3.05;actual.orientation.w=1;
 FrontierExplorerCoreParams params;params.region_observation_tasks_enabled=true;params.post_goal_min_settle=2.;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
 cb.wait_for_action_server=[](double){return true;};
 std::vector<GoalDispatchRequest> sent;cb.dispatch_goal_request=[&](const auto & g){sent.push_back(g);};
 cb.validate_frontier_goal=[](const auto &,const auto & g)->std::optional<geometry_msgs::msg::PoseStamped>{return g;};
 FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
 FrontierCandidate first{{3.05,3.05},{1.55,3.05},20};
 geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose=actual;
 ASSERT_TRUE(core.send_pose_goal(goal,"frontier",first,{first},"observe here"));
 FrontierCandidate next{{4.85,3.05},{1.55,2.05},20};core.pending_frontier_sequence={next};
 core.get_result_callback(sent.front().dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
 ASSERT_EQ(sent.size(),1u);
 for(int t=1;t<=5;++t) {
   now=t*1'000'000'000LL;stamp(m,t);core.map=OccupancyGrid2d(m);
   core.try_send_next_goal();
   EXPECT_EQ(sent.size(),t<5?1u:2u);
 }
 ASSERT_EQ(sent.size(),2u);
 EXPECT_NEAR(sent.back().goal_pose.pose.position.y,2.05,1e-6);
}
TEST(RegionObservationTask, MissionBoundaryDoesNotRetainOutsideUnknownCells) {
 auto m=rooms();RegionObservationTask task;
 task.boundary={{2.5,1.5},{4.5,1.5},{4.5,4.5},{2.5,4.5}};
 ASSERT_TRUE(task.start(OccupancyGrid2d(m),{3.05,3.05},0));
 for(int y=15;y<45;++y)for(int x=25;x<45;++x)m.data[y*100+x]=0;
 task.update(OccupancyGrid2d(m),1);EXPECT_TRUE(task.finished(OccupancyGrid2d(m)));
 EXPECT_LT(m.data[30*100+50],0);
}

TEST(RegionObservationTask, NewlyExposedUnknownsAdjoiningCompletedFootprintRetainTask) {
 auto m=rooms();m.info.width=120;m.data.assign(7200,-1);
 RegionObservationTask task;ASSERT_TRUE(task.start(OccupancyGrid2d(m),{1.05,3.05},0));
 for(int y=0;y<60;++y)for(int x=0;x<120;++x) {
   const double wx=(x+.5)*m.info.resolution,wy=(y+.5)*m.info.resolution;
   if(std::hypot(wx-1.05,wy-3.05)<=3.)m.data[y*120+x]=0;
 }
 task.update(OccupancyGrid2d(m),1);
 ASSERT_TRUE(task.finished(OccupancyGrid2d(m)));
 ASSERT_TRUE(task.matches(OccupancyGrid2d(m),{4.15,3.05}));
 task.extend(OccupancyGrid2d(m),{4.15,3.05});
 EXPECT_FALSE(task.finished(OccupancyGrid2d(m)));EXPECT_TRUE(task.active());
}

TEST(RegionPlanningBudget, NoArrivalStillDefersAfterThreeFailedRoundsAndSixtyFreshSeconds) {
 auto m=rooms();RegionObservationTask task;ASSERT_TRUE(task.start(OccupancyGrid2d(m),{3.05,3.05},0));
 for(int t=1;t<=60;++t) {
   if(t%20==0)task.planningFailure();
   stamp(m,t);EXPECT_EQ(task.planningWait(OccupancyGrid2d(m),t),t==60);
 }
 EXPECT_FALSE(task.active());EXPECT_EQ(task.triedPositions(),0u);
 EXPECT_TRUE(task.deferred(OccupancyGrid2d(m),{3.05,3.05},61));
 EXPECT_FALSE(task.deferred(OccupancyGrid2d(m),{7.05,3.05},61));
 EXPECT_LT(m.data[30*100+30],0); // Unknown never becomes occupied or complete.
}
TEST(RegionPlanningBudget, SparseFailuresTravelAndStaleDataCannotPrematurelyDefer) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.planningFailure();task.planningFailure();
 for(int t=1;t<=65;++t) {stamp(m,t);EXPECT_FALSE(task.planningWait(OccupancyGrid2d(m),t));}
 task.pause(1000);task.planningFailure();
 EXPECT_FALSE(task.planningWait(OccupancyGrid2d(m),1001)); // Same map cannot finish a task.
 stamp(m,1001);EXPECT_TRUE(task.planningWait(OccupancyGrid2d(m),1001));
}
TEST(RegionPlanningBudget, ActualInformationResetsFailureBudgetAndEmptyRoundsAreRateLimited) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 for(int i=0;i<100;++i)task.noCandidate(0);
 EXPECT_EQ(task.planningFailures(),1u);
 for(int t=1;t<=30;++t) {stamp(m,t);task.planningWait(OccupancyGrid2d(m),t);}
 m.data[30*100+30]=0;EXPECT_TRUE(task.update(OccupancyGrid2d(m),31));
 EXPECT_EQ(task.planningFailures(),0u);EXPECT_DOUBLE_EQ(task.planningSeconds(),0);
 task.pause(1000);stamp(m,1001);EXPECT_FALSE(task.planningWait(OccupancyGrid2d(m),1001));
 EXPECT_DOUBLE_EQ(task.planningSeconds(),1);
}
TEST(RegionPlanningBudget, RepeatedSchedulerTicksDoNotUnderCountFreshMapIntervals) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 for(int t=1;t<=600;++t) {
   if(t%10==0)stamp(m,t/10);
   task.planningWait(OccupancyGrid2d(m),t*.1);
 }
 EXPECT_NEAR(task.planningSeconds(),60,1e-6);
}

TEST(RegionPlanningBudget, CoreLeavesRegionWhoseFirstValidationNeverSucceeds) {
 auto m=rooms();int64_t now=0;
 geometry_msgs::msg::Pose actual;actual.position.x=1.55;actual.position.y=3.05;actual.orientation.w=1;
 FrontierExplorerCoreParams params;params.region_observation_tasks_enabled=true;
 params.reachability_filter_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
 cb.wait_for_action_server=[](double){return true;};
 cb.floor_cleanup_data_ready=[](){return true;};
 std::vector<GoalDispatchRequest> sent;cb.dispatch_goal_request=[&](const auto & g){sent.push_back(g);};
 cb.validate_frontier_goal=[](const auto &,const auto & g)->std::optional<geometry_msgs::msg::PoseStamped>{
   if(g.pose.position.y<3.5)return std::nullopt;
   return g;
 };
 FrontierCandidate first{{3.05,3.05},{1.55,3.05},20};
 FrontierCandidate other{{7.05,3.05},{1.55,4.05},30};
 cb.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){
   FrontierSearchResult r;r.robot_map_cell={15,30};r.frontiers={first,other};return r;};
 FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
 EXPECT_FALSE(core.send_frontier_goal({first},actual,"never verified"));
 for(int t=1;t<=61;++t) {
   now=t*1'000'000'000LL;stamp(m,t);core.map=OccupancyGrid2d(m);
   if(t==1 || t==20 || t==40)core.report_frontier_validation({3.05,3.05},false);
   core.try_send_next_goal();
   if(!sent.empty())break;
 }
 ASSERT_EQ(sent.size(),1u);ASSERT_TRUE(sent.front().frontier);
 EXPECT_NEAR(core.frontier_reference_point(*sent.front().frontier).first,7.05,1e-6);
 EXPECT_TRUE(core.exploration_enabled);
 EXPECT_LT(m.data[30*100+30],0);
}

static void exhaustObserved(RegionObservationTask & task,nav_msgs::msg::OccupancyGrid & m) {
 task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.no_gain_wait_s=20;int now=0;
 for(int a=0;a<3;++a) {
  RegionObservationTask::Point p{1.5,2.+a*.5};task.arrived(OccupancyGrid2d(m),p,now);
  for(int i=0;i<20;++i){stamp(m,++now);task.poll(OccupancyGrid2d(m),p,now,false,false);}
 }
}
TEST(RegionalEvidence, ObservedFootprintSurvivesCooldownAndUnrelatedMapChanges) {
 auto m=rooms();RegionObservationTask task;exhaustObserved(task,m);
 auto mask=task.evidenceMask(OccupancyGrid2d(m),61);
 EXPECT_EQ(mask[30*100+30],1);EXPECT_EQ(mask[30*100+70],0);
 m.data[0]=100;stamp(m,1000);
 mask=task.evidenceMask(OccupancyGrid2d(m),1000);EXPECT_EQ(mask[30*100+30],1);
 EXPECT_LT(m.data[30*100+30],0);
}
TEST(RegionalEvidence, NewInformationAndDoorOpeningRevokeOnlyLocalArchive) {
 auto m=rooms();RegionObservationTask task;exhaustObserved(task,m);
 m.data[30*100+30]=0;stamp(m,61);
 auto mask=task.evidenceMask(OccupancyGrid2d(m),61);
 EXPECT_EQ(mask[31*100+30],0);
 auto n=rooms();RegionObservationTask other;exhaustObserved(other,n);
 n.data[30*100+20]=0;stamp(n,61);
 EXPECT_EQ(other.evidenceMask(OccupancyGrid2d(n),61)[30*100+30],0);
 EXPECT_FALSE(other.deferred(OccupancyGrid2d(n),{3.05,3.05},61));
}
TEST(RegionalEvidence, PlanningFailureNeedsDifferentMeasuredOriginsAndIsNotNoGainObservation) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 for(int i=0;i<3;++i)task.planningFailure(RegionObservationTask::Point{1.,1.+i*.6});
 for(int t=1;t<=60;++t){stamp(m,t);task.planningWait(OccupancyGrid2d(m),t);}
 auto mask=task.evidenceMask(OccupancyGrid2d(m),61);EXPECT_EQ(mask[30*100+30],2);
 EXPECT_EQ(task.triedPositions(),0u);
}
TEST(RegionalEvidence, OneOriginFailuresOrStaleDataCannotCertifyTaskExhaustion) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 for(int i=0;i<100;++i)task.planningFailure(RegionObservationTask::Point{1.,1.});
 for(int t=1;t<=60;++t){stamp(m,t);task.planningWait(OccupancyGrid2d(m),t);}
 EXPECT_EQ(task.evidenceMask(OccupancyGrid2d(m),61)[30*100+30],0);
 RegionObservationTask stale;stale.start(OccupancyGrid2d(m),{3.05,3.05},0);
 for(int i=0;i<3;++i)stale.planningFailure(RegionObservationTask::Point{1.,1.+i*.6});
 for(int t=1;t<=100;++t)stale.planningWait(OccupancyGrid2d(m),t);
 EXPECT_EQ(stale.evidenceMask(OccupancyGrid2d(m),101)[30*100+30],0);
}
TEST(RegionalEvidence, ReleasingForAnotherTaskDoesNotEraseActualObservationEffort) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.5,3.},0);
 for(int t=1;t<=5;++t){stamp(m,t);task.poll(OccupancyGrid2d(m),{1.5,3.},t,false,false);}
 task.release();ASSERT_TRUE(task.start(OccupancyGrid2d(m),{3.05,3.05},6));
 EXPECT_EQ(task.triedPositions(),1u);EXPECT_DOUBLE_EQ(task.idleSeconds(),5.);
}

static void oneScan(RegionObservationTask & task,nav_msgs::msg::OccupancyGrid & m) {
 task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.55,3.05},0,0.);
 for(int t=1;t<=5;++t){stamp(m,t);task.poll(OccupancyGrid2d(m),{1.55,3.05},t,false,false);}
}
TEST(ScanHistory, ReusesOneCompletedScanWithoutRequiringWholeRegionExhaustion) {
 auto m=rooms();RegionObservationTask task;oneScan(task,m);
 EXPECT_TRUE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.55,3.05},0));
 EXPECT_FALSE(task.deferred(OccupancyGrid2d(m),{3.05,3.05},6));
 EXPECT_EQ(task.evidenceMask(OccupancyGrid2d(m),6)[30*100+30],0);
 EXPECT_LT(m.data[30*100+30],0);
}
TEST(ScanHistory, OtherCornersPositionsAndHeadingsRemainEligible) {
 auto m=rooms();RegionObservationTask task;oneScan(task,m);
 EXPECT_FALSE(task.alreadyObserved(OccupancyGrid2d(m),{3.25,3.05},{1.55,3.05},0));
 EXPECT_FALSE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.75,3.05},0));
 EXPECT_FALSE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.55,3.05},1.));
}
TEST(ScanHistory, TraversalArrivalAndStaleMapsAreNotCompletedScans) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.55,3.05},0,0);
 for(int t=1;t<=30;++t)task.poll(OccupancyGrid2d(m),{1.55,3.05},t,false,false);
 EXPECT_FALSE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.55,3.05},0));
}
TEST(ScanHistory, UnrelatedChangesPreserveHistoryButLocalGainAndDoorOpeningRevokeIt) {
 auto m=rooms();RegionObservationTask task;oneScan(task,m);task.release();
 m.data[0]=100;stamp(m,1000);
 EXPECT_TRUE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.55,3.05},0));
 m.data[30*100+20]=0;
 EXPECT_FALSE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.55,3.05},0));
 auto n=rooms();RegionObservationTask other;oneScan(other,n);n.data[30*100+30]=0;
 EXPECT_FALSE(other.alreadyObserved(OccupancyGrid2d(n),{3.05,3.05},{1.55,3.05},0));
}
TEST(ScanHistory, ActualInformationGainDoesNotCreateNoGainHistory) {
 auto m=rooms();RegionObservationTask task;task.start(OccupancyGrid2d(m),{3.05,3.05},0);
 task.arrived(OccupancyGrid2d(m),{1.55,3.05},0,0);stamp(m,1);
 EXPECT_EQ(task.poll(OccupancyGrid2d(m),{1.55,3.05},1,true,false),RegionObservationTask::Decision::Gain);
 EXPECT_FALSE(task.alreadyObserved(OccupancyGrid2d(m),{3.05,3.05},{1.55,3.05},0));
}

TEST(ScanHistory, CoreSkipsOldScanAfterTemporarySuppressionExpiresAndKeepsAnotherView) {
 auto m=rooms();int64_t now=0;bool allow=false;
 geometry_msgs::msg::Pose actual;actual.position.x=1.55;actual.position.y=3.05;actual.orientation.w=1;
 FrontierExplorerCoreParams params;params.region_observation_tasks_enabled=true;
 params.reachability_filter_enabled=false;params.post_goal_min_settle=2.;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[&](){return std::optional<geometry_msgs::msg::Pose>(actual);};
 cb.wait_for_action_server=[](double){return true;};
 std::vector<GoalDispatchRequest> sent;std::vector<geometry_msgs::msg::PoseStamped> last_fan;
 cb.dispatch_goal_request=[&](const auto & g){sent.push_back(g);};
 cb.validate_frontier_goal_set=[&](const auto &,const auto & fan)->std::optional<geometry_msgs::msg::PoseStamped>{
   last_fan=fan;if(allow && !fan.empty())return fan.front();return std::nullopt;
 };
 FrontierCandidate first{{3.05,3.05},{1.55,3.05},20};
 first.requires_visible_viewpoint=true;first.viewpoint_candidates={{1.55,3.05},{1.55,2.05}};
 cb.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){
   FrontierSearchResult r;r.robot_map_cell={15,30};r.frontiers={first};return r;
 };
 FrontierExplorerCore core(params,cb);core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
 geometry_msgs::msg::PoseStamped goal;goal.header.frame_id="map";goal.pose=actual;
 ASSERT_TRUE(core.send_pose_goal(goal,"frontier",first,{first},"observe"));
 core.get_result_callback(sent.front().dispatch_id,action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,0,"","");
 for(int t=1;t<=5;++t){now=t*1'000'000'000LL;stamp(m,t);core.map=OccupancyGrid2d(m);core.try_send_next_goal();}
 allow=true;now=1000'000'000'000LL;stamp(m,1000);core.map=OccupancyGrid2d(m);
 core.pending_frontier_sequence={first};core.try_send_next_goal();
 ASSERT_FALSE(last_fan.empty());
 for(const auto & g:last_fan)EXPECT_GT(std::hypot(g.pose.position.x-1.55,g.pose.position.y-3.05),.1);
 EXPECT_TRUE(std::any_of(last_fan.begin(),last_fan.end(),[](const auto & g){return std::abs(g.pose.position.y-2.05)<.1;}));
}
