#include <gtest/gtest.h>
#include "frontier_exploration_ros2/floor_coverage_policy.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"
using namespace frontier_exploration_ros2;
static nav_msgs::msg::OccupancyGrid coverageGrid() {
 nav_msgs::msg::OccupancyGrid m;m.header.frame_id="map";m.info.width=60;m.info.height=50;m.info.resolution=.1;
 m.info.origin.orientation.w=1;m.data.assign(3000,0);return m;
}
static void room(nav_msgs::msg::OccupancyGrid & m,int x,int y,int size,bool door) {
 int w=m.info.width;
 for(int j=y;j<y+size;++j)for(int i=x;i<x+size;++i)
   m.data[j*w+i]=(j==y || j==y+size-1 || i==x || i==x+size-1)?100:-1;
 if(door)m.data[(y+size/2)*w+x]=0;
}
static void observe(FloorCoveragePolicy & p,nav_msgs::msg::OccupancyGrid & m,int t) {
 m.header.stamp.sec=t;p.observe(t,OccupancyGrid2d(m),{},0);
}
TEST(FloorCoveragePolicy, FourOpenRoomsNeverHandOffBecauseOfTimeout) {
 auto m=coverageGrid();
 for(auto xy:{std::pair<int,int>{3,3},{30,3},{3,30},{30,30}})
   room(m,xy.first,xy.second,12,true);
 FloorCoveragePolicy p;p.idle_timeout=10;p.cleanup_budget=20;p.minimum_attempts=1;
 observe(p,m,0);p.enterCleanup(10);
 for(int t:{10,20,40,100,10000}) {p.attempt(t);observe(p,m,t);EXPECT_FALSE(p.shouldHandoff(t));}
 EXPECT_TRUE(p.hasActionableUnknown());EXPECT_DOUBLE_EQ(p.excluded_unknown_area,0);
 EXPECT_GT(p.actionable_unknown_area,3.);
}
TEST(FloorCoveragePolicy, LargeSealedRoomExcludedWithoutChangingRawMap) {
 auto m=coverageGrid();room(m,10,10,22,false);auto original=m.data;
 FloorCoveragePolicy p;observe(p,m,0);observe(p,m,5);
 EXPECT_FALSE(p.shouldHandoff(5));
 observe(p,m,12);
 EXPECT_TRUE(p.shouldHandoff(12));EXPECT_FALSE(p.hasActionableUnknown());
 EXPECT_NEAR(p.excluded_unknown_area,4.,1e-6);EXPECT_EQ(m.data,original);
 EXPECT_TRUE(p.excludedCell(15,15));EXPECT_FALSE(p.excludedCell(0,0));
}
TEST(FloorCoveragePolicy, RepeatedSameMapIsNotIndependentClosureEvidence) {
 auto m=coverageGrid();room(m,10,10,22,false);FloorCoveragePolicy p;
 observe(p,m,0);
 for(double now:{5.,12.,100.,1000.})p.observe(now,OccupancyGrid2d(m),{},0);
 EXPECT_FALSE(p.shouldHandoff(1000));EXPECT_TRUE(p.hasActionableUnknown());
}
TEST(FloorCoveragePolicy, DoorReopeningImmediatelyRevokesExclusion) {
 auto m=coverageGrid();room(m,10,10,22,false);FloorCoveragePolicy p;
 for(int t:{0,5,12})observe(p,m,t);
 ASSERT_TRUE(p.shouldHandoff(12));
 m.data[21*60+10]=0;observe(p,m,13);
 EXPECT_FALSE(p.shouldHandoff(13));EXPECT_TRUE(p.hasActionableUnknown());
 EXPECT_DOUBLE_EQ(p.excluded_unknown_area,0);
}
TEST(FloorCoveragePolicy, DiagonalGapAndMissionBoundaryNeverCountAsClosed) {
 auto m=coverageGrid();room(m,10,10,22,false);m.data[10*60+10]=0;
 FloorCoveragePolicy p;for(int t:{0,5,12})observe(p,m,t);
 EXPECT_FALSE(p.shouldHandoff(12));
 room(m,10,10,22,false);
 ExplorationBoundaryPolygon boundary{{0.,0.},{2.,0.},{2.,5.},{0.,5.}};
 for(int t:{20,25,32}) {m.header.stamp.sec=t;p.observe(t,OccupancyGrid2d(m),boundary,0);}
 EXPECT_FALSE(p.shouldHandoff(32));EXPECT_TRUE(p.hasActionableUnknown());
}
TEST(FloorCoveragePolicy, UnknownAtMapEdgeAndUncertainBoundaryRemainActionable) {
 auto m=coverageGrid();for(int y=10;y<20;++y)m.data[y*60]=-1;
 FloorCoveragePolicy p;for(int t:{0,5,12})observe(p,m,t);
 EXPECT_FALSE(p.shouldHandoff(12));
 m=coverageGrid();room(m,10,10,22,false);m.data[21*60+10]=50;p.clear();
 for(int t:{0,5,12})observe(p,m,t);
 EXPECT_FALSE(p.shouldHandoff(12));
}
TEST(FloorCoveragePolicy, MissingSensorDataPausesClosureEvidenceTime) {
 auto m=coverageGrid();room(m,10,10,22,false);FloorCoveragePolicy p;
 observe(p,m,0);observe(p,m,5);p.waitForData(5);
 EXPECT_FALSE(p.shouldHandoff(100));
 observe(p,m,105);EXPECT_FALSE(p.shouldHandoff(105));
 observe(p,m,110);EXPECT_TRUE(p.shouldHandoff(110));
}
TEST(FloorCoveragePolicy, NoGoalExpandsAndRealGainCanRestoreNormalSearch) {
 auto m=coverageGrid();room(m,10,10,22,true);FloorCoveragePolicy p;observe(p,m,0);
 EXPECT_TRUE(p.expand(1));EXPECT_EQ(p.expansion(),1u);
 EXPECT_FALSE(p.expand(2));EXPECT_TRUE(p.expand(21));EXPECT_EQ(p.expansion(),2u);
 EXPECT_TRUE(p.expand(41));EXPECT_EQ(p.expansion(),3u);
 EXPECT_FALSE(p.shouldHandoff(10000));
 for(int y=11;y<16;++y)for(int x=11;x<26;++x)m.data[y*60+x]=0;
 observe(p,m,42);EXPECT_FALSE(p.cleanup());EXPECT_EQ(p.expansion(),0u);
}
TEST(FloorCoveragePolicy, FloorClockAndGeometryChangesDiscardOldExclusions) {
 auto m=coverageGrid();room(m,10,10,22,false);FloorCoveragePolicy p;
 for(int t:{0,5,12})observe(p,m,t);
 ASSERT_TRUE(p.shouldHandoff(12));
 m.info.origin.position.z=3;observe(p,m,13);EXPECT_FALSE(p.shouldHandoff(13));
 observe(p,m,18);observe(p,m,25);ASSERT_TRUE(p.shouldHandoff(25));
 observe(p,m,0);EXPECT_FALSE(p.shouldHandoff(0));
}
TEST(FloorCoveragePolicy, CoreCannotCompleteOrHandoffFourUnfinishedRooms) {
 int64_t now=0;int complete=0,reports=0;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 params.floor_no_gain_timeout_s=10;params.floor_cleanup_budget_s=20;
 params.reachability_filter_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.5;p.position.y=.5;p.orientation.w=1;
   return std::optional<geometry_msgs::msg::Pose>(p);};
 cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool) {
   FrontierSearchResult result;result.robot_map_cell={5,5};return result;};
 cb.on_exploration_complete=[&](){++complete;};
 cb.on_floor_exhausted=[&](const auto &){++reports;};
 FrontierExplorerCore core(params,cb);auto m=coverageGrid();
 for(auto xy:{std::pair<int,int>{3,3},{30,3},{3,30},{30,30}})
   room(m,xy.first,xy.second,12,true);
 for(int t:{0,10,30,60,300,10000}) {
   now=t*1000000000LL;m.header.stamp.sec=t;
   core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);core.try_send_next_goal();
 }
 EXPECT_EQ(reports,0);EXPECT_EQ(complete,0);
 EXPECT_EQ(core.frontier_search_options().viewpoint_expansion_level,3u);
 EXPECT_EQ(core.frontier_search_options().min_frontier_size_cells,1);
}
TEST(FloorCoveragePolicy, CoreClosedRoomReportRequiresPersistentEvidence) {
 int64_t now=0;int complete=0;std::vector<std::string> reports;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.5;p.position.y=.5;p.orientation.w=1;
   return std::optional<geometry_msgs::msg::Pose>(p);};
 cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){return FrontierSearchResult{};};
 cb.on_exploration_complete=[&](){++complete;};
 cb.on_floor_exhausted=[&](const auto & text){reports.push_back(text);};
 FrontierExplorerCore core(params,cb);auto m=coverageGrid();room(m,10,10,22,false);
 for(int t:{0,5,12}) {
   now=t*1000000000LL;m.header.stamp.sec=t;core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
   core.try_send_next_goal();if(t<12) {EXPECT_TRUE(reports.empty());}
 }
 ASSERT_EQ(reports.size(),1u);EXPECT_EQ(complete,0);
 EXPECT_NE(reports.back().find("reason=closed_regions_excluded"),std::string::npos);
 EXPECT_NE(reports.back().find("actionable_unknown_area=0.000000"),std::string::npos);
 EXPECT_EQ(m.data[15*60+15],-1);
}
TEST(FloorCoveragePolicy, CoreDoesNotConsumeUnavailableMapAsNoGain) {
 int64_t now=0;bool healthy=false;int searches=0,reports=0;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 params.floor_no_gain_timeout_s=10;params.floor_cleanup_budget_s=10;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.floor_cleanup_data_ready=[&](){return healthy;};
 cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.5;p.position.y=.5;p.orientation.w=1;
   return std::optional<geometry_msgs::msg::Pose>(p);};
 cb.frontier_search=[&](const auto &,const auto &,const auto &,const auto &,double,bool){++searches;return FrontierSearchResult{};};
 cb.on_floor_exhausted=[&](const auto &){++reports;};
 FrontierExplorerCore core(params,cb);auto m=coverageGrid();room(m,10,10,22,true);
 core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
 for(int t:{0,100,1000}) {now=t*1000000000LL;core.try_send_next_goal();}
 EXPECT_EQ(searches,0);EXPECT_EQ(reports,0);
 healthy=true;core.try_send_next_goal();EXPECT_GT(searches,0);
}

TEST(FloorCoveragePolicy, ClosedRoomWithObservedFreeInteriorCanBeExcludedButRobotRoomCannot) {
 auto m=coverageGrid();room(m,10,10,22,false);m.data[20*60+20]=0;
 FloorCoveragePolicy p;
 for(int t:{0,5,12}) {m.header.stamp.sec=t;p.observe(t,OccupancyGrid2d(m),{},0,std::make_pair(.5,.5));}
 EXPECT_TRUE(p.shouldHandoff(12));EXPECT_EQ(m.data[20*60+20],0);
 FloorCoveragePolicy own;
 for(int t:{0,5,12}) {m.header.stamp.sec=t;own.observe(t,OccupancyGrid2d(m),{},0,std::make_pair(2.05,2.05));}
 EXPECT_FALSE(own.shouldHandoff(12));EXPECT_TRUE(own.hasActionableUnknown());
}

static void expanded(FloorCoveragePolicy & p) {
 p.expand(0);p.expand(20);p.expand(40);
}
TEST(FloorResiduals, EmptyExpandedSearchCannotCompleteOpenRoom) {
 auto m=coverageGrid();room(m,10,10,22,true);const auto original=m.data;
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=181;++t) {observe(p,m,t);p.exhaustedSearch(t,true,false);}
 EXPECT_FALSE(p.residualDeferred());EXPECT_FALSE(p.shouldHandoff(181));
 EXPECT_TRUE(p.hasActionableUnknown());EXPECT_GT(p.actionable_unknown_area,0);
 EXPECT_DOUBLE_EQ(p.excluded_unknown_area,0);EXPECT_EQ(m.data,original);
}
TEST(FloorResiduals, TimeoutOrPendingValidationAloneNeverQualifies) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=250;++t) {
  observe(p,m,t);p.validationEvidence(false,24,24);p.exhaustedSearch(t,false,t<120);
 }
 EXPECT_FALSE(p.shouldHandoff(250));EXPECT_EQ(p.failedGroups(),0U);
}
TEST(FloorResiduals, CompletedBroadFansCannotCertifyAllUnknownRegions) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int i=0;i<3;++i)p.validationEvidence(true,12,12);
 for(int t=1;t<=181;++t){observe(p,m,t);p.exhaustedSearch(t,false,false);}
 EXPECT_FALSE(p.shouldHandoff(181));EXPECT_EQ(p.failedGroups(),3U);
}
TEST(FloorResiduals, IncompleteExpansionAndFreshObservationBudgetAreProtected) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);p.expand(0);
 for(int t=1;t<=250;++t){observe(p,m,t);p.exhaustedSearch(t,true,false);}
 EXPECT_FALSE(p.shouldHandoff(250));EXPECT_EQ(p.residualRounds(),0U);
}
TEST(FloorResiduals, MissingDataAndRepeatedMapStampCannotSupplySearchEvidence) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=200;++t){p.observe(t,OccupancyGrid2d(m),{},0);p.exhaustedSearch(t,true,false);}
 EXPECT_LT(p.residualRounds(),6U);EXPECT_FALSE(p.shouldHandoff(200));
 p.waitForData(200);observe(p,m,2000);EXPECT_FALSE(p.shouldHandoff(2000));
}
TEST(FloorResiduals, NewMapInformationImmediatelyRevokesResidualHandoff) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=181;++t){observe(p,m,t);p.exhaustedSearch(t,true,false);}
 ASSERT_FALSE(p.shouldHandoff(181));
 for(int i=0;i<10;++i)m.data[11*60+11+i]=0;
 observe(p,m,182);EXPECT_FALSE(p.shouldHandoff(182));EXPECT_EQ(p.residualRounds(),0U);
}
TEST(FloorResiduals, FloorChangeClearsAllPreviousFailureEvidence) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=181;++t){observe(p,m,t);p.exhaustedSearch(t,true,false);}
 ASSERT_FALSE(p.shouldHandoff(181));m.info.origin.position.z=3.;
 observe(p,m,182);EXPECT_FALSE(p.shouldHandoff(182));EXPECT_EQ(p.residualRounds(),0U);
}
TEST(FloorResiduals, CoreKeepsExploringUnresolvedAreaAfterRepeatedEmptySearches) {
 int64_t now=0;int completed=0;std::vector<std::string> reports;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 params.floor_no_gain_timeout_s=10;params.reachability_filter_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.5;p.position.y=.5;p.orientation.w=1;
   return std::optional<geometry_msgs::msg::Pose>(p);};
 cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){return FrontierSearchResult{};};
 cb.on_exploration_complete=[&](){++completed;};
 cb.on_floor_exhausted=[&](const auto & text){reports.push_back(text);};
 FrontierExplorerCore core(params,cb);auto m=coverageGrid();room(m,10,10,22,true);
 for(int t=0;t<=200;++t) {
   now=int64_t(t)*1000000000LL;m.header.stamp.sec=t;
   core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);core.try_send_next_goal();
 }
 EXPECT_TRUE(reports.empty());EXPECT_EQ(completed,0);
 EXPECT_EQ(m.data[15*60+15],-1);
}

TEST(FloorResiduals, NavigationAndMeasuredApproachProgressCannotBecomeExhaustion) {
 auto m=coverageGrid();room(m,10,10,22,true);FloorCoveragePolicy p;
 observe(p,m,0);expanded(p);
 for(int t=1;t<=200;++t) {
   p.pauseResidual(t);
   if(t%10==0){observe(p,m,t);p.exhaustedSearch(t,true,false);}
 }
 EXPECT_NEAR(p.residualIdleSeconds(),0.,1e-9);EXPECT_FALSE(p.shouldHandoff(200));
 for(int t=201;t<=400;++t){observe(p,m,t);p.exhaustedSearch(t,true,false);}
 ASSERT_FALSE(p.shouldHandoff(400));p.observationEvidence({1,1},true);
 EXPECT_FALSE(p.shouldHandoff(400));EXPECT_EQ(p.residualRounds(),0U);
}

TEST(FloorResiduals, NewlyOpenedPassageRevokesEvidenceWithoutNewUnknownCells) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=181;++t){observe(p,m,t);p.exhaustedSearch(t,true,false);}
 ASSERT_FALSE(p.shouldHandoff(181));m.data[11*60+10]=0;
 observe(p,m,182);EXPECT_FALSE(p.shouldHandoff(182));EXPECT_EQ(p.residualRounds(),0U);
}

TEST(FloorResiduals, UnsafeMeasuredRobotCannotTurnPlanningFailuresIntoFloorCompletion) {
 int64_t now=0;bool safe=false;std::vector<std::string> reports;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 params.floor_no_gain_timeout_s=10;params.reachability_filter_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.5;p.position.y=.5;p.orientation.w=1;
   return std::optional<geometry_msgs::msg::Pose>(p);};
 cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){return FrontierSearchResult{};};
 cb.residual_handoff_pose_safe=[&](){return safe;};
 cb.on_floor_exhausted=[&](const auto & text){reports.push_back(text);};
 FrontierExplorerCore core(params,cb);auto m=coverageGrid();room(m,10,10,22,true);
 for(int t=0;t<=200;++t) {
   now=int64_t(t)*1000000000LL;m.header.stamp.sec=t;
   core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);core.try_send_next_goal();
 }
 EXPECT_TRUE(reports.empty());
 safe=true;now=201000000000LL;m.header.stamp.sec=201;core.map=OccupancyGrid2d(m);core.try_send_next_goal();
 EXPECT_TRUE(reports.empty());
}

TEST(FloorResiduals, SingletonAndSmallSetFailuresAreDiagnosticOnly) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 p.validationEvidence(true,1,1);p.validationEvidence(true,2,2);p.validationEvidence(true,5,5);
 for(int t=1;t<=181;++t){observe(p,m,t);p.exhaustedSearch(t,false,false);}
 EXPECT_EQ(p.failedGroups(),3U);EXPECT_EQ(p.failedOptions(),8U);
 EXPECT_FALSE(p.shouldHandoff(181));EXPECT_TRUE(p.hasActionableUnknown());
}
TEST(FloorResiduals, PartialEmptyOrTimedOutSetsNeverCountEvenWithManyTestedPoints) {
 auto m=coverageGrid();room(m,10,10,22,true);
 FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=200;++t) {
   observe(p,m,t);p.validationEvidence(true,1,12);
   p.validationEvidence(true,0,0);p.validationEvidence(false,12,12);
   p.validationEvidence(true,13,12);p.exhaustedSearch(t,false,false);
 }
 EXPECT_EQ(p.failedGroups(),0U);EXPECT_FALSE(p.shouldHandoff(200));
}
TEST(FloorResiduals, SingletonRepetitionsStillNeedTimeAndFullFloorSearchEvidence) {
 auto m=coverageGrid();room(m,10,10,22,true);FloorCoveragePolicy p;
 observe(p,m,0);expanded(p);
 for(int i=0;i<10;++i)p.validationEvidence(true,1,1);
 for(int t=1;t<=181;++t)observe(p,m,t);
 EXPECT_FALSE(p.shouldHandoff(181));EXPECT_EQ(p.residualRounds(),0U);
}
TEST(FloorCoveragePolicy, ClosedReferenceExcludedOnlyAfterStableTopologyAndRevokedOnDoorOpening) {
 auto m=coverageGrid();room(m,10,10,22,false);FloorCoveragePolicy p;
 observe(p,m,0);EXPECT_FALSE(p.closedReference(OccupancyGrid2d(m),{2.05,2.05}));
 observe(p,m,5);observe(p,m,12);
 EXPECT_TRUE(p.closedReference(OccupancyGrid2d(m),{2.05,2.05}));
 EXPECT_FALSE(p.closedReference(OccupancyGrid2d(m),{.5,.5}));
 m.data[21*60+10]=0;observe(p,m,13);
 EXPECT_FALSE(p.closedReference(OccupancyGrid2d(m),{2.05,2.05}));
}
TEST(FloorCoveragePolicy, NearbyActionableUnknownPreventsOverbroadClosedReferenceFilter) {
 auto m=coverageGrid();room(m,10,10,8,false);m.data[13*60+9]=-1;
 FloorCoveragePolicy p;for(int t:{0,5,12})observe(p,m,t);
 EXPECT_FALSE(p.closedReference(OccupancyGrid2d(m),{1.25,1.35}));
 EXPECT_TRUE(p.hasActionableUnknown());
}
TEST(FloorResiduals, SchedulerDoesNotRebindAReleasedSealedFrontier) {
 int64_t now=0;std::vector<FrontierLike> published;
 FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
 params.region_observation_tasks_enabled=false;params.reachability_filter_enabled=false;
 FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
 cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.5;p.position.y=.5;p.orientation.w=1;
   return std::optional<geometry_msgs::msg::Pose>(p);};
 cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){
   FrontierSearchResult result;result.frontiers={FrontierCandidate{{2.05,2.05},{2.05,2.05},5}};
   result.robot_map_cell={5,5};return result;};
 cb.publish_frontier_markers=[&](const auto & fs){published=fs;};
 FrontierExplorerCore core(params,cb);auto m=coverageGrid();room(m,10,10,22,false);
 for(int y=10;y<20;++y)m.data[y*60]=-1; // Separate open edge residual remains.
 for(int t:{0,5,12,15}) {
   if(t==15)core.pending_frontier_sequence={FrontierCandidate{{2.05,2.05},{2.05,2.05},5}};
   now=int64_t(t)*1000000000LL;m.header.stamp.sec=t;
   core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);core.frontier_snapshot.reset();
   core.try_send_next_goal();
 }
 EXPECT_TRUE(published.empty());EXPECT_TRUE(core.pending_frontier_sequence.empty());
 EXPECT_FALSE(core.return_to_start_completed);
}

TEST(FloorResiduals, EndToEndSingletonFailuresAndTimeoutsBothBlockHandoff) {
 for(bool complete_set:{false,true}) {
  int64_t now=0;std::vector<std::string> reports;FrontierExplorerCore * instance=nullptr;
  std::string diagnostic;
  FrontierExplorerCoreParams params;params.floor_cleanup_enabled=true;
  params.floor_no_gain_timeout_s=10;params.reachability_filter_enabled=false;
  params.region_observation_tasks_enabled=false;
  FrontierExplorerCoreCallbacks cb;cb.now_ns=[&](){return now;};
  cb.log_info=[&](const std::string & text){diagnostic+=text+"\n";if(diagnostic.size()>10000)diagnostic.erase(0,diagnostic.size()-10000);};
  cb.log_warn=cb.log_info;
  cb.get_current_pose=[](){geometry_msgs::msg::Pose p;p.position.x=.55;p.position.y=.55;p.orientation.w=1;
    return std::optional<geometry_msgs::msg::Pose>(p);};
  cb.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool) {
    FrontierSearchResult result;
    result.frontiers={FrontierCandidate{{.05,1.05},{.55,1.05},1}};
    result.robot_map_cell={5,5};return result;
  };
  cb.validate_frontier_goal=[&](const auto &,const auto &)
    ->std::optional<geometry_msgs::msg::PoseStamped> {
    instance->report_frontier_validation({.05,1.05},false,complete_set,1,1);
    return std::nullopt;
  };
  cb.frontier_goal_validation_pending=[](){return false;};
  cb.residual_handoff_pose_safe=[](){return true;};
  cb.on_floor_exhausted=[&](const auto & report){reports.push_back(report);};
  FrontierExplorerCore core(params,cb);instance=&core;auto m=coverageGrid();
  for(int y=10;y<20;++y)m.data[y*60]=-1;
  for(int t=0;t<=220;++t) {
    now=int64_t(t)*1000000000LL;m.header.stamp.sec=t;
    core.map=OccupancyGrid2d(m);core.costmap=OccupancyGrid2d(m);
    core.try_send_next_goal();
  }
  EXPECT_TRUE(reports.empty())<<diagnostic;
  EXPECT_EQ(m.data[10*60],-1);EXPECT_FALSE(core.return_to_start_completed);
 }
}

TEST(FloorResiduals, ClosedBoxAndUnvisitedRoomsCannotBeGloballyDeferred) {
 auto m=coverageGrid();room(m,3,3,10,false);
 room(m,30,3,12,true);room(m,30,30,12,true);
 const auto original=m.data;FloorCoveragePolicy p;observe(p,m,0);expanded(p);
 for(int t=1;t<=3600;++t) {
   observe(p,m,t);p.validationEvidence(true,13,13);
   p.observationEvidence({double(t%4),double(t%5)},false);
   p.exhaustedSearch(t,true,false);
   EXPECT_FALSE(p.shouldHandoff(t));
 }
 EXPECT_GT(p.excluded_unknown_area,0.);EXPECT_GT(p.actionable_unknown_area,1.);
 EXPECT_EQ(m.data,original);
 for(int y=0;y<50;++y)for(int x=25;x<60;++x)
   if(m.data[y*60+x]<0)m.data[y*60+x]=0;
 observe(p,m,3601);EXPECT_TRUE(p.shouldHandoff(3601));
}
TEST(FloorResiduals, CurrentLogCountsCannotCertifyUnvisitedRoom) {
 auto m=coverageGrid();room(m,10,10,22,true);FloorCoveragePolicy p;
 observe(p,m,0);expanded(p);
 for(int i=0;i<17;++i)p.validationEvidence(true,13,13);
 p.validationEvidence(true,9,9);
 for(int t=1;t<=200;++t){observe(p,m,t);p.exhaustedSearch(t,false,false);}
 EXPECT_EQ(p.failedGroups(),18U);EXPECT_EQ(p.failedOptions(),230U);
 EXPECT_FALSE(p.residualDeferred());EXPECT_FALSE(p.shouldHandoff(200));
 EXPECT_TRUE(p.hasActionableUnknown());
}
