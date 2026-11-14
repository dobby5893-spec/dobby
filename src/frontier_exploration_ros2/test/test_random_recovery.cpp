#include <gtest/gtest.h>
#include "frontier_exploration_ros2/random_recovery.hpp"
using namespace frontier_exploration_ros2;
using namespace go2_surface_3d_planner;

namespace {
ObservationGrid openGrid() {
  ObservationGrid g;g.width=80;g.height=80;g.resolution=.1;g.x=-4;g.y=-4;
  g.cells.assign(6400,0);return g;
}
SurfaceSearchConfig searchConfig() {
  SurfaceSearchConfig c;c.collision_radius=.08;c.cylinder_offset=0;c.snap_radius=.1;
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);return c;
}
void addFloor(octomap::OcTree & tree,double radius=4) {
  const int cells=std::ceil(radius/.1);
  for(int x=-cells;x<cells;++x)for(int y=-cells;y<cells;++y)
    tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  tree.updateInnerOccupancy();
}
}
TEST(RecoveryPolicy, OrdinaryForwardTravelDoesNotTriggerRandomGoals) {
  RecoveryPolicy p;
  for(int i=0;i<=100;++i)p.observe(i,i*.1,0,10);
  EXPECT_FALSE(p.ready(100));
}
TEST(RecoveryPolicy, BlockedTargetsRequireThirtySecondsBeforeAnAttempt) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);
  EXPECT_FALSE(p.ready(29));EXPECT_TRUE(p.ready(30));
}
TEST(RecoveryPolicy, NormalDispatchClearsBlockedWait) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);p.normalDispatch();
  p.observe(40,0,0,10);EXPECT_FALSE(p.ready(40));
}
TEST(RecoveryPolicy, DetectsRepeatedNoGainRoundTrips) {
  RecoveryPolicy p;
  for(int i=0;i<=4;++i)p.observe(i*20,(i%2)*.7,0,10);
  EXPECT_TRUE(p.ready(80));
}
TEST(RecoveryPolicy, NewInformationClearsLoopTrigger) {
  RecoveryPolicy p;
  for(int i=0;i<=4;++i)p.observe(i*20,(i%2)*.7,0,10);
  p.observe(81,0,0,10.2);EXPECT_FALSE(p.ready(81));
}
TEST(RecoveryPolicy, StationaryEstimatorJitterIsNotACompletedLoop) {
  RecoveryPolicy p;
  for(int i=0;i<=1000;++i)p.observe(i*.1,(i%2)*.002,0,10);
  EXPECT_FALSE(p.ready(100));
}
TEST(RecoveryPolicy, CooldownAndThreeAttemptBatchAreBounded) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);
  EXPECT_TRUE(p.ready(30));p.begin(30);
  EXPECT_FALSE(p.ready(59));EXPECT_TRUE(p.ready(60));p.begin(60);
  EXPECT_TRUE(p.ready(90));p.begin(90);
  EXPECT_FALSE(p.ready(120));EXPECT_FALSE(p.ready(209));EXPECT_TRUE(p.ready(210));
}
TEST(RecoveryPolicy, BackwardSimulationClockClearsOldEpisode) {
  RecoveryPolicy p;p.observe(100,0,0,10);p.blocked(100);p.begin(140);
  p.observe(0,0,0,10);EXPECT_FALSE(p.ready(0));EXPECT_EQ(p.attempts,0);
}
TEST(RandomRecovery, PointIsHalfToOneAndAHalfMetersAwayAndHasVerifiedRoute) {
  octomap::OcTree tree(.1);addFloor(tree);auto c=searchConfig();
  auto selected=selectRandomRecovery(tree,c,{0,0,0,0},openGrid(),{}, {},42);
  ASSERT_TRUE(selected);
  const double distance=std::hypot(selected->x,selected->y);
  EXPECT_GE(distance,.5);EXPECT_LE(distance,1.5);
  EXPECT_FALSE(SurfaceGraph(tree,c).plan({0,0,0,0},*selected).empty());
}
TEST(RandomRecovery, RecentUnproductivePointsAreAvoidedAndSecondRingIsUsed) {
  octomap::OcTree tree(.1);addFloor(tree);auto c=searchConfig();
  std::vector<SurfacePoint> avoid;
  for(double x=-1.5;x<=1.5;x+=.25)for(double y=-1.5;y<=1.5;y+=.25)
    if(std::hypot(x,y)<=1.5)avoid.push_back({x,y,0,0});
  auto selected=selectRandomRecovery(tree,c,{0,0,0,0},openGrid(),{},avoid,9);
  ASSERT_TRUE(selected);
  EXPECT_GE(std::hypot(selected->x,selected->y),1.5);
  EXPECT_LE(std::hypot(selected->x,selected->y),3);
  for(const auto & p:avoid)EXPECT_GE(std::hypot(selected->x-p.x,selected->y-p.y),.5);
}
TEST(RandomRecovery, FreeTwoDimensionalEndpointsCannotCrossAnEnclosingWall) {
  octomap::OcTree tree(.1);addFloor(tree);
  for(int x=-3;x<=3;++x)for(int y=-3;y<=3;++y) {
    if(std::abs(x)!=3 && std::abs(y)!=3)continue;
    for(int z=1;z<=6;++z)tree.updateNode(octomap::point3d(x*.1,y*.1,z*.1),true);
  }tree.updateInnerOccupancy();
  EXPECT_FALSE(selectRandomRecovery(tree,searchConfig(),{0,0,0,0},openGrid(),{},{},7));
}
TEST(RandomRecovery, UnknownSupportNeverBecomesASafeRecoveryTarget) {
  octomap::OcTree tree(.1);addFloor(tree,.2);
  EXPECT_FALSE(selectRandomRecovery(tree,searchConfig(),{0,0,0,0},openGrid(),{},{},12));
}
TEST(RandomRecovery, InvalidMinimumDistanceIsRejected) {
  octomap::OcTree tree(.1);addFloor(tree);RandomRecoveryConfig c;c.minimum_distance=.2;
  EXPECT_THROW(selectRandomRecovery(tree,searchConfig(),{0,0,0,0},openGrid(),c,{},1),std::invalid_argument);
}

TEST(RecoveryPolicy, KnownCellsFlickeringUnknownDoNotHideAnExistingLoop) {
  RecoveryPolicy p;
  for(int i=0;i<=4;++i)p.observe(i*20,(i%2)*.7,0,10);
  p.observe(85,0,0,8);EXPECT_TRUE(p.ready(85));
  p.observe(86,0,0,10);EXPECT_TRUE(p.ready(86));
}

#include "frontier_exploration_ros2/recovery_start.hpp"
namespace {
void narrowFixture(octomap::OcTree & tree) {
  for(int x=-40;x<40;++x)for(int y=-40;y<40;++y)
    tree.updateNode(octomap::point3d((x+.5)*.01,(y+.5)*.01,-.005),true);
  for(int y=-40;y<40;++y)for(int z=15;z<65;++z)
    tree.updateNode(octomap::point3d(.105,(y+.5)*.01,(z+.5)*.01),true);
  tree.updateInnerOccupancy();
}
SurfaceSearchConfig narrowConfig() {
  auto c=searchConfig();c.snap_radius=0;c.clearance_margin=.04;
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(10);
  return c;
}
}
TEST(RecoveryStart, PositiveGapInsideNominalMarginRequestsSpecializedRestoration) {
  octomap::OcTree tree(.01);narrowFixture(tree);
  const auto assessment=assessRecoveryStart(tree,narrowConfig(),{.015,0,0,0});
  EXPECT_EQ(assessment.state,RecoveryStartState::needs_clearance);
  EXPECT_NEAR(assessment.gap,.005,1e-5);
}
TEST(RecoveryStart, AlreadyClearDoesNotRequestMarginRestoration) {
  octomap::OcTree tree(.01);narrowFixture(tree);
  EXPECT_EQ(assessRecoveryStart(tree,narrowConfig(),{-.05,0,0,0}).state,
    RecoveryStartState::ready);
}
TEST(RecoveryStart, DistinguishesBodyContactFromMissingSupport) {
  octomap::OcTree tree(.01);narrowFixture(tree);
  auto overlap=assessRecoveryStart(tree,narrowConfig(),{.03,0,0,0});
  EXPECT_EQ(overlap.state,RecoveryStartState::contact);
  EXPECT_LT(overlap.gap,0);
  EXPECT_NE(overlap.state,RecoveryStartState::ready);
  octomap::OcTree empty(.01);
  EXPECT_EQ(assessRecoveryStart(empty,narrowConfig(),{0,0,0,0}).state,
    RecoveryStartState::unsupported);
}
TEST(ClearanceRecovery, ImprovesInSmallSweptStepsWhenFullTargetIsBeyondOneCycle) {
  octomap::OcTree tree(.01);narrowFixture(tree);auto c=narrowConfig();
  c.clearance_margin=0;SurfaceGraph graph(tree,c);SurfacePoint start{.015,0,0,0};
  ASSERT_TRUE(graph.snapToSurface(start));
  EXPECT_FALSE(graph.findClearanceRecovery(start,.05,.02,false));
  const auto end=graph.findClearanceRecovery(start,.05,.02,true);
  ASSERT_TRUE(end);EXPECT_TRUE(graph.isMotionValid(start,*end));
  EXPECT_LE(std::hypot(end->x-start.x,end->y-start.y),.020001);
  EXPECT_GT(graph.bodyClearanceAt(*end),graph.bodyClearanceAt(start));
  EXPECT_LT(graph.bodyClearanceAt(*end),.05);
  SurfacePoint contact{.03,0,0,0};
  const auto retreat=graph.findClearanceRecovery(contact,.05,.02,true);
  ASSERT_TRUE(retreat);
  EXPECT_TRUE(graph.isClearanceRecoveryMotionValid(contact,*retreat));
  EXPECT_GT(graph.bodyClearanceAt(*retreat),graph.bodyClearanceAt(contact));
}

TEST(CrossGoalStagnation, RepeatedDispatchCannotEraseMeasuredStationaryTime) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.normalDispatch();
  for(int t=1;t<=59;++t) {
    p.motion.observe(t,(t%2)*.002,0);
    if(t%10==0)p.normalDispatch();
    EXPECT_FALSE(p.ready(t));
  }
  p.motion.observe(60,.001,0);p.normalDispatch();EXPECT_TRUE(p.ready(60));
}
TEST(CrossGoalStagnation, TwoStalledGoalsAtSamePlaceTriggerWithoutOneMeterLoop) {
  RecoveryPolicy p;p.observe(0,1,2,10);p.normalDispatch();
  p.motion.failedGoal(10,1.01,2);p.normalDispatch();EXPECT_FALSE(p.ready(10));
  p.motion.failedGoal(20,1.02,2);p.normalDispatch();EXPECT_TRUE(p.ready(20));
}
TEST(CrossGoalStagnation, ActualTranslationResetsFailuresButNewGoalDoesNot) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.normalDispatch();
  p.motion.failedGoal(10,0,0);p.normalDispatch();
  p.motion.observe(15,.2,0);EXPECT_EQ(p.motion.failures,0U);
  p.motion.failedGoal(20,.21,0);EXPECT_FALSE(p.ready(20));
}
TEST(CrossGoalStagnation, IdleWithoutGoalsDoesNotArmAndBackwardClockClearsStagnation) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.observe(100,0,0,10);EXPECT_FALSE(p.ready(100));
  p.normalDispatch();p.motion.failedGoal(110,0,0);p.motion.failedGoal(120,0,0);
  EXPECT_TRUE(p.ready(120));p.observe(0,0,0,10);EXPECT_FALSE(p.ready(0));
}
TEST(CrossGoalStagnation, RecoveryAttemptsStillRespectCooldownAndBatchLimit) {
  RecoveryPolicy p;p.observe(0,0,0,10);p.normalDispatch();
  for(int t:{30,60,90}) {
    p.motion.failedGoal(t,0,0);p.motion.failedGoal(t,0,0);
    EXPECT_TRUE(p.ready(t));p.begin(t);
    EXPECT_FALSE(p.ready(t+1));
  }
  p.motion.failedGoal(120,0,0);p.motion.failedGoal(120,0,0);
  EXPECT_FALSE(p.ready(120));EXPECT_FALSE(p.ready(209));EXPECT_TRUE(p.ready(210));
}

TEST(CoarseStartDeparture, VerifiedRawFreeStartCanLeaveDiagonalInflation) {
  ObservationGrid grid;grid.width=12;grid.height=12;grid.resolution=.1;
  grid.cells.assign(144,0);grid.cells[6*12+6]=100;
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  SurfacePoint start{.45,.45,0,0},goal{.45,.15,0,0};
  EXPECT_TRUE(mappedObservationRoute(grid,start,goal,.22,deadline).empty());
  auto route=mappedObservationRoute(grid,start,goal,.22,deadline,true);
  ASSERT_FALSE(route.empty());
  EXPECT_NEAR(route.front().x,.45,1e-9);
  EXPECT_NEAR(route.back().y,.15,1e-9);
}
TEST(CoarseStartDeparture, OccupiedAndUnknownMeasuredCellsRemainForbidden) {
  for(int value:{-1,100}) {
    ObservationGrid grid;grid.width=12;grid.height=12;grid.resolution=.1;
    grid.cells.assign(144,0);grid.cells[4*12+4]=value;
    EXPECT_TRUE(mappedObservationRoute(grid,{.45,.45,0,0},{.45,.15,0,0},.22,
      std::chrono::steady_clock::now()+std::chrono::seconds(1),true).empty());
  }
}
TEST(CoarseStartDeparture, DoesNotUnblockEndpointOrCrossSolidWall) {
  ObservationGrid grid;grid.width=12;grid.height=12;grid.resolution=.1;
  grid.cells.assign(144,0);grid.cells[6*12+6]=100;
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  EXPECT_TRUE(mappedObservationRoute(grid,{.45,.45,0,0},{.65,.45,0,0},.22,
    deadline,true).empty());
  for(int x=0;x<12;++x)grid.cells[2*12+x]=100;
  EXPECT_TRUE(mappedObservationRoute(grid,{.45,.45,0,0},{.45,.05,0,0},.22,
    deadline,true).empty());
}
TEST(CoarseStartDeparture, PhysicalBodyContactStillRejectsBothSelectors) {
  octomap::OcTree tree(.1);addFloor(tree);
  tree.updateNode(octomap::point3d(.05,.05,.25),true);tree.updateInnerOccupancy();
  auto c=searchConfig();auto grid=openGrid();
  EXPECT_FALSE(selectMappedObservation(tree,c,{0,0,0,0},{1,0,0,0},grid));
  EXPECT_FALSE(selectRandomRecovery(tree,c,{0,0,0,0},grid,{}, {},42));
}

TEST(RecoveryPolicy, ExhaustionCoolsThenRetriesWithoutAnyMapGain) {
 RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);
 for(int t:{30,60,90}) {ASSERT_TRUE(p.ready(t));p.begin(t);}
 for(int t:{120,180,209}) {p.observe(t,0,0,10);p.normalDispatch();EXPECT_FALSE(p.ready(t));}
 p.observe(210,0,0,10);p.motion.failedGoal(210,0,0);p.motion.failedGoal(210,0,0);
 EXPECT_TRUE(p.ready(210));EXPECT_EQ(p.attempts,0);EXPECT_EQ(p.batches,1U);
 p.begin(210);EXPECT_EQ(p.attempts,1);EXPECT_FALSE(p.ready(211));
}
TEST(RecoveryPolicy, MapGainCannotEraseFailedAttemptsOrBypassBatchCooldown) {
 RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);
 for(int t:{30,60,90}) {ASSERT_TRUE(p.ready(t));p.begin(t);}
 p.observe(100,0,0,20);EXPECT_EQ(p.attempts,3);EXPECT_FALSE(p.ready(120));
}
TEST(RecoveryPolicy, OnlyMeasuredDisplacementAndSafeStanceClearEpisode) {
 RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);p.begin(30);
 EXPECT_FALSE(p.recovered(31,.01,0,true));
 EXPECT_FALSE(p.recovered(32,.6,0,false));EXPECT_EQ(p.attempts,1);
 EXPECT_TRUE(p.recovered(33,.6,0,true));EXPECT_EQ(p.attempts,0);
 EXPECT_FALSE(p.episode);EXPECT_FALSE(p.ready(33));EXPECT_FALSE(p.ready(100));
}
TEST(RecoveryPolicy, ReturnToStuckOriginDoesNotCountAsEscape) {
 RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);p.begin(30);
 p.observe(31,.6,0,10);p.observe(32,.01,0,10);
 EXPECT_FALSE(p.recovered(32,.01,0,true));EXPECT_EQ(p.attempts,1);
}
TEST(RecoveryPolicy, MultipleBatchesRemainBoundedWithoutPermanentExhaustion) {
 RecoveryPolicy p;p.observe(0,0,0,10);p.blocked(0);
 for(int batch=0;batch<4;++batch) {
   const double first=30+batch*180;
   for(int j=0;j<3;++j) {
     const double t=first+j*30;
     p.observe(t,0,0,10);p.blocked(t);ASSERT_TRUE(p.ready(t));p.begin(t);
     EXPECT_FALSE(p.ready(t+1));
   }
   EXPECT_FALSE(p.ready(first+179));
 }
 EXPECT_EQ(p.batches,3U);
}
TEST(KnownMapProgress, FlickerAndMapExpansionDoNotInventNewInformation) {
 nav_msgs::msg::OccupancyGrid m;m.info.width=4;m.info.height=1;
 m.info.resolution=.1;m.info.origin.orientation.w=1;m.data={0,0,-1,-1};
 KnownMapProgress progress;const auto first=progress.observe(m,0);
 m.data={-1,-1,-1,-1};EXPECT_DOUBLE_EQ(progress.observe(m,1),first);
 m.data={0,0,-1,-1};EXPECT_DOUBLE_EQ(progress.observe(m,2),first);
 m.info.origin.position.x=-.1;m.info.width=5;m.data={-1,0,0,-1,-1};
 EXPECT_DOUBLE_EQ(progress.observe(m,3),first);
 m.data[3]=0;EXPECT_GT(progress.observe(m,4),first);
 m.info.origin.position.z=3.;EXPECT_NEAR(progress.observe(m,5),.03,1e-6);
}


TEST(RandomRecovery, InsetBandStartFindsOnlyInwardVerifiedTargets) {
 octomap::OcTree tree(.1);addFloor(tree);auto c=searchConfig();
 const ExplorationBoundaryPolygon b{{-4,-4},{4,-4},{4,.3},{-4,.3}};
 auto goal=selectRandomRecovery(tree,c,{0,.2,0,0},openGrid(),{}, {},42,b,.3);
 ASSERT_TRUE(goal);EXPECT_LE(goal->y,1e-8);
 auto path=SurfaceGraph(tree,c).plan({0,.2,0,0},*goal);
 ASSERT_FALSE(path.empty());EXPECT_TRUE(boundary_return_path(SurfacePoint{0,.2,0,0},path,b,.3));
}

#include "frontier_exploration_ros2/visited_open_return.hpp"
TEST(VisitedOpenReturn, RecordsOnlyWideMeasuredPositionsAndUsesRecency) {
 VisitedOpenHistory h;
 h.observe(0,0,{0,0,0,0},.03);EXPECT_EQ(h.size(),0U);
 h.observe(1,0,{0,0,0,0},.10);
 h.observe(2,0,{1,0,0,0},.15);
 h.observe(3,0,{1.1,0,0,0},.15);EXPECT_EQ(h.size(),2U);
 auto choices=h.candidates({2,0,0,0});ASSERT_EQ(choices.size(),2U);
 EXPECT_DOUBLE_EQ(choices[0].x,1);
 h.sync(4,3);EXPECT_EQ(h.size(),0U);
 h.observe(5,3,{0,0,3,0},.15);h.sync(0,3);EXPECT_EQ(h.size(),0U);
}
TEST(VisitedOpenReturn, CanReturnThroughNarrowPassageWithoutFirstRestoringNominalMargin) {
 octomap::OcTree tree(.05);
 for(int x=-20;x<30;++x)for(int y=-12;y<12;++y)
   tree.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,-.025),true);
 for(int x=0;x<15;++x)for(int z=1;z<10;++z)
   for(double y:{-.125,.125})
     tree.updateNode(octomap::point3d((x+.5)*.05,y,(z+.5)*.05),true);
 tree.updateInnerOccupancy();
 auto c=searchConfig();c.clearance_margin=.04;c.narrow_clearance_margin=.01;
 const SurfacePoint current{.35,0,0,0},old_open{-.65,0,0,0};
 auto hard=c;hard.snap_radius=0;hard.clearance_margin=.01;
 auto actual=SurfaceGraph(tree,hard).snapToSurface(current);ASSERT_TRUE(actual);
 EXPECT_LT(SurfaceGraph(tree,hard).bodyClearanceAt(*actual),.04);
 auto target=selectVisitedOpenReturn(tree,c,current,{old_open});
 ASSERT_TRUE(target);EXPECT_NEAR(target->x,old_open.x,1e-9);
 auto route=SurfaceGraph(tree,hard).plan(*actual,*target);ASSERT_GT(route.size(),1U);
 for(size_t i=1;i<route.size();++i)EXPECT_TRUE(SurfaceGraph(tree,hard).isMotionValid(route[i-1],route[i]));
}
TEST(VisitedOpenReturn, ChangedOrTooCloseHistoricalEndpointIsRejected) {
 octomap::OcTree tree(.1);addFloor(tree);
 auto c=searchConfig();const SurfacePoint current{0,0,0,0};
 EXPECT_FALSE(selectVisitedOpenReturn(tree,c,current,{{.2,0,0,0}}));
 for(int y=-4;y<=4;++y)for(int z=1;z<5;++z)
   tree.updateNode(octomap::point3d(1.,y*.1,z*.1),true);
 tree.updateInnerOccupancy();
 EXPECT_FALSE(selectVisitedOpenReturn(tree,c,current,{{1.,0,0,0}}));
}
TEST(VisitedOpenReturn, BlockedStartCannotUseHistoricalAnchorToTeleport) {
 octomap::OcTree tree(.1);
 // A safe endpoint exists, but no observed support connects the actual start.
 for(int x=8;x<15;++x)for(int y=-4;y<4;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 tree.updateInnerOccupancy();auto c=searchConfig();c.snap_radius=.6;
 EXPECT_FALSE(selectVisitedOpenReturn(tree,c,{0,0,0,0},{{1.,0,0,0}}));
}

TEST(VisitedOpenReturn, UnavailableNewestAreaFallsBackToOlderVerifiedArea) {
 octomap::OcTree tree(.1);addFloor(tree);
 for(int y=-4;y<=4;++y)for(int z=1;z<5;++z)
   tree.updateNode(octomap::point3d(1.,y*.1,z*.1),true);
 tree.updateInnerOccupancy();auto c=searchConfig();
 auto target=selectVisitedOpenReturn(tree,c,{0,0,0,0},{{1.,0,0,0},{-1.,0,0,0}});
 ASSERT_TRUE(target);EXPECT_NEAR(target->x,-1.,1e-9);
}
TEST(VisitedOpenReturn, MissionBoundaryStillAppliesToHistoricalAreas) {
 octomap::OcTree tree(.1);addFloor(tree);auto c=searchConfig();
 ExplorationBoundaryPolygon boundary{{-1.,-1.},{1.,-1.},{1.,1.},{-1.,1.}};
 EXPECT_FALSE(selectVisitedOpenReturn(tree,c,{0,0,0,0},{{2.,0,0,0}},boundary,.15));
}

TEST(VisitedOpenReturn, NarrowPassageBetweenCoarseGridLinesStillHasVerifiedReturn) {
 octomap::OcTree tree(.05);
 for(int x=-20;x<30;++x)for(int y=-12;y<12;++y)
   tree.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,-.025),true);
 for(int x=0;x<15;++x)for(int z=1;z<10;++z)
   for(double y:{-.075,.175})
     tree.updateNode(octomap::point3d((x+.5)*.05,y,(z+.5)*.05),true);
 tree.updateInnerOccupancy();
 auto c=searchConfig();c.clearance_margin=.04;c.narrow_clearance_margin=.01;
 const SurfacePoint current{.35,.05,0,0},old_open{-.65,.05,0,0};
 auto hard=c;hard.snap_radius=0;hard.clearance_margin=.01;
 auto actual=SurfaceGraph(tree,hard).snapToSurface(current);ASSERT_TRUE(actual);
 EXPECT_LT(SurfaceGraph(tree,hard).bodyClearanceAt(*actual),.04);
 auto target=selectVisitedOpenReturn(tree,c,current,{old_open});
 ASSERT_TRUE(target);EXPECT_NEAR(target->x,old_open.x,1e-9);
 EXPECT_TRUE(SurfaceGraph(tree,hard).plan(*actual,*target).empty());
 hard.xy_resolution=tree.getResolution()*.5;
 auto route=SurfaceGraph(tree,hard).plan(*actual,*target);ASSERT_GT(route.size(),1U);
 for(size_t i=1;i<route.size();++i)EXPECT_TRUE(SurfaceGraph(tree,hard).isMotionValid(route[i-1],route[i]));
}

TEST(VisitedOpenReturn, BoundaryReentryUsesObservedSupportedInteriorWithoutOutsideExploration) {
 octomap::OcTree tree(.1);addFloor(tree);auto c=searchConfig();
 ExplorationBoundaryPolygon b{{-2,-2},{2,-2},{2,2},{-2,2}};
 const SurfacePoint actual{0,2.42,0,0},interior{0,1.4,0,0};
 EXPECT_FALSE(selectVisitedOpenReturn(tree,c,actual,{interior},b,.35));
 auto end=selectVisitedOpenReturn(tree,c,actual,{interior},b,.35,true);
 ASSERT_TRUE(end);EXPECT_NEAR(end->y,1.4,1e-9);
 EXPECT_FALSE(selectVisitedOpenReturn(tree,c,{0,2.8,0,0},{interior},b,.35,true));
}


TEST(VisitedOpenReturn, ReentryDoesNotRoundActualStartFartherOutside) {
 octomap::OcTree tree(.1);addFloor(tree);auto c=searchConfig();
 ExplorationBoundaryPolygon b{{-2,-2},{2,-2},{2,2},{-2,2}};
 auto end=selectVisitedOpenReturn(tree,c,{0,2.43,0,0},{{0,1.4,0,0}},b,.35,true);
 ASSERT_TRUE(end);EXPECT_NEAR(end->y,1.4,1e-9);
}
