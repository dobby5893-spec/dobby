#include <gtest/gtest.h>
#include "frontier_exploration_ros2/nearby_observation.hpp"
#include "frontier_exploration_ros2/mapped_observation.hpp"
#include "frontier_exploration_ros2/viewpoint_visibility_3d.hpp"

namespace {
using namespace go2_surface_3d_planner;
SurfaceSearchConfig config()
{
  SurfaceSearchConfig c;
  c.collision_radius=0.08; c.cylinder_offset=0; c.snap_radius=0.15;
  c.snap_vertical_range=0.55; c.search_padding_xy=0.3;
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  return c;
}
void floor(octomap::OcTree & tree)
{
  for (int x=-10;x<=20;++x) {
    for (int y=-10;y<=10;++y) {tree.updateNode(octomap::point3d(x*.1,y*.1,-.05),true);}
  }
  tree.updateInnerOccupancy();
}
TEST(NearbyObservation, CanApproachWhenDistantAreaHasNoObservedSupport)
{
  octomap::OcTree tree(.1); floor(tree); auto c=config();
  SurfacePoint start{0,0,.2,0}, far{4,0,0,0};
  auto selected=frontier_exploration_ros2::selectNearbyObservation(tree,c,start,far,12);
  ASSERT_TRUE(selected);
  EXPECT_GE(std::hypot(selected->x,selected->y),.5);
  EXPECT_LE(std::hypot(selected->x,selected->y),1.4);
  EXPECT_FALSE(SurfaceGraph(tree,c).plan(start,*selected).empty());
}
TEST(NearbyObservation, CannotCrossEnclosingWallEvenWithFreeCandidateEndpoints)
{
  octomap::OcTree tree(.1); floor(tree);
  for (int x=-5;x<=5;++x) {
    for (int y=-5;y<=5;++y) {
      if (std::abs(x)!=4 && std::abs(y)!=4) {continue;}
      for (int z=1;z<=6;++z) {tree.updateNode(octomap::point3d(x*.1,y*.1,z*.1),true);}
    }
  }
  tree.updateInnerOccupancy();
  EXPECT_FALSE(frontier_exploration_ros2::selectNearbyObservation(
    tree,config(),{0,0,.2,0},{4,0,0,0},12));
}
TEST(NearbyObservation, DoesNotBypassRequestedRetreatLimit)
{
  octomap::OcTree tree(.1); floor(tree);
  EXPECT_FALSE(frontier_exploration_ros2::selectNearbyObservation(
    tree,config(),{0,0,.2,0},{4,0,0,0},.2));
}
}

TEST(NearbyObservation, LateralApproachEscapesCornerWithoutCrossingWalls)
{
  octomap::OcTree tree(.1); floor(tree); auto c=config();
  for (double x : {-0.25, 0.25}) {
    for (int y=-10;y<=10;++y) {
      for (int z=1;z<=6;++z) {
        tree.updateNode(octomap::point3d(x,y*.1,z*.1),true);
      }
    }
  }
  tree.updateInnerOccupancy();
  SurfacePoint start{0,0,0,0}, requested{4,1,0,0};
  auto selected=frontier_exploration_ros2::selectNearbyObservation(
    tree,c,start,requested,12);
  ASSERT_TRUE(selected);
  EXPECT_LT(std::abs(selected->x),.12);
  EXPECT_GT(selected->y,.5);
  EXPECT_LT(std::hypot(selected->x-requested.x,selected->y-requested.y),
    std::hypot(start.x-requested.x,start.y-requested.y));
  EXPECT_TRUE(SurfaceGraph(tree,c).isMotionValid(start,*selected));
}

#include "frontier_exploration_ros2/mapped_observation.hpp"
#include "frontier_exploration_ros2/viewpoint_visibility_3d.hpp"

TEST(MappedObservation, FollowsDoorDetourInsteadOfReturningTowardRoomCorner)
{
  using namespace frontier_exploration_ros2;
  ObservationGrid grid;grid.width=40;grid.height=40;grid.resolution=.1;
  grid.cells.assign(1600,0);
  for(int y=0;y<30;++y) grid.cells[y*40+20]=100;
  auto route=mappedObservationRoute(grid,{1,1,0,0},{3,1,0,0},.1,
    std::chrono::steady_clock::now()+std::chrono::seconds(1));
  ASSERT_FALSE(route.empty());
  double maximum_y=0;
  for(const auto & p:route) maximum_y=std::max(maximum_y,p.y);
  EXPECT_GT(maximum_y,3.1);
  EXPECT_NEAR(route.back().x,3.05,.01);
  EXPECT_NEAR(route.back().y,1.05,.01);
}
TEST(MappedObservation, UnknownBarrierDoesNotProduceAnUnverifiedShortcut)
{
  using namespace frontier_exploration_ros2;
  ObservationGrid grid;grid.width=40;grid.height=40;grid.resolution=.1;
  grid.cells.assign(1600,0);
  for(int y=0;y<40;++y) grid.cells[y*40+20]=-1;
  EXPECT_TRUE(mappedObservationRoute(grid,{1,1,0,0},{3,1,0,0},.1,
    std::chrono::steady_clock::now()+std::chrono::seconds(1)).empty());
}
TEST(MappedObservation, DiagonalDoesNotCutAcrossTwoBlockedCorners)
{
  using namespace frontier_exploration_ros2;
  ObservationGrid grid;grid.width=2;grid.height=2;grid.resolution=1;
  grid.cells={0,100,100,0};
  EXPECT_TRUE(mappedObservationRoute(grid,{.5,.5,0,0},{1.5,1.5,0,0},0,
    std::chrono::steady_clock::now()+std::chrono::seconds(1)).empty());
}
TEST(MappedObservation, TwoDimensionalRouteCannotBypassThreeDimensionalWall)
{
  using namespace frontier_exploration_ros2;
  octomap::OcTree tree(.1);floor(tree);auto c=config();
  for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y) {
    if(std::abs(x)!=4 && std::abs(y)!=4)continue;
    for(int z=1;z<=6;++z)tree.updateNode(octomap::point3d(x*.1,y*.1,z*.1),true);
  }
  tree.updateInnerOccupancy();
  ObservationGrid grid;grid.width=30;grid.height=20;grid.resolution=.1;
  grid.x=-1;grid.y=-1;grid.cells.assign(600,0);
  EXPECT_FALSE(selectMappedObservation(tree,c,{0,0,0,0},{1.5,0,0,0},grid));
}

TEST(MappedContinuation, SkipsCooledEndpointButMayTraverseIt) {
  using namespace frontier_exploration_ros2;
  octomap::OcTree tree(.1);
  for(int x=-10;x<=40;++x)for(int y=-20;y<=20;++y)
    tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  tree.updateInnerOccupancy();
  ObservationGrid grid;grid.width=60;grid.height=40;grid.resolution=.1;
  grid.x=-1;grid.y=-2;grid.cells.assign(2400,0);
  auto c=config();c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  auto first=selectMappedObservation(tree,c,{0,0,0,0},{1,0,0,0},grid);
  ASSERT_TRUE(first);
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  auto next=selectMappedObservation(tree,c,{0,0,0,0},{3,0,0,0},grid,
    {{first->x,first->y}});
  ASSERT_TRUE(next);
  EXPECT_GT(std::hypot(next->x-first->x,next->y-first->y),.60);
  EXPECT_GT(next->x,first->x);
  auto path=SurfaceGraph(tree,c).plan({0,0,0,0},*next);
  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(std::any_of(path.begin(),path.end(),[&](const auto & point){
    return std::hypot(point.x-first->x,point.y-first->y)<.30;}));
}
TEST(MappedContinuation, AllCooledEndpointsNeverReturnARepeatedObservation) {
  using namespace frontier_exploration_ros2;
  octomap::OcTree tree(.1);
  for(int x=-10;x<=40;++x)for(int y=-20;y<=20;++y)
    tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  tree.updateInnerOccupancy();
  ObservationGrid grid;grid.width=60;grid.height=40;grid.resolution=.1;
  grid.x=-1;grid.y=-2;grid.cells.assign(2400,0);
  auto c=config();c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
  EXPECT_FALSE(selectMappedObservation(tree,c,{0,0,0,0},{3,0,0,0},grid,
    {{.5,0},{1.,0},{1.5,0},{2.,0},{2.5,0},{3.,0}}));
}

TEST(ViewpointVisibility3D, OccupiedFaceOccludesUnknownColumnAtEveryVisibleHeight) {
 using namespace frontier_exploration_ros2;
 ObservationGrid grid;grid.width=50;grid.height=30;grid.resolution=.1;
 grid.x=-1;grid.y=-1;grid.cells.assign(1500,0);grid.cells[10*50+30]=-1;
 VisibilitySensor sensor;sensor.z=.3;octomap::OcTree tree(.1);
 auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
 EXPECT_TRUE(visibleUnknown3D(tree,grid,{0,0,0,0},{2.05,.05},sensor,deadline));
 for(int y=-20;y<=20;++y)for(int z=-1;z<=20;++z)
   tree.updateNode(octomap::point3d(1.05,(y+.5)*.1,(z+.5)*.1),true);
 tree.updateInnerOccupancy();
 EXPECT_FALSE(visibleUnknown3D(tree,grid,{0,0,0,0},{2.05,.05},sensor,deadline));
}
TEST(ViewpointVisibility3D, RejectsTargetsOutsideSensorVerticalFieldOfView) {
 using namespace frontier_exploration_ros2;
 ObservationGrid grid;grid.width=50;grid.height=30;grid.resolution=.1;
 grid.x=-1;grid.y=-1;grid.cells.assign(1500,0);grid.cells[10*50+30]=-1;
 VisibilitySensor sensor;sensor.z=.3;sensor.min_pitch=.8;sensor.max_pitch=.9;
 octomap::OcTree tree(.1);
 EXPECT_FALSE(visibleUnknown3D(tree,grid,{0,0,0,0},{2.05,.05},sensor,
   std::chrono::steady_clock::now()+std::chrono::seconds(2)));
}

TEST(MappedLongAdvance, KnownOpenRoutePrefersFiveMetersWithoutOvershoot) {
 using namespace frontier_exploration_ros2;
 ObservationGrid grid;grid.width=100;grid.height=40;grid.resolution=.1;
 grid.x=-1;grid.y=-2;grid.cells.assign(4000,0);
 std::vector<SurfacePoint> route;
 for(int i=0;i<=70;++i)route.push_back({i*.1,0,0,0});
 auto targets=mappedAdvanceTargets(route,grid,{0,0,0,0},.25,5.);
 ASSERT_FALSE(targets.empty());EXPECT_NEAR(route[targets.front()].x,5.,1e-6);
 for(auto i:targets)EXPECT_LE(route[i].x,5.+1e-6);
}
TEST(MappedLongAdvance, DoorAndUnknownBoundaryLimitAdvanceBeforeConstriction) {
 using namespace frontier_exploration_ros2;
 ObservationGrid grid;grid.width=100;grid.height=40;grid.resolution=.1;
 grid.x=-1;grid.y=-2;grid.cells.assign(4000,0);
 std::vector<SurfacePoint> route;for(int i=0;i<=70;++i)route.push_back({i*.1,0,0,0});
 for(int x=30;x<35;++x)grid.cells[22*100+x]=100;
 auto targets=mappedAdvanceTargets(route,grid,{0,0,0,0},.25,5.);
 ASSERT_FALSE(targets.empty());EXPECT_LT(route[targets.front()].x,2.);
 for(int x=30;x<35;++x)grid.cells[22*100+x]=-1;
 targets=mappedAdvanceTargets(route,grid,{0,0,0,0},.25,5.);
 ASSERT_FALSE(targets.empty());EXPECT_LT(route[targets.front()].x,2.);
}
TEST(MappedLongAdvance, CornerRetainsStagedApproach) {
 using namespace frontier_exploration_ros2;
 ObservationGrid grid;grid.width=100;grid.height=100;grid.resolution=.1;
 grid.x=-1;grid.y=-2;grid.cells.assign(10000,0);
 std::vector<SurfacePoint> route;
 for(int i=0;i<=20;++i)route.push_back({i*.1,0,0,0});
 for(int i=1;i<=30;++i)route.push_back({2.,i*.1,0,0});
 auto targets=mappedAdvanceTargets(route,grid,{0,0,0,0},.25,5.);
 ASSERT_FALSE(targets.empty());EXPECT_LE(route[targets.front()].x,1.5);
 EXPECT_NEAR(route[targets.front()].y,0,1e-6);
}

TEST(MappedLongAdvance, FiveMeterCandidateRequiresActualThreeDimensionalVerifiedRoute) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);
 for(int x=-15;x<=75;++x)for(int y=-20;y<=20;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 tree.updateInnerOccupancy();
 ObservationGrid grid;grid.width=100;grid.height=40;grid.resolution=.1;
 grid.x=-1;grid.y=-2;grid.cells.assign(4000,0);
 auto c=config();c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{7,0,0,0},grid);
 ASSERT_TRUE(selected);EXPECT_GT(selected->x,4.5);EXPECT_LE(selected->x,5.);
 auto path=SurfaceGraph(tree,c).plan({0,0,0,0},*selected);
 ASSERT_FALSE(path.empty());
 double length=0;SurfacePoint previous{0,0,0,0};
 for(auto p:path) {length+=std::hypot(p.x-previous.x,p.y-previous.y);previous=p;}
 EXPECT_LE(length,5.+1e-6);
}

TEST(MappedObservationBudget, BlockedLongSearchCannotConsumeShortDepartureBudget) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);
 for(int x=-15;x<=85;++x)for(int y=-40;y<=40;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 for(int y=-50;y<=50;++y)for(int z=1;z<=12;++z)
   tree.updateNode(octomap::point3d(.95,(y+.5)*.1,(z+.5)*.1),true);
 tree.updateInnerOccupancy();
 ObservationGrid grid;grid.width=100;grid.height=80;grid.resolution=.1;
 grid.x=-1;grid.y=-4;grid.cells.assign(8000,0); // 2D projection missed this 3D wall.
 auto c=config();c.search_padding_xy=3.;c.max_iterations=500000;
 const auto began=std::chrono::steady_clock::now();
 c.search_deadline=began+std::chrono::milliseconds(400);
 bool timed_out=false;
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{7,0,0,0},grid,{},.25,5.,&timed_out);
 ASSERT_TRUE(selected);
 EXPECT_LT(selected->x,.8);EXPECT_GE(std::hypot(selected->x,selected->y),.25);
 EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count(),.65);
 c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
 EXPECT_FALSE(SurfaceGraph(tree,c).plan({0,0,0,0},*selected).empty());
}
TEST(MappedObservationBudget, ShortFallbackStillRespectsMinimumAdvance) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);
 for(int x=-15;x<=85;++x)for(int y=-30;y<=30;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 for(int y=-40;y<=40;++y)for(int z=1;z<=12;++z)
   tree.updateNode(octomap::point3d(.95,(y+.5)*.1,(z+.5)*.1),true);
 tree.updateInnerOccupancy();
 ObservationGrid grid;grid.width=100;grid.height=60;grid.resolution=.1;
 grid.x=-1;grid.y=-3;grid.cells.assign(6000,0);
 auto c=config();c.search_padding_xy=3.;
 c.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(400);
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{7,0,0,0},grid,{},.5,5.);
 ASSERT_TRUE(selected);EXPECT_GE(std::hypot(selected->x,selected->y),.5);EXPECT_LT(selected->x,.8);
}
TEST(MappedObservationBudget, OpenFiveMeterLegIsStillPreferredWithinAViewBudget) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);
 for(int x=-15;x<=85;++x)for(int y=-20;y<=20;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 tree.updateInnerOccupancy();
 ObservationGrid grid;grid.width=100;grid.height=40;grid.resolution=.1;
 grid.x=-1;grid.y=-2;grid.cells.assign(4000,0);
 auto c=config();c.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(400);
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{7,0,0,0},grid);
 ASSERT_TRUE(selected);EXPECT_GT(selected->x,4.5);EXPECT_LE(selected->x,5.);
}


TEST(MappedObservationBudget, RuntimeViewBudgetKeepsLongStrideWithinMissionBoundary) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);
 for(int x=-15;x<=85;++x)for(int y=-20;y<=20;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 tree.updateInnerOccupancy();
 ObservationGrid grid;grid.width=100;grid.height=40;grid.resolution=.1;
 grid.x=-1;grid.y=-2;grid.cells.assign(4000,0);
 const ExplorationBoundaryPolygon boundary{{-1,-2},{8,-2},{8,2},{-1,2}};
 auto c=config();c.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(600);
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{7,0,0,0},grid,{},.5,5.,nullptr,boundary,.15);
 ASSERT_TRUE(selected);EXPECT_GT(selected->x,4.5);EXPECT_LE(selected->x,5.);
 EXPECT_TRUE(is_inside_exploration_boundary({selected->x,selected->y},boundary,.15));
}

TEST(MappedContinuation, NarrowStartCanTraverseCooledDepartureToANewScanDestination) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);
 for(int x=-10;x<=40;++x)for(int y=-10;y<=10;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
 tree.updateInnerOccupancy();
 ObservationGrid grid;grid.width=60;grid.height=40;grid.resolution=.1;grid.x=-1;grid.y=-2;
 grid.cells.assign(2400,-1);
 for(int x=0;x<60;++x)for(int y=18;y<=21;++y)grid.cells[y*60+x]=0;
 auto c=config();c.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(600);
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{3,0,0,0},grid,
   {{.25,0},{.65,0},{1.,0}},.25,5.);
 ASSERT_TRUE(selected);EXPECT_NEAR(selected->x,3.,.1);
 for(auto old:{.25,.65,1.})EXPECT_GT(std::hypot(selected->x-old,selected->y),.6);
 EXPECT_FALSE(SurfaceGraph(tree,c).plan({0,0,0,0},*selected).empty());
}
TEST(MappedContinuation, FullDestinationStillRespectsExcludedScanEndpoint) {
 using namespace frontier_exploration_ros2;
 octomap::OcTree tree(.1);floor(tree);auto c=config();
 ObservationGrid grid;grid.width=40;grid.height=20;grid.resolution=.1;grid.x=-1;grid.y=-1;
 grid.cells.assign(800,0);
 auto selected=selectMappedObservation(tree,c,{0,0,0,0},{1.5,0,0,0},grid,{{1.5,0}},.25,5.);
 if(selected)EXPECT_GT(std::hypot(selected->x-1.5,selected->y),.6);
}
