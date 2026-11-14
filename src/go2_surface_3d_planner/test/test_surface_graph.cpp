#include "go2_surface_3d_planner/surface_graph.hpp"

#include <gtest/gtest.h>
#include <octomap/OcTree.h>

#include <cmath>

namespace
{

using go2_surface_3d_planner::SurfaceGraph;
using go2_surface_3d_planner::SurfacePoint;
using go2_surface_3d_planner::SurfaceSearchConfig;

void observeFreeColumn(octomap::OcTree & tree, double x, double y,
  double top, double bottom)
{
  for (double z=top; z>=bottom-1e-9; z-=tree.getResolution()*.5) {
    tree.setNodeValue(octomap::point3d(x,y,z),tree.getClampingThresMinLog());
  }
}

void addSupport(octomap::OcTree & tree, double x, double y, double ground_z)
{
  constexpr double offsets[] = {-0.05, 0.0, 0.05};
  for (const double dx : offsets) {
    for (const double dy : offsets) {
      tree.updateNode(octomap::point3d(x + dx, y + dy, ground_z - 0.05), true);
    }
  }
}

SurfaceSearchConfig config()
{
  SurfaceSearchConfig cfg;
  cfg.xy_resolution = 0.10;
  cfg.z_resolution = 0.05;
  cfg.collision_radius = 0.08;
  cfg.cylinder_offset = 0.0;
  cfg.body_height = 0.40;
  cfg.max_step_height = 0.11;
  cfg.support_probe_depth = 0.12;
  cfg.snap_radius = 0.15;
  cfg.snap_vertical_range = 0.15;
  cfg.search_padding_xy = 0.30;
  cfg.search_padding_z = 0.15;
  cfg.require_known_clearance = false;
  return cfg;
}

TEST(SurfaceGraph, IterationBudgetExhaustionIsUncertainRatherThanNoRoute)
{
 octomap::OcTree tree(.1);
 for(int x=-4;x<=24;++x)for(int y=-4;y<=4;++y)addSupport(tree,x*.1,y*.1,0.);
 tree.updateInnerOccupancy();auto cfg=config();cfg.max_iterations=1;
 EXPECT_THROW(SurfaceGraph(tree,cfg).plan({0.,0.,0.},{2.,0.,0.}),
   go2_surface_3d_planner::SurfaceSearchTimeout);
}

TEST(SurfaceGraph, ClimbsConnectedStepsInThreeDimensions)
{
  octomap::OcTree tree(0.10);
  for (int ix = 0; ix <= 20; ++ix) {
    const double x = ix * 0.10;
    const double ground_z = std::floor(ix / 2.0) * 0.05;
    for (int iy = -2; iy <= 2; ++iy) {
      addSupport(tree, x, iy * 0.10, ground_z);
    }
  }
  tree.updateInnerOccupancy();

  SurfaceGraph graph(tree, config());
  const auto path = graph.plan({0.0, 0.0, 0.0}, {2.0, 0.0, 0.50});
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().z, 0.0, 0.06);
  EXPECT_NEAR(path.back().z, 0.50, 0.06);
  bool contains_vertical_motion = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    contains_vertical_motion = contains_vertical_motion ||
      std::abs(path[i].z - path[i - 1].z) > 0.01;
    EXPECT_LE(std::abs(path[i].z - path[i - 1].z), 0.11 + 1e-9);
  }
  EXPECT_TRUE(contains_vertical_motion);
}

TEST(SurfaceGraph, SnapsNoisyFloorHeightToVoxelTopRatherThanOccupiedCentre)
{
  octomap::OcTree tree(0.10);
  for (int x=-4;x<=12;++x) {
    for (int y=-4;y<=4;++y) {addSupport(tree,x*0.1,y*0.1,-0.20);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config(); cfg.snap_vertical_range=0.55;
  SurfaceGraph graph(tree,cfg);
  const auto goal=graph.snapToSurface({0.80,0.0,-0.239,0.0});
  ASSERT_TRUE(goal); EXPECT_NEAR(goal->z,-0.20,1e-9);
  EXPECT_FALSE(graph.plan({0.0,0.0,0.16,0.0},{0.8,0.0,-0.239,0.0}).empty());
}

TEST(SurfaceGraph, ExplorationRetreatsFromUnsupportedBoundaryToVerifiedObservationPoint)
{
  octomap::OcTree tree(0.10);
  for (int x=-4;x<=8;++x) {
    for (int y=-4;y<=4;++y) {addSupport(tree,x*0.1,y*0.1,0.0);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config(); cfg.snap_vertical_range=0.55;
  const auto goal=go2_surface_3d_planner::selectExplorationGoal(
    tree,cfg,{0.0,0.0,0.16,0.0},{1.4,0.0,0.02,0.0});
  ASSERT_TRUE(goal); EXPECT_LE(goal->x,0.8+1e-9); EXPECT_GE(goal->x,0.5);
  EXPECT_FALSE(SurfaceGraph(tree,cfg).plan({0.0,0.0,0.16,0.0},*goal).empty());
}

TEST(SurfaceGraph, ExplorationDoesNotAcceptAnEndpointBehindAnImpassableWall)
{
  octomap::OcTree tree(0.10);
  for (int x=-4;x<=15;++x) {
    for (int y=-10;y<=10;++y) {addSupport(tree,x*0.1,y*0.1,0.0);}
  }
  for (int y=-10;y<=10;++y) {
    for (int z=1;z<=6;++z) {tree.updateNode(octomap::point3d(0.45,y*0.1,z*0.1),true);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config(); cfg.max_iterations=10000;
  EXPECT_FALSE(go2_surface_3d_planner::selectExplorationGoal(
    tree,cfg,{0.0,0.0,0.16,0.0},{1.3,0.0,0.0,0.0},0.2));
}

TEST(SurfaceGraph, DoesNotFlyBetweenDisconnectedFloors)
{
  octomap::OcTree tree(0.10);
  for (int ix = 0; ix <= 10; ++ix) {
    for (int iy = -2; iy <= 2; ++iy) {
      addSupport(tree, ix * 0.10, iy * 0.10, 0.0);
      addSupport(tree, ix * 0.10, iy * 0.10, 2.0);
    }
  }
  tree.updateInnerOccupancy();

  auto cfg = config();
  cfg.body_height = 0.30;
  cfg.snap_vertical_range = 0.20;
  cfg.search_padding_z = 0.20;
  SurfaceGraph graph(tree, cfg);
  const auto path = graph.plan({0.0, 0.0, 0.0}, {1.0, 0.0, 2.0});
  EXPECT_TRUE(path.empty());
}

void addFloor(octomap::OcTree & tree, double length, double half_width)
{
  const double r = tree.getResolution();
  for (int ix = static_cast<int>(std::floor(-0.6 / r));
    ix <= static_cast<int>(std::ceil((length + 0.6) / r)); ++ix)
  {
    for (int iy = static_cast<int>(std::floor(-half_width / r));
      iy <= static_cast<int>(std::ceil(half_width / r)); ++iy)
    {
      tree.updateNode(octomap::point3d((ix + 0.5) * r, (iy + 0.5) * r, -0.5 * r), true);
    }
  }
}

void addWall(octomap::OcTree & tree, double length, double y)
{
  const double r = tree.getResolution();
  for (int ix = static_cast<int>(std::floor(-0.6 / r));
    ix <= static_cast<int>(std::ceil((length + 0.6) / r)); ++ix)
  {
    for (int iz = static_cast<int>(std::floor(0.1 / r));
      iz <= static_cast<int>(std::ceil(0.8 / r)); ++iz)
    {
      tree.updateNode(octomap::point3d((ix + 0.5) * r, y, (iz + 0.5) * r), true);
    }
  }
}

TEST(SurfaceGraph, Passes082MetreCorridorWithOneCentimetreMargin)
{
  octomap::OcTree tree(0.02);
  addFloor(tree, 1.0, 0.6);
  addWall(tree, 1.0, 0.42);
  addWall(tree, 1.0, -0.44);
  tree.updateInnerOccupancy();
  auto cfg = config();
  cfg.collision_radius = 0.21;
  cfg.cylinder_offset = 0.10;
  cfg.clearance_margin = 0.01;
  SurfaceGraph graph(tree, cfg);
  const auto path = graph.plan({0, 0, 0, 0}, {1.0, 0, 0, 0});
  ASSERT_FALSE(path.empty());
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(graph.isMotionValid(path[i - 1], path[i]));
  }
}

TEST(SurfaceGraph, CannotTurnFullBodyAcrossNarrowCorridor)
{
  octomap::OcTree tree(0.02);
  addFloor(tree, 1.0, 0.6);
  addWall(tree, 1.0, 0.26);
  addWall(tree, 1.0, -0.28);
  tree.updateInnerOccupancy();
  auto cfg = config();
  cfg.collision_radius = 0.21;
  cfg.cylinder_offset = 0.10;
  cfg.clearance_margin = 0.01;
  SurfaceGraph graph(tree, cfg);
  EXPECT_TRUE(graph.isMotionValid({0, 0, 0, 0}, {0.1, 0, 0, 0}));
  EXPECT_FALSE(graph.isMotionValid({0, 0, 0, 0}, {0, 0, 0, M_PI / 2}));
  const auto path = graph.plan({0, 0, 0, 0}, {1.0, 0, 0, 0});
  ASSERT_FALSE(path.empty());
  for (const auto & pose : path) {
    EXPECT_NEAR(std::remainder(pose.yaw, 2 * M_PI), 0.0, 1e-6);
  }
}

TEST(SurfaceGraph, SweepRejectsObstacleBetweenFreeEndpoints)
{
  octomap::OcTree tree(0.01);
  addFloor(tree, 0.2, 0.3);
  tree.updateNode(octomap::point3d(0.05, 0.05, 0.3), true);
  tree.updateInnerOccupancy();
  auto cfg = config();
  cfg.collision_radius = 0.015;
  cfg.clearance_margin = 0.01;
  SurfaceGraph graph(tree, cfg);
  const SurfacePoint a{0, 0, 0, 0}, b{0.1, 0.1, 0, 0};
  EXPECT_TRUE(graph.isWalkableSurface(a));
  EXPECT_TRUE(graph.isWalkableSurface(b));
  EXPECT_FALSE(graph.isMotionValid(a, b));
}

TEST(SurfaceGraph, PreservesSidewaysHeadingAndCheckedExactEndpoints)
{
  octomap::OcTree tree(0.05);
  addFloor(tree, 1.0, 0.8);
  tree.updateInnerOccupancy();
  SurfaceGraph graph(tree, config());
  const SurfacePoint a{0.023, 0, 0, M_PI / 2}, b{0.923, 0, 0, M_PI / 2};
  const auto path = graph.plan(a, b);
  ASSERT_FALSE(path.empty());
  EXPECT_NEAR(path.front().x, a.x, 1e-8);
  EXPECT_NEAR(path.back().x, b.x, 1e-8);
  for (const auto & pose : path) {
    EXPECT_NEAR(pose.yaw, M_PI / 2, 1e-8);
  }
}

TEST(SurfaceGraph, InfersObservedFlatFloorAcrossLidarBlindCellsButRejectsKnownVoid)
{
  octomap::OcTree tree(0.05);
  for (int ix=20;ix<50;++ix) {
    for (int iy=-15;iy<=15;++iy) {
      tree.updateNode(octomap::point3d((ix+0.5)*0.05,(iy+0.5)*0.05,-0.025),true);
    }
  }
  for (int ix=-10;ix<=20;++ix) {
    for (int iy=-10;iy<=10;++iy) {
      tree.updateNode(octomap::point3d((ix+0.5)*0.05,(iy+0.5)*0.05,0.325),false);
    }
  }
  tree.updateInnerOccupancy();
  const auto plane=go2_surface_3d_planner::inferSupportPlane(tree,{0,0,0.4,0},0.4);
  ASSERT_TRUE(plane);
  EXPECT_NEAR(*plane,-0.025,1e-8);
  auto cfg=config();
  cfg.infer_flat_support=true;
  cfg.flat_support_height=*plane;
  cfg.support_reference_height=.21;
  const auto prior=cfg;
  cfg.flat_support_height=std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(go2_surface_3d_planner::restoreCompatibleSupportPlane(
    cfg,prior,{0,0,.21+*plane,0}));
  auto incompatible=cfg;
  incompatible.flat_support_height=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(go2_surface_3d_planner::restoreCompatibleSupportPlane(
    incompatible,prior,{0,0,.8,0}));
  EXPECT_TRUE(std::isnan(incompatible.flat_support_height));
  EXPECT_FALSE(go2_surface_3d_planner::restoreCompatibleSupportPlane(
    incompatible,incompatible,{0,0,.21+*plane,0}));
  SurfaceGraph graph(tree,cfg);
  EXPECT_FALSE(graph.plan({0,0,0,0},{0.8,0,0,0}).empty());
  tree.updateNode(octomap::point3d(0.425,0.025,-0.125),false);
  tree.updateNode(octomap::point3d(0.425,-0.025,-0.125),false);
  tree.updateNode(octomap::point3d(0.425,0.025,-0.075),false);
  tree.updateNode(octomap::point3d(0.425,-0.025,-0.075),false);
  // Subsurface free cells alone leave the possible floor unobserved.
  EXPECT_TRUE(SurfaceGraph(tree,cfg).isWalkableSurface({0.425,0.025,0,0}));
  observeFreeColumn(tree,.425,.025,*plane+.05,*plane-.30);
  observeFreeColumn(tree,.425,-.025,*plane+.05,*plane-.30);
  SurfaceGraph with_void(tree,cfg);
  EXPECT_FALSE(with_void.isWalkableSurface({0.425,0.025,0,0}));
}

TEST(SurfaceGraph, FitsTiltedFloorAtRobotInsteadOfMostPopulatedHeight)
{
  octomap::OcTree tree(0.05);
  for(int ix=-60;ix<=60;++ix) {
    for(int iy=-35;iy<=35;++iy) {
      const double x=(ix+0.5)*0.05,y=(iy+0.5)*0.05;
      if(std::hypot(x,y)<0.8) {continue;}
      tree.updateNode(octomap::point3d(x,y,-0.075+0.04*x+0.01*y),true);
    }
  }
  for(int ix=-15;ix<=15;++ix) {
    for(int iy=-15;iy<=15;++iy) {
      tree.updateNode(octomap::point3d((ix+0.5)*0.05,(iy+0.5)*0.05,0.25),false);
    }
  }
  tree.updateInnerOccupancy();
  SurfaceSearchConfig cfg;
  cfg.infer_flat_support=true;
  cfg.support_reference_height=0.215;
  ASSERT_TRUE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,{0,0,0.14,0}));
  EXPECT_NEAR(cfg.flat_support_height,0.14-cfg.support_reference_height,1e-6);
  EXPECT_NEAR(cfg.flat_support_slope_x,0.04,0.01);
  SurfaceGraph graph(tree,cfg);
  const auto start=graph.snapToSurface({0,0,0.14,0});
  ASSERT_TRUE(start);
  EXPECT_NEAR(start->z,-0.075,0.03);
  EXPECT_TRUE(graph.isWalkableSurface(*start));
}

TEST(SurfaceGraph, DoesNotUseDenseRemoteHigherPatchAsCurrentFloor)
{
  octomap::OcTree tree(0.05);
  for(int ix=-30;ix<-10;++ix) {
    for(int iy=-15;iy<=15;++iy) {
      tree.updateNode(octomap::point3d(ix*0.05,iy*0.05,-0.075),true);
    }
  }
  for(int ix=15;ix<=55;++ix) {
    for(int iy=-30;iy<=30;++iy) {
      tree.updateNode(octomap::point3d(ix*0.05,iy*0.05,0.225),true);
    }
  }
  tree.updateInnerOccupancy();
  const auto p=go2_surface_3d_planner::estimateSupportPlane(tree,{0,0,0.14,0},0.215);
  ASSERT_TRUE(p);
  EXPECT_NEAR(p->height,-0.075,0.025);
}

TEST(SurfaceGraph, RejectsPlaneOutsideStanceHeightConstraint)
{
  octomap::OcTree tree(0.05);
  addFloor(tree,1.0,0.8);
  tree.updateInnerOccupancy();
  EXPECT_FALSE(go2_surface_3d_planner::estimateSupportPlane(tree,{0,0,0.8,0},0.21));
}


TEST(SurfaceGraph, ExpiredBudgetStopsEvenSurfaceSnapping)
{
  octomap::OcTree tree(0.10);
  addFloor(tree,2.0,0.8);tree.updateInnerOccupancy();
  auto cfg=config();
  cfg.search_deadline=std::chrono::steady_clock::now()-std::chrono::milliseconds(1);
  SurfaceGraph graph(tree,cfg);
  EXPECT_THROW(graph.plan({0,0,0,0},{1,0,0,0}),
    go2_surface_3d_planner::SurfaceSearchTimeout);
}

TEST(SurfaceGraph, ExplorationChecksNearbyObservationBeforeDistantEndpoint)
{
  octomap::OcTree tree(0.10);
  addFloor(tree,5.0,0.8);tree.updateInnerOccupancy();
  auto cfg=config();
  const auto goal=go2_surface_3d_planner::selectExplorationGoal(
    tree,cfg,{0,0,0,0},{4,0,0,0},12);
  ASSERT_TRUE(goal);
  EXPECT_NEAR(goal->x,1.5,1e-8);
  EXPECT_FALSE(SurfaceGraph(tree,cfg).plan({0,0,0,0},*goal).empty());
}


TEST(SurfaceGraph, ContinuousTiltedSupportSurvivesHalfBinBoundary)
{
  octomap::OcTree tree(0.1);
  for(int ix=-15;ix<=15;++ix) {
    for(int iy=-10;iy<=10;++iy) {
      for(double z:{0.025,0.125}) {
        tree.updateNode(octomap::point3d((ix+.5)*.1,(iy+.5)*.1,z),false);
      }
    }
  }
  tree.updateInnerOccupancy();
  SurfaceSearchConfig cfg;cfg.infer_flat_support=true;
  cfg.flat_support_height=-.275604;cfg.flat_support_slope_x=.0548947;
  cfg.flat_support_slope_y=.0135459;cfg.snap_radius=0;
  SurfaceGraph graph(tree,cfg);
  const auto snapped=graph.snapToSurface({.05787,0,-.23188,0});
  ASSERT_TRUE(snapped);
  EXPECT_NEAR(snapped->x,.05787,1e-9);
  EXPECT_NEAR(snapped->z,-.275604+.0548947*.05787,1e-9);
  EXPECT_TRUE(graph.isWalkableSurface(*snapped));
  const SurfacePoint end{.85787,0,-.275604+.0548947*.85787,0};
  EXPECT_TRUE(graph.isMotionValid(*snapped,end));
  cfg.snap_radius=.15;
  EXPECT_FALSE(SurfaceGraph(tree,cfg).plan(*snapped,end).empty());
  tree.updateNode(octomap::point3d(end.x,end.y,end.z-.05),false);
  EXPECT_TRUE(SurfaceGraph(tree,cfg).isWalkableSurface(end));
  tree.updateNode(octomap::point3d(end.x,end.y,end.z-.15),false);
  EXPECT_TRUE(SurfaceGraph(tree,cfg).isWalkableSurface(end));
  observeFreeColumn(tree,end.x,end.y,end.z+.1,end.z-.30);
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface(end));
}

TEST(SurfaceGraph, ObservedGroundRemainsUsableWhenInferenceFails)
{
  octomap::OcTree tree(.05);addFloor(tree,1.,.8);tree.updateInnerOccupancy();
  auto cfg=config();cfg.infer_flat_support=true;cfg.support_reference_height=.21;
  EXPECT_FALSE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,{0,0,.8,0}));
  EXPECT_TRUE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,0,0}));
  EXPECT_FALSE(SurfaceGraph(tree,cfg).plan({0,0,0,0},{.8,0,0,0}).empty());
}


TEST(SurfaceGraph, GroundFitAcceptsRegistrationAtVoxelBoundaryButRejectsWrongFloor)
{
  octomap::OcTree tree(.1);
  for(int x=-15;x<=15;++x) {
    for(int y=-15;y<=15;++y) {
      tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.25),true);
    }
  }
  tree.updateInnerOccupancy();
  SurfaceSearchConfig cfg;
  cfg.infer_flat_support=true;
  cfg.support_reference_height=.2071331099;
  const SurfacePoint body{0,0,-.04455790269,0};
  ASSERT_TRUE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,body));
  EXPECT_NEAR(cfg.flat_support_height,body.z-cfg.support_reference_height,1e-6);
  EXPECT_TRUE(SurfaceGraph(tree,cfg).snapToSurface({0,0,-.251691,0}));
  EXPECT_TRUE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,{0,0,-.057465,0}));
  EXPECT_NEAR(cfg.flat_support_height,-.057465-cfg.support_reference_height,1e-6);
  // The added registration tolerance cannot accept a different floor.
  EXPECT_FALSE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,{0,0,-.20,0}));
  EXPECT_TRUE(std::isnan(cfg.flat_support_height));
}

TEST(SurfaceGraph, UnsupportedFrontierCanUseRotatedReachableObservationInNarrowSlot)
{
  octomap::OcTree tree(.1);
  for(int x=-5;x<=15;++x)for(int y=-8;y<=17;++y) {
    tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  }
  for(double y=.65;y<2.6;y+=.1)for(double z=.15;z<.5;z+=.1) {
    tree.updateNode(octomap::point3d(.45,y,z),true);
    tree.updateNode(octomap::point3d(1.15,y,z),true);
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.collision_radius=.21;cfg.cylinder_offset=.10;
  cfg.clearance_margin=.04;cfg.search_padding_xy=1.;
  const SurfacePoint start{.8,0,0,0},frontier{.8,2.4,0,0};
  ASSERT_FALSE(SurfaceGraph(tree,cfg).snapToSurface(frontier));
  const auto observer=go2_surface_3d_planner::selectExplorationGoal(tree,cfg,start,frontier,1.4);
  ASSERT_TRUE(observer);
  EXPECT_GE(observer->y,1.0);
  EXPECT_LT(std::abs(std::cos(observer->yaw)),.7);
  const auto path=SurfaceGraph(tree,cfg).plan(start,*observer);
  ASSERT_FALSE(path.empty());
  for(std::size_t i=1;i<path.size();++i) {EXPECT_TRUE(SurfaceGraph(tree,cfg).isMotionValid(path[i-1],path[i]));}
}

TEST(SurfaceGraph, RegisteredGroundVoxelBandDoesNotAcceptHigherTerrainOrObservedVoid)
{
  octomap::OcTree tree(.05);
  for(int x=-12;x<=12;++x)for(int y=-12;y<=12;++y) {
    tree.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,.025),true);
    tree.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,.325),false);
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  const SurfacePoint stance{.025,.025,0,0};
  ASSERT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
  ASSERT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).support);
  auto raised=tree;
  raised.updateNode(octomap::point3d(.025,.025,.075),true);
  EXPECT_FALSE(SurfaceGraph(raised,cfg).diagnosePose(stance).surface);
  tree.updateNode(octomap::point3d(.025,.025,-.075),false);
  EXPECT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
  tree.updateNode(octomap::point3d(.025,.025,-.125),false);
  // The occupied floor cell still separates the free subsurface volume.
  EXPECT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
  observeFreeColumn(tree,.025,.025,.10,-.30);
  EXPECT_FALSE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
}

TEST(SurfaceGraph, OpenFloorPrefersSeparateTurnsAndStraightTravel)
{
  octomap::OcTree tree(.1);
  addFloor(tree,1.8,1.8);tree.updateInnerOccupancy();
  auto cfg=config();
  const auto path=SurfaceGraph(tree,cfg).plan({0,0,0,0},{1.,0,0,1.5707963267948966});
  ASSERT_GT(path.size(),2U);
  bool translated=false,rotated=false;
  for(std::size_t i=1;i<path.size();++i) {
    const double translation=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
    const double turn=std::abs(std::remainder(path[i].yaw-path[i-1].yaw,2*3.14159265358979323846));
    if(translation>1e-8) {translated=true;EXPECT_NEAR(turn,0,1e-8);}
    if(turn>1e-8) {rotated=true;EXPECT_NEAR(translation,0,1e-8);}
  }
  EXPECT_TRUE(translated);EXPECT_TRUE(rotated);
}

TEST(SurfaceGraph, MarginRecoveryIncreasesGapWithoutAllowingCollisionOrUnsupportedMotion)
{
  octomap::OcTree tree(.1);addFloor(tree,1.,.8);
  for(double y=-.5;y<=.5;y+=.1) {
    for(double z=.15;z<=.45;z+=.1) {
      tree.updateNode(octomap::point3d(.25,y,z),true);
    }
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.clearance_margin=0;cfg.preferred_clearance=.1;cfg.snap_radius=0;
  SurfaceGraph graph(tree,cfg);
  const SurfacePoint start{.115,0,0,0};
  ASSERT_GT(graph.bodyClearanceAt(start),0);
  ASSERT_LT(graph.bodyClearanceAt(start),.01);
  const auto end=graph.findClearanceRecovery(start,.01,.06);
  ASSERT_TRUE(end);EXPECT_LT(end->x,start.x);
  EXPECT_GE(graph.bodyClearanceAt(*end),.01);
  EXPECT_TRUE(graph.isMotionValid(start,*end));
  // A legal stance near a blocked turn may seek extra clearance proactively.
  const auto turn_space=graph.findClearanceRecovery({.10,0,0,0},.03,.06);
  ASSERT_TRUE(turn_space);EXPECT_GE(graph.bodyClearanceAt(*turn_space),.03);
  EXPECT_TRUE(graph.findClearanceRecovery({.15,0,0,0},.01,.06));
  EXPECT_FALSE(graph.findClearanceRecovery(start,.01,.001));
  // A short controller horizon may safely improve clearance before the
  // requested comfort gap is reachable. Existing contact may only decrease.
  const auto partial=graph.findClearanceRecovery(start,.10,.02,true);
  ASSERT_TRUE(partial);
  EXPECT_GT(graph.bodyClearanceAt(*partial),graph.bodyClearanceAt(start)+.005);
  EXPECT_LT(graph.bodyClearanceAt(*partial),.10);
  EXPECT_TRUE(graph.isMotionValid(start,*partial));
  const SurfacePoint contact{.15,0,0,0};
  const auto retreat=graph.findClearanceRecovery(contact,.10,.02,true);
  ASSERT_TRUE(retreat);EXPECT_TRUE(graph.isClearanceRecoveryMotionValid(contact,*retreat));
  EXPECT_GT(graph.bodyClearanceAt(*retreat),graph.bodyClearanceAt(contact));
  EXPECT_FALSE(graph.findClearanceRecovery({.115,0,1.,0},.10,.02,true));
  EXPECT_FALSE(graph.findClearanceRecovery({.115,0,1.,0},.01,.06));
  cfg.clearance_margin=.01;
  EXPECT_FALSE(SurfaceGraph(tree,cfg).findClearanceRecovery(start,.01,.06));
}

TEST(SurfaceGraph, ExplorationApproachFollowsOffsetDoorRouteInsteadOfStraightRay)
{
  octomap::OcTree tree(.1);addFloor(tree,2.5,2.);
  for(double y=-2.;y<=2.;y+=.1) {
    if(y>.8 && y<1.6) {continue;}
    for(double z=.15;z<=.45;z+=.1) {tree.updateNode(octomap::point3d(1.,y,z),true);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config();
  cfg.search_padding_xy=1.8;
  const auto approach=go2_surface_3d_planner::selectExplorationGoal(
    tree,cfg,{0,0,0,0},{2,0,0,0},12.);
  ASSERT_TRUE(approach);
  EXPECT_GT(approach->y,.5);
  EXPECT_FALSE(SurfaceGraph(tree,cfg).plan({0,0,0,0},*approach).empty());
}

TEST(SurfaceGraph, NarrowFallbackPreservesBodySweepAndRejectsContact)
{
  octomap::OcTree tree(.1);addFloor(tree,2.,1.);
  for(double x=-1.5;x<=1.5;x+=.1)for(double y:{-.25,.25}) {
    for(double z=.15;z<=.45;z+=.1) {tree.updateNode(octomap::point3d(x,y,z),true);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.collision_radius=.18;cfg.clearance_margin=.04;cfg.snap_radius=0;
  const SurfacePoint start{0,0,0,0},goal{1,0,0,0};
  EXPECT_TRUE(SurfaceGraph(tree,cfg).plan(start,goal).empty());
  cfg.narrow_clearance_margin=.01;
  const auto path=SurfaceGraph(tree,cfg).plan(start,goal);
  ASSERT_GT(path.size(),1U);
  auto hard=cfg;hard.clearance_margin=.01;hard.narrow_clearance_margin=0;
  SurfaceGraph verifier(tree,hard);
  for(std::size_t i=1;i<path.size();++i) {EXPECT_TRUE(verifier.isMotionValid(path[i-1],path[i]));}
  cfg.collision_radius=.21;
  EXPECT_TRUE(SurfaceGraph(tree,cfg).plan(start,goal).empty());
}

TEST(SurfaceGraph, NarrowFallbackNeverBridgesMissingSupport)
{
  octomap::OcTree tree(.1);addFloor(tree,2.,1.);
  for(double x=-1.5;x<=1.5;x+=.1)for(double y:{-.25,.25}) {
    for(double z=.15;z<=.45;z+=.1) {tree.updateNode(octomap::point3d(x,y,z),true);}
  }
  for(double y=-.2;y<=.2;y+=.05) {
    for(double x=.4;x<=.7;x+=.05) {tree.setNodeValue(octomap::point3d(x,y,-.05),tree.getClampingThresMinLog());}
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.collision_radius=.18;cfg.clearance_margin=.04;
  cfg.narrow_clearance_margin=.01;cfg.snap_radius=0;
  EXPECT_TRUE(SurfaceGraph(tree,cfg).plan({0,0,0,0},{1,0,0,0}).empty());
}

TEST(SurfaceGraph, HalfVoxelPoseSearchPreservesNominalMarginBetweenGridCentres)
{
  octomap::OcTree tree(.1);addFloor(tree,2.,1.);
  for(double x=-1.5;x<=1.5;x+=.1)for(double y:{-.15,.25}) {
    for(double z=.15;z<=.45;z+=.1) {tree.updateNode(octomap::point3d(x,y,z),true);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.collision_radius=.10;cfg.clearance_margin=.04;cfg.snap_radius=0;
  const SurfacePoint start{0,.05,0,0},goal{1,.05,0,0};
  EXPECT_TRUE(SurfaceGraph(tree,cfg).plan(start,goal).empty());
  cfg.narrow_clearance_margin=.01;
  const auto path=SurfaceGraph(tree,cfg).plan(start,goal);ASSERT_GT(path.size(),1U);
  for(std::size_t i=1;i<path.size();++i) {EXPECT_TRUE(SurfaceGraph(tree,cfg).isMotionValid(path[i-1],path[i]));}
  cfg.snap_radius=.15;
  const auto approach=go2_surface_3d_planner::selectExplorationGoal(tree,cfg,start,goal,1.5);
  ASSERT_TRUE(approach);EXPECT_NEAR(approach->y,.05,1e-9);
}

TEST(SurfaceGraph, FrontierApproachCanChangeHeadingBeforeEnteringNarrowDoor)
{
  octomap::OcTree tree(.1);addFloor(tree,2.,1.);
  for(double x=.5;x<=1.5;x+=.1)for(double y:{-.25,.25}) {
    for(double z=.15;z<=.45;z+=.1) {tree.updateNode(octomap::point3d(x,y,z),true);}
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.collision_radius=.18;cfg.cylinder_offset=.10;
  cfg.clearance_margin=.01;cfg.search_padding_xy=.6;
  const SurfacePoint start{0,0,0,.8},requested{1,0,0,1.5707963267948966};
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface(requested));
  const auto approach=go2_surface_3d_planner::selectExplorationGoal(tree,cfg,start,requested,1.5);
  ASSERT_TRUE(approach);EXPECT_GT(approach->x,.8);
  const auto path=SurfaceGraph(tree,cfg).plan(start,*approach);ASSERT_GT(path.size(),1U);
  for(std::size_t i=1;i<path.size();++i) {EXPECT_TRUE(SurfaceGraph(tree,cfg).isMotionValid(path[i-1],path[i]));}
}
}  // namespace

TEST(SupportPlaneRegression, KeepsConstrainedCandidateWhenQuantizedRefinementCrossesLimit)
{
  for (double ground_height : {-.183, -.185}) {
    octomap::OcTree tree(.1);
    for (int x=-40; x<=40; ++x) {
      for (int y=-40; y<=40; ++y) {
        const double px=x*.1+.05, py=y*.1+.05;
        tree.updateNode(octomap::point3d(px,py,ground_height+.02*px-.0077*py),true);
      }
    }
    const go2_surface_3d_planner::SurfacePoint measured{0,0,.0718745195337807,0};
    constexpr double body_reference=.196592923650097;
    const auto plane=go2_surface_3d_planner::estimateSupportPlane(tree,measured,body_reference);
    ASSERT_TRUE(plane);
    EXPECT_LE(std::abs(plane->height-(measured.z-body_reference)),.06);
    EXPECT_LE(std::hypot(plane->slope_x,plane->slope_y),.10);
    EXPECT_FALSE(go2_surface_3d_planner::estimateSupportPlane(
      tree,{0,0,2.0,0},body_reference));
  }
}

TEST(SurfaceGraph, DisconnectedSubsurfaceFreeDoesNotProveVoidAcrossUnknownFloor)
{
  octomap::OcTree tree(.05);
  for(int x=-5;x<=5;++x)for(int y=-5;y<=5;++y) {
    tree.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,.325),false);
  }
  tree.updateNode(octomap::point3d(.025,.025,-.075),false);
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  const SurfacePoint stance{.025,.025,0,0};
  EXPECT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
  EXPECT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).support);
  tree.updateNode(octomap::point3d(.025,.025,-.175),false);
  EXPECT_TRUE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
  cfg.infer_flat_support=false;
  EXPECT_FALSE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
  cfg.infer_flat_support=true;
  observeFreeColumn(tree,.025,.025,.10,-.30);
  EXPECT_FALSE(SurfaceGraph(tree,cfg).diagnosePose(stance).surface);
}

TEST(SurfaceGraph, MissingGroundBandSeparatesDeepFreeFromBodySpace)
{
  octomap::OcTree tree(.05);
  // Captured failure: floor [-.05,0] unknown, body above and an older volume
  // [-.15,-.05] free. The fitted floor uncertainty band includes z=0.
  for(int x=-8;x<=8;++x)for(int y=-8;y<=8;++y) {
    const double px=(x+.5)*.05, py=(y+.5)*.05;
    for(double z:{-.125,-.075,.025,.075,.125,.175,.225,.275,.325}) {
      tree.setNodeValue(octomap::point3d(px,py,z),tree.getClampingThresMinLog());
    }
  }
  SurfaceSearchConfig cfg;cfg.infer_flat_support=true;cfg.flat_support_height=.017;
  const SurfacePoint stance{.025,.025,.025,0};
  auto d=SurfaceGraph(tree,cfg).diagnosePose(stance);
  EXPECT_TRUE(d.surface); EXPECT_TRUE(d.support); EXPECT_TRUE(d.clearance);
  observeFreeColumn(tree,.025,.025,.10,-.30);
  d=SurfaceGraph(tree,cfg).diagnosePose(stance);
  EXPECT_FALSE(d.surface); EXPECT_FALSE(d.support);
}


TEST(ContinuousFloorPrior, AllowsBlindOrHistoricallyClearedSolidSlabButKeepsBodyCollisions)
{
  octomap::OcTree tree(.05);
  for(int ix=-16;ix<=16;++ix) for(int iy=-10;iy<=10;++iy)
    observeFreeColumn(tree,(ix+.5)*.05,(iy+.5)*.05,.725,-.375);
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,0,0}));
  cfg.continuous_floor_region={0,0,0,0,0,0,1,2,2,.10};
  const auto size=tree.size();
  EXPECT_TRUE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,0,0}));
  EXPECT_EQ(tree.size(),size);
  EXPECT_TRUE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,{0,0,.21,0}));
  EXPECT_FALSE(SurfaceGraph(tree,cfg).plan({0,0,0,0},{.6,0,0,0}).empty());
  tree.setNodeValue(octomap::point3d(0,0,.25),tree.getClampingThresMaxLog());
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,0,0}));
}

TEST(ContinuousFloorPrior, BoundsHeightAndStairModesStayStrict)
{
  octomap::OcTree tree(.05);
  for(double x=-.5;x<=.5;x+=.05) for(double y=-.5;y<=.5;y+=.05)
    observeFreeColumn(tree,x,y,3,-.4);
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  cfg.continuous_floor_region={0,0,0,0,0,0,1,.2,.2,.10};
  EXPECT_FALSE(go2_surface_3d_planner::continuousFloorPlane(cfg,{.3,0,0,0}));
  EXPECT_FALSE(go2_surface_3d_planner::continuousFloorPlane(cfg,{0,0,2.4,0}));
  cfg.flat_support_height=2.4;
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,2.4,0}));
  cfg.flat_support_height=0;cfg.infer_flat_support=false;
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,0,0}));
}

TEST(ContinuousFloorPrior, ResumeTransformAndInvalidConfigurations)
{
  auto cfg=config();cfg.infer_flat_support=true;
  const double a=.3;
  cfg.continuous_floor_region={3,-2,.05,0,0,std::sin(a/2),std::cos(a/2),1,.3,.08};
  EXPECT_TRUE(go2_surface_3d_planner::continuousFloorPlane(cfg,{3+.8*std::cos(a),-2+.8*std::sin(a),.05,0}));
  EXPECT_FALSE(go2_surface_3d_planner::continuousFloorPlane(cfg,{3-std::sin(a),-2+std::cos(a),.05,0}));
  cfg.continuous_floor_region[6]=2;
  EXPECT_FALSE(go2_surface_3d_planner::continuousFloorPlane(cfg,{3,-2,.05,0}));
  cfg.continuous_floor_region.clear();
  EXPECT_FALSE(go2_surface_3d_planner::continuousFloorPlane(cfg,{0,0,0,0}));
}


TEST(ContinuousFloorPrior, DoesNotOpenUnobservedBodySpaceAndHandlesTiltedMapFrame)
{
  octomap::OcTree tree(.05);
  auto cfg=config();cfg.infer_flat_support=true;
  cfg.continuous_floor_region={0,0,0,0,0,0,1,2,2,.10};
  EXPECT_TRUE(go2_surface_3d_planner::configureSupportPlane(cfg,tree,{0,0,.21,0}));
  EXPECT_FALSE(SurfaceGraph(tree,cfg).isWalkableSurface({0,0,0,0}));
  const double angle=.04;
  cfg.continuous_floor_region={3,-2,.05,0,std::sin(angle/2),0,std::cos(angle/2),1,.3,.08};
  auto plane=go2_surface_3d_planner::continuousFloorPlane(cfg,
    {3+.8*std::cos(angle),-2,.05-.8*std::sin(angle),0});
  ASSERT_TRUE(plane);
  EXPECT_NEAR(plane->slope_x,-std::tan(angle),1e-9);
}

TEST(TurnFailureConstraint, KeepsTranslationAndSmallAlignmentButBlocksLargeTurns) {
  using namespace go2_surface_3d_planner;
  TurnExclusionRegion r{0,0,0,0,.25};
  EXPECT_FALSE(violatesTurnExclusion({0,0,0,0},{.3,0,0,0},r));
  EXPECT_FALSE(violatesTurnExclusion({0,0,0,.05},{0,0,0,0},r));
  EXPECT_TRUE(violatesTurnExclusion({0,0,0,0},{0,0,0,.196},r));
  EXPECT_TRUE(violatesTurnExclusion({0,0,0,.10},{0,0,0,.20},r));
  EXPECT_FALSE(violatesTurnExclusion({.4,0,0,0},{.4,0,0,.5},r));
  EXPECT_FALSE(violatesTurnExclusion({0,0,2,0},{0,0,2,.5},r));
}
TEST(TurnFailureConstraint, PlansTranslationBeforeTurningWithAllSweepsPreserved) {
  using namespace go2_surface_3d_planner;
  octomap::OcTree tree(.1);
  for(int x=-12;x<20;++x)for(int y=-12;y<12;++y)
    tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  tree.updateInnerOccupancy();
  auto c=config();c.snap_radius=.1;c.search_padding_xy=.6;
  c.turn_exclusions.push_back({0,0,0,0,.25});
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(4);
  SurfaceGraph graph(tree,c);const auto path=graph.plan({0,0,0,0},{.8,0,0,1.57});
  ASSERT_FALSE(path.empty());bool large_turn=false;
  for(std::size_t i=1;i<path.size();++i) {
    EXPECT_TRUE(graph.isMotionValid(path[i-1],path[i]));
    EXPECT_FALSE(violatesTurnExclusion(path[i-1],path[i],c.turn_exclusions.front()));
    if(std::abs(std::remainder(path[i].yaw,2*M_PI))>.12)large_turn=true;
  }
  EXPECT_TRUE(large_turn);
}

TEST(SurfaceGraphMeasuredStart, NearbySurfaceCannotReplaceInvalidMeasuredHeight) {
 octomap::OcTree tree(.1);addFloor(tree,1.,.4);
 // The measured column has a higher top; nearby floor cells are lower.
 // At the lower height, support probes can hit occupied volume even though
 // there is no surface on which the controller can stand.
 for(double x:{-.05,.05})for(double y:{-.05,.05})
   tree.updateNode(octomap::point3d(x,y,.05),true);
 tree.updateInnerOccupancy();
 auto cfg=config();cfg.snap_radius=.25;
 cfg.snap_vertical_range=.15;
 const SurfacePoint measured{.025,.025,-.15,0},goal{.8,.025,0,0};
 auto exact=cfg;exact.snap_radius=0.;
 EXPECT_FALSE(SurfaceGraph(tree,exact).snapToSurface(measured));
 const auto nearby=SurfaceGraph(tree,cfg).snapToSurface(measured);
 ASSERT_TRUE(nearby);
 EXPECT_GT(std::hypot(nearby->x-measured.x,nearby->y-measured.y),.01);
 EXPECT_TRUE(SurfaceGraph(tree,cfg).plan(measured,goal).empty());
}
TEST(SurfaceGraphMeasuredStart, SafeOffGridStartIsRetainedAndConnected) {
 octomap::OcTree tree(.1);addFloor(tree,1.,.5);tree.updateInnerOccupancy();
 auto cfg=config();cfg.snap_radius=.25;
 const SurfacePoint measured{.023,.017,.01,.17},goal{.8,0,0,0};
 auto exact=cfg;exact.snap_radius=0.;
 const auto stance=SurfaceGraph(tree,exact).snapToSurface(measured);
 ASSERT_TRUE(stance);
 const auto path=SurfaceGraph(tree,cfg).plan(measured,goal);
 ASSERT_GT(path.size(),1U);
 EXPECT_NEAR(path.front().x,measured.x,1e-9);
 EXPECT_NEAR(path.front().y,measured.y,1e-9);
 EXPECT_NEAR(path.front().z,stance->z,1e-9);
 EXPECT_NEAR(path.front().yaw,measured.yaw,1e-9);
 for(size_t i=1;i<path.size();++i)
   EXPECT_TRUE(SurfaceGraph(tree,cfg).isMotionValid(path[i-1],path[i]));
}

TEST(SupportFallback, Local3DPatchRecoversFiniteFalseFreeColumn)
{
  octomap::OcTree tree(.05);
  for(int x=-8;x<=8;++x)for(int y=-8;y<=8;++y) {
    tree.updateNode(octomap::point3d(x*.05+.025,y*.05+.025,-.025),true);
    observeFreeColumn(tree,x*.05+.025,y*.05+.025,.45,.075);
  }
  // A finite ray-clearing streak contradicts the local plane at the centre,
  // but observed ground spans both axes around it; below the streak is unknown.
  observeFreeColumn(tree,.025,.025,.05,-.15);
  tree.deleteNode(octomap::point3d(.025,.025,-.175));
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  cfg.flat_support_origin_x=.025;cfg.flat_support_origin_y=.025;
  SurfaceGraph graph(tree,cfg);
  const SurfacePoint p{.025,.025,0,0};
  EXPECT_EQ(graph.inspectCloudSupport(p),SurfaceGraph::CloudSupport::confirmed);
  EXPECT_TRUE(graph.diagnosePose(p).surface);
  EXPECT_TRUE(graph.diagnosePose(p).support);
}

TEST(SupportFallback, ObservedDropOverridesNearbyPatchAndLowerFloor)
{
  octomap::OcTree tree(.05);
  for(int x=-8;x<=8;++x)for(int y=-8;y<=8;++y)
    tree.updateNode(octomap::point3d(x*.05+.025,y*.05+.025,-.025),true);
  observeFreeColumn(tree,.025,.025,.45,-.50);
  tree.updateNode(octomap::point3d(.025,.025,-2.525),true);
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  SurfaceGraph graph(tree,cfg);const SurfacePoint p{.025,.025,0,0};
  EXPECT_EQ(graph.inspectCloudSupport(p),SurfaceGraph::CloudSupport::drop);
  EXPECT_FALSE(graph.diagnosePose(p).surface);
  EXPECT_FALSE(graph.diagnosePose(p).support);
}

TEST(SupportFallback, MissingCloudAndOtherStoreyAreNotConfirmation)
{
  octomap::OcTree tree(.05);
  for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)
    tree.updateNode(octomap::point3d(x*.05+.025,y*.05+.025,-2.525),true);
  auto cfg=config();cfg.infer_flat_support=true;cfg.flat_support_height=0;
  EXPECT_EQ(SurfaceGraph(tree,cfg).inspectCloudSupport({.025,.025,0,0}),
    SurfaceGraph::CloudSupport::unknown);
}

TEST(SupportDetour, RejectsEntryAndAllowsMonotonicExit)
{
  octomap::OcTree tree(.05);
  addFloor(tree,1.8,.8);tree.updateInnerOccupancy();
  auto cfg=config();
  ASSERT_TRUE(SurfaceGraph(tree,cfg).isMotionValid({.5,0,0,0},{.8,0,0,0}));
  cfg.support_avoidance.push_back({.5,0,0,.18});
  SurfaceGraph graph(tree,cfg);
  EXPECT_FALSE(graph.isMotionValid({.2,0,0,0},{.8,0,0,0}));
  EXPECT_TRUE(graph.isMotionValid({.5,0,0,0},{.8,0,0,0}));
  EXPECT_FALSE(graph.isMotionValid({.6,0,0,0},{.5,0,0,0}));
  auto path=graph.plan({0,0,0,0},{1,0,0,0});
  ASSERT_FALSE(path.empty());
  bool detoured=false;
  for(size_t i=1;i<path.size();++i) {
    EXPECT_TRUE(graph.isMotionValid(path[i-1],path[i]));
    detoured|=std::abs(path[i].y)>.15;
  }
  EXPECT_TRUE(detoured);
}

TEST(MicroEscape, KeepsShortPrefixWithoutEnteringAnotherObstacle)
{
  octomap::OcTree tree(.05);addFloor(tree,1.,1.);
  for(int y=-20;y<=20;++y)for(int z=3;z<=10;++z) {
    tree.updateNode(octomap::point3d(.225,(y+.5)*.05,(z+.5)*.05),true);
    tree.updateNode(octomap::point3d(.025,(y+.5)*.05,(z+.5)*.05),true);
  }
  tree.updateInnerOccupancy();
  auto cfg=config();cfg.clearance_margin=0;cfg.snap_radius=0;cfg.preferred_clearance=.1;
  SurfaceGraph graph(tree,cfg);SurfacePoint start{.15,0,0,0};
  ASSERT_LT(graph.bodyClearanceAt(start),0);
  const auto end=graph.findClearanceRecovery(start,.05,.06,true);
  ASSERT_TRUE(end);
  EXPECT_GT(graph.bodyClearanceAt(*end),graph.bodyClearanceAt(start));
  EXPECT_TRUE(graph.isClearanceRecoveryMotionValid(start,*end));
  EXPECT_GE(end->x,.13-1e-8); // New left-wall penetration is forbidden.
  EXPECT_FALSE(graph.isClearanceRecoveryMotionValid(start,{.10,0,0,0}));
  EXPECT_FALSE(graph.isClearanceRecoveryMotionValid(start,{.16,0,0,0}));
}
TEST(MicroEscape, DoesNotRotateOrLeaveSupport)
{
  octomap::OcTree tree(.05);addFloor(tree,.1,.1);
  auto cfg=config();cfg.clearance_margin=0;SurfaceGraph graph(tree,cfg);
  EXPECT_FALSE(graph.isClearanceRecoveryMotionValid({0,0,0,0},{0,0,0,.1}));
  EXPECT_FALSE(graph.isClearanceRecoveryMotionValid({0,0,0,0},{2,0,0,0}));
}

TEST(ContinuousFloorPrior, RegisteredSlabSupportsLongRoutesBeyondLocalFitRadius) {
 octomap::OcTree tree(.1);
 for(int x=-3;x<=48;++x)for(int y=-4;y<=4;++y) {
   if(x<5)addSupport(tree,x*.1,y*.1,0);
   observeFreeColumn(tree,x*.1+.05,y*.1+.05,.45,.10);
 }
 tree.updateInnerOccupancy();auto cfg=config();cfg.infer_flat_support=true;
 cfg.flat_support_height=0;cfg.flat_support_origin_x=cfg.flat_support_origin_y=0;
 cfg.flat_support_radius=1.;cfg.snap_radius=0;
 cfg.continuous_floor_region={2.,0,0,0,0,0,1,2.5,1.,.1};
 EXPECT_TRUE(SurfaceGraph(tree,cfg).snapToSurface({4.,0,0,0}));
 EXPECT_FALSE(SurfaceGraph(tree,cfg).plan({0,0,0,0},{4.,0,0,0}).empty());
 cfg.continuous_floor_region.clear();
 EXPECT_FALSE(SurfaceGraph(tree,cfg).snapToSurface({4.,0,0,0}));
}
TEST(ContinuousFloorPrior, DistantPriorDoesNotCreateUpperFloorSupportOrUnobservedSpace) {
 octomap::OcTree tree(.1);auto cfg=config();cfg.infer_flat_support=true;
 cfg.flat_support_height=0;cfg.flat_support_radius=1.;cfg.snap_radius=0;
 cfg.continuous_floor_region={2.,0,0,0,0,0,1,2.5,1.,.1};
 EXPECT_FALSE(SurfaceGraph(tree,cfg).snapToSurface({4.,0,0,0}));
 for(int x=35;x<=45;++x)for(int y=-4;y<=4;++y)
   observeFreeColumn(tree,x*.1+.05,y*.1+.05,.45,.1);
 tree.updateInnerOccupancy();
 EXPECT_TRUE(SurfaceGraph(tree,cfg).snapToSurface({4.,0,0,0}));
 EXPECT_FALSE(SurfaceGraph(tree,cfg).snapToSupportedSurface({4.,0,2.4,0}));
 EXPECT_FALSE(SurfaceGraph(tree,cfg).snapToSurface({5.,0,0,0}));
}
