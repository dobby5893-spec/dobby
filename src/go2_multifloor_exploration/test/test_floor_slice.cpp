#include "go2_multifloor_exploration/floor_slice.hpp"

#include <gtest/gtest.h>
#include <octomap/OcTree.h>

namespace
{

using go2_multifloor_exploration::FloorSliceBuilder;
using go2_multifloor_exploration::FloorSliceConfig;

void addFloor(octomap::OcTree & tree, double z)
{
  for (int ix = 0; ix < 10; ++ix) {
    for (int iy = 0; iy < 10; ++iy) {
      tree.updateNode(octomap::point3d(0.05 + ix * 0.1, 0.05 + iy * 0.1, z - 0.05), true);
    }
  }
}

FloorSliceConfig config()
{
  FloorSliceConfig cfg;
  cfg.resolution = 0.1;
  cfg.min_x = 0.0;
  cfg.max_x = 1.0;
  cfg.min_y = 0.0;
  cfg.max_y = 1.0;
  cfg.support_xy_tolerance = 0.04;
  return cfg;
}

TEST(FloorSlice, SeparatesOverlappingFloorsByHeight)
{
  octomap::OcTree tree(0.1);
  addFloor(tree, 0.0);
  addFloor(tree, 2.4);
  tree.updateInnerOccupancy();
  FloorSliceBuilder builder(config());

  const auto lower = builder.build(tree, 0.0);
  const auto upper = builder.build(tree, 2.4);
  ASSERT_EQ(lower.data.size(), 100u);
  ASSERT_EQ(upper.data.size(), 100u);
  EXPECT_EQ(lower.data[55], 0);
  EXPECT_EQ(upper.data[55], 0);
}

TEST(FloorSlice, MarksObstacleOnlyOnItsOwnFloor)
{
  octomap::OcTree tree(0.1);
  addFloor(tree, 0.0);
  addFloor(tree, 2.4);
  tree.updateNode(octomap::point3d(0.55, 0.55, 0.35), true);
  tree.updateInnerOccupancy();
  FloorSliceBuilder builder(config());

  const auto lower = builder.build(tree, 0.0);
  const auto upper = builder.build(tree, 2.4);
  EXPECT_EQ(lower.data[55], 100);
  EXPECT_EQ(upper.data[55], 0);
}

TEST(FloorSlice, UsesObservedFreeSpaceWithoutGroundReturn)
{
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.55, 0.55, 0.25), false);
  tree.updateInnerOccupancy();
  FloorSliceBuilder builder(config());

  const auto lower = builder.build(tree, 0.0);
  const auto upper = builder.build(tree, 2.4);
  EXPECT_EQ(lower.data[55], 0);
  EXPECT_EQ(upper.data[55], -1);
}

TEST(FloorSlice, ProjectsOffCentreFineVoxelWithoutExpandingOutsideOutputCell)
{
  octomap::OcTree tree(.05);
  tree.updateNode(octomap::point3d(.425,.425,.325),true);
  tree.updateInnerOccupancy();
  const auto grid=FloorSliceBuilder(config()).build(tree,0);
  EXPECT_EQ(grid.data[44],100);
  EXPECT_EQ(grid.data[43],-1); EXPECT_EQ(grid.data[45],-1);
  EXPECT_EQ(grid.data[34],-1); EXPECT_EQ(grid.data[54],-1);
}

TEST(FloorSlice, ProjectsFineVoxelBoxBoundaryAndFillsItsInterior)
{
  octomap::OcTree tree(.05);
  for(int y=2;y<=14;++y)for(int x=2;x<=14;++x) {
    if(x==2 || x==14 || y==2 || y==14)
      tree.updateNode(octomap::point3d(x*.05+.025,y*.05+.025,.325),true);
  }
  tree.updateInnerOccupancy();
  const auto grid=FloorSliceBuilder(config()).build(tree,0);
  EXPECT_EQ(grid.data[44],100);
  EXPECT_EQ(grid.data[0],-1);
}

go2_multifloor_exploration::FloorSliceGrid enclosedGrid(int sx = 12, int sy = 7)
{
  go2_multifloor_exploration::FloorSliceGrid grid;
  grid.width = 40; grid.height = 40; grid.resolution = .1;
  grid.data.assign(1600, 0);
  for (int y = 3; y < 3 + sy; ++y) {
    for (int x = 3; x < 3 + sx; ++x) {
      grid.data[y * 40 + x] = (x == 3 || y == 3 || x == 2 + sx || y == 2 + sy) ? 100 : -1;
    }
  }
  return grid;
}

TEST(FloorSlice, FillsOnlyCompactFullyEnclosedUnknownInterior)
{
  auto grid = enclosedGrid(); const auto original = grid.data;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 50u);
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    EXPECT_EQ(grid.data[i], original[i] < 0 ? 100 : original[i]);
  }
}

TEST(FloorSlice, PreservesOneCellDoorOpeningToUnknownExterior)
{
  auto grid = enclosedGrid();
  grid.data[3 * 40 + 8] = -1; grid.data[2 * 40 + 8] = -1;
  const auto original = grid.data;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data, original);
}

TEST(FloorSlice, PreservesDiagonalOpening)
{
  auto grid = enclosedGrid(); grid.data[3 * 40 + 3] = -1;
  const auto original = grid.data;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data, original);
}

TEST(FloorSlice, PreservesClosedRoomLargerThanAreaLimit)
{
  auto grid = enclosedGrid(16, 16); const auto original = grid.data;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data, original);
}

TEST(FloorSlice, PreservesLongNarrowEnclosure)
{
  auto grid = enclosedGrid(25, 4); const auto original = grid.data;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data, original);
}

TEST(FloorSlice, NeverFillsUnknownBesideObservedFreeInterior)
{
  auto grid = enclosedGrid(); grid.data[6 * 40 + 8] = 0;
  const auto original = grid.data;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data, original);
}

TEST(FloorSlice, DoesNotFillUnknownAtMapBoundary)
{
  auto grid = enclosedGrid(); grid.data.assign(1600, 100);
  grid.data[0] = -1;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data[0], -1);
}

TEST(FloorSlice, FillingCanBeDisabledAndIsRecomputedForNewEvidence)
{
  auto grid = enclosedGrid(); const auto original = grid;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 0., 1.6), 0u);
  EXPECT_EQ(grid.data, original.data);
  EXPECT_GT(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  grid = original;
  grid.data[3 * 40 + 8] = 0;
  EXPECT_EQ(go2_multifloor_exploration::fillEnclosedUnknown(grid, 1., 1.6), 0u);
  EXPECT_EQ(grid.data[6 * 40 + 8], -1);
}

}  // namespace
