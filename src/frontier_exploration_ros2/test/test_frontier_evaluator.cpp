/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "frontier_exploration_ros2/frontier_evaluator.hpp"

namespace frontier_exploration_ros2
{
namespace
{

nav_msgs::msg::OccupancyGrid build_grid(int width, int height, int value)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = 1.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(
    static_cast<std::size_t>(width * height),
    static_cast<int8_t>(value));
  return grid;
}

void set_cell(nav_msgs::msg::OccupancyGrid & grid, int x, int y, int value)
{
  const auto index = static_cast<std::size_t>(y) * grid.info.width +
    static_cast<std::size_t>(x);
  grid.data[index] = static_cast<int8_t>(value);
}

geometry_msgs::msg::Pose make_pose(double x, double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  return pose;
}

TEST(FrontierEvaluatorTests, OpenMapKeepsReachableCandidateAndStoresPathCost)
{
  const OccupancyGrid2d map(build_grid(7, 5, 0));
  const OccupancyGrid2d costmap(build_grid(7, 5, 0));
  FrontierCandidate candidate{{5.5, 2.5}, {5.5, 2.5}, 8};

  FrontierEvaluator evaluator(ReachabilityConfig{});
  const auto result = evaluator.filter_reachable(
    make_pose(1.5, 2.5), map, costmap, {candidate});

  ASSERT_EQ(result.size(), 1U);
  EXPECT_TRUE(result.front().reachable);
  EXPECT_NEAR(result.front().path_cost, 4.0, 1e-9);
  ASSERT_TRUE(result.front().goal_point.has_value());
  EXPECT_DOUBLE_EQ(result.front().goal_point->first, 5.5);
  EXPECT_DOUBLE_EQ(result.front().goal_point->second, 2.5);
}

TEST(FrontierEvaluatorTests, SolidWallRejectsCandidate)
{
  auto map_message = build_grid(7, 5, 0);
  for (int y = 0; y < 5; ++y) {
    set_cell(map_message, 3, y, 100);
  }
  const OccupancyGrid2d map(map_message);
  const OccupancyGrid2d costmap(build_grid(7, 5, 0));
  FrontierCandidate candidate{{5.5, 2.5}, {5.5, 2.5}, 8};

  FrontierEvaluator evaluator(ReachabilityConfig{});
  const auto result = evaluator.filter_reachable(
    make_pose(1.5, 2.5), map, costmap, {candidate});

  EXPECT_TRUE(result.empty());
}

TEST(FrontierEvaluatorTests, DiagonalPathCannotCutObstacleCorner)
{
  auto map_message = build_grid(3, 3, 0);
  set_cell(map_message, 1, 0, 100);
  set_cell(map_message, 0, 1, 100);
  const OccupancyGrid2d map(map_message);
  const OccupancyGrid2d costmap(build_grid(3, 3, 0));
  FrontierCandidate candidate{{1.5, 1.5}, {1.5, 1.5}, 4};

  FrontierEvaluator evaluator(ReachabilityConfig{});
  const auto result = evaluator.filter_reachable(
    make_pose(0.5, 0.5), map, costmap, {candidate});

  EXPECT_TRUE(result.empty());
}

}  // namespace
}  // namespace frontier_exploration_ros2
