/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

#pragma once

#include <cstddef>
#include <optional>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

struct ReachabilityConfig
{
  int occ_threshold{65};
  double costmap_weight{2.0};
  std::size_t max_expanded_cells{200000U};
  bool allow_diagonal{true};
};

class FrontierEvaluator
{
public:
  explicit FrontierEvaluator(ReachabilityConfig config);

  FrontierSequence filter_reachable(
    const geometry_msgs::msg::Pose & robot_pose,
    const OccupancyGrid2d & occupancy_map,
    const OccupancyGrid2d & global_costmap,
    FrontierSequence candidates) const;

private:
  std::optional<int> traversal_cost(
    const OccupancyGrid2d & occupancy_map,
    const OccupancyGrid2d & global_costmap,
    int map_x,
    int map_y) const;

  std::optional<double> astar_path_cost(
    const OccupancyGrid2d & occupancy_map,
    const OccupancyGrid2d & global_costmap,
    const std::pair<int, int> & start,
    const std::pair<int, int> & goal) const;

  ReachabilityConfig config_;
};

}  // namespace frontier_exploration_ros2
