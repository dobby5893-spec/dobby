/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

#include "frontier_exploration_ros2/frontier_evaluator.hpp"
#include "frontier_exploration_ros2/viewpoint_sampling.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace frontier_exploration_ros2
{

namespace
{

struct QueueEntry
{
  double estimated_total_cost{0.0};
  double path_cost{0.0};
  int map_x{0};
  int map_y{0};
};

struct QueueEntryGreater
{
  bool operator()(const QueueEntry & left, const QueueEntry & right) const
  {
    return left.estimated_total_cost > right.estimated_total_cost;
  }
};

constexpr std::array<std::pair<int, int>, 8> kNeighborOffsets{{
  {-1, 0},
  {1, 0},
  {0, -1},
  {0, 1},
  {-1, -1},
  {-1, 1},
  {1, -1},
  {1, 1},
}};

}  // namespace

FrontierEvaluator::FrontierEvaluator(ReachabilityConfig config)
: config_(std::move(config))
{
  config_.occ_threshold = std::clamp(config_.occ_threshold, 0, 100);
  config_.costmap_weight = std::max(0.0, config_.costmap_weight);
  config_.max_expanded_cells = std::max<std::size_t>(1U, config_.max_expanded_cells);
}

std::optional<int> FrontierEvaluator::traversal_cost(
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & global_costmap,
  int map_x,
  int map_y) const
{
  if (
    map_x < 0 || map_y < 0 ||
    map_x >= occupancy_map.getSizeX() || map_y >= occupancy_map.getSizeY())
  {
    return std::nullopt;
  }

  if (
    occupancy_map.getCost(map_x, map_y) !=
    static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
  {
    return std::nullopt;
  }

  const auto world = occupancy_map.mapToWorld(map_x, map_y);
  int costmap_x = 0;
  int costmap_y = 0;
  if (!global_costmap.worldToMapNoThrow(
      world.first, world.second, costmap_x, costmap_y))
  {
    return std::nullopt;
  }

  const int cost = global_costmap.getCost(costmap_x, costmap_y);
  if (cost < 0 || cost >= config_.occ_threshold) {
    return std::nullopt;
  }
  return cost;
}

std::optional<double> FrontierEvaluator::astar_path_cost(
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & global_costmap,
  const std::pair<int, int> & start,
  const std::pair<int, int> & goal) const
{
  if (!traversal_cost(
      occupancy_map, global_costmap, start.first, start.second).has_value() ||
    !traversal_cost(
      occupancy_map, global_costmap, goal.first, goal.second).has_value())
  {
    return std::nullopt;
  }

  const int width = occupancy_map.getSizeX();
  const int height = occupancy_map.getSizeY();
  const double resolution = occupancy_map.map().info.resolution;
  const auto index_of = [width](int x, int y) {
      return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(x);
    };
  const auto heuristic = [&goal, resolution](int x, int y) {
      return std::hypot(
        static_cast<double>(goal.first - x),
        static_cast<double>(goal.second - y)) * resolution;
    };

  const std::size_t cell_count =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<double> best_cost(cell_count, std::numeric_limits<double>::infinity());
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater> open_queue;

  best_cost[index_of(start.first, start.second)] = 0.0;
  open_queue.push(QueueEntry{
    heuristic(start.first, start.second), 0.0, start.first, start.second});

  std::size_t expanded_cells = 0U;
  while (!open_queue.empty()) {
    const QueueEntry current = open_queue.top();
    open_queue.pop();
    const std::size_t current_index = index_of(current.map_x, current.map_y);
    if (current.path_cost > best_cost[current_index] + 1e-9) {
      continue;
    }
    if (current.map_x == goal.first && current.map_y == goal.second) {
      return current.path_cost;
    }
    ++expanded_cells;
    if (expanded_cells > config_.max_expanded_cells) {
      return std::nullopt;
    }

    for (const auto & offset : kNeighborOffsets) {
      const int next_x = current.map_x + offset.first;
      const int next_y = current.map_y + offset.second;
      const bool diagonal = offset.first != 0 && offset.second != 0;
      if (diagonal && !config_.allow_diagonal) {
        continue;
      }
      const auto next_cost = traversal_cost(
        occupancy_map, global_costmap, next_x, next_y);
      if (!next_cost.has_value()) {
        continue;
      }
      if (diagonal &&
        (!traversal_cost(
          occupancy_map, global_costmap,
          current.map_x + offset.first, current.map_y).has_value() ||
        !traversal_cost(
          occupancy_map, global_costmap,
          current.map_x, current.map_y + offset.second).has_value()))
      {
        continue;
      }

      const double step_distance = diagonal ? resolution * std::sqrt(2.0) : resolution;
      const double normalized_cost = static_cast<double>(*next_cost) / 100.0;
      const double candidate_cost = current.path_cost +
        step_distance * (1.0 + config_.costmap_weight * normalized_cost);
      const std::size_t next_index = index_of(next_x, next_y);
      if (candidate_cost + 1e-9 >= best_cost[next_index]) {
        continue;
      }
      best_cost[next_index] = candidate_cost;
      open_queue.push(QueueEntry{
        candidate_cost + heuristic(next_x, next_y), candidate_cost, next_x, next_y});
    }
  }
  return std::nullopt;
}

FrontierSequence FrontierEvaluator::filter_reachable(
  const geometry_msgs::msg::Pose & robot_pose,
  const OccupancyGrid2d & occupancy_map,
  const OccupancyGrid2d & global_costmap,
  FrontierSequence candidates) const
{
  int start_x = 0;
  int start_y = 0;
  if (!occupancy_map.worldToMapNoThrow(
      robot_pose.position.x, robot_pose.position.y, start_x, start_y))
  {
    return {};
  }

  // One bounded Dijkstra field serves all viewpoints and frontiers. Running
  // A* up to 25 times per frontier would stall the exploration callback.
  const int width=occupancy_map.getSizeX(),height=occupancy_map.getSizeY();
  const auto index=[&](int x,int y){return static_cast<size_t>(y)*width+x;};
  std::vector<double> distances(static_cast<size_t>(width)*height,
    std::numeric_limits<double>::infinity());
  std::vector<int> costs(distances.size(),-1);
  for(int y=0;y<height;++y)for(int x=0;x<width;++x) {
    const auto cost=traversal_cost(occupancy_map,global_costmap,x,y);
    if(cost)costs[index(x,y)]=*cost;
  }
  if(costs[index(start_x,start_y)]<0)return {};
  std::priority_queue<QueueEntry,std::vector<QueueEntry>,QueueEntryGreater> open;
  distances[index(start_x,start_y)]=0;open.push({0,0,start_x,start_y});
  size_t expanded=0;
  const double resolution=occupancy_map.map().info.resolution;
  while(!open.empty() && expanded<config_.max_expanded_cells) {
    const auto cur=open.top();open.pop();
    if(cur.path_cost>distances[index(cur.map_x,cur.map_y)]+1e-9)continue;
    ++expanded;
    for(auto [dx,dy]:kNeighborOffsets) {
      const int x=cur.map_x+dx,y=cur.map_y+dy;
      const bool diagonal=dx && dy;
      if(x<0 || y<0 || x>=width || y>=height || costs[index(x,y)]<0 ||
        (diagonal && !config_.allow_diagonal))continue;
      if(diagonal && (costs[index(cur.map_x+dx,cur.map_y)]<0 ||
        costs[index(cur.map_x,cur.map_y+dy)]<0))continue;
      const double next=cur.path_cost+resolution*(diagonal?std::sqrt(2.):1.)*
        (1.+config_.costmap_weight*costs[index(x,y)]/100.);
      if(next+1e-9>=distances[index(x,y)])continue;
      distances[index(x,y)]=next;open.push({next,next,x,y});
    }
  }
  FrontierSequence reachable_candidates;
  reachable_candidates.reserve(candidates.size());
  for (auto & candidate : candidates) {
    if(candidate.viewpoint_search_exhausted)continue;
    candidate.reachable = false;
    candidate.path_cost = std::numeric_limits<double>::infinity();
    std::vector<std::pair<double, double>> goals = candidate.viewpoint_candidates;
    if (goals.empty() && candidate.fallback_viewpoint_candidates.empty() && candidate.goal_point.has_value()) {
      goals.push_back(*candidate.goal_point);
    }

    const auto & q=robot_pose.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    std::optional<std::pair<double, double>> best_goal;
    double best_path_cost=std::numeric_limits<double>::infinity();
    auto rank_reachable=[&](const std::vector<std::pair<double,double>> & points) {
      struct Ranked {std::pair<double,double> point;double cost,score;};
      std::vector<Ranked> ranked;
      for(const auto & goal:points) {
        int x=0,y=0;
        if(!occupancy_map.worldToMapNoThrow(goal.first,goal.second,x,y))continue;
        const double cost=distances[index(x,y)];
        if(!std::isfinite(cost))continue;
        const int visible=viewpointVisibleGain(occupancy_map,goal,candidate.centroid);
        if(candidate.requires_visible_viewpoint && visible==0)continue;
        const double turn=std::atan2(goal.second-robot_pose.position.y,
          goal.first-robot_pose.position.x)-yaw;
        ranked.push_back({goal,cost,viewpointUtility(
          visible,cost,turn)});
      }
      std::stable_sort(ranked.begin(),ranked.end(),
        [](const auto & a,const auto & b){return a.score>b.score;});
      std::vector<std::pair<double,double>> result;
      for(const auto & item:ranked)result.push_back(item.point);
      if(!ranked.empty()) {best_goal=ranked.front().point;best_path_cost=ranked.front().cost;}
      return result;
    };
    candidate.viewpoint_candidates=rank_reachable(goals);
    const auto primary_goal=best_goal;
    const double primary_cost=best_path_cost;
    candidate.fallback_viewpoint_candidates=rank_reachable(candidate.fallback_viewpoint_candidates);
    if(primary_goal) {best_goal=primary_goal;best_path_cost=primary_cost;}
    // Keep second-round geometry for 3D validation if all reachable primary
    // viewpoints fail; connected dispatch rechecks every alternative.
    if(!best_goal)continue;
    candidate.goal_point=*best_goal;
    candidate.path_cost=best_path_cost;
    candidate.reachable = true;
    reachable_candidates.push_back(std::move(candidate));
  }
  return reachable_candidates;
}

}  // namespace frontier_exploration_ros2
