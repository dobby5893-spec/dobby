#include "frontier_exploration_ros2/topology_graph.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace frontier_exploration_ros2
{

TopologyGraph::TopologyGraph(TopologyConfig config)
: config_(std::move(config))
{
  config_.minimum_entrance_width_m = std::max(0.1, config_.minimum_entrance_width_m);
  config_.doorway_max_width_m = std::max(
    config_.minimum_entrance_width_m, config_.doorway_max_width_m);
  config_.corridor_max_width_m = std::max(
    config_.doorway_max_width_m, config_.corridor_max_width_m);
  config_.lateral_probe_distance_m = std::max(
    config_.corridor_max_width_m, config_.lateral_probe_distance_m);
  config_.node_merge_distance_m = std::max(0.1, config_.node_merge_distance_m);
  config_.node_merge_height_m = std::max(0.1, config_.node_merge_height_m);
  config_.visit_radius_m = std::max(0.1, config_.visit_radius_m);
}

double TopologyGraph::entrance_score(
  const FrontierCandidate & frontier,
  const OccupancyGrid2d & map,
  TopologyNodeType & type) const
{
  const auto goal = frontier.goal_point.value_or(frontier.center_point);
  const double heading = std::atan2(
    frontier.centroid.second - goal.second,
    frontier.centroid.first - goal.first);
  const double lateral_x = -std::sin(heading);
  const double lateral_y = std::cos(heading);
  const double step = std::max(0.05, static_cast<double>(map.map().info.resolution));

  const auto distance_to_boundary = [&](double sign) {
      for (double distance = step; distance <= config_.lateral_probe_distance_m; distance += step) {
        int map_x = 0;
        int map_y = 0;
        const double x = goal.first + sign * lateral_x * distance;
        const double y = goal.second + sign * lateral_y * distance;
        if (!map.worldToMapNoThrow(x, y, map_x, map_y)) {
          return std::pair<bool, double>{false, distance};
        }
        if (map.getCost(map_x, map_y) != static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace)) {
          return std::pair<bool, double>{true, distance};
        }
      }
      return std::pair<bool, double>{false, config_.lateral_probe_distance_m};
    };

  const auto left = distance_to_boundary(1.0);
  const auto right = distance_to_boundary(-1.0);
  if (!left.first || !right.first) {
    type = TopologyNodeType::CorridorEntrance;
    return 0.15;
  }

  const double width = left.second + right.second;
  if (width < config_.minimum_entrance_width_m) {
    return 0.0;
  }
  if (width <= config_.doorway_max_width_m) {
    type = TopologyNodeType::Doorway;
    return 1.0;
  }
  if (width <= config_.corridor_max_width_m) {
    type = TopologyNodeType::CorridorEntrance;
    return 0.75;
  }
  type = TopologyNodeType::CorridorEntrance;
  return 0.30;
}

TopologyNode & TopologyGraph::find_or_create_node(
  const FrontierCandidate & frontier,
  double z,
  TopologyNodeType type)
{
  const auto goal = frontier.goal_point.value_or(frontier.center_point);
  for (auto & node : nodes_) {
    if (
      std::hypot(node.x - goal.first, node.y - goal.second) <= config_.node_merge_distance_m &&
      std::abs(node.z - z) <= config_.node_merge_height_m)
    {
      node.type = type;
      return node;
    }
  }
  nodes_.push_back(TopologyNode{next_id_++, goal.first, goal.second, z, type, false});
  return nodes_.back();
}

void TopologyGraph::mark_visited(const geometry_msgs::msg::Pose & robot_pose)
{
  for (auto & node : nodes_) {
    if (
      std::hypot(node.x - robot_pose.position.x, node.y - robot_pose.position.y) <=
        config_.visit_radius_m &&
      std::abs(node.z - robot_pose.position.z) <= config_.node_merge_height_m)
    {
      node.visited = true;
    }
  }
}

void TopologyGraph::score_frontiers(
  FrontierSequence & frontiers,
  const OccupancyGrid2d & map,
  const geometry_msgs::msg::Pose & robot_pose)
{
  mark_visited(robot_pose);
  for (auto & frontier : frontiers) {
    TopologyNodeType type = TopologyNodeType::CorridorEntrance;
    const double structural_score = entrance_score(frontier, map, type);
    if (structural_score < 0.5) {
      frontier.topological_gain = structural_score;
      continue;
    }
    const double z = frontier.goal_z.value_or(robot_pose.position.z);
    TopologyNode & node = find_or_create_node(frontier, z, type);
    frontier.topological_gain = node.visited ? 0.1 * structural_score : structural_score;
  }
}

const std::vector<TopologyNode> & TopologyGraph::nodes() const
{
  return nodes_;
}

void TopologyGraph::clear()
{
  nodes_.clear();
  next_id_ = 1U;
}

}  // namespace frontier_exploration_ros2
