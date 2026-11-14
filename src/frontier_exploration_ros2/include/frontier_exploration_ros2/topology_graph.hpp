#pragma once

#include <cstddef>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

enum class TopologyNodeType
{
  Doorway,
  CorridorEntrance,
};

struct TopologyNode
{
  std::size_t id{0U};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  TopologyNodeType type{TopologyNodeType::CorridorEntrance};
  bool visited{false};
};

struct TopologyConfig
{
  double minimum_entrance_width_m{0.65};
  double doorway_max_width_m{1.8};
  double corridor_max_width_m{3.5};
  double lateral_probe_distance_m{3.0};
  double node_merge_distance_m{1.0};
  double node_merge_height_m{1.0};
  double visit_radius_m{0.8};
};

class TopologyGraph
{
public:
  explicit TopologyGraph(TopologyConfig config = {});

  void score_frontiers(
    FrontierSequence & frontiers,
    const OccupancyGrid2d & map,
    const geometry_msgs::msg::Pose & robot_pose);

  [[nodiscard]] const std::vector<TopologyNode> & nodes() const;
  void clear();

private:
  double entrance_score(
    const FrontierCandidate & frontier,
    const OccupancyGrid2d & map,
    TopologyNodeType & type) const;
  TopologyNode & find_or_create_node(
    const FrontierCandidate & frontier,
    double z,
    TopologyNodeType type);
  void mark_visited(const geometry_msgs::msg::Pose & robot_pose);

  TopologyConfig config_;
  std::vector<TopologyNode> nodes_;
  std::size_t next_id_{1U};
};

}  // namespace frontier_exploration_ros2
