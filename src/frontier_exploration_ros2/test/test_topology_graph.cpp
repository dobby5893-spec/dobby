#include <gtest/gtest.h>

#include <memory>

#include "frontier_exploration_ros2/topology_graph.hpp"

namespace frontier_exploration_ros2
{
namespace
{

OccupancyGrid2d corridor_map()
{
  auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  msg->info.resolution = 0.25;
  msg->info.width = 24;
  msg->info.height = 20;
  msg->info.origin.position.x = 0.0;
  msg->info.origin.position.y = 0.0;
  msg->data.assign(msg->info.width * msg->info.height, 100);
  for (int y = 7; y <= 12; ++y) {
    for (int x = 0; x < 24; ++x) {
      msg->data[static_cast<std::size_t>(y) * msg->info.width + x] = 0;
    }
  }
  return OccupancyGrid2d(msg);
}

FrontierCandidate corridor_frontier()
{
  FrontierCandidate candidate({4.0, 2.5}, {4.0, 2.5}, 20);
  candidate.goal_point = std::pair<double, double>{3.0, 2.5};
  candidate.goal_z = 0.0;
  return candidate;
}

TEST(TopologyGraphTests, UnvisitedStructuredEntranceGetsHighGain)
{
  TopologyGraph graph;
  FrontierSequence candidates{corridor_frontier()};
  geometry_msgs::msg::Pose robot;
  robot.position.x = 0.5;
  robot.position.y = 2.5;
  graph.score_frontiers(candidates, corridor_map(), robot);
  EXPECT_GE(candidates.front().topological_gain, 0.75);
  ASSERT_EQ(graph.nodes().size(), 1U);
  EXPECT_FALSE(graph.nodes().front().visited);
}

TEST(TopologyGraphTests, RevisitedEntranceIsDeprioritized)
{
  TopologyGraph graph;
  geometry_msgs::msg::Pose robot;
  robot.position.x = 0.5;
  robot.position.y = 2.5;
  FrontierSequence first{corridor_frontier()};
  graph.score_frontiers(first, corridor_map(), robot);

  robot.position.x = 3.0;
  FrontierSequence second{corridor_frontier()};
  graph.score_frontiers(second, corridor_map(), robot);
  EXPECT_LT(second.front().topological_gain, 0.2);
  EXPECT_TRUE(graph.nodes().front().visited);
}

TEST(TopologyGraphTests, SameXYOnDifferentFloorCreatesSeparateNode)
{
  TopologyGraph graph;
  geometry_msgs::msg::Pose robot;
  robot.position.x = 0.5;
  robot.position.y = 2.5;
  FrontierSequence lower{corridor_frontier()};
  graph.score_frontiers(lower, corridor_map(), robot);

  FrontierSequence upper{corridor_frontier()};
  upper.front().goal_z = 3.0;
  robot.position.z = 3.0;
  graph.score_frontiers(upper, corridor_map(), robot);
  EXPECT_EQ(graph.nodes().size(), 2U);
}

}  // namespace
}  // namespace frontier_exploration_ros2
