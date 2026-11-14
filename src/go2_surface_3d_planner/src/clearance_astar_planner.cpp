#include "go2_surface_3d_planner/clearance_astar_planner.hpp"

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace go2_surface_3d_planner
{
namespace
{
struct QueueEntry
{
  double score;
  int index;
  bool operator>(const QueueEntry & other) const {return score > other.score;}
};

constexpr std::array<int, 8> kDx{{1, -1, 0, 0, 1, 1, -1, -1}};
constexpr std::array<int, 8> kDy{{0, 0, 1, -1, 1, -1, 1, -1}};
constexpr double kSqrt2 = 1.4142135623730951;
}  // namespace

void ClearanceAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("ClearanceAStarPlanner failed to lock lifecycle node");
  }
  plugin_name_ = std::move(name);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();

  tolerance_ = node_->declare_parameter<double>(plugin_name_ + ".tolerance", 0.20);
  preferred_clearance_ = node_->declare_parameter<double>(
    plugin_name_ + ".preferred_clearance", 0.35);
  clearance_weight_ = node_->declare_parameter<double>(
    plugin_name_ + ".clearance_weight", 6.0);
  costmap_weight_ = node_->declare_parameter<double>(
    plugin_name_ + ".costmap_weight", 1.0);
  unknown_penalty_ = node_->declare_parameter<double>(
    plugin_name_ + ".unknown_penalty", 3.0);
  max_iterations_ = node_->declare_parameter<int>(
    plugin_name_ + ".max_iterations", 500000);
  allow_unknown_ = node_->declare_parameter<bool>(
    plugin_name_ + ".allow_unknown", true);

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: clearance-aware A*, preferred_clearance=%.2fm, "
    "clearance_weight=%.2f, inflation unchanged",
    plugin_name_.c_str(), preferred_clearance_, clearance_weight_);
}

void ClearanceAStarPlanner::cleanup()
{
  costmap_ = nullptr;
  costmap_ros_.reset();
}

void ClearanceAStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activated %s", plugin_name_.c_str());
}

void ClearanceAStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivated %s", plugin_name_.c_str());
}

bool ClearanceAStarPlanner::traversable(unsigned char cost) const
{
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

int ClearanceAStarPlanner::nearestTraversable(
  unsigned int mx, unsigned int my, double radius,
  const std::vector<unsigned char> & costs,
  unsigned int width, unsigned int height) const
{
  const int cells = std::max(0, static_cast<int>(std::ceil(radius / costmap_->getResolution())));
  int best = -1;
  double best_squared = std::numeric_limits<double>::infinity();
  for (int dy = -cells; dy <= cells; ++dy) {
    for (int dx = -cells; dx <= cells; ++dx) {
      const int x = static_cast<int>(mx) + dx;
      const int y = static_cast<int>(my) + dy;
      if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height)) {
        continue;
      }
      const double squared = static_cast<double>(dx * dx + dy * dy);
      if (squared > static_cast<double>(cells * cells) || squared >= best_squared) {
        continue;
      }
      const int index = y * static_cast<int>(width) + x;
      if (traversable(costs[index])) {
        best = index;
        best_squared = squared;
      }
    }
  }
  return best;
}

std::vector<double> ClearanceAStarPlanner::computeClearance(
  const std::vector<unsigned char> & costs,
  unsigned int width, unsigned int height, double resolution) const
{
  const std::size_t size = static_cast<std::size_t>(width) * height;
  std::vector<double> distance(size, std::numeric_limits<double>::infinity());
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

  for (std::size_t index = 0; index < size; ++index) {
    const auto cost = costs[index];
    if (cost != nav2_costmap_2d::NO_INFORMATION &&
      cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      distance[index] = 0.0;
      queue.push({0.0, static_cast<int>(index)});
    }
  }

  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    if (current.score > distance[current.index]) {
      continue;
    }
    const int x = current.index % static_cast<int>(width);
    const int y = current.index / static_cast<int>(width);
    for (std::size_t direction = 0; direction < kDx.size(); ++direction) {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) ||
        ny >= static_cast<int>(height))
      {
        continue;
      }
      const int next = ny * static_cast<int>(width) + nx;
      const double step = direction < 4 ? resolution : resolution * kSqrt2;
      const double candidate = current.score + step;
      if (candidate < distance[next]) {
        distance[next] = candidate;
        queue.push({candidate, next});
      }
    }
  }
  return distance;
}

nav_msgs::msg::Path ClearanceAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!costmap_) {
    throw nav2_core::PlannerException("ClearanceAStarPlanner is not configured");
  }
  const std::string frame = costmap_ros_->getGlobalFrameID();
  if (start.header.frame_id != frame || goal.header.frame_id != frame) {
    throw nav2_core::PlannerException(
            "ClearanceAStarPlanner requires start and goal in frame '" + frame + "'");
  }

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
  const unsigned int width = costmap_->getSizeInCellsX();
  const unsigned int height = costmap_->getSizeInCellsY();
  const std::size_t size = static_cast<std::size_t>(width) * height;
  const double resolution = costmap_->getResolution();

  unsigned int start_x, start_y, goal_x, goal_y;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y) ||
    !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
  {
    throw nav2_core::PlannerException("ClearanceAStarPlanner start or goal lies outside costmap");
  }

  std::vector<unsigned char> costs(costmap_->getCharMap(), costmap_->getCharMap() + size);
  int start_index = static_cast<int>(start_y * width + start_x);
  int goal_index = static_cast<int>(goal_y * width + goal_x);
  const bool exact_goal_is_traversable = traversable(costs[goal_index]);

  if (!traversable(costs[start_index])) {
    start_index = nearestTraversable(start_x, start_y, tolerance_, costs, width, height);
  }
  if (!traversable(costs[goal_index])) {
    goal_index = nearestTraversable(goal_x, goal_y, tolerance_, costs, width, height);
  }
  if (start_index < 0 || goal_index < 0) {
    throw nav2_core::PlannerException(
            "ClearanceAStarPlanner could not snap start or goal to traversable space");
  }

  const auto clearance = computeClearance(costs, width, height, resolution);
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> g_score(size, infinity);
  std::vector<int> parent(size, -1);
  std::vector<bool> closed(size, false);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;

  const int goal_mx = goal_index % static_cast<int>(width);
  const int goal_my = goal_index / static_cast<int>(width);
  auto heuristic = [&](int index) {
      const int x = index % static_cast<int>(width);
      const int y = index / static_cast<int>(width);
      return std::hypot(x - goal_mx, y - goal_my) * resolution;
    };

  g_score[start_index] = 0.0;
  open.push({heuristic(start_index), start_index});
  int iterations = 0;

  while (!open.empty() && iterations++ < max_iterations_) {
    const int current = open.top().index;
    open.pop();
    if (closed[current]) {
      continue;
    }
    if (current == goal_index) {
      break;
    }
    closed[current] = true;
    const int x = current % static_cast<int>(width);
    const int y = current / static_cast<int>(width);

    for (std::size_t direction = 0; direction < kDx.size(); ++direction) {
      const int nx = x + kDx[direction];
      const int ny = y + kDy[direction];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) ||
        ny >= static_cast<int>(height))
      {
        continue;
      }
      const int next = ny * static_cast<int>(width) + nx;
      if (!traversable(costs[next]) || closed[next]) {
        continue;
      }

      if (direction >= 4) {
        const int side_x = y * static_cast<int>(width) + nx;
        const int side_y = ny * static_cast<int>(width) + x;
        if (!traversable(costs[side_x]) || !traversable(costs[side_y])) {
          continue;
        }
      }

      const double step = direction < 4 ? resolution : resolution * kSqrt2;
      double clearance_penalty = 0.0;
      if (preferred_clearance_ > 0.0 && clearance[next] < preferred_clearance_) {
        const double ratio =
          (preferred_clearance_ - clearance[next]) / preferred_clearance_;
        clearance_penalty = clearance_weight_ * ratio * ratio;
      }

      double map_penalty = 0.0;
      if (costs[next] == nav2_costmap_2d::NO_INFORMATION) {
        map_penalty = unknown_penalty_;
      } else {
        const double normalized = static_cast<double>(costs[next]) / 252.0;
        map_penalty = costmap_weight_ * normalized * normalized;
      }

      double turn_penalty = 0.0;
      if (parent[current] >= 0) {
        const int previous = parent[current];
        const int previous_dx = x - previous % static_cast<int>(width);
        const int previous_dy = y - previous / static_cast<int>(width);
        if (previous_dx != kDx[direction] || previous_dy != kDy[direction]) {
          turn_penalty = 0.05 * resolution;
        }
      }

      const double candidate =
        g_score[current] + step * (1.0 + clearance_penalty + map_penalty) + turn_penalty;
      if (candidate < g_score[next]) {
        g_score[next] = candidate;
        parent[next] = current;
        open.push({candidate + heuristic(next), next});
      }
    }
  }

  if (start_index != goal_index && parent[goal_index] < 0) {
    throw nav2_core::PlannerException("ClearanceAStarPlanner found no path");
  }

  std::vector<int> indices;
  for (int current = goal_index; current >= 0; current = parent[current]) {
    indices.push_back(current);
    if (current == start_index) {
      break;
    }
  }
  if (indices.empty() || indices.back() != start_index) {
    throw nav2_core::PlannerException("ClearanceAStarPlanner path reconstruction failed");
  }
  std::reverse(indices.begin(), indices.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = frame;
  path.header.stamp = node_->now();
  path.poses.reserve(indices.size() + 1);

  for (std::size_t i = 0; i < indices.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    const int mx = indices[i] % static_cast<int>(width);
    const int my = indices[i] / static_cast<int>(width);
    costmap_->mapToWorld(mx, my, pose.pose.position.x, pose.pose.position.y);
    pose.pose.position.z = 0.0;

    double yaw = tf2::getYaw(goal.pose.orientation);
    if (i + 1 < indices.size()) {
      const int next_mx = indices[i + 1] % static_cast<int>(width);
      const int next_my = indices[i + 1] / static_cast<int>(width);
      yaw = std::atan2(next_my - my, next_mx - mx);
    }
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(orientation);
    path.poses.push_back(std::move(pose));
  }

  path.poses.front().pose.position = start.pose.position;
  if (exact_goal_is_traversable) {
    path.poses.back().pose.position = goal.pose.position;
  }
  path.poses.back().pose.orientation = goal.pose.orientation;

  double minimum_clearance = infinity;
  for (const int index : indices) {
    minimum_clearance = std::min(minimum_clearance, clearance[index]);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "Clearance A* path: %zu poses, minimum grid clearance %.2fm, iterations %d",
    path.poses.size(), minimum_clearance, iterations);
  return path;
}

}  // namespace go2_surface_3d_planner

PLUGINLIB_EXPORT_CLASS(
  go2_surface_3d_planner::ClearanceAStarPlanner,
  nav2_core::GlobalPlanner)
