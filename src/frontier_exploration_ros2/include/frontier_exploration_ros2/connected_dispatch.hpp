#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{
// Conservative known-free connectivity on the global costmap. No start-cell snapping.
class ConnectedDispatch
{
public:
  std::string failure;
  ConnectedDispatch(
    const OccupancyGrid2d & global, const OccupancyGrid2d & occupancy,
    const std::optional<OccupancyGrid2d> & local,
    const geometry_msgs::msg::Pose & robot, const std::string & frame,
    double clearance, int threshold,
    const ExplorationBoundaryPolygon & boundary_polygon = {},
    double boundary_margin_m = 0.0, bool allow_reentry = false)
  : global_(global), robot_(robot), boundary_(boundary_polygon), margin_(boundary_margin_m)
  {
    if (!valid(global, frame) || !valid(occupancy, frame) ||
      (local && !valid(*local, frame)) || !std::isfinite(clearance) || clearance < 0.0)
    {
      failure = "invalid grid geometry/frame or clearance";
      return;
    }
    width_ = global.getSizeX();
    height_ = global.getSizeY();
    reachable_.assign(static_cast<std::size_t>(width_) * height_, 0);
    std::vector<unsigned char> free(reachable_.size(), 0);
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const auto p = global.mapToWorld(x, y);
        free[index(x, y)] =
          (is_inside_exploration_boundary(p, boundary_polygon, 0.0) ||
           (allow_reentry && boundary_outside_distance(p,boundary_polygon)<=.75)) &&
          clear(global, p, clearance, threshold, false) &&
          clear(occupancy, p, clearance, threshold, false) &&
          (!local || clear(*local, p, clearance, threshold, true));
      }
    }
    int sx = 0, sy = 0;
    if (!global.worldToMapNoThrow(robot.position.x, robot.position.y, sx, sy)) {
      failure = "robot outside global costmap";
      return;
    }
    if(!is_inside_exploration_boundary(
        {robot.position.x,robot.position.y},boundary_,0.) &&
       !(allow_reentry && boundary_outside_distance(
          {robot.position.x,robot.position.y},boundary_)<=.75)) {
      failure="robot outside outer exploration boundary";
      return;
    }
    if (!free[index(sx, sy)]) {
      failure = "robot start occupied, unknown, or lacks required grid clearance";
      return;
    }
    std::vector<std::pair<int, int>> queue{{sx, sy}};
    reachable_[index(sx, sy)] = 1;
    constexpr int dx[4] = {1, -1, 0, 0};
    constexpr int dy[4] = {0, 0, 1, -1};
    for (std::size_t head = 0; head < queue.size(); ++head) {
      const auto cell = queue[head];
      for (int k = 0; k < 4; ++k) {
        const int x = cell.first + dx[k], y = cell.second + dy[k];
        if (x < 0 || y < 0 || x >= width_ || y >= height_) {continue;}
        const auto i = index(x, y);
        const auto from=(head==0)?
          std::make_pair(robot_.position.x,robot_.position.y):
          global.mapToWorld(cell.first,cell.second);
        if (free[i] && !reachable_[i] &&
            (allow_reentry?boundary_reentry_step(from,global.mapToWorld(x,y),boundary_):
             boundary_return_step(from,global.mapToWorld(x,y),boundary_,margin_))) {
          reachable_[i] = 1;
          queue.emplace_back(x, y);
        }
      }
    }
  }

  std::optional<std::pair<double, double>> choose(
    const std::pair<double, double> & target, double search_radius,
    double min_robot_distance) const
  {
    if (
  	!failure.empty() ||
  	!std::isfinite(target.first) ||
  	!std::isfinite(target.second) ||
	  !std::isfinite(search_radius) ||
	  search_radius <= 0.0 ||
	  !std::isfinite(min_robot_distance) ||
	  min_robot_distance < 0.0)
	{
	  return std::nullopt;
	}

	// If the original target already belongs to the robot-reachable component,
	// preserve its exact world coordinates instead of snapping it to the cell center.
	int target_x = 0;
	int target_y = 0;
	if (
	  global_.worldToMapNoThrow(
	    target.first, target.second, target_x, target_y) &&
	  reachable_[index(target_x, target_y)] &&
      is_inside_exploration_boundary(target,boundary_,margin_) &&
	  std::hypot(
	    target.first - robot_.position.x,
	    target.second - robot_.position.y) >= min_robot_distance)
	{
	  return target;
	}

	std::optional<std::pair<double, double>> best;
    double best_distance = search_radius;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (!reachable_[index(x, y)]) {continue;}
        const auto p = global_.mapToWorld(x, y);
        if(!is_inside_exploration_boundary(p,boundary_,margin_))continue;
        const double d = std::hypot(p.first - target.first, p.second - target.second);
        if (d <= best_distance && std::hypot(
            p.first - robot_.position.x, p.second - robot_.position.y) >= min_robot_distance)
        {
          best = p;
          best_distance = d;
        }
      }
    }
    return best;
  }

private:
  static bool valid(const OccupancyGrid2d & grid, const std::string & frame)
  {
    if (!grid.valid()) {return false;}
    const auto & m = grid.map();
    const auto & q = m.info.origin.orientation;
    return m.header.frame_id == frame && m.info.width > 0 && m.info.height > 0 &&
      std::isfinite(m.info.resolution) && m.info.resolution > 0 &&
      std::isfinite(m.info.origin.position.x) && std::isfinite(m.info.origin.position.y) &&
      m.data.size() == static_cast<std::size_t>(m.info.width) * m.info.height &&
      std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6 &&
      std::abs(std::abs(q.w) - 1.0) < 1e-6;
  }

  static bool clear(const OccupancyGrid2d & grid, const std::pair<double, double> & p,
    double radius, int threshold, bool partial)
  {
    const auto & info = grid.map().info;
    const double r = info.resolution;
    const int cx = static_cast<int>(std::floor((p.first - info.origin.position.x) / r));
    const int cy = static_cast<int>(std::floor((p.second - info.origin.position.y) / r));
    const int n = static_cast<int>(std::ceil(radius / r)) + 1;
    for (int y = cy - n; y <= cy + n; ++y) {
      for (int x = cx - n; x <= cx + n; ++x) {
        // Distance to the square cell, not merely its center.
        const auto c = grid.mapToWorld(x, y);
        const double dx = std::max(0.0, std::abs(c.first - p.first) - r * 0.5);
        const double dy = std::max(0.0, std::abs(c.second - p.second) - r * 0.5);
        if (dx * dx + dy * dy > radius * radius) {continue;}
        if (x < 0 || y < 0 || x >= grid.getSizeX() || y >= grid.getSizeY()) {
          if (partial) {continue;}
          return false;
        }
        const int cost = grid.getCost(x, y);
        if (cost < 0 || cost >= threshold) {return false;}
      }
    }
    return true;
  }
  std::size_t index(int x, int y) const {return static_cast<std::size_t>(y) * width_ + x;}
  const OccupancyGrid2d & global_;
  geometry_msgs::msg::Pose robot_;
  ExplorationBoundaryPolygon boundary_;
  double margin_{0};
  int width_{0}, height_{0};
  std::vector<unsigned char> reachable_;
};
}  // namespace frontier_exploration_ros2
