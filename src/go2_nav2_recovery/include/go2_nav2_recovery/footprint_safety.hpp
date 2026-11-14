#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"

namespace go2_nav2_recovery
{
namespace detail
{

struct Point2
{
  double x;
  double y;
};

inline double cross(const Point2 & a, const Point2 & b, const Point2 & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

inline bool onSegment(const Point2 & a, const Point2 & b, const Point2 & p)
{
  constexpr double epsilon = 1e-9;
  return std::abs(cross(a, b, p)) <= epsilon &&
         p.x >= std::min(a.x, b.x) - epsilon &&
         p.x <= std::max(a.x, b.x) + epsilon &&
         p.y >= std::min(a.y, b.y) - epsilon &&
         p.y <= std::max(a.y, b.y) + epsilon;
}

inline bool segmentsIntersect(
  const Point2 & a, const Point2 & b, const Point2 & c, const Point2 & d)
{
  constexpr double epsilon = 1e-9;
  const double ab_c = cross(a, b, c);
  const double ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a);
  const double cd_b = cross(c, d, b);
  if (((ab_c > epsilon && ab_d < -epsilon) || (ab_c < -epsilon && ab_d > epsilon)) &&
      ((cd_a > epsilon && cd_b < -epsilon) || (cd_a < -epsilon && cd_b > epsilon)))
  {
    return true;
  }
  return onSegment(a, b, c) || onSegment(a, b, d) ||
         onSegment(c, d, a) || onSegment(c, d, b);
}

inline bool pointInPolygon(const Point2 & point, const std::vector<Point2> & polygon)
{
  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    if (onSegment(polygon[j], polygon[i], point)) {
      return true;
    }
    const bool crosses =
      ((polygon[i].y > point.y) != (polygon[j].y > point.y)) &&
      (point.x <
      (polygon[j].x - polygon[i].x) * (point.y - polygon[i].y) /
      (polygon[j].y - polygon[i].y) + polygon[i].x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

inline std::array<Point2, 4> cellCorners(
  double minimum_x, double minimum_y, double maximum_x, double maximum_y)
{
  return {{
    {minimum_x, minimum_y}, {maximum_x, minimum_y},
    {maximum_x, maximum_y}, {minimum_x, maximum_y}
  }};
}

inline bool polygonIntersectsCell(
  const std::vector<Point2> & polygon,
  double minimum_x, double minimum_y, double maximum_x, double maximum_y)
{
  for (const auto & point : polygon) {
    if (point.x >= minimum_x && point.x <= maximum_x &&
        point.y >= minimum_y && point.y <= maximum_y)
    {
      return true;
    }
  }

  const auto corners = cellCorners(minimum_x, minimum_y, maximum_x, maximum_y);
  for (const auto & corner : corners) {
    if (pointInPolygon(corner, polygon)) {
      return true;
    }
  }

  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    for (size_t edge = 0; edge < corners.size(); ++edge) {
      if (segmentsIntersect(a, b, corners[edge], corners[(edge + 1) % corners.size()])) {
        return true;
      }
    }
  }
  return false;
}

inline double pointSegmentDistance(
  const Point2 & point, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-12) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  const double t = std::clamp(
    ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared, 0.0, 1.0);
  return std::hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

inline double polygonCellDistance(
  const std::vector<Point2> & polygon,
  double minimum_x, double minimum_y, double maximum_x, double maximum_y)
{
  if (polygonIntersectsCell(polygon, minimum_x, minimum_y, maximum_x, maximum_y)) {
    return 0.0;
  }

  const auto corners = cellCorners(minimum_x, minimum_y, maximum_x, maximum_y);
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    for (const auto & corner : corners) {
      best = std::min(best, pointSegmentDistance(corner, a, b));
    }
  }
  for (const auto & point : polygon) {
    for (size_t edge = 0; edge < corners.size(); ++edge) {
      best = std::min(
        best, pointSegmentDistance(point, corners[edge], corners[(edge + 1) % corners.size()]));
    }
  }
  return best;
}

}  // namespace detail

struct FootprintSafety
{
  int hard_overlap_cells{0};
  int first_hard_x{-1}, first_hard_y{-1}, first_hard_cost{-1};
  double first_hard_world_x{0.0}, first_hard_world_y{0.0};
  bool outside_map{false};
  double clearance{-1.0};       // Distance to cost 253 or higher.
  double hard_clearance{-1.0};  // Distance to lethal/unknown cells only.

  bool collisionFree() const
  {
    return !outside_map && hard_overlap_cells == 0;
  }
};

// Reports both clearance and the number of hard cells intersecting the padded
// footprint. The overlap count gives recovery a gradient even when the robot
// already starts in a lethal or unknown cell.
inline FootprintSafety assessFootprint(
  nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw, double margin = 0.05)
{
  FootprintSafety result;
  if (footprint.size() < 3 || !std::isfinite(x + y + yaw) || margin < 0.0) {
    result.outside_map = true;
    return result;
  }

  auto padded = footprint;
  nav2_costmap_2d::padFootprint(padded, margin);

  std::vector<detail::Point2> polygon;
  polygon.reserve(padded.size());
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();

  for (const auto & point : padded) {
    const detail::Point2 world{
      x + cosine * point.x - sine * point.y,
      y + sine * point.x + cosine * point.y};
    unsigned int map_x;
    unsigned int map_y;
    if (!map.worldToMap(world.x, world.y, map_x, map_y)) {
      result.outside_map = true;
      return result;
    }
    polygon.push_back(world);
    minimum_x = std::min(minimum_x, world.x);
    minimum_y = std::min(minimum_y, world.y);
    maximum_x = std::max(maximum_x, world.x);
    maximum_y = std::max(maximum_y, world.y);
  }

  constexpr double search_distance = 0.60;
  int low_x;
  int low_y;
  int high_x;
  int high_y;
  map.worldToMapEnforceBounds(
    minimum_x - search_distance, minimum_y - search_distance, low_x, low_y);
  map.worldToMapEnforceBounds(
    maximum_x + search_distance, maximum_y + search_distance, high_x, high_y);

  double best = search_distance;
  double best_hard = search_distance;
  const double resolution = map.getResolution();
  const double origin_x = map.getOriginX();
  const double origin_y = map.getOriginY();

  for (int map_y = low_y; map_y <= high_y; ++map_y) {
    for (int map_x = low_x; map_x <= high_x; ++map_x) {
      const unsigned char cost = map.getCost(map_x, map_y);
      if (cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        continue;
      }

      const double cell_minimum_x = origin_x + map_x * resolution;
      const double cell_minimum_y = origin_y + map_y * resolution;
      const double cell_maximum_x = cell_minimum_x + resolution;
      const double cell_maximum_y = cell_minimum_y + resolution;
      const bool intersects = detail::polygonIntersectsCell(
        polygon, cell_minimum_x, cell_minimum_y, cell_maximum_x, cell_maximum_y);

      const bool hard_cell =
        cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::NO_INFORMATION;
      if (intersects && hard_cell) {
        if (result.hard_overlap_cells == 0) {
          result.first_hard_x = map_x;
          result.first_hard_y = map_y;
          result.first_hard_cost = cost;
          result.first_hard_world_x = cell_minimum_x + resolution * 0.5;
          result.first_hard_world_y = cell_minimum_y + resolution * 0.5;
        }
        ++result.hard_overlap_cells;
      }

      const double distance = detail::polygonCellDistance(
        polygon, cell_minimum_x, cell_minimum_y, cell_maximum_x, cell_maximum_y);
      best = std::min(best, distance);
      if (hard_cell) {
        best_hard = std::min(best_hard, distance);
      }
    }
  }
  result.clearance = result.hard_overlap_cells == 0 ? best : -1.0;
  result.hard_clearance = result.hard_overlap_cells == 0 ? best_hard : -1.0;
  return result;
}

// Returns the distance from the padded, oriented robot polygon to cost 253.
// A negative value means that the polygon intersects a lethal (254) or
// unknown (255) costmap cell, or extends outside the costmap.
inline double clearance(
  nav2_costmap_2d::Costmap2D & map,
  const std::vector<geometry_msgs::msg::Point> & footprint,
  double x, double y, double yaw, double margin = 0.05)
{
  return assessFootprint(map, footprint, x, y, yaw, margin).clearance;
}

}  // namespace go2_nav2_recovery
