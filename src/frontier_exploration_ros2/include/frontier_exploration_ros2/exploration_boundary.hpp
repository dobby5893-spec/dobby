#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace frontier_exploration_ros2
{

using ExplorationBoundaryPolygon = std::vector<std::pair<double, double>>;

inline double point_to_segment_distance(
  double px, double py,
  const std::pair<double, double> & a,
  const std::pair<double, double> & b)
{
  const double dx = b.first - a.first;
  const double dy = b.second - a.second;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(px - a.first, py - a.second);
  }
  const double t = std::clamp(
    ((px - a.first) * dx + (py - a.second) * dy) / length_sq,
    0.0, 1.0);
  return std::hypot(px - (a.first + t * dx), py - (a.second + t * dy));
}

// Empty polygons intentionally disable the boundary for backwards compatibility.
// For an enabled polygon, margin keeps accepted points away from every polygon edge.
inline bool is_inside_exploration_boundary(
  double x, double y,
  const ExplorationBoundaryPolygon & polygon,
  double margin_m = 0.0)
{
  if (polygon.empty()) {
    return true;
  }
  if (polygon.size() < 3 || !std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }

  bool inside = false;
  double edge_distance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[j];
    const auto & b = polygon[i];
    if (!std::isfinite(a.first) || !std::isfinite(a.second) ||
      !std::isfinite(b.first) || !std::isfinite(b.second))
    {
      return false;
    }
    edge_distance = std::min(edge_distance, point_to_segment_distance(x, y, a, b));
    const bool crosses = ((a.second > y) != (b.second > y)) &&
      (x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first);
    if (crosses) {
      inside = !inside;
    }
  }

  const double effective_margin = std::max(0.0, margin_m);
  return inside && edge_distance + 1e-9 >= effective_margin;
}

inline bool is_inside_exploration_boundary(
  const std::pair<double, double> & point,
  const ExplorationBoundaryPolygon & polygon,
  double margin_m = 0.0)
{
  return is_inside_exploration_boundary(point.first, point.second, polygon, margin_m);
}

// The inset restricts goal positions; it is not an obstacle layer.
// Transit may use the margin band, but never leave the outer polygon.
// Full-body collision and support checks remain separate and mandatory.
inline double exploration_boundary_depth(
  const std::pair<double,double> & p,const ExplorationBoundaryPolygon & polygon)
{
  if(polygon.empty())return std::numeric_limits<double>::infinity();
  if(!is_inside_exploration_boundary(p,polygon,0.))return -1.;
  double depth=std::numeric_limits<double>::infinity();
  for(size_t i=0;i<polygon.size();++i)
    depth=std::min(depth,point_to_segment_distance(p.first,p.second,
      polygon[i],polygon[(i+1)%polygon.size()]));
  return depth;
}

inline bool boundary_return_step(
  const std::pair<double,double> & from,const std::pair<double,double> & to,
  const ExplorationBoundaryPolygon & polygon,double margin)
{
  if(polygon.empty())return true;
  const double a=exploration_boundary_depth(from,polygon);
  if(a<0)return false;
  const double distance=std::hypot(to.first-from.first,to.second-from.second);
  const int steps=std::max(1,static_cast<int>(std::ceil(distance/.02)));
  (void)margin;
  for(int i=1;i<=steps;++i) {
    const double t=double(i)/steps;
    const double depth=exploration_boundary_depth(
      {from.first+t*(to.first-from.first),from.second+t*(to.second-from.second)},polygon);
    if(depth<0)return false;
  }
  return true;
}

// Separate exception for accidental, small mission-boundary excursions.
// Occupancy/support checks are never relaxed. Outside distance cannot increase;
// after entering the polygon a path cannot leave it again.
inline double boundary_outside_distance(std::pair<double,double> p,
 const ExplorationBoundaryPolygon & polygon) {
 if(polygon.empty() || is_inside_exploration_boundary(p,polygon,0.))return 0.;
 double distance=std::numeric_limits<double>::infinity();
 for(size_t i=0;i<polygon.size();++i)
  distance=std::min(distance,point_to_segment_distance(p.first,p.second,
    polygon[i],polygon[(i+1)%polygon.size()]));
 return distance;
}
inline bool boundary_reentry_step(std::pair<double,double> from,
 std::pair<double,double> to,const ExplorationBoundaryPolygon & polygon) {
 double previous=boundary_outside_distance(from,polygon);
 if(!std::isfinite(previous) || previous>.75+1e-9)return false;
 int steps=std::max(1,int(std::ceil(std::hypot(to.first-from.first,to.second-from.second)/.02)));
 for(int i=1;i<=steps;++i) {
  double t=double(i)/steps;
  double distance=boundary_outside_distance(
    {from.first+t*(to.first-from.first),from.second+t*(to.second-from.second)},polygon);
  if(!std::isfinite(distance) || distance>previous+1e-6)return false;
  previous=distance;
 }
 return true;
}
template<class Path,class Point>
inline bool boundary_reentry_path(const Point & start,const Path & path,
 const ExplorationBoundaryPolygon & polygon,double margin) {
 std::pair<double,double> previous{start.x,start.y};
 for(auto p:path) {
  std::pair<double,double> next{p.x,p.y};
  if(!boundary_reentry_step(previous,next,polygon))return false;
  previous=next;
 }
 return !path.empty() && is_inside_exploration_boundary(previous,polygon,margin);
}

template<class Path,class Point>
inline bool boundary_return_path(const Point & start,const Path & path,
  const ExplorationBoundaryPolygon & polygon,double margin)
{
  std::pair<double,double> previous{start.x,start.y};
  for(const auto & p:path) {
    const std::pair<double,double> next{p.x,p.y};
    if(!boundary_return_step(previous,next,polygon,margin))return false;
    previous=next;
  }
  return !path.empty() && is_inside_exploration_boundary(previous,polygon,margin);
}

}  // namespace frontier_exploration_ros2
