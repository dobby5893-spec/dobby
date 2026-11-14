#pragma once
#include "go2_surface_3d_planner/surface_graph.hpp"
#include <algorithm>
#include <cmath>

namespace frontier_exploration_ros2
{
// Retry fallback: a short, fully verified approach toward the requested area.
// Endpoints alone are insufficient; every returned point has a checked route.
inline std::optional<go2_surface_3d_planner::SurfacePoint> selectNearbyObservation(
  const octomap::OcTree & tree, go2_surface_3d_planner::SurfaceSearchConfig config,
  const go2_surface_3d_planner::SurfacePoint & start,
  const go2_surface_3d_planner::SurfacePoint & requested, double max_retreat)
{
  using namespace go2_surface_3d_planner;
  if (config.infer_flat_support) {configureSupportPlane(config, tree, start);}
  auto stance_start=start;
  if(config.infer_flat_support) {stance_start.z-=config.support_reference_height;}
  const auto origin=SurfaceGraph(tree,config).snapToSurface(stance_start);
  if (!origin) {return std::nullopt;}
  const double heading=std::atan2(requested.y-start.y,requested.x-start.x);
  const std::vector<double> offsets{0.0, M_PI/6, -M_PI/6, M_PI/3, -M_PI/3,
    5*M_PI/12, -5*M_PI/12, M_PI/2, -M_PI/2};
  // Check short swept motions before spending the batch on detour searches.
  // Lateral approaches can leave a corner while still reducing target distance.
  for (double margin : {config.clearance_margin, config.narrow_clearance_margin}) {
    if (margin <= 0 || margin > config.clearance_margin) {continue;}
    auto direct_config=config;
    direct_config.clearance_margin=std::max(0.01,margin);
    SurfaceGraph graph(tree,direct_config);
    for (double distance : {0.75, 1.25}) {
      for (double offset : offsets) {
        if (std::chrono::steady_clock::now() >= config.search_deadline) {return std::nullopt;}
        const SurfacePoint candidate{start.x+distance*std::cos(heading+offset),
          start.y+distance*std::sin(heading+offset),origin->z,start.yaw};
        const auto snapped=graph.snapToSurface(candidate);
        if(!snapped || std::hypot(snapped->x-start.x,snapped->y-start.y)<0.50 ||
          std::hypot(snapped->x-requested.x,snapped->y-requested.y)>max_retreat ||
          std::hypot(snapped->x-requested.x,snapped->y-requested.y)>
            std::hypot(start.x-requested.x,start.y-requested.y)-0.02) {continue;}
        if(graph.isMotionValid(stance_start,*snapped)) {return snapped;}
      }
    }
  }
  for (double distance : {0.75, 1.25}) {
    for (double offset : offsets) {
      if (std::chrono::steady_clock::now() >= config.search_deadline) {return std::nullopt;}
      const SurfacePoint candidate{start.x+distance*std::cos(heading+offset),
        start.y+distance*std::sin(heading+offset),origin->z,start.yaw};
      if (std::hypot(candidate.x-requested.x,candidate.y-requested.y)>max_retreat) {continue;}
      auto attempt=config;
      attempt.search_deadline=std::min(config.search_deadline,
        std::chrono::steady_clock::now()+std::chrono::milliseconds(400));
      try {
        const auto route=SurfaceGraph(tree,attempt).plan(stance_start,candidate);
        if (!route.empty() && std::hypot(route.back().x-start.x,route.back().y-start.y)>=0.50 &&
          std::hypot(route.back().x-requested.x,route.back().y-requested.y)<=max_retreat)
        {return route.back();}
      } catch (const SurfaceSearchTimeout &) {}
    }
  }
  return std::nullopt;
}
}  // namespace frontier_exploration_ros2
