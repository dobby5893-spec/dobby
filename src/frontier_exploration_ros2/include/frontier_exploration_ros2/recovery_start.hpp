#pragma once
#include "go2_surface_3d_planner/surface_graph.hpp"

namespace frontier_exploration_ros2 {
enum class RecoveryStartState {ready,needs_clearance,unsupported,contact};
struct RecoveryStartAssessment {
  RecoveryStartState state{RecoveryStartState::unsupported};
  double gap{-1};
};
inline RecoveryStartAssessment assessRecoveryStart(
  const octomap::OcTree & tree,go2_surface_3d_planner::SurfaceSearchConfig config,
  const go2_surface_3d_planner::SurfacePoint & start)
{
  using namespace go2_surface_3d_planner;
  const double required=std::max(.01,config.clearance_margin);
  if(config.infer_flat_support && !configureSupportPlane(config,tree,start))return {};
  auto stance=start;
  if(config.infer_flat_support)stance.z-=config.support_reference_height;
  // Inspect the measured XY, without snapping sideways to a different pose.
  config.clearance_margin=0;config.narrow_clearance_margin=0;config.snap_radius=0;
  SurfaceGraph graph(tree,config);
  const auto surface=graph.snapToSupportedSurface(stance);
  if(!surface)return {};
  const auto diagnostic=graph.diagnosePose(*surface);
  if(!diagnostic.surface || !diagnostic.support)return {};
  if(diagnostic.gap<=1e-9)
    return {RecoveryStartState::contact,diagnostic.gap};
  return {diagnostic.gap<required?RecoveryStartState::needs_clearance:
    RecoveryStartState::ready,diagnostic.gap};
}
}
