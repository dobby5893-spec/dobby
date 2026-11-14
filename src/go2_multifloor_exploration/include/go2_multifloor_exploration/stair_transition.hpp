#pragma once
#include "go2_surface_3d_planner/surface_graph.hpp"
#include <chrono>
#include <vector>
namespace go2_multifloor_exploration {
using go2_surface_3d_planner::SurfacePoint;
using go2_surface_3d_planner::SurfaceSearchConfig;
using go2_surface_3d_planner::SurfaceExclusionRegion;
struct StairOptions {
 double range{8.0},minimum_rise{.03},maximum_rise{.12},maximum_run{.40};
 double minimum_width{.50},minimum_chain_rise{.14},maximum_segment_length{1.2};
 double approach_distance{.85},landing_radius{.70};
 int minimum_steps{3},maximum_surfaces{16000},maximum_candidates{12};
};
struct StairCandidate {
 SurfacePoint first_tread,approach,target;
 SurfaceExclusionRegion region;
 std::vector<SurfacePoint> chain;
};
struct StairScan {std::vector<StairCandidate> candidates;bool timed_out{false};};
StairScan detectStairs(const octomap::OcTree &,const SurfaceSearchConfig &,
 const SurfacePoint & stance,const StairOptions &);
bool hasLandingPatch(const octomap::OcTree &,const SurfaceSearchConfig &,
 const SurfacePoint &,double radius);
bool reachedStairTarget(const SurfacePoint & actual,const SurfacePoint & target,
 double xy_tolerance=.25,double z_tolerance=.08);
std::vector<SurfacePoint> verifyStairApproach(const octomap::OcTree &,
 SurfaceSearchConfig,const SurfacePoint &,const StairCandidate &);
}  // namespace go2_multifloor_exploration
