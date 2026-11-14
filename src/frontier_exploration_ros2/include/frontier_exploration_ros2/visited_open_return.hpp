#pragma once
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include "go2_surface_3d_planner/surface_graph.hpp"
#include <vector>
#include <optional>
#include <cmath>
namespace frontier_exploration_ros2 {
// Only measured, supported, collision-free poses enter this bounded history.
// Recency defines "last open area"; every candidate is revalidated on today's map.
class VisitedOpenHistory {
public:
 using Pose=go2_surface_3d_planner::SurfacePoint;
 void sync(double now,double floor) {
  if(now<clock_ || (floor_ && std::abs(floor-*floor_)>.5))points_.clear();
  clock_=now;floor_=floor;
 }
 void observe(double now,double floor,Pose p,double gap) {
  sync(now,floor);
  if(!std::isfinite(p.x+p.y+p.z+p.yaw+gap) || gap<.10)return;
  if(!points_.empty() && std::hypot(points_.back().x-p.x,points_.back().y-p.y)<.75)return;
  points_.push_back(p);
  if(points_.size()>120)points_.erase(points_.begin());
 }
 std::vector<Pose> candidates(Pose current) const {
  std::vector<Pose> result;
  for(auto it=points_.rbegin();it!=points_.rend();++it) {
   double d=std::hypot(it->x-current.x,it->y-current.y);
   if(d>=.5 && d<=8.)result.push_back(*it);
   if(result.size()==8)break;
  }
  return result;
 }
 size_t size() const {return points_.size();}
private:
 std::vector<Pose> points_;std::optional<double> floor_;double clock_{-1};
};
inline std::optional<go2_surface_3d_planner::SurfacePoint> selectVisitedOpenReturn(
 const octomap::OcTree & tree,go2_surface_3d_planner::SurfaceSearchConfig config,
 go2_surface_3d_planner::SurfacePoint body,
 const std::vector<go2_surface_3d_planner::SurfacePoint> & candidates,
 const ExplorationBoundaryPolygon & boundary={},double margin=0.,bool reentry=false) {
 using namespace go2_surface_3d_planner;
 if(candidates.empty())return std::nullopt;
 if(config.infer_flat_support && !configureSupportPlane(config,tree,body))return std::nullopt;
 auto stance=body;if(config.infer_flat_support)stance.z-=config.support_reference_height;
 // Use the existing hard passage margin, not zero clearance. Recovery must
 // be allowed to traverse a supported narrow passage before reaching open space.
 config.clearance_margin=config.narrow_clearance_margin>0?
   std::max(.01,config.narrow_clearance_margin):std::max(.01,config.clearance_margin);
 config.narrow_clearance_margin=0;
 config.xy_resolution=std::min(config.xy_resolution,tree.getResolution()*.5);
 for(auto candidate:candidates) {
  const double distance=std::hypot(candidate.x-body.x,candidate.y-body.y);
  if(distance<(reentry?.15:.5) || distance>8. ||
     !is_inside_exploration_boundary({candidate.x,candidate.y},boundary,margin))continue;
  auto attempt=config;
  attempt.search_deadline=std::min(config.search_deadline,
    std::chrono::steady_clock::now()+std::chrono::milliseconds(350));
  try {
   auto endpoint_config=attempt;endpoint_config.snap_radius=0;endpoint_config.preferred_clearance=.15;
   SurfaceGraph endpoints(tree,endpoint_config);
   auto end=endpoints.snapToSurface(candidate);
   if(!end || endpoints.bodyClearanceAt(*end)<.10)continue;
   const auto path=SurfaceGraph(tree,attempt).plan(stance,*end);

   if(reentry?boundary_reentry_path(stance,path,boundary,margin):
       boundary_return_path(stance,path,boundary,margin))return path.back();
   if(reentry) {
    // A nearest grid seed can be a few millimetres farther outside. Try
    // inward grid-aligned seeds, validating the measured connector explicitly.
    const double resolution=config.xy_resolution;
    const double cx=std::round(stance.x/resolution)*resolution;
    const double cy=std::round(stance.y/resolution)*resolution;
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy) {
      auto bridge=endpoints.snapToSurface({cx+dx*resolution,cy+dy*resolution,stance.z,stance.yaw});
      if(!bridge || !boundary_reentry_step({stance.x,stance.y},{bridge->x,bridge->y},boundary) ||
         !endpoints.isMotionValid(stance,*bridge))continue;
      auto via=SurfaceGraph(tree,attempt).plan(*bridge,*end);
      if(via.size()>1 && boundary_reentry_path(*bridge,via,boundary,margin))return via.back();
    }
   }
  } catch(const SurfaceSearchTimeout &) {}
 }
 return std::nullopt;
}
}
