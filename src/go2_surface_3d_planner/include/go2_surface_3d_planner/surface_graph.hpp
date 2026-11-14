#ifndef GO2_SURFACE_3D_PLANNER__SURFACE_GRAPH_HPP_
#define GO2_SURFACE_3D_PLANNER__SURFACE_GRAPH_HPP_

#include <octomap/OcTree.h>
#include "go2_surface_3d_planner/surface_regions.hpp"

#include <cstddef>
#include <chrono>
#include <stdexcept>
#include <optional>
#include <limits>
#include <unordered_map>
#include <vector>

namespace go2_surface_3d_planner
{

struct SurfacePoint
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct SupportAvoidanceRegion {double x,y,z,radius;};

struct TurnExclusionRegion {double x,y,z,yaw,radius;};

// Temporary execution evidence, not occupied space. Translation and turns
// toward the retained heading remain possible; large turns need room elsewhere.
inline bool violatesTurnExclusion(
  const SurfacePoint & a,const SurfacePoint & b,const TurnExclusionRegion & region)
{
  const double turn=std::remainder(b.yaw-a.yaw,2*M_PI);
  if(std::abs(turn)<1e-8)return false;
  const double dx=b.x-a.x,dy=b.y-a.y,length2=dx*dx+dy*dy;
  const double t=length2>1e-12?std::clamp(
    ((region.x-a.x)*dx+(region.y-a.y)*dy)/length2,0.,1.):0.;
  if(std::hypot(a.x+t*dx-region.x,a.y+t*dy-region.y)>region.radius)return false;
  const double allowed=std::max(.12,std::abs(std::remainder(a.yaw-region.yaw,2*M_PI)));
  const int steps=std::max({1,static_cast<int>(std::ceil(std::sqrt(length2)/.02)),
    static_cast<int>(std::ceil(std::abs(turn)/.03))});
  for(int i=0;i<=steps;++i) {
    const double u=double(i)/steps;
    if(std::hypot(a.x+u*dx-region.x,a.y+u*dy-region.y)<=region.radius &&
       std::abs(a.z+u*(b.z-a.z)-region.z)<=.15 &&
       std::abs(std::remainder(a.yaw+u*turn-region.yaw,2*M_PI))>allowed+1e-8)return true;
  }
  return false;
}

struct SurfaceSearchConfig
{
  double xy_resolution{0.10};
  double z_resolution{0.05};
  double collision_radius{0.21};
  double cylinder_offset{0.10};
  int yaw_bins{32};
  double body_bottom{0.15};
  double body_height{0.48};
  double clearance_margin{0.01};
  // Zero disables fallback. It never changes the collision model or support rules.
  double narrow_clearance_margin{0.0};
  double preferred_clearance{0.20};
  double clearance_weight{2.0};
  double turn_cost_weight{0.10};
  double sweep_resolution{0.02};
  double sweep_angle{0.05235987755982989};
  double support_probe_depth{0.16};
  double max_step_height{0.12};
  double snap_radius{0.60};
  double snap_vertical_range{0.55};
  double search_padding_xy{1.50};
  double search_padding_z{0.35};
  double vertical_cost_weight{2.0};
  double goal_tolerance{0.16};
  int max_iterations{250000};
  bool require_known_clearance{false};
  bool infer_flat_support{false};
  bool flat_support_requires_prior{false};
  std::vector<SurfaceExclusionRegion> excluded_regions{};
  std::vector<TurnExclusionRegion> turn_exclusions{};
  std::vector<SupportAvoidanceRegion> support_avoidance{};
  // Explicit continuous slab prior: map pose xyz+xyzw, half extents xy, height tolerance.
  // Empty by default. This is scene knowledge, not an inferred absence of holes.
  std::vector<double> continuous_floor_region{};
  double flat_support_height{std::numeric_limits<double>::quiet_NaN()};
  double support_reference_height{0.21};
  double flat_support_origin_x{0.0};
  double flat_support_origin_y{0.0};
  double flat_support_slope_x{0.0};
  double flat_support_slope_y{0.0};
  double flat_support_radius{4.0};
  std::chrono::steady_clock::time_point search_deadline{
    std::chrono::steady_clock::time_point::max()};
};

class SurfaceSearchTimeout : public std::runtime_error
{
public:
  SurfaceSearchTimeout() : std::runtime_error("3D search time budget exceeded") {}
};
struct SupportPlane
{
  double height, origin_x, origin_y, slope_x, slope_y;
};

std::optional<SupportPlane> estimateSupportPlane(
  const octomap::OcTree & tree, const SurfacePoint & robot, double reference_height = 0.21);
std::optional<double> inferSupportPlane(
  const octomap::OcTree & tree, const SurfacePoint & robot, double reference_height = 0.21);
std::optional<SupportPlane> continuousFloorPlane(
  const SurfaceSearchConfig & config, const SurfacePoint & point);
bool configureSupportPlane(
  SurfaceSearchConfig & config, const octomap::OcTree & tree, const SurfacePoint & robot);
bool restoreCompatibleSupportPlane(
  SurfaceSearchConfig & config, const SurfaceSearchConfig & prior, const SurfacePoint & robot);

// Autonomous boundary targets may need an observation point inside mapped space.
// Every returned endpoint has a verified full-body support-connected path.
std::optional<SurfacePoint> selectExplorationGoal(
  const octomap::OcTree & tree, SurfaceSearchConfig config,
  const SurfacePoint & start, const SurfacePoint & requested, double max_retreat = 1.5);

class SurfaceGraph
{
public:
  explicit SurfaceGraph(const octomap::OcTree & tree, SurfaceSearchConfig config);

  std::optional<SurfacePoint> snapToSurface(const SurfacePoint & requested) const;
  std::optional<SurfacePoint> snapToSupportedSurface(const SurfacePoint & requested) const;
  bool isClearanceRecoveryMotionValid(const SurfacePoint & start,const SurfacePoint & end) const;
  bool isWalkableSurface(const SurfacePoint & point) const;
  struct PoseDiagnostics {bool surface, support, clearance; double gap;};
  PoseDiagnostics diagnosePose(const SurfacePoint & point) const
  {return {hasSurface(point),hasSupport(point),hasClearance(point),bodyClearance(point)};}

  enum class CloudSupport {confirmed, unknown, drop};
  CloudSupport inspectCloudSupport(const SurfacePoint & point) const;
  double bodyClearanceAt(const SurfacePoint & point) const;
  // Recovery-only zero-margin graph. Keep support, reduce each existing overlap,
  // introduce no new contact, and retain short improving prefixes.
  std::optional<SurfacePoint> findClearanceRecovery(
    const SurfacePoint & start, double required_margin, double max_distance,
    bool allow_partial = false) const;
  bool isMotionValid(const SurfacePoint & start, const SurfacePoint & end) const;
  std::vector<SurfacePoint> plan(
    const SurfacePoint & requested_start,
    const SurfacePoint & requested_goal) const;

private:
  void checkSearchBudget() const;
  struct Key
  {
    int x{0};
    int y{0};
    int z{0};
    int yaw{0};
    bool operator==(const Key & other) const noexcept;
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const noexcept;
  };

  Key pointToKey(const SurfacePoint & point) const;
  SurfacePoint keyToPoint(const Key & key) const;
  bool occupied(double x, double y, double z) const;
  bool known(double x, double y, double z) const;
  bool hasSupport(const SurfacePoint & point) const;
  bool hasSurface(const SurfacePoint & point) const;
  bool hasPrimarySurface(const SurfacePoint & point) const;
  double supportHeightAt(const SurfacePoint & point) const;
  bool hasClearance(const SurfacePoint & point) const;
  double bodyClearance(const SurfacePoint & point) const;
  bool walkable(const Key & key) const;
  std::vector<SurfacePoint> planAtMargin(const SurfacePoint & start, const SurfacePoint & goal) const;
  std::vector<Key> neighbors(const Key & key, const Key & min_key, const Key & max_key) const;

  const octomap::OcTree & tree_;
  SurfaceSearchConfig config_;
  mutable std::unordered_map<Key, bool, KeyHash> walkability_cache_;
  mutable std::unordered_map<Key, double, KeyHash> clearance_cache_;
};

}  // namespace go2_surface_3d_planner

#endif  // GO2_SURFACE_3D_PLANNER__SURFACE_GRAPH_HPP_
