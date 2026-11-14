#include "go2_surface_3d_planner/surface_graph.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace go2_surface_3d_planner
{

namespace
{

double distance3d(const SurfacePoint & a, const SurfacePoint & b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

}  // namespace

std::optional<SupportPlane> estimateSupportPlane(
  const octomap::OcTree & tree, const SurfacePoint & robot, double reference_height)
{
  // Sensor mounting and current stance constrain elevation; world z=0 is valid.
  // Fit occupied voxel centres, not their tops (which bias height by half a voxel).
  const double r = tree.getResolution();
  // The actual ground can lie anywhere inside its occupied voxel.
  // Its centre is within half a cell of the foot surface, not necessarily
  // exactly half a cell below it (especially across registration updates).
  const double expected = robot.z - reference_height;
  // Half-cell quantization plus 1 cm of TF/scan registration uncertainty.
  // This only gates fitting of observed ground; body/drop tests remain separate.
  const double tolerance = std::max(0.04, r * 0.5 + 0.01);
  const double residual_limit = std::max(0.035, r * 0.6);
  std::vector<SurfacePoint> points;
  for (auto it = tree.begin_leafs_bbx(
      octomap::point3d(robot.x-4,robot.y-4,expected-0.75),
      octomap::point3d(robot.x+4,robot.y+4,expected+0.75));
    it != tree.end_leafs_bbx(); ++it)
  {
    if (!tree.isNodeOccupied(*it) || it.getSize() > 2*r) {continue;}
    const double distance = std::hypot(it.getX()-robot.x,it.getY()-robot.y);
    if (distance > 4.0 || std::abs(it.getZ()-expected) > tolerance+0.15*distance) {continue;}
    bool wall=false;
    for (double h : {0.15,0.30}) {
      const auto * above=tree.search(it.getX(),it.getY(),it.getZ()+h);
      wall=wall || (above && tree.isNodeOccupied(above));
    }
    if (!wall) {points.push_back({it.getX()-robot.x,it.getY()-robot.y,it.getZ(),0});}
  }
  if (points.size() < 12) {return std::nullopt;}
  std::mt19937 random(17);
  std::uniform_int_distribution<std::size_t> choose(0,points.size()-1);
  SupportPlane best{expected,robot.x,robot.y,0,0};
  std::size_t best_count=0;
  const auto acceptable = [&](const SupportPlane & p) {
      return std::hypot(p.slope_x,p.slope_y) <= 0.10 &&
             std::abs(p.height-expected) <= tolerance;
    };
  const auto residual = [](const SurfacePoint & point, const SupportPlane & p) {
      return std::abs(point.z-p.height-p.slope_x*point.x-p.slope_y*point.y);
    };
  for (int trial=0;trial<192;++trial) {
    const auto & a=points[choose(random)];
    const auto & b=points[choose(random)];
    const auto & c=points[choose(random)];
    const double det=(b.x-a.x)*(c.y-a.y)-(c.x-a.x)*(b.y-a.y);
    if (std::abs(det) < 0.05) {continue;}
    const double sx=((b.z-a.z)*(c.y-a.y)-(c.z-a.z)*(b.y-a.y))/det;
    const double sy=((b.x-a.x)*(c.z-a.z)-(c.x-a.x)*(b.z-a.z))/det;
    const SupportPlane candidate{a.z-sx*a.x-sy*a.y,robot.x,robot.y,sx,sy};
    if (!acceptable(candidate)) {continue;}
    std::size_t count=0;
    for (const auto & p:points) {if (residual(p,candidate)<=residual_limit) {++count;}}
    if (count > best_count ||
      (count == best_count && std::abs(candidate.height-expected)<std::abs(best.height-expected)))
    {best_count=count; best=candidate;}
  }
  if (best_count < 12) {return std::nullopt;}
  // Least squares on the consensus only. Reject a line or a tiny patch.
  for (int iteration=0;iteration<2;++iteration) {
    double matrix[3][4]{};
    double minx=1e9,maxx=-1e9,miny=1e9,maxy=-1e9;
    std::size_t count=0;
    for (const auto & p:points) {
      if (residual(p,best)>residual_limit) {continue;}
      const double row[3]{1,p.x,p.y};
      for (int i=0;i<3;++i) {
        for (int j=0;j<3;++j) {matrix[i][j]+=row[i]*row[j];}
        matrix[i][3]+=row[i]*p.z;
      }
      minx=std::min(minx,p.x); maxx=std::max(maxx,p.x);
      miny=std::min(miny,p.y); maxy=std::max(maxy,p.y); ++count;
    }
    if (count<12 || maxx-minx<0.5 || maxy-miny<0.5) {return std::nullopt;}
    for (int i=0;i<3;++i) {
      int pivot=i;
      for (int j=i+1;j<3;++j) {if (std::abs(matrix[j][i])>std::abs(matrix[pivot][i])) {pivot=j;}}
      if (std::abs(matrix[pivot][i])<1e-8) {return std::nullopt;}
      for (int j=i;j<4;++j) {std::swap(matrix[i][j],matrix[pivot][j]);}
      const double value=matrix[i][i];
      for (int j=i;j<4;++j) {matrix[i][j]/=value;}
      for (int k=0;k<3;++k) {
        if(k==i) {continue;}
        const double factor=matrix[k][i];
        for (int j=i;j<4;++j) {matrix[k][j]-=factor*matrix[i][j];}
      }
    }
    const SupportPlane refined{matrix[0][3],robot.x,robot.y,matrix[1][3],matrix[2][3]};
    // A quantized consensus can pull least-squares just beyond the stance
    // constraint. Keep the already verified RANSAC candidate in that case;
    // discarding it makes fitting fail despite observed compatible ground.
    if (!acceptable(refined)) {break;}
    best=refined;
  }
  return best;
}

std::optional<double> inferSupportPlane(
  const octomap::OcTree & tree, const SurfacePoint & robot, double reference_height)
{
  const auto plane=estimateSupportPlane(tree,robot,reference_height);
  if (!plane) {return std::nullopt;}
  return plane->height;
}

std::optional<SupportPlane> continuousFloorPlane(
  const SurfaceSearchConfig & config, const SurfacePoint & point)
{
  const auto & r=config.continuous_floor_region;
  if(!config.infer_flat_support || r.size()!=10 ||
    !std::all_of(r.begin(),r.end(),[](double v){return std::isfinite(v);}) ||
    r[7]<=0 || r[8]<=0 || r[9]<=0) {return std::nullopt;}
  const double x=r[3],y=r[4],z=r[5],w=r[6];
  if(std::abs(x*x+y*y+z*z+w*w-1.0)>1e-5) {return std::nullopt;}
  const double ux=1-2*(y*y+z*z),uy=2*(x*y+w*z),uz=2*(x*z-w*y);
  const double vx=2*(x*y-w*z),vy=1-2*(x*x+z*z),vz=2*(y*z+w*x);
  const double nx=2*(x*z+w*y),ny=2*(y*z-w*x),nz=1-2*(x*x+y*y);
  if(nz<0.98) {return std::nullopt;}
  const double dx=point.x-r[0],dy=point.y-r[1],dz=point.z-r[2];
  if(std::abs(ux*dx+uy*dy+uz*dz)>r[7] ||
    std::abs(vx*dx+vy*dy+vz*dz)>r[8] ||
    std::abs(nx*dx+ny*dy+nz*dz)>r[9]) {return std::nullopt;}
  return SupportPlane{r[2],r[0],r[1],-nx/nz,-ny/nz};
}

bool configureSupportPlane(
  SurfaceSearchConfig & config, const octomap::OcTree & tree, const SurfacePoint & robot)
{
  auto plane=estimateSupportPlane(tree,robot,config.support_reference_height);
  // A configured solid slab also supplies a bounded fallback in the LiDAR blind zone.
  if(!plane) {
    auto feet=robot;feet.z-=config.support_reference_height;
    plane=continuousFloorPlane(config,feet);
    if(plane) {plane->height+=plane->slope_x*(feet.x-plane->origin_x)+
      plane->slope_y*(feet.y-plane->origin_y);plane->origin_x=feet.x;plane->origin_y=feet.y;}
  }
  // Never silently reuse an incompatible plane when the new observation fails.
  config.flat_support_height=std::numeric_limits<double>::quiet_NaN();
  if (!plane) {return false;}
  // The physical floor can lie anywhere inside an occupied ground voxel.
  // Register the fitted centre band to the measured stance instead of always
  // lifting it to the voxel top, which can place the floor above the feet.
  const double expected=robot.z-config.support_reference_height;
  const double half=tree.getResolution()*0.5;
  config.flat_support_height=std::clamp(expected,plane->height-half,plane->height+half);
  config.flat_support_origin_x=plane->origin_x; config.flat_support_origin_y=plane->origin_y;
  config.flat_support_slope_x=plane->slope_x; config.flat_support_slope_y=plane->slope_y;
  return true;
}


bool restoreCompatibleSupportPlane(
  SurfaceSearchConfig & config, const SurfaceSearchConfig & prior, const SurfacePoint & robot)
{
  const double floor=prior.flat_support_height+
    prior.flat_support_slope_x*(robot.x-prior.flat_support_origin_x)+
    prior.flat_support_slope_y*(robot.y-prior.flat_support_origin_y);
  if(!config.infer_flat_support || !prior.infer_flat_support || !std::isfinite(floor) ||
    std::abs(floor-(robot.z-config.support_reference_height))>.04 ||
    std::hypot(robot.x-prior.flat_support_origin_x,robot.y-prior.flat_support_origin_y)>
    prior.flat_support_radius) {return false;}
  config.flat_support_height=prior.flat_support_height;
  config.flat_support_origin_x=prior.flat_support_origin_x;
  config.flat_support_origin_y=prior.flat_support_origin_y;
  config.flat_support_slope_x=prior.flat_support_slope_x;
  config.flat_support_slope_y=prior.flat_support_slope_y;
  return true;
}

SurfaceGraph::SurfaceGraph(const octomap::OcTree & tree, SurfaceSearchConfig config)
: tree_(tree), config_(std::move(config))
{
  config_.xy_resolution = std::max(config_.xy_resolution, tree_.getResolution()*0.5);
  config_.z_resolution = std::max(config_.z_resolution, tree_.getResolution() * 0.5);
  config_.collision_radius = std::max(0.0, config_.collision_radius);
  config_.cylinder_offset = std::max(0.0, config_.cylinder_offset);
  config_.yaw_bins = std::max(8, config_.yaw_bins);
  config_.body_bottom = std::max(0.0, config_.body_bottom);
  config_.preferred_clearance = std::max(0.01, config_.preferred_clearance);
  config_.clearance_weight = std::max(0.0, config_.clearance_weight);
  config_.turn_cost_weight = std::max(0.001, config_.turn_cost_weight);
  config_.sweep_resolution = std::clamp(config_.sweep_resolution, 0.005, 0.02);
  config_.sweep_angle = std::clamp(config_.sweep_angle, 0.005, M_PI / 60.0);
  config_.body_height = std::max(tree_.getResolution(), config_.body_height);
  config_.clearance_margin = std::max(0.0, config_.clearance_margin);
  config_.support_probe_depth = std::max(tree_.getResolution(), config_.support_probe_depth);
  config_.max_step_height = std::max(config_.z_resolution, config_.max_step_height);
  config_.snap_radius = std::max(0.0, config_.snap_radius);
  config_.snap_vertical_range = std::max(config_.z_resolution, config_.snap_vertical_range);
  config_.max_iterations = std::max(1, config_.max_iterations);
}

std::optional<SurfacePoint> selectExplorationGoal(
  const octomap::OcTree & tree, SurfaceSearchConfig config,
  const SurfacePoint & start, const SurfacePoint & requested, double max_retreat)
{
  if (config.infer_flat_support) {
    configureSupportPlane(config, tree, start);
  }
  SurfacePoint stance_start=start;
  if(config.infer_flat_support) {stance_start.z-=config.support_reference_height;}
  SurfaceGraph graph(tree, config);
  auto origin = graph.snapToSurface(stance_start);
  if(!origin && config.narrow_clearance_margin>0.0 && config.narrow_clearance_margin<config.clearance_margin) {
    auto fine=config;fine.xy_resolution=std::min(fine.xy_resolution,tree.getResolution()*0.5);
    origin=SurfaceGraph(tree,fine).snapToSurface(stance_start);
  }
  if(!origin && config.narrow_clearance_margin>0.0 && config.narrow_clearance_margin<config.clearance_margin) {
    auto narrow=config;narrow.clearance_margin=std::max(0.01,config.narrow_clearance_margin);
    narrow.xy_resolution=std::min(narrow.xy_resolution,tree.getResolution()*0.5);
    origin=SurfaceGraph(tree,narrow).snapToSurface(stance_start);
  }
  if (!origin) {return std::nullopt;}
  const double distance = std::hypot(requested.x-start.x, requested.y-start.y);
  if (distance < 0.50) {return std::nullopt;}
  const double minimum_distance=std::max(0.50,distance-std::max(0.0,max_retreat));
  std::vector<double> observations;
  const auto add=[&](double d) {
      d=std::clamp(d,minimum_distance,distance);
      if(std::none_of(observations.begin(),observations.end(),
        [&](double old){return std::abs(old-d)<1e-6;})) {observations.push_back(d);}
    };
  // Start with a useful nearby observation, then shorter safe approaches.
  for(double d:{1.5,1.0,0.75}) {add(d);}
  for(double d=2.0;d<distance;d+=0.5) {add(d);}
  add(distance);
  bool timed_out=false;
  // Follow a verified route to the boundary rather than placing every approach
  // on the straight line, which may point into a door frame or furniture.
  auto route_config=config;
  if(config.search_deadline!=std::chrono::steady_clock::time_point::max()) {
    const auto now=std::chrono::steady_clock::now();
    route_config.search_deadline=now+(config.search_deadline-now)*4/5;
  }
  // Frontier observations do not require one fixed terminal heading. A
  // doorway can require a different heading after entering the vestibule.
  std::vector<double> headings{requested.yaw,start.yaw};
  for(int i=0;i<config.yaw_bins;++i) {
    const double yaw=i*2*M_PI/config.yaw_bins;
    if(std::none_of(headings.begin(),headings.end(),[&](double old){return std::abs(std::remainder(old-yaw,2*M_PI))<1e-6;})) {headings.push_back(yaw);}
  }
  for(double yaw:headings) {
    auto endpoint=requested;endpoint.yaw=yaw;
    auto endpoint_config=route_config;
    if(endpoint_config.narrow_clearance_margin>0 && endpoint_config.narrow_clearance_margin<endpoint_config.clearance_margin) {
      endpoint_config.clearance_margin=std::max(.01,endpoint_config.narrow_clearance_margin);
      endpoint_config.xy_resolution=std::min(endpoint_config.xy_resolution,tree.getResolution()*0.5);
    }
    if(!SurfaceGraph(tree,endpoint_config).snapToSurface(endpoint)) {continue;}
    auto attempt_config=route_config;
    if(attempt_config.search_deadline!=std::chrono::steady_clock::time_point::max()) {
      attempt_config.search_deadline=std::min(attempt_config.search_deadline,
        std::chrono::steady_clock::now()+std::chrono::milliseconds(500));
    }
    try {
      const auto route=SurfaceGraph(tree,attempt_config).plan(stance_start,endpoint);
      double arc=0.0;
      for(std::size_t i=1;i<route.size();++i) {
        arc+=std::hypot(route[i].x-route[i-1].x,route[i].y-route[i-1].y);
        if(arc>=1.5-1e-9 &&
          std::hypot(route[i].x-start.x,route[i].y-start.y)>=0.5 &&
          std::hypot(requested.x-route[i].x,requested.y-route[i].y)<=
            std::max(0.0,max_retreat)+1e-6) {return route[i];}
      }
      if(!route.empty() && distance<=1.5+1e-9) {return route.back();}
    } catch(const SurfaceSearchTimeout &) {timed_out=true;}
  }
  for(double d:observations) {
    SurfacePoint candidate=requested;
    candidate.x=start.x+d*(requested.x-start.x)/distance;
    candidate.y=start.y+d*(requested.y-start.y)/distance;
    if(std::abs(candidate.z-origin->z)<=config.max_step_height) {candidate.z=origin->z;}
    for(double yaw:headings) {
      candidate.yaw=yaw;
      auto attempt_config=config;
      // A difficult first candidate cannot consume the whole batch budget.
      if(config.search_deadline!=std::chrono::steady_clock::time_point::max()) {
        if(std::chrono::steady_clock::now()>=config.search_deadline) {throw SurfaceSearchTimeout();}
        attempt_config.search_deadline=std::min(config.search_deadline,
          std::chrono::steady_clock::now()+std::chrono::milliseconds(150));
      }
      try {
        auto endpoint_config=attempt_config;
        if(endpoint_config.narrow_clearance_margin>0 && endpoint_config.narrow_clearance_margin<endpoint_config.clearance_margin) {
          endpoint_config.clearance_margin=std::max(.01,endpoint_config.narrow_clearance_margin);
          endpoint_config.xy_resolution=std::min(endpoint_config.xy_resolution,tree.getResolution()*.5);
        }
        const auto snapped=SurfaceGraph(tree,endpoint_config).snapToSurface(candidate);
        if(!snapped || std::hypot(snapped->x-start.x,snapped->y-start.y)<.5 ||
          std::hypot(snapped->x-requested.x,snapped->y-requested.y)>max_retreat+1e-6) {continue;}
        const auto path=SurfaceGraph(tree,attempt_config).plan(stance_start,*snapped);
        if(!path.empty()) {return path.back();}
      } catch(const SurfaceSearchTimeout &) {timed_out=true;}
    }
  }
  if(timed_out) {throw SurfaceSearchTimeout();}
  return std::nullopt;
}

void SurfaceGraph::checkSearchBudget() const
{
  if(std::chrono::steady_clock::now()>=config_.search_deadline) {
    throw SurfaceSearchTimeout();
  }
}

bool SurfaceGraph::Key::operator==(const Key & other) const noexcept
{
  return x == other.x && y == other.y && z == other.z && yaw == other.yaw;
}

std::size_t SurfaceGraph::KeyHash::operator()(const Key & key) const noexcept
{
  std::size_t seed = std::hash<int>{}(key.x);
  seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int>{}(key.yaw) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

SurfaceGraph::Key SurfaceGraph::pointToKey(const SurfacePoint & point) const
{
  const int bin = static_cast<int>(std::llround(
    std::remainder(point.yaw, 2.0 * M_PI) * config_.yaw_bins / (2.0 * M_PI)));
  return {
    static_cast<int>(std::llround(point.x / config_.xy_resolution)),
    static_cast<int>(std::llround(point.y / config_.xy_resolution)),
    static_cast<int>(std::llround(point.z / config_.z_resolution)),
    (bin % config_.yaw_bins + config_.yaw_bins) % config_.yaw_bins};
}

double SurfaceGraph::supportHeightAt(const SurfacePoint & point) const
{
  if(!config_.infer_flat_support)return std::numeric_limits<double>::quiet_NaN();
  if(std::isfinite(config_.flat_support_height) &&
     std::hypot(point.x-config_.flat_support_origin_x,point.y-config_.flat_support_origin_y)<=
       config_.flat_support_radius) {
    return config_.flat_support_height+
      config_.flat_support_slope_x*(point.x-config_.flat_support_origin_x)+
      config_.flat_support_slope_y*(point.y-config_.flat_support_origin_y);
  }
  // A measured local fit is range-limited. A registered continuous slab is
  // separate scene evidence and remains usable inside its explicit XYZ bounds.
  const auto prior=continuousFloorPlane(config_,point);
  if(prior)return prior->height+prior->slope_x*(point.x-prior->origin_x)+
    prior->slope_y*(point.y-prior->origin_y);
  return std::numeric_limits<double>::quiet_NaN();
}

SurfacePoint SurfaceGraph::keyToPoint(const Key & key) const
{
  SurfacePoint point{
    key.x * config_.xy_resolution,
    key.y * config_.xy_resolution,
    key.z * config_.z_resolution,
    key.yaw * 2.0 * M_PI / config_.yaw_bins};
  const double height = supportHeightAt(point);
  const double epsilon=tree_.getResolution()*0.01;
  // Keep observed voxel surfaces. Only the plane's nearest height layer uses
  // its continuous elevation; z bins identify states, not physical ground.
  if (config_.infer_flat_support && std::isfinite(height) &&
    std::abs(point.z-height)<=config_.z_resolution*0.5+1e-9 &&
    !(occupied(point.x,point.y,point.z-epsilon) &&
      !occupied(point.x,point.y,point.z+epsilon))) {point.z=height;}
  return point;
}

bool SurfaceGraph::occupied(double x, double y, double z) const
{
  const auto * node = tree_.search(x, y, z);
  return node != nullptr && tree_.isNodeOccupied(node);
}

bool SurfaceGraph::known(double x, double y, double z) const
{
  return tree_.search(x, y, z) != nullptr;
}

bool SurfaceGraph::hasSupport(const SurfacePoint & point) const
{
  checkSearchBudget();
  const double map_resolution = tree_.getResolution();
  const std::array<std::pair<double, double>, 5> probes{{
    {0.0, 0.0},
    {config_.collision_radius * 0.55, 0.0},
    {-config_.collision_radius * 0.55, 0.0},
    {0.0, config_.collision_radius * 0.55},
    {0.0, -config_.collision_radius * 0.55}}};

  // Center support is mandatory. The extra probes prevent paths along unsupported edges,
  // while allowing a narrow stair tread to support the robot.
  bool center_supported = false;
  int supported_probes = 0;
  for (std::size_t i = 0; i < probes.size(); ++i) {
    bool supported = false;
    for (double depth = 0.5 * map_resolution;
      depth <= config_.support_probe_depth + 0.5 * map_resolution;
      depth += map_resolution)
    {
      if (occupied(point.x + probes[i].first, point.y + probes[i].second, point.z - depth)) {
        supported = true;
        break;
      }
    }
    const double plane_here=supportHeightAt(point);
    double probe_z=point.z;
    if(config_.infer_flat_support && std::isfinite(plane_here) &&
      std::abs(point.z-plane_here)<=config_.z_resolution*0.5+1e-9) {
      probe_z+=config_.flat_support_slope_x*probes[i].first +
        config_.flat_support_slope_y*probes[i].second;
    }
    if (!supported && hasSurface({point.x + probes[i].first,
      point.y + probes[i].second,probe_z,point.yaw}))
    {supported=true;}
    if (i == 0) {
      center_supported = supported;
    }
    if (supported) {
      ++supported_probes;
    }
  }
  return center_supported && supported_probes >= 3;
}


// This is an independent local 3D check, not another planar extrapolation.
// Only observations close to this floor count; a lower storey cannot support it.
SurfaceGraph::CloudSupport SurfaceGraph::inspectCloudSupport(const SurfacePoint & p) const
{
  checkSearchBudget();
  const double r=tree_.getResolution(), half=r*.5;
  const double depth=std::max(config_.max_step_height+2*r, .20);
  bool void_column=true;
  for(double z=p.z+half;z>=p.z-depth-1e-9;z-=half) {
    const auto * n=tree_.search(p.x,p.y,z);
    if(!n || tree_.isNodeOccupied(n)){void_column=false;break;}
  }
  if(void_column)return CloudSupport::drop;
  // Do not use neighbouring floor points to erase raised terrain at the centre.
  if(occupied(p.x,p.y,p.z+half) && occupied(p.x,p.y,p.z+r+.01))
    return CloudSupport::unknown;
  bool body_observed=false;
  for(double h=.15;h<=.45+1e-9;h+=half) {
    const auto * n=tree_.search(p.x,p.y,p.z+h);
    if(n && !tree_.isNodeOccupied(n)){body_observed=true;break;}
  }
  if(!body_observed)return CloudSupport::unknown;
  // A small occupied patch must span both XY axes. Unknown is not free space.
  unsigned supported=0;bool negative_x=false,positive_x=false,negative_y=false,positive_y=false;
  const double band=std::min(config_.max_step_height, r+.01);
  for(int ix=-1;ix<=1;++ix)for(int iy=-1;iy<=1;++iy) {
    bool ground=false;
    const double x=p.x+ix*r,y=p.y+iy*r;
    for(double z=p.z-band;z<=p.z+half+1e-9;z+=half) {
      const auto * n=tree_.search(x,y,z);
      if(n && tree_.isNodeOccupied(n)) {
        const auto * above=tree_.search(x,y,z+r);
        if(!above || !tree_.isNodeOccupied(above)){ground=true;break;}
      }
    }
    if(ground) {
      ++supported;negative_x|=ix<0;positive_x|=ix>0;
      negative_y|=iy<0;positive_y|=iy>0;
    }
  }
  return supported>=6 && negative_x && positive_x && negative_y && positive_y?
    CloudSupport::confirmed:CloudSupport::unknown;
}

bool SurfaceGraph::hasSurface(const SurfacePoint & point) const
{
  if(hasPrimarySurface(point))return true;
  if(!config_.infer_flat_support || config_.flat_support_requires_prior ||
     !std::isfinite(config_.flat_support_height))return false;
  const double expected=config_.flat_support_height+
    config_.flat_support_slope_x*(point.x-config_.flat_support_origin_x)+
    config_.flat_support_slope_y*(point.y-config_.flat_support_origin_y);
  if(std::abs(point.z-expected)>config_.z_resolution*.5+1e-9 ||
     std::hypot(point.x-config_.flat_support_origin_x,point.y-config_.flat_support_origin_y)>
       config_.flat_support_radius)return false;
  return inspectCloudSupport(point)==CloudSupport::confirmed;
}

bool SurfaceGraph::hasPrimarySurface(const SurfacePoint & point) const
{
  const double half=tree_.getResolution()*0.5;
  // A surface is a voxel boundary, not the occupied voxel's centre. Sampling
  // half a voxel away incorrectly accepted ground centres as another floor.
  const double epsilon = tree_.getResolution() * 0.01;
  if (occupied(point.x,point.y,point.z-epsilon) &&
    !occupied(point.x,point.y,point.z+epsilon)) {return true;}
  const double expected = supportHeightAt(point);
  if (!config_.infer_flat_support || !std::isfinite(expected) ||
    std::abs(point.z-expected) > config_.z_resolution*0.5 + 1e-9) {return false;}
  // A registered foot plane may pass inside a ground voxel. Accept that
  // quantization band only if occupation does not extend beyond one cell
  // plus registration uncertainty; higher terrain still needs its observed top.
  if(occupied(point.x,point.y,point.z+half) &&
    occupied(point.x,point.y,expected+tree_.getResolution()+.01)) {return false;}
  // A voxel touching the fitted floor is not evidence of a drop: sensor noise
  // and voxel quantization can mark that boundary cell free. Reject observed
  // void below the floor's one-cell uncertainty band instead.
  const auto known_free_at = [&](double z) {
      const auto * node=tree_.search(point.x,point.y,z);
      return node && !tree_.isNodeOccupied(node);
    };
  // Free space below an unknown surface does not prove that surface absent:
  // a thin floor (or a missing ground return) can separate the two volumes.
  // A measured void must cross the whole fitted-floor uncertainty band and
  // continue down to the rejected depth, without an unknown/occupied break.
  const auto observed_void_to = [&](double depth) {
      if (!known_free_at(expected-depth)) {return false;}
      for (double z=expected+half; z>=expected-depth-1e-9; z-=half) {
        if (!known_free_at(z)) {return false;}
      }
      return true;
    };
  const bool continuous_floor=continuousFloorPlane(config_,point).has_value();
  if(config_.flat_support_requires_prior && !continuous_floor) {return false;}
  if (!continuous_floor && observed_void_to(3*half) &&
    (3*half >= config_.max_step_height || observed_void_to(5*half))) {return false;}
  for (double depth=config_.max_step_height+half;
    depth<=config_.max_step_height+5*half+1e-9; depth+=2*half) {
    if (!continuous_floor && observed_void_to(depth)) {return false;}
  }
  // Extrapolate only into observed free body space. Observed continuous voids
  // were rejected above; this remains limited to the fitted local plane.
  for (double height = 0.15; height <= 0.45+1e-9;
    height += tree_.getResolution()*0.5) {
    const auto * node=tree_.search(point.x,point.y,point.z+height);
    if (node && !tree_.isNodeOccupied(node)) {return true;}
  }
  return false;
}


double SurfaceGraph::bodyClearance(const SurfacePoint & point) const
{
  checkSearchBudget();
  // Raw occupied voxel volumes versus the full two-cylinder body. The ground
  // below body_bottom supports the feet and is not a body collision.
  const double r = config_.collision_radius;
  const double reach = r + config_.preferred_clearance + config_.clearance_margin;
  const double dx = config_.cylinder_offset * std::cos(point.yaw);
  const double dy = config_.cylinder_offset * std::sin(point.yaw);
  const double bottom = point.z + config_.body_bottom;
  const double top = point.z + config_.body_height + config_.clearance_margin;
  const octomap::point3d low(point.x - std::abs(dx) - reach,
    point.y - std::abs(dy) - reach, bottom);
  const octomap::point3d high(point.x + std::abs(dx) + reach,
    point.y + std::abs(dy) + reach, top);
  double clearance = config_.preferred_clearance + config_.clearance_margin;
  for(const auto & region:config_.excluded_regions) {
    if(point.z<region.min_z || point.z>region.max_z) continue;
    // A circumscribed disk conservatively reserves the complete body for flat navigation.
    clearance=std::min(clearance,distanceToRegion(point.x,point.y,region)-r-config_.cylinder_offset);
  }

  for (auto it = tree_.begin_leafs_bbx(low, high); it != tree_.end_leafs_bbx(); ++it) {
    if (!tree_.isNodeOccupied(*it)) {continue;}
    const double half = it.getSize() * 0.5;
    if (it.getZ() + half <= bottom + 1e-9 || it.getZ() - half >= top - 1e-9) {continue;}
    for (double sign : {-1.0, 1.0}) {
      const double cx = point.x + sign * dx, cy = point.y + sign * dy;
      const double ex = std::max(0.0, std::abs(cx - it.getX()) - half);
      const double ey = std::max(0.0, std::abs(cy - it.getY()) - half);
      clearance = std::min(clearance, std::hypot(ex, ey) - r);
    }

  }
  if (config_.require_known_clearance) {
    const double step = tree_.getResolution();
    for (double height = config_.body_bottom + 0.5 * step;
      height < config_.body_height; height += step)
    {
      for (double x = -r; x <= r; x += step) {
        for (double y = -r; y <= r; y += step) {
          if (std::hypot(x, y) > r) {continue;}
          for (double sign : {-1.0, 1.0}) {
            if (!known(point.x + sign * dx + x, point.y + sign * dy + y,
              point.z + height)) {return -1.0;}
          }
        }
      }
    }
  }
  return clearance;
}

bool SurfaceGraph::hasClearance(const SurfacePoint & point) const
{
  return bodyClearance(point) >= config_.clearance_margin - 1e-9;
}

bool SurfaceGraph::isMotionValid(const SurfacePoint & a, const SurfacePoint & b) const
{
  checkSearchBudget();
  for(const auto & region:config_.support_avoidance) {
    if(std::abs(a.z-region.z)>.15 || std::abs(b.z-region.z)>.15)continue;
    const double ax=a.x-region.x,ay=a.y-region.y,dx=b.x-a.x,dy=b.y-a.y;
    const double start=std::hypot(ax,ay),length2=dx*dx+dy*dy;
    // A robot already inside may leave monotonically; never block its exit.
    if(start<region.radius) {
      if(ax*dx+ay*dy<-1e-10 || std::hypot(b.x-region.x,b.y-region.y)+1e-9<start)return false;
    } else {
      const double t=length2>1e-12?std::clamp(-(ax*dx+ay*dy)/length2,0.,1.):0.;
      if(std::hypot(ax+t*dx,ay+t*dy)<region.radius-1e-9)return false;
    }
  }
  for(const auto & region:config_.turn_exclusions)
    if(violatesTurnExclusion(a,b,region))return false;
  if (std::abs(a.z - b.z) > config_.max_step_height + 1e-9 ||
    (std::hypot(a.x - b.x, a.y - b.y) < 1e-8 && std::abs(a.z - b.z) > 1e-8))
  {return false;}
  const double turn = std::remainder(b.yaw - a.yaw, 2.0 * M_PI);
  const int steps = std::max({1,
    static_cast<int>(std::ceil(distance3d(a, b) / config_.sweep_resolution)),
    static_cast<int>(std::ceil(std::abs(turn) / config_.sweep_angle))});
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const SurfacePoint point{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
      a.z + t * (b.z - a.z), a.yaw + t * turn};
    if (!hasSupport(point) || !hasClearance(point)) {return false;}
  }
  return true;
}


bool SurfaceGraph::walkable(const Key & key) const
{
  checkSearchBudget();
  const auto cached = walkability_cache_.find(key);
  if (cached != walkability_cache_.end()) {
    return cached->second;
  }
  const auto point = keyToPoint(key);
  const bool on_surface = hasSurface(point);
  double clearance = -1.0;
  if (on_surface && hasSupport(point)) {clearance = bodyClearance(point);}
  clearance_cache_[key] = clearance;
  const bool value = clearance >= config_.clearance_margin - 1e-9;
  walkability_cache_.emplace(key, value);
  return value;
}

std::optional<SurfacePoint> SurfaceGraph::snapToSurface(const SurfacePoint & requested) const
{
  checkSearchBudget();
  const Key center = pointToKey(requested);
  const int radius_steps = static_cast<int>(std::ceil(config_.snap_radius / config_.xy_resolution));
  const int vertical_steps =
    static_cast<int>(std::ceil(config_.snap_vertical_range / config_.z_resolution));

  if (config_.snap_radius <= 1e-9) {
    std::optional<SurfacePoint> best;
    double best_error = std::numeric_limits<double>::infinity();
    const double height=supportHeightAt(requested);
    const SurfacePoint continuous{requested.x,requested.y,height,requested.yaw};
    if(config_.infer_flat_support && std::isfinite(height) &&
      std::abs(height-requested.z)<=config_.snap_vertical_range &&
      hasSurface(continuous) && hasSupport(continuous) && hasClearance(continuous)) {
      best=continuous;best_error=std::abs(height-requested.z);
    }
    for (int dz = -vertical_steps; dz <= vertical_steps; ++dz) {
      const SurfacePoint point{requested.x, requested.y,
        (center.z + dz) * config_.z_resolution, requested.yaw};
      const double error = std::abs(point.z - requested.z);
      const double epsilon=tree_.getResolution()*0.01;
      if(best && std::abs(best->z-height)<1e-9 &&
        !(occupied(point.x,point.y,point.z-epsilon) &&
          !occupied(point.x,point.y,point.z+epsilon))) {continue;}
      if (error >= best_error || !hasSurface(point) ||
        !hasSupport(point) || !hasClearance(point)) {continue;}
      best = point; best_error = error;
    }
    return best;
  }

  std::optional<SurfacePoint> best;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int dx = -radius_steps; dx <= radius_steps; ++dx) {
    for (int dy = -radius_steps; dy <= radius_steps; ++dy) {
      const double horizontal = std::hypot(
        dx * config_.xy_resolution, dy * config_.xy_resolution);
      if (horizontal > config_.snap_radius + 1e-9) {
        continue;
      }
      for (int dz = -vertical_steps; dz <= vertical_steps; ++dz) {
        const Key candidate{center.x + dx, center.y + dy, center.z + dz, center.yaw};
        if (!walkable(candidate)) {
          continue;
        }
        const auto point = keyToPoint(candidate);
        const double cost = horizontal + 1.5 * std::abs(point.z - requested.z);
        if (cost < best_cost) {
          best_cost = cost;
          best = point;
        }
      }
    }
  }
  return best;
}


std::optional<SurfacePoint> SurfaceGraph::snapToSupportedSurface(const SurfacePoint & requested) const
{
  checkSearchBudget();
  if(hasSurface(requested) && hasSupport(requested))return requested;
  std::optional<SurfacePoint> best;double error=std::numeric_limits<double>::infinity();
  const double plane=supportHeightAt(requested);
  auto consider=[&](double z) {
    auto p=requested;p.z=z;
    const double e=std::abs(z-requested.z);
    if(e<=config_.snap_vertical_range && e<error && hasSurface(p) && hasSupport(p)){
      best=p;error=e;
    }
  };
  if(config_.infer_flat_support && std::isfinite(plane))consider(plane);
  const int centre=std::lround(requested.z/config_.z_resolution);
  const int vertical=std::ceil(config_.snap_vertical_range/config_.z_resolution);
  for(int dz=-vertical;dz<=vertical;++dz)consider((centre+dz)*config_.z_resolution);
  return best;
}

bool SurfaceGraph::isClearanceRecoveryMotionValid(const SurfacePoint & a,const SurfacePoint & b) const
{
  checkSearchBudget();
  const int count=std::ceil(std::hypot(a.x-b.x,a.y-b.y)/.0025);
  if(count>1) {
    auto previous=a;
    for(int i=1;i<=count;++i) {
      const double t=double(i)/count;
      SurfacePoint next{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),a.z+t*(b.z-a.z),b.yaw};
      if(!isClearanceRecoveryMotionValid(previous,next))return false;
      previous=next;
    }
    return true;
  }
  if(std::abs(std::remainder(b.yaw-a.yaw,2*M_PI))>1e-8 ||
     std::abs(a.z-b.z)>.02 || !hasSupport(a) || !hasSupport(b))return false;
  // No existing obstacle may penetrate more deeply; no new contact is allowed.
  // Evaluate each cylinder against each occupied voxel, not just the worst gap.
  const double r=config_.collision_radius,ox=config_.cylinder_offset*std::cos(a.yaw),
    oy=config_.cylinder_offset*std::sin(a.yaw);
  const double reach=r+std::abs(config_.cylinder_offset)+tree_.getResolution();
  const octomap::point3d low(std::min(a.x,b.x)-reach,std::min(a.y,b.y)-reach,
    std::min(a.z,b.z)+config_.body_bottom);
  const octomap::point3d high(std::max(a.x,b.x)+reach,std::max(a.y,b.y)+reach,
    std::max(a.z,b.z)+config_.body_height);
  for(const auto & region:config_.excluded_regions) {
    if(a.z<region.min_z || a.z>region.max_z)continue;
    const double ga=distanceToRegion(a.x,a.y,region)-r-config_.cylinder_offset;
    const double gb=distanceToRegion(b.x,b.y,region)-r-config_.cylinder_offset;
    if(gb<std::min(0.,ga)-1e-9)return false;
  }
  for(auto it=tree_.begin_leafs_bbx(low,high);it!=tree_.end_leafs_bbx();++it) {
    if(!tree_.isNodeOccupied(*it))continue;
    const double half=it.getSize()*.5;
    const auto gap=[&](const SurfacePoint & p,double sign) {
      if(it.getZ()+half<=p.z+config_.body_bottom+1e-9 ||
         it.getZ()-half>=p.z+config_.body_height-1e-9)
        return std::numeric_limits<double>::infinity();
      const double dx=std::max(0.,std::abs(p.x+sign*ox-it.getX())-half);
      const double dy=std::max(0.,std::abs(p.y+sign*oy-it.getY())-half);
      return std::hypot(dx,dy)-r;
    };
    for(double sign:{-1.,1.})if(gap(b,sign)<std::min(0.,gap(a,sign))-1e-9)return false;
  }
  return bodyClearance(b)+1e-9>=bodyClearance(a);
}

bool SurfaceGraph::isWalkableSurface(const SurfacePoint & point) const
{
  return hasSupport(point) && hasClearance(point);
}


double SurfaceGraph::bodyClearanceAt(const SurfacePoint & point) const
{
  return bodyClearance(point);
}

std::optional<SurfacePoint> SurfaceGraph::findClearanceRecovery(
  const SurfacePoint & start, double required_margin, double max_distance,
  bool allow_partial) const
{
  if (config_.clearance_margin > 1e-9 || !std::isfinite(required_margin) ||
    !std::isfinite(max_distance) || required_margin <= 0 || max_distance <= 0 ||
    !hasSupport(start)) {return std::nullopt;}
  const double initial = bodyClearance(start);
  if(initial>=required_margin)return std::nullopt;
  std::optional<SurfacePoint> best;
  const double minimum_gain=std::min(.0005,max_distance*.1);
  double best_clearance=allow_partial?initial+minimum_gain:required_margin-1e-9;
  const double step=std::min(.0025,config_.sweep_resolution);
  const int steps=std::max(1,int(std::ceil(max_distance/step)));
  if(steps>240)return std::nullopt;
  for(int direction=0;direction<16;++direction) {
    const double angle=direction*M_PI/8;
    auto previous=start;double previous_gap=initial;
    for(int i=1;i<=steps;++i) {
      const double distance=max_distance*i/steps;
      const auto point=snapToSupportedSurface({start.x+distance*std::cos(angle),
        start.y+distance*std::sin(angle),start.z,start.yaw});
      if(!point || !isClearanceRecoveryMotionValid(previous,*point))break;
      const double gap=bodyClearance(*point);
      if(gap+1e-9<previous_gap)break;
      previous=*point;previous_gap=gap;
      // Keep even a short valid prefix if a longer movement later becomes blocked.
      if((allow_partial || gap>=required_margin-1e-9) && gap>best_clearance) {
        best=previous;best_clearance=gap;
      }
    }
  }
  return best;
}

std::vector<SurfaceGraph::Key> SurfaceGraph::neighbors(
  const Key & key, const Key & min_key, const Key & max_key) const
{
  std::vector<Key> output;
  const auto source = keyToPoint(key);
  const int max_vertical_steps = static_cast<int>(
    std::floor(config_.max_step_height / config_.z_resolution + 1e-9));
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      const bool rotation_only = dx == 0 && dy == 0;
      const int nx = key.x + dx, ny = key.y + dy;
      if (nx < min_key.x || nx > max_key.x || ny < min_key.y || ny > max_key.y) {continue;}
      for (int dz = -max_vertical_steps; dz <= max_vertical_steps; ++dz) {
        if (rotation_only && dz != 0) {continue;}
        for (int turn : {0, -1, 1}) {
          if (rotation_only && turn == 0) {continue;}
          const Key candidate{nx, ny, key.z + dz,
            (key.yaw + turn + config_.yaw_bins) % config_.yaw_bins};
          if (candidate.z < min_key.z || candidate.z > max_key.z || !walkable(candidate)) {continue;}
          if (isMotionValid(source, keyToPoint(candidate))) {output.push_back(candidate);}
        }
      }
    }
  }
  return output;
}


std::vector<SurfacePoint> SurfaceGraph::plan(
  const SurfacePoint & start,const SurfacePoint & goal) const
{
  if(config_.narrow_clearance_margin<=0.0 ||
    config_.narrow_clearance_margin>=config_.clearance_margin) {return planAtMargin(start,goal);}
  auto nominal=config_;
  if(nominal.search_deadline!=std::chrono::steady_clock::time_point::max()) {
    const auto now=std::chrono::steady_clock::now();
    nominal.search_deadline=now+(nominal.search_deadline-now)/3;
  }
  try {
    auto path=SurfaceGraph(tree_,nominal).planAtMargin(start,goal);
    if(!path.empty()) {return path;}
  } catch(const SurfaceSearchTimeout &) {}
  auto fine=config_;
  fine.xy_resolution=std::min(fine.xy_resolution,tree_.getResolution()*0.5);
  if(fine.search_deadline!=std::chrono::steady_clock::time_point::max()) {
    const auto now=std::chrono::steady_clock::now();
    fine.search_deadline=now+(fine.search_deadline-now)/2;
  }
  try {
    auto path=SurfaceGraph(tree_,fine).planAtMargin(start,goal);
    if(!path.empty()) {return path;}
  } catch(const SurfaceSearchTimeout &) {}
  auto narrow=config_;
  narrow.clearance_margin=std::max(0.01,narrow.narrow_clearance_margin);
  // Search half-cell poses; collisions still use the full original voxels.
  narrow.xy_resolution=std::min(narrow.xy_resolution,tree_.getResolution()*0.5);
  return SurfaceGraph(tree_,narrow).planAtMargin(start,goal);
}

std::vector<SurfacePoint> SurfaceGraph::planAtMargin(
  const SurfacePoint & requested_start,
  const SurfacePoint & requested_goal) const
{
  checkSearchBudget();
  walkability_cache_.clear();
  clearance_cache_.clear();
  // The controller resolves support at the measured XY, never at a nearby
  // grid cell. Resolve that same stance before selecting an A* seed. In
  // particular, a neighbouring cell's height must not redefine the real floor.
  auto stance_config = config_;
  stance_config.snap_radius = 0.0;
  const auto measured_start = SurfaceGraph(tree_, stance_config).snapToSurface(requested_start);
  if (!measured_start) {return {};}
  const auto snapped_start = snapToSurface(*measured_start);
  const auto snapped_goal = snapToSurface(requested_goal);
  if (!snapped_start.has_value() || !snapped_goal.has_value()) {
    return {};
  }

  const SurfacePoint actual_start = *measured_start;
  const SurfacePoint actual_goal{requested_goal.x, requested_goal.y,
    snapped_goal->z, requested_goal.yaw};
  // Never snap the robot sideways across a wall or emit an unchecked endpoint.
  if (!isMotionValid(actual_start, *snapped_start) ||
    !isMotionValid(*snapped_goal, actual_goal)) {return {};}
  const Key start = pointToKey(*snapped_start);
  const Key goal = pointToKey(*snapped_goal);
  if(!walkable(start) || !walkable(goal) ||
    !isMotionValid(*snapped_start,keyToPoint(start)) ||
    !isMotionValid(keyToPoint(goal),*snapped_goal)) {return {};}
  const int pad_xy = static_cast<int>(std::ceil(config_.search_padding_xy / config_.xy_resolution));
  const int pad_z = static_cast<int>(std::ceil(config_.search_padding_z / config_.z_resolution));
  const Key min_key{
    std::min(start.x, goal.x) - pad_xy,
    std::min(start.y, goal.y) - pad_xy,
    std::min(start.z, goal.z) - pad_z};
  const Key max_key{
    std::max(start.x, goal.x) + pad_xy,
    std::max(start.y, goal.y) + pad_xy,
    std::max(start.z, goal.z) + pad_z};

  // Orientation-free distance around walls guides the full pose search.
  // The central disk is contained inside every orientation of the two-cylinder
  // body. This relaxation only supplies a heuristic; full body/support sweeps
  // remain mandatory for every actual edge.
  auto relaxed_config=config_;
  relaxed_config.collision_radius=std::sqrt(std::max(0.0,config_.collision_radius*config_.collision_radius-config_.cylinder_offset*config_.cylinder_offset));
  relaxed_config.cylinder_offset=0.0;
  relaxed_config.require_known_clearance=false;
  SurfaceGraph relaxed(tree_,relaxed_config);
  std::unordered_map<Key,double,KeyHash> relaxed_gap;
  const auto traversable=[&](const Key & k) {
    const auto found=relaxed_gap.find(k);
    if(found!=relaxed_gap.end()) {return found->second>=config_.clearance_margin-1e-9;}
    const auto p=keyToPoint(k);
    const bool disk_contained=std::abs(config_.cylinder_offset)<=config_.collision_radius;
    const double gap=!hasSurface(p)?-1.0:(disk_contained?relaxed.bodyClearance(p):config_.preferred_clearance+config_.clearance_margin);
    relaxed_gap.emplace(k,gap);return gap>=config_.clearance_margin-1e-9;
  };
  struct DistanceEntry {double distance;Key key;};
  struct DistanceGreater {bool operator()(const DistanceEntry & a,const DistanceEntry & b) const {return a.distance>b.distance;}};
  std::priority_queue<DistanceEntry,std::vector<DistanceEntry>,DistanceGreater> distances_open;
  std::unordered_map<Key,double,KeyHash> distances;
  Key distance_goal=goal;distance_goal.yaw=0;
  if(!traversable(distance_goal)) {return {};}
  distances[distance_goal]=0;distances_open.push({0,distance_goal});
  const int vertical_steps=static_cast<int>(std::floor(config_.max_step_height/config_.z_resolution+1e-9));
  int distance_iterations=0;
  while(!distances_open.empty() && distance_iterations++<config_.max_iterations) {
    checkSearchBudget();
    const auto current=distances_open.top();distances_open.pop();
    if(current.distance>distances.at(current.key)+1e-9) {continue;}
    const auto from=keyToPoint(current.key);
    for(int dx=-1;dx<=1;++dx)for(int dy=-1;dy<=1;++dy) {
      if(dx==0 && dy==0) {continue;}
      for(int dz=-vertical_steps;dz<=vertical_steps;++dz) {
        const Key next{current.key.x+dx,current.key.y+dy,current.key.z+dz,0};
        if(next.x<min_key.x || next.x>max_key.x || next.y<min_key.y || next.y>max_key.y ||
          next.z<min_key.z || next.z>max_key.z || !traversable(next)) {continue;}
        const auto to=keyToPoint(next);
        const double gap=std::min(relaxed_gap.at(current.key),relaxed_gap.at(next));
        const double penalty=config_.preferred_clearance>1e-9?config_.clearance_weight*std::max(0.0,1.0-(gap-config_.clearance_margin)/config_.preferred_clearance):0.0;
        const double candidate=current.distance+std::hypot(to.x-from.x,to.y-from.y)*(1.0+penalty)+config_.vertical_cost_weight*std::abs(to.z-from.z);
        const auto old=distances.find(next);
        if(old!=distances.end() && old->second<=candidate+1e-9) {continue;}
        distances[next]=candidate;distances_open.push({candidate,next});
      }
    }
  }
  Key distance_start=start;distance_start.yaw=0;
  if(distances_open.empty() && !distances.count(distance_start)) {return {};}

  struct QueueEntry
  {
    double f{0.0};
    double g{0.0};
    Key key;
  };
  struct Greater
  {
    bool operator()(const QueueEntry & a, const QueueEntry & b) const
    {
      if (a.f == b.f) {return a.g < b.g;}
      return a.f > b.f;
    }
  };

  auto heuristic = [this, &goal, &distances](const Key & key) {
      const auto a = keyToPoint(key);
      const auto b = keyToPoint(goal);
      // Eight-connected grid distance is a tighter admissible lower bound
      // than Euclidean distance; at equal f, prefer progress toward the goal.
      const double dx=std::abs(a.x-b.x),dy=std::abs(a.y-b.y);
      const double horizontal=std::max(dx,dy)+(std::sqrt(2.0)-1.0)*std::min(dx,dy);
      Key flat=key;flat.yaw=0;
      const auto found=distances.find(flat);
      const double geometric=found==distances.end()?horizontal+config_.vertical_cost_weight*std::abs(a.z-b.z):found->second;
      return geometric +
             config_.turn_cost_weight * std::abs(std::remainder(a.yaw - b.yaw, 2.0 * M_PI));
    };

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, Greater> open;
  std::unordered_map<Key, double, KeyHash> g_score;
  std::unordered_map<Key, Key, KeyHash> came_from;
  std::unordered_set<Key,KeyHash> expanded;
  g_score[start] = 0.0;
  // Prioritize a feasible route in this real-time pose search. Weighting the
  // lower bound trades cost optimality for latency; all body/support edges
  // still undergo exactly the same collision and swept-motion checks.
  constexpr double heuristic_weight = 1.5;
  open.push({heuristic_weight * heuristic(start), 0.0, start});

  Key reached = start;
  bool success = false;
  int iterations = 0;
  while (!open.empty() && iterations++ < config_.max_iterations) {
    checkSearchBudget();
    const QueueEntry current = open.top();
    open.pop();
    const auto known_g = g_score.find(current.key);
    if (known_g == g_score.end() || current.g > known_g->second + 1e-9) {
      continue;
    }
    if(!expanded.insert(current.key).second) {continue;}


    const auto current_point = keyToPoint(current.key);
    if (current.key == goal) {
      reached = current.key;
      success = true;
      break;
    }

    for (const auto & next : neighbors(current.key, min_key, max_key)) {
      if(expanded.count(next)) {continue;}

      const auto next_point = keyToPoint(next);
      const double horizontal = std::hypot(
        next_point.x - current_point.x, next_point.y - current_point.y);
      const double vertical = std::abs(next_point.z - current_point.z);
      const double clearance = std::min(
        clearance_cache_.at(current.key), clearance_cache_.at(next));
      const double penalty = config_.clearance_weight * std::max(0.0,
        1.0 - (clearance - config_.clearance_margin) / config_.preferred_clearance);
      const double turn = std::abs(std::remainder(next_point.yaw - current_point.yaw, 2.0 * M_PI));
      const double step_cost = horizontal * (1.0 + penalty) +
        config_.vertical_cost_weight * vertical +
        config_.turn_cost_weight * turn * (horizontal > 1e-9 ? 2.0 : 1.0);
      const double tentative = current.g + step_cost;
      const auto old = g_score.find(next);
      if (old == g_score.end() || tentative + 1e-9 < old->second) {
        g_score[next] = tentative;
        came_from[next] = current.key;
        open.push({tentative + heuristic_weight * heuristic(next), tentative, next});
      }
    }
  }

  if (!success) {
    if(!open.empty())throw SurfaceSearchTimeout();
    return {};
  }

  std::vector<SurfacePoint> reversed;
  reversed.push_back(keyToPoint(reached));
  while (!(reached == start)) {
    const auto parent = came_from.find(reached);
    if (parent == came_from.end()) {
      return {};
    }
    reached = parent->second;
    reversed.push_back(keyToPoint(reached));
  }
  std::reverse(reversed.begin(), reversed.end());
  if (distance3d(actual_start, reversed.front()) > 1e-8 ||
    std::abs(std::remainder(actual_start.yaw - reversed.front().yaw, 2.0 * M_PI)) > 1e-8)
  {reversed.insert(reversed.begin(), actual_start);}
  if (distance3d(actual_goal, reversed.back()) > 1e-8 ||
    std::abs(std::remainder(actual_goal.yaw - reversed.back().yaw, 2.0 * M_PI)) > 1e-8)
  {reversed.push_back(actual_goal);}
  // Keep the verified pose sequence; no unchecked corner cutting or yaw overwrite.
  return reversed;
}

}  // namespace go2_surface_3d_planner
