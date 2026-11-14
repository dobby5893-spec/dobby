#pragma once
#include "frontier_exploration_ros2/observation_grid.hpp"
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include "go2_surface_3d_planner/surface_graph.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>
#include <limits>

namespace frontier_exploration_ros2 {


// This disk is only a coarse route filter. Every dispatched motion is still
// checked against the oriented full body, support and stair exclusions in 3D.
inline std::vector<go2_surface_3d_planner::SurfacePoint> mappedObservationRoute(
  const ObservationGrid & grid,
  const go2_surface_3d_planner::SurfacePoint & start,
  const go2_surface_3d_planner::SurfacePoint & goal, double radius,
  std::chrono::steady_clock::time_point deadline, bool verified_start = false)
{
  using go2_surface_3d_planner::SurfacePoint;
  const int w=grid.width,h=grid.height;
  if(w<=0 || h<=0 || w>2000 || h>2000 || grid.resolution<=0 ||
     grid.cells.size()!=static_cast<size_t>(w)*h) return {};
  const double res=grid.resolution,c=std::cos(grid.yaw),s=std::sin(grid.yaw);
  auto index=[&](const SurfacePoint & p) {
    const double dx=p.x-grid.x,dy=p.y-grid.y;
    const int x=std::floor((c*dx+s*dy)/res),y=std::floor((-s*dx+c*dy)/res);
    return x>=0 && x<w && y>=0 && y<h?y*w+x:-1;
  };
  auto point=[&](int i) {
    const double x=(i%w+.5)*res,y=(i/w+.5)*res;
    return SurfacePoint{grid.x+c*x-s*y,grid.y+s*x+c*y,start.z,0};
  };
  const int begin=index(start),end=index(goal);
  if(begin<0 || end<0) return {};
  std::vector<bool> free(w*h);
  for(int i=0;i<w*h;++i) free[i]=grid.cells[i]>=0 && grid.cells[i]<90;
  const int reach=std::ceil(radius/res+.5);
  std::vector<std::pair<int,int>> offsets;
  for(int dy=-reach;dy<=reach;++dy) for(int dx=-reach;dx<=reach;++dx)
    if(std::hypot(std::max(0.,std::abs(dx)*res-.5*res),
                  std::max(0.,std::abs(dy)*res-.5*res))<=radius)
      offsets.emplace_back(dx,dy);
  for(int i=0;i<w*h;++i) {
    if((i%1024)==0 && std::chrono::steady_clock::now()>=deadline) return {};
    if(grid.cells[i]>=0 && grid.cells[i]<90) continue;
    for(auto [dx,dy]:offsets) {
      const int x=i%w+dx,y=i/w+dy;
      if(x>=0 && x<w && y>=0 && y<h) free[y*w+x]=false;
    }
  }
  // Cell-centre quantization can put a physically verified measured start
  // inside the coarse inflation of a diagonal wall cell. Permit departure
  // from that one raw-free start cell; every other inflated cell stays blocked.
  // The caller must verify the exact measured full-body stance in 3D first.
  if(verified_start && grid.cells[begin]>=0 && grid.cells[begin]<90)
    free[begin]=true;
  if(!free[begin] || !free[end]) return {};
  auto heuristic=[&](int i){return std::hypot(i%w-end%w,i/w-end/w);};
  std::vector<double> costs(w*h,std::numeric_limits<double>::infinity());
  std::vector<int> parent(w*h,-1);
  using Entry=std::pair<double,int>;
  std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
  costs[begin]=0;open.emplace(heuristic(begin),begin);
  while(!open.empty()) {
    if(std::chrono::steady_clock::now()>=deadline) return {};
    const auto [priority,cur]=open.top();open.pop();
    if(priority>costs[cur]+heuristic(cur)+1e-9) continue;
    if(cur==end) {
      std::vector<SurfacePoint> route;
      for(int i=end;i!=-1;i=parent[i]) route.push_back(point(i));
      std::reverse(route.begin(),route.end());return route;
    }
    for(int dy=-1;dy<=1;++dy) for(int dx=-1;dx<=1;++dx) {
      if(dx==0 && dy==0) continue;
      const int x=cur%w+dx,y=cur/w+dy;
      if(x<0 || x>=w || y<0 || y>=h || !free[y*w+x]) continue;
      if(dx && dy && (!free[(cur/w)*w+x] || !free[y*w+cur%w])) continue;
      const int next=y*w+x;const double cost=costs[cur]+std::hypot(dx,dy);
      if(cost+1e-9>=costs[next]) continue;
      costs[next]=cost;parent[next]=cur;open.emplace(cost+heuristic(next),next);
    }
  }
  return {};
}

// Choose a long prefix only through roomy, known, nearly straight space.
// This selects candidates; the complete oriented 3D path is still mandatory.
inline std::vector<size_t> mappedAdvanceTargets(
  const std::vector<go2_surface_3d_planner::SurfacePoint> & route,
  const ObservationGrid & grid,const go2_surface_3d_planner::SurfacePoint & start,
  double body_radius,double maximum_advance)
{
  if(route.size()<2 || grid.resolution<=0 || !std::isfinite(maximum_advance) || maximum_advance<=0 ||
      grid.cells.size()!=size_t(grid.width)*grid.height)return {};
  const double c=std::cos(grid.yaw),s=std::sin(grid.yaw);
  auto clear=[&](const auto & p) {
    const double dx=p.x-grid.x,dy=p.y-grid.y;
    int x=std::floor((c*dx+s*dy)/grid.resolution),y=std::floor((-s*dx+c*dy)/grid.resolution);
    const double radius=body_radius+.18;
    const int n=std::ceil(radius/grid.resolution);
    for(int iy=-n;iy<=n;++iy)for(int ix=-n;ix<=n;++ix) {
      if(std::hypot(ix*grid.resolution,iy*grid.resolution)>radius+grid.resolution*.5)continue;
      int nx=x+ix,ny=y+iy;
      if(nx<0 || ny<0 || nx>=grid.width || ny>=grid.height ||
        grid.cells[ny*grid.width+nx]!=0)return false;
    }
    return true;
  };
  std::vector<double> arc(route.size(),0);
  arc[0]=std::hypot(start.x-route[0].x,start.y-route[0].y);
  for(size_t i=1;i<route.size();++i)
    arc[i]=arc[i-1]+std::hypot(route[i].x-route[i-1].x,route[i].y-route[i-1].y);
  double safe=maximum_advance;
  for(size_t i=0;i<route.size() && arc[i]<=maximum_advance;++i) {
    if(!clear(route[i])) {safe=std::max(0.,arc[i]-.5);break;}
    size_t before=i,after=i;
    while(before>0 && arc[i]-arc[before]<.45)--before;
    while(after+1<route.size() && arc[after]-arc[i]<.45)++after;
    double ax=route[i].x-route[before].x,ay=route[i].y-route[before].y;
    double bx=route[after].x-route[i].x,by=route[after].y-route[i].y;
    double norm=std::hypot(ax,ay)*std::hypot(bx,by);
    if(norm>.01 && (ax*bx+ay*by)/norm<std::cos(M_PI/6)) {
      safe=std::max(0.,arc[i]-.5);break;
    }
  }
  // A verified narrow start must still be able to advance through a door,
  // but it does so in a short stage instead of a long open-space stride.
  const double limit=std::min(maximum_advance,safe>=1.5?safe:1.);
  std::vector<size_t> targets;
  for(double wanted:{limit,3.,2.,1.5,1.,.65,.4,.25}) {
    if(wanted>limit+1e-9)continue;
    auto it=std::upper_bound(arc.begin(),arc.end(),wanted+1e-9);
    if(it==arc.begin())continue;
    size_t index=std::distance(arc.begin(),it)-1;
    if(index>0 && std::find(targets.begin(),targets.end(),index)==targets.end())targets.push_back(index);
  }
  return targets;
}

inline std::optional<go2_surface_3d_planner::SurfacePoint> selectMappedObservation(
  const octomap::OcTree & tree,go2_surface_3d_planner::SurfaceSearchConfig config,
  const go2_surface_3d_planner::SurfacePoint & start,
  const go2_surface_3d_planner::SurfacePoint & requested,const ObservationGrid & grid,
  const std::vector<std::pair<double,double>> & excluded_observations = {},
  double minimum_advance = .5, double maximum_advance = 5., bool * timed_out = nullptr,
  const ExplorationBoundaryPolygon & boundary = {},double boundary_margin = 0.)
{
  using namespace go2_surface_3d_planner;
  if(timed_out)*timed_out=false;
  if(config.search_deadline==std::chrono::steady_clock::time_point::max())
    config.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  if(config.infer_flat_support && !configureSupportPlane(config,tree,start))
    return std::nullopt;
  auto stance=start;
  if(config.infer_flat_support) stance.z-=config.support_reference_height;
  auto start_config=config;
  start_config.clearance_margin=std::max(.01,config.narrow_clearance_margin>0?
    config.narrow_clearance_margin:config.clearance_margin);
  const auto measured=SurfaceGraph(tree,start_config).diagnosePose(stance);
  if(!measured.surface || !measured.support || !measured.clearance)
    return std::nullopt;
  const auto route=mappedObservationRoute(grid,start,requested,
    config.collision_radius+start_config.clearance_margin,config.search_deadline,true);
  if(route.size()<2) return std::nullopt;
  const auto origin=SurfaceGraph(tree,config).snapToSurface(stance);
  if(!origin) return std::nullopt;
  // Leave one snapping step inside the length cap; an endpoint may snap
   // forward by a grid cell during the oriented surface search.
   const auto targets=mappedAdvanceTargets(route,grid,start,config.collision_radius,
     maximum_advance>1.5?maximum_advance-std::max(.15,grid.resolution):maximum_advance);
  auto excluded=[&](double x,double y) {
    return std::any_of(excluded_observations.begin(),excluded_observations.end(),
      [&](const auto & old){return std::hypot(x-old.first,y-old.second)<=.60;});
  };
  // Observation history excludes a scan destination, not the intervening
  // floor. Before the short-prefix heuristic, try the actual new destination
  // with a full swept-body route, including detours through previously scanned
  // space. A narrow start or a corner alone must not cap every attempt at 1m.
  if(!excluded(requested.x,requested.y) &&
     is_inside_exploration_boundary({requested.x,requested.y},boundary,boundary_margin) &&
     std::hypot(requested.x-start.x,requested.y-start.y)>=minimum_advance &&
     std::hypot(requested.x-start.x,requested.y-start.y)<=maximum_advance) {
    std::vector<double> margins{std::max(config.clearance_margin,start_config.clearance_margin)};
    if(config.narrow_clearance_margin>0 && config.narrow_clearance_margin<config.clearance_margin)
      margins.push_back(std::max(config.narrow_clearance_margin,start_config.clearance_margin));
    const auto remaining=config.search_deadline-std::chrono::steady_clock::now();
    const auto direct_deadline=std::chrono::steady_clock::now()+remaining/2;
    for(size_t m=0;m<margins.size();++m) {
      auto attempt=config;attempt.clearance_margin=margins[m];attempt.narrow_clearance_margin=0;
      const auto slice=(direct_deadline-std::chrono::steady_clock::now())/
        static_cast<int>(margins.size()-m);
      attempt.search_deadline=std::min(direct_deadline,std::chrono::steady_clock::now()+slice);
      if(std::chrono::steady_clock::now()>=attempt.search_deadline)break;
      try {
        const auto path=SurfaceGraph(tree,attempt).plan(stance,requested);
        double length=0;bool flat=true;
        for(size_t i=0;i<path.size();++i) {
          if(i)length+=std::hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y);
          if(std::abs(path[i].z-origin->z)>.08)flat=false;
        }
        if(!path.empty() && flat && length<=maximum_advance+1e-6 &&
           boundary_return_path(stance,path,boundary,boundary_margin) &&
           !excluded(path.back().x,path.back().y) &&
           std::hypot(path.back().x-requested.x,path.back().y-requested.y)<=.10 &&
           std::hypot(path.back().x-start.x,path.back().y-start.y)>=minimum_advance)
          return path.back();
      }catch(const SurfaceSearchTimeout &){if(timed_out)*timed_out=true;}
    }
  }
  std::vector<double> arc(route.size(),0.);
  arc[0]=std::hypot(start.x-route.front().x,start.y-route.front().y);
  for(size_t i=1;i<route.size();++i)
    arc[i]=arc[i-1]+std::hypot(route[i].x-route[i-1].x,route[i].y-route[i-1].y);
  std::vector<size_t> primary,short_targets;
  for(size_t i:targets) {
    if(arc[i]>1.+1e-9)primary.push_back(i);
    else short_targets.push_back(i);
  }
  // Include the closest legal departure, even when fixed distance rings skip
  // it. A short fallback must not start by retrying another difficult long leg.
  for(size_t i=1;i<route.size() && arc[i]<=1.+1e-9;++i) {
    if(std::hypot(route[i].x-start.x,route[i].y-start.y)<minimum_advance ||
       excluded(route[i].x,route[i].y))continue;
    if(std::find(short_targets.begin(),short_targets.end(),i)==short_targets.end())
      short_targets.push_back(i);
    break;
  }
  std::sort(short_targets.begin(),short_targets.end());
  short_targets.erase(std::unique(short_targets.begin(),short_targets.end()),short_targets.end());
  if(primary.empty() && !short_targets.empty())primary.push_back(short_targets.back());
  const auto began=std::chrono::steady_clock::now();
  if(began>=config.search_deadline)return std::nullopt;
  const auto remaining=config.search_deadline-began;
  const auto reserve=short_targets.empty()?std::chrono::steady_clock::duration::zero():
    std::min(remaining/2,std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::milliseconds(250)));
  const auto primary_deadline=config.search_deadline-reserve;
  auto search=[&](const std::vector<size_t> & indices,
      std::chrono::steady_clock::time_point deadline,bool short_phase)->std::optional<SurfacePoint> {
    for(size_t i:indices) {
      const size_t after=std::min(i+2,route.size()-1),before=i?i-1:0;
      const double tangent=std::atan2(route[after].y-route[before].y,route[after].x-route[before].x);
      if(!is_inside_exploration_boundary({route[i].x,route[i].y},boundary,boundary_margin) ||
         excluded(route[i].x,route[i].y) ||
         std::hypot(route[i].x-start.x,route[i].y-start.y)<minimum_advance)continue;
      const std::vector<double> yaws=short_phase?
        std::vector<double>{start.yaw,tangent,tangent+M_PI}:
        std::vector<double>{tangent,start.yaw,tangent+M_PI};
      std::vector<double> tried_yaws;
      for(double yaw:yaws) {
        if(std::any_of(tried_yaws.begin(),tried_yaws.end(),[&](double old) {
          return std::abs(std::remainder(yaw-old,2*M_PI))<.02;}))continue;
        tried_yaws.push_back(yaw);
        if(std::chrono::steady_clock::now()>=deadline)return std::nullopt;
        SurfacePoint candidate{route[i].x,route[i].y,origin->z,yaw};
        auto attempt=config;
        attempt.search_deadline=std::min(deadline,
          std::chrono::steady_clock::now()+std::chrono::milliseconds(350));
        try {
          const auto path=SurfaceGraph(tree,attempt).plan(stance,candidate);
          double length=path.empty()?0.:std::hypot(path.front().x-stance.x,path.front().y-stance.y);
          bool flat=true;
          for(size_t j=0;j<path.size();++j) {
            if(j)length+=std::hypot(path[j].x-path[j-1].x,path[j].y-path[j-1].y);
            if(std::abs(path[j].z-origin->z)>.08)flat=false;
          }
          if(!path.empty() && boundary_return_path(stance,path,boundary,boundary_margin) &&
             length<=maximum_advance+1e-6 &&
             (length<=1.5 || flat) && !excluded(path.back().x,path.back().y) &&
             std::hypot(path.back().x-start.x,path.back().y-start.y)>=minimum_advance)
            return path.back();
        }catch(const SurfaceSearchTimeout &) {if(timed_out)*timed_out=true;}
      }
    }
    return std::nullopt;
  };
  if(auto result=search(primary,primary_deadline,false))return result;
  if(auto result=search(short_targets,config.search_deadline,true))return result;
  return std::nullopt;
}
}
