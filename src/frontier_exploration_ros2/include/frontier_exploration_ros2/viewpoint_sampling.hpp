#pragma once
#include "frontier_exploration_ros2/frontier_types.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>

namespace frontier_exploration_ros2 {
using Viewpoint = std::pair<double, double>;

// Geometry only; callers must check free space, reachability and full 3D motion.
inline std::vector<Viewpoint> sampleViewpointFan(
  Viewpoint reference, Viewpoint direction, const std::vector<double> & distances,
  std::size_t limit)
{
  std::vector<Viewpoint> points;
  const double norm=std::hypot(direction.first,direction.second);
  if(norm<1e-6)return points;
  const double heading=std::atan2(direction.second,direction.first);
  for(double distance:distances) {
    if(!std::isfinite(distance) || distance<=0)continue;
    for(double degrees:{0.,-30.,30.,-60.,60.}) {
      if(points.size()>=limit)return points;
      const double angle=heading+degrees*3.14159265358979323846/180.;
      points.emplace_back(reference.first+distance*std::cos(angle),
        reference.second+distance*std::sin(angle));
    }
  }
  return points;
}

// Expanded observation rings cover the opposite sides of doorways as well.
// Every position still requires free-space, visibility and full-body validation.
inline std::vector<Viewpoint> sampleExpandedViewpoints(
  Viewpoint reference,Viewpoint direction,unsigned level)
{
  if(level==0)return {};
  const double heading=std::atan2(direction.second,direction.first);
  std::vector<double> radii{.35,.5,.8,1.1,1.4};
  if(level>=2) {radii.push_back(1.8);radii.push_back(2.4);}
  if(level>=3)radii.push_back(3.2);
  std::vector<Viewpoint> result;
  for(double r:radii)for(double degrees:{-90.,90.,-120.,120.,180.})
    result.emplace_back(reference.first+r*std::cos(heading+degrees*M_PI/180.),
      reference.second+r*std::sin(heading+degrees*M_PI/180.));
  if(level>=2)for(double r:{1.8,2.4})for(double degrees:{0.,-30.,30.,-60.,60.})
    result.emplace_back(reference.first+r*std::cos(heading+degrees*M_PI/180.),
      reference.second+r*std::sin(heading+degrees*M_PI/180.));
  if(level>=3)for(double degrees:{0.,-30.,30.,-60.,60.})
    result.emplace_back(reference.first+3.2*std::cos(heading+degrees*M_PI/180.),
      reference.second+3.2*std::sin(heading+degrees*M_PI/180.));
  return result;
}

// Count distinct first unknown hits. Walls occlude rays; remote unrelated
// boundaries do not inflate this frontier's predicted gain.
inline int viewpointVisibleGain(
  const OccupancyGrid2d & map, Viewpoint point, Viewpoint reference)
{
  std::set<std::pair<int,int>> hits;
  const double step=std::max(.025,map.map().info.resolution*.5);
  auto cast=[&](double angle,double range) {
    for(double d=step;d<=range+step*.25;d+=step) {
      const double x=point.first+d*std::cos(angle),y=point.second+d*std::sin(angle);
      int mx=0,my=0;
      if(!map.worldToMapNoThrow(x,y,mx,my))break;
      const int value=map.getCost(mx,my);
      if(value==static_cast<int>(OccupancyGrid2d::CostValues::NoInformation)) {
        if(std::hypot(x-reference.first,y-reference.second)<=2.)
          hits.emplace(mx,my);
        break;
      }
      if(value!=static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))break;
    }
  };
  for(int ray=0;ray<72;++ray)cast(ray*2.*3.14159265358979323846/72.,4.);
  // Explicit rays to small boundary cells prevent angular undersampling from
  // rejecting a one-cell residual between two of the 5-degree scan rays.
  int cx,cy;
  if(map.worldToMapNoThrow(reference.first,reference.second,cx,cy)) {
    const int reach=std::ceil(2./map.map().info.resolution);
    unsigned rays=0;
    for(int y=std::max(0,cy-reach);y<std::min(map.getSizeY(),cy+reach+1);++y)
      for(int x=std::max(0,cx-reach);x<std::min(map.getSizeX(),cx+reach+1);++x) {
        if(rays>=256 || map.getCost(x,y)>=0)continue;
        bool boundary=false;
        for(auto d:{std::pair<int,int>{-1,0},{1,0},{0,-1},{0,1}})
          if(x+d.first>=0 && x+d.first<map.getSizeX() && y+d.second>=0 &&
            y+d.second<map.getSizeY() && map.getCost(x+d.first,y+d.second)==0)boundary=true;
        if(!boundary)continue;
        auto target=map.mapToWorld(x,y);
        const double distance=std::hypot(target.first-point.first,target.second-point.second);
        if(distance>4. || std::hypot(target.first-reference.first,target.second-reference.second)>2.)continue;
        ++rays;cast(std::atan2(target.second-point.second,target.first-point.first),distance+step);
      }
  }
  return static_cast<int>(hits.size());
}

inline double viewpointUtility(
  int gain,double path_cost,double turn)
{
  return .65*gain/(gain+8.)-.25*path_cost/(path_cost+5.)-
    .10*std::abs(std::remainder(turn,2.*3.14159265358979323846))/
    3.14159265358979323846;
}
}  // namespace frontier_exploration_ros2
