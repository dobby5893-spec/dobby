#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace go2_surface_3d_planner {
struct SurfaceExclusionRegion {double x,y,yaw,half_length,half_width,min_z,max_z;};
inline std::vector<SurfaceExclusionRegion> decodeSurfaceRegions(const std::vector<double> & data) {
 if(data.size()%7) throw std::invalid_argument("Stair regions require seven values per rectangle");
 std::vector<SurfaceExclusionRegion> result;
 for(std::size_t i=0;i<data.size();i+=7) {
  if(!std::all_of(data.begin()+i,data.begin()+i+7,[](double v){return std::isfinite(v);}) ||
    data[i+3]<=0 || data[i+4]<=0 || data[i+5]>data[i+6]) throw std::invalid_argument("Invalid stair region");
  result.push_back({data[i],data[i+1],data[i+2],data[i+3],data[i+4],data[i+5],data[i+6]});
 }
 return result;
}
inline double distanceToRegion(double x,double y,const SurfaceExclusionRegion & r) {
 const double c=std::cos(r.yaw),s=std::sin(r.yaw),dx=x-r.x,dy=y-r.y;
 return std::hypot(std::max(0.,std::abs(c*dx+s*dy)-r.half_length),
   std::max(0.,std::abs(-s*dx+c*dy)-r.half_width));
}
}
