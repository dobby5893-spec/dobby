#pragma once
#include "frontier_exploration_ros2/mapped_observation.hpp"
#include <array>
namespace frontier_exploration_ros2 {
struct VisibilitySensor {
 double x{0},y{0},z{0},qx{0},qy{0},qz{0},qw{1};
 double min_pitch{-7.22*3.141592653589793/180.},max_pitch{55.22*3.141592653589793/180.};
};
inline bool visibleUnknown3D(const octomap::OcTree & tree,const ObservationGrid & grid,
 const go2_surface_3d_planner::SurfacePoint & view,std::pair<double,double> reference,
 const VisibilitySensor & sensor,std::chrono::steady_clock::time_point deadline) {
 using namespace go2_surface_3d_planner;
 const double cy=std::cos(view.yaw),sy=std::sin(view.yaw);
 const double ox=view.x+cy*sensor.x-sy*sensor.y,oy=view.y+sy*sensor.x+cy*sensor.y,oz=view.z+sensor.z;
 const double gc=std::cos(grid.yaw),gs=std::sin(grid.yaw);
 if(grid.resolution<=0 || grid.cells.size()!=static_cast<size_t>(grid.width)*grid.height)return false;
 auto column=[&](double x,double y) {
   double dx=x-grid.x,dy=y-grid.y;
   int ix=std::floor((gc*dx+gs*dy)/grid.resolution),iy=std::floor((-gs*dx+gc*dy)/grid.resolution);
   return ix>=0 && ix<grid.width && iy>=0 && iy<grid.height?iy*grid.width+ix:-1;
 };
 int center=column(reference.first,reference.second);if(center<0)return false;
 const int reach=std::ceil(2./grid.resolution);unsigned tested=0;
 const octomap::point3d origin(ox,oy,oz);
 for(int y=std::max(0,center/grid.width-reach);y<std::min(grid.height,center/grid.width+reach+1);++y)
 for(int x=std::max(0,center%grid.width-reach);x<std::min(grid.width,center%grid.width+reach+1);++x) {
   if(std::chrono::steady_clock::now()>=deadline)throw SurfaceSearchTimeout();
   const int i=y*grid.width+x;if(grid.cells[i]>=0)continue;
   bool boundary=false;
   for(auto d:{std::pair<int,int>{-1,0},{1,0},{0,-1},{0,1}})
     if(x+d.first>=0 && x+d.first<grid.width && y+d.second>=0 && y+d.second<grid.height &&
       grid.cells[(y+d.second)*grid.width+x+d.first]==0)boundary=true;
   if(!boundary)continue;
   const double px=(x+.5)*grid.resolution,py=(y+.5)*grid.resolution;
   const double tx=grid.x+gc*px-gs*py,ty=grid.y+gs*px+gc*py;
   if(std::hypot(tx-reference.first,ty-reference.second)>2.)continue;
   if(++tested>256)return false;
   for(double height:{.02,.20,.40,.60,.80}) {
     const double tz=view.z+height,wx=tx-ox,wy=ty-oy,wz=tz-oz;
     const double range=std::sqrt(wx*wx+wy*wy+wz*wz);if(range<.1 || range>4.)continue;
     // Rotate the ray back through the body yaw and measured sensor mounting.
     double bx=cy*wx+sy*wy,by=-sy*wx+cy*wy,bz=wz;
     const double ux=-sensor.qx,uy=-sensor.qy,uz=-sensor.qz,qw=sensor.qw;
     const double cx=uy*bz-uz*by,cyq=uz*bx-ux*bz,cz=ux*by-uy*bx;
     const double lx=bx+2*(qw*cx+uy*cz-uz*cyq);
     const double ly=by+2*(qw*cyq+uz*cx-ux*cz);
     const double lz=bz+2*(qw*cz+ux*cyq-uy*cx);
     const double pitch=std::atan2(lz,std::hypot(lx,ly));
     if(pitch<sensor.min_pitch || pitch>sensor.max_pitch)continue;
     octomap::KeyRay keys;
     if(!tree.computeRayKeys(origin,octomap::point3d(tx,ty,tz),keys))continue;
     bool blocked=false,potential=false;
     for(const auto & key:keys) {
       const auto * node=tree.search(key);
       if(node && tree.isNodeOccupied(node)){blocked=true;break;}
       if(!node) {
         const auto p=tree.keyToCoord(key);
         if(column(p.x(),p.y())==i)potential=true;
       }
     }
     const auto * end=tree.search(tx,ty,tz);
     if(end && tree.isNodeOccupied(end))blocked=true;
     if(!end)potential=true;
     if(!blocked && potential)return true;
   }
 }
 return false;
}
}
