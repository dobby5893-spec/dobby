
#pragma once
#include "go2_multifloor_exploration/stair_transition.hpp"
#include <algorithm>
#include <cstdint>
#include <queue>
namespace go2_multifloor_exploration {
struct TransitionFootprint {
 std::vector<int8_t> cells;
 bool complete{false};
 size_t marked{0};
};
// Flood only observed elevated top surfaces connected to a detected stair chain.
// No region rectangle and no whole unknown component is filled.
inline TransitionFootprint transitionFootprint(
 const octomap::OcTree & tree,const std::vector<StairCandidate> & stairs,
 int width,int height,double resolution,double ox,double oy,double floor,
 double max_step,std::chrono::steady_clock::time_point deadline) {
 TransitionFootprint result;result.cells.assign(width*height,0);
 if(stairs.empty()){result.complete=true;return result;}
 const double nan=std::numeric_limits<double>::quiet_NaN();
 std::vector<double> tops(width*height,nan);
 const double r=tree.getResolution();
 auto expired=[&](){return std::chrono::steady_clock::now()>=deadline;};
 if(expired())return result;
 for(auto it=tree.begin_leafs();it!=tree.end_leafs();++it) {
  if(expired())return result;
  if(!tree.isNodeOccupied(*it))continue;
  double top=it.getZ()+it.getSize()*.5;
  if(top<floor+.10 || top>floor+1.60)continue;
  int xmin=std::max(0,int(std::floor((it.getX()-it.getSize()*.5-ox)/resolution)));
  int xmax=std::min(width-1,int(std::floor((it.getX()+it.getSize()*.5-ox-1e-7)/resolution)));
  int ymin=std::max(0,int(std::floor((it.getY()-it.getSize()*.5-oy)/resolution)));
  int ymax=std::min(height-1,int(std::floor((it.getY()+it.getSize()*.5-oy-1e-7)/resolution)));
  for(int y=ymin;y<=ymax;++y)for(int x=xmin;x<=xmax;++x) {
   // Sample the occupied voxel's own XY; grid centres may lie outside a fine leaf.
   const double px=std::clamp(ox+(x+.5)*resolution,it.getX()-it.getSize()*.5+r*.01,it.getX()+it.getSize()*.5-r*.01);
   const double py=std::clamp(oy+(y+.5)*resolution,it.getY()-it.getSize()*.5+r*.01,it.getY()+it.getSize()*.5-r*.01);
   auto above=tree.search(px,py,top+r*.1);
   if(above && tree.isNodeOccupied(above))continue;
   // Reject an intermediate unobserved wall return mistaken for its top.
   bool vertical=false;
   for(double dz:{.10,.20}) {
    auto n=tree.search(px,py,top+dz);
    if(n && tree.isNodeOccupied(n))vertical=true;
   }
   if(vertical)continue;
   auto & z=tops[y*width+x];if(!std::isfinite(z) || top>z)z=top;
  }
 }
 std::vector<int> queue;
 for(const auto & stair:stairs)for(const auto & p:stair.chain) {
  int x=std::floor((p.x-ox)/resolution),y=std::floor((p.y-oy)/resolution);
  if(x<0 || y<0 || x>=width || y>=height)continue;
  int i=y*width+x;
  if(std::isfinite(tops[i]) && std::abs(tops[i]-p.z)<=r+.01 && !result.cells[i]){
   result.cells[i]=100;queue.push_back(i);
  }
 }
 for(size_t head=0;head<queue.size();++head) {
  if(expired())return result;
  int i=queue[head],x=i%width,y=i/width;
  for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
   if(dx==0 && dy==0)continue;
   int nx=x+dx,ny=y+dy,j=ny*width+nx;
   if(nx<0 || ny<0 || nx>=width || ny>=height || result.cells[j] || !std::isfinite(tops[j]))continue;
   if(std::abs(tops[j]-tops[i])>max_step+1e-6)continue;
   result.cells[j]=100;queue.push_back(j);
  }
 }
 result.complete=true;result.marked=queue.size();return result;
}
}
