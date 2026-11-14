#pragma once
#include "frontier_exploration_ros2/observation_grid.hpp"
#include <queue>
#include <chrono>
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include <deque>
#include <set>
#include <optional>

namespace frontier_exploration_ros2 {
using RelocationPoint=std::pair<double,double>;
struct KnownBounds {
 double min_x{0},max_x{0},min_y{0},max_y{0};
 RelocationPoint center() const {return {(min_x+max_x)/2,(min_y+max_y)/2};}
 RelocationPoint reflect(RelocationPoint p) const {return {min_x+max_x-p.first,min_y+max_y-p.second};}
};
inline RelocationPoint gridWorld(const ObservationGrid & g,int x,int y) {
 double px=(x+.5)*g.resolution,py=(y+.5)*g.resolution;
 return {g.x+std::cos(g.yaw)*px-std::sin(g.yaw)*py,
   g.y+std::sin(g.yaw)*px+std::cos(g.yaw)*py};
}
inline std::optional<KnownBounds> knownBounds(const ObservationGrid & g) {
 if(g.width<=0 || g.height<=0 || !std::isfinite(g.resolution+g.x+g.y+g.yaw) ||
    g.resolution<=0 || g.cells.size()!=size_t(g.width)*g.height)return {};
 std::optional<KnownBounds> b;
 for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x)if(g.cells[y*g.width+x]>=0) {
   auto p=gridWorld(g,x,y);
   if(!b)b=KnownBounds{p.first,p.first,p.second,p.second};
   else {b->min_x=std::min(b->min_x,p.first);b->max_x=std::max(b->max_x,p.first);
     b->min_y=std::min(b->min_y,p.second);b->max_y=std::max(b->max_y,p.second);}
 }
 return b;
}
// One connected, inflated component. Unknowns and obstacles use the same disk
// as mappedObservationRoute; diagonal corners cannot be cut.
inline std::vector<bool> mappedReachable(const ObservationGrid & g,RelocationPoint start,
 double radius,std::chrono::steady_clock::time_point deadline) {
 if(!knownBounds(g) || size_t(g.width)*g.height>200000 || radius<0)return {};
 int w=g.width,h=g.height;double c=std::cos(g.yaw),s=std::sin(g.yaw);
 int sx=std::floor((c*(start.first-g.x)+s*(start.second-g.y))/g.resolution);
 int sy=std::floor((-s*(start.first-g.x)+c*(start.second-g.y))/g.resolution);
 if(sx<0 || sx>=w || sy<0 || sy>=h || g.cells[sy*w+sx]<0 || g.cells[sy*w+sx]>=90)return {};
 std::vector<bool> free(w*h),seen(w*h,false);
 for(int i=0;i<w*h;++i)free[i]=g.cells[i]>=0 && g.cells[i]<90;
 int reach=std::ceil(radius/g.resolution+.5);std::vector<std::pair<int,int>> offsets;
 for(int dy=-reach;dy<=reach;++dy)for(int dx=-reach;dx<=reach;++dx)
   if(std::hypot(std::max(0.,std::abs(dx)*g.resolution-.5*g.resolution),
     std::max(0.,std::abs(dy)*g.resolution-.5*g.resolution))<=radius)offsets.emplace_back(dx,dy);
 for(int i=0;i<w*h;++i) {
   if((i%1024)==0 && std::chrono::steady_clock::now()>=deadline)return {};
   if(g.cells[i]>=0 && g.cells[i]<90)continue;
   for(auto d:offsets) {
     int x=i%w+d.first,y=i/w+d.second;
     if(x>=0 && x<w && y>=0 && y<h)free[y*w+x]=false;
   }
 }
 // This is only candidate generation. Exact measured start and every dispatched
 // segment still require full oriented-body/support validation.
 free[sy*w+sx]=true;std::queue<int> q;q.push(sy*w+sx);seen[sy*w+sx]=true;
 while(!q.empty()) {
   int i=q.front();q.pop();
   if((i%1024)==0 && std::chrono::steady_clock::now()>=deadline)return {};
   for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
     if(!dx && !dy)continue;
     int x=i%w+dx,y=i/w+dy;
     if(x<0 || x>=w || y<0 || y>=h || seen[y*w+x] || !free[y*w+x])continue;
     if(dx && dy && (!free[(i/w)*w+x] || !free[y*w+i%w]))continue;
     seen[y*w+x]=true;q.push(y*w+x);
   }
 }
 return seen;
}
struct SymmetricTarget {RelocationPoint center,mirror,destination;};
inline std::optional<SymmetricTarget> symmetricTarget(const ObservationGrid & g,
 RelocationPoint start,double radius,const std::deque<RelocationPoint> & history,
 const ExplorationBoundaryPolygon & boundary={},double margin=0) {
 auto b=knownBounds(g);if(!b)return {};
 auto center=b->center(),mirror=b->reflect(start);
 auto reachable=mappedReachable(g,start,radius,
   std::chrono::steady_clock::now()+std::chrono::milliseconds(300));
 if(reachable.empty())return {};
 bool near_center=std::hypot(mirror.first-start.first,mirror.second-start.second)<2.;
 double projection_radius=std::clamp(std::hypot(b->max_x-b->min_x,b->max_y-b->min_y)*.25,2.,4.);
 std::optional<RelocationPoint> best;double score=std::numeric_limits<double>::infinity();
 for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x) {
   int i=y*g.width+x;if(!reachable[i] || g.cells[i]!=0)continue;
   auto p=gridWorld(g,x,y);double travel=std::hypot(p.first-start.first,p.second-start.second);
   double mirror_distance=std::hypot(p.first-mirror.first,p.second-mirror.second);
   if(travel<2. || (!near_center && mirror_distance>projection_radius) ||
      !is_inside_exploration_boundary(p,boundary,margin))continue;
   if(std::any_of(history.begin(),history.end(),[&](auto old){
       return std::hypot(p.first-old.first,p.second-old.second)<1.5;}))continue;
   // Symmetry is the primary objective. Visible frontier adjacency breaks ties;
   // it cannot pull the destination back into the original neighbourhood.
   unsigned unknown=0;
   int probe=std::ceil(1.0/g.resolution);
   for(int dy=-probe;dy<=probe;dy+=std::max(1,probe/3))
    for(int dx=-probe;dx<=probe;dx+=std::max(1,probe/3)) {
     int nx=x+dx,ny=y+dy;
     if(nx>=0 && nx<g.width && ny>=0 && ny<g.height && g.cells[ny*g.width+nx]<0)++unknown;
    }
   double value=(near_center?-travel:mirror_distance)-.002*std::min(unknown,20u);
   if(value<score) {score=value;best=p;}
 }
 if(!best)return {};
 return SymmetricTarget{center,mirror,*best};
}
// History never expires on a timer. Genuine newly observed world cells may
// reopen the two-trip budget, while old origins remain excluded from ping-pong.
class SymmetricRelocation {
public:
 std::optional<SymmetricTarget> target;
 std::deque<RelocationPoint> history,stages;
 unsigned attempts{0},failures{0};
 double planning_seconds{0},last_time{-1},retry_at{0};
 bool observe(const ObservationGrid & g,double floor,double now) {
   if(last_time<0 || now<last_time || std::abs(floor-floor_)>.5)clear();
   floor_=floor;last_time=now;
   for(int y=0;y<g.height;++y)for(int x=0;x<g.width;++x)if(g.cells[y*g.width+x]>=0) {
     auto p=gridWorld(g,x,y);
     if(known_.insert({std::llround(p.first/.05),std::llround(p.second/.05)}).second)
       new_area_+=g.resolution*g.resolution;
   }
   if(!initialized_) {initialized_=true;new_area_=0;return false;}
   if(new_area_<.5)return false;
   attempts=0;new_area_=0;return true;
 }
 bool begin(const ObservationGrid & g,RelocationPoint start,double now,
   const ExplorationBoundaryPolygon & boundary,double margin) {
   if(target || attempts>=2 || now<retry_at)return false;
   ++attempts;retry_at=now+60;
   auto selected=symmetricTarget(g,start,.22,history,boundary,margin);
   if(!selected)return false;
   target=selected;remember(start);stages.clear();stages.push_back(start);
   failures=0;planning_seconds=0;sample_=now;return true;
 }
 bool active() const {return target.has_value();}
 void pause(double now) {sample_=now;}
 bool planningExpired(double now) {
   double dt=now-sample_;sample_=now;
   if(dt>=0 && dt<=3.+1e-6)planning_seconds+=dt;
   return target && failures>=3 && planning_seconds>=30.;
 }
 void rejected() {++failures;}
 bool arrived(RelocationPoint p) {
   if(!target)return false;
   return std::hypot(p.first-target->destination.first,p.second-target->destination.second)<=.4;
 }
 bool stage(RelocationPoint p) {
   if(stages.size()>=64)return false;
   if(std::any_of(stages.begin(),stages.end(),[&](auto old) {
     return std::hypot(p.first-old.first,p.second-old.second)<.2;}))return false;
   stages.push_back(p);
   planning_seconds=0;failures=0;return true;
 }
 void finish(double now) {
   if(target)remember(target->destination);
   target.reset();stages.clear();retry_at=now+60;planning_seconds=0;failures=0;
 }
 void clear() {target.reset();history.clear();stages.clear();known_.clear();
   attempts=failures=0;planning_seconds=0;last_time=-1;retry_at=0;new_area_=0;initialized_=false;sample_=0;}
private:
 std::set<std::pair<int64_t,int64_t>> known_;
 double floor_{0},new_area_{0},sample_{0};bool initialized_{false};
 void remember(RelocationPoint p){history.push_back(p);if(history.size()>16)history.pop_front();}
};
}
