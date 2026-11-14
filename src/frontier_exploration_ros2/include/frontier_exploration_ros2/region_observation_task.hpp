#pragma once
#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include <set>
#include <map>
#include <vector>
#include <optional>
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>

namespace frontier_exploration_ros2 {
// A local spatial task, not a semantic room classifier. Occupied cells stop
// association. Original unknown cells survive frontier splitting/centroid drift.
class RegionObservationTask {
public:
 using Point=std::pair<double,double>;
 using Key=std::pair<int64_t,int64_t>;
 enum class Decision {Waiting,Gain,Advance,ChangeView,Deferred};
 double no_gain_wait_s{5.},no_gain_budget_s{60.},cooldown_s{60.};
 unsigned minimum_positions{3};
 ExplorationBoundaryPolygon boundary;double boundary_margin{0};
 bool active() const {return task_.has_value();}
 Point reference() const {return task_?task_->reference:Point{};}
 double idleSeconds() const {return task_?task_->idle_seconds:last_idle_;}
 size_t triedPositions() const {return task_?task_->tried.size():last_positions_;}
 bool observing() const {return window_;}
 bool start(const OccupancyGrid2d & map,Point reference,double now) {
   sync(map,now);
   if(deferred(map,reference,now))return false;
   auto cells=component(map,reference,reference);
   if(cells.empty())return false;
   if(task_) {
     if(!associated(map,cells,task_->cells))return false;
     task_->cells.insert(cells.begin(),cells.end());
     for(auto k:cells)if(!known_.count(k))task_->unrevealed.insert(k);
     task_->reference=reference;
     return true;
   }
   Task t;t.anchor=t.reference=reference;t.cells=std::move(cells);
   for(const auto & previous:history_)if(associated(map,t.cells,previous.task.cells)) {
     t=previous.task;t.reference=reference;t.cells.insert(cells.begin(),cells.end());
     if(t.planning_seconds>=60.){t.planning_seconds=0;t.planning_failures=0;}
     break;
   }
   for(auto k:t.cells)if(!known_.count(k))t.unrevealed.insert(k);
   task_=std::move(t);planning_sample_=now;planning_stamp_=stamp(map);last_empty_round_=now-3.;return true;
 }
 bool matches(const OccupancyGrid2d & map,Point reference) const {
   return task_ && associated(map,component(map,reference,reference),task_->cells);
 }
 void extend(const OccupancyGrid2d & map,Point reference) {
   if(!matches(map,reference))return;
   auto cells=component(map,reference,reference);
   task_->cells.insert(cells.begin(),cells.end());
   for(auto k:cells)if(!known_.count(k))task_->unrevealed.insert(k);
   task_->reference=reference;
 }
 bool deferred(const OccupancyGrid2d & map,Point reference,double now) const {
   const auto current=component(map,reference,reference);
   for(const auto & item:archive_)if(!current.empty() && std::all_of(current.begin(),current.end(),
     [&](auto k){return item.task.cells.count(k)>0;}))return true;
   for(const auto & item:deferred_)
     if(item.second>now && associated(map,component(map,reference,reference),item.first.cells))return true;
   return false;
 }
 bool update(const OccupancyGrid2d & map,double now) {
   sync(map,now);
   if(!task_)return false;
   bool gained=false;
   for(auto it=task_->unrevealed.begin();it!=task_->unrevealed.end();) {
     int x,y;
     if(map.worldToMapNoThrow(it->first*.05,it->second*.05,x,y) && map.getCost(x,y)>=0) {
       known_.insert(*it);it=task_->unrevealed.erase(it);gained=true;
     }else ++it;
   }
   if(gained)resetFailureBudget();
   return gained;
 }
 bool finished(const OccupancyGrid2d & map) const {
   if(!task_)return false;
   for(auto k:task_->cells) {
     int x,y;
     if(!map.worldToMapNoThrow(k.first*.05,k.second*.05,x,y) || map.getCost(x,y)<0)return false;
   }
   return true;
 }
 template<typename Excluded>
 bool remainingAreClosed(const OccupancyGrid2d & map,Excluded excluded) const {
   if(!task_)return false;
   bool any=false;
   for(auto k:task_->cells) {
     int x,y;
     if(!map.worldToMapNoThrow(k.first*.05,k.second*.05,x,y))return false;
     if(map.getCost(x,y)<0) {any=true;if(!excluded(x,y))return false;}
   }
   return any;
 }
 std::vector<Point> remainingReferences(const OccupancyGrid2d & map,Point robot) const {
   std::vector<Point> points;
   if(!task_)return points;
   for(auto k:task_->cells) {
     int x,y;
     if(map.worldToMapNoThrow(k.first*.05,k.second*.05,x,y) && map.getCost(x,y)<0)
       points.push_back({k.first*.05,k.second*.05});
   }
   std::sort(points.begin(),points.end(),[&](auto a,auto b){return distance(a,robot)<distance(b,robot);});
   std::vector<Point> result;
   for(auto p:points) {
     if(std::all_of(result.begin(),result.end(),[&](auto q){return distance(p,q)>=.7;}))result.push_back(p);
     if(result.size()>=6)break;
   }
   return result;
 }
 void arrived(const OccupancyGrid2d & map,Point position,double now,std::optional<double> yaw=std::nullopt) {
   if(!task_)return;
   window_=true;window_seconds_=0;last_sample_=now;window_position_=position;
   last_stamp_=stamp(map);window_reference_=task_->reference;window_yaw_=yaw;
 }
 // This clock is independent of reached-view observation time. It only runs
 // on fresh scheduler opportunities while no navigation is in progress.
 void pause(double now) {if(window_)last_sample_=now;planning_sample_=now;}
 void planningFailure(std::optional<Point> origin=std::nullopt) {
   if(!task_)return;
   ++task_->planning_failures;
   if(origin && std::all_of(task_->failed_origins.begin(),task_->failed_origins.end(),
      [&](auto p){return distance(p,*origin)>=.5;}))task_->failed_origins.push_back(*origin);
 }
 void noCandidate(double now) {
   if(task_ && now-last_empty_round_>=3.) {planningFailure();last_empty_round_=now;}
 }
 unsigned planningFailures() const {return task_?task_->planning_failures:last_failures_;}
 double planningSeconds() const {return task_?task_->planning_seconds:last_planning_;}
 bool planningWait(const OccupancyGrid2d & map,double now) {
   if(!task_ || window_) {planning_sample_=now;return false;}
   if(stamp(map)<=planning_stamp_)return false;
   const double dt=now-planning_sample_;planning_sample_=now;
   planning_stamp_=stamp(map);
   if(dt>=0 && dt<=3.+1e-6)task_->planning_seconds+=dt;
   if(task_->planning_failures<3 || task_->planning_seconds<60.-1e-6)return false;
   last_failures_=task_->planning_failures;last_planning_=task_->planning_seconds;
   // Execution-budget exhaustion is reversible evidence, never a geometric
   // proof of no route. Require retries from different measured robot positions.
   if(task_->planning_seconds>=60. && task_->failed_origins.size()>=3)archiveTask(2);
   deferred_.push_back({*task_,now+cooldown_s});
   if(deferred_.size()>32)deferred_.erase(deferred_.begin());
   release();return true;
 }
 Decision poll(const OccupancyGrid2d & map,Point position,double now,
   bool any_new_information,bool route_progress) {
   sync(map,now);
   if(!task_ || !window_)return Decision::Waiting;
   if(distance(position,window_position_)>.15) {
     // Position drift must settle again; it is not a failed observation.
     window_position_=position;window_seconds_=0;last_sample_=now;last_stamp_=stamp(map);
     return Decision::Waiting;
   }
   if(stamp(map)<=last_stamp_)return Decision::Waiting;
   const double dt=now-last_sample_;last_sample_=now;last_stamp_=stamp(map);
   if(any_new_information) {
     resetFailureBudget();window_=false;return Decision::Gain;
   }
   // A stale-data gap cannot be retrospectively charged to the observation.
   if(dt>=0 && dt<=3.0+1e-6) {
     const double counted=std::min(dt,std::max(0.,no_gain_wait_s-window_seconds_));
     window_seconds_+=counted;task_->idle_seconds+=counted;
   }
   if(window_seconds_<no_gain_wait_s-1e-6)return Decision::Waiting;
   window_=false;
   if(window_yaw_) {
     task_->views.push_back({window_reference_,position,*window_yaw_});
     if(task_->views.size()>128)task_->views.erase(task_->views.begin());
   }
   if(std::all_of(task_->tried.begin(),task_->tried.end(),
     [&](auto p){return distance(p,position)>.35;}))task_->tried.push_back(position);
   rememberTask();
   if(!route_progress && task_->tried.size()>=minimum_positions && task_->idle_seconds>=no_gain_budget_s-1e-6) {
     last_idle_=task_->idle_seconds;last_positions_=task_->tried.size();
     deferred_.push_back({*task_,now+cooldown_s});
     if(deferred_.size()>32)deferred_.erase(deferred_.begin());
     if(task_->tried.size()>=3 && task_->idle_seconds>=60.-1e-6)archiveTask(1);
     release();return Decision::Deferred;
   }
   return route_progress?Decision::Advance:Decision::ChangeView;
 }
 // Only unknown cells in an evidenced footprint may be deferred. Adjacent
 // newly exposed unknowns remain independent tasks.
 std::vector<int8_t> evidenceMask(const OccupancyGrid2d & map,double now) {
   sync(map,now,false);std::vector<int8_t> mask(map.getSizeX()*map.getSizeY(),0);
   for(const auto & record:archive_)for(auto k:record.task.cells) {
     int x,y;if(map.worldToMapNoThrow(k.first*.05,k.second*.05,x,y) && map.getCost(x,y)<0)
       mask[y*map.getSizeX()+x]=record.kind;
   }
   return mask;
 }
 void release() {rememberTask();task_.reset();window_=false;}
 void clear() {release();archive_.clear();history_.clear();latest_.reset();deferred_.clear();known_.clear();floor_.reset();clock_=-1;last_idle_=0;last_positions_=0;planning_sample_=0;planning_stamp_=0;last_empty_round_=-1e9;last_planning_=0;last_failures_=0;}
 // Reuse a completed, fresh, no-gain observation only for the same
 // target cell, observation cell and heading. Other corners/views stay eligible.
 bool alreadyObserved(const OccupancyGrid2d & map,Point reference,Point position,double yaw) const {
   auto sameCell=[&](Point a,Point b) {
     int ax,ay,bx,by;
     return map.worldToMapNoThrow(a.first,a.second,ax,ay) &&
       map.worldToMapNoThrow(b.first,b.second,bx,by) && ax==bx && ay==by;
   };
   for(const auto * records:{&history_,&archive_})for(const auto & r:*records) {
     bool matched=false;
     for(const auto & v:r.task.views)
       if(sameCell(reference,v.reference) && sameCell(position,v.position) &&
          std::abs(std::remainder(yaw-v.yaw,2*M_PI))<=.20){matched=true;break;}
     if(matched && unchanged(r,map))return true;
   }
   return false;
 }
private:
 struct ObservedView {Point reference,position;double yaw;};
 struct Task {
   Point anchor,reference;
   std::set<Key> cells,unrevealed;
   std::vector<Point> tried,failed_origins;
   std::vector<ObservedView> views;
   double idle_seconds{0},planning_seconds{0};
   unsigned planning_failures{0};
 };
 struct Record {Task task;int8_t kind;std::map<Key,int> signature;};
 std::optional<OccupancyGrid2d> latest_;
 std::vector<Record> archive_,history_;
 static int cellClass(int value){return value<0?-1:value>=65?100:0;}
 std::map<Key,int> signature(const Task & task,const OccupancyGrid2d & map) const {
   std::map<Key,int> result;const double r=map.map().info.resolution;
   for(auto k:task.cells)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
     auto point=Point{k.first*.05+dx*r,k.second*.05+dy*r};
     int x,y;if(map.worldToMapNoThrow(point.first,point.second,x,y))
       result[key(point)]=cellClass(map.getCost(x,y));
   }
   return result;
 }
 bool unchanged(const Record & record,const OccupancyGrid2d & map) const {
   for(auto item:record.signature) {
     int x,y;if(!map.worldToMapNoThrow(item.first.first*.05,item.first.second*.05,x,y) ||
       cellClass(map.getCost(x,y))!=item.second)return false;
   }
   return true;
 }
 void rememberTask() {
   if(!task_ || !latest_)return;
   history_.erase(std::remove_if(history_.begin(),history_.end(),
     [&](const auto & r){return overlaps(r.task.cells,task_->cells);}),history_.end());
   history_.push_back({*task_,0,signature(*task_,*latest_)});
   if(history_.size()>32)history_.erase(history_.begin());
 }
 void archiveTask(int8_t kind) {
   if(!task_ || !latest_)return;
   archive_.erase(std::remove_if(archive_.begin(),archive_.end(),
     [&](const auto & r){return overlaps(r.task.cells,task_->cells);}),archive_.end());
   archive_.push_back({*task_,kind,signature(*task_,*latest_)});
   if(archive_.size()>32)archive_.erase(archive_.begin());
 }
 std::optional<Task> task_;
 std::vector<std::pair<Task,double>> deferred_;
 std::set<Key> known_;
 std::optional<double> floor_;
 double clock_{-1},last_sample_{0},window_seconds_{0},last_idle_{0};
 size_t last_positions_{0};
 bool window_{false};int64_t last_stamp_{0};Point window_position_{},window_reference_{};std::optional<double> window_yaw_;
 double planning_sample_{0},last_empty_round_{-1e9},last_planning_{0};
 int64_t planning_stamp_{0};unsigned last_failures_{0};
 static Key key(Point p){return {std::llround(p.first/.05),std::llround(p.second/.05)};}
 static double distance(Point a,Point b){return std::hypot(a.first-b.first,a.second-b.second);}
 static int64_t stamp(const OccupancyGrid2d & m) {
   return int64_t(m.map().header.stamp.sec)*1000000000LL+m.map().header.stamp.nanosec;
 }
 void resetFailureBudget() {
   if(task_) {task_->idle_seconds=0;task_->tried.clear();task_->views.clear();task_->planning_seconds=0;task_->planning_failures=0;task_->failed_origins.clear();}
 }
 void sync(const OccupancyGrid2d & map,double now,bool update_known=true) {
   const double z=map.map().info.origin.position.z;
   if((floor_ && std::abs(*floor_-z)>.5) || (clock_>=0 && now<clock_))clear();
   floor_=z;clock_=now;
   for(auto * records:{&archive_,&history_})
     records->erase(std::remove_if(records->begin(),records->end(),
       [&](const auto & r){
         if(unchanged(r,map))return false;
         deferred_.erase(std::remove_if(deferred_.begin(),deferred_.end(),
           [&](const auto & item){return overlaps(item.first.cells,r.task.cells);}),deferred_.end());
         if(task_ && overlaps(task_->cells,r.task.cells))resetFailureBudget();
         return true;
       }),records->end());
   latest_=map;
   deferred_.erase(std::remove_if(deferred_.begin(),deferred_.end(),
     [&](const auto & item){return item.second<=now;}),deferred_.end());
   if(update_known)for(int y=0;y<map.getSizeY();++y)for(int x=0;x<map.getSizeX();++x)
     if(map.getCost(x,y)>=0)known_.insert(key(map.mapToWorld(x,y)));
 }
 static bool associated(const OccupancyGrid2d & map,const std::set<Key> & current,
   const std::set<Key> & previous) {
   if(overlaps(current,previous))return true;
   // Newly exposed unknowns may touch the former footprint after every old
   // unknown cell became free. Require a cardinal free neighbour, not a jump
   // across an observed wall or a centroid-distance heuristic.
   const double resolution=map.map().info.resolution;
   for(auto k:current)for(auto d:{Point{-resolution,0},{resolution,0},{0,-resolution},{0,resolution}}) {
     Point neighbour{k.first*.05+d.first,k.second*.05+d.second};
     if(!previous.count(key(neighbour)))continue;
     int x,y;
     if(map.worldToMapNoThrow(neighbour.first,neighbour.second,x,y) &&
       map.getCost(x,y)==0)return true;
   }
   return false;
 }
 static bool overlaps(const std::set<Key> & a,const std::set<Key> & b) {
   for(auto k:a)if(b.count(k))return true;
   return false;
 }
 std::set<Key> component(const OccupancyGrid2d & map,Point ref,Point anchor) const {
   const int w=map.getSizeX(),h=map.getSizeY();
   if(w<=0 || h<=0 || size_t(w)*h>200000)return {};
   int cx,cy;if(!map.worldToMapNoThrow(ref.first,ref.second,cx,cy))return {};
   int seed=-1;double best=.65;
   int radius=std::ceil(.65/map.map().info.resolution);
   for(int y=std::max(0,cy-radius);y<std::min(h,cy+radius+1);++y)
    for(int x=std::max(0,cx-radius);x<std::min(w,cx+radius+1);++x) {
     auto p=map.mapToWorld(x,y);double d=distance(ref,p);
     if(map.getCost(x,y)<0 && d<best && distance(p,anchor)<=3. &&
       is_inside_exploration_boundary(p,boundary,boundary_margin)) {best=d;seed=y*w+x;}
   }
   if(seed<0)return {};
   std::queue<int> q;std::vector<bool> seen(w*h,false);q.push(seed);seen[seed]=true;
   std::set<Key> result;
   while(!q.empty()) {
     int i=q.front();q.pop();auto p=map.mapToWorld(i%w,i/w);
     if(map.getCost(i%w,i/w)>=0 || distance(p,anchor)>3. ||
       !is_inside_exploration_boundary(p,boundary,boundary_margin))continue;
     result.insert(key(p));
     for(auto d:{std::pair<int,int>{-1,0},{1,0},{0,-1},{0,1}}) {
       int x=i%w+d.first,y=i/w+d.second;
       if(x>=0 && y>=0 && x<w && y<h && !seen[y*w+x]) {seen[y*w+x]=true;q.push(y*w+x);}
     }
   }
   return result;
 }
};
}
