#pragma once
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include "frontier_exploration_ros2/frontier_types.hpp"
#include <set>
#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>

namespace frontier_exploration_ros2 {
// Exclusion is a planning classification only. Raw map cells never change.
// An unknown component must have an entirely observed occupied 8-neighbour
// boundary, persist across fresh map messages, and never touch map/mission edges.
class FloorCoveragePolicy {
public:
 double idle_timeout{180.},cleanup_budget{180.};
 unsigned minimum_attempts{4};
 void observe(double now,const OccupancyGrid2d & map,
   const ExplorationBoundaryPolygon & boundary,double margin,
   std::optional<std::pair<double,double>> robot=std::nullopt) {
   const double floor=map.map().info.origin.position.z;
   const double resolution=map.map().info.resolution;
   if(!initialized_ || now<clock_ || std::abs(floor-floor_)>.5 ||
      width_!=map.getSizeX() || height_!=map.getSizeY() ||
      std::abs(resolution-resolution_)>1e-8 ||
      std::abs(map.map().info.origin.position.x-origin_x_)>1e-8 ||
      std::abs(map.map().info.origin.position.y-origin_y_)>1e-8) {
     clear();initialized_=true;floor_=floor;last_gain_=now;
     width_=map.getSizeX();height_=map.getSizeY();resolution_=resolution;
     origin_x_=map.map().info.origin.position.x;origin_y_=map.map().info.origin.position.y;
   }
   if(paused_at_>=0) {
     const double paused=std::max(0.,now-paused_at_);
     last_gain_+=paused;if(cleanup_)cleanup_started_+=paused;
     for(auto & item:closed_)item.second.since+=paused;
     paused_at_=-1;residual_sample_=now;
   }
   const double dt=now-residual_sample_;
   if(paused_at_<0 && residual_sample_>=0 && dt>=0 && dt<=3.+1e-6)residual_idle_+=dt;
   residual_sample_=now;
   clock_=now;
   const auto & stamp=map.map().header.stamp;
   const int64_t map_stamp=int64_t(stamp.sec)*1000000000LL+stamp.nanosec;
   last_map_stamp_=map_stamp;
   std::vector<uint8_t> inside(width_*height_,0),seen(width_*height_,0);
   excluded_.assign(width_*height_,0);
   size_t added=0,unknown=0,excluded=0;
   bool opened_space=false;
   if(last_known_class_.size()!=inside.size())last_known_class_.assign(inside.size(),-1);
   const double area=resolution*resolution;
   for(int y=0;y<height_;++y)for(int x=0;x<width_;++x) {
     if(!is_inside_exploration_boundary(map.mapToWorld(x,y),boundary,margin))continue;
     inside[y*width_+x]=1;
     if(map.getCost(x,y)<0){++unknown;continue;}
     const int index=y*width_+x,value=map.getCost(x,y);
     if(last_known_class_[index]>=65 && value<65)opened_space=true;
     last_known_class_[index]=value;
     const auto p=map.mapToWorld(x,y);
     if(known_.insert({std::llround(p.first/.05),std::llround(p.second/.05)}).second)++added;
   }
   int robot_index=-1;
   if(robot) {
     int x,y;
     if(map.worldToMapNoThrow(robot->first,robot->second,x,y))robot_index=y*width_+x;
   }
   std::map<int,Closed> current;
   for(int start=0;start<width_*height_;++start) {
     if(seen[start] || !inside[start] || map.getCost(start%width_,start/width_)>=65)continue;
     std::vector<int> cells{start};seen[start]=1;bool sealed=true,has_free=false;size_t component_unknown=0;
     for(size_t head=0;head<cells.size();++head) {
       const int x=cells[head]%width_,y=cells[head]/width_;
       if(cells[head]==robot_index)sealed=false;
       if(map.getCost(x,y)<0)++component_unknown;else has_free=true;
       for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
         if(dx==0 && dy==0)continue;
         const int nx=x+dx,ny=y+dy;
         if(nx<0 || ny<0 || nx>=width_ || ny>=height_) {sealed=false;continue;}
         const int index=ny*width_+nx,value=map.getCost(nx,ny);
         if(!inside[index]) {sealed=false;continue;}
         if(value<65 && !seen[index]) {seen[index]=1;cells.push_back(index);}
       }
     }
     // Without a robot seed, a component containing free cells may be the
     // robot's own room. Never exclude it on geometry alone.
     if(!sealed || component_unknown==0 || (has_free && robot_index<0))continue;
     std::sort(cells.begin(),cells.end());
     Closed record;record.cells=cells;record.since=now;record.last_stamp=map_stamp;record.samples=1;
     auto old=closed_.find(start);
     if(old!=closed_.end() && old->second.cells==cells) {
       record=old->second;
       if(map_stamp>record.last_stamp) {++record.samples;record.last_stamp=map_stamp;}
     }
     // No area/diameter cutoff. A large closed room is excluded by topology too.
     if(record.samples>=3 && now-record.since>=10.) {
       for(int index:cells)if(map.getCost(index%width_,index/width_)<0)excluded_[index]=1;
       excluded+=component_unknown;
     }
     current.emplace(start,std::move(record));
   }
   closed_=std::move(current);
   remaining_unknown_area=unknown*area;
   excluded_unknown_area=excluded*area;
   actionable_unknown_area=(unknown-excluded)*area;
   actionable_cells_=unknown-excluded;
   if(first_map_) {first_map_=false;return;}
   if(opened_space)resetResidualEvidence();
   pending_gain_+=added*area;
   if(pending_gain_>=.05-1e-9) {
     last_gain_=now;if(cleanup_)cleanup_started_=now;
     if(pending_gain_>=.5-1e-9) {cleanup_=false;attempts_=0;expansion_=0;}
     pending_gain_=0;resetResidualEvidence();
   }
 }
 bool shouldCleanup(double now) const
 {return initialized_ && paused_at_<0 && !cleanup_ && now-last_gain_>=idle_timeout-1e-9;}
 void waitForData(double now) {if(initialized_ && paused_at_<0)paused_at_=now;}
 void enterCleanup(double now) {
   if(cleanup_)return;
   cleanup_=true;cleanup_started_=now;attempts_=0;last_attempt_=-1e9;
   expansion_=1;last_expansion_=now;
 }
 bool expand(double now) {
   if(!cleanup_) {enterCleanup(now);return true;}
   if(expansion_<3 && now-last_expansion_>=20.-1e-9) {
     ++expansion_;last_expansion_=now;return true;
   }
   return false;
 }
 void attempt(double now) {
   if(cleanup_ && now-last_attempt_>=5.-1e-9) {++attempts_;last_attempt_=now;}
 }
 // Diagnostic search evidence never changes an unknown cell classification.
 void pauseResidual(double now) {residual_sample_=now;}
 void validationEvidence(bool complete,unsigned options,unsigned available=0) {
   // A fully evaluated singleton is as complete as a large fan. Empty,
   // partial and timed-out sets are not exhaustion evidence.
   if(!complete || available==0 || options==0 || options!=available)return;
   ++failed_groups_;failed_options_+=options;
 }
 void observationEvidence(std::pair<double,double> point,bool gained) {
   if(gained) {resetResidualEvidence();return;}
   if(std::all_of(no_gain_positions_.begin(),no_gain_positions_.end(),[&](auto p) {
       return std::hypot(p.first-point.first,p.second-point.second)>=.5;}))
     no_gain_positions_.push_back(point);
 }
 void exhaustedSearch(double now,bool raw_empty,bool pending) {
   if(!initialized_ || paused_at_>=0 || expansion_<3 || pending ||
      now-last_residual_round_<10.-1e-6)return;
   const auto stamp=last_map_stamp_;
   if(stamp<=residual_round_stamp_)return;
   last_residual_round_=now;residual_round_stamp_=stamp;++residual_rounds_;
   if(raw_empty)++raw_empty_rounds_;
 }
 // Search failures and no-gain samples describe attempted viewpoints, not
 // every remaining unknown component. They cannot certify inaccessible space.
 // Keep these counters for diagnostics/retry scheduling only.
 bool residualDeferred() const {return false;}
 unsigned residualRounds() const {return residual_rounds_;}
 unsigned failedGroups() const {return failed_groups_;}
 unsigned failedOptions() const {return failed_options_;}
 unsigned emptyRounds() const {return raw_empty_rounds_;}
 unsigned observationPositions() const {return no_gain_positions_.size();}
 double residualIdleSeconds() const {return residual_idle_;}
 bool completionCandidate(double) const {
   return initialized_ && paused_at_<0 &&
     actionable_cells_==0;
 }
 bool hasActionableUnknown() const {return actionable_cells_>0;}
 bool excludedCell(int x,int y) const {
   return x>=0 && y>=0 && x<width_ && y<height_ &&
     !excluded_.empty() && excluded_[y*width_+x];
 }
 // Conservative exclusion: never discard a reference if any nearby raw
 // unknown remains actionable. Classification is recomputed from each map.
 bool closedReference(const OccupancyGrid2d & map,std::pair<double,double> reference) const {
   if(width_!=map.getSizeX() || height_!=map.getSizeY() || excluded_.empty())return false;
   int cx,cy;
   if(!map.worldToMapNoThrow(reference.first,reference.second,cx,cy))return false;
   const int radius=std::ceil(.65/map.map().info.resolution);
   bool any=false;
   for(int y=std::max(0,cy-radius);y<std::min(height_,cy+radius+1);++y)
     for(int x=std::max(0,cx-radius);x<std::min(width_,cx+radius+1);++x) {
       const auto point=map.mapToWorld(x,y);
       if(std::hypot(point.first-reference.first,point.second-reference.second)>.65 ||
          map.getCost(x,y)>=0)continue;
       any=true;if(!excludedCell(x,y))return false;
     }
   return any;
 }
 bool cleanup() const {return cleanup_;}
 unsigned expansion() const {return expansion_;}
 unsigned attempts() const {return attempts_;}
 void clear() {
   initialized_=false;first_map_=true;cleanup_=false;
   known_.clear();closed_.clear();excluded_.clear();last_known_class_.clear();pending_gain_=0;attempts_=0;
   remaining_unknown_area=excluded_unknown_area=actionable_unknown_area=0;
   actionable_cells_=0;paused_at_=-1;expansion_=0;width_=height_=0;
   resetResidualEvidence();residual_sample_=-1;last_map_stamp_=0;
 }
 double remaining_unknown_area{0},excluded_unknown_area{0},actionable_unknown_area{0};
private:
 void resetResidualEvidence() {
   residual_idle_=0;residual_rounds_=raw_empty_rounds_=failed_groups_=failed_options_=0;
   no_gain_positions_.clear();last_residual_round_=-1e9;residual_round_stamp_=-1;
 }
 double residual_sample_{-1},residual_idle_{0},last_residual_round_{-1e9};
 unsigned residual_rounds_{0},raw_empty_rounds_{0},failed_groups_{0},failed_options_{0};
 int64_t residual_round_stamp_{-1},last_map_stamp_{0};
 std::vector<std::pair<double,double>> no_gain_positions_;
 std::vector<int> last_known_class_;
 struct Closed {std::vector<int> cells;double since{0};int64_t last_stamp{0};unsigned samples{0};};
 bool initialized_{false},first_map_{true},cleanup_{false};
 int width_{0},height_{0};
 double resolution_{0},origin_x_{0},origin_y_{0};
 double floor_{0},clock_{0},last_gain_{0},pending_gain_{0},cleanup_started_{0},last_attempt_{-1e9};
 double paused_at_{-1},last_expansion_{0};
 unsigned attempts_{0},expansion_{0};size_t actionable_cells_{0};
 std::set<std::pair<int64_t,int64_t>> known_;
 std::map<int,Closed> closed_;
 std::vector<uint8_t> excluded_;
};
}
