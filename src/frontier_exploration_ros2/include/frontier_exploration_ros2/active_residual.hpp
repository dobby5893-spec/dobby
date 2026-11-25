#pragma once
#include "frontier_exploration_ros2/residual_region_audit.hpp"
#include "frontier_exploration_ros2/region_observation_task.hpp"
#include <set>
#include <deque>
namespace frontier_exploration_ros2 {
inline bool spatialEvidenceCurrent(const AuditMapEvidence & e,const PairedFloorMap & pair,const octomap::OcTree * live) {
 // Candidate execution may tolerate changes to unrelated unknown inventory.
 // Formal handoff still uses mapEvidenceCurrent with the full inventory check.
 if(!mapEvidenceVersionCompatible(e,pair))return false;
 for(const auto & d:e.dependencies)if(!dependencyUnchanged(d,e,pair,live))return false;
 return true;
}
using ResidualPose=go2_surface_3d_planner::SurfacePoint;
enum class SpatialOutcome {Ready,Occluded,OutsideFov,SupportUnverified,MapInsufficient,NoRoute,Timeout,
 NavigationFailed,LocalizationInvalid,ObservedNoGain,ObservedGain,StaleMap};
inline const char * spatialOutcomeName(SpatialOutcome s) {
 switch(s) {
 case SpatialOutcome::Ready:return "ready";case SpatialOutcome::Occluded:return "occluded";
 case SpatialOutcome::OutsideFov:return "outside_fov";case SpatialOutcome::SupportUnverified:return "support_unverified";
 case SpatialOutcome::MapInsufficient:return "map_insufficient";case SpatialOutcome::NoRoute:return "no_route";case SpatialOutcome::Timeout:return "verification_deferred";
 case SpatialOutcome::NavigationFailed:return "navigation_failed";case SpatialOutcome::LocalizationInvalid:return "localization_invalid";
 case SpatialOutcome::ObservedNoGain:return "observed_no_gain";case SpatialOutcome::ObservedGain:return "observed_gain";
 default:return "stale_map";
 }
}
struct SpatialAttempt {
 int cell{-1};ResidualPose pose;SpatialOutcome outcome{SpatialOutcome::Timeout};
 std::vector<int> visible_cells;unsigned retries{0};uint64_t coverage_hash{0};size_t visible_count{0};
 size_t free_voxels{0},occupied_voxels{0};double free_area{0},occupied_area{0},union_area{0},volume{0};
 bool nav_checked{false},arrived{false},fresh_cloud{false};double verified_at{0};
};
struct SpatialRegion {
 ResidualRegion region;std::vector<int> frontier;
 bool transition{false},enumerated{false},verified_enclosed{false};
 int cursor{0},scan_remaining{-1};
 double retry_visited_at{0};double spent{0},cooldown_until{0};uint64_t dependency_hash{0},executions{0},round_progress{0};unsigned rounds{0};
 std::optional<std::pair<double,double>> route_origin;std::set<std::pair<int64_t,int64_t>> evaluated_origins;
 std::map<int,SpatialAttempt> attempts;
 std::vector<std::pair<int,SpatialOutcome>> rejected;unsigned retry_limit{4};
 bool schedulable(double budget)const {
  if(transition || verified_enclosed || spent>=budget)return false;
  if(!enumerated)return true;
  for(const auto & a:attempts)if(a.second.outcome==SpatialOutcome::Ready || (a.second.outcome==SpatialOutcome::Timeout && !a.second.arrived && a.second.retries<retry_limit))return true;
  return false;
 }
 std::string classification(double budget)const {
  if(transition)return "transition";
  if(verified_enclosed)return "verified_enclosed";
  if(spent>=budget)return "verification_deferred";
  if(!enumerated)return "actionable";
  bool nav=false,support=false,map=false,fov=false,timeout=false,route=false,observed=false,hidden=false;
  for(const auto & entry:attempts) {
   switch(entry.second.outcome) {
   case SpatialOutcome::Ready:return "actionable";
   case SpatialOutcome::Timeout:timeout=true;break;
   case SpatialOutcome::SupportUnverified:support=true;break;
   case SpatialOutcome::MapInsufficient:map=true;break;
   case SpatialOutcome::OutsideFov:fov=true;break;
   case SpatialOutcome::NavigationFailed:case SpatialOutcome::LocalizationInvalid:case SpatialOutcome::StaleMap:nav=true;break;
   case SpatialOutcome::NoRoute:route=true;break;
   case SpatialOutcome::ObservedNoGain:case SpatialOutcome::ObservedGain:observed=true;break;
   default:hidden=true;break;
   }
  }
  if(timeout)return "verification_deferred";
  for(const auto & r:rejected) {
   timeout|=r.second==SpatialOutcome::Timeout;route|=r.second==SpatialOutcome::NoRoute;
   support|=r.second==SpatialOutcome::SupportUnverified;
   map|=r.second==SpatialOutcome::MapInsufficient;fov|=r.second==SpatialOutcome::OutsideFov;
   hidden|=r.second==SpatialOutcome::Occluded;
  }
  if(timeout)return "verification_deferred";
  if(nav)return "navigation_failed";
  if(support)return "support_unverified";
  if(map)return "map_insufficient";
  if(fov)return "outside_fov";
  if(observed)return "observed_no_gain";
  if(route)return "no_route";
  return hidden?"occluded":"no_known_access";
 }
};
struct SpatialPlan {
 SpatialRegion region;AuditMapEvidence evidence;uint64_t epoch{0};
 double elapsed{0};bool complete_input{true};std::optional<SpatialAttempt> selected;
};
// Incremental work keeps only compact regional evidence, never historical trees.
class ActiveResidualBook {
public:
 static constexpr size_t history_limit=256;
 struct Visit {uint64_t region;SpatialAttempt attempt;};
 std::deque<Visit> history;
 std::map<uint64_t,SpatialRegion> regions;
 void record(uint64_t region,SpatialAttempt attempt) {
  attempt.visible_cells={};
  history.push_back({region,std::move(attempt)});
  if(history.size()>history_limit)history.pop_front();
 }
 std::vector<SpatialAttempt> recentVisits(double now,double cooldown)const {
  std::vector<SpatialAttempt> visits;
  for(const auto & v:history)if(v.attempt.arrived && now>=v.attempt.verified_at && now-v.attempt.verified_at<cooldown)
   visits.push_back(v.attempt);
  return visits;
 }
 void releaseVisitedCooldown(double now) {
  for(auto & item:regions) {
   auto & r=item.second;
   if(r.retry_visited_at>0 && now>=r.retry_visited_at && r.enumerated) {
    r.retry_visited_at=0;r.enumerated=false;r.scan_remaining=-1;
   }
  }
 }
 uint64_t revision{0},last_region{0};
 void clear(){regions.clear();history.clear();schema_=0;++revision;last_region=0;}
 void sync(const ResidualLayout & layout,const PairedFloorMap & pair,
   const ExplorationBoundaryPolygon & boundary,double margin,double padding) {
  const auto schema=floorSchemaDigest(*pair.floor);if(schema_ && schema_!=schema)clear();schema_=schema;
  std::set<uint64_t> alive;OccupancyGrid2d grid(*pair.floor);
  // Fixed map-frame origin for cache identity. Changing the robot's pose alone
  // does not erase completed spatial observations.
  (void)boundary;(void)margin;
  const std::vector<uint8_t> membership(grid.getSizeX()*grid.getSizeY(),0);
  for(const auto & w:layout.work) {
   auto d=dependency(w.region,grid,padding);auto h=regionalFloorDigest(grid,d,membership);
   mapHashMix(h,regionalOctomapDigest(*pair.tree,d));mapHashMix(h,w.stair);
   alive.insert(w.region.id);auto it=regions.find(w.region.id);
   if(it==regions.end() || it->second.dependency_hash!=h) {
    SpatialRegion r;r.region=w.region;r.frontier=w.frontier;r.transition=w.stair;r.dependency_hash=h;
    // Preserve the next position in the lattice. Revisit a full pass in
    // rotated order, since newly opened space may precede the old cursor.
    if(it!=regions.end())r.cursor=it->second.cursor;
    regions[w.region.id]=std::move(r);++revision;
   }else it->second.frontier=w.frontier;
  }
  for(auto it=regions.begin();it!=regions.end();) {
   if(!alive.count(it->first)){it=regions.erase(it);++revision;}else ++it;
  }
 }
 static MapDependency dependency(const ResidualRegion & r,const OccupancyGrid2d & grid,double padding) {
  MapDependency d;d.region=r.id;d.min_x=d.min_y=INFINITY;d.max_x=d.max_y=-INFINITY;
  for(int i:r.cells){auto p=grid.mapToWorld(i%grid.getSizeX(),i/grid.getSizeX());d.min_x=std::min(d.min_x,p.first);d.max_x=std::max(d.max_x,p.first);d.min_y=std::min(d.min_y,p.second);d.max_y=std::max(d.max_y,p.second);}
  d.min_x-=4.+padding;d.min_y-=4.+padding;d.max_x+=4.+padding;d.max_y+=4.+padding;return d;
 }
 void recoverBudgets(double now,double budget,double cooldown) {
  for(auto & item:regions) {
   auto & r=item.second;if(r.spent<budget || !r.schedulable(std::numeric_limits<double>::infinity()))continue;
   const uint64_t progress=uint64_t(r.cursor)+r.rejected.size()+r.executions;
   // No progress means keep an explicit uncertain hold, not another identical
   // expensive proof. A yielding round resumes only new finite-lattice work.
   if(progress<=r.round_progress)continue;
   if(r.cooldown_until==0){r.cooldown_until=now+cooldown;continue;}
   if(now<r.cooldown_until)continue;
   r.round_progress=progress;r.spent=0;r.cooldown_until=0;++r.rounds;
  }
 }
 void changedOrigin(double x,double y,double budget) {
  for(auto & item:regions) {
   auto & r=item.second;if(!r.route_origin || r.verified_enclosed || r.transition || r.spent>=budget ||
     std::hypot(x-r.route_origin->first,y-r.route_origin->second)<=.35)continue;
   const auto state=r.classification(budget);
   if(state=="actionable")continue;
   const auto origin=std::make_pair(int64_t(std::floor(x/.35)),int64_t(std::floor(y/.35)));
   if(!r.evaluated_origins.insert(origin).second){r.route_origin=std::make_pair(x,y);continue;}
   r.round_progress=0;r.cursor=0;r.scan_remaining=-1;r.enumerated=false;r.rejected.clear();r.route_origin=std::make_pair(x,y);
   for(auto it=r.attempts.begin();it!=r.attempts.end();) {
    if(!it->second.arrived)it=r.attempts.erase(it);else ++it;
   }
  }
 }
 std::optional<uint64_t> next(double budget) {
  auto it=regions.upper_bound(last_region);
  for(size_t n=0;n<regions.size();++n) {
   if(it==regions.end())it=regions.begin();
   auto current=it++;
   if(current->second.schedulable(budget)){last_region=current->first;return current->first;}
  }return {};
 }
 std::optional<SpatialAttempt> best(uint64_t id)const {
  auto it=regions.find(id);if(it==regions.end())return {};std::optional<SpatialAttempt> selected;
  for(const auto & entry:it->second.attempts)if(entry.second.outcome==SpatialOutcome::Ready &&
    (!selected || entry.second.visible_cells.size()>selected->visible_cells.size()))selected=entry.second;
  return selected;
 }
 bool apply(SpatialPlan plan) {
  auto it=regions.find(plan.region.region.id);
  if(it==regions.end() || it->second.dependency_hash!=plan.region.dependency_hash)return false;
  it->second=std::move(plan.region);return true;
 }
 ResidualAudit audit(double budget,std::pair<double,double> start)const {
  ResidualAudit a;a.start=start;
  for(const auto & item:regions) {
   auto r=item.second.region;r.state=item.second.classification(budget);a.regions.push_back(r);a.remaining+=r.area;
   if(r.state=="transition")a.transition+=r.area;
   else if(r.state=="verified_enclosed" || r.state=="no_route" || r.state=="no_known_access")a.deferred+=r.area;
   else a.pending+=r.area;
  }return a;
 }
private:uint64_t schema_{0};
};
struct VoxelGain {
 size_t free_voxels{0},occupied_voxels{0};double free_area{0},occupied_area{0},union_area{0},volume{0};
 bool gained()const{return free_voxels+occupied_voxels>0;}
};
struct UnknownVoxelWindow {
 std::vector<octomap::OcTreeKey> keys;double resolution{0};bool complete{true};
};
bool knownFreeBodyFootprint(const OccupancyGrid2d &,const ResidualPose &,const go2_surface_3d_planner::SurfaceSearchConfig &,
 const ExplorationBoundaryPolygon & boundary={});
UnknownVoxelWindow captureUnknownVoxels(const PairedFloorMap &,const ResidualRegion &,
 double below,double above,std::chrono::steady_clock::time_point deadline,size_t limit);
VoxelGain measureVoxelGain(const UnknownVoxelWindow &,const octomap::OcTree &);
bool observedClosedFence(const PairedFloorMap &,const ResidualRegion &,
 const go2_surface_3d_planner::SurfaceSearchConfig &,const VisibilitySensor &,
 std::chrono::steady_clock::time_point deadline);
SpatialPlan planSpatialRegion(const PairedFloorMap &,SpatialRegion,
 const ResidualPose &,go2_surface_3d_planner::SurfaceSearchConfig,VisibilitySensor,
 const ExplorationBoundaryPolygon &,double margin,double budget,unsigned batch,unsigned retries,uint64_t epoch,
 const std::vector<SpatialAttempt> & recent={},double visit_cooldown=0.);
} // namespace frontier_exploration_ros2
