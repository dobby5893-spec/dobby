#pragma once
#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include <functional>
#include "frontier_exploration_ros2/floor_completion.hpp"
#include <queue>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <map>
#include <tuple>
#include "frontier_exploration_ros2/viewpoint_visibility_3d.hpp"
namespace frontier_exploration_ros2 {
struct ResidualRegion {
 uint64_t id{0};double area{0};std::string state;
};
struct ResidualAudit {
 uint64_t fingerprint{0},epoch{0};bool complete{true};
 std::pair<double,double> start{0,0};
 double remaining{0},closed{0},deferred{0},pending{0},transition{0};
 uint64_t transition_hash{0},runtime_hash{0};
 double verification_deferred{0};
 size_t sites{0};std::vector<ResidualRegion> regions;
 std::optional<std::pair<double,double>> reference,view;
 CompletionStatus status() const {
  if(!std::isfinite(remaining) || remaining<0 || closed!=0 ||
     std::abs(remaining-deferred-pending-transition)>1e-4)return CompletionStatus::ERROR;
  if(!complete || pending>0)return CompletionStatus::INCOMPLETE;
  double total=0;bool blocked=false;
  for(const auto & region:regions) {
   if(!std::isfinite(region.area) || region.area<=0)return CompletionStatus::ERROR;
   total+=region.area;
   if(region.state=="verification_deferred" || region.state=="observed_no_gain" ||
      region.state=="undetermined" || region.state=="actionable")return CompletionStatus::INCOMPLETE;
   if(region.state=="no_route" || region.state=="no_known_access")blocked=true;
   else if(region.state!="occluded" && region.state!="transition")return CompletionStatus::ERROR;
  }
  if(std::abs(total-remaining)>1e-4)return CompletionStatus::ERROR;
  if(blocked)return CompletionStatus::BLOCKED;
  return remaining==0?CompletionStatus::VERIFIED_COMPLETE:CompletionStatus::VERIFIED_WITH_RESIDUALS;
 }
 bool handoff() const {
  auto s=status();return s==CompletionStatus::VERIFIED_COMPLETE || s==CompletionStatus::VERIFIED_WITH_RESIDUALS;
 }
 std::string records() const {
  std::ostringstream s;s<<std::setprecision(10);
  for(size_t i=0;i<regions.size();++i) {
   if(i) {s<<",";}
   s<<regions[i].id<<":"<<regions[i].area<<":"<<regions[i].state;
  }return s.str();
 }
};
inline uint64_t residualMapFingerprint(const OccupancyGrid2d & m) {
 uint64_t hash=1469598103934665603ULL;
 auto mix=[&](uint64_t v){hash=(hash^v)*1099511628211ULL;};
 mix(m.getSizeX());mix(m.getSizeY());
 mix(std::llround(m.map().info.resolution*1e6));
 mix(std::llround(m.map().info.origin.position.x*1e4));
 mix(std::llround(m.map().info.origin.position.y*1e4));
 mix(std::llround(m.map().info.origin.position.z*1e4));
 for(auto v:m.map().data)mix(v<0?0:v>=65?2:1);
 return hash;
}
// Audit is exhaustive over the currently known-free floor-grid observation
// lattice and its connected boundary cells. It is a current-capability report,
// never proof that raw unknowns are occupied or permanently unreachable.
// can_observe: 1=admissible viewpoint exists, 0=geometrically rejected,
// -2=complete route search rejected, -1=uncertain/budget exceeded.
// Incomplete searches must never certify a region as inaccessible.
inline ResidualAudit auditResidualRegions(
 const OccupancyGrid2d & map,std::pair<double,double> robot,
 const ExplorationBoundaryPolygon & boundary,double margin,
 const std::function<bool(int,int)> & excluded,
 const std::function<int(std::pair<double,double>,std::pair<double,double>)> & can_observe,
 std::chrono::steady_clock::time_point deadline,
 const std::function<bool(int,int)> & transition_cell={},
 const std::function<int(int,int)> & runtime_cell={}) {
 ResidualAudit out;out.fingerprint=residualMapFingerprint(map);
 const int w=map.getSizeX(),h=map.getSizeY();const double area=map.map().info.resolution*map.map().info.resolution;
 auto valid=[&](int x,int y){return x>=0 && y>=0 && x<w && y<h;};
 auto timed=[&](){return std::chrono::steady_clock::now()>=deadline;};
 std::vector<uint8_t> reach(w*h,0),seen(w*h,0);
 int sx,sy;
 if(!map.worldToMapNoThrow(robot.first,robot.second,sx,sy) || map.getCost(sx,sy)!=0 ||
    !is_inside_exploration_boundary(robot,boundary,margin)) {out.complete=false;return out;}
 std::vector<int> queue{sy*w+sx};reach[sy*w+sx]=1;
 const std::pair<int,int> cardinal[]={{1,0},{-1,0},{0,1},{0,-1}};
 for(size_t head=0;head<queue.size();++head) {
  if(timed()){out.complete=false;return out;}
  int x=queue[head]%w,y=queue[head]/w;
  for(auto d:cardinal) {
   int nx=x+d.first,ny=y+d.second;
   if(valid(nx,ny) && !reach[ny*w+nx] && map.getCost(nx,ny)==0 &&
      boundary_return_step(map.mapToWorld(x,y),map.mapToWorld(nx,ny),boundary,0)) {
    reach[ny*w+nx]=1;queue.push_back(ny*w+nx);
   }
  }
 }
 struct Work {ResidualRegion region;std::vector<int> frontier;bool stair;int proof;};
 std::vector<Work> work;
 for(int start=0;start<w*h;++start) {
  if(timed()){out.complete=false;return out;}
  int x=start%w,y=start/w;
  if(map.getCost(x,y)>=0 || !is_inside_exploration_boundary(map.mapToWorld(x,y),boundary,margin))continue;
  out.remaining+=area;
  (void)excluded; // A 2D exclusion is a candidate classification, never 3D proof.
  if(seen[start])continue;
  const bool stair=transition_cell && transition_cell(x,y);
  const int proof=stair?0:(runtime_cell?runtime_cell(x,y):0);
  std::vector<int> component{start};seen[start]=1;
  for(size_t head=0;head<component.size();++head) {
   int cx=component[head]%w,cy=component[head]/w;
   for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
    int nx=cx+dx,ny=cy+dy;
    if(valid(nx,ny) && !seen[ny*w+nx] && map.getCost(nx,ny)<0 &&
       bool(transition_cell && transition_cell(nx,ny))==stair &&
       (stair?0:(runtime_cell?runtime_cell(nx,ny):0))==proof &&
       is_inside_exploration_boundary(map.mapToWorld(nx,ny),boundary,margin)) {
     seen[ny*w+nx]=1;component.push_back(ny*w+nx);
    }
   }
  }
  std::sort(component.begin(),component.end());
  ResidualRegion region;region.id=1469598103934665603ULL;region.area=component.size()*area;
  std::vector<int> frontier;
  for(int i:component) {
   region.id=(region.id^uint64_t(i))*1099511628211ULL;
   for(auto d:cardinal)if(valid(i%w+d.first,i/w+d.second) && reach[(i/w+d.second)*w+i%w+d.first]) {
    frontier.push_back(i);break;
   }
  }
  if(frontier.empty())frontier=component;
  work.push_back({region,std::move(frontier),stair,proof});
 }
 // Inspect large unexplored regions before spending the budget on tiny slivers.
 // A handoff still requires completion of every region.
 std::stable_sort(work.begin(),work.end(),[](const auto & a,const auto & b){return a.region.area>b.region.area;});
 for(auto & item:work) {
  auto region=item.region;const auto & frontier=item.frontier;
  if(timed()){out.complete=false;return out;}
  if(item.stair) {
    region.state="transition";out.transition+=region.area;out.regions.push_back(region);continue;
  }
  // Runtime evidence never skips visibility/support/route validation.
  bool opportunity=false,uncertain=false,route_rejected=false;
  size_t tested=0;
  for(int i:frontier) {
   auto target=map.mapToWorld(i%w,i/w);
   for(int cell:queue) {
    auto view=map.mapToWorld(cell%w,cell/w);
    double distance=std::hypot(view.first-target.first,view.second-target.second);
    if(distance<.1 || distance>4.)continue;
    if(timed()){out.complete=false;uncertain=true;break;}
    ++out.sites;++tested;int result=can_observe(view,target);
    if(result==-2){route_rejected=true;continue;}
    if(result<0){uncertain=true;continue;}
    if(result>0){opportunity=true;if(!out.reference){out.reference=target;out.view=view;}break;}
   }
   if(opportunity || timed())break;
  }
  if(opportunity)uncertain=false;
  if(uncertain)out.complete=false;
  region.state=uncertain?"undetermined":opportunity?"actionable":
    tested==0?"no_known_access":route_rejected?"no_route":"occluded";
  if(opportunity || uncertain)out.pending+=region.area;else out.deferred+=region.area;
  out.regions.push_back(region);

 }
 return out;
}
// Exact ray test for one residual boundary cell; no 256-cell truncation.
// A known-free endpoint can still contain an unknown half-voxel on the ray.
inline bool auditCellVisible(const octomap::OcTree & tree,go2_surface_3d_planner::SurfacePoint view,
 std::pair<double,double> target,double floor,const VisibilitySensor & sensor) {
 double cy=std::cos(view.yaw),sy=std::sin(view.yaw);
 double ox=view.x+cy*sensor.x-sy*sensor.y,oy=view.y+sy*sensor.x+cy*sensor.y,oz=view.z+sensor.z;
 for(double height:{-.10,-.025,.02,.10}) {
  double tz=floor+height,wx=target.first-ox,wy=target.second-oy,wz=tz-oz;
  double range=std::sqrt(wx*wx+wy*wy+wz*wz);if(range<.1 || range>4.)continue;
  double bx=cy*wx+sy*wy,by=-sy*wx+cy*wy,bz=wz;
  double ux=-sensor.qx,uy=-sensor.qy,uz=-sensor.qz,qw=sensor.qw;
  double cx=uy*bz-uz*by,cyp=uz*bx-ux*bz,cz=ux*by-uy*bx;
  double lx=bx+2*(qw*cx+uy*cz-uz*cyp),ly=by+2*(qw*cyp+uz*cx-ux*cz),lz=bz+2*(qw*cz+ux*cyp-uy*cx);
  double pitch=std::atan2(lz,std::hypot(lx,ly));
  if(pitch<sensor.min_pitch || pitch>sensor.max_pitch)continue;
  auto endpoint=tree.search(target.first,target.second,tz);
  if(endpoint && tree.isNodeOccupied(endpoint))continue;
  thread_local octomap::KeyRay ray;if(!tree.computeRayKeys(octomap::point3d(ox,oy,oz),
     octomap::point3d(target.first,target.second,tz),ray))continue;
  bool blocked=false,potential=false;
  for(auto key:ray) {
   auto node=tree.search(key);if(node && tree.isNodeOccupied(node)){blocked=true;break;}
   auto p=tree.keyToCoord(key);
   if(!node && std::abs(p.x()-target.first)<=.05 && std::abs(p.y()-target.second)<=.05)potential=true;
  }
  auto node=tree.search(target.first,target.second,tz);
  if(node && tree.isNodeOccupied(node))blocked=true;
  if(!node)potential=true;
  if(!blocked && potential)return true;
 }
 return false;
}
struct AuditRouteCache {
 using Key=std::tuple<long long,long long,int>;
 std::map<Key,int> routes;
 std::map<Key,std::optional<go2_surface_3d_planner::SurfacePoint>> stances;
};
inline int auditAdmissibleView(const octomap::OcTree & tree,
 go2_surface_3d_planner::SurfaceSearchConfig config,
 std::pair<double,double> view,std::pair<double,double> target,double floor,
 const VisibilitySensor & sensor,std::chrono::steady_clock::time_point deadline,
 const std::optional<go2_surface_3d_planner::SurfacePoint> & start=std::nullopt,
 AuditRouteCache * cache=nullptr,double route_budget=.5) {
 using namespace go2_surface_3d_planner;
 config.snap_radius=0;config.clearance_margin=std::max(.01,config.narrow_clearance_margin);
 config.preferred_clearance=.01;config.search_deadline=deadline;
 SurfaceGraph graph(tree,config);bool uncertain=false,route_rejected=false;
 // A centred sensor with only mounting yaw has identical ray origins and
 // vertical FOV for every body yaw. Reject occlusion once, not yaw_bins times.
 const bool yaw_invariant=std::hypot(sensor.x,sensor.y)<1e-9 &&
   std::hypot(sensor.qx,sensor.qy)<1e-9;
 bool shared_potential=false;
 if(yaw_invariant) {
   for(double dz:{-.1,0.,.1})if(auditCellVisible(tree,{view.first,view.second,floor+dz,0},target,floor,sensor)){
     shared_potential=true;break;
   }
   if(!shared_potential)return 0;
 }
 for(int k=0;k<config.yaw_bins;++k) {
  if(std::chrono::steady_clock::now()>=deadline)return -1;
  double yaw=k*2*M_PI/config.yaw_bins;
  const auto key=std::make_tuple(std::llround(view.first*1e4),std::llround(view.second*1e4),k);
  if(cache && cache->stances.count(key) && !cache->stances.at(key))continue;
  // Fast occlusion screening over the flat-floor height uncertainty band.
  bool potential=shared_potential;
  if(!yaw_invariant)for(double dz:{-.1,0.,.1})if(auditCellVisible(tree,{view.first,view.second,floor+dz,yaw},target,floor,sensor)){potential=true;break;}
  if(!potential)continue;
  try {
   auto stance=(cache && cache->stances.count(key))?cache->stances.at(key):
     graph.snapToSurface({view.first,view.second,floor,yaw});
   if(cache)cache->stances[key]=stance;
   if(stance && auditCellVisible(tree,*stance,target,floor,sensor)) {
     if(!start)return 1;
     auto origin=graph.snapToSurface(*start);if(!origin)return -1;
     if(cache && cache->routes.count(key)) {
       if(cache->routes.at(key)>0)return 1;
       if(cache->routes.at(key)<0){uncertain=true;continue;}
       route_rejected=true;continue;
     }
     auto attempt=config;
     attempt.search_deadline=std::min(deadline,std::chrono::steady_clock::now()+
       std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(route_budget)));
     // Try the configured lattice first; any returned path still has full
     // swept-body validation. Refine an empty completed search before rejecting.
     auto path=SurfaceGraph(tree,attempt).plan(*origin,*stance);
     if(path.empty() && attempt.xy_resolution>tree.getResolution()*.5) {
       attempt.xy_resolution=tree.getResolution()*.5;
       path=SurfaceGraph(tree,attempt).plan(*origin,*stance);
     }
     if(cache)cache->routes[key]=path.empty()?0:1;
     if(!path.empty())return 1;
     route_rejected=true;
   }
   }catch(const SurfaceSearchTimeout &){
    if(cache)cache->routes[key]=-1;
    return -1; // Do not spend another full route budget on each heading.
  }
 }
 return uncertain?-1:route_rejected?-2:0;
}
class ResidualAuditStability {
public:
 void reset(){hash_=0;records_.clear();samples_=0;since_=0;stamp_=-1;last_now_=-1;}
 bool observe(const ResidualAudit & audit,double now,int64_t stamp) {
  if(!audit.handoff()){reset();return false;}
  auto records=audit.records()+":"+std::to_string(audit.transition_hash)+":"+std::to_string(audit.runtime_hash);
  if(hash_!=audit.fingerprint || records_!=records || now<last_now_) {
   reset();hash_=audit.fingerprint;records_=records;since_=now;
  }
  if(stamp>stamp_){++samples_;stamp_=stamp;}
  last_now_=now;return samples_>=3 && now-since_>=10.;
 }
 unsigned samples()const{return samples_;}
 double seconds(double now)const{return std::max(0.,now-since_);}
private:
 uint64_t hash_{0};std::string records_;unsigned samples_{0};double since_{0},last_now_{-1};int64_t stamp_{-1};
};

}
