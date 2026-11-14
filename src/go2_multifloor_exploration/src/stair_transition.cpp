#include "go2_multifloor_exploration/stair_transition.hpp"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
namespace go2_multifloor_exploration {
using go2_surface_3d_planner::SurfaceGraph;
using go2_surface_3d_planner::SurfaceSearchTimeout;
namespace {
bool expired(const SurfaceSearchConfig & c) {return std::chrono::steady_clock::now()>=c.search_deadline;}
bool observedTop(const octomap::OcTree & t,const SurfacePoint & p) {
 auto below=t.search(p.x,p.y,p.z-.001),above=t.search(p.x,p.y,p.z+.001);
 return below && t.isNodeOccupied(below) && (!above || !t.isNodeOccupied(above));
}
bool wideTread(const octomap::OcTree & t,const SurfacePoint & p,double yaw,double width) {
 const double dx=-std::sin(yaw)*width*.5,dy=std::cos(yaw)*width*.5;
 return observedTop(t,p) && observedTop(t,{p.x+dx,p.y+dy,p.z,yaw}) &&
  observedTop(t,{p.x-dx,p.y-dy,p.z,yaw});
}
}
StairScan detectStairs(const octomap::OcTree & tree,const SurfaceSearchConfig & config,
 const SurfacePoint & stance,const StairOptions & options) {
 StairScan result;std::vector<SurfacePoint> points;
 std::set<std::tuple<int,int,int>> cells;
 const double grid=std::max(.1,tree.getResolution()),zgrid=tree.getResolution();
 for(auto it=tree.begin_leafs();it!=tree.end_leafs();++it) {
  if(expired(config)) {result.timed_out=true;return result;}
  if(!tree.isNodeOccupied(*it)) {continue;}
  const double z=it.getZ()+it.getSize()*.5;
  if(z<stance.z+options.minimum_rise || z>stance.z+1.5 ||
   std::hypot(it.getX()-stance.x,it.getY()-stance.y)>options.range) {continue;}
  const int n=std::max(1,static_cast<int>(std::ceil(it.getSize()/grid)));
  for(int ix=0;ix<n && static_cast<int>(points.size())<options.maximum_surfaces;++ix)
   for(int iy=0;iy<n && static_cast<int>(points.size())<options.maximum_surfaces;++iy) {
   SurfacePoint p{it.getX()-it.getSize()*.5+(ix+.5)*it.getSize()/n,
    it.getY()-it.getSize()*.5+(iy+.5)*it.getSize()/n,z,0};
   auto key=std::make_tuple(static_cast<int>(std::floor(p.x/grid)),
    static_cast<int>(std::floor(p.y/grid)),static_cast<int>(std::llround(p.z/zgrid)));
   if(cells.count(key) || !observedTop(tree,p)) {continue;}
   cells.insert(key);points.push_back(p);
   if(static_cast<int>(points.size())>=options.maximum_surfaces) {break;}
  }
  if(static_cast<int>(points.size())>=options.maximum_surfaces) {break;}
 }
 // Local spatial buckets bound adjacency work; walls are eliminated by their observed tops.
 std::map<std::pair<int,int>,std::vector<std::size_t>> buckets;
 auto neighbors=[&](const SurfacePoint & p) {
  std::vector<std::size_t> ids;int x=std::floor(p.x/options.maximum_run),y=std::floor(p.y/options.maximum_run);
  for(int dx=-1;dx<=1;++dx) for(int dy=-1;dy<=1;++dy) {
   auto it=buckets.find({x+dx,y+dy});if(it!=buckets.end()) ids.insert(ids.end(),it->second.begin(),it->second.end());
  }return ids;
 };
 std::sort(points.begin(),points.end(),[](const auto & a,const auto & b){return a.z<b.z;});
 // Build spatial indices after the height ordering is fixed.
 buckets.clear();for(std::size_t i=0;i<points.size();++i)
  buckets[{static_cast<int>(std::floor(points[i].x/options.maximum_run)),
    static_cast<int>(std::floor(points[i].y/options.maximum_run))}].push_back(i);
 SurfaceSearchConfig strict=config;strict.infer_flat_support=false;strict.excluded_regions.clear();
 SurfaceGraph graph(tree,strict);
 for(const auto & seed:points) {
  if(expired(config)) {result.timed_out=true;break;}
  if(seed.z>stance.z+options.maximum_rise+.05) {continue;}
  for(auto id:neighbors(seed)) {
   const auto & next=points[id];double rise=next.z-seed.z,run=std::hypot(next.x-seed.x,next.y-seed.y);
   if(rise<options.minimum_rise || rise>options.maximum_rise+1e-6 || run<.07 || run>options.maximum_run) {continue;}
   const double yaw=std::atan2(next.y-seed.y,next.x-seed.x);
   if(!wideTread(tree,seed,yaw,options.minimum_width) || !wideTread(tree,next,yaw,options.minimum_width)) {continue;}
   std::vector<SurfacePoint> chain{seed,next};double length=run;
   while(length<options.maximum_segment_length) {
    if(expired(config)) {result.timed_out=true;return result;}
    std::optional<SurfacePoint> best;double best_score=1e9;
    for(auto ni:neighbors(chain.back())) {
     auto p=points[ni];const auto & back=chain.back();double dz=p.z-back.z,dx=p.x-back.x,dy=p.y-back.y;
     double along=std::cos(yaw)*dx+std::sin(yaw)*dy,cross=-std::sin(yaw)*dx+std::cos(yaw)*dy;
     double distance=std::hypot(dx,dy);
     if(dz<options.minimum_rise || dz>options.maximum_rise+1e-6 || along<.07 ||
      distance>options.maximum_run || std::abs(cross)>.12 || length+distance>options.maximum_segment_length ||
      !wideTread(tree,p,yaw,options.minimum_width)) {continue;}
     const double score=distance+2*std::abs(cross);
     if(score<best_score) {best=p;best_score=score;}
    }
    if(!best) {break;}
    length+=std::hypot(best->x-chain.back().x,best->y-chain.back().y);chain.push_back(*best);
   }
   if(static_cast<int>(chain.size())<options.minimum_steps || chain.back().z-stance.z<options.minimum_chain_rise) {continue;}
   bool valid=true;
   for(auto & p:chain) {p.yaw=yaw;if(!graph.isWalkableSurface(p)) {valid=false;break;}}
   if(!valid) {continue;}
   if(std::any_of(result.candidates.begin(),result.candidates.end(),[&](const auto & c){
    return std::hypot(c.first_tread.x-seed.x,c.first_tread.y-seed.y)<.65 &&
     std::abs(std::remainder(c.first_tread.yaw-yaw,2*M_PI))<.35;})) {continue;}
   auto first=chain.front(),target=chain.back();
   SurfacePoint approach{first.x-options.approach_distance*std::cos(yaw),
    first.y-options.approach_distance*std::sin(yaw),stance.z,yaw};
   const double span=std::hypot(target.x-first.x,target.y-first.y)+.25;
   SurfaceExclusionRegion region{(first.x+target.x)*.5,(first.y+target.y)*.5,yaw,
     span*.5,options.minimum_width*.5+.15,stance.z-.15,target.z+.15};
   result.candidates.push_back({first,approach,target,region,chain});
   if(static_cast<int>(result.candidates.size())>=options.maximum_candidates) return result;
   {break;}
  }
 }
 std::sort(result.candidates.begin(),result.candidates.end(),[&](const auto & a,const auto & b){
  return std::hypot(a.approach.x-stance.x,a.approach.y-stance.y)<std::hypot(b.approach.x-stance.x,b.approach.y-stance.y);});
 return result;
}
bool hasLandingPatch(const octomap::OcTree & tree,const SurfaceSearchConfig & config,
 const SurfacePoint & point,double radius) {
 auto strict=config;strict.infer_flat_support=false;strict.excluded_regions.clear();SurfaceGraph graph(tree,strict);
 if(!observedTop(tree,point) || !graph.isWalkableSurface(point)) return false;
 int count=0;for(int i=0;i<12;++i) {
  const double a=i*2*M_PI/12;SurfacePoint p{point.x+radius*std::cos(a),point.y+radius*std::sin(a),point.z,point.yaw};
  if(observedTop(tree,p) && graph.isWalkableSurface(p)) ++count;
 }return count>=10;
}
bool reachedStairTarget(const SurfacePoint & actual,const SurfacePoint & target,double xy,double z) {
 return std::isfinite(actual.x) && std::isfinite(actual.y) && std::isfinite(actual.z) &&
  std::hypot(actual.x-target.x,actual.y-target.y)<=xy && std::abs(actual.z-target.z)<=z;
}
std::vector<SurfacePoint> verifyStairApproach(const octomap::OcTree & tree,SurfaceSearchConfig config,
 const SurfacePoint & stance,const StairCandidate & candidate) {
 const double distance=std::hypot(candidate.approach.x-stance.x,candidate.approach.y-stance.y);
 auto valid=[&](const std::vector<SurfacePoint> & path) {
  if(path.empty()) return false;
  for(const auto & p:path) if(std::abs(p.z-stance.z)>.08) return false;
  return true;
 };
 if(distance<=2.) {
  auto path=SurfaceGraph(tree,config).plan(stance,candidate.approach);
  if(valid(path)) return path;
 }
 const double heading=std::atan2(candidate.approach.y-stance.y,candidate.approach.x-stance.x);
 for(double length:{1.2,.75}) for(double offset:{0.,M_PI/6,-M_PI/6,M_PI/3,-M_PI/3}) {
  SurfacePoint target{stance.x+length*std::cos(heading+offset),stance.y+length*std::sin(heading+offset),stance.z,stance.yaw};
  if(std::hypot(target.x-candidate.approach.x,target.y-candidate.approach.y)>distance-.25) {continue;}
  auto attempt=config;attempt.search_deadline=std::min(config.search_deadline,
    std::chrono::steady_clock::now()+std::chrono::milliseconds(180));
  try {auto path=SurfaceGraph(tree,attempt).plan(stance,target);if(valid(path)) return path;}
  catch(const SurfaceSearchTimeout &) {}
 }
 return {};
}
}  // namespace go2_multifloor_exploration
