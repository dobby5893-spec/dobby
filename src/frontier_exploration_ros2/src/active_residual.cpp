#include "frontier_exploration_ros2/active_residual.hpp"
namespace frontier_exploration_ros2 {
using namespace go2_surface_3d_planner;
namespace {
using Clock=std::chrono::steady_clock;
void budgetCheck(Clock::time_point deadline){if(Clock::now()>=deadline)throw SurfaceSearchTimeout();}
bool knownSearchEnvelope(const octomap::OcTree & tree,const ResidualPose & a,const ResidualPose & b,
 const SurfaceSearchConfig & config,Clock::time_point deadline) {
 octomap::OcTreeKey lo,hi;
 const double pad=config.search_padding_xy+config.collision_radius+std::abs(config.cylinder_offset);
 if(!tree.coordToKeyChecked(std::min(a.x,b.x)-pad,std::min(a.y,b.y)-pad,std::min(a.z,b.z)-config.search_padding_z-config.support_probe_depth,lo) ||
    !tree.coordToKeyChecked(std::max(a.x,b.x)+pad,std::max(a.y,b.y)+pad,std::max(a.z,b.z)+config.search_padding_z+config.body_height,hi))return false;
 for(unsigned x=lo[0];x<=hi[0];++x)for(unsigned y=lo[1];y<=hi[1];++y)for(unsigned z=lo[2];z<=hi[2];++z) {
  budgetCheck(deadline);if(!tree.search(octomap::OcTreeKey(x,y,z)))return false;
 }return true;
}
SpatialOutcome rayOutcome(const octomap::OcTree & tree,const ResidualPose & view,
 std::pair<double,double> target,double floor,const VisibilitySensor & sensor) {
 if(auditCellVisible(tree,view,target,floor,sensor))return SpatialOutcome::Ready;
 bool in_fov=false,unknown_reason=false;
 const double cy=std::cos(view.yaw),sy=std::sin(view.yaw);
 const double ox=view.x+cy*sensor.x-sy*sensor.y,oy=view.y+sy*sensor.x+cy*sensor.y,oz=view.z+sensor.z;
 for(double h:{-.10,-.025,.02,.10}) {
  double x=target.first-ox,y=target.second-oy,z=floor+h-oz;
  if(std::sqrt(x*x+y*y+z*z)<.1 || std::sqrt(x*x+y*y+z*z)>4.)continue;
  double bx=cy*x+sy*y,by=-sy*x+cy*y,bz=z,ux=-sensor.qx,uy=-sensor.qy,uz=-sensor.qz;
  double cx=uy*bz-uz*by,cyp=uz*bx-ux*bz,cz=ux*by-uy*bx;
  double lx=bx+2*(sensor.qw*cx+uy*cz-uz*cyp),ly=by+2*(sensor.qw*cyp+uz*cx-ux*cz),lz=bz+2*(sensor.qw*cz+ux*cyp-uy*cx);
  double pitch=std::atan2(lz,std::hypot(lx,ly));if(pitch<sensor.min_pitch || pitch>sensor.max_pitch)continue;
  in_fov=true;octomap::KeyRay ray;
  if(!tree.computeRayKeys({float(ox),float(oy),float(oz)},{float(target.first),float(target.second),float(floor+h)},ray)){unknown_reason=true;continue;}
  bool blocked=false;for(const auto & key:ray){auto n=tree.search(key);if(n && tree.isNodeOccupied(n)){blocked=true;break;}}
  auto endpoint=tree.search(target.first,target.second,floor+h);blocked|=endpoint && tree.isNodeOccupied(endpoint);
  if(!blocked)unknown_reason=true;
 }
 return !in_fov?SpatialOutcome::OutsideFov:unknown_reason?SpatialOutcome::MapInsufficient:SpatialOutcome::Occluded;
}
}
bool knownFreeBodyFootprint(const OccupancyGrid2d & grid,const ResidualPose & pose,
 const SurfaceSearchConfig & config,const ExplorationBoundaryPolygon & boundary) {
 const auto & info=grid.map().info;const double res=info.resolution,r=config.collision_radius+config.clearance_margin;
 for(double sign:{-1.,1.}) {
  const double cx=pose.x+sign*config.cylinder_offset*std::cos(pose.yaw),cy=pose.y+sign*config.cylinder_offset*std::sin(pose.yaw);
  const int low_x=std::floor((cx-r-info.origin.position.x)/res),high_x=std::floor((cx+r-info.origin.position.x)/res);
  const int low_y=std::floor((cy-r-info.origin.position.y)/res),high_y=std::floor((cy+r-info.origin.position.y)/res);
  if(low_x<0 || low_y<0 || high_x>=grid.getSizeX() || high_y>=grid.getSizeY())return false;
  for(int y=low_y;y<=high_y;++y)for(int x=low_x;x<=high_x;++x) {
   auto p=grid.mapToWorld(x,y);double dx=std::max(0.,std::abs(cx-p.first)-res*.5),dy=std::max(0.,std::abs(cy-p.second)-res*.5);
   if(std::hypot(dx,dy)>r)continue;
   if(grid.getCost(x,y)!=0 || !is_inside_exploration_boundary(p,boundary,0.))return false;
  }
 }return true;
}
UnknownVoxelWindow captureUnknownVoxels(const PairedFloorMap & pair,const ResidualRegion & region,
 double below,double above,Clock::time_point deadline,size_t limit) {
 UnknownVoxelWindow w;w.resolution=pair.tree->getResolution();std::set<std::tuple<unsigned,unsigned,unsigned>> seen;
 OccupancyGrid2d grid(*pair.floor);const double half=grid.map().info.resolution*.5,floor=pair.floor->info.origin.position.z;
 for(int cell:region.cells) {
  auto xy=grid.mapToWorld(cell%grid.getSizeX(),cell/grid.getSizeX());
  octomap::OcTreeKey lo,hi;
  if(!pair.tree->coordToKeyChecked(xy.first-half+1e-8,xy.second-half+1e-8,floor-below+1e-8,lo) ||
     !pair.tree->coordToKeyChecked(xy.first+half-1e-8,xy.second+half-1e-8,floor+above-1e-8,hi)){w.complete=false;continue;}
  for(unsigned x=lo[0];x<=hi[0];++x)for(unsigned y=lo[1];y<=hi[1];++y)for(unsigned z=lo[2];z<=hi[2];++z) {
   if(Clock::now()>=deadline || w.keys.size()>=limit){w.complete=false;return w;}
   octomap::OcTreeKey key(x,y,z);
   if(!pair.tree->search(key) && seen.insert({key[0],key[1],key[2]}).second)w.keys.push_back(key);
  }
 }return w;
}
VoxelGain measureVoxelGain(const UnknownVoxelWindow & before,const octomap::OcTree & after) {
 VoxelGain gain;if(std::abs(before.resolution-after.getResolution())>1e-9)return gain;
 std::set<std::pair<unsigned,unsigned>> free,occupied,all;
 for(const auto & key:before.keys)if(auto n=after.search(key)) {
  auto xy=std::make_pair(unsigned(key[0]),unsigned(key[1]));all.insert(xy);
  if(after.isNodeOccupied(n)){++gain.occupied_voxels;occupied.insert(xy);}else{++gain.free_voxels;free.insert(xy);}
 }
 double area=before.resolution*before.resolution;gain.free_area=free.size()*area;gain.occupied_area=occupied.size()*area;
 gain.union_area=all.size()*area;gain.volume=(gain.free_voxels+gain.occupied_voxels)*area*before.resolution;return gain;
}
bool observedClosedFence(const PairedFloorMap & pair,const ResidualRegion & region,
 const SurfaceSearchConfig & config,const VisibilitySensor & sensor,Clock::time_point deadline) {
 OccupancyGrid2d grid(*pair.floor);int w=grid.getSizeX(),h=grid.getSizeY();std::set<int> cells(region.cells.begin(),region.cells.end()),fence;
 if(cells.empty())return false;
 for(int i:cells)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
  budgetCheck(deadline);int x=i%w+dx,y=i/w+dy;if(x<0 || y<0 || x>=w || y>=h)return false;
  int j=y*w+x;if(cells.count(j))continue;if(grid.getCost(x,y)<65)return false;fence.insert(j);
 }
 if(fence.empty())return false;
 const double half=pair.floor->info.resolution*.5,floor=pair.floor->info.origin.position.z;
 const double top=std::max(config.body_height,sensor.z)+config.max_step_height;
 const double bottom=std::min(-.10,sensor.z-config.max_step_height);
 if(top<=config.max_step_height)return false;
 // Prove enclosure on the actual OctoMap lattice, rather than requiring an
 // entire float-rounded 2D boundary cell to be a solid voxel column. Unknown
 // columns are traversable in this proof, so a missing wall voxel breaks it.
 double min_x=INFINITY,min_y=INFINITY,max_x=-INFINITY,max_y=-INFINITY;
 for(int i:cells){auto p=grid.mapToWorld(i%w,i/w);min_x=std::min(min_x,p.first-half);min_y=std::min(min_y,p.second-half);max_x=std::max(max_x,p.first+half);max_y=std::max(max_y,p.second+half);}
 octomap::OcTreeKey lo,hi,zlo,zhi;double pad=grid.map().info.resolution+pair.tree->getResolution();
 if(!pair.tree->coordToKeyChecked(min_x-pad,min_y-pad,floor+bottom,lo) ||
    !pair.tree->coordToKeyChecked(max_x+pad,max_y+pad,floor+top,hi))return false;
 zlo=lo;zhi=hi;
 using XY=std::pair<unsigned,unsigned>;std::map<XY,bool> solid_cache;
 auto solid=[&](XY xy) {
  auto old=solid_cache.find(xy);if(old!=solid_cache.end())return old->second;
  bool value=true;
  for(unsigned z=zlo[2];z<=zhi[2];++z){budgetCheck(deadline);auto n=pair.tree->search(octomap::OcTreeKey(xy.first,xy.second,z));if(!n || !pair.tree->isNodeOccupied(n)){value=false;break;}}
  solid_cache[xy]=value;return value;
 };
 std::set<XY> seen;std::vector<XY> queue;
 for(int i:cells) {
  auto p=grid.mapToWorld(i%w,i/w);octomap::OcTreeKey a,b;
  if(!pair.tree->coordToKeyChecked(p.first-half+1e-8,p.second-half+1e-8,floor,a) ||
     !pair.tree->coordToKeyChecked(p.first+half-1e-8,p.second+half-1e-8,floor,b))return false;
  for(unsigned x=a[0];x<=b[0];++x)for(unsigned y=a[1];y<=b[1];++y)
   if(!solid({x,y}) && seen.insert({x,y}).second)queue.push_back({x,y});
 }
 for(size_t head=0;head<queue.size();++head) {
  budgetCheck(deadline);const auto p=queue[head];
  if(p.first<=lo[0] || p.first>=hi[0] || p.second<=lo[1] || p.second>=hi[1])return false;
  for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
   XY next{unsigned(int(p.first)+dx),unsigned(int(p.second)+dy)};
   if(!solid(next) && seen.insert(next).second)queue.push_back(next);
  }
 }
 return true;
}

SpatialPlan planSpatialRegion(const PairedFloorMap & pair,SpatialRegion region,const ResidualPose & body,
 SurfaceSearchConfig config,VisibilitySensor sensor,const ExplorationBoundaryPolygon & boundary,double margin,
 double budget,unsigned batch,unsigned retries,uint64_t epoch) {
 SpatialPlan out;out.region=std::move(region);out.region.retry_limit=retries;out.epoch=epoch;out.region.route_origin=std::make_pair(body.x,body.y);
 out.region.evaluated_origins.insert({int64_t(std::floor(body.x/.35)),int64_t(std::floor(body.y/.35))});
 const auto begin=Clock::now();const auto deadline=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(budget));
 config.search_deadline=deadline;OccupancyGrid2d grid(*pair.floor);const int width=grid.getSizeX(),count=width*grid.getSizeY();
 out.evidence=beginMapEvidence(pair,body.x,body.y,boundary,margin);
 const double padding=config.snap_radius+config.collision_radius+std::abs(config.cylinder_offset)+std::hypot(sensor.x,sensor.y)+grid.map().info.resolution;
 sealDependency(out.evidence,ActiveResidualBook::dependency(out.region.region,grid,padding),pair);
 sealDependency(out.evidence,{0,body.x-4.-padding,body.y-4.-padding,body.x+4.+padding,body.y+4.+padding,0,0},pair);
 try {
  if(observedClosedFence(pair,out.region.region,config,sensor,deadline))out.region.verified_enclosed=true;
  else if(!configureSupportPlane(config,*pair.tree,body)) {
   out.region.rejected.push_back({-1,SpatialOutcome::SupportUnverified});out.region.enumerated=true;out.complete_input=false;
   for(auto & a:out.region.attempts)if(a.second.outcome==SpatialOutcome::Ready || a.second.outcome==SpatialOutcome::Timeout)a.second.outcome=SpatialOutcome::SupportUnverified;
  } else {
   const ResidualPose origin{body.x,body.y,body.z-config.support_reference_height,body.yaw};
   unsigned tested=0;
   auto evaluate=[&](int cell,unsigned attempt) -> bool {
    budgetCheck(deadline);auto xy=grid.mapToWorld(cell%width,cell/width);SpatialAttempt option;option.cell=cell;option.retries=attempt;
    const auto target=grid.mapToWorld(out.region.frontier.front()%width,out.region.frontier.front()/width);
    const double preferred=std::atan2(target.second-xy.second,target.first-xy.first);
    bool supported=false,hidden=false,map_unknown=false,in_fov=false,route_rejected=false;
    std::optional<ResidualPose> last_stance;
    // Orientation bins select a safe navigation stance at ONE XY, following
    // the existing v4 audit. They never create separate in-place scan tasks.
    for(int k=0;k<=config.yaw_bins;++k) {
     budgetCheck(deadline);const int bin=(k+int(attempt-1)*std::max(1,config.yaw_bins/int(std::max(1u,retries))))%config.yaw_bins;
     const double yaw=(k==0 && attempt==1)?preferred:bin*2*M_PI/config.yaw_bins;
     option.pose={xy.first,xy.second,pair.floor->info.origin.position.z,yaw};
     if(!knownFreeBodyFootprint(grid,option.pose,config,boundary))continue;
     auto stance_config=config;stance_config.snap_radius=0;
     auto stance=SurfaceGraph(*pair.tree,stance_config).snapToSurface(option.pose);
     if(!stance || std::abs(stance->z-pair.floor->info.origin.position.z)>config.max_step_height)continue;
     supported=true;option.pose=*stance;last_stance=stance;option.visible_cells.clear();
     for(int i:out.region.frontier) {
      budgetCheck(deadline);auto t=grid.mapToWorld(i%width,i/width);
      const auto outcome=rayOutcome(*pair.tree,*stance,t,pair.floor->info.origin.position.z,sensor);
      if(outcome==SpatialOutcome::Ready){option.visible_cells.push_back(i);in_fov=true;}
      else if(outcome==SpatialOutcome::Occluded){hidden=true;in_fov=true;}
      else if(outcome==SpatialOutcome::MapInsufficient){map_unknown=true;in_fov=true;}
     }
     if(option.visible_cells.empty())continue;
     auto route_config=config;route_config.search_deadline=std::min(deadline,Clock::now()+
       std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(std::min(4.,std::max(.5,budget/10.)))));
     try {
      auto path=SurfaceGraph(*pair.tree,route_config).plan(origin,*stance);
      // Preserve v4's existing coarse-to-fine route verification, including
      // its full body/swept-motion checks and the same bounded route deadline.
      if(path.empty() && route_config.xy_resolution>pair.tree->getResolution()*.5) {
       route_config.xy_resolution=pair.tree->getResolution()*.5;
       path=SurfaceGraph(*pair.tree,route_config).plan(origin,*stance);
      }
      if(path.empty()){route_rejected=true;continue;}
      option.outcome=SpatialOutcome::Ready;
     }catch(const SurfaceSearchTimeout &){option.outcome=SpatialOutcome::Timeout;}
     option.visible_count=option.visible_cells.size();option.coverage_hash=mapHashSeed();
     for(int visible:option.visible_cells)mapHashMix(option.coverage_hash,visible);
     if(option.outcome==SpatialOutcome::Ready && (!out.selected || option.visible_cells.size()>out.selected->visible_cells.size()))out.selected=option;
     out.region.attempts[cell]=std::move(option);return true;
    }
    SpatialOutcome failure=!supported?SpatialOutcome::SupportUnverified:map_unknown?SpatialOutcome::MapInsufficient:
      !in_fov?SpatialOutcome::OutsideFov:hidden?SpatialOutcome::Occluded:SpatialOutcome::MapInsufficient;
    if(route_rejected && last_stance)failure=knownSearchEnvelope(*pair.tree,origin,*last_stance,config,deadline)?SpatialOutcome::NoRoute:SpatialOutcome::MapInsufficient;
    out.region.rejected.push_back({cell,failure});out.region.attempts.erase(cell);return route_rejected;
   };
   // Revalidate cached positive options from this measured origin before reuse.
   for(const auto & a:std::map<int,SpatialAttempt>(out.region.attempts))if(a.second.outcome==SpatialOutcome::Ready) {
    if(std::hypot(a.second.pose.x-body.x,a.second.pose.y-body.y)<=.35) {
     auto & skipped=out.region.attempts[a.first];skipped.outcome=SpatialOutcome::StaleMap;skipped.visible_cells.clear();continue;
    }
    evaluate(a.first,a.second.retries);++tested;if(out.selected)break;
   }
   if(out.selected) {
    // Drain already validated options before allocating another candidate batch.
   } else if(out.region.enumerated) {
    std::vector<int> retry;
    for(const auto & a:out.region.attempts)if(a.second.outcome==SpatialOutcome::Timeout && !a.second.arrived && a.second.retries<retries)retry.push_back(a.first);
    for(int cell:retry){if(tested++>=batch)break;evaluate(cell,out.region.attempts[cell].retries+1);}
   } else {
    for(;out.region.cursor<count && tested<batch;) {
     budgetCheck(deadline);int cell=out.region.cursor++;
     if(grid.getCost(cell%width,cell/width)!=0)continue;
     auto xy=grid.mapToWorld(cell%width,cell/width);if(!is_inside_exploration_boundary(xy,boundary,margin))continue;
     bool near=false;for(int target:out.region.frontier) {
      auto t=grid.mapToWorld(target%width,target/width);double d=std::hypot(t.first-xy.first,t.second-xy.second);if(d>=.1 && d<=4.){near=true;break;}
     }
     if(!near || std::hypot(xy.first-body.x,xy.second-body.y)<=.35)continue;
     bool duplicate=false;
     for(const auto & a:out.region.attempts)if((a.second.arrived || a.second.outcome==SpatialOutcome::Ready) &&
       std::hypot(a.second.pose.x-xy.first,a.second.pose.y-xy.second)<=.35){duplicate=true;break;}
     if(duplicate)continue;
     try{if(evaluate(cell,1))++tested;}catch(const SurfaceSearchTimeout &){SpatialAttempt a;a.cell=cell;a.retries=1;a.outcome=SpatialOutcome::Timeout;out.region.attempts[cell]=a;}
    }
    out.region.enumerated=out.region.cursor>=count;
   }
  }
 }catch(const SurfaceSearchTimeout &){out.complete_input=false;}
 out.elapsed=std::chrono::duration<double>(Clock::now()-begin).count();out.region.spent+=out.elapsed;
 return out;
}
} // namespace frontier_exploration_ros2
