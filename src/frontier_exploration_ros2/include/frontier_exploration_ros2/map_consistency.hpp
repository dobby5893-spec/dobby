#pragma once
#include "frontier_exploration_ros2/frontier_types.hpp"
#include "frontier_exploration_ros2/exploration_boundary.hpp"
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <queue>
namespace frontier_exploration_ros2 {
// Generation identities come from the slice producer, never subscriber counters.
struct MapVersion {
 uint64_t octomap_generation{0},floor_map_generation{0},source_digest{0},floor_digest{0};
};
struct PairedFloorMap {
 MapVersion version;
 nav_msgs::msg::OccupancyGrid::ConstSharedPtr floor;
 std::shared_ptr<const octomap::OcTree> tree;
 double source_stamp{0};
};
inline void mapHashMix(uint64_t & h,uint64_t v){h=(h^v)*1099511628211ULL;}
inline uint64_t mapHashSeed(){return 1469598103934665603ULL;}
inline void mapHashDouble(uint64_t & h,double v){uint64_t bits;std::memcpy(&bits,&v,sizeof(bits));mapHashMix(h,bits);}
inline uint64_t octomapDigest(const octomap_msgs::msg::Octomap & m) {
 uint64_t h=mapHashSeed();for(auto c:m.header.frame_id)mapHashMix(h,uint8_t(c));
 for(auto c:m.id){mapHashMix(h,uint8_t(c));}
 mapHashMix(h,m.binary);mapHashDouble(h,m.resolution);
 for(auto v:m.data){mapHashMix(h,uint8_t(v));}
 return h;
}
inline uint64_t floorSchemaDigest(const nav_msgs::msg::OccupancyGrid & m) {
 uint64_t h=mapHashSeed();for(auto c:m.header.frame_id)mapHashMix(h,uint8_t(c));
 mapHashMix(h,m.info.width);mapHashMix(h,m.info.height);mapHashDouble(h,m.info.resolution);
 const auto & p=m.info.origin.position;const auto & q=m.info.origin.orientation;
 for(double v:{p.x,p.y,p.z,q.x,q.y,q.z,q.w}){mapHashDouble(h,v);}
 return h;
}
inline uint64_t floorDigest(const nav_msgs::msg::OccupancyGrid & m) {
 auto h=floorSchemaDigest(m);for(auto v:m.data){mapHashMix(h,uint8_t(v));}
 return h;
}
template<class Message> inline std::shared_ptr<const PairedFloorMap> decodeFloorSnapshot(const Message & m,
 std::shared_ptr<const octomap::OcTree> reusable={},uint64_t reusable_digest=0) {
 const auto & f=m.floor_map;
 if(!m.octomap_generation || !m.floor_map_generation || m.source_octomap_generation!=m.octomap_generation ||
    f.header.frame_id!=m.octomap.header.frame_id || f.header.frame_id.empty() ||
    !std::isfinite(f.info.resolution) || f.info.resolution<=0 || !f.info.width || !f.info.height ||
    uint64_t(f.info.width)*f.info.height!=f.data.size() || !std::isfinite(f.info.origin.position.z))return {};
 const auto source_digest=octomapDigest(m.octomap);
 auto out=std::make_shared<PairedFloorMap>();
 if(reusable && reusable_digest==source_digest)out->tree=std::move(reusable);
 else {
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap_msgs::msgToMap(m.octomap));
  auto tree=dynamic_cast<octomap::OcTree*>(raw.get());if(!tree)return {};
  raw.release();out->tree.reset(tree);
 }
 out->floor=std::make_shared<const nav_msgs::msg::OccupancyGrid>(f);
 out->version={m.octomap_generation,m.floor_map_generation,source_digest,floorDigest(f)};
 out->source_stamp=m.octomap.header.stamp.sec+m.octomap.header.stamp.nanosec*1e-9;return out;
}
struct MapDependency {
 uint64_t region{0};double min_x{0},min_y{0},max_x{0},max_y{0};
 uint64_t floor_hash{0},octomap_hash{0};
};
// Conservative vertical columns include support, body, sensor rays and upper-floor
// occluders. Pruned leaves intersecting a column must not be missed by center tests.
inline uint64_t regionalOctomapDigest(const octomap::OcTree & tree,const MapDependency & b) {
 uint64_t h=mapHashSeed();mapHashDouble(h,tree.getResolution());
 for(auto it=tree.begin_leafs();it!=tree.end_leafs();++it) {
  double r=it.getSize()*.5;
  if(it.getX()+r<b.min_x || it.getX()-r>b.max_x || it.getY()+r<b.min_y || it.getY()-r>b.max_y)continue;
  const auto & k=it.getKey();for(int i=0;i<3;++i)mapHashMix(h,k[i]);
  mapHashMix(h,it.getDepth());mapHashMix(h,tree.isNodeOccupied(*it)?2:1);
 }return h;
}
inline uint64_t wholeOctomapDigest(const octomap::OcTree & tree) {
 return regionalOctomapDigest(tree,{0,-INFINITY,-INFINITY,INFINITY,INFINITY,0,0});
}
// Regional evidence may survive an unrelated live update, but final authority
// must wait until a source-matched floor slice accounts for the new 3D contents.
// This is a publication/consumption gate, not a reason to clear regional caches.
inline bool pairedSourceCaughtUp(const PairedFloorMap & pair,const octomap::OcTree * live) {
 return !live || wholeOctomapDigest(*pair.tree)==wholeOctomapDigest(*live);
}
struct AuditMapEvidence {
 MapVersion version;double floor_z{0},margin{0},start_x{0},start_y{0};
 uint64_t schema{0},unknowns{0};ExplorationBoundaryPolygon boundary;
 std::vector<MapDependency> dependencies;
 bool present()const{return version.octomap_generation && version.floor_map_generation;}
 uint64_t stableDigest()const {
  auto h=schema;mapHashMix(h,unknowns);
  for(const auto & d:dependencies){mapHashMix(h,d.region);mapHashMix(h,d.floor_hash);mapHashMix(h,d.octomap_hash);}return h;
 }
};
inline std::vector<uint8_t> auditReachability(const OccupancyGrid2d & m,const AuditMapEvidence & e) {
 const int w=m.getSizeX(),h=m.getSizeY();std::vector<uint8_t> reach(w*h,0);int x,y;
 if(!m.worldToMapNoThrow(e.start_x,e.start_y,x,y) || m.getCost(x,y)!=0)return reach;
 std::vector<int> queue{y*w+x};reach[y*w+x]=1;
 for(size_t i=0;i<queue.size();++i) {
  int cx=queue[i]%w,cy=queue[i]/w;
  for(auto d:std::vector<std::pair<int,int>>{{1,0},{-1,0},{0,1},{0,-1}}) {
   int nx=cx+d.first,ny=cy+d.second;
   if(nx<0 || ny<0 || nx>=w || ny>=h || reach[ny*w+nx] || m.getCost(nx,ny)!=0)continue;
   if(!boundary_return_step(m.mapToWorld(cx,cy),m.mapToWorld(nx,ny),e.boundary,0))continue;
   reach[ny*w+nx]=1;queue.push_back(ny*w+nx);
  }
 }return reach;
}
inline uint64_t unknownInventoryDigest(const OccupancyGrid2d & m,const AuditMapEvidence & e) {
 auto h=mapHashSeed();for(int y=0;y<m.getSizeY();++y)for(int x=0;x<m.getSizeX();++x)
  if(m.getCost(x,y)<0 && is_inside_exploration_boundary(m.mapToWorld(x,y),e.boundary,e.margin))mapHashMix(h,uint64_t(y*m.getSizeX()+x));
 return h;
}
inline uint64_t regionalFloorDigest(const OccupancyGrid2d & m,const MapDependency & b,const std::vector<uint8_t> & reach) {
 auto h=mapHashSeed();for(int y=0;y<m.getSizeY();++y)for(int x=0;x<m.getSizeX();++x) {
  auto p=m.mapToWorld(x,y);double r=m.map().info.resolution*.5;
  if(p.first+r<b.min_x || p.first-r>b.max_x || p.second+r<b.min_y || p.second-r>b.max_y)continue;
  int i=y*m.getSizeX()+x;mapHashMix(h,i);mapHashMix(h,uint8_t(m.getCost(x,y)));mapHashMix(h,reach[i]);
 }return h;
}
inline AuditMapEvidence beginMapEvidence(const PairedFloorMap & pair,double x,double y,
 const ExplorationBoundaryPolygon & boundary={},double margin=0) {
 AuditMapEvidence e;e.version=pair.version;e.floor_z=pair.floor->info.origin.position.z;
 e.start_x=x;e.start_y=y;e.boundary=boundary;e.margin=margin;e.schema=floorSchemaDigest(*pair.floor);
 e.unknowns=unknownInventoryDigest(OccupancyGrid2d(*pair.floor),e);return e;
}
inline void sealDependency(AuditMapEvidence & e,MapDependency d,const PairedFloorMap & pair) {
 OccupancyGrid2d grid(*pair.floor);d.floor_hash=regionalFloorDigest(grid,d,auditReachability(grid,e));
 d.octomap_hash=regionalOctomapDigest(*pair.tree,d);e.dependencies.push_back(d);
}
inline bool dependencyUnchanged(const MapDependency & d,const AuditMapEvidence & e,const PairedFloorMap & current,
 const octomap::OcTree * live=nullptr) {
 OccupancyGrid2d grid(*current.floor);
 return regionalFloorDigest(grid,d,auditReachability(grid,e))==d.floor_hash &&
  regionalOctomapDigest(*current.tree,d)==d.octomap_hash &&
  (!live || regionalOctomapDigest(*live,d)==d.octomap_hash);
}
inline bool mapEvidenceCurrent(const AuditMapEvidence & e,const PairedFloorMap & current,const octomap::OcTree * live=nullptr) {
 if(!e.present() || e.dependencies.empty() || e.version.octomap_generation>current.version.octomap_generation ||
    e.version.floor_map_generation>current.version.floor_map_generation ||
    (e.version.octomap_generation==current.version.octomap_generation && e.version.source_digest!=current.version.source_digest) ||
    (e.version.floor_map_generation==current.version.floor_map_generation && e.version.floor_digest!=current.version.floor_digest) ||
    e.schema!=floorSchemaDigest(*current.floor) || std::abs(e.floor_z-current.floor->info.origin.position.z)>1e-6)return false;
 OccupancyGrid2d grid(*current.floor);if(unknownInventoryDigest(grid,e)!=e.unknowns)return false;
 for(const auto & d:e.dependencies)if(!dependencyUnchanged(d,e,current,live))return false;
 return true;
}
// Additive metadata on the existing authority channel, not the future v5 protocol.
// Delimiters deliberately exclude ';' and '=' used by the outer report.
inline std::string encodeMapEvidence(const AuditMapEvidence & e) {
 std::ostringstream s;s<<std::setprecision(17)<<e.version.octomap_generation<<","<<e.version.floor_map_generation<<","<<e.version.source_digest<<","<<e.version.floor_digest
 <<","<<e.floor_z<<","<<e.schema<<","<<e.unknowns<<","<<e.margin<<","<<e.start_x<<","<<e.start_y<<"|";
 for(size_t i=0;i<e.boundary.size();++i){if(i)s<<"/";s<<e.boundary[i].first<<","<<e.boundary[i].second;}s<<"|";
 for(size_t i=0;i<e.dependencies.size();++i){const auto & d=e.dependencies[i];if(i)s<<"/";
  s<<d.region<<","<<d.min_x<<","<<d.min_y<<","<<d.max_x<<","<<d.max_y<<","<<d.floor_hash<<","<<d.octomap_hash;}
 return s.str();
}
inline std::vector<std::string> mapTokens(const std::string & s,char delimiter) {
 std::vector<std::string> out;std::istringstream in(s);std::string part;
 while(std::getline(in,part,delimiter)){out.push_back(part);}
 return out;
}
inline std::optional<AuditMapEvidence> decodeMapEvidence(const std::string & text) {
 try {
  if(text.size()>2000000)return {};
  auto parts=mapTokens(text,'|');if(parts.size()!=3)return {};
  auto v=mapTokens(parts[0],',');if(v.size()!=10)return {};
  auto u=[](const std::string & s){if(s.empty() || s.find_first_not_of("0123456789")!=std::string::npos)throw 1;
   size_t n=0;auto x=std::stoull(s,&n);if(n!=s.size())throw 1;return uint64_t(x);};
  auto f=[](const std::string & s){size_t n=0;auto x=std::stod(s,&n);if(n!=s.size() || !std::isfinite(x))throw 1;return x;};
  AuditMapEvidence e;e.version={u(v[0]),u(v[1]),u(v[2]),u(v[3])};e.floor_z=f(v[4]);e.schema=u(v[5]);
  e.unknowns=u(v[6]);e.margin=f(v[7]);e.start_x=f(v[8]);e.start_y=f(v[9]);if(!e.present() || e.margin<0)return {};
  for(const auto & part:mapTokens(parts[1],'/')){auto p=mapTokens(part,',');if(p.size()!=2)return {};e.boundary.push_back({f(p[0]),f(p[1])});}
  if(e.boundary.size()>10000 || (!e.boundary.empty() && e.boundary.size()<3))return {};
  for(const auto & part:mapTokens(parts[2],'/')) {auto d=mapTokens(part,',');if(d.size()!=7)return {};
   MapDependency b{u(d[0]),f(d[1]),f(d[2]),f(d[3]),f(d[4]),u(d[5]),u(d[6])};
   if(b.min_x>b.max_x || b.min_y>b.max_y)return {};
   e.dependencies.push_back(b);}
  if(e.dependencies.empty() || e.dependencies.size()>10000)return {};
  return e;
 }catch(...){return {};}
}
inline std::optional<AuditMapEvidence> completionMapEvidence(const std::string & report) {
 const std::string key=";map_evidence=";auto at=report.find(key);if(at==std::string::npos)return {};
 auto end=report.find(';',at+key.size());return decodeMapEvidence(report.substr(at+key.size(),end==std::string::npos?end:end-at-key.size()));
}
} // namespace frontier_exploration_ros2
