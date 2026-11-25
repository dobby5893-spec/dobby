#include <octomap/OcTree.h>
#include <cmath>
#include <iostream>
#include <frontier_exploration_ros2/map_consistency.hpp>
void box(octomap::OcTree & t,double a,double b,double c,double d,double top) {
 for(int x=std::floor(a/.05);x<std::ceil(b/.05);++x)
 for(int y=std::floor(c/.05);y<std::ceil(d/.05);++y)
 for(int z=-2;z<std::round(top/.05);++z)t.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,(z+.5)*.05),true);
}
int main(int argc,char ** argv) {
 if(argc!=2 && argc!=3) {return 1;}octomap::OcTree tree(.05);box(tree,-2,0,-2,2,0);
 for(int i=0;i<6;++i)box(tree,i*.25,(i+1)*.25,-.8,.8,(i+1)*.05);
 box(tree,1.5,3.5,-1.5,1.5,.30);if(argc==3) {
  using namespace frontier_exploration_ros2;
  octomap_msgs::msg::Octomap source;octomap_msgs::fullMapToMsg(tree,source);source.header.frame_id="map";
  auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>();f->header.frame_id="map";
  f->info.resolution=.1;f->info.width=f->info.height=1;f->info.origin.orientation.w=1.;f->info.origin.position.z=std::stod(argv[2]);f->data={0};
  PairedFloorMap pair;pair.floor=f;pair.tree=std::make_shared<const octomap::OcTree>(tree);
  pair.version={1,1,octomapDigest(source),floorDigest(*f)};
  auto evidence=beginMapEvidence(pair,-.8,0);sealDependency(evidence,{0,-1,-1,1,1,0,0},pair);
  sealDependency(evidence,{11,-1,-1,1,1,0,0},pair);
  std::cout<<encodeMapEvidence(evidence);return 0;
 }return tree.write(argv[1])?0:2;
}
