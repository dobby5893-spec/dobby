#include <octomap/OcTree.h>
#include <cmath>
void box(octomap::OcTree & t,double a,double b,double c,double d,double top) {
 for(int x=std::floor(a/.05);x<std::ceil(b/.05);++x)
 for(int y=std::floor(c/.05);y<std::ceil(d/.05);++y)
 for(int z=-2;z<std::round(top/.05);++z)t.updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,(z+.5)*.05),true);
}
int main(int argc,char ** argv) {
 if(argc!=2) {return 1;}octomap::OcTree tree(.05);box(tree,-2,0,-2,2,0);
 for(int i=0;i<6;++i)box(tree,i*.25,(i+1)*.25,-.8,.8,(i+1)*.05);
 box(tree,1.5,3.5,-1.5,1.5,.30);return tree.write(argv[1])?0:2;
}
