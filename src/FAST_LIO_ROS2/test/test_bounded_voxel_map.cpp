#include "bounded_voxel_map.hpp"
#include <vector>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <fstream>
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(false)
struct Point {double x,y,z;int intensity;};
int main() {
  fastlio::BoundedVoxelMap<Point> map(.1,3);std::vector<Point> cloud;
  map.add({.01,.01,.01,1});map.add({.09,.09,.09,2});
  CHECK(map.size()==1);map.snapshot(cloud);CHECK(cloud.front().intensity==2);
  map.add({-.01,.01,.01,3});CHECK(map.size()==2); // floor, not truncation
  map.add({1,.01,.01,4});map.add({.02,.01,.01,5});map.add({2,.01,.01,6});
  CHECK(map.size()==3);map.snapshot(cloud);
  for(const auto & p:cloud)CHECK(p.intensity!=3); // least-recently used
  map.add({std::numeric_limits<double>::quiet_NaN(),0,0,9});
  map.add({std::numeric_limits<double>::infinity(),0,0,9});
  map.add({1.e30,0,0,9});CHECK(map.size()==3);
  bool threw=false;try{fastlio::BoundedVoxelMap<Point> invalid(0,1);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
  fastlio::BoundedVoxelMap<Point> stress(.05,500000);
  for(int batch=0;batch<12;++batch) {
    for(int i=0;i<500000;++i) {
      stress.add({(i+batch*500000)*.1+.01,.01,.01,i});
    }
    CHECK(stress.size()==500000);
    stress.snapshot(cloud);CHECK(cloud.size()==500000);
    std::ifstream status("/proc/self/status");std::string line;
    while(std::getline(status,line))if(line.find("VmRSS:")==0)std::cout<<"batch="<<batch<<" "<<line<<'\n';
  }
  const auto size=stress.size();
  for(int pass=0;pass<20;++pass)for(int i=0;i<500000;++i)
    stress.add({(i+11*500000)*.1+.01,.01,.01,i});
  CHECK(stress.size()==size);std::cout<<"PASS: deduplication, eviction, invalid points, 6M unique + 10M repeated inserts\n";
}
