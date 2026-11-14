#pragma once
#include <cmath>
#include <cstdint>
#include <list>
#include <iterator>
#include <stdexcept>
#include <unordered_map>

namespace fastlio {
// Display/export cache only. The estimator and obstacle map keep their own data.
template<class Point> class BoundedVoxelMap {
  struct Key {
    int64_t x,y,z;
    bool operator==(const Key & other) const {return x==other.x && y==other.y && z==other.z;}
  };
  struct Hash {
    std::size_t operator()(const Key & k) const {
      std::size_t h=std::hash<int64_t>{}(k.x);
      h^=std::hash<int64_t>{}(k.y)+0x9e3779b9+(h<<6)+(h>>2);
      return h^(std::hash<int64_t>{}(k.z)+0x9e3779b9+(h<<6)+(h>>2));
    }
  };
  using Order=std::list<Key>;
  struct Entry {Point point;typename Order::iterator order;};
  double resolution_;
  std::size_t capacity_;
  Order order_;
  std::unordered_map<Key,Entry,Hash> cells_;
public:
  BoundedVoxelMap(double resolution,std::size_t capacity):resolution_(resolution),capacity_(capacity) {
    if(!std::isfinite(resolution) || resolution<=0 || capacity==0)
      throw std::invalid_argument("Display voxel size and capacity must be positive");
  }
  BoundedVoxelMap(const BoundedVoxelMap &)=delete;
  BoundedVoxelMap & operator=(const BoundedVoxelMap &)=delete;
  bool add(const Point & p) {
    const double x=std::floor(double(p.x)/resolution_),y=std::floor(double(p.y)/resolution_),z=std::floor(double(p.z)/resolution_);
    constexpr double limit=4.0e18;
    if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||
      std::abs(x)>limit||std::abs(y)>limit||std::abs(z)>limit)return false;
    const Key key{static_cast<int64_t>(x),static_cast<int64_t>(y),static_cast<int64_t>(z)};
    auto it=cells_.find(key);
    if(it!=cells_.end()) {
      it->second.point=p;order_.splice(order_.end(),order_,it->second.order);return false;
    }
    if(cells_.size()==capacity_) {cells_.erase(order_.front());order_.pop_front();}
    order_.push_back(key);cells_.emplace(key,Entry{p,std::prev(order_.end())});return true;
  }
  std::size_t size() const {return cells_.size();}
  template<class Cloud> void snapshot(Cloud & cloud) const {
    cloud.clear();cloud.reserve(cells_.size());
    for(const auto & key:order_)cloud.push_back(cells_.at(key).point);
  }
};
}
