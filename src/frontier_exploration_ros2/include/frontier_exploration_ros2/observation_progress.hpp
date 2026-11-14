#pragma once
#include "frontier_exploration_ros2/frontier_types.hpp"
#include <map>
#include <set>
#include <queue>
#include <limits>

namespace frontier_exploration_ros2
{
// Navigation success and information gain are different outcomes. Remember
// previously observed cells so unknown/known flicker cannot count as new gain.
class ObservationProgress
{
public:
  using Point=std::pair<double,double>;
  struct Snapshot
  {
    Point reference,observation;
    double cell_area, floor_z;
    std::vector<Point> unknown;
  };
  struct Outcome {double revealed_area; unsigned no_gain_visits; bool cooling; bool route_progress{false};};

  double revealedArea(const Snapshot & snapshot,const OccupancyGrid2d & map) const {
    return revealed(snapshot,map);
  }
  bool routeProgress(const Snapshot & snapshot,const OccupancyGrid2d & map) const {
    for(const auto & item:visits_) {
      const auto & visit=item.second;
      if(distance(visit.reference,snapshot.reference)>1.5 &&
        std::none_of(visit.positions.begin(),visit.positions.end(),
          [&](Point p){return distance(p,snapshot.observation)<=.60;}))continue;
      if(std::any_of(visit.positions.begin(),visit.positions.end(),
        [&](Point p){return distance(p,snapshot.observation)<=.35;}))return false;
      if(visit.positions.empty() && distance(visit.observation,snapshot.observation)<=.35)return false;
      const auto remaining=mappedRemainingDistance(map,snapshot.observation,visit.reference);
      return remaining && std::isfinite(visit.best_remaining) &&
        *remaining<=visit.best_remaining-.25+1e-9;
    }
    return false;
  }
  void primeRoute(const OccupancyGrid2d & map,Point reference,Point actual,int64_t now) {
    checkFloor(map);
    auto & visit=record(reference,actual,now);
    if(auto remaining=mappedRemainingDistance(map,actual,reference))
      visit.best_remaining=std::min(visit.best_remaining,*remaining);
  }
  void blockObservation(Point point,int64_t now) {
    manual_blocks_.push_back({point,now+60'000'000'000LL});
    if(manual_blocks_.size()>64)manual_blocks_.erase(manual_blocks_.begin());
  }

  Snapshot capture(const OccupancyGrid2d & map,Point reference,Point observation)
  {
    checkFloor(map);
    rememberKnown(map);
    Snapshot result{reference,observation,std::pow(map.map().info.resolution,2),map.map().info.origin.position.z,{}};
    for (int y=0;y<map.getSizeY();++y) {
      for (int x=0;x<map.getSizeX();++x) {
        const auto p=map.mapToWorld(x,y);
        if (map.getCost(x,y)<0 && !known_.count(key(p)) &&
          (distance(p,reference)<=2.0 || distance(p,observation)<=2.0))
        {result.unknown.push_back(p);}
      }
    }
    return result;
  }

  Outcome complete(const Snapshot & snapshot,const OccupancyGrid2d & map,int64_t now)
  {
    checkFloor(map);
    if(std::abs(snapshot.floor_z-map.map().info.origin.position.z)>0.5) {return {0.0,0,false};}
    const double area=revealed(snapshot,map);
    rememberKnown(map);
    auto & visit=record(snapshot.reference,snapshot.observation,now);
    const bool new_position=std::none_of(visit.positions.begin(),visit.positions.end(),
      [&](Point p){return distance(p,snapshot.observation)<=0.35;});
    if(new_position) {
      if(visit.positions.size()>=32) {visit.positions.erase(visit.positions.begin());}
      visit.positions.push_back(snapshot.observation);
    }
    const double displacement=distance(visit.first_observation,snapshot.observation);
    const auto remaining=mappedRemainingDistance(map,snapshot.observation,visit.reference);
    const bool route_progress=new_position && (remaining ?
      (std::isfinite(visit.best_remaining) && *remaining<=visit.best_remaining-.25) :
      (displacement>=visit.furthest_displacement+.50-1e-9 ||
       distance(visit.observation,visit.reference)-distance(snapshot.observation,visit.reference)>=.50-1e-9));
    if(remaining)visit.best_remaining=std::min(visit.best_remaining,*remaining);
    visit.furthest_displacement=std::max(visit.furthest_displacement,displacement);
    visit.observation=snapshot.observation;
    visit.snapshot=snapshot; visit.updated=now;
    if (area>=gainThreshold(snapshot)-1e-9) {
      visit.strikes=0;visit.until=0;visit.bad_positions.clear();visit.blocked.clear();
      if(remaining)visit.best_remaining=*remaining;
    } else if(route_progress) {
      visit.strikes=1;visit.until=0;visit.bad_positions.clear();
    } else {
      visit.strikes=std::min(100U,visit.strikes+1);
      if(std::none_of(visit.bad_positions.begin(),visit.bad_positions.end(),
        [&](Point p){return distance(p,snapshot.observation)<=.35;})) {
        if(visit.bad_positions.size()>=32)visit.bad_positions.erase(visit.bad_positions.begin());
        visit.bad_positions.push_back(snapshot.observation);
      }
      if(visit.strikes>=2) {
        visit.until=now+600'000'000'000LL;
        for(const auto & point:visit.bad_positions) {
          auto old=std::find_if(visit.blocked.begin(),visit.blocked.end(),
            [&](const auto & v){return distance(v.first,point)<=.35;});
          if(old!=visit.blocked.end())old->second=visit.until;
          else {
            if(visit.blocked.size()>=64)visit.blocked.erase(visit.blocked.begin());
            visit.blocked.push_back({point,visit.until});
          }
        }
      }
    }
    return {area,visit.strikes,blockedObservation(snapshot.observation,now),route_progress};
  }

  void refresh(const OccupancyGrid2d & map,int64_t now)
  {
    checkFloor(map);
    for (auto it=visits_.begin();it!=visits_.end();) {
      auto & visit=it->second;
      if (now-visit.updated>1'200'000'000'000LL && visit.until<=now) {it=visits_.erase(it);continue;}
      const bool blocked=std::any_of(visit.blocked.begin(),visit.blocked.end(),
        [&](const auto & entry){return entry.second>now;});
      if ((visit.until>now || blocked) && revealed(visit.snapshot,map)>=gainThreshold(visit.snapshot)-1e-9) {
        visit.strikes=0;visit.until=0;visit.blocked.clear();visit.bad_positions.clear();
      }
      ++it;
    }
  }
  bool blockedReference(Point point,int64_t now) const
  {
    for (const auto & item:visits_) {
      if (item.second.until>now && distance(point,item.second.reference)<=1.50) {return true;}
    }
    return false;
  }
  bool blockedObservation(Point point,int64_t now) const
  {
    for(const auto & blocked:manual_blocks_)
      if(blocked.second>now && distance(point,blocked.first)<=.35)return true;
    for (const auto & item:visits_) {
      for(const auto & blocked:item.second.blocked)
        if(blocked.second>now && distance(point,blocked.first)<=0.60)return true;
    }
    return false;
  }
  std::vector<Point> blockedObservationCenters(int64_t now) const
  {
    std::vector<Point> result;
    for(const auto & blocked:manual_blocks_)if(blocked.second>now)result.push_back(blocked.first);
    for(const auto & item:visits_)
      for(const auto & blocked:item.second.blocked)
        if(blocked.second>now)result.push_back(blocked.first);
    return result;
  }
  void clear() {known_.clear();manual_blocks_.clear();visits_.clear();floor_.reset();next_id_=0;}

private:
  struct Visit
  {
    Point reference{},observation{},first_observation{};
    double furthest_displacement{0},best_remaining{std::numeric_limits<double>::infinity()};
    std::vector<Point> bad_positions;
    std::vector<std::pair<Point,int64_t>> blocked;
    Snapshot snapshot{};
    std::vector<Point> positions;
    unsigned strikes{0};
    int64_t until{0},updated{0};
  };
  static double distance(Point a,Point b) {return std::hypot(a.first-b.first,a.second-b.second);}
  static std::pair<int64_t,int64_t> key(Point p)
  {return {std::llround(p.first/.05),std::llround(p.second/.05)};}
  static double gainThreshold(const Snapshot & s)
  {return s.cell_area;}

  // Remaining known-space grid distance, rather than distance travelled away
  // from the first view. This lets a real detour advance while rejecting circuits.
  static std::optional<double> mappedRemainingDistance(
    const OccupancyGrid2d & map,Point from,Point reference)
  {
    int sx,sy;
    if(!map.worldToMapNoThrow(from.first,from.second,sx,sy) || map.getCost(sx,sy)!=0)
      return std::nullopt;
    const int w=map.getSizeX(),h=map.getSizeY();
    if(static_cast<size_t>(w)*h>200000)return std::nullopt;
    std::vector<int> distances(static_cast<size_t>(w)*h,-1);
    std::queue<int> open;distances[sy*w+sx]=0;open.push(sy*w+sx);
    double closest=std::numeric_limits<double>::infinity(),remaining=closest;
    while(!open.empty()) {
      const int i=open.front();open.pop();
      const auto point=map.mapToWorld(i%w,i/w);
      const double approach=distance(point,reference);
      if(approach<closest-1e-8) {closest=approach;remaining=distances[i]*map.map().info.resolution;}
      else if(std::abs(approach-closest)<1e-8)
        remaining=std::min(remaining,distances[i]*static_cast<double>(map.map().info.resolution));
      for(auto delta:{std::pair<int,int>{-1,0},{1,0},{0,-1},{0,1}}) {
        const int x=i%w+delta.first,y=i/w+delta.second;
        if(x<0 || x>=w || y<0 || y>=h || distances[y*w+x]>=0 || map.getCost(x,y)!=0)continue;
        distances[y*w+x]=distances[i]+1;open.push(y*w+x);
      }
    }
    if(!std::isfinite(remaining))return std::nullopt;
    return remaining;
  }
  static double revealed(const Snapshot & s,const OccupancyGrid2d & map)
  {
    std::size_t cells=0;
    for (const auto & p:s.unknown) {
      int x,y;
      if(map.worldToMapNoThrow(p.first,p.second,x,y) && map.getCost(x,y)>=0) {++cells;}
    }
    return cells*s.cell_area;
  }
  void rememberKnown(const OccupancyGrid2d & map)
  {
    for (int y=0;y<map.getSizeY();++y) {
      for (int x=0;x<map.getSizeX();++x) {
        if(map.getCost(x,y)>=0) {known_.insert(key(map.mapToWorld(x,y)));}
      }
    }
  }
  void checkFloor(const OccupancyGrid2d & map)
  {
    const double floor=map.map().info.origin.position.z;
    if(floor_ && std::abs(floor-*floor_)>0.5) {clear();}
    floor_=floor;
  }
  Visit & record(Point reference,Point observation,int64_t now)
  {
    for(auto & item:visits_) {if(distance(item.second.reference,reference)<=1.50 ||
      std::any_of(item.second.positions.begin(),item.second.positions.end(),
        [&](Point p){return distance(p,observation)<=.60;})) {return item.second;}}
    if(visits_.size()>=128) {
      auto oldest=std::min_element(visits_.begin(),visits_.end(),[](const auto & a,const auto & b){return a.second.updated<b.second.updated;});
      visits_.erase(oldest);
    }
    auto & visit=visits_[next_id_++];visit.updated=now;
    visit.reference=reference;visit.observation=observation;visit.first_observation=observation;
    return visit;
  }
  std::vector<std::pair<Point,int64_t>> manual_blocks_;
  std::set<std::pair<int64_t,int64_t>> known_;
  std::map<unsigned,Visit> visits_;
  unsigned next_id_{0};
  std::optional<double> floor_;
};
}  // namespace frontier_exploration_ros2
