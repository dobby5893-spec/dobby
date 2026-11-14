#pragma once
#include "frontier_exploration_ros2/mapped_observation.hpp"
#include <deque>
#include <set>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <random>
#include <stdexcept>

namespace frontier_exploration_ros2 {
// Counts measured stagnation across ordinary goal replacements. Sending a
// goal is not evidence that the robot moved. Used only in a running session.
struct MotionStallGuard {
  double radius{.15}, timeout{60};
  double anchor_time{-1}, last_time{-1}, anchor_x{0}, anchor_y{0}, x{0}, y{0};
  unsigned failures{0};
  bool armed{false};
  void observe(double now,double px,double py) {
    if(!std::isfinite(now+px+py))return;
    if(last_time<0 || now<last_time) {
      anchor_time=now;anchor_x=px;anchor_y=py;failures=0;armed=false;
    }
    x=px;y=py;last_time=now;
    if(std::hypot(x-anchor_x,y-anchor_y)>=radius) {
      anchor_time=now;anchor_x=x;anchor_y=y;failures=0;
    }
  }
  void arm() {
    if(!armed) {armed=true;anchor_time=last_time;}
  }
  void failedGoal(double now,double px,double py) {
    observe(now,px,py);arm();++failures;
  }
  bool ready(double now) const {
    return armed && anchor_time>=0 && now>=last_time &&
      (failures>=2 || now-anchor_time>=timeout);
  }
  void restart(double now) {
    anchor_time=now;anchor_x=x;anchor_y=y;failures=0;
  }
};

struct KnownMapProgress {
  std::set<std::pair<int64_t,int64_t>> known;
  double floor_z{0},last_time{-1},area{0};
  double observe(const nav_msgs::msg::OccupancyGrid & map,double now) {
    const double z=map.info.origin.position.z;
    if(last_time<0 || now<last_time || std::abs(z-floor_z)>.5) {
      known.clear();area=0;
    }
    floor_z=z;last_time=now;
    const auto & q=map.info.origin.orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const double c=std::cos(yaw),s=std::sin(yaw),res=map.info.resolution;
    if(res<=0 || map.data.size()!=size_t(map.info.width)*map.info.height)return area;
    for(size_t i=0;i<map.data.size();++i)if(map.data[i]>=0) {
      double x=(i%map.info.width+.5)*res,y=(i/map.info.width+.5)*res;
      auto key=std::make_pair(std::llround((map.info.origin.position.x+c*x-s*y)/.05),
        std::llround((map.info.origin.position.y+s*x+c*y)/.05));
      if(known.insert(key).second)area+=res*res;
    }
    return area;
  }
};

struct RecoveryPolicy {
  MotionStallGuard motion;
  double blocked_timeout{30},window{90},cooldown{30},batch_cooldown{120};
  int max_attempts{3},attempts{0};
  unsigned batches{0};
  bool episode{false};
  double episode_x{0},episode_y{0};
  bool displaced(double x,double y) const {
    return episode && std::isfinite(x+y) && std::hypot(x-episode_x,y-episode_y)>=.5;
  }
  // Only a measured, supported collision-free position can end the episode.
  bool recovered(double now,double x,double y,bool safe) {
    if(!safe || !displaced(x,y))return false;
    attempts=0;batches=0;episode=false;blocked_since=-1;last_attempt=-1e9;
    samples.clear();no_gain_since=now;motion=MotionStallGuard{};
    motion.observe(now,x,y);return true;
  }
  struct Sample {double time,x,y;};
  std::deque<Sample> samples;
  double last_time{-1},known_peak{-1},no_gain_since{0},blocked_since{-1},last_attempt{-1e9};
  void observe(double now,double x,double y,double area) {
    if(last_time<0 || now<last_time) {
      samples.clear();known_peak=area;no_gain_since=now;blocked_since=-1;
      attempts=0;batches=0;episode=false;last_attempt=-1e9;motion=MotionStallGuard{};
    }
    last_time=now;motion.observe(now,x,y);
    if(area>=known_peak+.1) {
      known_peak=area;no_gain_since=now;
    }
    if(samples.empty() || std::hypot(x-samples.back().x,y-samples.back().y)>=.05 ||
       now-samples.back().time>=5) samples.push_back({now,x,y});
    while(samples.size()>1 && samples[1].time<now-window) samples.pop_front();
  }
  void blocked(double now) {if(blocked_since<0)blocked_since=now;}
  void normalDispatch() {blocked_since=-1;motion.arm();}
  bool looping(double now) const {
    if(samples.size()<3 || now-no_gain_since<blocked_timeout ||
       samples.back().time-samples.front().time<blocked_timeout) return false;
    if(samples.size()<4)return false;
    double length=0;
    for(size_t i=samples.size()-1;i>0;--i) {
      length+=std::hypot(samples[i].x-samples[i-1].x,samples[i].y-samples[i-1].y);
      if(length>=1.0 && now-samples[i-1].time>=blocked_timeout &&
        std::hypot(samples.back().x-samples[i-1].x,
          samples.back().y-samples[i-1].y)<.35)return true;
    }
    return false;
  }
  bool ready(double now) {
    if(now-last_attempt<cooldown) return false;
    if(attempts>=max_attempts) {
      if(now-last_attempt<batch_cooldown)return false;
      attempts=0;++batches;
    }
    return (blocked_since>=0 && now-blocked_since>=blocked_timeout) || looping(now) || motion.ready(now);
  }
  void begin(double now) {
    if(!episode) {episode=true;episode_x=motion.x;episode_y=motion.y;}
    last_attempt=now;++attempts;blocked_since=now;motion.restart(now);
  }
};

struct RandomRecoveryConfig {
  double minimum_distance{.5},initial_radius{1.5},expanded_radius{3},avoid_radius{.5};
  unsigned samples_per_ring{48};
  void validate() const {
    if(!std::isfinite(minimum_distance) || !std::isfinite(initial_radius) ||
       !std::isfinite(expanded_radius) || !std::isfinite(avoid_radius) ||
       minimum_distance<.5 || initial_radius<=minimum_distance ||
       expanded_radius<=initial_radius || expanded_radius>10 ||
       avoid_radius<0 || samples_per_ring<1 || samples_per_ring>256)
      throw std::invalid_argument("Invalid bounded random recovery configuration");
  }
};

inline std::optional<go2_surface_3d_planner::SurfacePoint> selectRandomRecovery(
  const octomap::OcTree & tree,go2_surface_3d_planner::SurfaceSearchConfig config,
  const go2_surface_3d_planner::SurfacePoint & start,const ObservationGrid & grid,
  const RandomRecoveryConfig & sampling,
  const std::vector<go2_surface_3d_planner::SurfacePoint> & avoid,uint32_t seed,
  const ExplorationBoundaryPolygon & boundary = {},double boundary_margin = 0.)
{
  using namespace go2_surface_3d_planner;
  sampling.validate();
  if(config.search_deadline==std::chrono::steady_clock::time_point::max())
    config.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
  if(config.infer_flat_support && !configureSupportPlane(config,tree,start))return std::nullopt;
  auto stance=start;if(config.infer_flat_support)stance.z-=config.support_reference_height;
  const auto origin=SurfaceGraph(tree,config).snapToSurface(stance);
  if(!origin)return std::nullopt;
  std::mt19937 rng(seed);std::uniform_real_distribution<double> unit(0,1);
  const double margin=config.narrow_clearance_margin>0?
    std::max(.01,config.narrow_clearance_margin):config.clearance_margin;
  auto endpoint_config=config;endpoint_config.clearance_margin=margin;
  SurfaceGraph endpoints(tree,endpoint_config);
  const auto measured=endpoints.diagnosePose(stance);
  if(!measured.surface || !measured.support || !measured.clearance)return std::nullopt;
  const double c=std::cos(grid.yaw),s=std::sin(grid.yaw);
  auto value=[&](double x,double y) {
    if(grid.resolution<=0)return -2;
    const double dx=x-grid.x,dy=y-grid.y;
    const int ix=std::floor((c*dx+s*dy)/grid.resolution);
    const int iy=std::floor((-s*dx+c*dy)/grid.resolution);
    if(ix<0 || iy<0 || ix>=grid.width || iy>=grid.height ||
      grid.cells.size()!=static_cast<size_t>(grid.width)*grid.height)return -2;
    return static_cast<int>(grid.cells[iy*grid.width+ix]);
  };
  struct Candidate {SurfacePoint pose;double score;};
  for(auto ring:std::vector<std::pair<double,double>>{
      {sampling.minimum_distance,sampling.initial_radius},
      {sampling.initial_radius,sampling.expanded_radius}}) {
    const auto began=std::chrono::steady_clock::now();
    const auto ring_deadline=ring.first==sampling.minimum_distance?
      began+(config.search_deadline-began)/2:config.search_deadline;
    std::vector<Candidate> candidates;
    for(unsigned i=0;i<sampling.samples_per_ring;++i) {
      if(std::chrono::steady_clock::now()>=ring_deadline)break;
      const double r=std::sqrt(ring.first*ring.first+
        unit(rng)*(ring.second*ring.second-ring.first*ring.first));
      const double angle=unit(rng)*2*M_PI;
      SurfacePoint p{start.x+r*std::cos(angle),start.y+r*std::sin(angle),origin->z,angle};
      if(!is_inside_exploration_boundary({p.x,p.y},boundary,boundary_margin) ||
         value(p.x,p.y)<0 || value(p.x,p.y)>=90)continue;
      const auto snapped=endpoints.snapToSurface(p);
      if(!snapped)continue;
      p=*snapped;
      if(!is_inside_exploration_boundary({p.x,p.y},boundary,boundary_margin) ||
         value(p.x,p.y)<0 || value(p.x,p.y)>=90)continue;
      const double distance=std::hypot(p.x-start.x,p.y-start.y);
      if(distance<ring.first || distance>ring.second ||
         std::abs(p.z-origin->z)>.12)continue;
      if(std::any_of(avoid.begin(),avoid.end(),[&](const auto & old) {
          return std::abs(old.z-p.z)<.3 &&
            std::hypot(old.x-p.x,old.y-p.y)<sampling.avoid_radius;}))continue;
      // Unknown cells near an otherwise safe viewpoint are a gain proxy,
      // not proof that a ray can see through walls or reveal new map cells.
      double gain=0;
      for(double a=0;a<2*M_PI;a+=M_PI/12)
        if(value(p.x+std::cos(a),p.y+std::sin(a))==-1)gain+=1;
      candidates.push_back({p,gain+std::min(.5,endpoints.bodyClearanceAt(p))+unit(rng)*.1});
    }
    std::sort(candidates.begin(),candidates.end(),
      [](const auto & a,const auto & b){return a.score>b.score;});
    for(const auto & candidate:candidates) {
      if(std::chrono::steady_clock::now()>=ring_deadline)break;
      // Reject disconnected and long roundabout routes before a 3D search.
      const auto coarse=mappedObservationRoute(grid,stance,candidate.pose,
        config.collision_radius+margin,ring_deadline,true);
      if(coarse.empty())continue;
      double coarse_length=0;
      for(size_t i=1;i<coarse.size();++i)
        coarse_length+=std::hypot(coarse[i].x-coarse[i-1].x,coarse[i].y-coarse[i-1].y);
      if(coarse_length>2*ring.second+1)continue;
      for(double yaw:{candidate.pose.yaw,start.yaw,candidate.pose.yaw+M_PI}) {
        auto goal=candidate.pose;goal.yaw=yaw;auto attempt=config;
        attempt.search_deadline=std::min(ring_deadline,
          std::chrono::steady_clock::now()+std::chrono::milliseconds(200));
        try {
          const auto path=SurfaceGraph(tree,attempt).plan(stance,goal);
          if(path.empty() || !boundary_return_path(stance,path,boundary,boundary_margin))continue;
          const auto & end=path.back();const double distance=std::hypot(end.x-start.x,end.y-start.y);
          if(distance<ring.first || distance>ring.second)continue;
          if(std::any_of(path.begin(),path.end(),[&](const auto & p) {
              return std::abs(p.z-origin->z)>.12;}))continue;
          if(std::any_of(avoid.begin(),avoid.end(),[&](const auto & p) {
              return std::abs(end.z-p.z)<.3 &&
                std::hypot(end.x-p.x,end.y-p.y)<sampling.avoid_radius;}))continue;
          return end;
        } catch(const SurfaceSearchTimeout &) {}
      }
    }
  }
  return std::nullopt;
}
}
