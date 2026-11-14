#pragma once
#include <cmath>
#include <algorithm>
namespace frontier_exploration_ros2 {
// Simulated idle time without verified movement. Accepting a goal pauses
// counting but cannot forgive an immediately failed execution. Missing data
// and active navigation pause time; real translation or a new floor resets it.
class NoNextGoalWatchdog {
public:
 double timeout{300.};
 void clear() {elapsed_=0;last_=-1;counting_=false;floor_set_=false;retry_at_=-1;anchor_set_=false;}
 void pause(double now) {
  if(last_>=0 && now<last_)clear();
  last_=now;counting_=false;
 }
 void observe(double now,double floor) {
  if(!std::isfinite(now+floor))return;
  if((last_>=0 && now<last_) || (floor_set_ && std::abs(floor-floor_)>.5))clear();
  if(last_>=0 && counting_)elapsed_+=std::max(0.,now-last_);
  last_=now;floor_=floor;floor_set_=true;counting_=true;
 }
 void accepted(double now) {pause(now);}
 void position(double now,double x,double y) {
  if(!std::isfinite(now+x+y))return;
  if(!anchor_set_){anchor_x_=x;anchor_y_=y;anchor_set_=true;return;}
  if(std::hypot(x-anchor_x_,y-anchor_y_)<.20)return;
  elapsed_=0;retry_at_=-1;last_=now;
  anchor_x_=x;anchor_y_=y;
 }
 bool due(double now) const {
  return counting_ && elapsed_>=timeout-1e-6 && now>=retry_at_;
 }
 void attempted(double now) {retry_at_=now+30.;}
 double seconds() const {return elapsed_;}
private:
 double elapsed_{0},last_{-1},floor_{0},retry_at_{-1};
 double anchor_x_{0},anchor_y_{0};
 bool anchor_set_{false},counting_{false},floor_set_{false};
};
}
