#pragma once
#include <cstdint>
#include <optional>
namespace frontier_exploration_ros2 {
class LocalizationFreshness {
public:
  enum class State {Healthy,Waiting,Expired};
  static bool accepts(int64_t now,int64_t stamp) {
    return stamp>=0 && stamp<=now+100'000'000LL && now-stamp<=2'000'000'000LL;
  }
  void reset(int64_t now) {started_=last_clock_=now;last_fresh_.reset();}
  State observe(int64_t now,std::optional<int64_t> stamp) {
    if(now<last_clock_)reset(now);
    last_clock_=now;
    if(stamp && accepts(now,*stamp)) {last_fresh_=now;return State::Healthy;}
    const bool expired=last_fresh_ ? now-*last_fresh_>=5'000'000'000LL : now-started_>=30'000'000'000LL;
    return expired?State::Expired:State::Waiting;
  }
private:
  int64_t started_{0},last_clock_{0};
  std::optional<int64_t> last_fresh_;
};
}
