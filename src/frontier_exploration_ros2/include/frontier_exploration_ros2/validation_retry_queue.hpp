#pragma once

#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <optional>
#include <string>

namespace frontier_exploration_ros2
{
// One worker, FIFO candidate service, and per-candidate backoff. A timed-out
// search is deferred, never classified as a permanently unreachable frontier.
class ValidationRetryQueue
{
public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  struct Job {std::string key; double budget; unsigned attempt;};

  explicit ValidationRetryQueue(double base_budget = 5.0, double max_budget = 10.0)
  : base_budget_(base_budget), max_budget_(std::max(base_budget, max_budget)) {}

  void enqueue(const std::string & key, Time now)
  {
    auto & entry = entries_[key];
    entry.last_seen = now;
    if (active_ == key || entry.queued) {return;}
    entry.queued = true;
    queue_.push_back(key);
  }

  std::optional<Job> next(Time now)
  {
    if (!active_.empty()) {return std::nullopt;}
    const auto count = queue_.size();
    for (std::size_t i = 0; i < count; ++i) {
      auto key = queue_.front(); queue_.pop_front();
      auto & entry = entries_.at(key);
      if (now - entry.last_seen > std::chrono::seconds(30)) {
        entry.queued = false; continue;
      }
      if (now < entry.retry_at) {queue_.push_back(key); continue;}
      entry.queued = false; active_ = key;
      const double multiplier = entry.timeouts == 0 ? 1.0 : entry.timeouts == 1 ? 1.5 : 2.0;
      return Job{key, std::min(max_budget_, base_budget_ * multiplier), entry.timeouts + entry.rejections + 1};
    }
    return std::nullopt;
  }

  double finish(const std::string & key, bool timed_out, Time now, bool verified = true)
  {
    if (active_ != key) {return 0.0;}
    active_.clear();
    auto & entry = entries_.at(key);
    double delay = 3.0;
    if (timed_out) {
      entry.timeouts = std::min(100U, entry.timeouts + 1);
      delay = entry.timeouts == 1 ? 10.0 : entry.timeouts == 2 ? 30.0 : 120.0;
    } else if(!verified) {
      entry.timeouts=0;entry.rejections=std::min(100U,entry.rejections+1);
      delay=entry.rejections==1?5.:entry.rejections==2?15.:entry.rejections==3?60.:120.;
    } else {entry.timeouts = 0;entry.rejections=0;}
    entry.retry_at = now + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(delay));
    return delay;
  }

  bool active() const {return !active_.empty();}
  bool hasReady(Time now) const
  {
    for (const auto & key : queue_) {
      const auto & entry = entries_.at(key);
      if (now >= entry.retry_at && now - entry.last_seen <= std::chrono::seconds(30)) {return true;}
    }
    return false;
  }
  void clear() {queue_.clear(); entries_.clear(); active_.clear();}

private:
  struct Entry
  {
    unsigned timeouts{0},rejections{0};
    bool queued{false};
    Time retry_at{Time::min()};
    Time last_seen{};
  };
  double base_budget_, max_budget_;
  std::deque<std::string> queue_;
  std::map<std::string, Entry> entries_;
  std::string active_;
};
}  // namespace frontier_exploration_ros2
