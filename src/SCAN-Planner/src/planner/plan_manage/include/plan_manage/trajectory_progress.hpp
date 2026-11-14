#pragma once

#include <algorithm>
#include <cmath>
#include <Eigen/Core>

namespace scan_planner
{
// Follow the nearby portion of a trajectory in order. Time is only its
// parameter here: waiting at the same physical position cannot advance it.
template<class Evaluate>
double projectProgress(Evaluate evaluate, const Eigen::Vector3d & actual,
                       double previous, double duration, double search_arc = 0.5)
{
  double best_t = std::clamp(previous, 0.0, duration);
  auto last = evaluate(best_t).eval();
  double best_distance = (last - actual).template head<2>().norm();
  double arc = 0.0;
  for (double t = best_t; t < duration;) {
    t = std::min(duration, t + 0.05);
    const auto point = evaluate(t).eval();
    arc += (point - last).template head<2>().norm();
    if (arc > search_arc) break;
    const double distance = (point - actual).template head<2>().norm();
    if (distance + 1e-6 < best_distance) {
      best_distance = distance;
      best_t = t;
    } else if (distance > best_distance + 0.02) {
      break;  // Do not jump to a later branch at a crossing or hairpin.
    }
    last = point;
  }
  return best_t;
}

template<class Evaluate>
double nearbyReference(Evaluate evaluate, double progress, double duration,
                       double time_ahead, double max_arc = 0.06)
{
  auto last = evaluate(progress).eval();
  double arc = 0.0;
  double result = progress;
  const double end = std::min(duration, progress + std::max(0.0, time_ahead));
  for (double t = progress; t < end;) {
    t = std::min(end, t + 0.02);
    const auto point = evaluate(t).eval();
    arc += (point - last).template head<2>().norm();
    if (arc > max_arc) break;
    result = t;
    last = point;
  }
  return result;
}

inline bool reachedActualGoal(const Eigen::Vector3d & actual,
                              const Eigen::Vector3d & goal, double tolerance)
{
  return (actual - goal).norm() <= tolerance;
}
}  // namespace scan_planner
