#ifndef SCAN_PLANNER_NAV2_CONTROLLER__POSE_PATH_TRACKER_HPP_
#define SCAN_PLANNER_NAV2_CONTROLLER__POSE_PATH_TRACKER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace scan_planner_nav2_controller
{
struct PoseReference {double x, y, z, yaw;};
struct PoseCommand {double vx{0.0}, vy{0.0}, wz{0.0};};

class PosePathTracker
{
public:
  // A grid connector must not demand centimetre-perfect settling from the gait.
  // The controller still checks the measured full-body sweep before every command.
  static constexpr double checkpoint_tolerance = 0.05;
  static bool positionReached(const PoseReference & actual, const PoseReference & target)
  {
    // The grid connector has independent x/y errors. Combining them into a
    // radial gate can trap the gait just outside the checkpoint while heading
    // interpolation prevents completion of the planned turn.
    return std::max(std::abs(actual.x-target.x), std::abs(actual.y-target.y)) <=
      checkpoint_tolerance;
  }
  static double angle(double value) {return std::remainder(value, 2.0 * 3.14159265358979323846);}
  void setPlan(const std::vector<PoseReference> & path)
  {
    if (path.empty()) {throw std::invalid_argument("Empty pose path");}
    bool same = path.size() == path_.size();
    for (std::size_t i = 0; same && i < path.size(); ++i) {
      const auto & a = path[i]; const auto & b = path_[i];
      same = std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z) +
        std::abs(angle(a.yaw - b.yaw)) < 1e-8;
    }
    if (same) {return;}
    for (const auto & pose : path) {
      if (!std::isfinite(pose.x + pose.y + pose.z + pose.yaw)) {
        throw std::invalid_argument("Non-finite pose path");
      }
    }
    path_ = path; index_ = 0; have_height_offset_ = false;turn_watch_active_=false;
  }

  PoseCommand command(const PoseReference & measured, double max_speed,
    double max_yaw_speed, double prediction_time)
  {
    if (path_.empty()) {throw std::runtime_error("Waiting for a verified pose path");}
    if (!have_height_offset_) {
      if (std::hypot(measured.x - path_.front().x, measured.y - path_.front().y) > 0.25) {
        throw std::runtime_error("Pose path start is too far from measured robot position");
      }
      height_offset_ = measured.z - path_.front().z;
      have_height_offset_ = true;
    }
    PoseReference actual = measured; actual.z -= height_offset_;
    while (index_ < path_.size() &&
      positionReached(actual, path_[index_]) &&
      std::abs(actual.z - path_[index_].z) <= 0.15 &&
      std::abs(angle(actual.yaw - path_[index_].yaw)) <= 0.035)
    {++index_;}
    if (index_ >= path_.size()) {return {};}

    // Merge only collinear, horizontal, fixed-heading edges. Door turns,
    // stair height changes, and in-place rotations remain explicit stages.
    while (index_ > 0 && index_ + 1 < path_.size()) {
      const auto & a = path_[index_ - 1];
      const auto & b = path_[index_];
      const auto & c = path_[index_ + 1];
      const double ux = b.x - a.x, uy = b.y - a.y;
      const double vx = c.x - b.x, vy = c.y - b.y;
      const double lengths = std::hypot(ux, uy) * std::hypot(vx, vy);
      if (lengths < 1e-9 || ux * vx + uy * vy < lengths * 0.999999 ||
        std::abs(a.z - b.z) + std::abs(c.z - b.z) > 1e-8 ||
        std::abs(angle(a.yaw - b.yaw)) + std::abs(angle(c.yaw - b.yaw)) > 1e-8)
      {break;}
      ++index_;
    }
    const auto & target = path_[index_];
    const auto & previous = path_[index_ == 0 ? 0 : index_ - 1];
    const double ex = target.x - actual.x, ey = target.y - actual.y;
    const double distance = std::hypot(ex, ey);
    const double sx = target.x - previous.x, sy = target.y - previous.y;
    const double squared = sx * sx + sy * sy;
    const double progress = squared > 1e-9 ? std::clamp(
      ((actual.x - previous.x) * sx + (actual.y - previous.y) * sy) / squared, 0.0, 1.0) : 1.0;
    // Heading comes from the planned poses, including sideways travel.
    double desired_yaw = previous.yaw + progress * angle(target.yaw - previous.yaw);
    const bool position_reached = positionReached(actual, target);
    if (position_reached || squared < 1e-9) {desired_yaw = target.yaw;}
    const double yaw_error = angle(desired_yaw - actual.yaw);
    const double horizon = std::max(0.1, prediction_time);
    PoseCommand result;
    double yaw_feedforward = 0.0;
    const bool hold_stance = position_reached || squared < 1e-9;
    if (distance > 1e-9 && (hold_stance || std::abs(yaw_error) <= 0.10)) {
      // The quadruped gait drifts during rotation. Correct position throughout
      // a turn instead of letting that drift consume the doorway clearance.
      const double speed = std::min({std::max(0.0, max_speed),
        hold_stance ? 0.06 : max_speed, 2.0 * distance,
        std::sqrt(2.0 * 0.16 * distance), distance / horizon});
      const double wx = speed * ex / distance, wy = speed * ey / distance;
      if (!hold_stance && squared > 1e-9) {
        // Follow the change in planned heading while traversing a curved edge,
        // even when measured heading exactly matches its current reference.
        yaw_feedforward = angle(target.yaw - previous.yaw) *
          (wx * sx + wy * sy) / squared;
      }
      result.vx = std::cos(actual.yaw) * wx + std::sin(actual.yaw) * wy;
      result.vy = -std::sin(actual.yaw) * wx + std::cos(actual.yaw) * wy;
    }
    result.wz = std::clamp(2.0 * yaw_error + yaw_feedforward,
      -max_yaw_speed, max_yaw_speed);
    // Cap motion toward the endpoint heading. A curved edge can reach that
    // heading before its position: it must still be able to turn back toward
    // the current interpolated heading rather than locking its yaw there.
    const double endpoint_error=angle(target.yaw-actual.yaw);
    if(result.wz*endpoint_error>0) {
      const double remaining_yaw=std::abs(endpoint_error);
      result.wz=std::clamp(result.wz,-remaining_yaw/horizon,remaining_yaw/horizon);
    }
    return result;
  }
  // Finite progress on a rotation stage: periodic estimator jitter cannot
  // repeatedly reset the timer because the best target error is retained.
  bool turnStalled(const PoseReference & measured,double now,double timeout=8.0)
  {
    if(!std::isfinite(now) || index_>=path_.size()) {turn_watch_active_=false;return false;}
    const auto & target=path_[index_];
    const bool explicit_turn=index_>0 &&
      std::hypot(target.x-path_[index_-1].x,target.y-path_[index_-1].y)<1e-8 &&
      std::abs(angle(target.yaw-path_[index_-1].yaw))>1e-8;
    if(!explicit_turn && !positionReached(measured,target)) {
      turn_watch_active_=false;return false;
    }
    const double error=std::abs(angle(target.yaw-measured.yaw));
    const double distance=std::hypot(target.x-measured.x,target.y-measured.y);
    if(!turn_watch_active_ || turn_watch_index_!=index_ || now<turn_watch_clock_) {
      turn_watch_active_=true;turn_watch_index_=index_;
      best_turn_error_=error;best_turn_distance_=distance;turn_progress_time_=now;
    }
    turn_watch_clock_=now;
    if(error<=best_turn_error_-.05235987755982989 ||
       (best_turn_distance_>.05 && distance<=best_turn_distance_-.02)) {
      best_turn_error_=std::min(best_turn_error_,error);
      best_turn_distance_=std::min(best_turn_distance_,distance);
      turn_progress_time_=now;
    }
    return now-turn_progress_time_>=timeout;
  }
  PoseReference currentTarget() const {return path_.at(index_);}
  std::size_t index() const {return index_;}
  PoseCommand turnPositionCorrection(const PoseReference & measured,double max_speed,double horizon) const
  {
    if(index_==0 || index_>=path_.size()) {return {};}
    const auto & target=path_[index_];const auto & previous=path_[index_-1];
    if(std::hypot(target.x-previous.x,target.y-previous.y)>1e-8 ||
      std::abs(angle(target.yaw-previous.yaw))<1e-8) {return {};}
    const double ex=target.x-measured.x,ey=target.y-measured.y,distance=std::hypot(ex,ey);
    if(distance<=.003) {return {};}
    const double speed=std::min({std::max(0.0,max_speed),distance/std::max(.1,horizon),2*distance});
    const double wx=speed*ex/distance,wy=speed*ey/distance;
    return {std::cos(measured.yaw)*wx+std::sin(measured.yaw)*wy,
      -std::sin(measured.yaw)*wx+std::cos(measured.yaw)*wy,0};
  }
  double heightOffset() const {return height_offset_;}
  double initialSurfaceHeight() const {return path_.front().z;}

private:
  std::vector<PoseReference> path_;
  std::size_t index_{0};
  bool have_height_offset_{false};
  bool turn_watch_active_{false};
  std::size_t turn_watch_index_{0};
  double best_turn_error_{0},best_turn_distance_{0},turn_progress_time_{0},turn_watch_clock_{0};
  double height_offset_{0.0};
};
}  // namespace scan_planner_nav2_controller
#endif
