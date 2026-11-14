#include "go2_nav2_recovery/escape_move.hpp"

#include <cmath>
#include <limits>
#include "octomap_msgs/conversions.h"
#include "go2_surface_3d_planner/surface_graph.hpp"
#include "go2_surface_3d_planner/parameter_utils.hpp"

#include "go2_nav2_recovery/footprint_safety.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"

namespace go2_nav2_recovery
{

using nav2_behaviors::Status;

void EscapeMove::onConfigure()
{
  auto node = node_.lock();
  stair_mode_sub_=node->create_subscription<std_msgs::msg::Bool>("/multifloor/stair_mode",rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Bool::SharedPtr msg){stair_mode_.store(msg->data);});
  stair_regions_sub_=node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/multifloor/stair_regions",rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Float64MultiArray::SharedPtr message) {
      if(message->layout.dim.empty() || message->layout.dim[0].label!=global_frame_)return;
      try {
        auto regions=go2_surface_3d_planner::decodeSurfaceRegions(message->data);
        std::lock_guard<std::mutex> lock(mutex_);excluded_regions_=std::move(regions);
      } catch(const std::invalid_argument & error) {RCLCPP_WARN(logger_,"%s",error.what());}
    });
  continuous_floor_region_=go2_surface_3d_planner::getOrDeclareParameter<std::vector<double>>(
    node,"continuous_floor_region",{});
  const auto parameter = [&](const std::string & key, double value) {
      nav2_util::declare_parameter_if_not_declared(
        node, behavior_name_ + "." + key, rclcpp::ParameterValue(value));
      return node->get_parameter(behavior_name_ + "." + key).as_double();
    };

  replan_pub_=rclcpp::create_publisher<std_msgs::msg::Header>(
    node,"/planning/go2_replan_request",rclcpp::QoS(1).reliable().transient_local());
  surface_target_gap_=parameter("surface_recovery_target_clearance",0.01);
  surface_sub_=node->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_full",rclcpp::QoS(1).reliable().transient_local(),
    [this](octomap_msgs::msg::Octomap::SharedPtr message) {
      if(message->header.frame_id!=global_frame_ || message->binary) {return;}
      std::unique_ptr<octomap::AbstractOcTree> decoded(octomap_msgs::msgToMap(*message));
      auto * tree=dynamic_cast<octomap::OcTree*>(decoded.get());
      if(!tree) {return;}
      decoded.release();
      std::lock_guard<std::mutex> lock(mutex_);
      surface_map_.reset(tree);surface_stamp_=message->header.stamp;
    });
  radius_ = parameter("search_radius", 0.8);
  step_ = parameter("search_resolution", 0.1);
  probe_ = parameter("direction_probe_distance", 0.2);
  configured_margin_ = parameter("safety_margin", 0.05);
  minimum_margin_ = std::clamp(
    parameter("minimum_safety_margin", 0.03), 0.0, configured_margin_);
  margin_ = configured_margin_;
  no_progress_timeout_ = parameter("no_progress_timeout", 3.0);
  progress_epsilon_ = parameter("progress_epsilon", 0.01);
  if (radius_ <= 0.0 || step_ < 0.02 || probe_ <= 0.0 || configured_margin_ < 0.0 ||
    no_progress_timeout_ <= 0.0 || progress_epsilon_ <= 0.0)
  {
    throw std::runtime_error("Invalid escape parameters");
  }

  std::string costmap_topic;
  std::string footprint_topic;
  node->get_parameter("costmap_topic", costmap_topic);
  node->get_parameter("footprint_topic", footprint_topic);
  footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
    node_, footprint_topic, *tf_, robot_base_frame_, transform_tolerance_);
  map_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_ = message;
    });
}

void EscapeMove::requestFreshPath()
{
  std_msgs::msg::Header message;message.stamp=clock_->now();message.frame_id=global_frame_;
  replan_pub_->publish(message);
}

EscapeMove::SurfaceRecovery EscapeMove::surfaceCommand(
  const geometry_msgs::msg::PoseStamped & pose,geometry_msgs::msg::Twist & command)
{
  std::shared_ptr<const octomap::OcTree> map;
  rclcpp::Time stamp(0,0,RCL_ROS_TIME);
  {std::lock_guard<std::mutex> lock(mutex_);map=surface_map_;stamp=surface_stamp_;}
  if(!map || (clock_->now()-stamp).seconds()<0 ||
    (clock_->now()-stamp).seconds()>1.5) {return SurfaceRecovery::unavailable;}
  try {
  go2_surface_3d_planner::SurfaceSearchConfig cfg;
  cfg.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
  cfg.infer_flat_support=true;cfg.snap_radius=0;cfg.snap_vertical_range=.2;
  cfg.continuous_floor_region=continuous_floor_region_;
  {std::lock_guard<std::mutex> lock(mutex_);cfg.excluded_regions=excluded_regions_;}
  cfg.flat_support_requires_prior=stair_mode_.load();
  cfg.clearance_margin=0;cfg.preferred_clearance=.10;
  const auto & p=pose.pose.position;
  const double yaw=tf2::getYaw(pose.pose.orientation);
  geometry_msgs::msg::TransformStamped feet;
  try {feet=tf_->lookupTransform(global_frame_,"base_footprint",tf2::TimePointZero);}
  catch(const tf2::TransformException &) {return SurfaceRecovery::unavailable;}
  const double feet_age=(clock_->now()-rclcpp::Time(feet.header.stamp)).seconds();
  if(feet_age < -0.1 || feet_age > 0.5) {return SurfaceRecovery::unavailable;}
  cfg.support_reference_height=p.z-feet.transform.translation.z;
  if(!go2_surface_3d_planner::configureSupportPlane(cfg,*map,{p.x,p.y,p.z,yaw})) {
    return SurfaceRecovery::blocked;
  }
  go2_surface_3d_planner::SurfaceGraph graph(*map,cfg);
  const auto stance=graph.snapToSupportedSurface({p.x,p.y,feet.transform.translation.z,yaw});
  if(!stance) {return SurfaceRecovery::blocked;}
  current_surface_gap_=graph.bodyClearanceAt(*stance);
  if(current_surface_gap_>=surface_target_gap_) {return SurfaceRecovery::clear;}
  const auto end=graph.findClearanceRecovery(*stance,surface_target_gap_,speed_,true);
  if(!end) {return SurfaceRecovery::blocked;}
  const double wx=end->x-stance->x,wy=end->y-stance->y;
  command.linear.x=std::cos(yaw)*wx+std::sin(yaw)*wy;
  command.linear.y=-std::sin(yaw)*wx+std::cos(yaw)*wy;
  const double norm=std::hypot(command.linear.x,command.linear.y);
  if(norm>speed_) {command.linear.x*=speed_/norm;command.linear.y*=speed_/norm;}
  return SurfaceRecovery::recover;
  } catch(const std::exception & error) {
    RCLCPP_WARN(logger_,"Clearance recovery geometry rejected: %s",error.what());
    return SurfaceRecovery::unavailable;
  }
}

bool EscapeMove::refresh()
{
  nav2_msgs::msg::Costmap::SharedPtr message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    message = latest_;
  }
  if (!message || message->header.frame_id != global_frame_) {
    return false;
  }

  const double age = (clock_->now() - rclcpp::Time(message->header.stamp)).seconds();
  if (age < 0.0 || age > 1.5 ||
    message->data.size() !=
    static_cast<size_t>(message->metadata.size_x) * message->metadata.size_y)
  {
    return false;
  }

  std_msgs::msg::Header header;
  if (!footprint_sub_->getFootprintInRobotFrame(footprint_, header)) {
    return false;
  }

  map_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
    message->metadata.size_x, message->metadata.size_y,
    message->metadata.resolution,
    message->metadata.origin.position.x, message->metadata.origin.position.y);
  std::copy(message->data.begin(), message->data.end(), map_->getCharMap());
  return true;
}

double EscapeMove::space(double x, double y, double yaw)
{
  return clearance(*map_, footprint_, x, y, yaw, margin_);
}

bool EscapeMove::segment(
  double x, double y, double yaw, double end_x, double end_y, bool monotonic)
{
  auto previous = assessFootprint(*map_, footprint_, x, y, yaw, margin_);
  if (previous.outside_map) {
    return false;
  }
  const auto initial = previous;
  const int samples = std::max(
    1, static_cast<int>(std::ceil(std::hypot(end_x - x, end_y - y) / 0.01)));
  for (int index = 1; index <= samples; ++index) {
    const double ratio = static_cast<double>(index) / samples;
    const auto value = assessFootprint(
      *map_, footprint_,
      x + (end_x - x) * ratio, y + (end_y - y) * ratio, yaw, margin_);
    if (value.outside_map) {
      return false;
    }

    if (initial.hard_overlap_cells > 0) {
      // Starting overlap is permitted only while the number of intersected
      // lethal/unknown cells stays level or falls on the way out.
      if (value.hard_overlap_cells > previous.hard_overlap_cells) {
        return false;
      }
    } else if (!value.collisionFree() ||
      (monotonic &&
      (value.clearance + 0.005 < previous.clearance ||
      value.clearance + 0.005 < initial.clearance)))
    {
      return false;
    }
    previous = value;
  }

  return initial.hard_overlap_cells == 0 ||
         previous.hard_overlap_cells < initial.hard_overlap_cells;
}

int EscapeMove::directions(double x, double y, double yaw)
{
  int count = 0;
  for (int index = 0; index < 4; ++index) {
    const double angle = yaw + index * M_PI_2;
    if (segment(
        x, y, yaw,
        x + probe_ * std::cos(angle), y + probe_ * std::sin(angle), false))
    {
      ++count;
    }
  }
  return count;
}

bool EscapeMove::directionFailed(double angle) const
{
  constexpr double sector_half_width = M_PI / 8.0;
  for (const double failed_angle : failed_direction_angles_) {
    if (std::abs(std::remainder(angle - failed_angle, 2.0 * M_PI)) <=
      sector_half_width)
    {
      return true;
    }
  }
  return false;
}

void EscapeMove::rememberFailedDirection(double x, double y)
{
  if (!have_goal_direction_) {
    return;
  }

  const double angle = std::atan2(goal_y_ - y, goal_x_ - x);
  if (!directionFailed(angle)) {
    failed_direction_angles_.push_back(angle);
  }
  if (!have_failure_origin_) {
    have_failure_origin_ = true;
    failure_origin_x_ = x;
    failure_origin_y_ = y;
  }
  RCLCPP_WARN(
    logger_,
    "Safe escape blacklisted stalled direction %.0f deg (%zu direction sectors blocked)",
    angle * 180.0 / M_PI, failed_direction_angles_.size());
}

void EscapeMove::clearFailureMemory()
{
  failed_direction_angles_.clear();
  have_failure_origin_ = false;
  have_goal_direction_ = false;
}

bool EscapeMove::selectGoal(double x, double y, double yaw)
{
  const auto initial = assessFootprint(*map_, footprint_, x, y, yaw, margin_);
  if (initial.outside_map) {
    RCLCPP_WARN(logger_, "Safe escape cannot start: padded footprint is outside the costmap");
    return false;
  }
  if (initial.hard_overlap_cells > 0) {
    RCLCPP_WARN(
      logger_,
      "Safe escape starting inside %d lethal/unknown cells; searching only for a "
      "non-worsening route out",
      initial.hard_overlap_cells);
  }

  double best_score = -std::numeric_limits<double>::infinity();
  int best_directions = 0;
  double selected_x = x;
  double selected_y = y;
  double selected_angle = yaw;

  // The first pass excludes directions that produced no physical motion.
  // If every geometrically viable direction has been tried, clear the short
  // failure memory and make one fresh pass against the latest costmap.
  for (int pass = 0; pass < 2; ++pass) {
    best_score = -std::numeric_limits<double>::infinity();
    for (int direction = 0; direction < 8; ++direction) {
      const double angle = yaw + direction * M_PI_4;
      if (directionFailed(angle)) {
        continue;
      }
      for (double distance = step_; distance <= radius_ + 1e-6; distance += step_) {
        const double candidate_x = x + distance * std::cos(angle);
        const double candidate_y = y + distance * std::sin(angle);
        if (!segment(x, y, yaw, candidate_x, candidate_y, true)) {
          continue;
        }

        const auto candidate = assessFootprint(
          *map_, footprint_, candidate_x, candidate_y, yaw, margin_);
        if (!candidate.collisionFree()) {
          continue;
        }
        const double candidate_clearance = candidate.clearance;
        const int free_directions = directions(candidate_x, candidate_y, yaw);
        const double required_clearance =
          initial.hard_overlap_cells > 0 ? 0.0 : initial.clearance + 0.02;
        if (free_directions < 2 || candidate_clearance < required_clearance) {
          continue;
        }

        const double score =
          100.0 * free_directions + 10.0 * candidate_clearance - distance;
        if (score > best_score) {
          best_score = score;
          selected_x = candidate_x;
          selected_y = candidate_y;
          selected_angle = angle;
          best_directions = free_directions;
        }
      }
    }

    if (std::isfinite(best_score)) {
      break;
    }
    if (failed_direction_angles_.empty()) {
      break;
    }
    RCLCPP_WARN(
      logger_,
      "Safe escape exhausted the remaining direction sectors; clearing %zu stalled sectors",
      failed_direction_angles_.size());
    clearFailureMemory();
  }

  if (!std::isfinite(best_score)) {
    RCLCPP_WARN(
      logger_, "Safe escape: no non-worsening route to a collision-free endpoint; staying stopped");
    return false;
  }

  goal_x_ = selected_x;
  goal_y_ = selected_y;
  goal_direction_angle_ = selected_angle;
  have_goal_direction_ = true;
  best_remaining_ = std::hypot(goal_x_ - x, goal_y_ - y);
  last_progress_time_ = clock_->now();
  RCLCPP_INFO(
    logger_,
    "Safe escape selected world endpoint (%.3f, %.3f), direction %.0f deg, "
    "free directions %d; robot yaw is unconstrained",
    goal_x_, goal_y_, goal_direction_angle_ * 180.0 / M_PI, best_directions);
  return true;
}

Status EscapeMove::onRun(const std::shared_ptr<const Action::Goal> command)
{
  stopRobot();
  geometry_msgs::msg::PoseStamped pose;
  surface_mode_=false;
  const bool clearance_only=command->target.z<0.0;
  if(stair_mode_.load()) {return Status::FAILED;}
  speed_=std::min(0.06,std::abs(static_cast<double>(command->speed)));
  if(speed_<0.001) {return Status::FAILED;}
  if(nav2_util::getCurrentPose(
    pose,*tf_,global_frame_,robot_base_frame_,transform_tolerance_)) {
    geometry_msgs::msg::Twist command;
    const auto status=surfaceCommand(pose,command);
    if(status==SurfaceRecovery::recover || (clearance_only && status==SurfaceRecovery::clear)) {
      surface_mode_=true;start_x_=pose.pose.position.x;start_y_=pose.pose.position.y;
      best_surface_gap_=current_surface_gap_;last_progress_time_=clock_->now();
      end_=clock_->now()+rclcpp::Duration::from_seconds(30.0);
      RCLCPP_INFO(logger_,"Starting supported 3D clearance recovery: gap %.4fm, target %.3fm",
        current_surface_gap_,surface_target_gap_);
      return Status::SUCCEEDED;
    }
    if(status==SurfaceRecovery::blocked || status==SurfaceRecovery::unavailable) {
      RCLCPP_WARN(logger_,"No fresh supported collision-free 3D margin recovery");
      return Status::FAILED;
    }
  }
  if(clearance_only) {return Status::FAILED;}
  if (!refresh() ||
    !nav2_util::getCurrentPose(
      pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_WARN(logger_, "Safe escape stopped: missing/stale map, footprint or pose");
    return Status::FAILED;
  }

  start_x_ = pose.pose.position.x;
  start_y_ = pose.pose.position.y;
  if (have_failure_origin_ &&
    std::hypot(start_x_ - failure_origin_x_, start_y_ - failure_origin_y_) > 0.25)
  {
    clearFailureMemory();
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  margin_ = configured_margin_;
  const auto normal_state = assessFootprint(
    *map_, footprint_, start_x_, start_y_, yaw, configured_margin_);
  if (minimum_margin_ < configured_margin_) {
    const auto minimum_state = assessFootprint(
      *map_, footprint_, start_x_, start_y_, yaw, minimum_margin_);
    if (minimum_state.collisionFree() ||
      minimum_state.hard_overlap_cells < normal_state.hard_overlap_cells)
    {
      margin_ = minimum_margin_;
      RCLCPP_WARN(
        logger_, "Safe escape using narrow-passage margin %.0f cm instead of %.0f cm",
        100.0 * margin_, 100.0 * configured_margin_);
    }
  }
  speed_ = std::min(0.06, std::abs(static_cast<double>(command->speed)));
  if (speed_ < 0.001 || !selectGoal(start_x_, start_y_, yaw)) {
    return Status::FAILED;
  }

  end_ = clock_->now() + rclcpp::Duration::from_seconds(30.0);
  return Status::SUCCEEDED;
}

Status EscapeMove::onCycleUpdate()
{
  geometry_msgs::msg::PoseStamped pose;
  if(stair_mode_.load()) {stopRobot();return Status::FAILED;}
  if(surface_mode_) {
    if(clock_->now()>end_ || !nav2_util::getCurrentPose(
      pose,*tf_,global_frame_,robot_base_frame_,transform_tolerance_)) {
      stopRobot();return Status::FAILED;
    }
    geometry_msgs::msg::Twist command;
    const auto status=surfaceCommand(pose,command);
    if(status==SurfaceRecovery::clear) {
      stopRobot();
      RCLCPP_INFO(logger_,"3D clearance restored to %.4fm; replan from measured pose",current_surface_gap_);
      requestFreshPath();return Status::SUCCEEDED;
    }
    if(status!=SurfaceRecovery::recover) {stopRobot();return Status::FAILED;}
    if(current_surface_gap_>best_surface_gap_+.002) {
      best_surface_gap_=current_surface_gap_;last_progress_time_=clock_->now();
    }
    if((clock_->now()-last_progress_time_).seconds()>no_progress_timeout_) {
      stopRobot();return Status::FAILED;
    }
    vel_pub_->publish(command);
    auto feedback=std::make_shared<Action::Feedback>();
    feedback->distance_traveled=std::hypot(pose.pose.position.x-start_x_,pose.pose.position.y-start_y_);
    action_server_->publish_feedback(feedback);
    return Status::RUNNING;
  }
  if (clock_->now() > end_ || !refresh() ||
    !nav2_util::getCurrentPose(
      pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    stopRobot();
    return Status::FAILED;
  }

  const double x = pose.pose.position.x;
  const double y = pose.pose.position.y;
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const auto current = assessFootprint(*map_, footprint_, x, y, yaw, margin_);
  if (current.outside_map) {
    stopRobot();
    RCLCPP_WARN(logger_, "Safe escape stopped: padded footprint left the local costmap");
    return Status::FAILED;
  }
  if (current.hard_overlap_cells > 0) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Safe escape is reducing an existing overlap of %d lethal/unknown cells",
      current.hard_overlap_cells);
  }

  double remaining = std::hypot(goal_x_ - x, goal_y_ - y);
  if (remaining < 0.025) {
    stopRobot();
    if (directions(x, y, yaw) >= 2) {
      clearFailureMemory();
      requestFreshPath();return Status::SUCCEEDED;
    }
    if (!selectGoal(x, y, yaw)) {
      return Status::FAILED;
    }
    remaining = best_remaining_;
  }

  if (remaining + progress_epsilon_ < best_remaining_) {
    best_remaining_ = remaining;
    last_progress_time_ = clock_->now();
  }

  const bool no_progress =
    (clock_->now() - last_progress_time_).seconds() > no_progress_timeout_;
  const bool route_blocked = !segment(x, y, yaw, goal_x_, goal_y_, false);
  if (no_progress || route_blocked) {
    stopRobot();
    RCLCPP_WARN(
      logger_, "Safe escape replanning from current pose: %s",
      no_progress ? "no position progress" : "remaining route became unsafe");
    if (no_progress) {
      rememberFailedDirection(x, y);
    }
    if (!selectGoal(x, y, yaw)) {
      return Status::FAILED;
    }
    remaining = best_remaining_;
  }

  const double velocity = std::min(speed_, remaining * 0.8);
  const double world_x = velocity * (goal_x_ - x) / remaining;
  const double world_y = velocity * (goal_y_ - y) / remaining;

  geometry_msgs::msg::Twist command;
  command.linear.x = std::cos(yaw) * world_x + std::sin(yaw) * world_y;
  command.linear.y = -std::sin(yaw) * world_x + std::cos(yaw) * world_y;
  vel_pub_->publish(command);

  auto feedback = std::make_shared<Action::Feedback>();
  feedback->distance_traveled = std::hypot(x - start_x_, y - start_y_);
  action_server_->publish_feedback(feedback);
  return Status::RUNNING;
}

}  // namespace go2_nav2_recovery

PLUGINLIB_EXPORT_CLASS(go2_nav2_recovery::EscapeMove, nav2_core::Behavior)
