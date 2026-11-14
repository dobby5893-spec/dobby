#include "scan_planner_nav2_controller/motion_speed_limit.hpp"
#include "go2_surface_3d_planner/parameter_utils.hpp"
#include "scan_planner_nav2_controller/scan_planner_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>
#include <vector>
#include "go2_nav2_recovery/footprint_safety.hpp"
#include "tf2/utils.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/AbstractOcTree.h"
#include "go2_surface_3d_planner/goal_checker_3d.hpp"

#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace scan_planner_nav2_controller
{

void ScanPlannerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("Unable to lock controller_server lifecycle node");
  }

  plugin_name_ = std::move(name);
  replan_pub_=node_->create_publisher<std_msgs::msg::Header>(
    "/planning/go2_replan_request",rclcpp::QoS(1).reliable().transient_local());
  failed_turn_pub_=node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/planning/go2_failed_turn",rclcpp::QoS(8).reliable().transient_local());
  unsupported_pub_=node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/navigation/unsupported_region",rclcpp::QoS(10).reliable());
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  path_topic_ = go2_surface_3d_planner::getOrDeclareParameter<std::string>(node_,
    plugin_name_ + ".path_topic", "/scanplanner/initial_path");
  command_topic_ = go2_surface_3d_planner::getOrDeclareParameter<std::string>(node_,
    plugin_name_ + ".command_topic", "/scanplanner/cmd_vel");
  planner_frame_ = go2_surface_3d_planner::getOrDeclareParameter<std::string>(node_,
    plugin_name_ + ".planner_frame", "map");
  command_timeout_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".command_timeout", 0.5);
  zero_command_timeout_ = std::max(
    0.0, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
      plugin_name_ + ".zero_command_timeout", 3.0));
  zero_command_epsilon_ = std::max(
    0.0, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
      plugin_name_ + ".zero_command_epsilon", 0.01));
  goal_change_tolerance_ = std::max(
    0.0, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
      plugin_name_ + ".goal_change_tolerance", 0.15));
  plan_gap_reset_timeout_ = std::max(
    0.0, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
      plugin_name_ + ".plan_gap_reset_timeout", 2.0));
  transform_tolerance_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".transform_tolerance", 0.2);
  max_linear_speed_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".max_linear_speed", 0.20);
  follow_pose_path_ = go2_surface_3d_planner::getOrDeclareParameter<bool>(node_,plugin_name_ + ".follow_pose_path", false);
  max_yaw_speed_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".max_yaw_speed", 0.35);
  replan_wait_timeout_=std::max(0.1,go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_+".replan_wait_timeout",12.0));
  safety_margin_ = std::max(
    0.0, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".safety_margin", 0.05));
  narrow_safety_margin_ = std::clamp(
    go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".narrow_safety_margin", 0.03),
    0.0, safety_margin_);
  narrow_mode_trigger_clearance_ = std::max(
    0.0, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
      plugin_name_ + ".narrow_mode_trigger_clearance", 0.025));
  narrow_max_linear_speed_ = std::max(
    0.01, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
      plugin_name_ + ".narrow_max_linear_speed", 0.06));
  caution_clearance_ = std::max(
    0.01, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".caution_clearance", 0.10));
  minimum_caution_scale_ = std::clamp(
    go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".minimum_caution_scale", 0.35),
    0.05, 1.0);
  safety_prediction_time_ = std::max(
    0.1, go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".safety_prediction_time", 1.0));

  if (follow_pose_path_) {
    surface_config_.collision_radius = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".collision_radius", 0.21);
    surface_config_.cylinder_offset = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".cylinder_offset", 0.10);
    surface_config_.body_bottom = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".body_bottom", 0.15);
    surface_config_.body_height = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".body_height", 0.48);
    surface_config_.clearance_margin = safety_margin_;
    surface_config_.infer_flat_support = go2_surface_3d_planner::getOrDeclareParameter<bool>(node_,plugin_name_ + ".infer_flat_support", true);
    surface_config_.continuous_floor_region = go2_surface_3d_planner::getOrDeclareParameter<std::vector<double>>(node_, "continuous_floor_region", {});
    stair_mode_sub_=node_->create_subscription<std_msgs::msg::Bool>("/multifloor/stair_mode",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr msg){stair_mode_.store(msg->data);});
    surface_config_.snap_radius = 0.0;
    surface_config_.snap_vertical_range = 0.20;
    surface_config_.max_step_height = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".max_step_height", 0.12);
    const auto topic = go2_surface_3d_planner::getOrDeclareParameter<std::string>(node_,plugin_name_ + ".octomap_topic", "/octomap_full");
    surface_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
      topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](octomap_msgs::msg::Octomap::SharedPtr message) {
        std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*message));
        auto * typed = dynamic_cast<octomap::OcTree *>(tree.get());
        if (!typed) {return;}
        tree.release();
        std::lock_guard<std::mutex> lock(surface_mutex_);
        surface_map_.reset(typed);
      });
    RCLCPP_INFO(node_->get_logger(), "Verified pose-path execution enabled: raw 3D body sweep, margin %.3fm", safety_margin_);
  }

  path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    path_topic_, rclcpp::QoS(1).reliable());
  command_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
    command_topic_, rclcpp::QoS(20).reliable(),
    std::bind(&ScanPlannerController::commandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: path %s, command %s, planner frame %s, zero-command timeout %.2fs",
    plugin_name_.c_str(), path_topic_.c_str(), command_topic_.c_str(), planner_frame_.c_str(),
    zero_command_timeout_);
}

void ScanPlannerController::cleanup()
{
  surface_sub_.reset();
  {std::lock_guard<std::mutex> lock(surface_mutex_); surface_map_.reset();}
  path_pub_.reset();
  replan_pub_.reset();failed_turn_pub_.reset();unsupported_pub_.reset();
  command_sub_.reset();
  tf_.reset();
  costmap_ros_.reset();
  node_.reset();
}

void ScanPlannerController::activate()
{
  path_pub_->on_activate();
  replan_pub_->on_activate();failed_turn_pub_->on_activate();unsupported_pub_->on_activate();
  std_msgs::msg::Header clear;clear.frame_id=planner_frame_;replan_pub_->publish(clear);
  clearance_recovery_active_=false;awaiting_replan_=false;
  std::lock_guard<std::mutex> lock(command_mutex_);
  have_command_ = false;
  zero_command_active_ = false;
  have_goal_ = false;
  have_plan_time_ = false;
}

void ScanPlannerController::deactivate()
{
  path_pub_->on_deactivate();
  replan_pub_->on_deactivate();failed_turn_pub_->on_deactivate();unsupported_pub_->on_deactivate();
  std::lock_guard<std::mutex> lock(command_mutex_);
  latest_command_ = geometry_msgs::msg::Twist();
  have_command_ = false;
  zero_command_active_ = false;
  have_goal_ = false;
  have_plan_time_ = false;
}

nav_msgs::msg::Path ScanPlannerController::transformPlan(const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty()) {
    throw nav2_core::PlannerException("SCAN-Planner received an empty Nav2 path");
  }

  nav_msgs::msg::Path transformed;
  transformed.header = path.header;
  transformed.header.frame_id = planner_frame_;
  transformed.header.stamp = node_->now();
  transformed.poses.reserve(path.poses.size());

  for (const auto & input_pose : path.poses) {
    geometry_msgs::msg::PoseStamped source = input_pose;
    if (source.header.frame_id.empty()) {
      source.header.frame_id = path.header.frame_id;
    }
    geometry_msgs::msg::PoseStamped output;
    if (source.header.frame_id == planner_frame_) {
      output = source;
    } else {
      try {
        output = tf_->transform(
          source, planner_frame_, tf2::durationFromSec(transform_tolerance_));
      } catch (const tf2::TransformException & error) {
        throw nav2_core::PlannerException(
                "Cannot transform Nav2 path from " + source.header.frame_id + " to " +
                planner_frame_ + ": " + error.what());
      }
    }
    output.header.frame_id = planner_frame_;
    output.header.stamp = transformed.header.stamp;
    transformed.poses.push_back(std::move(output));
  }
  return transformed;
}

void ScanPlannerController::setPlan(const nav_msgs::msg::Path & path)
{
  const auto transformed = transformPlan(path);
  if (follow_pose_path_) {
    std::vector<PoseReference> references;
    for (const auto & pose : transformed.poses) {
      const auto & p = pose.pose.position;
      references.push_back({p.x, p.y, p.z, tf2::getYaw(pose.pose.orientation)});
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if(awaiting_replan_ && rclcpp::Time(path.header.stamp)<=replan_stamp_) {return;}
    if(failed_turn_ && node_->now().seconds()-failed_turn_time_>=0 &&
       node_->now().seconds()-failed_turn_time_<=60) {
      for(std::size_t i=1;i<references.size();++i) {
        const auto & a=references[i-1];const auto & b=references[i];
        if(go2_surface_3d_planner::violatesTurnExclusion(
          {a.x,a.y,a.z,a.yaw},{b.x,b.y,b.z,b.yaw},*failed_turn_))
          throw nav2_core::PlannerException("New path repeats a recently stalled turn; wait for a translated route");
      }
    }
    pose_tracker_.setPlan(references);
    double low=references.front().z,high=low;
    for(const auto & p:references) {low=std::min(low,p.z);high=std::max(high,p.z);}
    stair_path_.store(high-low>.075);
    awaiting_replan_=false;clearance_recovery_active_=false;
    return;
  }
  const auto & goal = transformed.poses.back().pose.position;
  const auto now = node_->now();
  bool goal_changed = false;
  bool resumed_after_gap = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    goal_changed = !have_goal_ ||
      std::hypot(goal.x - current_goal_x_, goal.y - current_goal_y_) > goal_change_tolerance_;
    if (have_plan_time_ && plan_gap_reset_timeout_ > 0.0) {
      const double plan_gap = (now - last_plan_time_).seconds();
      resumed_after_gap = plan_gap < 0.0 || plan_gap > plan_gap_reset_timeout_;
    }
    last_plan_time_ = now;
    have_plan_time_ = true;
    if (goal_changed || resumed_after_gap) {
      current_goal_x_ = goal.x;
      current_goal_y_ = goal.y;
      have_goal_ = true;
      latest_command_ = geometry_msgs::msg::Twist();
      have_command_ = false;
      zero_command_active_ = false;
    }
  }
  path_pub_->publish(transformed);
  if (goal_changed || resumed_after_gap) {
    RCLCPP_INFO(
      node_->get_logger(),
      "SCAN-Planner controller reset stall monitoring for %s at endpoint (%.2f, %.2f)",
      goal_changed ? "a new goal" : "a resumed navigation attempt", goal.x, goal.y);
  }
  RCLCPP_DEBUG(
    node_->get_logger(), "Published %zu Nav2 path poses to SCAN-Planner",
    transformed.poses.size());
}

void ScanPlannerController::commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  const auto now = node_->now();
  latest_command_ = *msg;
  latest_command_time_ = now;
  have_command_ = true;

  const bool command_is_zero =
    std::abs(msg->linear.x) <= zero_command_epsilon_ &&
    std::abs(msg->linear.y) <= zero_command_epsilon_ &&
    std::abs(msg->angular.z) <= zero_command_epsilon_;
  if (command_is_zero) {
    if (!zero_command_active_) {
      zero_command_start_time_ = now;
      zero_command_active_ = true;
    }
  } else {
    zero_command_active_ = false;
  }
}

geometry_msgs::msg::TwistStamped ScanPlannerController::computePosePathCommand(
  const geometry_msgs::msg::PoseStamped & pose, nav2_core::GoalChecker * goal_checker)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if(awaiting_replan_) {
      if(std::chrono::duration<double>(std::chrono::steady_clock::now()-replan_requested_at_).count()>replan_wait_timeout_) {
        throw nav2_core::PlannerException("Timed out waiting for a strictly newer 3D path after replan request");
      }
      geometry_msgs::msg::TwistStamped stopped;
      stopped.header.stamp=node_->now();stopped.header.frame_id=costmap_ros_->getBaseFrameID();
      return stopped;
    }
  }
  geometry_msgs::msg::PoseStamped mapped = pose;
  if (mapped.header.frame_id != planner_frame_) {
    mapped = tf_->transform(mapped, planner_frame_, tf2::durationFromSec(transform_tolerance_));
  }
  std::shared_ptr<const octomap::OcTree> map;
  {std::lock_guard<std::mutex> lock(surface_mutex_); map = surface_map_;}
  if (!map) {throw nav2_core::PlannerException("Pose controller waiting for 3D support map");}
  PoseCommand requested;
  bool stalled_turn=false;PoseReference failed_target{};
  double height_offset = 0.0;
  const auto & p = mapped.pose.position;
  const double yaw = tf2::getYaw(mapped.pose.orientation);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    requested = pose_tracker_.command({p.x, p.y, p.z, yaw},
      motionSpeedLimit(max_linear_speed_, speed_scale_, stair_mode_.load() || stair_path_.load()), max_yaw_speed_, safety_prediction_time_);
    if(!stair_mode_.load() && !stair_path_.load()) {
      stalled_turn=pose_tracker_.turnStalled({p.x,p.y,p.z,yaw},node_->now().seconds());
      if(stalled_turn)failed_target=pose_tracker_.currentTarget();
    }
    height_offset = pose_tracker_.heightOffset();
    surface_config_.flat_support_height = pose_tracker_.initialSurfaceHeight();
  }
  if (auto * checker = dynamic_cast<go2_surface_3d_planner::GoalChecker3D *>(goal_checker)) {
    checker->setBodyReferenceHeight(height_offset);
  }
  auto collision_config = surface_config_;
  collision_config.flat_support_requires_prior=stair_mode_.load() || stair_path_.load();
  // The controller needs hard collision clearance only, not the A* soft-cost band.
  collision_config.preferred_clearance = collision_config.clearance_margin;
  collision_config.support_reference_height=height_offset;
  // Ground fitting uses current stance, not a path's quantized starting height.
  // Path tracking may retain its own offset, but it must not redefine the floor.
  try {
    const auto feet=tf_->lookupTransform(planner_frame_, "base_footprint", tf2::TimePointZero);
    collision_config.support_reference_height=p.z-feet.transform.translation.z;
  } catch (const tf2::TransformException &) {
    throw nav2_core::PlannerException("Pose controller waiting for current stance transform");
  }
  if (collision_config.infer_flat_support) {
    const auto prior=last_support_plane_.value_or(collision_config);
    if(go2_surface_3d_planner::configureSupportPlane(collision_config,*map,{p.x,p.y,p.z,yaw})) {
      last_support_plane_=collision_config;
    } else if(go2_surface_3d_planner::restoreCompatibleSupportPlane(
      collision_config,prior,{p.x,p.y,p.z,yaw})) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
        "Retaining stance-compatible previously verified ground; current map support and collision checks remain required");
    }
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const double age=node_->now().seconds()-failed_turn_time_;
    if(failed_turn_ && age>=0 && age<=60)collision_config.turn_exclusions.push_back(*failed_turn_);
  }
  go2_surface_3d_planner::SurfaceGraph graph(*map, collision_config);
  const double stance_z=p.z-collision_config.support_reference_height;
  const auto start = graph.snapToSurface({p.x, p.y, stance_z, yaw});
  const auto recover_clearance = [&](double required_gap) ->
    std::optional<geometry_msgs::msg::TwistStamped> {
    auto recovery_config=collision_config;
    recovery_config.clearance_margin=0.0;
    recovery_config.preferred_clearance=0.10;
    go2_surface_3d_planner::SurfaceGraph recovery_graph(*map,recovery_config);
    const auto stance=recovery_graph.snapToSupportedSurface({p.x,p.y,stance_z,yaw});
    if(stance) {
      const double horizon=std::max(0.1,safety_prediction_time_);
      const double speed=std::min(0.06,std::max(0.0,motionSpeedLimit(max_linear_speed_, speed_scale_, stair_mode_.load() || stair_path_.load())));
      const auto escape=recovery_graph.findClearanceRecovery(
        *stance,required_gap,speed*horizon,true);
      if(escape) {
        geometry_msgs::msg::TwistStamped command;
        command.header.stamp=node_->now();
        command.header.frame_id=costmap_ros_->getBaseFrameID();
        const double wx=(escape->x-stance->x)/horizon;
        const double wy=(escape->y-stance->y)/horizon;
        command.twist.linear.x=std::cos(yaw)*wx+std::sin(yaw)*wy;
        command.twist.linear.y=-std::sin(yaw)*wx+std::cos(yaw)*wy;
        RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
          "Verified small-step recovery: clearance %.4fm -> %.4fm; support retained, no new/deeper voxel contact",
          recovery_graph.bodyClearanceAt(*stance),recovery_graph.bodyClearanceAt(*escape));
        return command;
      }
    }
    return std::nullopt;
  };
  const auto request_replan = [&]() {
    std_msgs::msg::Header event;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      clearance_recovery_active_=false;awaiting_replan_=true;
      replan_stamp_=node_->now();event.stamp=replan_stamp_;
      replan_requested_at_=std::chrono::steady_clock::now();
    }
    event.frame_id=planner_frame_;replan_pub_->publish(event);
    RCLCPP_INFO(node_->get_logger(),"Stopping for a fresh 3D path after clearance recovery or blocked sweep");
    geometry_msgs::msg::TwistStamped stopped;
    stopped.header.stamp=node_->now();stopped.header.frame_id=costmap_ros_->getBaseFrameID();
    return stopped;
  };
  if(stalled_turn) {
    geometry_msgs::msg::PoseStamped failure;
    failure.header.stamp=node_->now();failure.header.frame_id=planner_frame_;
    failure.pose.position.x=failed_target.x;failure.pose.position.y=failed_target.y;
    failure.pose.position.z=failed_target.z;
    failure.pose.orientation.z=std::sin(yaw*.5);failure.pose.orientation.w=std::cos(yaw*.5);
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      failed_turn_=go2_surface_3d_planner::TurnExclusionRegion{
        failed_target.x,failed_target.y,failed_target.z,yaw,.25};
      failed_turn_time_=node_->now().seconds();
    }
    failed_turn_pub_->publish(failure);
    RCLCPP_WARN(node_->get_logger(),
      "Turn stage stalled for 8s at measured (%.3f, %.3f), target (%.3f, %.3f), xy_error %.3fm, measured yaw %.3f target %.3f, yaw_error %.3frad; request translation before turning",
      p.x,p.y,failed_target.x,failed_target.y,
      std::hypot(p.x-failed_target.x,p.y-failed_target.y),yaw,failed_target.yaw,
      std::remainder(failed_target.yaw-yaw,2*M_PI));
    return request_replan();
  }
  bool recovering;
  double recovery_target;
  {std::lock_guard<std::mutex> lock(command_mutex_);
    recovering=clearance_recovery_active_;recovery_target=clearance_recovery_target_;}
  if(recovering) {
    if(auto command=recover_clearance(recovery_target)) {return *command;}
    auto measure_config=collision_config;measure_config.preferred_clearance=0.10;
    if(start && go2_surface_3d_planner::SurfaceGraph(*map,measure_config).bodyClearanceAt(*start)>=collision_config.clearance_margin) {
      return request_replan();
    }
    throw nav2_core::PlannerException("Verified clearance recovery lost support or safe motion");
  }
  const double desired_recovery_gap=std::max(0.05,collision_config.clearance_margin+0.02);
  if (!start) {
    if (auto recovery=recover_clearance(desired_recovery_gap)) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      clearance_recovery_active_=true;clearance_recovery_target_=desired_recovery_gap;
      return *recovery;
    }
    const auto diagnostic=graph.diagnosePose({p.x,p.y,p.z-collision_config.support_reference_height,yaw});
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
      "Stance rejection details: body=(%.3f, %.3f, %.3f), feet_z=%.3f, plane_z=%.3f, surface=%d, support=%d, clearance=%d, gap=%.4f, margin=%.4f",
      p.x,p.y,p.z,p.z-collision_config.support_reference_height,
      collision_config.flat_support_height,diagnostic.surface,diagnostic.support,
      diagnostic.clearance,diagnostic.gap,collision_config.clearance_margin);
    throw nav2_core::PlannerException(
      "Pose safety stop: no supported collision-free stance or verified margin recovery");
  }
  geometry_msgs::msg::TwistStamped output;
  output.header.stamp = node_->now();
  output.header.frame_id = costmap_ros_->getBaseFrameID();

  std::optional<go2_surface_3d_planner::SurfacePoint> unsupported_next;
  const auto valid = [&](const PoseCommand & command) {
      auto previous = *start;
      const int steps = std::max({1,
        static_cast<int>(std::ceil(safety_prediction_time_ * std::hypot(command.vx, command.vy) / 0.02)),
        static_cast<int>(std::ceil(safety_prediction_time_ * std::abs(command.wz) / (M_PI / 60.0)))});
      for (int i = 0; i < steps; ++i) {
        const double dt = safety_prediction_time_ / steps;
        const double mid_yaw = previous.yaw + 0.5 * dt * command.wz;
        const go2_surface_3d_planner::SurfacePoint next{
          previous.x + dt * (std::cos(mid_yaw) * command.vx - std::sin(mid_yaw) * command.vy),
          previous.y + dt * (std::sin(mid_yaw) * command.vx + std::cos(mid_yaw) * command.vy),
          previous.z, previous.yaw + dt * command.wz};
        const auto surface = graph.snapToSurface(next);
        if (!surface || !graph.isMotionValid(previous, *surface)) {
          const auto state=graph.diagnosePose(next);
          if(!state.surface || !state.support)unsupported_next=next;
          return false;
        }
        previous = *surface;
      }
      return true;
    };
  // Pose-path execution uses the narrow/caution speed limits too. Measure
  // beyond the hard-margin query band before choosing the speed cap.
  auto probe_config=collision_config;
  probe_config.preferred_clearance=std::max(0.10,caution_clearance_);
  const double measured_gap=go2_surface_3d_planner::SurfaceGraph(*map,probe_config).bodyClearanceAt(*start);
  const double nominal=std::max(0.0,motionSpeedLimit(max_linear_speed_, speed_scale_, stair_mode_.load() || stair_path_.load()));
  double speed_cap=nominal;
  if(caution_clearance_>0 && measured_gap<caution_clearance_) {
    speed_cap=std::max(std::min(narrow_max_linear_speed_,nominal),
      nominal*std::clamp((measured_gap-safety_margin_)/caution_clearance_,0.0,1.0));
  }
  if(measured_gap<narrow_mode_trigger_clearance_) {
    speed_cap=std::min(speed_cap,narrow_max_linear_speed_);
    const double yaw_cap=std::min(max_yaw_speed_,0.15);
    if (std::abs(requested.wz)>yaw_cap) {
      const double scale=yaw_cap/std::abs(requested.wz);
      requested.vx*=scale; requested.vy*=scale; requested.wz*=scale;
    }
  }
  const double linear=std::hypot(requested.vx,requested.vy);
  if(linear>speed_cap && linear>1e-9) {
    const double scale=speed_cap/linear;
    requested.vx*=scale; requested.vy*=scale; requested.wz*=scale;
  }
  for (double scale : {1.0, 0.75, 0.5, 0.25}) {
    const PoseCommand candidate{scale * requested.vx, scale * requested.vy, scale * requested.wz};
    if (!valid(candidate)) {continue;}
    output.twist.linear.x = candidate.vx;
    output.twist.linear.y = candidate.vy;
    output.twist.angular.z = candidate.wz;
    return output;
  }
  PoseCommand correction;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    correction=pose_tracker_.turnPositionCorrection({p.x,p.y,p.z,yaw},
      std::min(.03,speed_cap),safety_prediction_time_);
  }
  if(std::hypot(correction.vx,correction.vy)>1e-4 && valid(correction)) {
    output.twist.linear.x=correction.vx;output.twist.linear.y=correction.vy;
    RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
      "Verified turn-position correction before rotation");
    return output;
  }
  // A measured stance can satisfy the hard margin while the next turn does
  // not. Move away monotonically before retrying that near-wall manoeuvre.
  if (auto recovery=recover_clearance(desired_recovery_gap)) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    clearance_recovery_active_=true;clearance_recovery_target_=desired_recovery_gap;
    return *recovery;
  }
  if(unsupported_next) {
    const double now=node_->now().seconds();
    if(now<unsupported_last_stamp_ || now-unsupported_last_stamp_>=2.) {
      geometry_msgs::msg::PoseStamped event;event.header.frame_id=planner_frame_;
      event.header.stamp=node_->now();event.pose.orientation.w=1;
      event.pose.position.x=unsupported_next->x;event.pose.position.y=unsupported_next->y;
      event.pose.position.z=unsupported_next->z;
      unsupported_pub_->publish(event);unsupported_last_stamp_=now;
      RCLCPP_WARN(node_->get_logger(),
        "Plane and 3D support checks failed ahead at (%.3f, %.3f, %.3f); stop and request temporary detour",
        unsupported_next->x,unsupported_next->y,unsupported_next->z);
    }
  }
  return request_replan();
}

geometry_msgs::msg::TwistStamped ScanPlannerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist &,
  nav2_core::GoalChecker * goal_checker)
{
  if (follow_pose_path_) {return computePosePathCommand(pose, goal_checker);}
  geometry_msgs::msg::Twist command;
  const auto now = node_->now();
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!have_command_) {
      throw nav2_core::PlannerException("Waiting for the first SCAN-Planner command");
    }
    const double age = (now - latest_command_time_).seconds();
    if (age < 0.0 || age > command_timeout_) {
      throw nav2_core::PlannerException("SCAN-Planner command timed out");
    }
    if (zero_command_timeout_ > 0.0 && zero_command_active_) {
      const double zero_duration = (now - zero_command_start_time_).seconds();
      if (zero_duration < 0.0 || zero_duration > zero_command_timeout_) {
        throw nav2_core::PlannerException(
                "SCAN-Planner continuously output zero velocity for " +
                std::to_string(zero_duration) + " seconds");
      }
    }
    command = latest_command_;
  }

  const double linear_norm = std::hypot(command.linear.x, command.linear.y);
  const double allowed_speed = std::max(0.0, motionSpeedLimit(max_linear_speed_, speed_scale_, stair_mode_.load() || stair_path_.load()));
  if (linear_norm > allowed_speed && linear_norm > 1e-6) {
    const double scale = allowed_speed / linear_norm;
    command.linear.x *= scale;
    command.linear.y *= scale;
  }

  // Check the padded footprint over the predicted motion. Cost 254 and
  // unknown space are hard stops; cost 253 alone does not override SCAN.
  if (!costmap_ros_->isCurrent()) {
    throw nav2_core::PlannerException("Safety stop: local costmap is stale");
  }
  if (!std::isfinite(command.linear.x + command.linear.y + command.angular.z)) {
    throw nav2_core::PlannerException("Safety stop: non-finite velocity");
  }
  geometry_msgs::msg::PoseStamped local_pose = pose;
  if (pose.header.frame_id != costmap_ros_->getGlobalFrameID()) {
    local_pose = tf_->transform(
      pose, costmap_ros_->getGlobalFrameID(),
      tf2::durationFromSec(transform_tolerance_));
  }

  auto * map = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
  const auto footprint = costmap_ros_->getRobotFootprint();
  const double pose_x = local_pose.pose.position.x;
  const double pose_y = local_pose.pose.position.y;
  const double pose_yaw = tf2::getYaw(local_pose.pose.orientation);
  double active_safety_margin = safety_margin_;
  bool narrow_mode = false;
  auto current_safety = go2_nav2_recovery::assessFootprint(
    *map, footprint, pose_x, pose_y, pose_yaw, safety_margin_);
  if (!current_safety.collisionFree() ||
    current_safety.hard_clearance < narrow_mode_trigger_clearance_)
  {
    const auto narrow_safety = go2_nav2_recovery::assessFootprint(
      *map, footprint, pose_x, pose_y, pose_yaw, narrow_safety_margin_);
    if (narrow_safety.collisionFree()) {
      active_safety_margin = narrow_safety_margin_;
      current_safety = narrow_safety;
      narrow_mode = true;
    } else if (!current_safety.collisionFree()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Current footprint rejected: frame=%s pose=(%.3f,%.3f,%.3f) margin=%.3f "
        "outside=%d cell=(%d,%d) cost=%d cell_center=(%.3f,%.3f)",
        local_pose.header.frame_id.c_str(), pose_x, pose_y, pose_yaw,
        narrow_safety_margin_, narrow_safety.outside_map,
        narrow_safety.first_hard_x, narrow_safety.first_hard_y,
        narrow_safety.first_hard_cost, narrow_safety.first_hard_world_x,
        narrow_safety.first_hard_world_y);
      throw nav2_core::PlannerException(
              "Safety stop: current minimum-margin footprint overlaps "
              "lethal/unknown space; requesting escape recovery");
    }
  }
  const double current_clearance = current_safety.hard_clearance;

  if (narrow_mode) {
    const double narrow_linear = std::hypot(command.linear.x, command.linear.y);
    if (narrow_linear > narrow_max_linear_speed_ && narrow_linear > 1e-6) {
      const double scale = narrow_max_linear_speed_ / narrow_linear;
      command.linear.x *= scale;
      command.linear.y *= scale;
    }
    // Keep SCAN-Planner's requested turn when its predicted footprint remains
    // collision-free. Replace the command only if the predicted body sweep
    // would enter lethal or unknown space.
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Narrow-passage mode: %.0f cm margin, translation limited to %.2f m/s",
      100.0 * active_safety_margin, narrow_max_linear_speed_);
  }

  const auto evaluate_trajectory =
    [&](const geometry_msgs::msg::Twist & candidate, double & minimum_clearance,
      double & final_clearance, bool report_rejection = false)
    {
      double x = pose_x;
      double y = pose_y;
      double yaw = pose_yaw;
      minimum_clearance = std::numeric_limits<double>::infinity();
      final_clearance = current_clearance;
      constexpr double time_step = 0.02;
      const int steps = std::max(
        1, static_cast<int>(std::ceil(safety_prediction_time_ / time_step)));
      for (int index = 0; index <= steps; ++index) {
        const auto state = go2_nav2_recovery::assessFootprint(
          *map, footprint, x, y, yaw, active_safety_margin);
        if (!state.collisionFree()) {
          if (report_rejection) {
            RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 2000,
              "SCAN sweep rejected: frame=%s t=%.2f pose=(%.3f,%.3f,%.3f) "
              "cmd=(%.3f,%.3f,%.3f) margin=%.3f outside=%d "
              "cell=(%d,%d) cost=%d cell_center=(%.3f,%.3f)",
              local_pose.header.frame_id.c_str(), index * time_step, x, y, yaw,
              candidate.linear.x, candidate.linear.y, candidate.angular.z,
              active_safety_margin, state.outside_map, state.first_hard_x,
              state.first_hard_y, state.first_hard_cost,
              state.first_hard_world_x, state.first_hard_world_y);
          }
          return false;
        }
        minimum_clearance = std::min(minimum_clearance, state.hard_clearance);
        final_clearance = state.hard_clearance;
        x += time_step *
          (std::cos(yaw) * candidate.linear.x - std::sin(yaw) * candidate.linear.y);
        y += time_step *
          (std::sin(yaw) * candidate.linear.x + std::cos(yaw) * candidate.linear.y);
        yaw += time_step * candidate.angular.z;
      }
      return true;
    };

  double minimum_clearance = 0.0;
  double final_clearance = 0.0;
  const bool requested_safe =
    evaluate_trajectory(command, minimum_clearance, final_clearance, true);

  // Cost 253 is a soft inflation band. A command that keeps the padded body
  // away from lethal and unknown cells retains SCAN's intended direction.
  // Search alternatives only when the predicted body sweep has a hard collision.
  if (!requested_safe) {
    const geometry_msgs::msg::Twist requested = command;
    const double requested_linear =
      std::hypot(requested.linear.x, requested.linear.y);
    std::vector<geometry_msgs::msg::Twist> candidates;
    candidates.push_back(requested);

    // Preserve the path direction while checking shorter emergency steps.
    for (const double scale : {0.75, 0.50, 0.25}) {
      auto shorter = requested;
      shorter.linear.x *= scale;
      shorter.linear.y *= scale;
      shorter.angular.z *= scale;
      candidates.push_back(shorter);
    }

    auto no_turn = requested;
    no_turn.angular.z = 0.0;
    candidates.push_back(no_turn);

    auto reduced_turn = requested;
    reduced_turn.angular.z *= 0.35;
    candidates.push_back(reduced_turn);

    if (requested_linear > 1e-4) {
      const double escape_speed = narrow_mode ? narrow_max_linear_speed_ : 0.06;
      for (const auto & direction : std::vector<std::pair<double, double>>{
          {1.0, 0.0}, {0.0, 1.0}, {0.0, -1.0}, {-1.0, 0.0},
          {0.707, 0.707}, {0.707, -0.707}, {-0.707, 0.707}, {-0.707, -0.707}})
      {
        geometry_msgs::msg::Twist alternative;
        alternative.linear.x = escape_speed * direction.first;
        alternative.linear.y = escape_speed * direction.second;
        candidates.push_back(alternative);
      }
    }

    bool found = false;
    double best_score = -std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Twist best_command;
    double best_minimum_clearance = 0.0;
    double best_final_clearance = 0.0;

    for (const auto & candidate : candidates) {
      const double candidate_linear =
        std::hypot(candidate.linear.x, candidate.linear.y);
      if (candidate_linear <= zero_command_epsilon_ &&
        std::abs(candidate.angular.z) <= zero_command_epsilon_)
      {
        continue;
      }

      double candidate_minimum = 0.0;
      double candidate_final = 0.0;
      if (!evaluate_trajectory(candidate, candidate_minimum, candidate_final)) {
        continue;
      }
      double alignment = 0.0;
      if (requested_linear > 1e-4) {
        if (candidate_linear <= 1e-4) {
          continue;
        }
        alignment =
          (requested.linear.x * candidate.linear.x +
          requested.linear.y * candidate.linear.y) /
          (requested_linear * candidate_linear);
        // SCAN's command follows the current global reference segment.
        // Clearance-only sideways or backward commands cannot replace it.
        if (alignment < 0.25) {
          continue;
        }
      }
      const double forward_progress = candidate_linear * alignment;
      const double turn_preference =
        requested.angular.z * candidate.angular.z > 0.0 ? 0.10 : 0.0;
      const double score =
        10.0 * forward_progress +
        1.5 * std::min(candidate_final, 0.60) +
        0.5 * std::min(candidate_minimum, 0.60) +
        turn_preference;
      if (score > best_score) {
        best_score = score;
        best_command = candidate;
        best_minimum_clearance = candidate_minimum;
        best_final_clearance = candidate_final;
        found = true;
      }
    }

    if (!found) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Safety rejected SCAN command: no collision-free forward-progress alternative");
      throw nav2_core::PlannerException(
              "Safety stop: no collision-free forward-progress controller alternative");
    }

    const bool command_changed =
      std::abs(best_command.linear.x - requested.linear.x) > 1e-4 ||
      std::abs(best_command.linear.y - requested.linear.y) > 1e-4 ||
      std::abs(best_command.angular.z - requested.angular.z) > 1e-4;
    command = best_command;
    minimum_clearance = best_minimum_clearance;
    final_clearance = best_final_clearance;
    if (command_changed) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Near-obstacle SCAN command replaced: vx=%.3f vy=%.3f wz=%.3f, "
        "clearance %.3f -> %.3f m",
        command.linear.x, command.linear.y, command.angular.z,
        current_clearance, final_clearance);
    }
  }

  // Caution scaling uses distance to hard cells, not the soft cost-253 band.
  // Narrow mode already limits translation and validates the full body sweep.
  if (!narrow_mode && minimum_clearance < caution_clearance_) {
    const double caution_scale = std::clamp(
      minimum_clearance / caution_clearance_,
      minimum_caution_scale_, 1.0);
    command.linear.x *= caution_scale;
    command.linear.y *= caution_scale;
    command.angular.z *= caution_scale;

    if (!evaluate_trajectory(command, minimum_clearance, final_clearance)) {
      throw nav2_core::PlannerException(
              "Safety stop: reduced command would enter lethal/unknown space");
    }
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Near hard obstacle: scaling SCAN command to %.0f%%",
      caution_scale * 100.0);
  }

  geometry_msgs::msg::TwistStamped result;
  result.header.stamp = now;
  result.header.frame_id = pose.header.frame_id;
  result.twist = command;
  return result;
}

void ScanPlannerController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    speed_scale_ = std::clamp(speed_limit / 100.0, 0.0, 1.0);
  } else if (speed_limit <= 0.0) {
    speed_scale_ = 1.0;
  } else {
    speed_scale_ = std::clamp(speed_limit / max_linear_speed_, 0.0, 1.0);
  }
}

}  // namespace scan_planner_nav2_controller

PLUGINLIB_EXPORT_CLASS(
  scan_planner_nav2_controller::ScanPlannerController,
  nav2_core::Controller)
