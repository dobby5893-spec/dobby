#include <atomic>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "go2_surface_3d_planner/surface_regions.hpp"
#pragma once

#include <mutex>
#include <optional>
#include <memory>
#include <octomap/OcTree.h>
#include "octomap_msgs/msg/octomap.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "nav2_msgs/msg/costmap.hpp"

namespace go2_nav2_recovery
{

class EscapeMove : public nav2_behaviors::TimedBehavior<nav2_msgs::action::DriveOnHeading>
{
public:
  using Action = nav2_msgs::action::DriveOnHeading;
  nav2_behaviors::Status onRun(const std::shared_ptr<const Action::Goal>) override;
  nav2_behaviors::Status onCycleUpdate() override;
  void onConfigure() override;

private:
  bool refresh();
  void requestFreshPath();
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr replan_pub_;
  enum class SurfaceRecovery {unavailable, blocked, clear, recover};
  SurfaceRecovery surfaceCommand(
    const geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::Twist & command);
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr surface_sub_;
  std::shared_ptr<const octomap::OcTree> surface_map_;
  rclcpp::Time surface_stamp_{0,0,RCL_ROS_TIME};
  bool surface_mode_{false};
  double surface_target_gap_{0.01};
  double best_surface_gap_{0.0};
  double current_surface_gap_{0.0};
  double space(double x, double y, double yaw);
  bool segment(
    double x, double y, double yaw, double end_x, double end_y, bool monotonic);
  int directions(double x, double y, double yaw);
  bool selectGoal(double x, double y, double yaw);
  bool directionFailed(double angle) const;
  void rememberFailedDirection(double x, double y);
  void clearFailureMemory();

  std::atomic<bool> stair_mode_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stair_mode_sub_;
  std::vector<double> continuous_floor_region_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stair_regions_sub_;
  std::vector<go2_surface_3d_planner::SurfaceExclusionRegion> excluded_regions_;
  std::mutex mutex_;
  nav2_msgs::msg::Costmap::SharedPtr latest_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr map_sub_;
  std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> map_;
  std::vector<geometry_msgs::msg::Point> footprint_;

  double radius_{0.8};
  double step_{0.10};
  double probe_{0.20};
  double configured_margin_{0.05};
  double minimum_margin_{0.03};
  double margin_{0.05};
  double no_progress_timeout_{3.0};
  double progress_epsilon_{0.01};
  double speed_{0.06};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double start_x_{0.0};
  double start_y_{0.0};
  double best_remaining_{0.0};
  std::vector<double> failed_direction_angles_;
  bool have_failure_origin_{false};
  double failure_origin_x_{0.0};
  double failure_origin_y_{0.0};
  bool have_goal_direction_{false};
  double goal_direction_angle_{0.0};
  rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time end_{0, 0, RCL_ROS_TIME};
};

}  // namespace go2_nav2_recovery
