#include <atomic>
#include <std_msgs/msg/bool.hpp>
#ifndef SCAN_PLANNER_NAV2_CONTROLLER__SCAN_PLANNER_CONTROLLER_HPP_
#define SCAN_PLANNER_NAV2_CONTROLLER__SCAN_PLANNER_CONTROLLER_HPP_

#include <memory>
#include <chrono>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"
#include "scan_planner_nav2_controller/pose_path_tracker.hpp"
#include "go2_surface_3d_planner/surface_graph.hpp"
#include "octomap_msgs/msg/octomap.hpp"

namespace scan_planner_nav2_controller
{

class ScanPlannerController : public nav2_core::Controller
{
public:
  ScanPlannerController() = default;
  ~ScanPlannerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  nav_msgs::msg::Path transformPlan(const nav_msgs::msg::Path & path) const;
  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  geometry_msgs::msg::TwistStamped computePosePathCommand(
    const geometry_msgs::msg::PoseStamped & pose, nav2_core::GoalChecker * goal_checker);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;

  mutable std::mutex command_mutex_;
  geometry_msgs::msg::Twist latest_command_;
  rclcpp::Time latest_command_time_{0, 0, RCL_ROS_TIME};
  bool have_command_{false};
  rclcpp::Time zero_command_start_time_{0, 0, RCL_ROS_TIME};
  bool zero_command_active_{false};
  bool have_goal_{false};
  double current_goal_x_{0.0};
  double current_goal_y_{0.0};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  bool have_plan_time_{false};

  std::string path_topic_;
  std::string command_topic_;
  std::string planner_frame_;
  double command_timeout_{0.5};
  double zero_command_timeout_{3.0};
  double zero_command_epsilon_{0.01};
  double goal_change_tolerance_{0.15};
  double plan_gap_reset_timeout_{2.0};
  double transform_tolerance_{0.2};
  double max_linear_speed_{0.20};
  double speed_scale_{1.0};
  double safety_margin_{0.05};
  double replan_wait_timeout_{12.0};
  std::chrono::steady_clock::time_point replan_requested_at_{};
  double narrow_safety_margin_{0.03};
  double narrow_mode_trigger_clearance_{0.025};
  double narrow_max_linear_speed_{0.06};
  double caution_clearance_{0.10};
  double minimum_caution_scale_{0.35};
  double safety_prediction_time_{1.0};
  bool follow_pose_path_{false};
  double max_yaw_speed_{0.35};
  PosePathTracker pose_tracker_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Header>::SharedPtr replan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr failed_turn_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr unsupported_pub_;
  double unsupported_last_stamp_{-1};
  std::optional<go2_surface_3d_planner::TurnExclusionRegion> failed_turn_;
  double failed_turn_time_{-1};
  bool clearance_recovery_active_{false};
  bool awaiting_replan_{false};
  double clearance_recovery_target_{0.05};
  rclcpp::Time replan_stamp_{0,0,RCL_ROS_TIME};
  go2_surface_3d_planner::SurfaceSearchConfig surface_config_;
  std::atomic<bool> stair_mode_{false},stair_path_{false};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stair_mode_sub_;
  std::optional<go2_surface_3d_planner::SurfaceSearchConfig> last_support_plane_;
  std::mutex surface_mutex_;
  std::shared_ptr<const octomap::OcTree> surface_map_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr surface_sub_;
};

}  // namespace scan_planner_nav2_controller

#endif  // SCAN_PLANNER_NAV2_CONTROLLER__SCAN_PLANNER_CONTROLLER_HPP_
