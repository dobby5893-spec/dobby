#include "go2_surface_3d_planner/parameter_utils.hpp"
#include "go2_surface_3d_planner/goal_checker_3d.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <cmath>
#include <limits>

namespace go2_surface_3d_planner
{

void GoalChecker3D::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  const auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("GoalChecker3D failed to lock lifecycle node");
  }
  xy_goal_tolerance_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node,
    plugin_name + ".xy_goal_tolerance", 0.20);
  z_goal_tolerance_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node,
    plugin_name + ".z_goal_tolerance", 0.18);
  yaw_goal_tolerance_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node,
    plugin_name + ".yaw_goal_tolerance", 0.40);
  body_reference_height_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node,plugin_name + ".body_reference_height", 0.0);
  RCLCPP_INFO(
    node->get_logger(),
    "Configured %s: xy=%.2fm z=%.2fm yaw=%.2frad",
    plugin_name.c_str(), xy_goal_tolerance_, z_goal_tolerance_, yaw_goal_tolerance_);
}

void GoalChecker3D::reset()
{
}

bool GoalChecker3D::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &)
{
  const double dx = query_pose.position.x - goal_pose.position.x;
  const double dy = query_pose.position.y - goal_pose.position.y;
  const double dz = std::abs(query_pose.position.z - body_reference_height_ - goal_pose.position.z);
  const double yaw_error = std::abs(
    std::remainder(
      tf2::getYaw(query_pose.orientation) - tf2::getYaw(goal_pose.orientation),
      2.0 * M_PI));
  return std::hypot(dx, dy) <= xy_goal_tolerance_ &&
         dz <= z_goal_tolerance_ && yaw_error <= yaw_goal_tolerance_;
}

bool GoalChecker3D::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  pose_tolerance = geometry_msgs::msg::Pose();
  pose_tolerance.position.x = xy_goal_tolerance_;
  pose_tolerance.position.y = xy_goal_tolerance_;
  pose_tolerance.position.z = z_goal_tolerance_;
  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, yaw_goal_tolerance_);
  pose_tolerance.orientation = tf2::toMsg(orientation);

  const double unused = std::numeric_limits<double>::lowest();
  vel_tolerance = geometry_msgs::msg::Twist();
  vel_tolerance.linear.x = unused;
  vel_tolerance.linear.y = unused;
  vel_tolerance.linear.z = unused;
  vel_tolerance.angular.x = unused;
  vel_tolerance.angular.y = unused;
  vel_tolerance.angular.z = unused;
  return true;
}

}  // namespace go2_surface_3d_planner

PLUGINLIB_EXPORT_CLASS(
  go2_surface_3d_planner::GoalChecker3D,
  nav2_core::GoalChecker)
