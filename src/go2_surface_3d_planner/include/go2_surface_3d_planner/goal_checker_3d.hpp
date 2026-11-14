#ifndef GO2_SURFACE_3D_PLANNER__GOAL_CHECKER_3D_HPP_
#define GO2_SURFACE_3D_PLANNER__GOAL_CHECKER_3D_HPP_

#include <nav2_core/goal_checker.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <string>

namespace go2_surface_3d_planner
{

class GoalChecker3D : public nav2_core::GoalChecker
{
public:
  void setBodyReferenceHeight(double height) {body_reference_height_ = height;}
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  double xy_goal_tolerance_{0.20};
  double z_goal_tolerance_{0.18};
  double yaw_goal_tolerance_{0.40};
  double body_reference_height_{0.0};
};

}  // namespace go2_surface_3d_planner

#endif  // GO2_SURFACE_3D_PLANNER__GOAL_CHECKER_3D_HPP_
