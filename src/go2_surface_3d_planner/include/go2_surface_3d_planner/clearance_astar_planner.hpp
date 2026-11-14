#ifndef GO2_SURFACE_3D_PLANNER__CLEARANCE_ASTAR_PLANNER_HPP_
#define GO2_SURFACE_3D_PLANNER__CLEARANCE_ASTAR_PLANNER_HPP_

#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace go2_surface_3d_planner
{

class ClearanceAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  ClearanceAStarPlanner() = default;
  ~ClearanceAStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool traversable(unsigned char cost) const;
  int nearestTraversable(
    unsigned int mx, unsigned int my, double radius,
    const std::vector<unsigned char> & costs,
    unsigned int width, unsigned int height) const;
  std::vector<double> computeClearance(
    const std::vector<unsigned char> & costs,
    unsigned int width, unsigned int height, double resolution) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  double tolerance_{0.20};
  double preferred_clearance_{0.35};
  double clearance_weight_{6.0};
  double costmap_weight_{1.0};
  double unknown_penalty_{3.0};
  int max_iterations_{500000};
  bool allow_unknown_{true};
};

}  // namespace go2_surface_3d_planner

#endif  // GO2_SURFACE_3D_PLANNER__CLEARANCE_ASTAR_PLANNER_HPP_
