#ifndef GO2_SURFACE_3D_PLANNER__SURFACE_3D_PLANNER_HPP_
#define GO2_SURFACE_3D_PLANNER__SURFACE_3D_PLANNER_HPP_

#include "go2_surface_3d_planner/surface_graph.hpp"

#include <nav2_core/global_planner.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <mutex>
#include <string>

namespace go2_surface_3d_planner
{

class Surface3DPlanner : public nav2_core::GlobalPlanner
{
public:
  Surface3DPlanner() = default;
  ~Surface3DPlanner() override = default;

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
  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  geometry_msgs::msg::PoseStamped toPlannerFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  std::string planner_frame_{"map"};
  std::string octomap_topic_{"/octomap_full"};
  double transform_tolerance_{0.2};
  SurfaceSearchConfig search_config_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr regions_sub_;
  std::vector<SurfaceExclusionRegion> stair_regions_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr failed_turn_sub_;
  std::vector<std::pair<double,TurnExclusionRegion>> failed_turns_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr unsupported_sub_;
  std::vector<std::pair<double,SupportAvoidanceRegion>> unsupported_;
  mutable std::mutex map_mutex_;
  std::shared_ptr<const octomap::OcTree> map_;
  double map_received_at_{-1};
};

}  // namespace go2_surface_3d_planner

#endif  // GO2_SURFACE_3D_PLANNER__SURFACE_3D_PLANNER_HPP_
