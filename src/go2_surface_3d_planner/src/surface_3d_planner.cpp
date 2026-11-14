#include "go2_surface_3d_planner/parameter_utils.hpp"
#include "go2_surface_3d_planner/surface_3d_planner.hpp"

#include <nav2_core/exceptions.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <memory>
#include <utility>

namespace go2_surface_3d_planner
{

void Surface3DPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS>)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("Surface3DPlanner failed to lock lifecycle node");
  }
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);

  if(!node_->has_parameter(plugin_name_+".planning_time_budget_s")) {
    go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_+".planning_time_budget_s",8.0);
  }
  planner_frame_ = go2_surface_3d_planner::getOrDeclareParameter<std::string>(node_,plugin_name_ + ".planner_frame", "map");
  octomap_topic_ = go2_surface_3d_planner::getOrDeclareParameter<std::string>(node_,
    plugin_name_ + ".octomap_topic", "/octomap_full");
  transform_tolerance_ = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".transform_tolerance", 0.2);
  search_config_.xy_resolution = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".xy_resolution", 0.10);
  search_config_.z_resolution = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".z_resolution", 0.05);
  search_config_.collision_radius = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".collision_radius", 0.21);
  search_config_.cylinder_offset = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".cylinder_offset", 0.10);
  search_config_.yaw_bins = go2_surface_3d_planner::getOrDeclareParameter<int>(node_,plugin_name_ + ".yaw_bins", 32);
  search_config_.body_bottom = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,plugin_name_ + ".body_bottom", 0.15);
  search_config_.preferred_clearance = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".preferred_clearance", 0.20);
  search_config_.clearance_weight = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".clearance_weight", 2.0);
  search_config_.turn_cost_weight = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".turn_cost_weight", 0.10);
  search_config_.sweep_resolution = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".sweep_resolution", 0.02);
  search_config_.sweep_angle = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".sweep_angle", 0.05235987755982989);
  search_config_.body_height = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".body_height", 0.48);
  search_config_.clearance_margin = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".clearance_margin", 0.01);
  search_config_.narrow_clearance_margin = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".narrow_clearance_margin", 0.0);
  search_config_.support_probe_depth = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".support_probe_depth", 0.16);
  search_config_.max_step_height = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".max_step_height", 0.12);
  search_config_.snap_radius = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".snap_radius", 0.60);
  search_config_.snap_vertical_range = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".snap_vertical_range", 0.55);
  search_config_.search_padding_xy = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".search_padding_xy", 1.50);
  search_config_.search_padding_z = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".search_padding_z", 0.35);
  search_config_.vertical_cost_weight = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".vertical_cost_weight", 2.0);
  search_config_.goal_tolerance = go2_surface_3d_planner::getOrDeclareParameter<double>(node_,
    plugin_name_ + ".goal_tolerance", 0.16);
  search_config_.max_iterations = go2_surface_3d_planner::getOrDeclareParameter<int>(node_,
    plugin_name_ + ".max_iterations", 250000);
  search_config_.require_known_clearance = go2_surface_3d_planner::getOrDeclareParameter<bool>(node_,
    plugin_name_ + ".require_known_clearance", false);
  search_config_.infer_flat_support = go2_surface_3d_planner::getOrDeclareParameter<bool>(node_,plugin_name_ + ".infer_flat_support", true);

  search_config_.continuous_floor_region = getOrDeclareParameter<std::vector<double>>(
    node_, "continuous_floor_region", {});
  search_config_.flat_support_requires_prior=getOrDeclareParameter<bool>(
    node_,plugin_name_+".flat_support_requires_prior",plugin_name_=="Stair3D");
  if(getOrDeclareParameter<bool>(node_,plugin_name_+".avoid_stair_regions",plugin_name_=="Floor3D")) {
    regions_sub_=node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/multifloor/stair_regions",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
        if(m->layout.dim.empty() || m->layout.dim[0].label!=planner_frame_) return;
        try {auto regions=decodeSurfaceRegions(m->data);std::lock_guard<std::mutex> lock(map_mutex_);stair_regions_=std::move(regions);}
        catch(const std::invalid_argument & e) {RCLCPP_WARN(node_->get_logger(),"%s",e.what());}
      });
  }
  if(plugin_name_=="Floor3D") {
    failed_turn_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/planning/go2_failed_turn",rclcpp::QoS(8).reliable().transient_local(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr message) {
        const auto & p=message->pose.position;const auto & q=message->pose.orientation;
        const double stamp=rclcpp::Time(message->header.stamp).seconds();
        const double age=node_->now().seconds()-stamp;
        if(message->header.frame_id!=planner_frame_ || age<-.1 || age>60 ||
           !std::isfinite(p.x+p.y+p.z+q.x+q.y+q.z+q.w) ||
           std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01)return;
        std::lock_guard<std::mutex> lock(map_mutex_);
        if(failed_turns_.size()>=8)failed_turns_.erase(failed_turns_.begin());
        failed_turns_.push_back({stamp,{p.x,p.y,p.z,tf2::getYaw(q),.25}});
        RCLCPP_WARN(node_->get_logger(),
          "Remembering failed turn at (%.3f, %.3f); require translation before large heading changes for 60s",
          p.x,p.y);
      });
  }

  unsupported_sub_=node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/navigation/unsupported_region",rclcpp::QoS(10).reliable(),
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      const auto & p=msg->pose.position;
      const double now=node_->now().seconds(),stamp=rclcpp::Time(msg->header.stamp).seconds();
      if(msg->header.frame_id!=planner_frame_ || !std::isfinite(p.x+p.y+p.z) ||
         now-stamp<-.1 || now-stamp>2.)return;
      std::lock_guard<std::mutex> lock(map_mutex_);
      for(auto & entry:unsupported_) {
        if(std::hypot(entry.second.x-p.x,entry.second.y-p.y)<.10 &&
           std::abs(entry.second.z-p.z)<.1){entry.first=now;return;}
      }
      if(unsupported_.size()>=32)unsupported_.erase(unsupported_.begin());
      unsupported_.push_back({now,{p.x,p.y,p.z,.18}});
      RCLCPP_WARN(node_->get_logger(),"Temporary support-uncertain region (%.3f, %.3f, %.3f); reroute for up to 60 simulated seconds",p.x,p.y,p.z);
    });
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  map_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, qos,
    std::bind(&Surface3DPlanner::mapCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: support-surface 3D A* on '%s' in frame '%s'",
    plugin_name_.c_str(), octomap_topic_.c_str(), planner_frame_.c_str());
}

void Surface3DPlanner::cleanup()
{
  map_sub_.reset();
  regions_sub_.reset();
  failed_turn_sub_.reset();failed_turns_.clear();unsupported_sub_.reset();unsupported_.clear();
  stair_regions_.clear();
  std::lock_guard<std::mutex> lock(map_mutex_);
  map_.reset();
}

void Surface3DPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activated %s", plugin_name_.c_str());
}

void Surface3DPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivated %s", plugin_name_.c_str());
}

void Surface3DPlanner::mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(*msg));
  if (!abstract) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to deserialize OctoMap message");
    return;
  }
  auto * raw_tree = dynamic_cast<octomap::OcTree *>(abstract.get());
  if (!raw_tree) {
    RCLCPP_ERROR(
      node_->get_logger(), "OctoMap type '%s' is not an OcTree", msg->id.c_str());
    return;
  }
  abstract.release();
  std::shared_ptr<const octomap::OcTree> next(raw_tree);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = std::move(next);map_received_at_=node_->now().seconds();
  }
}

geometry_msgs::msg::PoseStamped Surface3DPlanner::toPlannerFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::PoseStamped source = pose;
  if (source.header.frame_id.empty()) {
    source.header.frame_id = planner_frame_;
  }
  if (source.header.frame_id == planner_frame_) {
    return source;
  }
  try {
    return tf_->transform(source, planner_frame_, tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & error) {
    throw nav2_core::PlannerException(
            "Surface3DPlanner cannot transform from " + source.header.frame_id + " to " +
            planner_frame_ + ": " + error.what());
  }
}

nav_msgs::msg::Path Surface3DPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  std::shared_ptr<const octomap::OcTree> map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
  }
  if (!map) {
    throw nav2_core::PlannerException("Surface3DPlanner is waiting for /octomap_full");
  }

  const auto start_map = toPlannerFrame(start);
  const auto goal_map = toPlannerFrame(goal);
  auto config = search_config_;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    config.excluded_regions=stair_regions_;
    const double now=node_->now().seconds();
    failed_turns_.erase(std::remove_if(failed_turns_.begin(),failed_turns_.end(),
      [now](const auto & v){return now-v.first<0 || now-v.first>60;}),failed_turns_.end());
    for(const auto & failure:failed_turns_)config.turn_exclusions.push_back(failure.second);
  }
  const double budget=std::clamp(
    node_->get_parameter(plugin_name_+".planning_time_budget_s").as_double(),0.1,30.0);
  config.search_deadline=std::chrono::steady_clock::now()+
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(budget));
  if (config.infer_flat_support) {
    try {
      const auto feet=tf_->lookupTransform(planner_frame_, "base_footprint", tf2::TimePointZero);
      config.support_reference_height=start_map.pose.position.z-feet.transform.translation.z;
    } catch (const tf2::TransformException &) {
      throw nav2_core::PlannerException("Surface3DPlanner waiting for current stance transform");
    }
    configureSupportPlane(config, *map, {start_map.pose.position.x,
      start_map.pose.position.y,start_map.pose.position.z});
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    const double now=node_->now().seconds();
    SurfaceGraph verification(*map,config);
    unsupported_.erase(std::remove_if(unsupported_.begin(),unsupported_.end(),[&](const auto & entry) {
      const auto & region=entry.second;
      if(now-entry.first<0 || now-entry.first>60)return true;
      // A fresh 3D patch can release uncertainty early. A plane-only pass cannot.
      const SurfacePoint point{region.x,region.y,region.z,0};
      const auto check=verification.diagnosePose(point);
      return map_received_at_>entry.first && check.surface && check.support && check.clearance &&
        verification.inspectCloudSupport(point)==SurfaceGraph::CloudSupport::confirmed;
    }),unsupported_.end());
    for(const auto & entry:unsupported_)config.support_avoidance.push_back(entry.second);
  }
  SurfaceGraph graph(*map, config);
  std::vector<SurfacePoint> points;
  try {
    points = graph.plan(
    {start_map.pose.position.x, start_map.pose.position.y,
      config.infer_flat_support?start_map.pose.position.z-config.support_reference_height:start_map.pose.position.z,
      tf2::getYaw(start_map.pose.orientation)},
    {goal_map.pose.position.x, goal_map.pose.position.y, goal_map.pose.position.z,
      tf2::getYaw(goal_map.pose.orientation)});
  } catch (const SurfaceSearchTimeout & ex) {
    throw nav2_core::PlannerException(ex.what());
  }
  if (points.empty()) {
    throw nav2_core::PlannerException(
            "Surface3DPlanner found no support-connected 3D path");
  }

  auto verify_config=config;
  verify_config.search_deadline=std::chrono::steady_clock::time_point::max();
  SurfaceGraph nominal_graph(*map,verify_config);
  bool nominal_route=true;
  for (std::size_t i=1;i<points.size();++i) {
    if(!nominal_graph.isMotionValid(points[i-1],points[i])) {nominal_route=false;break;}
  }
  if(!nominal_route) {
    RCLCPP_WARN(node_->get_logger(),
      "Verified narrow passage route: nominal margin %.3fm, hard margin %.3fm; full body and support sweeps preserved",
      config.clearance_margin,config.narrow_clearance_margin);
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = planner_frame_;
  path.header.stamp = node_->now();
  path.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    pose.pose.position.z = points[i].z;
    const double yaw = points[i].yaw;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(orientation);
    path.poses.push_back(std::move(pose));
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "Surface pose A* (x,y,z,yaw): %zu poses, z %.2f -> %.2f",
    path.poses.size(), path.poses.front().pose.position.z, path.poses.back().pose.position.z);
  return path;
}

}  // namespace go2_surface_3d_planner

PLUGINLIB_EXPORT_CLASS(
  go2_surface_3d_planner::Surface3DPlanner,
  nav2_core::GlobalPlanner)
