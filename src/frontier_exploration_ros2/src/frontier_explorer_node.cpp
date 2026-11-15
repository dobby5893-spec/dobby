#include <iomanip>
#include <sstream>
#include "frontier_exploration_ros2/viewpoint_visibility_3d.hpp"
/*
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "frontier_exploration_ros2/frontier_explorer_node.hpp"
#include "frontier_exploration_ros2/nav2_compat.hpp"
#include "frontier_exploration_ros2/nearby_observation.hpp"
#include "frontier_exploration_ros2/mapped_observation.hpp"

#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rcutils/logging.h>
#include <rclcpp/duration.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <visualization_msgs/msg/marker.hpp>

namespace frontier_exploration_ros2
{

namespace
{

constexpr std::size_t kStartupMapRateSampleCount = 3;
constexpr double kStartupMapRateStabilityTolerance = 0.25;

// Adapts Nav2 goal handle API to the core's transport-agnostic interface.
template<class Action>
class OwnedActionHandleAdapter : public GoalHandleInterface
{
public:
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using CancelRequester = std::function<void(
    const std::shared_ptr<NavigateGoalHandle> &,
    std::function<void(bool accepted, const std::string & error_message)>)>;

  OwnedActionHandleAdapter(
    std::shared_ptr<NavigateGoalHandle> handle,
    CancelRequester cancel_requester)
  : handle_(std::move(handle)), cancel_requester_(std::move(cancel_requester))
  {
  }

  void cancel_goal_async(
    std::function<void(bool accepted, const std::string & error_message)> callback) override
  {
    cancel_requester_(handle_, std::move(callback));
  }

private:
  std::shared_ptr<NavigateGoalHandle> handle_;
  CancelRequester cancel_requester_;
};

using NavigateGoalHandleAdapter=OwnedActionHandleAdapter<nav2_msgs::action::NavigateToPose>;
using ClearanceGoalHandleAdapter=OwnedActionHandleAdapter<nav2_msgs::action::DriveOnHeading>;

}  // namespace

FrontierExplorerNode::FrontierExplorerNode(const rclcpp::NodeOptions & options)
: Node("frontier_explorer", options)
{
  // Declare full user-facing integration surface (topics, behavior, QoS, integration hooks).
  // Topic/action defaults are namespace-aware (no leading slash) for multi-robot composability.
  this->declare_parameter<std::string>("map_topic", "map");
  this->declare_parameter<std::string>("costmap_topic", "global_costmap/costmap");
  this->declare_parameter<std::string>("local_costmap_topic", "local_costmap/costmap");
  this->declare_parameter<std::string>("navigate_to_pose_action_name", "navigate_to_pose");
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
  clearance_client_=rclcpp_action::create_client<ClearanceAction>(
    this,declare_parameter<std::string>("clearance_recovery_action","/escape_move"));
  random_recovery_enabled_=declare_parameter<bool>("random_recovery_enabled",true);
  random_sampling_.minimum_distance=declare_parameter<double>("random_recovery_min_distance",.5);
  random_sampling_.initial_radius=declare_parameter<double>("random_recovery_initial_radius",1.5);
  random_sampling_.expanded_radius=declare_parameter<double>("random_recovery_expanded_radius",3.0);
  random_sampling_.avoid_radius=declare_parameter<double>("random_recovery_avoid_radius",.5);
  const int random_samples=declare_parameter<int>("random_recovery_samples_per_ring",48);
  if(random_samples<1 || random_samples>256)throw std::invalid_argument("Invalid recovery sample count");
  random_sampling_.samples_per_ring=random_samples;random_sampling_.validate();
  random_policy_.blocked_timeout=declare_parameter<double>("random_recovery_blocked_timeout_s",30.0);
  random_policy_.window=declare_parameter<double>("random_recovery_loop_window_s",90.0);
  random_policy_.cooldown=declare_parameter<double>("random_recovery_cooldown_s",30.0);
  random_policy_.max_attempts=declare_parameter<int>("random_recovery_max_attempts",3);
  random_policy_.batch_cooldown=declare_parameter<double>("random_recovery_batch_cooldown_s",120.0);
  random_recovery_budget_=declare_parameter<double>("random_recovery_search_budget_s",3.0);
  if(!std::isfinite(random_recovery_budget_) || random_recovery_budget_<.05 ||
    random_recovery_budget_>10 || !std::isfinite(random_policy_.blocked_timeout) ||
    random_policy_.blocked_timeout<5 || !std::isfinite(random_policy_.window) ||
    random_policy_.window<random_policy_.blocked_timeout || random_policy_.window>600 ||
    !std::isfinite(random_policy_.batch_cooldown) ||
    random_policy_.batch_cooldown<random_policy_.cooldown ||
    random_policy_.batch_cooldown>3600 ||
    !std::isfinite(random_policy_.cooldown) || random_policy_.cooldown<1 ||
    random_policy_.max_attempts<1 || random_policy_.max_attempts>10)
    throw std::invalid_argument("Invalid recovery timing/attempt configuration");
  random_stair_sub_=create_subscription<std_msgs::msg::Bool>("/multifloor/stair_mode",
    rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::SharedPtr message) {
      if(random_stair_mode_!=message->data) {
        random_stair_mode_=message->data;++random_epoch_;clearance_request_ready_=false;force_no_goal_recovery_=false;
        random_policy_.last_time=-1;random_avoid_.clear();
      }
    });
  surface_goal_validation_enabled_ = this->declare_parameter<bool>("surface_goal_validation_enabled",false);
  localization_watchdog_enabled_=declare_parameter<bool>("localization_watchdog_enabled",false);
  surface_goal_max_retreat_ = this->declare_parameter<double>("surface_goal_max_retreat",1.5);
  surface_goal_config_.snap_radius = 0.15;
  surface_goal_config_.infer_flat_support = true;
  transition_mask_sub_=create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/multifloor/transition_mask",rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg){
      if(msg->header.frame_id!=params_.global_frame || msg->data.size()!=size_t(msg->info.width)*msg->info.height)return;
      std::lock_guard<std::mutex> lock(surface_map_mutex_);transition_mask_=msg;
    });
  stair_regions_sub_=create_subscription<std_msgs::msg::Float64MultiArray>(
    "/multifloor/stair_regions",rclcpp::QoS(1).reliable().transient_local(),
    [this](std_msgs::msg::Float64MultiArray::SharedPtr m) {
      if(m->layout.dim.empty() || m->layout.dim[0].label!=params_.global_frame || m->data==stair_region_values_) return;
      try {
        auto regions=go2_surface_3d_planner::decodeSurfaceRegions(m->data);
        stair_region_values_=m->data;surface_goal_config_.excluded_regions=std::move(regions);
        ++surface_goal_epoch_; ++random_epoch_;clearance_request_ready_=false;force_no_goal_recovery_=false;surface_goal_queue_.clear();surface_goal_requests_.clear(); surface_goal_options_.clear(); surface_goal_exclusions_.clear();surface_goal_references_.clear();surface_goal_cache_.clear();
      } catch(const std::invalid_argument & e) {RCLCPP_WARN(get_logger(),"%s",e.what());}
    });
  surface_goal_config_.continuous_floor_region = this->declare_parameter<std::vector<double>>(
    "continuous_floor_region", std::vector<double>{});
  surface_goal_config_.max_iterations = 50000;
  surface_goal_budget_s_=std::clamp(
    declare_parameter<double>("surface_goal_validation_budget_s",5.0),0.05,10.0);
  surface_goal_queue_ = ValidationRetryQueue(surface_goal_budget_s_, 10.0);
  surface_goal_config_.clearance_margin = this->declare_parameter<double>("surface_goal_margin",0.01);
  surface_goal_config_.narrow_clearance_margin = this->declare_parameter<double>("surface_goal_narrow_margin",0.0);
  this->declare_parameter<std::string>("frontier_marker_topic", "explore/frontiers");
  this->declare_parameter<std::string>("selected_frontier_topic", "explore/selected_frontier");
  this->declare_parameter<std::string>("optimized_map_topic", "explore/optimized_map");
  this->declare_parameter<std::string>("map_qos_durability", "transient_local");
  this->declare_parameter<std::string>("map_qos_reliability", "reliable");
  this->declare_parameter<int>("map_qos_depth", 1);
  this->declare_parameter<bool>("map_qos_autodetect_on_startup", false);
  this->declare_parameter<double>("map_qos_autodetect_timeout_s", 2.0);
  this->declare_parameter<std::string>("costmap_qos_reliability", "reliable");
  this->declare_parameter<int>("costmap_qos_depth", 10);
  this->declare_parameter<std::string>("local_costmap_qos_reliability", "inherit");
  this->declare_parameter<int>("local_costmap_qos_depth", -1);
  this->declare_parameter<double>("frontier_marker_scale", 0.15);
  this->declare_parameter<bool>("autostart", true);
  this->declare_parameter<bool>("control_service_enabled", true);
  this->declare_parameter<bool>("frontier_map_optimization_enabled", true);
  this->declare_parameter<double>("sigma_s", 2.0);
  this->declare_parameter<double>("sigma_r", 30.0);
  this->declare_parameter<int>("dilation_kernel_radius_cells", 1);
  this->declare_parameter<double>("sensor_effective_range_m", 1.5);
  this->declare_parameter<double>("weight_distance_wd", 1.0);
  this->declare_parameter<double>("weight_gain_ws", 1.0);
  this->declare_parameter<double>("max_linear_speed_vmax", 0.5);
  this->declare_parameter<double>("max_angular_speed_wmax", 1.0);
  // Solver selection stays MRTSP-specific, while the bounded-DP limits are named
  // by algorithm so the same controls can describe other DP-based route policies.
  this->declare_parameter<std::string>("mrtsp_solver", "dp");
  this->declare_parameter<int>("dp_solver_candidate_limit", 15);
  this->declare_parameter<int>("dp_planning_horizon", 10);
  this->declare_parameter<bool>("local_completion_enabled", true);
  this->declare_parameter<double>("local_completion_path_slack_m", 2.0);
  this->declare_parameter<int>("occ_threshold", 50);
  this->declare_parameter<int>("min_frontier_size_cells", 5);
  this->declare_parameter<double>("frontier_candidate_min_goal_distance_m", 0.0);
  this->declare_parameter<bool>("viewpoint_generation_enabled", true);
  params_.floor_cleanup_enabled=this->declare_parameter<bool>("floor_cleanup_enabled",false);
  params_.floor_no_gain_timeout_s=this->declare_parameter<double>("floor_no_gain_timeout_s",180.);
  params_.no_next_goal_timeout_s=this->declare_parameter<double>("no_next_goal_timeout_s",300.);
  if(!std::isfinite(params_.no_next_goal_timeout_s) || params_.no_next_goal_timeout_s<5.)
    throw std::invalid_argument("no_next_goal_timeout_s must be finite and >=5 simulated seconds");
  params_.floor_cleanup_budget_s=this->declare_parameter<double>("floor_cleanup_budget_s",180.);
  const int cleanup_attempts=this->declare_parameter<int>("floor_cleanup_min_attempts",4);
  if(!std::isfinite(params_.floor_no_gain_timeout_s) || params_.floor_no_gain_timeout_s<5 ||
    !std::isfinite(params_.floor_cleanup_budget_s) || params_.floor_cleanup_budget_s<5 || cleanup_attempts<1)
    throw std::invalid_argument("Invalid floor cleanup budgets");
  params_.floor_cleanup_min_attempts=cleanup_attempts;
  visibility_3d_enabled_=this->declare_parameter<bool>("viewpoint_3d_visibility_enabled",false);
  visibility_sensor_frame_=this->declare_parameter<std::string>("viewpoint_sensor_frame","livox_frame");
  visibility_min_pitch_=this->declare_parameter<double>("viewpoint_sensor_min_pitch_deg",-7.22);
  visibility_max_pitch_=this->declare_parameter<double>("viewpoint_sensor_max_pitch_deg",55.22);
  if(!std::isfinite(visibility_min_pitch_+visibility_max_pitch_) ||
    visibility_min_pitch_<-90 || visibility_max_pitch_>90 || visibility_min_pitch_>=visibility_max_pitch_)
    throw std::invalid_argument("Invalid viewpoint sensor vertical field of view");
  this->declare_parameter<std::vector<double>>(
    "viewpoint_distances_m",
    std::vector<double>{0.35, 0.50, 0.80});
  this->declare_parameter<double>("viewpoint_clearance_m", 0.30);
  this->declare_parameter<double>("information_gain_saturation_cells", 80.0);
  this->declare_parameter<bool>("reachability_filter_enabled", true);
  this->declare_parameter<double>("reachability_costmap_weight", 2.0);
  this->declare_parameter<int>("reachability_max_expanded_cells", 200000);
  this->declare_parameter<bool>("reachability_allow_diagonal", true);
  this->declare_parameter<bool>("geometry_scoring_enabled", true);
  this->declare_parameter<std::string>("geometry_point_cloud_topic", "/cloud_registered");
  this->declare_parameter<int>("geometry_max_points", 30000);
  this->declare_parameter<double>("geometry_analysis_radius_m", 4.0);
  this->declare_parameter<double>("geometry_local_floor_radius_m", 1.25);
  this->declare_parameter<double>("geometry_wall_min_height_m", 0.25);
  this->declare_parameter<double>("geometry_wall_max_height_m", 2.0);
  this->declare_parameter<double>("geometry_ceiling_min_height_m", 1.6);
  this->declare_parameter<double>("geometry_ceiling_max_height_m", 3.2);
  this->declare_parameter<int>("geometry_minimum_nearby_points", 30);
  this->declare_parameter<bool>("multi_factor_scoring_enabled", true);
  this->declare_parameter<double>("score_weight_information_gain", 2.0);
  this->declare_parameter<double>("score_weight_topological_gain", 1.5);
  this->declare_parameter<double>("score_weight_enclosure", 0.8);
  this->declare_parameter<double>("score_weight_path_cost", 0.8);
  this->declare_parameter<double>("score_weight_open_space", 1.0);
  this->declare_parameter<double>("score_weight_traversability_risk", 0.6);
  this->declare_parameter<double>("score_path_cost_scale_m", 5.0);
  this->declare_parameter<bool>("topology_scoring_enabled", true);
  this->declare_parameter<double>("topology_minimum_entrance_width_m", 0.65);
  this->declare_parameter<double>("topology_doorway_max_width_m", 1.8);
  this->declare_parameter<double>("topology_corridor_max_width_m", 3.5);
  this->declare_parameter<double>("topology_lateral_probe_distance_m", 3.0);
  this->declare_parameter<double>("topology_node_merge_distance_m", 1.0);
  this->declare_parameter<double>("topology_node_merge_height_m", 1.0);
  this->declare_parameter<double>("topology_visit_radius_m", 0.8);
  this->declare_parameter<std::vector<double>>(
    "exploration_boundary_polygon", std::vector<double>{});
  this->declare_parameter<double>("exploration_boundary_margin_m", 0.0);
  this->declare_parameter<double>("frontier_selection_min_distance", 0.5);
  this->declare_parameter<bool>("escape_enabled", false);
  this->declare_parameter<double>("frontier_visit_tolerance", 0.30);
  this->declare_parameter<bool>("goal_preemption_enabled", false);
  this->declare_parameter<bool>("goal_skip_on_blocked_goal", true);
  this->declare_parameter<double>("goal_preemption_min_interval_s", 2.0);
  this->declare_parameter<double>("goal_preemption_lidar_range_m", 12.0);
  this->declare_parameter<double>("goal_preemption_lidar_fov_deg", 360.0);
  this->declare_parameter<double>("goal_preemption_lidar_ray_step_deg", 1.0);
  this->declare_parameter<double>("goal_preemption_complete_if_within_m", 0.0);
  this->declare_parameter<double>("goal_preemption_lidar_min_reveal_length_m", 0.5);
  this->declare_parameter<double>("goal_preemption_lidar_yaw_offset_deg", 0.0);
  this->declare_parameter<bool>("region_observation_tasks_enabled", false);
  this->declare_parameter<double>("region_no_gain_wait_s", 5.0);
  this->declare_parameter<double>("region_no_gain_budget_s", 60.0);
  this->declare_parameter<int>("region_minimum_observation_positions", 3);
  this->declare_parameter<double>("region_retry_cooldown_s", 60.0);
  this->declare_parameter<bool>("post_goal_settle_enabled", true);
  this->declare_parameter<double>("post_goal_min_settle", 0.80);
  this->declare_parameter<double>("map_processing_rate_hz", 1.0);
  this->declare_parameter<bool>("return_to_start_on_complete", true);
  this->declare_parameter<std::string>("all_frontiers_suppressed_behavior", "stay");
  this->declare_parameter<bool>("frontier_suppression_enabled", true);
  this->declare_parameter<int>("frontier_suppression_attempt_threshold", 1);
  this->declare_parameter<double>("frontier_suppression_base_size_m", 1.2);
  this->declare_parameter<double>("frontier_suppression_expansion_size_m", 0.5);
  this->declare_parameter<double>("frontier_suppression_timeout_s", 120.0);
  this->declare_parameter<double>("frontier_suppression_no_progress_timeout_s", 8.0);
  this->declare_parameter<double>("frontier_suppression_progress_epsilon_m", 0.08);
  this->declare_parameter<double>("frontier_suppression_startup_grace_period_s", 15.0);
  this->declare_parameter<int>("frontier_suppression_max_attempt_records", 256);
  this->declare_parameter<int>("frontier_suppression_max_regions", 64);
  this->declare_parameter<bool>("completion_event_enabled", false);
  this->declare_parameter<std::string>("completion_event_topic", "exploration_complete");

  // Read navigation/exploration behavior parameters first; QoS parsing is handled separately.
  params_.map_topic = this->get_parameter("map_topic").as_string();
  params_.costmap_topic = this->get_parameter("costmap_topic").as_string();
  params_.local_costmap_topic = this->get_parameter("local_costmap_topic").as_string();
  params_.navigate_to_pose_action_name = this->get_parameter("navigate_to_pose_action_name").as_string();
  params_.global_frame = this->get_parameter("global_frame").as_string();
  params_.robot_base_frame = this->get_parameter("robot_base_frame").as_string();
  params_.frontier_marker_topic = this->get_parameter("frontier_marker_topic").as_string();
  params_.selected_frontier_topic = this->get_parameter("selected_frontier_topic").as_string();
  params_.optimized_map_topic = this->get_parameter("optimized_map_topic").as_string();
  params_.frontier_marker_scale = this->get_parameter("frontier_marker_scale").as_double();
  autostart_ = this->get_parameter("autostart").as_bool();
  control_service_enabled_ = this->get_parameter("control_service_enabled").as_bool();
  if (!autostart_ && !control_service_enabled_) {
    control_service_enabled_ = true;
    RCLCPP_WARN(
      this->get_logger(),
      "control_service_enabled=false is ignored when autostart=false; control service remains enabled");
  }
  params_.frontier_map_optimization_enabled = this->get_parameter(
    "frontier_map_optimization_enabled").as_bool();
  params_.sigma_s = this->get_parameter("sigma_s").as_double();
  params_.sigma_r = this->get_parameter("sigma_r").as_double();
  params_.dilation_kernel_radius_cells = this->get_parameter("dilation_kernel_radius_cells").as_int();
  params_.sensor_effective_range_m = this->get_parameter("sensor_effective_range_m").as_double();
  params_.weight_distance_wd = this->get_parameter("weight_distance_wd").as_double();
  params_.weight_gain_ws = this->get_parameter("weight_gain_ws").as_double();
  params_.max_linear_speed_vmax = this->get_parameter("max_linear_speed_vmax").as_double();
  params_.max_angular_speed_wmax = this->get_parameter("max_angular_speed_wmax").as_double();
  params_.mrtsp_solver = this->get_parameter("mrtsp_solver").as_string();
  // ROS parameters are read as signed integers, then converted to positive size_t
  // limits before they enter the solver configuration.
  const auto dp_solver_candidate_limit = this->get_parameter(
    "dp_solver_candidate_limit").as_int();
  params_.dp_solver_candidate_limit = dp_solver_candidate_limit > 0 ?
    static_cast<std::size_t>(dp_solver_candidate_limit) : 1U;
  const auto dp_planning_horizon = this->get_parameter("dp_planning_horizon").as_int();
  params_.dp_planning_horizon = dp_planning_horizon > 0 ?
    static_cast<std::size_t>(dp_planning_horizon) : 1U;
  params_.local_completion_enabled = this->get_parameter(
    "local_completion_enabled").as_bool();
  params_.local_completion_path_slack_m = this->get_parameter(
    "local_completion_path_slack_m").as_double();
  if (!std::isfinite(params_.local_completion_path_slack_m) ||
    params_.local_completion_path_slack_m < 0.0)
  {
    throw std::runtime_error(
            "local_completion_path_slack_m must be finite and non-negative");
  }
  params_.occ_threshold = this->get_parameter("occ_threshold").as_int();
  params_.min_frontier_size_cells = this->get_parameter("min_frontier_size_cells").as_int();
  params_.frontier_candidate_min_goal_distance_m = this->get_parameter(
    "frontier_candidate_min_goal_distance_m").as_double();
  params_.viewpoint_generation_enabled = this->get_parameter(
    "viewpoint_generation_enabled").as_bool();
  params_.viewpoint_distances_m = this->get_parameter(
    "viewpoint_distances_m").as_double_array();
  params_.viewpoint_clearance_m = this->get_parameter(
    "viewpoint_clearance_m").as_double();
  params_.information_gain_saturation_cells = this->get_parameter(
    "information_gain_saturation_cells").as_double();
  params_.reachability_filter_enabled = this->get_parameter(
    "reachability_filter_enabled").as_bool();
  params_.reachability_costmap_weight = this->get_parameter(
    "reachability_costmap_weight").as_double();
  const auto max_expanded_cells = this->get_parameter(
    "reachability_max_expanded_cells").as_int();
  params_.reachability_max_expanded_cells = max_expanded_cells > 0 ?
    static_cast<std::size_t>(max_expanded_cells) : 1U;
  params_.reachability_allow_diagonal = this->get_parameter(
    "reachability_allow_diagonal").as_bool();
  params_.geometry_scoring_enabled = this->get_parameter(
    "geometry_scoring_enabled").as_bool();
  params_.geometry_point_cloud_topic = this->get_parameter(
    "geometry_point_cloud_topic").as_string();
  const auto geometry_max_points = this->get_parameter("geometry_max_points").as_int();
  params_.geometry_max_points = geometry_max_points > 0 ?
    static_cast<std::size_t>(geometry_max_points) : 1U;
  params_.geometry_analysis_radius_m = this->get_parameter(
    "geometry_analysis_radius_m").as_double();
  params_.geometry_local_floor_radius_m = this->get_parameter(
    "geometry_local_floor_radius_m").as_double();
  params_.geometry_wall_min_height_m = this->get_parameter(
    "geometry_wall_min_height_m").as_double();
  params_.geometry_wall_max_height_m = this->get_parameter(
    "geometry_wall_max_height_m").as_double();
  params_.geometry_ceiling_min_height_m = this->get_parameter(
    "geometry_ceiling_min_height_m").as_double();
  params_.geometry_ceiling_max_height_m = this->get_parameter(
    "geometry_ceiling_max_height_m").as_double();
  const auto geometry_minimum_nearby_points = this->get_parameter(
    "geometry_minimum_nearby_points").as_int();
  params_.geometry_minimum_nearby_points = geometry_minimum_nearby_points > 0 ?
    static_cast<std::size_t>(geometry_minimum_nearby_points) : 1U;
  params_.multi_factor_scoring_enabled = this->get_parameter(
    "multi_factor_scoring_enabled").as_bool();
  params_.frontier_score_weights.information_gain = this->get_parameter(
    "score_weight_information_gain").as_double();
  params_.frontier_score_weights.topological_gain = this->get_parameter(
    "score_weight_topological_gain").as_double();
  params_.frontier_score_weights.enclosure = this->get_parameter(
    "score_weight_enclosure").as_double();
  params_.frontier_score_weights.path_cost = this->get_parameter(
    "score_weight_path_cost").as_double();
  params_.frontier_score_weights.open_space = this->get_parameter(
    "score_weight_open_space").as_double();
  params_.frontier_score_weights.traversability_risk = this->get_parameter(
    "score_weight_traversability_risk").as_double();
  params_.frontier_score_weights.path_cost_scale_m = this->get_parameter(
    "score_path_cost_scale_m").as_double();
  params_.topology_scoring_enabled = this->get_parameter(
    "topology_scoring_enabled").as_bool();
  params_.topology_config.minimum_entrance_width_m = this->get_parameter(
    "topology_minimum_entrance_width_m").as_double();
  params_.topology_config.doorway_max_width_m = this->get_parameter(
    "topology_doorway_max_width_m").as_double();
  params_.topology_config.corridor_max_width_m = this->get_parameter(
    "topology_corridor_max_width_m").as_double();
  params_.topology_config.lateral_probe_distance_m = this->get_parameter(
    "topology_lateral_probe_distance_m").as_double();
  params_.topology_config.node_merge_distance_m = this->get_parameter(
    "topology_node_merge_distance_m").as_double();
  params_.topology_config.node_merge_height_m = this->get_parameter(
    "topology_node_merge_height_m").as_double();
  params_.topology_config.visit_radius_m = this->get_parameter(
    "topology_visit_radius_m").as_double();
  const auto boundary_values = this->get_parameter(
    "exploration_boundary_polygon").as_double_array();
  if (!boundary_values.empty() &&
    (boundary_values.size() < 6 || boundary_values.size() % 2 != 0))
  {
    throw std::runtime_error(
            "exploration_boundary_polygon must be empty or contain at least three x,y pairs");
  }
  params_.exploration_boundary_polygon.reserve(boundary_values.size() / 2);
  for (std::size_t i = 0; i < boundary_values.size(); i += 2) {
    if (!std::isfinite(boundary_values[i]) || !std::isfinite(boundary_values[i + 1])) {
      throw std::runtime_error("exploration_boundary_polygon values must be finite");
    }
    params_.exploration_boundary_polygon.emplace_back(
      boundary_values[i], boundary_values[i + 1]);
  }
  params_.exploration_boundary_margin_m = this->get_parameter(
    "exploration_boundary_margin_m").as_double();
  if (!std::isfinite(params_.exploration_boundary_margin_m) ||
    params_.exploration_boundary_margin_m < 0.0)
  {
    throw std::runtime_error("exploration_boundary_margin_m must be finite and non-negative");
  }
  params_.frontier_selection_min_distance = this->get_parameter(
    "frontier_selection_min_distance").as_double();
  params_.escape_enabled = this->get_parameter("escape_enabled").as_bool();
  params_.frontier_visit_tolerance = this->get_parameter("frontier_visit_tolerance").as_double();
  params_.goal_preemption_enabled = this->get_parameter(
    "goal_preemption_enabled").as_bool();
  params_.goal_skip_on_blocked_goal = this->get_parameter("goal_skip_on_blocked_goal").as_bool();
  params_.goal_preemption_min_interval_s = this->get_parameter("goal_preemption_min_interval_s").as_double();
  params_.goal_preemption_lidar_range_m = this->get_parameter(
    "goal_preemption_lidar_range_m").as_double();
  params_.goal_preemption_lidar_fov_deg = this->get_parameter(
    "goal_preemption_lidar_fov_deg").as_double();
  params_.goal_preemption_lidar_ray_step_deg = this->get_parameter(
    "goal_preemption_lidar_ray_step_deg").as_double();
  params_.goal_preemption_complete_if_within_m = this->get_parameter(
    "goal_preemption_complete_if_within_m").as_double();
  params_.goal_preemption_lidar_min_reveal_length_m = this->get_parameter(
    "goal_preemption_lidar_min_reveal_length_m").as_double();
  params_.goal_preemption_lidar_yaw_offset_deg = this->get_parameter(
    "goal_preemption_lidar_yaw_offset_deg").as_double();
  params_.region_observation_tasks_enabled=get_parameter("region_observation_tasks_enabled").as_bool();
  params_.region_no_gain_wait_s=get_parameter("region_no_gain_wait_s").as_double();
  params_.region_no_gain_budget_s=get_parameter("region_no_gain_budget_s").as_double();
  params_.region_minimum_observation_positions=get_parameter("region_minimum_observation_positions").as_int();
  params_.region_retry_cooldown_s=get_parameter("region_retry_cooldown_s").as_double();
  if(!std::isfinite(params_.region_no_gain_wait_s) || params_.region_no_gain_wait_s<=0 ||
     !std::isfinite(params_.region_no_gain_budget_s) || params_.region_no_gain_budget_s<params_.region_no_gain_wait_s ||
     params_.region_minimum_observation_positions<3 || params_.region_minimum_observation_positions>64 ||
     !std::isfinite(params_.region_retry_cooldown_s) || params_.region_retry_cooldown_s<=0)
    throw std::invalid_argument("Invalid regional observation budget");
  params_.post_goal_settle_enabled = this->get_parameter("post_goal_settle_enabled").as_bool();
  params_.post_goal_min_settle = this->get_parameter("post_goal_min_settle").as_double();
  params_.map_processing_rate_hz = this->get_parameter("map_processing_rate_hz").as_double();
  params_.return_to_start_on_complete = this->get_parameter("return_to_start_on_complete").as_bool();
  params_.all_frontiers_suppressed_behavior = this->get_parameter(
    "all_frontiers_suppressed_behavior").as_string();
  params_.frontier_suppression_enabled = this->get_parameter("frontier_suppression_enabled").as_bool();
  params_.frontier_suppression_attempt_threshold = this->get_parameter(
    "frontier_suppression_attempt_threshold").as_int();
  params_.frontier_suppression_base_size_m = this->get_parameter(
    "frontier_suppression_base_size_m").as_double();
  params_.frontier_suppression_expansion_size_m = this->get_parameter(
    "frontier_suppression_expansion_size_m").as_double();
  params_.frontier_suppression_timeout_s = this->get_parameter(
    "frontier_suppression_timeout_s").as_double();
  params_.frontier_suppression_no_progress_timeout_s = this->get_parameter(
    "frontier_suppression_no_progress_timeout_s").as_double();
  params_.frontier_suppression_progress_epsilon_m = this->get_parameter(
    "frontier_suppression_progress_epsilon_m").as_double();
  params_.frontier_suppression_startup_grace_period_s = this->get_parameter(
    "frontier_suppression_startup_grace_period_s").as_double();
  params_.frontier_suppression_max_attempt_records = this->get_parameter(
    "frontier_suppression_max_attempt_records").as_int();
  params_.frontier_suppression_max_regions = this->get_parameter(
    "frontier_suppression_max_regions").as_int();
  completion_event_config_.enabled = this->get_parameter("completion_event_enabled").as_bool();
  completion_event_config_.topic = this->get_parameter("completion_event_topic").as_string();
  if (completion_event_config_.enabled && completion_event_config_.topic.empty()) {
    throw std::runtime_error(
            "completion_event_topic must be set when completion_event_enabled=true");
  }
  // At this point, params_ contains only behavior parameters; QoS is parsed separately below.

  try {
    // Parse and validate QoS strings once at startup for explicit, predictable runtime profiles.
    topic_qos_profiles_ = resolve_topic_qos_profiles(
      this->get_parameter("map_qos_durability").as_string(),
      this->get_parameter("map_qos_reliability").as_string(),
      this->get_parameter("map_qos_depth").as_int(),
      this->get_parameter("costmap_qos_reliability").as_string(),
      this->get_parameter("costmap_qos_depth").as_int(),
      this->get_parameter("local_costmap_qos_reliability").as_string(),
      this->get_parameter("local_costmap_qos_depth").as_int());
  } catch (const std::exception & exc) {
    throw std::runtime_error(std::string("Invalid QoS parameter configuration: ") + exc.what());
  }

  map_qos_autodetect_on_startup_ = this->get_parameter("map_qos_autodetect_on_startup").as_bool();
  map_qos_autodetect_timeout_s_ = std::max(
    0.2,
    this->get_parameter("map_qos_autodetect_timeout_s").as_double());
  // Timeout lower bound avoids too-fast timer churn in startup autodetect mode.

  navigate_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this,
    params_.navigate_to_pose_action_name);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (completion_event_config_.enabled) {
    auto completion_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    completion_qos.reliable();
    completion_qos.transient_local();
    completion_event_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      completion_event_config_.topic,
      completion_qos);
  }
  frontier_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    params_.frontier_marker_topic,
    10);
  selected_frontier_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    params_.selected_frontier_topic,
    10);
  optimized_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    params_.optimized_map_topic,
    10);
  if (control_service_enabled_) {
    control_service_ = this->create_service<srv::ControlExploration>(
      "control_exploration",
      std::bind(
        &FrontierExplorerNode::handleControlRequest,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    RCLCPP_INFO(
      this->get_logger(),
      "Control service ready: '%s'",
      control_service_->get_service_name());
  } else {
    RCLCPP_INFO(this->get_logger(), "Control service is disabled by configuration");
  }

  // Core callbacks keep core logic independent from ROS transport and threading details.
  FrontierExplorerCoreCallbacks callbacks;
  callbacks.now_ns = [this]() {return this->get_clock()->now().nanoseconds();};
  callbacks.get_current_pose = [this]() {return this->getCurrentPose();};
  if (surface_goal_validation_enabled_) {
    surface_map_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "/octomap_full",rclcpp::QoS(1).reliable().transient_local(),
      [this](octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
        if (msg->header.frame_id != params_.global_frame) {return;}
        std::unique_ptr<octomap::AbstractOcTree> decoded(octomap_msgs::msgToMap(*msg));
        auto * tree = dynamic_cast<octomap::OcTree *>(decoded.get());
        if (!tree) {return;}
        decoded.release();
        std::shared_ptr<const octomap::OcTree> snapshot(tree);
        std::lock_guard<std::mutex> lock(surface_map_mutex_);
        surface_map_ = std::move(snapshot);
        surface_map_time_s_=rclcpp::Time(msg->header.stamp).seconds();
      });
    callbacks.frontier_goal_validation_pending = [this]() {
        return surface_goal_future_.valid() ||
          surface_goal_queue_.hasReady(std::chrono::steady_clock::now());
      };
    callbacks.validate_frontier_goal = [this](const geometry_msgs::msg::Pose & start,
      const geometry_msgs::msg::PoseStamped & goal) -> std::optional<geometry_msgs::msg::PoseStamped> {
      const auto now=std::chrono::steady_clock::now();
      const auto yaw = [](const geometry_msgs::msg::Quaternion & q) {
          return std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
        };
      // Small stance oscillations must not erase timeout history. A substantial
      // relocation starts a new validation context; old worker results are ignored.
      if (surface_retry_anchor_ &&
        (std::hypot(start.position.x-surface_retry_anchor_->position.x,
          start.position.y-surface_retry_anchor_->position.y)>0.50 ||
        std::abs(start.position.z-surface_retry_anchor_->position.z)>0.15 ||
        std::abs(std::remainder(yaw(start.orientation)-yaw(surface_retry_anchor_->orientation),2*M_PI))>0.50))
      {
        ++surface_goal_epoch_; ++random_epoch_;clearance_request_ready_=false;force_no_goal_recovery_=false; surface_goal_queue_.clear();
        surface_goal_requests_.clear(); surface_goal_options_.clear(); surface_goal_exclusions_.clear();surface_goal_references_.clear(); surface_goal_cache_.clear();
        surface_retry_anchor_=start;
      }
      if (!surface_retry_anchor_) {surface_retry_anchor_=start;}
      serviceSurfaceValidation();
      const std::string key=std::to_string(std::llround(goal.pose.position.x/0.05))+":"+
        std::to_string(std::llround(goal.pose.position.y/0.05))+":"+
        std::to_string(std::llround(yaw(goal.pose.orientation)/0.05))+
        [&]() {std::string suffix;
          for(const auto & option:submitted_surface_options_)
            suffix+=":"+std::to_string(std::llround(option.pose.position.x/.05))+","+
              std::to_string(std::llround(option.pose.position.y/.05));
          for(const auto & point:observation_exclusions_)
            suffix+=":avoid"+std::to_string(std::llround(point.first/.05))+","+
              std::to_string(std::llround(point.second/.05));
          suffix+=":ref"+std::to_string(std::llround(observation_reference_.first/.05))+","+
            std::to_string(std::llround(observation_reference_.second/.05));
          suffix+=floor_cleanup_mode_?":cleanup":":explore";
          return suffix;}();
      for(auto it=surface_goal_cache_.begin();it!=surface_goal_cache_.end();) {
        const auto & old=it->second.start;
        if (std::chrono::duration<double>(now-it->second.completed).count()>3.0 ||
          std::hypot(old.position.x-start.position.x,old.position.y-start.position.y)>0.10 ||
          std::abs(old.position.z-start.position.z)>0.05 ||
          std::abs(std::remainder(yaw(old.orientation)-yaw(start.orientation),2*M_PI))>0.10)
        {it=surface_goal_cache_.erase(it);} else {++it;}
      }
      const auto cached=surface_goal_cache_.find(key);
      if(cached!=surface_goal_cache_.end()) {
        if(cached->second.goal) {
          const auto & point=cached->second.goal->pose.position;
          RCLCPP_INFO(get_logger(),"Verified exploration observation point (%.2f, %.2f, %.2f)",
            point.x,point.y,point.z);
        }
        return cached->second.goal;
      }
      surface_goal_requests_[key]=goal;
      surface_goal_references_[key]=observation_reference_;
      surface_goal_exclusions_[key]=observation_exclusions_;
      surface_goal_options_[key]=submitted_surface_options_.empty() ?
        std::vector<geometry_msgs::msg::PoseStamped>{goal}:submitted_surface_options_;
      surface_goal_queue_.enqueue(key,now);
      serviceSurfaceValidation();
      return std::nullopt;
    };
    callbacks.update_observation_exclusions=[this](const auto & exclusions) {
      observation_exclusions_=exclusions;
    };
    const auto validate_single=callbacks.validate_frontier_goal;
    callbacks.validate_frontier_goal_set=
      [this,validate_single](const geometry_msgs::msg::Pose & start,
        const std::vector<geometry_msgs::msg::PoseStamped> & options) {
        if(options.empty())return std::optional<geometry_msgs::msg::PoseStamped>{};
        submitted_surface_options_=options;
        const auto result=validate_single(start,options.front());
        submitted_surface_options_.clear();
        return result;
      };
  }

  floor_exhausted_pub_=create_publisher<std_msgs::msg::String>(
    "/explore/floor_exhausted",rclcpp::QoS(1).reliable().transient_local());
  verified_completion_pub_=create_publisher<std_msgs::msg::String>(
    "/explore/floor_completion",rclcpp::QoS(1).reliable().transient_local());
  paired_floor_sub_=create_subscription<msg::FloorMapSnapshot>(
    params_.map_topic + "/snapshot",rclcpp::QoS(1).reliable().transient_local(),
    [this](msg::FloorMapSnapshot::ConstSharedPtr msg) {
      if(msg->floor_map.header.frame_id!=params_.global_frame)return;
      auto pair=decodeFloorSnapshot(*msg);if(!pair)return;
      std::lock_guard<std::mutex> lock(surface_map_mutex_);paired_floor_map_=std::move(pair);
    });
  callbacks.request_floor_completion_audit=[this]() {floor_cleanup_mode_=true;};
  callbacks.on_floor_exhausted=[this](const std::string & report) {
    std_msgs::msg::String msg;msg.data=report;floor_exhausted_pub_->publish(msg);
    RCLCPP_WARN(get_logger(),"Floor cleanup deferred residuals; requesting stair evaluation: %s",report.c_str());
  };
  callbacks.floor_cleanup_data_ready=[this]() {
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr hint;
    {std::lock_guard<std::mutex> lock(surface_map_mutex_);
      const double age=get_clock()->now().seconds()-surface_map_time_s_;
      if(!surface_map_ || surface_map_time_s_<0 || age<-.1 || age>3.) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
          "Floor evaluation waiting for fresh 3D map; age %.3fs",age);
        return false;
      }
      hint=surface_hint_map_;
    }
    if(!hint) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Floor evaluation waiting for /map_floor");
      return false;
    }
    const double age=get_clock()->now().seconds()-rclcpp::Time(hint->header.stamp).seconds();
    if(age<-.1 || age>3.) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,"Floor evaluation waiting for fresh /map_floor; age %.3fs",age);
      return false;
    }
    if(visibility_3d_enabled_) {
      try {tf_buffer_->lookupTransform("base_footprint",visibility_sensor_frame_,tf2::TimePointZero);}
      catch(const tf2::TransformException & error) {
        RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),5000,
          "Floor evaluation waiting for sensor transform: %s",error.what());return false;
      }
    }
    return true;
  };
  callbacks.residual_handoff_pose_safe=[this]() {return measuredClearanceRestored();};
  callbacks.set_floor_cleanup_mode=[this](bool enabled) {floor_cleanup_mode_=enabled;};
  callbacks.set_observation_reference=[this](auto point) {observation_reference_=point;};

  callbacks.wait_for_action_server = [this](double timeout_sec) {
      // Core uses a bounded wait to avoid hard-blocking in dispatch path.
      return navigate_to_pose_client_->wait_for_action_server(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(timeout_sec)));
    };
  callbacks.dispatch_goal_request = [this](const GoalDispatchRequest & request) {
      this->dispatchGoalRequest(request);
    };
  callbacks.publish_frontier_markers = [this](const FrontierSequence & frontiers) {
      this->publishFrontierMarkers(frontiers);
    };
  if(surface_goal_validation_enabled_)callbacks.request_symmetric_relocation_step=[this](const auto & current,RelocationPoint target) {
    return requestSymmetricRelocationStep(current,target);
  };
  callbacks.on_no_next_goal_timeout=[this]() {
    if(random_recovery_enabled_ && !random_stair_mode_ &&
       runtime_state_==RuntimeState::RUNNING)force_no_goal_recovery_=true;
  };
  callbacks.request_random_recovery=[this](const geometry_msgs::msg::Pose & current,bool blocked) {
    return requestRandomRecovery(current,blocked);
  };
  callbacks.take_clearance_recovery_request=[this]() {return takeClearanceRecoveryRequest();};
  callbacks.wait_for_clearance_server=[this](double) {
    return clearance_client_->action_server_is_ready();
  };
  callbacks.on_navigation_stall=[this](const geometry_msgs::msg::Pose & measured) {
    if(!random_recovery_enabled_ || random_stair_mode_ ||
       runtime_state_!=RuntimeState::RUNNING)return;
    random_policy_.motion.failedGoal(get_clock()->now().seconds(),
      measured.position.x,measured.position.y);
    RCLCPP_WARN(get_logger(),"Cross-goal stagnation: %u stalled goals in %.2fm; ordinary dispatch does not erase measured stagnation",
      random_policy_.motion.failures,random_policy_.motion.radius);
  };
  callbacks.random_recovery_pending=[this]() {
    return random_recovery_enabled_ && !random_stair_mode_ && random_future_.valid();
  };
  callbacks.publish_recovery_pose = [this](const auto & pose) {publishSelectedFrontierPose(pose);};
  callbacks.publish_selected_frontier_pose = [this](const geometry_msgs::msg::PoseStamped & pose) {
      random_policy_.normalDispatch();
      random_avoid_.push_back({get_clock()->now().seconds(),
        {pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,0}});
      while(random_avoid_.size()>64)random_avoid_.pop_front();
      this->publishSelectedFrontierPose(pose);
    };
  callbacks.publish_optimized_map = [this](const nav_msgs::msg::OccupancyGrid & map_msg) {
      this->publishOptimizedMap(map_msg);
    };
  callbacks.on_exploration_complete = [this]() {
      this->publishCompletionEvent();
    };
  callbacks.debug_outputs_enabled = [this]() {
      return this->debugOutputsEnabled();
    };
  callbacks.log_debug = [this](const std::string & message) {
      RCLCPP_DEBUG(this->get_logger(), "%s", message.c_str());
    };
  callbacks.log_info = [this](const std::string & message) {
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    };
  callbacks.log_warn = [this](const std::string & message) {
      RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    };
  callbacks.log_error = [this](const std::string & message) {
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    };
  core_ = std::make_unique<FrontierExplorerCore>(params_, callbacks);
  runtime_state_ = RuntimeState::COLD_IDLE;
  if (!autostart_) {
    core_->stop_exploration_session("Frontier exploration initialized in cold idle mode");
  } else {
    startExplorationRuntime();
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Frontier explorer initialized with autostart=%s, control_service_enabled=%s, map '%s', global costmap '%s', local costmap '%s', frontier action '%s'",
    autostart_ ? "true" : "false",
    control_service_enabled_ ? "true" : "false",
    params_.map_topic.c_str(),
    params_.costmap_topic.c_str(),
    params_.local_costmap_topic.c_str(),
    params_.navigate_to_pose_action_name.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Decision-map config: optimization=%s, sigma_s=%.2f, sigma_r=%.2f, dilation_radius=%d, occ_threshold=%d, min_frontier_size_cells=%d, frontier_candidate_min_goal_distance_m=%.2f, debug_outputs=%s",
    frontierMapOptimizationEnabled() ? "true" : "false",
    params_.sigma_s,
    params_.sigma_r,
    params_.dilation_kernel_radius_cells,
    params_.occ_threshold,
    params_.min_frontier_size_cells,
    params_.frontier_candidate_min_goal_distance_m,
    debugOutputsEnabled() ? "debug-log-level" : "disabled");
  RCLCPP_INFO(
    this->get_logger(),
    "Exploration boundary: %s, vertices=%zu, inward_margin=%.2fm",
    params_.exploration_boundary_polygon.empty() ? "disabled" : "enabled",
    params_.exploration_boundary_polygon.size(),
    params_.exploration_boundary_margin_m);
  RCLCPP_INFO(
    this->get_logger(),
    "MRTSP config: solver='%s', dp_candidate_limit=%zu, dp_planning_horizon=%zu, local_completion=%s, local_path_slack=%.2fm, sensor_effective_range_m=%.2f, weight_distance_wd=%.2f, weight_gain_ws=%.2f, max_linear_speed_vmax=%.2f, max_angular_speed_wmax=%.2f",
    core_->params.mrtsp_solver.c_str(),
    core_->params.dp_solver_candidate_limit,
    core_->params.dp_planning_horizon,
    params_.local_completion_enabled ? "true" : "false",
    params_.local_completion_path_slack_m,
    params_.sensor_effective_range_m,
    params_.weight_distance_wd,
    params_.weight_gain_ws,
    params_.max_linear_speed_vmax,
    params_.max_angular_speed_wmax);
  RCLCPP_INFO(
    this->get_logger(),
    "Using post-goal settle config: enabled=%s, post_goal_min_settle=%.2fs, map_processing_rate_hz=%.2f, all_frontiers_suppressed_behavior=%s",
    params_.post_goal_settle_enabled ? "true" : "false",
    params_.post_goal_min_settle,
    params_.map_processing_rate_hz,
    params_.all_frontiers_suppressed_behavior.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "QoS config: map=[durability=%s,reliability=%s,depth=%zu], "
    "costmap=[durability=volatile,reliability=%s,depth=%zu], "
    "local_costmap=[durability=volatile,reliability=%s,depth=%zu%s%s]",
    durability_policy_to_string(topic_qos_profiles_.map_durability).c_str(),
    reliability_policy_to_string(topic_qos_profiles_.map_reliability).c_str(),
    topic_qos_profiles_.map_depth,
    reliability_policy_to_string(topic_qos_profiles_.costmap_reliability).c_str(),
    topic_qos_profiles_.costmap_depth,
    reliability_policy_to_string(topic_qos_profiles_.local_costmap_reliability).c_str(),
    topic_qos_profiles_.local_costmap_depth,
    topic_qos_profiles_.local_costmap_reliability_inherited ? " (inherit reliability)" : "",
    topic_qos_profiles_.local_costmap_depth_inherited ? " (inherit depth)" : "");
  if (completion_event_config_.enabled) {
    RCLCPP_INFO(
      this->get_logger(),
      "Completion event enabled: topic='%s'",
      completion_event_config_.topic.c_str());
  }
  if (!autostart_ && control_service_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Explorer is in cold idle. Send a start request via the '%s' service or use "
      "'frontier_exploration_ctl start' or "
      "'ros2 run frontier_exploration_ros2 frontier_exploration_ctl start'.",
      control_service_->get_service_name());
  }
  if (params_.frontier_suppression_enabled) {
    RCLCPP_INFO(
      this->get_logger(),
      "Frontier suppression enabled: threshold=%d, base=%.2fm, expansion=%.2fm, timeout=%.2fs, no_progress_timeout=%.2fs, startup_grace=%.2fs, max_attempts=%d, max_regions=%d",
      params_.frontier_suppression_attempt_threshold,
      params_.frontier_suppression_base_size_m,
      params_.frontier_suppression_expansion_size_m,
      params_.frontier_suppression_timeout_s,
      params_.frontier_suppression_no_progress_timeout_s,
      params_.frontier_suppression_startup_grace_period_s,
      params_.frontier_suppression_max_attempt_records,
      params_.frontier_suppression_max_regions);
  }
}

FrontierExplorerNode::~FrontierExplorerNode()
{
  if (core_) {
    core_->request_shutdown();
  }
}

bool FrontierExplorerNode::hasActiveExplorationSubscriptions() const
{
  return static_cast<bool>(map_sub_) &&
         static_cast<bool>(costmap_sub_) &&
         static_cast<bool>(local_costmap_sub_);
}

bool FrontierExplorerNode::hasControlService() const
{
  return static_cast<bool>(control_service_);
}

bool FrontierExplorerNode::quitRequested() const
{
  return quit_requested_;
}

void FrontierExplorerNode::createMapSubscription(rclcpp::DurabilityPolicy map_durability)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.map_topic,
    topic_qos_profiles_.make_map_qos(map_durability),
    std::bind(&FrontierExplorerNode::occupancyGridCallback, this, std::placeholders::_1));
  // Reassigning map_sub_ replaces previous subscription instance.
}

void FrontierExplorerNode::ensureMapProcessingTimer()
{
  if (params_.map_processing_rate_hz <= 0.0) {
    return;
  }

  if (!effective_map_processing_rate_hz_.has_value() || *effective_map_processing_rate_hz_ <= 0.0) {
    return;
  }

  if (map_processing_timer_) {
    return;
  }

  map_processing_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / *effective_map_processing_rate_hz_)),
    std::bind(&FrontierExplorerNode::mapProcessingTimerCallback, this));
}

void FrontierExplorerNode::resetMapProcessingCalibrationWindow()
{
  last_map_arrival_at_.reset();
  startup_map_interval_samples_s_.clear();
}

bool FrontierExplorerNode::maybeFinalizeMapProcessingRateEstimate()
{
  if (params_.map_processing_rate_hz <= 0.0) {
    return false;
  }

  if (effective_map_processing_rate_hz_.has_value()) {
    return false;
  }

  if (startup_map_interval_samples_s_.size() < kStartupMapRateSampleCount) {
    return false;
  }

  auto sorted_samples = startup_map_interval_samples_s_;
  const auto middle = sorted_samples.begin() + static_cast<std::ptrdiff_t>(sorted_samples.size() / 2U);
  std::nth_element(sorted_samples.begin(), middle, sorted_samples.end());
  const double median_interval_s = *middle;
  const auto [min_it, max_it] = std::minmax_element(sorted_samples.begin(), sorted_samples.end());
  const bool invalid_interval = median_interval_s <= 0.0 || !std::isfinite(median_interval_s);
  const bool unstable_samples =
    !invalid_interval &&
    *min_it > 0.0 &&
    ((*max_it - *min_it) / median_interval_s) > kStartupMapRateStabilityTolerance;

  if (invalid_interval || unstable_samples) {
    effective_map_processing_rate_hz_ = params_.map_processing_rate_hz;
    if (unstable_samples) {
      RCLCPP_WARN(
        this->get_logger(),
        "Startup /map timing samples were unstable; using configured map_processing_rate_hz=%.2f Hz",
        params_.map_processing_rate_hz);
    }
  } else {
    const double observed_hz = 1.0 / median_interval_s;
    const double capped_observed_hz = observed_hz >= 1.0 ? std::floor(observed_hz) : observed_hz;
    effective_map_processing_rate_hz_ = std::min(
      params_.map_processing_rate_hz,
      std::max(0.1, capped_observed_hz));
    RCLCPP_INFO(
      this->get_logger(),
      "Calibrated /map processing rate from startup samples: observed=%.2f Hz, configured=%.2f Hz, effective=%.2f Hz",
      observed_hz,
      params_.map_processing_rate_hz,
      *effective_map_processing_rate_hz_);
  }

  ensureMapProcessingTimer();
  return true;
}

void FrontierExplorerNode::startExplorationRuntime()
{
  if(localization_watchdog_enabled_) {
    localization_freshness_.reset(get_clock()->now().nanoseconds());
    localization_watchdog_timer_=create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&FrontierExplorerNode::localizationWatchdogCallback,this));
  }
  completion_event_published_ = false;
  pending_quit_after_stop_ = false;
  quit_requested_ = false;
  runtime_state_ = RuntimeState::RUNNING;
  ensureDeferredShutdownTimerCanceled();
  ensureStopCompletionTimerCanceled();
  core_->start_exploration_session();

  if (params_.frontier_suppression_enabled) {
    suppression_activation_logged_ = false;
    suppression_activation_at_ =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(params_.frontier_suppression_startup_grace_period_s));
    if (params_.frontier_suppression_startup_grace_period_s <= 0.0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Frontier suppression startup grace period elapsed; suppression is now active");
      suppression_activation_logged_ = true;
    }
    ensureWatchdogTimer();
  }

  createMapSubscription(topic_qos_profiles_.map_durability);
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.costmap_topic,
    topic_qos_profiles_.make_costmap_qos(),
    std::bind(&FrontierExplorerNode::costmapCallback, this, std::placeholders::_1));
  local_costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    params_.local_costmap_topic,
    topic_qos_profiles_.make_local_costmap_qos(),
    std::bind(&FrontierExplorerNode::localCostmapCallback, this, std::placeholders::_1));
  if (params_.geometry_scoring_enabled) {
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.geometry_point_cloud_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&FrontierExplorerNode::pointCloudCallback, this, std::placeholders::_1));
  }
  if (params_.map_processing_rate_hz > 0.0) {
    if (effective_map_processing_rate_hz_.has_value()) {
      ensureMapProcessingTimer();
    } else {
      resetMapProcessingCalibrationWindow();
    }
  }

  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    map_received_once_ = false;
    map_autodetect_complete_logged_ = false;
    if (map_qos_autodetect_on_startup_) {
      map_qos_autodetect_ = MapQosStartupAutodetect(true, topic_qos_profiles_.map_durability);
      map_autodetect_started_at_ = std::chrono::steady_clock::now();
    } else {
      map_qos_autodetect_.reset();
    }
  }

  if (map_qos_autodetect_on_startup_) {
    map_autodetect_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(map_qos_autodetect_timeout_s_)),
      std::bind(&FrontierExplorerNode::mapAutodetectTimeoutCallback, this));
    logMapAutodetectStart(topic_qos_profiles_.map_durability);
  }
}

void FrontierExplorerNode::ensureWatchdogTimer()
{
  if (!params_.frontier_suppression_enabled || suppression_watchdog_timer_) {
    return;
  }

  const double watchdog_period_s = std::clamp(
    params_.frontier_suppression_no_progress_timeout_s / 4.0,
    0.25,
    1.0);
  suppression_watchdog_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(watchdog_period_s)),
    std::bind(&FrontierExplorerNode::suppressionWatchdogCallback, this));
}

void FrontierExplorerNode::enterColdIdle()
{
  localization_watchdog_timer_.reset();
  runtime_state_ = RuntimeState::COLD_IDLE;
  map_sub_.reset();
  costmap_sub_.reset();
  local_costmap_sub_.reset();
  point_cloud_sub_.reset();
  map_autodetect_timer_.reset();
  map_processing_timer_.reset();
  suppression_watchdog_timer_.reset();
  suppression_activation_logged_ = false;
  suppression_activation_at_.reset();
  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    map_qos_autodetect_.reset();
    map_received_once_ = false;
    map_autodetect_complete_logged_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    latest_pending_map_msg_.reset();
    pending_map_update_ = false;
  }
}

void FrontierExplorerNode::ensureControlTimerCanceled()
{
  if (control_timer_) {
    control_timer_->cancel();
    control_timer_.reset();
  }
  scheduled_control_request_.reset();
}

void FrontierExplorerNode::ensureStopCompletionTimerCanceled()
{
  if (stop_completion_timer_) {
    stop_completion_timer_->cancel();
    stop_completion_timer_.reset();
  }
}

void FrontierExplorerNode::ensureDeferredShutdownTimerCanceled()
{
  if (deferred_shutdown_timer_) {
    deferred_shutdown_timer_->cancel();
    deferred_shutdown_timer_.reset();
  }
}

uint8_t FrontierExplorerNode::controlState() const
{
  if (scheduled_control_request_.has_value()) {
    return scheduled_control_request_->action == srv::ControlExploration::Request::ACTION_START ?
           srv::ControlExploration::Request::STATE_START_SCHEDULED :
           srv::ControlExploration::Request::STATE_STOP_SCHEDULED;
  }

  switch (runtime_state_) {
    case RuntimeState::RUNNING:
      return srv::ControlExploration::Request::STATE_RUNNING;
    case RuntimeState::STOPPING:
      return srv::ControlExploration::Request::STATE_STOPPING;
    case RuntimeState::SHUTDOWN_PENDING:
      return srv::ControlExploration::Request::STATE_SHUTDOWN_PENDING;
    case RuntimeState::COLD_IDLE:
    default:
      return srv::ControlExploration::Request::STATE_IDLE;
  }
}

std::string FrontierExplorerNode::controlStateMessage() const
{
  switch (controlState()) {
    case srv::ControlExploration::Request::STATE_RUNNING:
      return "running";
    case srv::ControlExploration::Request::STATE_START_SCHEDULED:
      return "start scheduled";
    case srv::ControlExploration::Request::STATE_STOP_SCHEDULED:
      return "stop scheduled";
    case srv::ControlExploration::Request::STATE_STOPPING:
      return "stopping";
    case srv::ControlExploration::Request::STATE_SHUTDOWN_PENDING:
      return "shutdown pending";
    case srv::ControlExploration::Request::STATE_IDLE:
    default:
      return "idle";
  }
}

void FrontierExplorerNode::scheduleControlRequest(
  uint8_t action,
  double delay_seconds,
  bool quit_after_stop)
{
  ensureControlTimerCanceled();
  scheduled_control_request_ = ScheduledControlRequest{action, quit_after_stop};
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(delay_seconds)),
    std::bind(&FrontierExplorerNode::controlTimerCallback, this));
}

void FrontierExplorerNode::requestStopExplorationRuntime(
  bool quit_after_stop,
  const std::string & reason)
{
  ++surface_goal_epoch_; ++random_epoch_;clearance_request_ready_=false;force_no_goal_recovery_=false;
  random_policy_.last_time=-1;random_avoid_.clear();
  surface_goal_cache_.clear();
  surface_goal_queue_.clear();
  surface_goal_requests_.clear(); surface_goal_options_.clear(); surface_goal_exclusions_.clear();surface_goal_references_.clear();
  surface_retry_anchor_.reset();
  ensureControlTimerCanceled();
  pending_quit_after_stop_ = quit_after_stop;
  core_->stop_exploration_session(reason);
  publishFrontierMarkers({});
  enterColdIdle();

  if (core_->ready_for_shutdown()) {
    if (pending_quit_after_stop_) {
      runtime_state_ = RuntimeState::SHUTDOWN_PENDING;
      deferred_shutdown_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&FrontierExplorerNode::deferredShutdownCallback, this));
    }
    return;
  }

  runtime_state_ = RuntimeState::STOPPING;
  stop_completion_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&FrontierExplorerNode::stopCompletionPollCallback, this));
}

void FrontierExplorerNode::handleControlRequest(
  const std::shared_ptr<srv::ControlExploration::Request> request,
  std::shared_ptr<srv::ControlExploration::Response> response)
{
  const double delay_seconds = static_cast<double>(request->delay_seconds);
  const bool start_is_scheduled =
    scheduled_control_request_.has_value() &&
    scheduled_control_request_->action == srv::ControlExploration::Request::ACTION_START;
  const bool stop_is_scheduled =
    scheduled_control_request_.has_value() &&
    scheduled_control_request_->action == srv::ControlExploration::Request::ACTION_STOP;
  if (delay_seconds < 0.0) {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "delay_seconds must be non-negative";
    return;
  }

  if (request->quit_after_stop && request->action != srv::ControlExploration::Request::ACTION_STOP) {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "quit_after_stop is only valid for stop requests";
    return;
  }

  if (
    request->action != srv::ControlExploration::Request::ACTION_START &&
    request->action != srv::ControlExploration::Request::ACTION_STOP)
  {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "Unsupported control action";
    return;
  }

  if (request->action == srv::ControlExploration::Request::ACTION_START) {
    if (delay_seconds <= 0.0) {
      // Immediate start is the strongest operator intent and should clear any stale timer.
      ensureControlTimerCanceled();
    }

    if (runtime_state_ == RuntimeState::RUNNING) {
      response->accepted = delay_seconds <= 0.0;
      response->scheduled = false;
      response->state = controlState();
      response->message = delay_seconds <= 0.0 && stop_is_scheduled ?
        "Exploration is already running; cleared scheduled stop" :
        delay_seconds <= 0.0 ?
        "Exploration is already running" :
        "Cannot schedule a future start while exploration is already running";
      return;
    }
    if (runtime_state_ == RuntimeState::STOPPING || runtime_state_ == RuntimeState::SHUTDOWN_PENDING) {
      response->accepted = false;
      response->scheduled = false;
      response->state = controlState();
      response->message = "Exploration is stopping or shutting down";
      return;
    }
    if (delay_seconds > 0.0) {
      scheduleControlRequest(request->action, delay_seconds, false);
      response->accepted = true;
      response->scheduled = true;
      response->state = controlState();
      response->message = "Scheduled exploration start";
      return;
    }

    startExplorationRuntime();
    response->accepted = true;
    response->scheduled = false;
    response->state = controlState();
    response->message = "Exploration started";
    return;
  }

  if (delay_seconds > 0.0) {
    if (runtime_state_ == RuntimeState::COLD_IDLE && !start_is_scheduled) {
      response->accepted = false;
      response->scheduled = false;
      response->state = controlState();
      response->message = "Cannot schedule a stop while exploration is idle";
      return;
    }

    scheduleControlRequest(request->action, delay_seconds, request->quit_after_stop);
    response->accepted = true;
    response->scheduled = true;
    response->state = controlState();
    response->message = "Scheduled exploration stop";
    return;
  }

  if (runtime_state_ == RuntimeState::SHUTDOWN_PENDING) {
    response->accepted = false;
    response->scheduled = false;
    response->state = controlState();
    response->message = "Shutdown is already pending";
    return;
  }

  requestStopExplorationRuntime(
    request->quit_after_stop,
    request->quit_after_stop ?
    "Stopping exploration and shutting down the node" :
    "Stopping exploration");
  response->accepted = true;
  response->scheduled = false;
  response->state = controlState();
  response->message = request->quit_after_stop ?
    "Stopping exploration and preparing node shutdown" :
    "Stopping exploration";
}

void FrontierExplorerNode::controlTimerCallback()
{
  if (!scheduled_control_request_.has_value()) {
    ensureControlTimerCanceled();
    return;
  }

  const ScheduledControlRequest scheduled_request = *scheduled_control_request_;
  ensureControlTimerCanceled();

  if (scheduled_request.action == srv::ControlExploration::Request::ACTION_START) {
    if (runtime_state_ == RuntimeState::COLD_IDLE) {
      startExplorationRuntime();
    }
    return;
  }

  if (runtime_state_ == RuntimeState::SHUTDOWN_PENDING) {
    return;
  }

  requestStopExplorationRuntime(
    scheduled_request.quit_after_stop,
    scheduled_request.quit_after_stop ?
    "Stopping exploration and shutting down the node" :
    "Stopping exploration");
}

void FrontierExplorerNode::stopCompletionPollCallback()
{
  if (!core_ || !core_->ready_for_shutdown()) {
    return;
  }

  ensureStopCompletionTimerCanceled();
  if (pending_quit_after_stop_) {
    runtime_state_ = RuntimeState::SHUTDOWN_PENDING;
    deferred_shutdown_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&FrontierExplorerNode::deferredShutdownCallback, this));
    return;
  }

  runtime_state_ = RuntimeState::COLD_IDLE;
}

void FrontierExplorerNode::deferredShutdownCallback()
{
  ensureDeferredShutdownTimerCanceled();
  runtime_state_ = RuntimeState::SHUTDOWN_PENDING;
  quit_requested_ = true;
}

void FrontierExplorerNode::mapAutodetectTimeoutCallback()
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }

  // Work in two phases:
  //   1) Decide state transition under lock.
  //   2) Perform ROS side effects (re-subscribe / log) outside lock.
  std::optional<rclcpp::DurabilityPolicy> switched_durability;
  bool should_log_failed_completion = false;
  rclcpp::DurabilityPolicy selected_durability = topic_qos_profiles_.map_durability;

  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    if (
      !map_qos_autodetect_.has_value() ||
      map_autodetect_complete_logged_ ||
      !map_qos_autodetect_->active() ||
      map_received_once_)
    {
      return;
    }

    switched_durability = map_qos_autodetect_->on_timeout();
    // Read active durability after transition so logs and subscription stay consistent.
    selected_durability = map_qos_autodetect_->active_durability();
    if (!switched_durability.has_value() && !map_qos_autodetect_->active()) {
      // No more fallback attempts remain; emit terminal failed completion once.
      map_autodetect_complete_logged_ = true;
      should_log_failed_completion = true;
    }
  }

  if (switched_durability.has_value()) {
    // Re-create map subscription with fallback durability for the second and final attempt.
    createMapSubscription(*switched_durability);
    logMapAutodetectSwitch(*switched_durability);
    return;
  }

  if (should_log_failed_completion) {
    if (map_autodetect_timer_) {
      // Cancel timer after terminal state to keep autodetect strictly startup-only.
      map_autodetect_timer_->cancel();
    }
    logMapAutodetectComplete("failed", selected_durability);
  }
}

void FrontierExplorerNode::logMapAutodetectStart(rclcpp::DurabilityPolicy selected_durability)
{
  if (!map_qos_autodetect_on_startup_) {
    return;
  }
  // One-shot startup marker for log filtering.
  RCLCPP_INFO(
    this->get_logger(),
    "[qos-autodetect] START selected=%s timeout=%.2fs",
    durability_policy_to_string(selected_durability).c_str(),
    map_qos_autodetect_timeout_s_);
}

void FrontierExplorerNode::logMapAutodetectSwitch(rclcpp::DurabilityPolicy selected_durability)
{
  if (!map_qos_autodetect_on_startup_) {
    return;
  }
  // Warn-level by design: initial durability did not match publisher QoS.
  RCLCPP_WARN(
    this->get_logger(),
    "[qos-autodetect] SWITCH selected=%s elapsed=%.2fs",
    durability_policy_to_string(selected_durability).c_str(),
    mapAutodetectElapsedSeconds());
}

void FrontierExplorerNode::logMapAutodetectComplete(
  const std::string & result,
  rclcpp::DurabilityPolicy selected_durability)
{
  if (!map_qos_autodetect_on_startup_) {
    return;
  }
  const std::string selected = durability_policy_to_string(selected_durability);
  const double elapsed = mapAutodetectElapsedSeconds();
  if (result == "failed") {
    // Failed means neither initial nor fallback durability received map within timeout windows.
    RCLCPP_WARN(
      this->get_logger(),
      "[qos-autodetect] COMPLETE result=%s selected=%s elapsed=%.2fs",
      result.c_str(),
      selected.c_str(),
      elapsed);
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "[qos-autodetect] COMPLETE result=%s selected=%s elapsed=%.2fs",
    result.c_str(),
    selected.c_str(),
    elapsed);
}

double FrontierExplorerNode::mapAutodetectElapsedSeconds() const
{
  if (!map_qos_autodetect_on_startup_) {
    return 0.0;
  }
  // Steady clock keeps elapsed time robust against ROS/system clock jumps.
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - map_autodetect_started_at_).count();
}

RelocationStep FrontierExplorerNode::requestSymmetricRelocationStep(
  const geometry_msgs::msg::Pose & current,RelocationPoint destination)
{
  RelocationStep step;
  if(random_stair_mode_ || runtime_state_!=RuntimeState::RUNNING ||
     !surface_goal_validation_enabled_)return step;
  const double now=get_clock()->now().seconds();
  if(symmetry_clock_>=0 && now<symmetry_clock_)symmetry_retry_at_=0;
  symmetry_clock_=now;
  std::shared_ptr<const octomap::OcTree> tree;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  double tree_stamp;
  {std::lock_guard<std::mutex> lock(surface_map_mutex_);
    tree=surface_map_;map=surface_hint_map_;tree_stamp=surface_map_time_s_;}
  if(!tree || !map || map->header.frame_id!=params_.global_frame)return step;
  const double age=now-rclcpp::Time(map->header.stamp).seconds();
  if(age<-.1 || age>3 || now-tree_stamp<-.1 || now-tree_stamp>3)return step;
  const auto yaw=[](const auto & q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
  if(symmetry_future_.valid()) {
    if(symmetry_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return step;
    RecoveryResult result;
    try {result=symmetry_future_.get();}
    catch(const std::exception & e) {
      symmetry_retry_at_=now+3;step.failed=true;
      RCLCPP_WARN(get_logger(),"Symmetric relocation validator failed: %s",e.what());return step;
    }
    if(result.epoch!=random_epoch_ ||
       std::hypot(symmetry_worker_target_.first-destination.first,
         symmetry_worker_target_.second-destination.second)>.05)return step;
    const double age=now-result.stamp_ns*1e-9;
    if(age<-.1 || age>3 ||
       std::hypot(current.position.x-result.start.position.x,current.position.y-result.start.position.y)>.1 ||
       std::abs(current.position.z-result.start.position.z)>.08 ||
       std::abs(std::remainder(yaw(current.orientation)-yaw(result.start.orientation),2*M_PI))>.15 ||
       std::abs(map->info.origin.position.z-result.floor_z)>.15)return step;
    symmetry_retry_at_=now+3;
    step.goal=result.goal;step.failed=!result.goal;
    RCLCPP_INFO(get_logger(),"Symmetric relocation 3D segment validation: destination=(%.2f, %.2f), outcome=%s",
      destination.first,destination.second,result.goal?"verified":"no-verified-route");
    return step;
  }
  if(now<symmetry_retry_at_ &&
    std::hypot(symmetry_worker_target_.first-destination.first,
      symmetry_worker_target_.second-destination.second)<.05)return step;
  auto config=surface_goal_config_;
  try {
    const auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
    if(now-rclcpp::Time(feet.header.stamp).seconds()>.5)return step;
    config.support_reference_height=current.position.z-feet.transform.translation.z;
  }catch(const tf2::TransformException &){return step;}
  const auto epoch=random_epoch_;const auto stamp=get_clock()->now();
  const auto frame=params_.global_frame;const auto angle=yaw(current.orientation);
  const auto boundary=params_.exploration_boundary_polygon;
  const auto boundary_margin=params_.exploration_boundary_margin_m;
  symmetry_worker_target_=destination;
  symmetry_future_=std::async(std::launch::async,
    [tree,map,config,current,destination,epoch,stamp,frame,angle,boundary,boundary_margin]() mutable {
      RecoveryResult result;result.start=current;result.epoch=epoch;
      result.stamp_ns=stamp.nanoseconds();result.floor_z=map->info.origin.position.z;
      ObservationGrid grid;grid.width=map->info.width;grid.height=map->info.height;
      grid.resolution=map->info.resolution;grid.x=map->info.origin.position.x;grid.y=map->info.origin.position.y;
      const auto & q=map->info.origin.orientation;
      grid.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));grid.cells=map->data;
      config.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
      try {
        auto selected=selectMappedObservation(*tree,config,
          {current.position.x,current.position.y,current.position.z,angle},
          {destination.first,destination.second,map->info.origin.position.z,angle},grid,{},.25,5.,nullptr,boundary,boundary_margin);
        if(selected) {
          geometry_msgs::msg::PoseStamped goal;goal.header.frame_id=frame;goal.header.stamp=stamp;
          goal.pose.position.x=selected->x;goal.pose.position.y=selected->y;goal.pose.position.z=selected->z;
          goal.pose.orientation.z=std::sin(selected->yaw/2);goal.pose.orientation.w=std::cos(selected->yaw/2);
          result.goal=goal;
        }
      }catch(const go2_surface_3d_planner::SurfaceSearchTimeout &) {}
      return result;
    });
  return step;
}

void FrontierExplorerNode::recordVisitedOpen(const geometry_msgs::msg::Pose & current)
{
 if(random_stair_mode_ || runtime_state_!=RuntimeState::RUNNING)return;
 double now=get_clock()->now().seconds();
 if(last_open_sample_>=0 && now>=last_open_sample_ && now-last_open_sample_<2.)return;
 last_open_sample_=now;
 std::shared_ptr<const octomap::OcTree> tree;nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
 {std::lock_guard<std::mutex> lock(surface_map_mutex_);tree=surface_map_;map=surface_hint_map_;}
 if(!tree || !map || map->header.frame_id!=params_.global_frame)return;
 double age=now-rclcpp::Time(map->header.stamp).seconds();if(age<-.1 || age>1.5)return;
 visited_open_.sync(now,map->info.origin.position.z);
 try {
  auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
  double age=now-rclcpp::Time(feet.header.stamp).seconds();if(age<-.1 || age>.5)return;
  auto c=surface_goal_config_;c.snap_radius=0;c.preferred_clearance=.15;
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(50);
  c.support_reference_height=current.position.z-feet.transform.translation.z;
  const auto & q=current.orientation;
  double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  go2_surface_3d_planner::SurfacePoint body{current.position.x,current.position.y,current.position.z,yaw};
  if(!go2_surface_3d_planner::configureSupportPlane(c,*tree,body))return;
  go2_surface_3d_planner::SurfaceGraph graph(*tree,c);
  auto stance=graph.snapToSurface({body.x,body.y,feet.transform.translation.z,yaw});
  if(stance && is_inside_exploration_boundary({stance->x,stance->y},
       params_.exploration_boundary_polygon,params_.exploration_boundary_margin_m))
    visited_open_.observe(now,map->info.origin.position.z,*stance,graph.bodyClearanceAt(*stance));
 }catch(const std::exception &) {}
}

std::optional<geometry_msgs::msg::PoseStamped> FrontierExplorerNode::requestRandomRecovery(
  const geometry_msgs::msg::Pose & current,bool blocked)
{
  if(!random_recovery_enabled_ || random_stair_mode_ ||
    runtime_state_!=RuntimeState::RUNNING) return std::nullopt;
  const double now=get_clock()->now().seconds();
  std::shared_ptr<const octomap::OcTree> tree;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  {std::lock_guard<std::mutex> lock(surface_map_mutex_);tree=surface_map_;map=surface_hint_map_;}
  if(!tree || !map || map->header.frame_id!=params_.global_frame)return std::nullopt;
  const double map_age=now-rclcpp::Time(map->header.stamp).seconds();
  if(map_age<-.1 || map_age>3.0)return std::nullopt;
  const bool reentry=!is_inside_exploration_boundary(
    {current.position.x,current.position.y},params_.exploration_boundary_polygon,0.);
  if(reentry && boundary_outside_distance(
      {current.position.x,current.position.y},params_.exploration_boundary_polygon)>.75) {
    RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),10000,
      "Boundary excursion exceeds 0.75m automatic return limit; holding for localization/mission-boundary inspection");
    return std::nullopt;
  }
  if(random_policy_.last_time<0)random_map_progress_=KnownMapProgress{};
  const double area=random_map_progress_.observe(*map,now);
  random_policy_.observe(now,current.position.x,current.position.y,area);
  if(random_policy_.displaced(current.position.x,current.position.y) &&
     random_policy_.recovered(now,current.position.x,current.position.y,measuredClearanceRestored())) {
    RCLCPP_INFO(get_logger(),"Recovery episode cleared: measured displacement >=0.50m and safe support/clearance; no map gain required");
  }
  // Waiting for planning is not an executing robot being physically stuck.
  // Only the independent 300s no-next-goal deadline arms this blocked trigger.
  if(blocked && force_no_goal_recovery_)random_policy_.blocked(now);
  if(core_ && !core_->goal_in_progress && random_policy_.motion.failures==0)
    random_policy_.motion.armed=false;
  while(!random_avoid_.empty() && now-random_avoid_.front().first>300)random_avoid_.pop_front();
  if(random_future_.valid()) {
    if(random_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)
      return std::nullopt;
    RecoveryResult result;
    try {result=random_future_.get();}
    catch(const std::exception & error) {
      RCLCPP_WARN(get_logger(),"Random recovery validation failed: %s",error.what());
      return std::nullopt;
    }
    if(result.epoch!=random_epoch_)return std::nullopt;
    const auto yaw=[](const auto & q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));};
    const double age=now-result.stamp_ns*1e-9;
    if(age<-.1 || age>3 ||
      std::hypot(current.position.x-result.start.position.x,current.position.y-result.start.position.y)>.1 ||
      std::abs(current.position.z-result.start.position.z)>.08 ||
      std::abs(map->info.origin.position.z-result.floor_z)>.15 ||
      std::abs(std::remainder(yaw(current.orientation)-yaw(result.start.orientation),2*M_PI))>.15)
      return std::nullopt;
    if(result.clearance_needed) {
      clearance_request_ready_=true;
      RCLCPP_WARN(get_logger(),"Recovery start has %.4fm clearance; restoring reserved margin before random sampling",
        result.start_gap);
      return std::nullopt;
    }
    if(result.start_state!=RecoveryStartState::ready) {
      RCLCPP_WARN(get_logger(),"Recovery refused at measured start: %s; body gap %.4fm",
        result.start_state==RecoveryStartState::contact?"actual body contact/excluded region":"missing support",
        result.start_gap);
      return std::nullopt;
    }
    if(!result.goal) {
      RCLCPP_WARN(get_logger(),"No safe random recovery point in either ring; holding position and cooling down");
      return std::nullopt;
    }
    const double actual_distance=std::hypot(result.goal->pose.position.x-current.position.x,
      result.goal->pose.position.y-current.position.y);
    if(actual_distance<(reentry?.15:random_sampling_.minimum_distance) ||
       actual_distance>(result.historical_return?8.:random_sampling_.expanded_radius))return std::nullopt;
    random_avoid_.push_back({now,{result.goal->pose.position.x,result.goal->pose.position.y,
      result.goal->pose.position.z,0}});
    RCLCPP_WARN(get_logger(),"%s candidate: (%.2f, %.2f), distance %.2fm",
      reentry?"Verified inward boundary return":
        result.historical_return?"Verified return to last visited open area":"Verified random recovery",
      result.goal->pose.position.x,result.goal->pose.position.y,
      std::hypot(result.goal->pose.position.x-current.position.x,
        result.goal->pose.position.y-current.position.y));
    return result.goal;
  }
  // The independent no-next-goal deadline arms existing recovery immediately;
  // normal attempt/batch cooldowns and all safety checks remain in force.
  if(force_no_goal_recovery_)
    random_policy_.blocked_since=now-random_policy_.blocked_timeout;
  const auto previous_batch=random_policy_.batches;
  if(reentry ? now-random_policy_.last_attempt<10. : !random_policy_.ready(now))return std::nullopt;
  if(random_policy_.batches!=previous_batch)
    RCLCPP_WARN(get_logger(),"Random recovery batch cooldown complete; retry batch=%u with fresh samples and retained avoidance history",
      random_policy_.batches+1);
  auto config=surface_goal_config_;
  try {
    const auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
    if(now-rclcpp::Time(feet.header.stamp).seconds()>.5)return std::nullopt;
    config.support_reference_height=current.position.z-feet.transform.translation.z;
  }catch(const tf2::TransformException &){return std::nullopt;}
  std::vector<go2_surface_3d_planner::SurfacePoint> avoid;
  for(const auto & old:random_avoid_)avoid.push_back(old.second);
  for(const auto & p:random_policy_.samples)
    avoid.push_back({p.x,p.y,current.position.z-config.support_reference_height,0});
  random_policy_.begin(now);force_no_goal_recovery_=false;
  visited_open_.sync(now,map->info.origin.position.z);
  auto historical=visited_open_.candidates({current.position.x,current.position.y,current.position.z,0});
  const auto sampling=random_sampling_;
  const auto boundary=params_.exploration_boundary_polygon;
  const auto boundary_margin=std::max(.35,params_.exploration_boundary_margin_m);
  if(reentry) {
    std::vector<go2_surface_3d_planner::SurfacePoint> interior;
    OccupancyGrid2d grid(*map);
    for(int y=0;y<grid.getSizeY();++y)for(int x=0;x<grid.getSizeX();++x) {
      if(grid.getCost(x,y)!=0)continue;
      auto point=grid.mapToWorld(x,y);
      double distance=std::hypot(point.first-current.position.x,point.second-current.position.y);
      if(distance<=1.5 && distance>=.15 &&
         is_inside_exploration_boundary(point,params_.exploration_boundary_polygon,boundary_margin))
        interior.push_back({point.first,point.second,map->info.origin.position.z,0});
    }
    std::sort(interior.begin(),interior.end(),[&](auto a,auto b) {
      return std::hypot(a.x-current.position.x,a.y-current.position.y)<
        std::hypot(b.x-current.position.x,b.y-current.position.y);
    });
    // Spread endpoint attempts instead of spending the budget on adjacent cells.
    for(auto p:interior) {
      if(std::all_of(historical.begin(),historical.end(),[&](auto q) {
        return std::hypot(p.x-q.x,p.y-q.y)>=.3;}))historical.push_back(p);
      if(historical.size()>=12)break;
    }
    RCLCPP_WARN(get_logger(),"Boundary reentry search: outside=%.3fm, interior candidates=%zu",
      boundary_outside_distance({current.position.x,current.position.y},params_.exploration_boundary_polygon),
      historical.size());
  }
  const auto budget=random_recovery_budget_;
  const auto epoch=random_epoch_;
  const uint32_t seed=random_rng_();
  const auto stamp=get_clock()->now();
  const auto frame=params_.global_frame;
  RCLCPP_WARN(get_logger(),"Random recovery search: attempt=%d, seed=%u, rings %.2f..%.2fm then %.2f..%.2fm",
    random_policy_.attempts,seed,sampling.minimum_distance,sampling.initial_radius,
    sampling.initial_radius,sampling.expanded_radius);
  random_future_=std::async(std::launch::async,
    [tree,map,config,current,avoid,historical,sampling,budget,epoch,seed,stamp,frame,boundary,boundary_margin,reentry]() mutable {
      RecoveryResult result;result.start=current;result.epoch=epoch;
      result.stamp_ns=stamp.nanoseconds();result.floor_z=map->info.origin.position.z;
      ObservationGrid grid;grid.width=map->info.width;grid.height=map->info.height;
      grid.resolution=map->info.resolution;grid.x=map->info.origin.position.x;
      grid.y=map->info.origin.position.y;
      const auto & q=map->info.origin.orientation;
      grid.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      grid.cells=map->data;
      config.search_deadline=std::chrono::steady_clock::now()+
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(budget));
      const auto & oq=current.orientation;
      const double yaw=std::atan2(2*(oq.w*oq.z+oq.x*oq.y),1-2*(oq.y*oq.y+oq.z*oq.z));
      try {
        const auto assessment=assessRecoveryStart(*tree,config,
          {current.position.x,current.position.y,current.position.z,yaw});
        result.start_state=assessment.state;result.start_gap=assessment.gap;
        if(assessment.state==RecoveryStartState::ready ||
           assessment.state==RecoveryStartState::needs_clearance) {
          auto target=selectVisitedOpenReturn(*tree,config,
            {current.position.x,current.position.y,current.position.z,yaw},
            historical,boundary,boundary_margin,reentry);
          if(target) {
            geometry_msgs::msg::PoseStamped goal;goal.header.frame_id=frame;goal.header.stamp=stamp;
            goal.pose.position.x=target->x;goal.pose.position.y=target->y;goal.pose.position.z=target->z;
            goal.pose.orientation.z=std::sin(target->yaw/2);goal.pose.orientation.w=std::cos(target->yaw/2);
            result.goal=goal;result.historical_return=true;result.start_state=RecoveryStartState::ready;
            return result;
          }
        }
        // Outside the mission, never replace a failed inward route by random
        // movement or an unrelated local clearance action.
        if(reentry)return result;
        if(assessment.state==RecoveryStartState::needs_clearance ||
           assessment.state==RecoveryStartState::contact) {
          result.clearance_needed=true;return result;
        }
        if(assessment.state!=RecoveryStartState::ready)return result;
        const auto target=selectRandomRecovery(*tree,config,
          {current.position.x,current.position.y,current.position.z,yaw},grid,sampling,avoid,seed,boundary,boundary_margin);
        if(target) {
          geometry_msgs::msg::PoseStamped goal;goal.header.frame_id=frame;goal.header.stamp=stamp;
          goal.pose.position.x=target->x;goal.pose.position.y=target->y;goal.pose.position.z=target->z;
          goal.pose.orientation.z=std::sin(target->yaw/2);goal.pose.orientation.w=std::cos(target->yaw/2);
          result.goal=goal;
        }
      }catch(const go2_surface_3d_planner::SurfaceSearchTimeout &) {}
      return result;
    });
  return std::nullopt;
}

void FrontierExplorerNode::serviceResidualAudit()
{
 if(!core_ || !floor_cleanup_mode_ || runtime_state_!=RuntimeState::RUNNING ||
    random_stair_mode_ || core_->goal_in_progress)return;
 double now=get_clock()->now().seconds();
 if(now<residual_audit_clock_ || residual_audit_epoch_!=random_epoch_) {
  residual_audit_stability_.reset();residual_audit_cache_.reset();residual_closed_policy_.clear();residual_audit_next_=now;
 }
 residual_audit_clock_=now;residual_audit_epoch_=random_epoch_;
 std::shared_ptr<const PairedFloorMap> pair;
 std::shared_ptr<const octomap::OcTree> tree,live;nav_msgs::msg::OccupancyGrid::ConstSharedPtr hint,transition_mask;
 {std::lock_guard<std::mutex> lock(surface_map_mutex_);pair=paired_floor_map_;live=surface_map_;hint=surface_hint_map_;transition_mask=transition_mask_;}
 if(!pair || !hint || floorDigest(*hint)!=pair->version.floor_digest) {
  RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),10000,"Residual audit waiting for source-matched /map_floor/snapshot");return;
 }
 tree=pair->tree;hint=pair->floor;
 auto actual=getCurrentPose();
 if(!tree || !hint || !actual || hint->header.frame_id!=params_.global_frame)return;
 double age=now-rclcpp::Time(hint->header.stamp).seconds();
 if(age<-.1 || age>1.5){residual_audit_stability_.reset();return;}
 OccupancyGrid2d grid(*hint);
 const auto runtime=core_->residual_evidence_mask(grid,now);
 uint64_t runtime_hash=1469598103934665603ULL;
 for(auto value:runtime)runtime_hash=(runtime_hash^uint64_t(value))*1099511628211ULL;
 std::vector<int8_t> transition(hint->data.size(),0);uint64_t transition_hash=0;
 if(transition_mask && transition_mask->info.width==hint->info.width &&
    transition_mask->info.height==hint->info.height &&
    std::abs(transition_mask->info.resolution-hint->info.resolution)<1e-6 &&
    std::abs(transition_mask->info.origin.position.x-hint->info.origin.position.x)<1e-6 &&
    std::abs(transition_mask->info.origin.position.y-hint->info.origin.position.y)<1e-6 &&
    std::abs(transition_mask->info.origin.position.z-hint->info.origin.position.z)<.02 &&
    now-rclcpp::Time(transition_mask->header.stamp).seconds()>=-.1 &&
    now-rclcpp::Time(transition_mask->header.stamp).seconds()<=10.) {
   transition=transition_mask->data;
   transition_hash=residualMapFingerprint(OccupancyGrid2d(*transition_mask));
 }

 residual_closed_policy_.observe(now,grid,params_.exploration_boundary_polygon,
   params_.exploration_boundary_margin_m,std::make_pair(actual->position.x,actual->position.y));
 if(residual_audit_future_.valid()) {
  if(residual_audit_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
  ResidualAudit result;
  try{result=residual_audit_future_.get();}
  catch(const std::exception & e){
   residual_audit_stability_.reset();
   RCLCPP_ERROR(get_logger(),"Floor completion assessment: ERROR; residual audit failed: %s",e.what());return;
  }
  if(!mapEvidenceCurrent(result.map_evidence,*pair,live.get()) || result.transition_hash!=transition_hash || result.runtime_hash!=runtime_hash || result.epoch!=random_epoch_ ||
     std::hypot(actual->position.x-result.start.first,actual->position.y-result.start.second)>.3) {
    RCLCPP_INFO(get_logger(),"Discarding stale residual audit: map dependencies, runtime evidence, epoch or pose changed");
    residual_audit_stability_.reset();residual_audit_next_=now+2;return;
  }
  residual_audit_cache_=result;
  RCLCPP_INFO(get_logger(),"Per-region residual audit: regions=%zu, deferred=%.3fm2, pending=%.3fm2, complete=%s, tested_sites=%zu, reused_regions=%zu",
    result.regions.size(),result.deferred,result.pending,result.complete?"true":"false",result.sites,result.reused_regions);
  RCLCPP_INFO(get_logger(),"Residual region classifications: %s",result.records().c_str());
  if(result.reference && result.view && core_->pending_frontier_sequence.empty()) {
    FrontierCandidate candidate{*result.reference,*result.view,1};
    candidate.viewpoint_candidates={*result.view};candidate.requires_visible_viewpoint=true;
    core_->pending_frontier_sequence.push_back(candidate);
  }
  if(!result.complete)residual_audit_budget_=std::min(60.,residual_audit_budget_*2);
  else residual_audit_budget_=5.;
  int64_t stamp=int64_t(hint->header.stamp.sec)*1000000000LL+hint->header.stamp.nanosec;
  RCLCPP_INFO(get_logger(),"Floor completion assessment: %s",completionStatusName(result.status()));
  bool stable=residual_audit_stability_.observe(result,now,stamp);
  if(stable && pairedSourceCaughtUp(*pair,live.get()) && measuredClearanceRestored(true)) {
    std::ostringstream report;report<<std::setprecision(10);
    report<<"v4;floor_z="<<hint->info.origin.position.z<<";stamp="<<now
      <<";reason=region_observability_audit;remaining_unknown_area="<<result.remaining
      <<";actionable_unknown_area=0;excluded_unknown_area="<<result.closed
      <<";deferred_unknown_area="<<result.deferred<<";transition_unknown_area="<<result.transition<<";unresolved_unknown_area=0"
      <<";scope=per_region_current_floor;region_count="<<result.regions.size()
      <<";stable_samples="<<residual_audit_stability_.samples()
      <<";stable_seconds="<<residual_audit_stability_.seconds(now)
      <<";safe_start=1;complete_views=1;complete_routes="<<(result.verification_deferred>0?0:1)
      <<";runtime_evidence=1;runtime_min_positions=3;runtime_min_healthy_s=60;attempts=0;regions="<<result.records();
    std_msgs::msg::String msg;msg.data=report.str();floor_exhausted_pub_->publish(msg);
    std::ostringstream formal;formal<<std::setprecision(10);
    formal<<"floor_completion;authority=joint_residual_audit;status="<<completionStatusName(result.status())
      <<";floor_z="<<hint->info.origin.position.z<<";stamp="<<now
      <<";remaining="<<result.remaining<<";residual="<<result.deferred<<";transition="<<result.transition
      <<";region_count="<<result.regions.size()<<";regions="<<result.records()
      <<";safe_start=1;complete_views=1;complete_routes=1;stable_samples="<<residual_audit_stability_.samples()
      <<";stable_seconds="<<residual_audit_stability_.seconds(now)
      <<";map_evidence="<<encodeMapEvidence(result.map_evidence);
    if(parseVerifiedFloorCompletion(formal.str())) {
      std_msgs::msg::String verified;verified.data=formal.str();verified_completion_pub_->publish(verified);
    }
    RCLCPP_WARN(get_logger(),"Current-floor residual regions individually audited; requesting stair evaluation: %s",msg.data.c_str());
  }
  residual_audit_next_=now+3.;
 }
 if(now<residual_audit_next_)return;
 residual_audit_next_=now+3.;
 auto config=surface_goal_config_;VisibilitySensor sensor;
 try {
  const auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
  if(now-rclcpp::Time(feet.header.stamp).seconds()>.5)return;
  config.support_reference_height=actual->position.z-feet.transform.translation.z;
  const auto mount=tf_buffer_->lookupTransform("base_footprint",visibility_sensor_frame_,tf2::TimePointZero);
  sensor.x=mount.transform.translation.x;sensor.y=mount.transform.translation.y;sensor.z=mount.transform.translation.z;
  sensor.qx=mount.transform.rotation.x;sensor.qy=mount.transform.rotation.y;sensor.qz=mount.transform.rotation.z;sensor.qw=mount.transform.rotation.w;
  sensor.min_pitch=visibility_min_pitch_*M_PI/180.;sensor.max_pitch=visibility_max_pitch_*M_PI/180.;
 }catch(const tf2::TransformException &){return;}
 // No failed start, stale map or planning timeout can certify a residual.
 if(!measuredClearanceRestored(true)){residual_audit_stability_.reset();return;}
 std::vector<uint8_t> excluded(hint->data.size(),0);
 for(int y=0;y<grid.getSizeY();++y)for(int x=0;x<grid.getSizeX();++x)
  excluded[y*grid.getSizeX()+x]=residual_closed_policy_.excludedCell(x,y);
 auto boundary=params_.exploration_boundary_polygon;double margin=params_.exploration_boundary_margin_m;
 auto pose=*actual;double budget=residual_audit_budget_;auto epoch=random_epoch_;
 auto previous=residual_audit_cache_;
 residual_audit_future_=std::async(std::launch::async,[pair,tree,hint,config,sensor,excluded,boundary,margin,pose,budget,epoch,transition,transition_hash,runtime,runtime_hash,previous]() mutable {
  auto deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(budget));
  config.search_deadline=deadline;
  const auto & q=pose.orientation;double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  OccupancyGrid2d grid(*hint);
  if(!go2_surface_3d_planner::configureSupportPlane(config,*tree,{pose.position.x,pose.position.y,pose.position.z,yaw})) {
    ResidualAudit result;result.fingerprint=residualMapFingerprint(grid);result.epoch=epoch;result.transition_hash=transition_hash;result.runtime_hash=runtime_hash;
    result.start={pose.position.x,pose.position.y};result.complete=false;
    sealResidualAudit(result,*pair,boundary,margin,1.);return result;
  }
  uint64_t support_context=mapHashSeed();
  for(double value:{config.flat_support_height,config.flat_support_origin_x,config.flat_support_origin_y,
      config.flat_support_slope_x,config.flat_support_slope_y,config.support_reference_height,
      pose.position.z,sensor.x,sensor.y,sensor.z,sensor.qx,sensor.qy,sensor.qz,sensor.qw,sensor.min_pitch,sensor.max_pitch})
    mapHashDouble(support_context,value);
  AuditRouteCache routes;
  const go2_surface_3d_planner::SurfacePoint origin{pose.position.x,pose.position.y,
    pose.position.z-config.support_reference_height,yaw};
  auto result=auditResidualRegions(grid,{pose.position.x,pose.position.y},boundary,margin,
    [&](int x,int y){return bool(excluded[y*grid.getSizeX()+x]);},
    [&](auto view,auto target){return auditAdmissibleView(*tree,config,view,target,hint->info.origin.position.z,sensor,deadline,origin,&routes,std::min(4.,std::max(.5,budget/10.)));},deadline,
    [&](int x,int y){return transition[y*grid.getSizeX()+x]==100;},
    [&](int x,int y){return runtime[y*grid.getSizeX()+x];},
    [&](const ResidualRegion & region){
      return previous && canReuseOccludedRegion(*previous,region,*pair,pose.position.x,pose.position.y,
        support_context,transition_hash,runtime_hash);
    });
  result.start={pose.position.x,pose.position.y};result.support_context=support_context;
  const double padding=config.snap_radius+config.collision_radius+std::abs(config.cylinder_offset)+
    std::hypot(sensor.x,sensor.y)+grid.map().info.resolution;
  sealResidualAudit(result,*pair,boundary,margin,std::max(.3,padding));
  result.epoch=epoch;result.transition_hash=transition_hash;result.runtime_hash=runtime_hash;result.start={pose.position.x,pose.position.y};return result;
 });
}

void FrontierExplorerNode::serviceSurfaceValidation()
{
  serviceResidualAudit();
  const auto now=std::chrono::steady_clock::now();
  if (surface_goal_future_.valid() &&
    surface_goal_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready)
  {
    auto result=surface_goal_future_.get();
    if (result.epoch==surface_goal_epoch_) {
      const double cooldown=surface_goal_queue_.finish(result.key,result.timed_out,now,bool(result.goal));
      const auto & goal=result.requested.pose.position;
      RCLCPP_INFO(get_logger(),
        "3D validation result: goal=(%.2f, %.2f, %.2f), attempt=%u, budget=%.2fs, elapsed=%.2fs, outcome=%s, cooldown=%.1fs, viewpoints=%u/%u, hidden=%u, route_not_verified=%u, option_timeouts=%u",
        goal.x,goal.y,goal.z,result.attempt,result.budget,result.elapsed,
        result.timed_out?"timeout-deferred":result.goal?"verified":"no-verified-route",cooldown,
        result.options_tested,result.options_available,result.hidden_options,result.route_rejections,result.timeout_options);
      if(core_ && surface_goal_references_.count(result.key))
        core_->report_frontier_validation(surface_goal_references_.at(result.key),bool(result.goal),
          !result.timed_out && result.timeout_options==0,result.options_tested,result.options_available);
      surface_goal_cache_[result.key]=std::move(result);
    }
  }
  if (surface_goal_future_.valid() || !surface_goal_validation_enabled_) {return;}
  const auto start=getCurrentPose();
  if (!start) {return;}
  recordVisitedOpen(*start);
  std::shared_ptr<const octomap::OcTree> map;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr hint_map;
  {std::lock_guard<std::mutex> lock(surface_map_mutex_); map=surface_map_;hint_map=surface_hint_map_;}
  if (!map || !hint_map || hint_map->header.frame_id!=params_.global_frame) {return;}
  auto config=surface_goal_config_;
  try {
    const auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
    config.support_reference_height=start->position.z-feet.transform.translation.z;
  } catch (const tf2::TransformException &) {return;}
  VisibilitySensor sensor;
  sensor.min_pitch=visibility_min_pitch_*M_PI/180.;sensor.max_pitch=visibility_max_pitch_*M_PI/180.;
  if(visibility_3d_enabled_) {
    try {
      const auto mount=tf_buffer_->lookupTransform("base_footprint",
        visibility_sensor_frame_,tf2::TimePointZero);
      sensor.x=mount.transform.translation.x;sensor.y=mount.transform.translation.y;
      sensor.z=mount.transform.translation.z;
      sensor.qx=mount.transform.rotation.x;sensor.qy=mount.transform.rotation.y;
      sensor.qz=mount.transform.rotation.z;sensor.qw=mount.transform.rotation.w;
    }catch(const tf2::TransformException &) {return;}
  }
  const auto job=surface_goal_queue_.next(now);
  if (!job) {return;}
  const auto request=surface_goal_requests_.find(job->key);
  if (request==surface_goal_requests_.end()) {
    surface_goal_queue_.finish(job->key,false,now); return;
  }
  const auto goal=request->second;
  const double base_budget=surface_goal_budget_s_;
  const bool visibility_3d=visibility_3d_enabled_;
  const auto reference=surface_goal_references_.count(job->key)?
    surface_goal_references_.at(job->key):std::make_pair(goal.pose.position.x,goal.pose.position.y);
  const bool cleanup_mode=floor_cleanup_mode_;
  const auto boundary=params_.exploration_boundary_polygon;
  const auto boundary_margin=params_.exploration_boundary_margin_m;
  const auto exclusions=surface_goal_exclusions_.count(job->key)?
    surface_goal_exclusions_.at(job->key):std::vector<std::pair<double,double>>{};
  const auto options=surface_goal_options_.count(job->key)?
    surface_goal_options_.at(job->key):std::vector<geometry_msgs::msg::PoseStamped>{goal};
  const auto yaw = [](const geometry_msgs::msg::Quaternion & q) {
      return std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
    };
  const double start_yaw=yaw(start->orientation);
  const auto epoch=surface_goal_epoch_;
  RCLCPP_INFO(get_logger(),"3D validation start: goal=(%.2f, %.2f, %.2f), attempt=%u, budget=%.2fs",
    goal.pose.position.x,goal.pose.position.y,goal.pose.position.z,job->attempt,job->budget);
  surface_goal_future_=std::async(std::launch::async,
    [map,hint_map,config,start=*start,goal,options,job=*job,epoch,start_yaw,yaw,base_budget,exclusions,cleanup_mode,sensor,visibility_3d,reference,boundary,boundary_margin]() mutable {
      SurfaceValidationResult result;
      result.key=job.key; result.start=start; result.epoch=epoch;
      result.requested=goal; result.budget=job.budget; result.attempt=job.attempt;
      result.options_available=options.size();
      const auto began=std::chrono::steady_clock::now();
      config.search_deadline=began+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(job.budget));
      try {
        ObservationGrid grid;
        grid.width=hint_map->info.width;grid.height=hint_map->info.height;
        grid.resolution=hint_map->info.resolution;
        grid.x=hint_map->info.origin.position.x;grid.y=hint_map->info.origin.position.y;
        const auto & q=hint_map->info.origin.orientation;
        grid.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        grid.cells=hint_map->data;
        // Follow the mapped route around furniture and through doors. Straight
        // approaches to unrelated distant frontiers can otherwise oscillate.
        const auto total_deadline=config.search_deadline;
        // Retry different portions first so broad fans cannot starve behind
        // the same difficult first 25 viewpoints on every bounded attempt.
        for(size_t offset=0;offset<options.size();++offset) {
          const auto & option=options[(offset+(job.attempt-1)*10)%options.size()];
          if(std::chrono::steady_clock::now()>=total_deadline) {
            result.timed_out=true;break;
          }
          ++result.options_tested;
          auto attempt=config;
          // Bound each viewpoint so one difficult door cannot consume the
          // entire group budget. Timeout backoff also increases its slice.
          const auto slice=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(.60*job.budget/std::max(.1,base_budget)));
          attempt.search_deadline=std::min(total_deadline,
            std::chrono::steady_clock::now()+slice);
          const auto & target=option.pose.position;
          std::optional<go2_surface_3d_planner::SurfacePoint> selected;
          bool mapped_timeout=false;
          try {
            if(visibility_3d && !visibleUnknown3D(*map,grid,
              {target.x,target.y,hint_map->info.origin.position.z,yaw(option.pose.orientation)},
              reference,sensor,attempt.search_deadline)) {
              ++result.hidden_options;continue;
            }
            selected=selectMappedObservation(*map,attempt,
              {start.position.x,start.position.y,start.position.z,start_yaw},
              {target.x,target.y,target.z,yaw(option.pose.orientation)},grid,exclusions,cleanup_mode?.25:.5,5.,&mapped_timeout,boundary,boundary_margin);
          }catch(const go2_surface_3d_planner::SurfaceSearchTimeout &) {
            result.timed_out=true;++result.timeout_options;continue;
          }
          if(selected) {
            result.goal=option;
            auto & point=result.goal->pose.position;
            point.x=selected->x;point.y=selected->y;point.z=selected->z;
            auto & q=result.goal->pose.orientation;
            q.x=0;q.y=0;q.z=std::sin(selected->yaw/2);q.w=std::cos(selected->yaw/2);
            result.timed_out=false;break;
          }
          ++result.route_rejections;
          if(mapped_timeout || std::chrono::steady_clock::now()>=attempt.search_deadline) {
            result.timed_out=true;++result.timeout_options;
          }
        }
      } catch(const go2_surface_3d_planner::SurfaceSearchTimeout &) {result.timed_out=true;}
      result.completed=std::chrono::steady_clock::now();
      result.elapsed=std::chrono::duration<double>(result.completed-began).count();
      return result;
    });
}

void FrontierExplorerNode::mapProcessingTimerCallback()
{
  if (runtime_state_ != RuntimeState::RUNNING || !core_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    pending_map_update_ = false;
    latest_pending_map_msg_.reset();
  }

  serviceSurfaceValidation();
  core_->processPendingMapUpdate();
}

void FrontierExplorerNode::suppressionWatchdogCallback()
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }

  if (
    !suppression_activation_logged_ &&
    suppression_activation_at_.has_value() &&
    std::chrono::steady_clock::now() >= *suppression_activation_at_)
  {
    RCLCPP_INFO(this->get_logger(), "Frontier suppression startup grace period elapsed; suppression is now active");
    suppression_activation_logged_ = true;
  }

  if (core_) {
    core_->evaluate_active_goal_progress_timeout();
  }
}

void FrontierExplorerNode::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }

  bool should_log_complete = false;
  std::string complete_result;
  rclcpp::DurabilityPolicy selected_durability = topic_qos_profiles_.map_durability;

  {
    std::lock_guard<std::mutex> lock(map_autodetect_mutex_);
    if (!map_received_once_) {
      // Autodetect is startup-only; first valid map finalizes the selected durability.
      map_received_once_ = true;
      if (map_qos_autodetect_.has_value() && !map_autodetect_complete_logged_) {
        complete_result = map_qos_autodetect_->fallback_attempted() ? "fallback" : "initial";
        selected_durability = map_qos_autodetect_->active_durability();
        map_qos_autodetect_->on_map_received();
        map_autodetect_complete_logged_ = true;
        should_log_complete = true;
      }
    }
  }

  if (should_log_complete && map_autodetect_timer_) {
    // COMPLETE log is emitted exactly once when first map is accepted.
    map_autodetect_timer_->cancel();
    logMapAutodetectComplete(complete_result, selected_durability);
  }

  {std::lock_guard<std::mutex> lock(surface_map_mutex_);surface_hint_map_=msg;}
  const OccupancyGrid2d map_msg(msg);
  if (params_.map_processing_rate_hz <= 0.0) {
    core_->occupancyGridCallback(map_msg);
    return;
  }

  const auto arrival_at = std::chrono::steady_clock::now();
  if (!effective_map_processing_rate_hz_.has_value()) {
    if (last_map_arrival_at_.has_value()) {
      const double interval_s = std::chrono::duration<double>(arrival_at - *last_map_arrival_at_).count();
      if (interval_s > 0.0) {
        startup_map_interval_samples_s_.push_back(interval_s);
      }
    }
    last_map_arrival_at_ = arrival_at;
  }

  {
    std::lock_guard<std::mutex> lock(pending_map_mutex_);
    latest_pending_map_msg_ = msg;
    pending_map_update_ = true;
  }

  core_->ingestRawMapUpdate(map_msg);
  maybeFinalizeMapProcessingRateEstimate();
  //   core_->handleUrgentRawMapUpdateForActiveGoal();
  // if (maybeFinalizeMapProcessingRateEstimate()) {
  //   mapProcessingTimerCallback();
  // }
}

void FrontierExplorerNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }
  core_->costmapCallback(OccupancyGrid2d(msg));
}

void FrontierExplorerNode::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  if (runtime_state_ != RuntimeState::RUNNING) {
    return;
  }
  core_->localCostmapCallback(OccupancyGrid2d(msg));
}

void FrontierExplorerNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (
    runtime_state_ != RuntimeState::RUNNING ||
    !params_.geometry_scoring_enabled ||
    msg->width * msg->height == 0U)
  {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      params_.global_frame,
      msg->header.frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Cannot transform geometry cloud from '%s' to '%s': %s",
      msg->header.frame_id.c_str(),
      params_.global_frame.c_str(),
      ex.what());
    return;
  }

  const auto & q = transform.transform.rotation;
  const double q_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (q_norm < 1e-9) {
    return;
  }
  const double qx = q.x / q_norm;
  const double qy = q.y / q_norm;
  const double qz = q.z / q_norm;
  const double qw = q.w / q_norm;
  const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double r01 = 2.0 * (qx * qy - qz * qw);
  const double r02 = 2.0 * (qx * qz + qy * qw);
  const double r10 = 2.0 * (qx * qy + qz * qw);
  const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  const double r12 = 2.0 * (qy * qz - qx * qw);
  const double r20 = 2.0 * (qx * qz - qy * qw);
  const double r21 = 2.0 * (qy * qz + qx * qw);
  const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);
  const auto & translation = transform.transform.translation;

  const std::size_t point_count = static_cast<std::size_t>(msg->width) * msg->height;
  const std::size_t stride = std::max<std::size_t>(
    1U,
    (point_count + params_.geometry_max_points - 1U) / params_.geometry_max_points);
  std::vector<GeometryPoint3d> cloud;
  cloud.reserve(std::min(point_count, params_.geometry_max_points));

  try {
    sensor_msgs::PointCloud2ConstIterator<float> x_iter(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_iter(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_iter(*msg, "z");
    for (std::size_t index = 0U; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter, ++index) {
      if (index % stride != 0U) {
        continue;
      }
      const double x = static_cast<double>(*x_iter);
      const double y = static_cast<double>(*y_iter);
      const double z = static_cast<double>(*z_iter);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      cloud.push_back({
        r00 * x + r01 * y + r02 * z + translation.x,
        r10 * x + r11 * y + r12 * z + translation.y,
        r20 * x + r21 * y + r22 * z + translation.z,
      });
    }
  } catch (const std::runtime_error & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Geometry point cloud is missing usable x/y/z fields: %s",
      ex.what());
    return;
  }

  core_->update_geometry_cloud(std::move(cloud));
}

void FrontierExplorerNode::publishCompletionEvent()
{
  if (!completion_event_config_.enabled || !completion_event_pub_) {
    // Completion publishing stays dormant unless explicitly configured.
    return;
  }
  if (completion_event_published_) {
    // Frontier exhaustion may be observed repeatedly while the node remains alive.
    return;
  }

  completion_event_published_ = true;
  completion_event_pub_->publish(std_msgs::msg::Empty{});
  RCLCPP_INFO(
    this->get_logger(),
    "Published exploration completion event on '%s'",
    completion_event_config_.topic.c_str());
}

void FrontierExplorerNode::localizationWatchdogCallback()
{
  if(runtime_state_!=RuntimeState::RUNNING) {return;}
  std::optional<int64_t> stamp;
  try {
    const auto transform=tf_buffer_->lookupTransform(params_.global_frame,
      params_.robot_base_frame,tf2::TimePointZero);
    stamp=rclcpp::Time(transform.header.stamp).nanoseconds();
  } catch(const tf2::TransformException &) {}
  if(localization_freshness_.observe(get_clock()->now().nanoseconds(),stamp)==
      LocalizationFreshness::State::Expired)
  {
    RCLCPP_ERROR(get_logger(),"Localization unavailable or stale; cancelling navigation and stopping exploration");
    requestStopExplorationRuntime(false,"Localization unavailable or stale");
  }
}

std::optional<geometry_msgs::msg::Pose> FrontierExplorerNode::getCurrentPose()
{
  try {
    // Bounded TF lookup keeps scheduler responsive under frame delays.
    const auto transform = tf_buffer_->lookupTransform(
      params_.global_frame,
      params_.robot_base_frame,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));

    if(localization_watchdog_enabled_ && !LocalizationFreshness::accepts(
        get_clock()->now().nanoseconds(),rclcpp::Time(transform.header.stamp).nanoseconds()))
    {throw tf2::TransformException("Localization transform timestamp is stale or in the future");}
    geometry_msgs::msg::Pose pose;
    // Convert transform directly to pose used by frontier/core decisions.
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return pose;
  } catch (const tf2::TransformException & exc) {
    const int64_t now_ns = this->get_clock()->now().nanoseconds();
    const int64_t throttle_ns = static_cast<int64_t>(tf_warning_throttle_seconds_ * 1e9);
    if (!last_tf_warning_time_ns_.has_value() || now_ns - *last_tf_warning_time_ns_ >= throttle_ns) {
      // Throttle repeated TF warnings during map startup or transient frame drops.
      last_tf_warning_time_ns_ = now_ns;
      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform %s -> %s: %s",
        params_.robot_base_frame.c_str(),
        params_.global_frame.c_str(),
        exc.what());
    }
    return std::nullopt;
  }
}

void FrontierExplorerNode::publishFrontierMarkers(const FrontierSequence & frontiers)
{
  visualization_msgs::msg::MarkerArray marker_array;
  const auto publish_stamp = this->now();

  visualization_msgs::msg::Marker clear_marker;
  // Always clear previous marker namespace first to avoid stale points in RViz.
  clear_marker.header.frame_id = params_.global_frame;
  clear_marker.header.stamp = publish_stamp;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  if (!frontiers.empty()) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = params_.global_frame;
    marker.header.stamp = publish_stamp;
    marker.ns = "frontier_frontiers_primitive";
    marker.id = 0;
    // POINTS renders as screen-aligned squares in RViz, matching the MRTSP reference package.
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.frame_locked = true;
    marker.scale.x = params_.frontier_marker_scale;
    marker.scale.y = params_.frontier_marker_scale;
    marker.color.a = 1.0;
    marker.color.r = 0.15;
    marker.color.g = 0.9;
    marker.color.b = 0.2;
    // Color/namespace choices stay stable for predictable RViz overlays and MRTSP parity.

    for (const auto & frontier : frontiers) {
      // Marker position reflects selected goal point for each frontier entry.
      const auto [frontier_x, frontier_y] = frontier_position(frontier);
      geometry_msgs::msg::Point point;
      point.x = frontier_x;
      point.y = frontier_y;
      point.z = 0.0;
      marker.points.push_back(point);
    }

    marker_array.markers.push_back(marker);
  }

  frontier_marker_pub_->publish(marker_array);
}

void FrontierExplorerNode::publishSelectedFrontierPose(const geometry_msgs::msg::PoseStamped & pose)
{
  if (!selected_frontier_pub_) {
    return;
  }
  selected_frontier_pub_->publish(pose);
}

void FrontierExplorerNode::publishOptimizedMap(const nav_msgs::msg::OccupancyGrid & map_msg)
{
  if (!debugOutputsEnabled() || !optimized_map_pub_) {
    return;
  }
  optimized_map_pub_->publish(map_msg);
}

bool FrontierExplorerNode::frontierMapOptimizationEnabled() const
{
  return params_.frontier_map_optimization_enabled;
}

bool FrontierExplorerNode::debugOutputsEnabled() const
{
  return rcutils_logging_logger_is_enabled_for(
    this->get_logger().get_name(),
    RCUTILS_LOG_SEVERITY_DEBUG);
}


bool FrontierExplorerNode::takeClearanceRecoveryRequest()
{
  if(clearance_request_ready_){clearance_request_ready_=false;return true;}
  if(random_stair_mode_ || runtime_state_!=RuntimeState::RUNNING || !core_ ||
     core_->goal_in_progress || !clearance_client_->action_server_is_ready())return false;
  const double now=get_clock()->now().seconds();
  if(now<contact_probe_at_ || now<contact_request_at_){contact_probe_at_=contact_request_at_=-1;}
  if(contact_probe_at_>=0 && now-contact_probe_at_<2.)return false;
  if(contact_request_at_>=0 && now-contact_request_at_<10.)return false;
  contact_probe_at_=now;
  auto current=getCurrentPose();if(!current)return false;
  if(!is_inside_exploration_boundary({current->position.x,current->position.y},
    params_.exploration_boundary_polygon,params_.exploration_boundary_margin_m))return false;
  std::shared_ptr<const octomap::OcTree> tree;nav_msgs::msg::OccupancyGrid::ConstSharedPtr hint;
  {std::lock_guard<std::mutex> lock(surface_map_mutex_);tree=surface_map_;hint=surface_hint_map_;}
  if(!tree || !hint || hint->header.frame_id!=params_.global_frame)return false;
  const double age=now-rclcpp::Time(hint->header.stamp).seconds();
  if(age<-.1 || age>1.5)return false;
  try {
    auto config=surface_goal_config_;
    const auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
    const double age=now-rclcpp::Time(feet.header.stamp).seconds();
    if(age<-.1 || age>.5)return false;
    config.support_reference_height=current->position.z-feet.transform.translation.z;
    config.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
    const auto & q=current->orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto state=assessRecoveryStart(*tree,config,
      {current->position.x,current->position.y,current->position.z,yaw});
    if(state.state!=RecoveryStartState::contact)return false;
    contact_request_at_=now;
    RCLCPP_WARN(get_logger(),"Measured mapped contact %.4fm: request small-step clearance recovery immediately, without the 300s planning wait",state.gap);
    return true;
  }catch(const std::exception &){return false;}
}

bool FrontierExplorerNode::measuredClearanceRestored(bool hard_only)
{
  if(random_stair_mode_ || runtime_state_!=RuntimeState::RUNNING)return false;
  const auto current=getCurrentPose();if(!current)return false;
  if(!is_inside_exploration_boundary({current->position.x,current->position.y},
       params_.exploration_boundary_polygon,params_.exploration_boundary_margin_m))return false;
  std::shared_ptr<const octomap::OcTree> tree;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  {std::lock_guard<std::mutex> lock(surface_map_mutex_);tree=surface_map_;map=surface_hint_map_;}
  const double now=get_clock()->now().seconds();
  if(!tree || !map || map->header.frame_id!=params_.global_frame ||
     now-rclcpp::Time(map->header.stamp).seconds()<-.1 ||
     now-rclcpp::Time(map->header.stamp).seconds()>1.5)return false;
  auto config=surface_goal_config_;
  if(hard_only)config.clearance_margin=.01;
  try {
    const auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
    if(now-rclcpp::Time(feet.header.stamp).seconds()>.5)return false;
    config.support_reference_height=current->position.z-feet.transform.translation.z;
    config.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(250);
    const auto & q=current->orientation;
    const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    const auto check=assessRecoveryStart(*tree,config,
      {current->position.x,current->position.y,current->position.z,yaw});
    RCLCPP_INFO(get_logger(),"Measured body clearance %.4fm; nominal margin %.3fm",
      check.gap,config.clearance_margin);
    return check.state==RecoveryStartState::ready;
  } catch(const std::exception & error) {
    RCLCPP_WARN(get_logger(),"Cannot verify recovered clearance: %s",error.what());return false;
  }
}

void FrontierExplorerNode::dispatchClearanceRequest(const GoalDispatchRequest & request)
{
  if(random_stair_mode_ || runtime_state_!=RuntimeState::RUNNING ||
    !clearance_client_->action_server_is_ready()) {
    core_->goal_response_callback(request.dispatch_id,nullptr,false,
      "Clearance restoration unavailable or exploration stopped");return;
  }
  ClearanceAction::Goal goal;
  // Private mode understood by EscapeMove: margin restoration only, never
  // switch to its general 2D escape if a start is unsupported or already clear.
  goal.target.z=-1.0;goal.speed=.04;goal.time_allowance.sec=30;
  rclcpp_action::Client<ClearanceAction>::SendGoalOptions options;
  using Handle=rclcpp_action::ClientGoalHandle<ClearanceAction>;
  options.goal_response_callback=[this,id=request.dispatch_id](std::shared_ptr<Handle> handle) {
    if(handle && id!=core_->current_dispatch_id) {
      try {clearance_client_->async_cancel_goal(handle);}
      catch(const std::exception & e) {RCLCPP_ERROR(get_logger(),"Late clearance cancellation failed: %s",e.what());}
      return;
    }
    std::shared_ptr<GoalHandleInterface> owned;
    if(handle)owned=std::make_shared<ClearanceGoalHandleAdapter>(handle,
      [this](const std::shared_ptr<Handle> & active,
        std::function<void(bool,const std::string &)> callback) {
        try {
          clearance_client_->async_cancel_goal(active,[callback=std::move(callback)](
            action_msgs::srv::CancelGoal::Response::SharedPtr response) mutable {
              callback(response && !response->goals_canceling.empty(),"");
            });
        }catch(const std::exception & e){callback(false,e.what());}
      });
    core_->goal_response_callback(id,owned,bool(handle),
      handle?"":"Clearance-only recovery goal rejected");
  };
  options.result_callback=[this,id=request.dispatch_id](const Handle::WrappedResult & result) {
    if(id!=core_->current_dispatch_id)return;
    int status=mapResultCodeToGoalStatus(result.code);
    std::string error;
    if(status==action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && !measuredClearanceRestored(true)) {
      status=action_msgs::msg::GoalStatus::STATUS_ABORTED;
      error="Recovery action reported success but current measured support/clearance is not verified";
    }
    ++surface_goal_epoch_;++random_epoch_;clearance_request_ready_=false;force_no_goal_recovery_=false;
    surface_goal_cache_.clear();surface_goal_requests_.clear(); surface_goal_options_.clear(); surface_goal_exclusions_.clear();surface_goal_references_.clear();surface_goal_queue_.clear();
    surface_retry_anchor_.reset();
    random_policy_.last_attempt=get_clock()->now().seconds();
    random_policy_.normalDispatch();
    core_->get_result_callback(id,status,0,error);
  };
  try {clearance_client_->async_send_goal(goal,options);}
  catch(const std::exception & error) {
    core_->goal_response_callback(request.dispatch_id,nullptr,false,error.what());
  }
}

void FrontierExplorerNode::dispatchGoalRequest(const GoalDispatchRequest & request)
{
  if(request.goal_kind=="clearance_recovery") {dispatchClearanceRequest(request);return;}
  NavigateToPose::Goal goal_request;
  // Core provides fully prepared pose/action metadata in request.
  goal_request.pose = request.goal_pose;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback = [this, dispatch_id = request.dispatch_id](
    std::shared_ptr<NavigateGoalHandle> goal_handle)
    {
      std::shared_ptr<GoalHandleInterface> wrapped_handle;
      bool accepted = static_cast<bool>(goal_handle);
      if (accepted) {
        // Wrap Nav2 handle so core can request cancel through transport-agnostic interface.
        wrapped_handle = std::make_shared<NavigateGoalHandleAdapter>(
          goal_handle,
          [this](
            const std::shared_ptr<NavigateGoalHandle> & active_goal_handle,
            std::function<void(bool accepted, const std::string & error_message)> callback)
          {
            try {
              navigate_to_pose_client_->async_cancel_goal(
                active_goal_handle,
                [callback = std::move(callback)](
                  action_msgs::srv::CancelGoal::Response::SharedPtr response) mutable
                {
                  const bool accepted = response && !response->goals_canceling.empty();
                  callback(accepted, "");
                });
            } catch (const std::exception & exc) {
              callback(false, exc.what());
            }
          });
      }

      core_->goal_response_callback(
        dispatch_id,
        wrapped_handle,
        accepted,
        accepted ? "" : "Frontier goal was rejected");
    };

  options.feedback_callback = [this, dispatch_id = request.dispatch_id](
    std::shared_ptr<NavigateGoalHandle>,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      if (!feedback) {
        return;
      }
      if(dispatch_id==core_->current_dispatch_id && random_recovery_enabled_ &&
         !random_stair_mode_ && runtime_state_==RuntimeState::RUNNING) {
        const auto & p=feedback->current_pose.pose.position;
        random_policy_.motion.observe(get_clock()->now().seconds(),p.x,p.y);
      }
      core_->note_navigation_motion(feedback->current_pose.pose, feedback->number_of_recoveries, dispatch_id);
      core_->feedback_callback(feedback->distance_remaining, dispatch_id);
    };

  options.result_callback = [this, dispatch_id = request.dispatch_id](
    const NavigateGoalHandle::WrappedResult & wrapped_result)
    {
      const int status = mapResultCodeToGoalStatus(wrapped_result.code);
      int error_code = 0;
      std::string error_msg;
      if (wrapped_result.result) {
        // Nav2 result fields differ across ROS distros; extract when available.
        error_code = compat::extractNav2ResultErrorCode(*wrapped_result.result);
        error_msg = compat::extractNav2ResultErrorMessage(*wrapped_result.result);
      }

      core_->get_result_callback(
        dispatch_id,
        status,
        error_code,
        error_msg);
    };

  try {
    navigate_to_pose_client_->async_send_goal(goal_request, options);
  } catch (const std::exception & exc) {
    core_->goal_response_callback(
      request.dispatch_id,
      nullptr,
      false,
      std::string("Failed to send frontier goal: ") + exc.what());
  }
}

int FrontierExplorerNode::mapResultCodeToGoalStatus(rclcpp_action::ResultCode code)
{
  // Normalize transport-specific result codes into GoalStatus values expected by core.
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      return action_msgs::msg::GoalStatus::STATUS_SUCCEEDED;
    case rclcpp_action::ResultCode::ABORTED:
      return action_msgs::msg::GoalStatus::STATUS_ABORTED;
    case rclcpp_action::ResultCode::CANCELED:
      return action_msgs::msg::GoalStatus::STATUS_CANCELED;
    case rclcpp_action::ResultCode::UNKNOWN:
    default:
      return action_msgs::msg::GoalStatus::STATUS_UNKNOWN;
  }
}

}  // namespace frontier_exploration_ros2
