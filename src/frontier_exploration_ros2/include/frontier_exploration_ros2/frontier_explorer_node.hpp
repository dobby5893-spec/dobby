#include <std_msgs/msg/float64_multi_array.hpp>
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

#pragma once

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include "frontier_exploration_ros2/recovery_start.hpp"
#include "frontier_exploration_ros2/residual_region_audit.hpp"
#include "frontier_exploration_ros2/visited_open_return.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "frontier_exploration_ros2/random_recovery.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include "go2_surface_3d_planner/surface_graph.hpp"

#include "frontier_exploration_ros2/srv/control_exploration.hpp"
#include "frontier_exploration_ros2/qos_utils.hpp"
#include "frontier_exploration_ros2/validation_retry_queue.hpp"
#include "frontier_exploration_ros2/localization_freshness.hpp"
#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

namespace frontier_exploration_ros2
{

// ROS 2 wiring layer for FrontierExplorerCore (subscriptions, actions, worker queue, diagnostics).
class FrontierExplorerNode : public rclcpp::Node
{
public:
  explicit FrontierExplorerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FrontierExplorerNode() override;
  bool hasActiveExplorationSubscriptions() const;
  bool hasControlService() const;
  bool quitRequested() const;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateGoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void publishCompletionEvent();

  std::optional<geometry_msgs::msg::Pose> getCurrentPose();
  void publishFrontierMarkers(const FrontierSequence & frontiers);
  void publishSelectedFrontierPose(const geometry_msgs::msg::PoseStamped & pose);
  void publishOptimizedMap(const nav_msgs::msg::OccupancyGrid & map_msg);
  bool frontierMapOptimizationEnabled() const;
  bool debugOutputsEnabled() const;
  void ensureMapProcessingTimer();
  void resetMapProcessingCalibrationWindow();
  bool maybeFinalizeMapProcessingRateEstimate();

  void dispatchGoalRequest(const GoalDispatchRequest & request);
  void createMapSubscription(rclcpp::DurabilityPolicy map_durability);
  void mapAutodetectTimeoutCallback();
  void logMapAutodetectStart(rclcpp::DurabilityPolicy selected_durability);
  void logMapAutodetectSwitch(rclcpp::DurabilityPolicy selected_durability);
  void logMapAutodetectComplete(
    const std::string & result,
    rclcpp::DurabilityPolicy selected_durability);
  double mapAutodetectElapsedSeconds() const;
  void mapProcessingTimerCallback();
  void suppressionWatchdogCallback();
  void controlTimerCallback();
  void stopCompletionPollCallback();
  void deferredShutdownCallback();
  void handleControlRequest(
    const std::shared_ptr<srv::ControlExploration::Request> request,
    std::shared_ptr<srv::ControlExploration::Response> response);
  void startExplorationRuntime();
  void localizationWatchdogCallback();
  bool localization_watchdog_enabled_{false};
  LocalizationFreshness localization_freshness_;
  rclcpp::TimerBase::SharedPtr localization_watchdog_timer_;
  void requestStopExplorationRuntime(bool quit_after_stop, const std::string & reason);
  void enterColdIdle();
  void ensureWatchdogTimer();
  void ensureControlTimerCanceled();
  void ensureStopCompletionTimerCanceled();
  void ensureDeferredShutdownTimerCanceled();
  void scheduleControlRequest(
    uint8_t action,
    double delay_seconds,
    bool quit_after_stop);
  uint8_t controlState() const;
  std::string controlStateMessage() const;
  static int mapResultCodeToGoalStatus(rclcpp_action::ResultCode code);

  FrontierExplorerCoreParams params_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr stair_regions_sub_;
  std::vector<double> stair_region_values_;
  bool random_recovery_enabled_{true},random_stair_mode_{false};
  bool clearance_request_ready_{false};
  bool force_no_goal_recovery_{false};
  using ClearanceAction=nav2_msgs::action::DriveOnHeading;
  rclcpp_action::Client<ClearanceAction>::SharedPtr clearance_client_;
  void dispatchClearanceRequest(const GoalDispatchRequest & request);
  bool measuredClearanceRestored(bool hard_only=false);
  bool takeClearanceRecoveryRequest();
  double contact_probe_at_{-1},contact_request_at_{-1};
  void serviceResidualAudit();
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr transition_mask_sub_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr transition_mask_;

  std::future<ResidualAudit> residual_audit_future_;
  FloorCoveragePolicy residual_closed_policy_;
  ResidualAuditStability residual_audit_stability_;
  uint64_t residual_audit_epoch_{std::numeric_limits<uint64_t>::max()};
  double residual_audit_next_{0},residual_audit_clock_{-1},residual_audit_budget_{5.};

  double random_recovery_budget_{3.0};
  RecoveryPolicy random_policy_;
  VisitedOpenHistory visited_open_;
  double last_open_sample_{-1};
  void recordVisitedOpen(const geometry_msgs::msg::Pose & current);
  KnownMapProgress random_map_progress_;
  RandomRecoveryConfig random_sampling_;
  std::mt19937 random_rng_{std::random_device{}()};
  uint64_t random_epoch_{0};
  struct RecoveryResult {
    std::optional<geometry_msgs::msg::PoseStamped> goal;
    geometry_msgs::msg::Pose start;
    uint64_t epoch{0};
    int64_t stamp_ns{0};
    double floor_z{0};
    bool clearance_needed{false};
    bool historical_return{false};
    RecoveryStartState start_state{RecoveryStartState::ready};
    double start_gap{-1};
  };
  std::future<RecoveryResult> random_future_;
  std::future<RecoveryResult> symmetry_future_;
  RelocationPoint symmetry_worker_target_{0,0};
  double symmetry_retry_at_{0},symmetry_clock_{-1};
  RelocationStep requestSymmetricRelocationStep(
    const geometry_msgs::msg::Pose & current,RelocationPoint destination);
  std::deque<std::pair<double,go2_surface_3d_planner::SurfacePoint>> random_avoid_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr random_stair_sub_;
  std::optional<geometry_msgs::msg::PoseStamped> requestRandomRecovery(
    const geometry_msgs::msg::Pose & current,bool blocked);
  bool surface_goal_validation_enabled_{false};
  bool floor_cleanup_mode_{false};
  bool visibility_3d_enabled_{false};
  std::string visibility_sensor_frame_{"livox_frame"};
  double visibility_min_pitch_{-7.22},visibility_max_pitch_{55.22};
  std::pair<double,double> observation_reference_{0,0};
  std::map<std::string,std::pair<double,double>> surface_goal_references_;
  double surface_goal_max_retreat_{1.5};
  double surface_goal_budget_s_{1.0};
  void serviceSurfaceValidation();
  ValidationRetryQueue surface_goal_queue_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> surface_goal_requests_;
  std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> surface_goal_options_;
  std::vector<geometry_msgs::msg::PoseStamped> submitted_surface_options_;
  std::vector<std::pair<double,double>> observation_exclusions_;
  std::map<std::string,std::vector<std::pair<double,double>>> surface_goal_exclusions_;
  std::optional<geometry_msgs::msg::Pose> surface_retry_anchor_;
  struct SurfaceValidationResult
  {
    std::string key;
    geometry_msgs::msg::Pose start;
    std::optional<geometry_msgs::msg::PoseStamped> goal;
    bool timed_out{false};
    geometry_msgs::msg::PoseStamped requested;
    double budget{0.0}, elapsed{0.0};
    unsigned attempt{0};
    unsigned options_available{0},options_tested{0},hidden_options{0},route_rejections{0},timeout_options{0};
    uint64_t epoch{0};
    std::chrono::steady_clock::time_point completed;
  };
  std::future<SurfaceValidationResult> surface_goal_future_;
  std::map<std::string, SurfaceValidationResult> surface_goal_cache_;
  uint64_t surface_goal_epoch_{0};
  std::mutex surface_map_mutex_;
  std::shared_ptr<const octomap::OcTree> surface_map_;
  double surface_map_time_s_{-1};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr surface_hint_map_;
  go2_surface_3d_planner::SurfaceSearchConfig surface_goal_config_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr surface_map_sub_;
  std::unique_ptr<FrontierExplorerCore> core_;
  struct CompletionEventConfig
  {
    bool enabled{false};
    // A transient-local Empty message keeps the event transport generic and cheap.
    std::string topic{"exploration_complete"};
  } completion_event_config_;

  // Parsed QoS policy and startup autodetect configuration.
  TopicQosProfiles topic_qos_profiles_;
  bool autostart_{true};
  bool control_service_enabled_{true};
  bool map_qos_autodetect_on_startup_{false};
  double map_qos_autodetect_timeout_s_{2.0};

  enum class RuntimeState
  {
    COLD_IDLE,
    RUNNING,
    STOPPING,
    SHUTDOWN_PENDING,
  };

  struct ScheduledControlRequest
  {
    uint8_t action{srv::ControlExploration::Request::ACTION_START};
    bool quit_after_stop{false};
  };

  // ROS interfaces.
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Service<srv::ControlExploration>::SharedPtr control_service_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr completion_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr floor_exhausted_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr verified_completion_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_frontier_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr optimized_map_pub_;

  // Subscriptions and timers.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::TimerBase::SharedPtr map_autodetect_timer_;
  rclcpp::TimerBase::SharedPtr map_processing_timer_;
  rclcpp::TimerBase::SharedPtr suppression_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr stop_completion_timer_;
  rclcpp::TimerBase::SharedPtr deferred_shutdown_timer_;
  std::optional<ScheduledControlRequest> scheduled_control_request_;
  bool pending_quit_after_stop_{false};
  bool quit_requested_{false};
  RuntimeState runtime_state_{RuntimeState::COLD_IDLE};
  bool suppression_activation_logged_{false};
  std::optional<std::chrono::steady_clock::time_point> suppression_activation_at_;

  // Startup-only map QoS autodetect state.
  std::mutex map_autodetect_mutex_;
  std::mutex pending_map_mutex_;
  bool map_received_once_{false};
  bool map_autodetect_complete_logged_{false};
  std::chrono::steady_clock::time_point map_autodetect_started_at_;
  std::optional<MapQosStartupAutodetect> map_qos_autodetect_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_pending_map_msg_;
  bool pending_map_update_{false};
  std::optional<double> effective_map_processing_rate_hz_;
  std::optional<std::chrono::steady_clock::time_point> last_map_arrival_at_;
  std::vector<double> startup_map_interval_samples_s_;

  // Completion can be observed multiple times while the frontier set stays empty.
  bool completion_event_published_{false};

  // TF warning throttling state to avoid log spam.
  std::optional<int64_t> last_tf_warning_time_ns_;
  double tf_warning_throttle_seconds_{5.0};
};

}  // namespace frontier_exploration_ros2
