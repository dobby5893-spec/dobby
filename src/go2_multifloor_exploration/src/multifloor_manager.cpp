#include "go2_multifloor_exploration/floor_handoff.hpp"
#include <frontier_exploration_ros2/map_consistency.hpp>
#include <frontier_exploration_ros2/floor_completion_v5.hpp>
#include "go2_multifloor_exploration/transition_footprint.hpp"
#include "go2_multifloor_exploration/simulation_progress.hpp"
#include "go2_multifloor_exploration/stair_transition.hpp"
#include <frontier_exploration_ros2/srv/control_exploration.hpp>
#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker.hpp>
#include <algorithm>
#include <atomic>
#include <limits>
#include <functional>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <future>
#include <optional>
#include <sstream>
#include <iomanip>

namespace go2_multifloor_exploration {
namespace maps=frontier_exploration_ros2;
using go2_surface_3d_planner::SurfaceGraph;
using go2_surface_3d_planner::configureSupportPlane;
using go2_surface_3d_planner::SurfaceSearchTimeout;
class MultifloorManager : public rclcpp::Node {
 friend struct MapConsistencyTestAccess;
 using Compute=nav2_msgs::action::ComputePathToPose;
 using Follow=nav2_msgs::action::FollowPath;
 using Control=frontier_exploration_ros2::srv::ControlExploration;
 using Session=maps::msg::FloorSession;
 using Participant=maps::msg::FloorParticipant;
 using Completion=maps::msg::FloorCompletion;
 // Values 0..8 are retained for existing checkpoint files.
 enum class State {WAIT_INITIAL_MAP,EXPLORE_FLOOR,STOPPING_FRONTIER,SEARCH_TRANSITION,
  PREPARING_STAIR_GAIT,COMPUTE_3D_PATH,FOLLOW_3D_PATH,STARTING_NEXT_FLOOR,COMPLETE,
  APPROACH_STAIR,OBSERVE_STAIR,VERIFY_LANDING,RESTORING_FLAT_GAIT,SESSION_WAIT};
 enum class Motion {APPROACH,ASCENT,OBSERVATION};
 struct Robot {SurfacePoint feet,body;};
 struct PendingHandoff {
  Completion message;
  maps::VerifiedFloorCompletion report;
  maps::AuditMapEvidence evidence;
  uint64_t request_id;
 };
 struct Handoff {
  std::string id, task, floor, report;
  double source_z;
  std::optional<StairCandidate> stairs;
 };
 struct SeenParticipant {
  Participant value;
  std::chrono::steady_clock::time_point seen;
 };
 struct SearchResult {
  StairScan scan;std::optional<StairCandidate> selected;
  std::optional<SurfacePoint> observation;std::vector<SurfacePoint> path;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr slice;TransitionFootprint footprint;
  bool selecting{false},handoff_check{false};SurfacePoint start;
  uint64_t handoff_request_id{0},source_tree_hash{0},context_epoch{0};maps::AuditMapEvidence source_evidence;
  rclcpp::Time stamp{0,0,RCL_ROS_TIME};
 };
public:
 MultifloorManager():Node("multifloor_manager") {
  map_frame_=declare_parameter<std::string>("map_frame","map");
  body_frame_=declare_parameter<std::string>("robot_base_frame","base_link");
  feet_frame_=declare_parameter<std::string>("robot_footprint_frame","base_footprint");
  floor_planner_=declare_parameter<std::string>("floor_planner_id","Floor3D");
  stair_planner_=declare_parameter<std::string>("stair_planner_id","Stair3D");
  controller_=declare_parameter<std::string>("controller_id","FollowPath");
  stair_checker_=declare_parameter<std::string>("stair_goal_checker_id","stair_goal_checker");
  entry_checker_=declare_parameter<std::string>("stair_entry_goal_checker_id","stair_entry_goal_checker");
  gait_node_=declare_parameter<std::string>("gait_controller_node","/quadruped_controller_node");
  gait_required_=declare_parameter<bool>("gait_control_required",true);
  flat_swing_=declare_parameter<double>("flat_swing_height",.04);
  flat_stance_=declare_parameter<double>("flat_stance_duration",.25);
  stair_swing_=declare_parameter<double>("stair_swing_height",.14);
  stair_stance_=declare_parameter<double>("stair_stance_duration",.36);
  separation_=declare_parameter<double>("floor_height_separation",1.0);
  visited_tolerance_=declare_parameter<double>("visited_floor_tolerance",.45);
  expected_floors_=declare_parameter<int>("expected_floor_count",0);
  settle_=declare_parameter<double>("floor_slice_settle_s",3.0);
  observe_wait_=declare_parameter<double>("stair_observation_wait_s",2.0);
  search_interval_=declare_parameter<double>("stair_scan_interval_s",5.0);
  search_budget_=declare_parameter<double>("stair_search_budget_s",3.0);
  rpc_timeout_=declare_parameter<double>("transition_rpc_timeout_s",10.0);
  motion_timeout_=declare_parameter<double>("stair_segment_timeout_s",90.0);
  progress_timeout_=declare_parameter<double>("stair_progress_timeout_s",25.0);
  max_map_age_=declare_parameter<double>("transition_map_max_age_s",3.0);
  max_observations_=declare_parameter<int>("stair_observation_attempts",4);
  suppression_time_=declare_parameter<double>("suppression_timeout_s",60.0);
  exploration_enabled_=declare_parameter<bool>("exploration_enabled",true);
  require_startup_ready_=declare_parameter<bool>("require_startup_ready",false);
  resume_=declare_parameter<bool>("resume_enabled",false);
  resume_z_=declare_parameter<double>("resume_floor_z",0.0);
  resume_phase_=declare_parameter<int>("resume_phase",1);
  has_residuals_=declare_parameter<bool>("resume_has_residuals",false);
  if(!resume_)has_residuals_=false;
  visited_=declare_parameter<std::vector<double>>("resume_visited_floors",std::vector<double>{});
  cfg_.continuous_floor_region=declare_parameter<std::vector<double>>("continuous_floor_region",std::vector<double>{});
  cfg_.xy_resolution=declare_parameter<double>("xy_resolution",.10);
  cfg_.z_resolution=declare_parameter<double>("z_resolution",.05);
  cfg_.collision_radius=declare_parameter<double>("collision_radius",.21);
  cfg_.cylinder_offset=declare_parameter<double>("cylinder_offset",.10);
  cfg_.body_height=declare_parameter<double>("body_height",.48);
  cfg_.clearance_margin=declare_parameter<double>("clearance_margin",.01);
  cfg_.support_probe_depth=declare_parameter<double>("support_probe_depth",.16);
  cfg_.max_step_height=declare_parameter<double>("max_step_height",.12);
  cfg_.snap_radius=declare_parameter<double>("snap_radius",.15);
  cfg_.snap_vertical_range=declare_parameter<double>("snap_vertical_range",.55);
  cfg_.max_iterations=declare_parameter<int>("max_iterations",50000);
  cfg_.infer_flat_support=true;
  options_.range=declare_parameter<double>("stair_detection_range",8.0);
  options_.minimum_steps=declare_parameter<int>("stair_minimum_treads",3);
  options_.minimum_chain_rise=declare_parameter<double>("stair_minimum_chain_rise",.14);
  options_.maximum_rise=cfg_.max_step_height;
  options_.maximum_segment_length=declare_parameter<double>("stair_segment_length",1.2);
  options_.minimum_width=declare_parameter<double>("stair_minimum_width",.50);
  options_.approach_distance=declare_parameter<double>("stair_approach_distance",.85);
  options_.landing_radius=declare_parameter<double>("new_floor_patch_radius",.70);
  if(search_budget_<=0 || rpc_timeout_<=0 || options_.minimum_steps<3 ||
   options_.maximum_segment_length<.5 || options_.minimum_width<2*cfg_.collision_radius ||
   options_.approach_distance<cfg_.collision_radius+cfg_.cylinder_offset+.15 ||
   stair_swing_<cfg_.max_step_height+.01 || expected_floors_<0)
   throw std::invalid_argument("Invalid stair transition parameters");
  clock_jump_=get_clock()->create_jump_callback(nullptr,
   [this](const rcl_time_jump_t &) noexcept {clock_rewound_.store(true);},
   rcl_jump_threshold_t{false,{0},{-1}});
  tf_=std::make_shared<tf2_ros::Buffer>(get_clock());listener_=std::make_shared<tf2_ros::TransformListener>(*tf_);
  gait_=std::make_shared<rclcpp::AsyncParametersClient>(this,gait_node_);
  auto qos=rclcpp::QoS(1).reliable().transient_local();
  map_sub_=create_subscription<octomap_msgs::msg::Octomap>(declare_parameter<std::string>("octomap_topic","/octomap_full"),qos,
   [this](octomap_msgs::msg::Octomap::SharedPtr msg) {
    if(msg->header.frame_id!=map_frame_ || msg->binary) {return;}
    std::unique_ptr<octomap::AbstractOcTree> raw(octomap_msgs::msgToMap(*msg));
    auto tree=dynamic_cast<octomap::OcTree*>(raw.get());if(!tree) {return;}
    if(map_ && rclcpp::Time(msg->header.stamp)<map_stamp_)return;
    raw.release();map_.reset(tree);map_stamp_=rclcpp::Time(msg->header.stamp);map_source_digest_=maps::octomapDigest(*msg);
   });
  exhausted_sub_=create_subscription<std_msgs::msg::String>(
   "/explore/floor_exhausted",qos,[this](std_msgs::msg::String::SharedPtr msg) {
    const auto diagnostic=parseFloorExhaustion(msg->data);
    RCLCPP_DEBUG(get_logger(),"Historical floor report (diagnostic only): parsed=%d",bool(diagnostic));
   });
  verified_sub_=create_subscription<Completion>(
   "/explore/floor_completion",qos,[this](Completion::ConstSharedPtr msg) {
    acceptFloorCompletion(*msg);
   });
  auto session_qos=rclcpp::QoS(5).reliable();
  session_pub_=create_publisher<Session>("/multifloor/floor_session",session_qos);
  participant_sub_=create_subscription<Participant>("/multifloor/session_participant",session_qos,
   [this](Participant::ConstSharedPtr msg){receiveParticipant(*msg);});
  startup_sub_=create_subscription<Session>("/multifloor/startup_ready",session_qos,
   [this](Session::ConstSharedPtr msg) {
    if(state_==State::STARTING_NEXT_FLOOR && session_.phase==Session::PREPARING &&
      msg->explorer_instance==session_.explorer_instance && msg->map_producer==session_.map_producer &&
      maps::sameSession(session_,msg->task_id,
      msg->floor_session_id,msg->generation) && msg->phase==Session::PREPARING)
     startup_ready_=true;
   });
  deferred_=create_publisher<std_msgs::msg::String>("/multifloor/exploration_deferred",qos);
  completion_sub_=create_subscription<std_msgs::msg::Empty>(
   declare_parameter<std::string>("floor_completion_topic","/exploration_complete"),qos,
   [this](std_msgs::msg::Empty::SharedPtr){
    RCLCPP_DEBUG(get_logger(),"Legacy Empty completion is diagnostic only; joint audit required");
   });
  paired_floor_sub_=create_subscription<maps::msg::FloorMapSnapshot>("/map_floor/snapshot",qos,
    [this](maps::msg::FloorMapSnapshot::ConstSharedPtr msg) {
      if(msg->floor_map.header.frame_id!=map_frame_ || !maps::sessionSnapshot(session_,*msg))return;
      if(paired_floor_map_ && (msg->floor_map_generation<=paired_floor_map_->version.floor_map_generation ||
        msg->octomap_generation<paired_floor_map_->version.octomap_generation))return;
      const auto pair=maps::decodeFloorSnapshot(*msg,map_,map_source_digest_);
      if(!pair)return;
      paired_floor_map_=pair;
      slice_map_=pair->floor;
    });
  checkpoint_=create_publisher<std_msgs::msg::String>("/multifloor/checkpoint_state",qos);
  floor_pub_=create_publisher<std_msgs::msg::Float64>(declare_parameter<std::string>("floor_height_topic","/multifloor/active_floor_height"),qos);
  mode_pub_=create_publisher<std_msgs::msg::Bool>("/multifloor/stair_mode",qos);
  transition_mask_pub_=create_publisher<nav_msgs::msg::OccupancyGrid>("/multifloor/transition_mask",qos);
  regions_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("/multifloor/stair_regions",qos);
  done_=create_publisher<std_msgs::msg::Empty>("/multifloor/exploration_complete",qos);
  marker_=create_publisher<visualization_msgs::msg::Marker>("/multifloor/transition_target",qos);
  control_=create_client<Control>(declare_parameter<std::string>("frontier_control_service","/control_exploration"));
  compute_=rclcpp_action::create_client<Compute>(this,"/compute_path_to_pose");
  follow_=rclcpp_action::create_client<Follow>(this,"/follow_path");
  const double period=declare_parameter<double>("timer_period_s",1.0);
  timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(std::max(.1,period))),[this]{tick();});
  publishMode(false);publishRegions();
 }
private:
 void receiveParticipant(const Participant & participant) {
  if(!maps::sessionIdentity(participant.instance_id))return;
  auto * seen=participant.role==Participant::EXPLORER?&explorer_:
    participant.role==Participant::SLICE?&slice_:nullptr;
  if(!seen)return;
  const auto & expected=participant.role==Participant::EXPLORER?session_.explorer_instance:session_.map_producer;
  if((!expected.empty() && participant.instance_id!=expected) ||
    (expected.empty() && participant.generation))return;
  *seen=SeenParticipant{participant,std::chrono::steady_clock::now()};
 }
 bool participantCurrent(const SeenParticipant & seen,uint8_t phase=0) const {
  return std::chrono::duration<double>(std::chrono::steady_clock::now()-seen.seen).count()<=rpc_timeout_ &&
    maps::sameSession(session_,seen.value.task_id,seen.value.floor_session_id,seen.value.generation) &&
    seen.value.phase && (!phase || seen.value.phase==phase);
 }
 bool available(const SeenParticipant & seen) const {
  return !seen.value.instance_id.empty() && seen.value.idle && !seen.value.generation &&
    std::chrono::duration<double>(std::chrono::steady_clock::now()-seen.seen).count()<=rpc_timeout_;
 }
 void setPhase(uint8_t phase) {
  session_.phase=phase;
  publishSession();
 }
 void publishSession() {
  if(!session_.generation)return;
  session_.heartbeat=now();
  session_pub_->publish(session_);
 }
 void openFloorSession() {
  if(session_.task_id.empty()) {
   session_.task_id=maps::newSessionIdentity();
   session_.explorer_instance=explorer_.value.instance_id;
   session_.map_producer=slice_.value.instance_id;
   RCLCPP_INFO(get_logger(),"Created exploration task %s",session_.task_id.c_str());
  }
  session_.floor_session_id=maps::newSessionIdentity();
  ++session_.generation;
  session_.floor_z=active_floor_;
  session_.started_at=now();
  ++context_epoch_;
  ++epoch_;
  invalidateHandoff();
  report_sequence_=0;
  handled_reports_.clear();
  paired_floor_map_.reset();
  slice_map_.reset();
  handoff_.reset();
  pre_handoff_pose_.reset();
  selected_.reset();
  publishFloor();
  switch_time_=sim();
  state_=State::STARTING_NEXT_FLOOR;
  setPhase(Session::PREPARING);
  RCLCPP_INFO(get_logger(),"Opened floor session %s task=%s z=%.3f",
    session_.floor_session_id.c_str(),session_.task_id.c_str(),active_floor_);
 }
 void acceptFloorCompletion(const Completion & message) {
  if(clock_rewound_.load() || !maps::completionIdentity(message,session_,now().nanoseconds())) {
   RCLCPP_DEBUG(get_logger(),"Ignored floor completion from an incompatible session or producer");
   return;
  }
  if(message.revoked) {
   report_sequence_=std::max(report_sequence_,message.sequence);
   if(pending_handoff_ && message.report_id==pending_handoff_->message.report_id &&
     message.sequence==pending_handoff_->message.sequence) {
    RCLCPP_INFO(get_logger(),"Publisher revoked floor report %s",message.report_id.c_str());
    invalidateHandoff();
   }
   return;
  }
  if(state_!=State::EXPLORE_FLOOR || session_.phase!=Session::ACTIVE ||
    !participantCurrent(explorer_,Session::ACTIVE) || !participantCurrent(slice_,Session::ACTIVE))return;
  const auto report=maps::verifyFloorCompletion(message);
  if(!report || !reportFresh(*report))return;
  const auto evidence=maps::completionMapEvidence(message);
  const auto measured=robot();
  if(!evidence || !measured || std::abs(measured->feet.z-active_floor_)>=.15 ||
    std::hypot(measured->feet.x-evidence->start_x,measured->feet.y-evidence->start_y)>.3 ||
    !pairedInputsReady() || !maps::mapEvidenceCurrent(*evidence,*paired_floor_map_,liveAuditTree()))return;
  if(message.sequence<=report_sequence_) {
   // A live reassessment may refresh validity of the same still-pending report.
   // It cannot replace its request identity or create a new handoff.
   if(pending_handoff_ && message.sequence==report_sequence_ &&
     message.report_id==pending_handoff_->message.report_id &&
     report->stamp>pending_handoff_->report.stamp &&
     report->records==pending_handoff_->report.records &&
     report->remaining==pending_handoff_->report.remaining &&
     report->residual==pending_handoff_->report.residual &&
     report->transition==pending_handoff_->report.transition &&
     report->status==pending_handoff_->report.status &&
     message.transition_digest==pending_handoff_->message.transition_digest &&
     message.runtime_digest==pending_handoff_->message.runtime_digest &&
     evidence->stableDigest()==pending_handoff_->evidence.stableDigest()) {
     pending_handoff_->report=*report;
     pending_handoff_->message.generated_at=message.generated_at;
   }
   RCLCPP_DEBUG(get_logger(),"Ignored duplicate authorization %s",message.report_id.c_str());
   return;
  }
  if(handled_reports_.size()>=4096 || !handled_reports_.insert(message.report_id).second)return;
  report_sequence_=message.sequence;
  pending_handoff_=PendingHandoff{message,*report,*evidence,++handoff_request_id_};
  completion_pending_=lastFloor();
  next_scan_=sim();
  RCLCPP_INFO(get_logger(),"Accepted floor report %s floor=%s",message.report_id.c_str(),message.floor_session_id.c_str());
 }
 double sim() const {return now().seconds();}
 bool lastFloor() const {return expected_floors_>0 && static_cast<int>(visited_.size())>=expected_floors_;}
 bool reportFresh(const maps::VerifiedFloorCompletion & report) const {
  return std::abs(report.floor_z-active_floor_)<1e-6 && sim()-report.stamp>=-.1 && sim()-report.stamp<=5.;
 }
 const octomap::OcTree * liveAuditTree() const {
  return map_ && (!paired_floor_map_ || map_stamp_.nanoseconds()>=paired_floor_map_->source_stamp_ns)?map_.get():nullptr;
 }
 bool pairedInputsReady() const {
  return paired_floor_map_ && paired_floor_map_->floor && liveAuditTree() && freshMap() &&
   (now()-rclcpp::Time(paired_floor_map_->floor->header.stamp)).seconds()>=-.1 &&
   (now()-rclcpp::Time(paired_floor_map_->floor->header.stamp)).seconds()<=max_map_age_ &&
   std::abs(paired_floor_map_->floor->info.origin.position.z-active_floor_)<1e-6;
 }
 bool pendingHandoffValid() const {
  // At entry to CLOSING the report was fresh. Subsequent gates revalidate
  // current maps and stance within the bounded, already-admitted transaction.
  const bool time_valid=pending_handoff_ && (closing_started_at_<0 ?
    reportFresh(pending_handoff_->report) :
    sim()>=closing_started_at_ && sim()-closing_started_at_<=rpc_timeout_);
  return time_valid && pairedInputsReady() &&
   maps::completionIdentity(pending_handoff_->message,session_,now().nanoseconds()) &&
   participantCurrent(explorer_) && participantCurrent(slice_) &&
   maps::mapEvidenceCurrent(pending_handoff_->evidence,*paired_floor_map_,liveAuditTree());
 }
 bool handoffReady() const {
  if(!pendingHandoffValid() || !maps::pairedSourceCaughtUp(*paired_floor_map_,liveAuditTree()))return false;
  const auto measured=robot();
  const auto & evidence=pending_handoff_->evidence;
  return measured && std::hypot(measured->feet.x-evidence.start_x,measured->feet.y-evidence.start_y)<=.3 &&
    std::abs(measured->feet.z-active_floor_)<.15;
 }
 void invalidateHandoff() {
  pending_handoff_.reset();completion_pending_=false;closing_started_at_=-1.;++handoff_request_id_;
 }
 void waitForSession(const char * reason) {
  invalidateHandoff();
  if(state_!=State::SESSION_WAIT) {
   RCLCPP_WARN(get_logger(),"Floor session waiting: %s (task=%s floor=%s)",
     reason,session_.task_id.c_str(),session_.floor_session_id.c_str());
  }
  state_=State::SESSION_WAIT;
 }
 void consumeHandoff() {
  const auto & pending=*pending_handoff_;
  handoff_=Handoff{maps::newSessionIdentity(),session_.task_id,session_.floor_session_id,
    pending.message.report_id,active_floor_,selected_};
  has_residuals_|=pending.report.remaining>0;
  residual_report_="report="+pending.message.report_id+" regions="+pending.report.records;
  RCLCPP_INFO(get_logger(),"Closed floor %s and consumed report %s as handoff %s",
    handoff_->floor.c_str(),handoff_->report.c_str(),handoff_->id.c_str());
  invalidateHandoff();
  setPhase(Session::CLOSED);
 }
 void publishMode(bool enabled) {stair_mode_=enabled;std_msgs::msg::Bool m;m.data=enabled;mode_pub_->publish(m);}
 void publishFloor() {std_msgs::msg::Float64 m;m.data=active_floor_;floor_pub_->publish(m);}
 std::optional<Robot> robot() const {
  try {
   const auto body=tf_->lookupTransform(map_frame_,body_frame_,tf2::TimePointZero);
   const double age=(now()-rclcpp::Time(body.header.stamp)).seconds();if(age<-.1 || age>.5) return {};
   const auto feet=tf_->lookupTransform(map_frame_,feet_frame_,rclcpp::Time(body.header.stamp));
   Robot r;const auto & p=body.transform.translation;const auto & f=feet.transform.translation;
   r.body={p.x,p.y,p.z,tf2::getYaw(body.transform.rotation)};
   // Use the body projection at measured foot height consistently with the path controller.
   r.feet={p.x,p.y,f.z,r.body.yaw};return r;
  }catch(const tf2::TransformException &){return {};}
 }
 bool freshMap() const {double age=(now()-map_stamp_).seconds();return map_ && age>=-.1 && age<=max_map_age_;}
 SurfaceSearchConfig config(const Robot & r,bool strict) const {
  auto c=cfg_;c.support_reference_height=r.body.z-r.feet.z;c.flat_support_requires_prior=strict;
  configureSupportPlane(c,*map_,r.body);c.excluded_regions.clear();
  if(!strict) for(const auto & entry:entries_) c.excluded_regions.push_back(entry.region);
  return c;
 }
 bool visited(double z) const {return std::any_of(visited_.begin(),visited_.end(),[&](double v){return std::abs(v-z)<visited_tolerance_;});}
 void publishRegions() {
  std_msgs::msg::Float64MultiArray m;m.layout.dim.resize(1);m.layout.dim[0].label=map_frame_;
  m.layout.dim[0].size=entries_.size();m.layout.dim[0].stride=7;
  for(const auto & e:entries_) {const auto & r=e.region;m.data.insert(m.data.end(),{r.x,r.y,r.yaw,r.half_length,r.half_width,r.min_z,r.max_z});}
  regions_pub_->publish(m);
 }
 void remember(const StairScan & scan) {
  bool changed=false;
  for(const auto & c:scan.candidates) {
   if(std::none_of(entries_.begin(),entries_.end(),[&](const auto & old){return std::hypot(old.first_tread.x-c.first_tread.x,old.first_tread.y-c.first_tread.y)<.65 &&
    std::abs(old.first_tread.z-c.first_tread.z)<.15 && std::abs(std::remainder(old.first_tread.yaw-c.first_tread.yaw,2*M_PI))<.35;})) {
    if(entries_.size()<128) {entries_.push_back(c);changed=true;}
   }
  }
  if(changed) publishRegions();
 }
 void publishState() {
  std::ostringstream s;s<<"{\"phase\":"<<static_cast<int>(state_)<<",\"active_floor_z\":"<<active_floor_<<",\"visited_floors\":[";
  for(std::size_t i=0;i<visited_.size();++i) {if(i)s<<",";s<<visited_[i];}
  s<<"],\"stair_segments\":"<<segments_<<",\"stair_mode\":"<<(stair_mode_?"true":"false")<<",\"has_residuals\":"<<(has_residuals_?"true":"false")<<",\"residual_report\":"<<std::quoted(residual_report_)<<",\"task_id\":"<<std::quoted(session_.task_id)
   <<",\"floor_session_id\":"<<std::quoted(session_.floor_session_id)<<"}";
  std_msgs::msg::String m;m.data=s.str();checkpoint_->publish(m);
 }
 uint64_t begin(State state) {state_=state;rpc_pending_=true;rpc_deadline_=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(rpc_timeout_));return ++epoch_;}
 void cancelOwnGoals() {
  if(compute_handle_) compute_->async_cancel_goal(compute_handle_);
  if(follow_handle_) follow_->async_cancel_goal(follow_handle_);
  compute_handle_.reset();
  // FollowPath owns motion until its terminal result, even after requesting cancellation.
 }
 void fail(const std::string & reason) {
   const auto measured=robot();
   if(stair_mode_ && measured && measured->feet.z>active_floor_+.06)ascent_started_=true;
   ++epoch_;rpc_pending_=false;cancelOwnGoals();
   RCLCPP_WARN(get_logger(),"Stair transition waiting: %s; no floor switch committed",reason.c_str());
   if(selected_) {suppressed_.push_back({selected_->first_tread,sim()+suppression_time_});selected_.reset();}
   state_=ascent_started_?State::OBSERVE_STAIR:State::SEARCH_TRANSITION;
   observe_since_=sim();next_search_=sim()+3.;
   if(!ascent_started_ && stair_mode_ && measured &&
      std::abs(measured->feet.z-active_floor_)<.04)restoreEntryGait();
 }
 void restoreEntryGait() {
   gait(false,[this](bool ok) {
     if(ok) {state_=State::SEARCH_TRANSITION;next_search_=sim()+3.;}
     else {
       gait_callback_=[this](bool accepted) {
         if(accepted) {state_=State::SEARCH_TRANSITION;next_search_=sim()+3.;}
         else restoreEntryGait();
       };
       gait_retry_=std::chrono::steady_clock::now()+std::chrono::seconds(1);
     }
   });
 }
 void stopFrontier() {
  if(!control_->service_is_ready() || rpc_pending_ || !participantCurrent(explorer_,Session::CLOSING))return;
  if(pending_handoff_ && !handoffReady())invalidateHandoff();
  const auto id=begin(state_==State::SESSION_WAIT?State::SESSION_WAIT:State::STOPPING_FRONTIER);
  auto request=std::make_shared<Control::Request>();request->action=Control::Request::ACTION_STOP;
  maps::stampSessionControl(*request,session_,id);
  const auto request_id=pending_handoff_?pending_handoff_->request_id:0;
  control_->async_send_request(request,[this,id,request_id](rclcpp::Client<Control>::SharedFuture future) {
   if(id!=epoch_)return;
   rpc_pending_=false;
   try {
    const auto reply=future.get();
    if(!reply->accepted || !maps::sessionControlMatches(*reply,session_) ||
      reply->command_sequence!=id || reply->state!=Control::Request::STATE_IDLE)return;
    if(!handoffReady() || pending_handoff_->request_id!=request_id) {
     const bool recoverable=state_!=State::SESSION_WAIT;
     waitForSession("authorization changed before Explorer reached idle");
     restart_floor_when_closed_=recoverable;
     setPhase(Session::CLOSED);
     return;
    }
    if(lastFloor()) {
     consumeHandoff();
     state_=State::COMPLETE;
     if(has_residuals_) {
      std_msgs::msg::String msg;msg.data="Expected floors processed; residual unknowns remain; "+residual_report_;
      deferred_->publish(msg);
     } else done_->publish(std_msgs::msg::Empty());
     return;
    }
    state_=State::SEARCH_TRANSITION;
    next_search_=sim();
   }catch(const std::exception & e){RCLCPP_WARN(get_logger(),"%s",e.what());}
  });
 }
 void startFrontier() {
  if(!control_->service_is_ready() || rpc_pending_ || follow_owner_)return;
  const auto id=begin(State::STARTING_NEXT_FLOOR);
  auto request=std::make_shared<Control::Request>();request->action=Control::Request::ACTION_START;
  maps::stampSessionControl(*request,session_,id);
  control_->async_send_request(request,[this,id](rclcpp::Client<Control>::SharedFuture future) {
   if(id!=epoch_)return;
   rpc_pending_=false;
   try {
    const auto reply=future.get();
    if(reply->accepted && maps::sessionControlMatches(*reply,session_) && reply->command_sequence==id &&
      reply->state==Control::Request::STATE_RUNNING && participantCurrent(explorer_,Session::ACTIVE) &&
      participantCurrent(slice_,Session::ACTIVE) && pairedInputsReady()) {
     state_=State::EXPLORE_FLOOR;
     RCLCPP_INFO(get_logger(),"Explorer running for floor %s",session_.floor_session_id.c_str());
    }
   }catch(const std::exception & e){RCLCPP_WARN(get_logger(),"%s",e.what());}
  });
 }
 bool checkBeforeMotion() {
  if(clock_rewound_.load())return false;
  if(handoff_)return true;
  if(handoffReady() && participantCurrent(explorer_,Session::CLOSING))return true;
  waitForSession("completion evidence changed before motion");
  restart_floor_when_closed_=true;
  setPhase(Session::CLOSED);
  return false;
 }
 void gait(bool stairs,std::function<void(bool)> callback) {
  desired_gait_=stairs;gait_callback_=std::move(callback);requestGait();
 }
 void requestGait() {
  if(!gait_->service_is_ready()) {
   auto callback=gait_callback_;gait_callback_={};
   if(callback) callback(!gait_required_);
   return;
  }
  const bool stairs=desired_gait_;
  const auto id=begin(stairs?State::PREPARING_STAIR_GAIT:State::RESTORING_FLAT_GAIT);
  gait_->set_parameters_atomically({rclcpp::Parameter("gait.swing_height",stairs?stair_swing_:flat_swing_),
    rclcpp::Parameter("gait.stance_duration",stairs?stair_stance_:flat_stance_)},
   [this,id,stairs](auto future) {
    if(id!=epoch_) {return;}
    rpc_pending_=false;
    bool ok=false;
    try {ok=future.get().successful;}
    catch(const std::exception &) {
     gait_retry_=std::chrono::steady_clock::now()+std::chrono::seconds(1);
     return;
    }
    if(ok) publishMode(stairs);
    auto callback=gait_callback_;gait_callback_={};
    if(callback) callback(ok || !gait_required_);
   });
 }
 void requestPath(const SurfacePoint & target,Motion motion) {
  if(follow_owner_ || !checkBeforeMotion())return;
  if(!compute_->action_server_is_ready()) {fail("planner action unavailable");return;}
  visualization_msgs::msg::Marker marker;marker.header.frame_id=map_frame_;marker.header.stamp=now();
  marker.ns="multifloor_transition";marker.id=0;marker.type=visualization_msgs::msg::Marker::SPHERE;
  marker.action=visualization_msgs::msg::Marker::ADD;marker.pose.position.x=target.x;marker.pose.position.y=target.y;
  marker.pose.position.z=target.z+.15;marker.pose.orientation.w=1.;marker.scale.x=marker.scale.y=marker.scale.z=.25;
  marker.color.r=1.;marker.color.g=.55;marker.color.a=1.;marker_->publish(marker);
  motion_=motion;motion_target_=target;const auto id=begin(State::COMPUTE_3D_PATH);
  const auto context=context_epoch_;
  const auto authorization=pending_handoff_?pending_handoff_->request_id:0;
  const auto approach_map=authorization?maps::wholeOctomapDigest(*map_):0;
  Compute::Goal goal;goal.goal.header.frame_id=map_frame_;goal.goal.header.stamp=now();
  goal.goal.pose.position.x=target.x;goal.goal.pose.position.y=target.y;goal.goal.pose.position.z=target.z;
  tf2::Quaternion q;q.setRPY(0,0,target.yaw);goal.goal.pose.orientation=tf2::toMsg(q);
  goal.planner_id=(!ascent_started_ && motion!=Motion::ASCENT)?floor_planner_:stair_planner_;
  rclcpp_action::Client<Compute>::SendGoalOptions options;
  options.goal_response_callback=[this,id,context](auto h) {
   if(id!=epoch_ || context!=context_epoch_) {if(h)compute_->async_cancel_goal(h);return;}
   compute_handle_=h;if(!h) fail("path goal rejected");
  };
  options.result_callback=[this,id,context,authorization,approach_map](const auto & result) {
   if(id!=epoch_ || context!=context_epoch_)return;
   rpc_pending_=false;compute_handle_.reset();
   if(authorization) {
    if(!checkBeforeMotion() || !pending_handoff_ || pending_handoff_->request_id!=authorization)return;
    if(approach_map!=maps::wholeOctomapDigest(*map_)) {
     fail("map geometry changed during stair approach planning");return;
    }
   }
   if(result.code!=rclcpp_action::ResultCode::SUCCEEDED || !result.result || result.result->path.poses.empty()) {fail("no fresh verified segment path");return;}
   auto path=result.result->path;
   if(path.header.frame_id!=map_frame_) {fail("wrong path frame");return;}
   if(motion_!=Motion::APPROACH) {
    double length=0.;std::size_t end=1;
    for(;end<path.poses.size();++end) {
     const auto & a=path.poses[end-1].pose.position;const auto & b=path.poses[end].pose.position;
     length+=std::hypot(b.x-a.x,b.y-a.y);
     if(length>options_.maximum_segment_length && end>1) {break;}
    }
    path.poses.resize(end);
   }
   const auto & p=path.poses.back().pose;motion_target_={p.position.x,p.position.y,p.position.z,tf2::getYaw(p.orientation)};
   followPath(path);
  };
  compute_->async_send_goal(goal,options);
 }
 void followPath(const nav_msgs::msg::Path & path) {
  if(follow_owner_ || !checkBeforeMotion())return;
  if(!follow_->action_server_is_ready()) {fail("controller action unavailable");return;}
  const auto id=++epoch_;rpc_pending_=true;follow_owner_=id;
  if(!handoff_)consumeHandoff();
  rpc_deadline_=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(rpc_timeout_));
  state_=motion_==Motion::APPROACH?State::APPROACH_STAIR:State::FOLLOW_3D_PATH;
  motion_start_=last_progress_=sim();progress_pose_.reset();
  Follow::Goal goal;goal.path=path;goal.controller_id=controller_;goal.goal_checker_id=motion_==Motion::APPROACH?entry_checker_:stair_checker_;
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback=[this,id](auto h) {
   if(!h && follow_owner_==id)follow_owner_=0;
   if(id!=epoch_) {if(h)follow_->async_cancel_goal(h);return;}
   rpc_pending_=false;follow_handle_=h;if(!h) fail("segment rejected by controller");
  };
  options.result_callback=[this,id](const auto & result) {
   if(follow_owner_==id){follow_owner_=0;follow_handle_.reset();}
   if(id!=epoch_) {return;}
   if(result.code!=rclcpp_action::ResultCode::SUCCEEDED) {fail("segment execution failed");return;}
   auto actual=robot();if(!actual || !reachedStairTarget(actual->feet,motion_target_)) {fail("action success without measured segment arrival");return;}
   if(motion_==Motion::APPROACH) {
    if(!selected_) {fail("missing stair entry");return;}
    if(!reachedStairTarget(actual->feet,selected_->approach,.20,.08) ||
     std::abs(std::remainder(actual->feet.yaw-selected_->approach.yaw,2*M_PI))>.20) {state_=State::SEARCH_TRANSITION;next_search_=sim();return;}
    gait(true,[this](bool ok) {
     if(!ok) {fail("stair gait not acknowledged");return;}
     requestPath(selected_->target,Motion::ASCENT);
    });return;
   }
   if(motion_==Motion::ASCENT) {ascent_started_|=actual->feet.z>active_floor_+.06;++segments_;}observe_since_=sim();last_segment_map_=map_stamp_;stable_since_=-1;
   state_=State::OBSERVE_STAIR;
  };
  follow_->async_send_goal(goal,options);
 }
 void launchSearch(const Robot & robot,bool selecting) {
  if(search_.valid() || !freshMap() || follow_owner_) {return;}
  suppressed_.erase(std::remove_if(suppressed_.begin(),suppressed_.end(),[this](const auto & item){return item.second<=sim();}),suppressed_.end());
  auto blocked=suppressed_;
  const bool handoff_check=!handoff_ && pending_handoff_.has_value();
  if(handoff_check && !handoffReady())return;
  auto tree=handoff_check?paired_floor_map_->tree:map_;auto c=config(robot,ascent_started_);auto known=entries_;auto points=observation_points_;
  if(handoff_check)configureSupportPlane(c,*tree,robot.body);
  const auto stance=robot.feet;auto opt=options_;bool climbing=ascent_started_;auto stamp=map_stamp_;
  auto slice=handoff_check?paired_floor_map_->floor:slice_map_;
  const auto request_id=handoff_check?pending_handoff_->request_id:0;
  const auto context_epoch=context_epoch_;
  const bool before_stop=state_==State::EXPLORE_FLOOR;
  const auto evidence=handoff_check?pending_handoff_->evidence:maps::AuditMapEvidence{};
  c.search_deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(search_budget_));
  search_=std::async(std::launch::async,[tree,c,known,points,stance,opt,climbing,stamp,selecting,blocked,slice,handoff_check,request_id,evidence,context_epoch,before_stop]() mutable {
   SearchResult result;result.stamp=stamp;result.selecting=selecting;result.handoff_check=handoff_check;result.start=stance;result.slice=slice;
   result.handoff_request_id=request_id;result.source_evidence=evidence;result.context_epoch=context_epoch;
   if(handoff_check)result.source_tree_hash=maps::wholeOctomapDigest(*tree);
   try {
    result.scan=detectStairs(*tree,c,stance,opt);
    for(const auto & candidate:result.scan.candidates) known.push_back(candidate);
    if(slice)result.footprint=transitionFootprint(*tree,known,slice->info.width,slice->info.height,
      slice->info.resolution,slice->info.origin.position.x,slice->info.origin.position.y,
      slice->info.origin.position.z,c.max_step_height,c.search_deadline);
    if(!selecting)return result;
    if(!climbing) {c.excluded_regions.clear();for(const auto & entry:known)c.excluded_regions.push_back(entry.region);}
    for(const auto & candidate:known) {
     if(std::any_of(blocked.begin(),blocked.end(),[&](const auto & item){return std::hypot(item.first.x-candidate.first_tread.x,item.first.y-candidate.first_tread.y)<.65 && std::abs(item.first.z-candidate.first_tread.z)<.15;})) {continue;}
     if(candidate.first_tread.z<stance.z+.025 || candidate.first_tread.z>stance.z+opt.maximum_rise+.05) {continue;}
     if(std::chrono::steady_clock::now()>=c.search_deadline) {break;}
     auto attempt=c;attempt.search_deadline=std::min(c.search_deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(handoff_check?2000:500));
     try {
      auto path=handoff_check && before_stop?SurfaceGraph(*tree,attempt).plan(stance,candidate.approach):
        climbing?SurfaceGraph(*tree,attempt).plan(stance,candidate.target):verifyStairApproach(*tree,attempt,stance,candidate);
      if(handoff_check && (!path.empty()) && (std::hypot(path.back().x-candidate.approach.x,path.back().y-candidate.approach.y)>.10 ||
        std::any_of(path.begin(),path.end(),[&](auto p){return std::abs(p.z-stance.z)>.08;})))path.clear();
      if(!path.empty()) {result.selected=candidate;result.path=std::move(path);return result;}
     }catch(const SurfaceSearchTimeout &) {}
    }
    if(handoff_check)return result;
    // At a partially observed flight/turnaround, advance only on verified, same-height support.
    for(double distance:{.8,1.2}) for(int i=0;i<8;++i) {
     if(std::chrono::steady_clock::now()>=c.search_deadline) return result;
     const double yaw=i*M_PI/4;SurfacePoint target{stance.x+distance*std::cos(yaw),stance.y+distance*std::sin(yaw),stance.z,stance.yaw};
     if(std::any_of(points.begin(),points.end(),[&](const auto & p){return std::hypot(p.x-target.x,p.y-target.y)<.5;})) {continue;}
     auto attempt=c;attempt.search_deadline=std::min(c.search_deadline,std::chrono::steady_clock::now()+std::chrono::milliseconds(150));
     try {
      auto path=SurfaceGraph(*tree,attempt).plan(stance,target);bool flat=!path.empty();
      for(const auto & p:path)if(std::abs(p.z-stance.z)>.08)flat=false;
      if(flat) {result.observation=path.back();result.path=std::move(path);return result;}
     }catch(const SurfaceSearchTimeout &) {}
    }
   }catch(const SurfaceSearchTimeout &){result.scan.timed_out=true;}
   return result;
  });
 }
 void collectSearch() {
  if(!search_.valid() || search_.wait_for(std::chrono::seconds(0))!=std::future_status::ready) {return;}
  SearchResult result;
  try {result=search_.get();}catch(const std::exception & e){RCLCPP_WARN(get_logger(),"Stair search: %s",e.what());next_search_=sim()+3;return;}
  if(result.context_epoch!=context_epoch_ || state_==State::SESSION_WAIT ||
    (result.selecting && (now()-result.stamp).seconds()<-.1))return;
  if((now()-result.stamp).seconds()>max_map_age_) {next_search_=sim();return;}
  if(result.handoff_check) {
   // Correlate with the exact report captured at launch, before any side effects.
   if((state_!=State::EXPLORE_FLOOR && state_!=State::SEARCH_TRANSITION) || !handoffReady() ||
      result.handoff_request_id!=pending_handoff_->request_id ||
      !maps::mapEvidenceCurrent(result.source_evidence,*paired_floor_map_,liveAuditTree()) ||
      result.source_tree_hash!=maps::wholeOctomapDigest(*paired_floor_map_->tree) ||
      (liveAuditTree() && result.source_tree_hash!=maps::wholeOctomapDigest(*liveAuditTree()))) {
     next_scan_=sim();return;
   }
  }
  if(!result.handoff_check && !result.selecting && state_!=State::EXPLORE_FLOOR)return;
  remember(result.scan);
  if(result.slice && result.footprint.complete && std::abs(result.slice->info.origin.position.z-active_floor_)<.15) {
    auto mask=*result.slice;mask.header.stamp=now();mask.data=result.footprint.cells;
    transition_mask_pub_->publish(mask);
  }
  if(result.handoff_check && state_==State::EXPLORE_FLOOR) {
    const auto current=robot();
    if(result.selected && pendingHandoffValid() &&
       current && std::hypot(current->feet.x-result.start.x,current->feet.y-result.start.y)<.2) {
      completion_pending_=true;selected_=result.selected;pre_handoff_pose_=current->feet;
      RCLCPP_WARN(get_logger(),"Complete stair-entry route verified; flat-floor audit may now hand over execution");
    } else RCLCPP_INFO(get_logger(),"Handoff remains pending: no complete fresh stair-entry route yet");
    return;
  }
  if(!result.selecting || (state_!=State::SEARCH_TRANSITION && state_!=State::OBSERVE_STAIR)) {return;}
  next_search_=sim()+search_interval_;
  if(result.selected) {
   selected_=result.selected;observation_attempts_=0;
   if(ascent_started_) requestPath(result.path.back(),Motion::ASCENT);
   else requestPath(result.path.back(),Motion::APPROACH);
  } else if(result.observation && observation_attempts_<max_observations_) {
   ++observation_attempts_;observation_points_.push_back(*result.observation);
   if(!ascent_started_) requestPath(*result.observation,Motion::OBSERVATION);
   else if(!stair_mode_) gait(true,[this,target=*result.observation](bool ok){if(ok)requestPath(target,Motion::OBSERVATION);else fail("observation gait unavailable");});
   else requestPath(*result.observation,Motion::OBSERVATION);
  } else {
   RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),10000,
    "No verified stair continuation yet; waiting for observed support, not declaring exploration complete");
  }
 }
 void checkLanding(const Robot & r) {
  if(r.feet.z<active_floor_+separation_ || visited(r.feet.z) || !freshMap()) {stable_since_=-1;if(state_==State::VERIFY_LANDING)state_=State::OBSERVE_STAIR;return;}
  auto c=config(r,true);c.search_deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(200);
  auto strict=c;strict.infer_flat_support=false;
  auto surface=SurfaceGraph(*map_,strict).snapToSurface(r.feet);
  if(!surface || !hasLandingPatch(*map_,strict,*surface,options_.landing_radius)) {stable_since_=-1;if(state_==State::VERIFY_LANDING)state_=State::OBSERVE_STAIR;return;}
  state_=State::VERIFY_LANDING;
  if(stable_since_<0 || std::hypot(r.feet.x-stable_pose_.x,r.feet.y-stable_pose_.y)>.08 || std::abs(r.feet.z-stable_pose_.z)>.04) {
   stable_since_=sim();stable_pose_=r.feet;return;
  }
  if(sim()-stable_since_<settle_) {return;}
  const double new_height=r.feet.z;
  gait(false,[this,new_height](bool ok) {
   if(!ok) {fail("flat gait restoration not acknowledged");return;}
   active_floor_=new_height;visited_.push_back(new_height);ascent_started_=false;selected_.reset();
   observation_points_.clear();observation_attempts_=0;
   landing_ready_=true;state_=State::STARTING_NEXT_FLOOR;
   RCLCPP_INFO(get_logger(),"Verified new floor at z=%.2f; waiting for refreshed floor slice",active_floor_);
  });
 }
 void tick() {
  publishState();
  const auto wall=std::chrono::steady_clock::now();
  const bool rewound=clock_rewound_.exchange(false);
  if(simulation_progress_.initialized && (rewound || sim()<simulation_progress_.last_sim)) {
   ++epoch_;++context_epoch_;rpc_pending_=false;cancelOwnGoals();restart_floor_when_closed_=false;
   waitForSession("ROS clock moved backwards; restart the launch after confirming motion has stopped");
   setPhase(Session::CLOSED);
   simulation_progress_.sample(sim(),std::chrono::duration<double>(wall.time_since_epoch()).count());
   return;
  }
  publishSession();
  const bool advancing=simulation_progress_.sample(sim(),std::chrono::duration<double>(wall.time_since_epoch()).count());
  if(simulation_progress_.extend_deadlines>0) {
    const auto extension=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(simulation_progress_.extend_deadlines));
    rpc_deadline_+=extension;gait_retry_+=extension;
  }
  if(!advancing)return;
  auto current=robot();
   if(stair_mode_ && motion_==Motion::ASCENT && current &&
      current->feet.z>active_floor_+.06)ascent_started_=true;
  if(state_==State::OBSERVE_STAIR && ascent_started_ && !follow_owner_ && current && freshMap()) {
   try {checkLanding(*current);}catch(const SurfaceSearchTimeout &){stable_since_=-1;}
  }
  collectSearch();
  if(state_==State::COMPLETE) {return;}
  if(rpc_pending_ && std::chrono::steady_clock::now()>rpc_deadline_) {
   if(state_==State::STOPPING_FRONTIER || state_==State::STARTING_NEXT_FLOOR || state_==State::SESSION_WAIT) {
    ++epoch_;rpc_pending_=false;return;
   }
   if(state_==State::PREPARING_STAIR_GAIT || state_==State::RESTORING_FLAT_GAIT) {
    ++epoch_;rpc_pending_=false;
    gait_retry_=std::chrono::steady_clock::now()+std::chrono::seconds(1);
    RCLCPP_WARN(get_logger(),"Gait response timeout; holding motion until the desired gait is acknowledged");
    return;
   }
   fail("service/action response timeout");return;
  }
  if(state_==State::SESSION_WAIT) {
   if(session_.phase==Session::CLOSING)stopFrontier();
   if(restart_floor_when_closed_ && !follow_owner_ && current && freshMap() &&
     std::abs(current->feet.z-active_floor_)<.15 &&
     participantCurrent(explorer_,Session::CLOSED) && explorer_.value.idle &&
     participantCurrent(slice_,Session::CLOSED)) {
    restart_floor_when_closed_=false;
    openFloorSession();
   }
   return;
  }
  if(state_==State::WAIT_INITIAL_MAP) {
   if(!current || !freshMap() ||
     (exploration_enabled_ && (!available(explorer_) || !available(slice_))))return;
   if(resume_ && resume_phase_!=1 && resume_phase_!=3 && resume_phase_!=8) {
    waitForSession("checkpoint has no verified flat-floor recovery state");return;
   }
   auto c=config(*current,false);
   if(!SurfaceGraph(*map_,c).snapToSurface(current->feet))return;
   if(resume_) {
    if(std::abs(current->feet.z-resume_z_)>=.15)return;
    active_floor_=resume_z_;
    if(visited_.empty())visited_.push_back(active_floor_);
    resume_=false;
   }else {
    active_floor_=current->feet.z;
    visited_.push_back(active_floor_);
   }
   if(exploration_enabled_)openFloorSession();
   else {publishFloor();state_=State::EXPLORE_FLOOR;}
  }
  if(!exploration_enabled_)return;
  if(state_==State::EXPLORE_FLOOR) {
   if(!participantCurrent(explorer_,Session::ACTIVE) || !participantCurrent(slice_,Session::ACTIVE)) {
    setPhase(Session::CLOSING);
    waitForSession("Explorer or slice producer lost; new processes cannot adopt a live task");
    return;
   }
   if(pending_handoff_ && paired_floor_map_ &&
     !maps::mapEvidenceCurrent(pending_handoff_->evidence,*paired_floor_map_,liveAuditTree())) {
    RCLCPP_INFO(get_logger(),"Pending floor completion invalidated: relevant map evidence changed");invalidateHandoff();
   }
   if(completion_pending_ && handoffReady() && (lastFloor() || (current && pre_handoff_pose_ &&
     std::hypot(current->feet.x-pre_handoff_pose_->x,current->feet.y-pre_handoff_pose_->y)<.2))) {
    closing_started_at_=sim();
    setPhase(Session::CLOSING);
    state_=State::STOPPING_FRONTIER;
    stopFrontier();
    return;
   }
   if(current && sim()>=next_scan_) {launchSearch(*current,pending_handoff_.has_value());next_scan_=sim()+search_interval_;}
  } else if(state_==State::STOPPING_FRONTIER) stopFrontier();
  else if(state_==State::SEARCH_TRANSITION || state_==State::OBSERVE_STAIR) {
   if(!checkBeforeMotion() || follow_owner_)return;
   if(!current || !freshMap()) {return;}
   if(ascent_started_) {
    try {checkLanding(*current);}catch(const SurfaceSearchTimeout &) {stable_since_=-1;}
    if(state_==State::RESTORING_FLAT_GAIT || state_==State::VERIFY_LANDING) {return;}
    if(sim()-observe_since_<observe_wait_ || map_stamp_<=last_segment_map_) {return;}
   }
   if(sim()>=next_search_) launchSearch(*current,true);
  } else if(state_==State::PREPARING_STAIR_GAIT || state_==State::RESTORING_FLAT_GAIT) {
   if(!rpc_pending_ && std::chrono::steady_clock::now()>=gait_retry_) requestGait();
  } else if(state_==State::VERIFY_LANDING) {
   if(current) {try {checkLanding(*current);}catch(const SurfaceSearchTimeout &){stable_since_=-1;state_=State::OBSERVE_STAIR;}}
  } else if(state_==State::FOLLOW_3D_PATH || state_==State::APPROACH_STAIR) {
   if(!current || !freshMap()) {fail("stale map or stance while executing a segment");return;}
   if(!progress_pose_ || std::hypot(current->feet.x-progress_pose_->x,current->feet.y-progress_pose_->y)>.08 ||
    std::abs(current->feet.z-progress_pose_->z)>.04) {progress_pose_=current->feet;last_progress_=sim();}
   if(sim()-motion_start_>motion_timeout_ || sim()-last_progress_>progress_timeout_) fail("segment made no measured progress");
  } else if(state_==State::STARTING_NEXT_FLOOR) {
   if(landing_ready_) {
    if(!current || !freshMap() || std::abs(current->feet.z-active_floor_)>=.15 || follow_owner_ ||
      !participantCurrent(explorer_,Session::CLOSED) || !explorer_.value.idle ||
      !participantCurrent(slice_,Session::CLOSED))return;
    landing_ready_=false;
    openFloorSession();
    return;
   }
   if(sim()-switch_time_<settle_ || !current || std::abs(current->feet.z-active_floor_)>=.15 || !pairedInputsReady() ||
     (require_startup_ready_ && !startup_ready_))return;
   if(session_.phase==Session::PREPARING && participantCurrent(explorer_,Session::PREPARING) &&
     explorer_.value.idle && participantCurrent(slice_,Session::PREPARING))setPhase(Session::ACTIVE);
   if(session_.phase==Session::ACTIVE && participantCurrent(explorer_,Session::ACTIVE) &&
     participantCurrent(slice_,Session::ACTIVE))startFrontier();
  }
 }
 std::string map_frame_,body_frame_,feet_frame_,floor_planner_,stair_planner_,controller_,stair_checker_,entry_checker_,gait_node_;
 bool gait_required_{true},resume_{false},stair_mode_{false},ascent_started_{false},completion_pending_{false},rpc_pending_{false};
 int resume_phase_{1},expected_floors_{0},segments_{0},max_observations_{4},observation_attempts_{0};
 double active_floor_{0},resume_z_{0},separation_{1},visited_tolerance_{.45},settle_{3},observe_wait_{2},search_interval_{5},search_budget_{3};
 double flat_swing_{.04},flat_stance_{.25},stair_swing_{.14},stair_stance_{.36},rpc_timeout_{10},motion_timeout_{90},progress_timeout_{25},max_map_age_{3},suppression_time_{60};
 double closing_started_at_{-1.};
 double next_scan_{0},next_search_{0},motion_start_{0},last_progress_{0},observe_since_{0},stable_since_{-1},switch_time_{0};
 bool desired_gait_{false};std::function<void(bool)> gait_callback_;
 std::chrono::steady_clock::time_point gait_retry_{};
 uint64_t epoch_{0};State state_{State::WAIT_INITIAL_MAP};Motion motion_{Motion::APPROACH};
 SurfaceSearchConfig cfg_;StairOptions options_;SurfacePoint motion_target_,stable_pose_;
 std::vector<std::pair<SurfacePoint,double>> suppressed_;
 std::vector<double> visited_;std::vector<SurfacePoint> observation_points_;std::vector<StairCandidate> entries_;
 std::optional<SurfacePoint> progress_pose_;std::optional<StairCandidate> selected_;std::future<SearchResult> search_;
 std::shared_ptr<const octomap::OcTree> map_;rclcpp::Time map_stamp_{0,0,RCL_ROS_TIME},last_segment_map_{0,0,RCL_ROS_TIME};
 std::chrono::steady_clock::time_point rpc_deadline_;
 std::shared_ptr<tf2_ros::Buffer> tf_;std::shared_ptr<tf2_ros::TransformListener> listener_;
 std::shared_ptr<rclcpp::AsyncParametersClient> gait_;
 rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
 rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr completion_sub_;
 rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exhausted_sub_;
 rclcpp::Subscription<Completion>::SharedPtr verified_sub_;
 Session session_;
 SeenParticipant explorer_,slice_;
 std::optional<Handoff> handoff_;
 std::optional<SurfacePoint> pre_handoff_pose_;
 bool exploration_enabled_{true},require_startup_ready_{false},startup_ready_{false},restart_floor_when_closed_{false},landing_ready_{false};
 std::unordered_set<std::string> handled_reports_;
 uint64_t report_sequence_{0},context_epoch_{0},follow_owner_{0};
 rclcpp::Publisher<Session>::SharedPtr session_pub_;
 rclcpp::Subscription<Session>::SharedPtr startup_sub_;
 rclcpp::Subscription<Participant>::SharedPtr participant_sub_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr deferred_;
 bool has_residuals_{false};std::string residual_report_;
 SimulationProgress simulation_progress_;
 std::atomic<bool> clock_rewound_{false};
 rclcpp::JumpHandler::SharedPtr clock_jump_;
 std::optional<PendingHandoff> pending_handoff_;
 uint64_t handoff_request_id_{0},map_source_digest_{0};
 std::shared_ptr<const maps::PairedFloorMap> paired_floor_map_;
 rclcpp::Subscription<maps::msg::FloorMapSnapshot>::SharedPtr paired_floor_sub_;
 nav_msgs::msg::OccupancyGrid::ConstSharedPtr slice_map_;
 rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr transition_mask_pub_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr checkpoint_;
 rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr floor_pub_;
 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
 rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr regions_pub_;
 rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr done_;
 rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_;
 rclcpp::Client<Control>::SharedPtr control_;
 rclcpp_action::Client<Compute> ::SharedPtr compute_;
 rclcpp_action::Client<Follow>::SharedPtr follow_;
 rclcpp_action::ClientGoalHandle<Compute>::SharedPtr compute_handle_;
 rclcpp_action::ClientGoalHandle<Follow>::SharedPtr follow_handle_;
 rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace go2_multifloor_exploration
int main(int argc,char ** argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<go2_multifloor_exploration::MultifloorManager>());rclcpp::shutdown();return 0;}
