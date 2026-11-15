#include "frontier_exploration_ros2/frontier_explorer_node.hpp"
#include "frontier_exploration_ros2/connected_dispatch.hpp"
#include <action_msgs/msg/goal_status.hpp>
namespace frontier_exploration_ros2 {
namespace {
bool spatialEvidenceCurrent(const AuditMapEvidence & e,const PairedFloorMap & pair,const octomap::OcTree * live) {
 if(!e.present() || e.schema!=floorSchemaDigest(*pair.floor) || e.version.octomap_generation>pair.version.octomap_generation ||
    e.version.floor_map_generation>pair.version.floor_map_generation)return false;
 for(const auto & d:e.dependencies)if(!dependencyUnchanged(d,e,pair,live))return false;
 return true;
}
double residualYaw(const geometry_msgs::msg::Quaternion & q){return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));}
}
void FrontierExplorerNode::finishResidualAttempt(SpatialOutcome outcome) {
 const double now=get_clock()->now().seconds();auto region=active_residual_.regions.find(residual_region_);
 if(region!=active_residual_.regions.end() && residual_option_) {
  residual_option_->outcome=outcome;residual_option_->verified_at=now;
  // Keep compact coverage identity/results for history; full per-cell coverage
  // is needed only by the bounded set of unexecuted positive options.
  residual_option_->visible_cells.clear();residual_option_->visible_cells.shrink_to_fit();
  region->second.attempts[residual_option_->cell]=*residual_option_;
  region->second.spent+=std::max(0.,now-residual_phase_started_);++region->second.executions;
  RCLCPP_INFO(get_logger(),"Active residual region=%lu view=(%.2f,%.2f): %s; U->F=%zu U->O=%zu projected_union=%.4fm2 volume=%.5fm3",
   residual_region_,residual_option_->pose.x,residual_option_->pose.y,spatialOutcomeName(outcome),
   residual_option_->free_voxels,residual_option_->occupied_voxels,residual_option_->union_area,residual_option_->volume);
 }
 ++residual_token_;residual_phase_=ResidualPhase::Idle;residual_option_.reset();residual_unknown_window_={};
 active_observation_window_.clear();residual_cloud_stamp_=-1;residual_compute_handle_.reset();residual_dispatch_=-1;
}
void FrontierExplorerNode::activeResidualNavigationResult(int dispatch,int status) {
 if(dispatch!=residual_dispatch_ || residual_phase_!=ResidualPhase::Navigating || !residual_option_)return;
 if(status!=action_msgs::msg::GoalStatus::STATUS_SUCCEEDED){finishResidualAttempt(SpatialOutcome::NavigationFailed);return;}
 const auto actual=getCurrentPose();
 if(!actual || std::hypot(actual->position.x-residual_option_->pose.x,actual->position.y-residual_option_->pose.y)>surface_goal_config_.goal_tolerance ||
    !measuredClearanceRestored(true)){finishResidualAttempt(SpatialOutcome::LocalizationInvalid);return;}
 residual_option_->arrived=true;residual_option_->pose.x=actual->position.x;residual_option_->pose.y=actual->position.y;
 residual_option_->pose.yaw=residualYaw(actual->orientation);
 residual_arrived_at_=get_clock()->now().seconds();residual_phase_=ResidualPhase::AwaitBaseline;
 // Do not retain a full source tree while waiting for observation.
 {std::lock_guard<std::mutex> lock(surface_map_mutex_);residual_arrival_generation_=paired_floor_map_?paired_floor_map_->version.octomap_generation:0;}
}
void FrontierExplorerNode::beginResidualNav2Check(const SpatialAttempt & option,uint64_t region,const AuditMapEvidence & evidence) {
 residual_audit_stability_.reset();
 residual_option_=option;residual_region_=region;residual_option_evidence_=evidence;
 residual_phase_=ResidualPhase::CheckingPath;residual_phase_started_=get_clock()->now().seconds();const auto token=++residual_token_;
 if(!residual_compute_client_->action_server_is_ready()){finishResidualAttempt(SpatialOutcome::NavigationFailed);return;}
 ResidualCompute::Goal goal;goal.goal.header.frame_id=params_.global_frame;goal.goal.header.stamp=get_clock()->now();
 goal.goal.pose.position.x=option.pose.x;goal.goal.pose.position.y=option.pose.y;goal.goal.pose.position.z=option.pose.z;
 goal.goal.pose.orientation.z=std::sin(option.pose.yaw*.5);goal.goal.pose.orientation.w=std::cos(option.pose.yaw*.5);
 // Same flat-floor plugin selected by config/navigate_go2_recovery.xml.
 // The production server has multiple plugins and cannot infer an empty ID.
 goal.planner_id="Floor3D";goal.use_start=false;
 rclcpp_action::Client<ResidualCompute>::SendGoalOptions callbacks;
 callbacks.goal_response_callback=[this,token](auto handle) {
  if(token!=residual_token_ || runtime_state_!=RuntimeState::RUNNING || random_stair_mode_) {
   if(handle){residual_compute_client_->async_cancel_goal(handle);}
   return;
  }
  residual_compute_handle_=handle;if(!handle)finishResidualAttempt(SpatialOutcome::NavigationFailed);
 };
 callbacks.result_callback=[this,token,goal](const auto & result) {
  if(token!=residual_token_ || runtime_state_!=RuntimeState::RUNNING || random_stair_mode_)return;
  if(!residual_option_ || result.code!=rclcpp_action::ResultCode::SUCCEEDED || !result.result || result.result->path.poses.empty()) {
   finishResidualAttempt(SpatialOutcome::NavigationFailed);return;
  }
  std::shared_ptr<const PairedFloorMap> pair;std::shared_ptr<const octomap::OcTree> live;
  {std::lock_guard<std::mutex> lock(surface_map_mutex_);pair=paired_floor_map_;live=surface_map_;}
  const auto actual=getCurrentPose();
  if(!pair || !actual || !spatialEvidenceCurrent(residual_option_evidence_,*pair,live.get()) ||
     std::hypot(actual->position.x-residual_option_evidence_.start_x,actual->position.y-residual_option_evidence_.start_y)>.3) {
   finishResidualAttempt(SpatialOutcome::StaleMap);return;
  }
  OccupancyGrid2d grid(*pair->floor);const auto & path=result.result->path;
  const auto & end=path.poses.back().pose.position;
  if(path.header.frame_id!=params_.global_frame || std::hypot(end.x-goal.goal.pose.position.x,end.y-goal.goal.pose.position.y)>surface_goal_config_.goal_tolerance) {
   finishResidualAttempt(SpatialOutcome::NavigationFailed);return;
  }
  for(const auto & p:path.poses) {
   int x,y;if(!grid.worldToMapNoThrow(p.pose.position.x,p.pose.position.y,x,y) || grid.getCost(x,y)!=0 ||
      !is_inside_exploration_boundary({p.pose.position.x,p.pose.position.y},params_.exploration_boundary_polygon,0.)) {
    finishResidualAttempt(SpatialOutcome::NavigationFailed);return;
   }
  }
  if(!core_->map || !core_->costmap){finishResidualAttempt(SpatialOutcome::NavigationFailed);return;}
  ConnectedDispatch connected(*core_->costmap,*core_->map,core_->local_costmap,*actual,params_.global_frame,
   0.,params_.occ_threshold,params_.exploration_boundary_polygon,params_.exploration_boundary_margin_m);
  if(!connected.failure.empty() || !connected.choose({goal.goal.pose.position.x,goal.goal.pose.position.y},1e-6,1e-6)) {
   finishResidualAttempt(SpatialOutcome::NavigationFailed);return;
  }
  residual_option_->nav_checked=true;residual_phase_=ResidualPhase::Navigating;residual_dispatch_=core_->current_dispatch_id+1;
  if(!core_->send_pose_goal(goal.goal,"residual_observation",std::nullopt,{},"Sending lidar spatial residual observation") &&
     residual_phase_==ResidualPhase::Navigating)finishResidualAttempt(SpatialOutcome::NavigationFailed);
 };
 try{residual_compute_client_->async_send_goal(goal,callbacks);}catch(const std::exception &){finishResidualAttempt(SpatialOutcome::NavigationFailed);}
}
void FrontierExplorerNode::serviceActiveResidual() {
 const double now=get_clock()->now().seconds();
 if(active_clock_>now || active_residual_epoch_!=random_epoch_) {
  if(core_ && core_->goal_in_progress && core_->active_goal_kind=="residual_observation")core_->request_active_goal_cancel("Residual floor/map context changed");
  if(residual_phase_!=ResidualPhase::Idle)finishResidualAttempt(SpatialOutcome::StaleMap);
  latest_lidar_stamp_=-1;active_residual_.clear();active_layout_digest_=0;active_sync_octomap_=active_sync_floor_=active_sync_layout_=0;residual_audit_stability_.reset();
 }
 active_clock_=now;active_residual_epoch_=random_epoch_;
 if(!core_ || !floor_cleanup_mode_ || runtime_state_!=RuntimeState::RUNNING || random_stair_mode_)return;
 // Timeout accounting must not depend on fresh map/TF availability; otherwise
 // a stopped sensor could leave ownership permanently busy before its guard.
 if(residual_phase_==ResidualPhase::CheckingPath && now-residual_phase_started_>=residual_audit_budget_) {
  if(residual_compute_handle_)residual_compute_client_->async_cancel_goal(residual_compute_handle_);
  finishResidualAttempt(SpatialOutcome::Timeout);return;
 }
 if((residual_phase_==ResidualPhase::AwaitBaseline || residual_phase_==ResidualPhase::Observing) &&
    now-residual_phase_started_>=params_.floor_cleanup_budget_s){finishResidualAttempt(SpatialOutcome::Timeout);return;}

 std::shared_ptr<const PairedFloorMap> pair;std::shared_ptr<const octomap::OcTree> live;
 nav_msgs::msg::OccupancyGrid::ConstSharedPtr hint,mask;
 {std::lock_guard<std::mutex> lock(surface_map_mutex_);pair=paired_floor_map_;live=surface_map_;hint=surface_hint_map_;mask=transition_mask_;}
 if(!pair || !hint || floorDigest(*hint)!=pair->version.floor_digest ||
    now-rclcpp::Time(pair->floor->header.stamp).seconds()>1.5 || now-rclcpp::Time(pair->floor->header.stamp).seconds()<-.1) {
  if(active_plan_future_.valid() && active_plan_future_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
   try{auto dropped=active_plan_future_.get();auto region=active_residual_.regions.find(dropped.region.region.id);if(region!=active_residual_.regions.end())region->second.spent+=dropped.elapsed;}
   catch(const std::exception &){}
  }
  RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"Active residual waiting for fresh source-paired map; uncertainty retained");return;
 }
 const auto pose=getCurrentPose();if(!pose){
  active_observation_window_.pause(now);
  if(residual_phase_!=ResidualPhase::Idle && now-residual_phase_started_>=params_.floor_cleanup_budget_s)finishResidualAttempt(SpatialOutcome::LocalizationInvalid);
  return;
 }
 if(residual_phase_==ResidualPhase::CheckingPath) {
  if(now-residual_phase_started_>=residual_audit_budget_) {
   if(residual_compute_handle_)residual_compute_client_->async_cancel_goal(residual_compute_handle_);
   finishResidualAttempt(SpatialOutcome::Timeout);
  }return;
 }
 if(residual_phase_==ResidualPhase::Navigating)return;
 if(residual_phase_==ResidualPhase::AwaitBaseline || residual_phase_==ResidualPhase::Observing) {
  if(floorSchemaDigest(*pair->floor)!=residual_option_evidence_.schema ||
     (residual_phase_==ResidualPhase::Observing && std::abs(residual_unknown_window_.resolution-pair->tree->getResolution())>1e-9)) {
   finishResidualAttempt(SpatialOutcome::StaleMap);return;
  }
  if(!residual_option_){finishResidualAttempt(SpatialOutcome::LocalizationInvalid);return;}
  if(now-residual_phase_started_>=params_.floor_cleanup_budget_s){finishResidualAttempt(SpatialOutcome::Timeout);return;}
  if(std::hypot(pose->position.x-residual_option_->pose.x,pose->position.y-residual_option_->pose.y)>.15) {
   active_observation_window_.pause(now);return;
  }
  auto region=active_residual_.regions.find(residual_region_);if(region==active_residual_.regions.end()){finishResidualAttempt(SpatialOutcome::StaleMap);return;}
  if(residual_phase_==ResidualPhase::AwaitBaseline) {
   if(params_.post_goal_settle_enabled && now-residual_arrived_at_<params_.post_goal_min_settle)return;
   if(pair->source_stamp<residual_arrived_at_ || pair->version.octomap_generation<=residual_arrival_generation_)return;
   auto deadline=std::chrono::steady_clock::now()+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(residual_audit_budget_));
   residual_unknown_window_=captureUnknownVoxels(*pair,region->second.region,surface_goal_config_.support_probe_depth,
     std::max(surface_goal_config_.body_height,.8),deadline,params_.geometry_max_points);
   active_observation_window_.clear();active_observation_window_.boundary=params_.exploration_boundary_polygon;
   active_observation_window_.boundary_margin=params_.exploration_boundary_margin_m;
   active_observation_window_.no_gain_wait_s=params_.region_no_gain_wait_s;
   active_observation_window_.no_gain_budget_s=params_.region_no_gain_budget_s;
   OccupancyGrid2d observation_grid(*pair->floor);std::optional<std::pair<double,double>> reference;
   for(int cell:region->second.region.cells)if(observation_grid.getCost(cell%pair->floor->info.width,cell/pair->floor->info.width)<0) {
    reference=observation_grid.mapToWorld(cell%pair->floor->info.width,cell/pair->floor->info.width);break;
   }
   if(!reference || !active_observation_window_.start(observation_grid,*reference,now)) {
    finishResidualAttempt(SpatialOutcome::StaleMap);return;
   }
   active_observation_window_.arrived(OccupancyGrid2d(*pair->floor),{pose->position.x,pose->position.y},now,residual_option_->pose.yaw);
   residual_arrived_at_=now;residual_arrival_generation_=pair->version.octomap_generation;residual_cloud_stamp_=-1;
   residual_phase_=ResidualPhase::Observing;return;
  }
  if(residual_cloud_stamp_<0 && latest_lidar_stamp_>residual_arrived_at_)residual_cloud_stamp_=latest_lidar_stamp_;
  if(residual_cloud_stamp_<0 || pair->source_stamp<residual_cloud_stamp_ || pair->version.octomap_generation<=residual_arrival_generation_) {
   active_observation_window_.pause(now);
   RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),5000,"Active residual waiting for fresh post-arrival lidar and source-matched map");return;
  }
  residual_option_->fresh_cloud=true;
  const auto gain=measureVoxelGain(residual_unknown_window_,*pair->tree);
  auto decision=active_observation_window_.poll(OccupancyGrid2d(*pair->floor),{pose->position.x,pose->position.y},now,gain.gained(),false);
  if(decision==RegionObservationTask::Decision::Waiting)return;
  residual_option_->free_voxels=gain.free_voxels;residual_option_->occupied_voxels=gain.occupied_voxels;
  residual_option_->free_area=gain.free_area;residual_option_->occupied_area=gain.occupied_area;
  residual_option_->union_area=gain.union_area;residual_option_->volume=gain.volume;
  finishResidualAttempt(gain.gained()?SpatialOutcome::ObservedGain:residual_unknown_window_.complete?SpatialOutcome::ObservedNoGain:SpatialOutcome::Timeout);return;
 }
 if(core_->goal_in_progress)return;
 const bool valid_mask=mask && mask->info.width==pair->floor->info.width && mask->info.height==pair->floor->info.height &&
  floorSchemaDigest(*mask)==floorSchemaDigest(*pair->floor) && now-rclcpp::Time(mask->header.stamp).seconds()>=-.1 && now-rclcpp::Time(mask->header.stamp).seconds()<=10.;
 uint64_t mask_hash=valid_mask?floorDigest(*mask):0;
 auto layout_key=pair->version.floor_digest;mapHashMix(layout_key,mask_hash);
 int robot_x=0,robot_y=0;if(OccupancyGrid2d(*pair->floor).worldToMapNoThrow(pose->position.x,pose->position.y,robot_x,robot_y))mapHashMix(layout_key,robot_y*pair->floor->info.width+robot_x);
 if(active_layout_digest_!=layout_key) {
  active_layout_=extractResidualLayout(OccupancyGrid2d(*pair->floor),{pose->position.x,pose->position.y},params_.exploration_boundary_polygon,
    params_.exploration_boundary_margin_m,std::chrono::steady_clock::now()+std::chrono::seconds(5),
    [&](int x,int y){return valid_mask && mask->data[y*mask->info.width+x]==100;});
  if(!active_layout_.complete){residual_audit_stability_.reset();return;}
  active_layout_digest_=layout_key;
 }
 double sensor_offset=0;
 try{const auto mount=tf_buffer_->lookupTransform("base_footprint",visibility_sensor_frame_,tf2::TimePointZero);sensor_offset=std::hypot(mount.transform.translation.x,mount.transform.translation.y);}
 catch(const tf2::TransformException &){return;}
 const double padding=sensor_offset+surface_goal_config_.snap_radius+surface_goal_config_.collision_radius+std::abs(surface_goal_config_.cylinder_offset)+pair->floor->info.resolution;
 if(active_sync_octomap_!=pair->version.octomap_generation || active_sync_floor_!=pair->version.floor_map_generation || active_sync_layout_!=layout_key) {
  const auto revision=active_residual_.revision;
  active_residual_.sync(active_layout_,*pair,params_.exploration_boundary_polygon,params_.exploration_boundary_margin_m,padding);
  if(active_residual_.revision!=revision)residual_audit_stability_.reset();
  active_sync_octomap_=pair->version.octomap_generation;active_sync_floor_=pair->version.floor_map_generation;active_sync_layout_=layout_key;
 }
 if(active_plan_future_.valid()) {
  if(active_plan_future_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
  SpatialPlan result;
  try{result=active_plan_future_.get();}catch(const std::exception & e){RCLCPP_ERROR(get_logger(),"Active residual worker failed: %s",e.what());return;}
  if(result.epoch!=random_epoch_ || !spatialEvidenceCurrent(result.evidence,*pair,live.get()) ||
     std::hypot(pose->position.x-result.evidence.start_x,pose->position.y-result.evidence.start_y)>.3)return;
  auto id=result.region.region.id;const auto evidence=result.evidence;const auto selected=result.selected;
  bool timed=!result.complete_input;for(const auto & a:result.region.attempts)timed|=a.second.outcome==SpatialOutcome::Timeout;
  residual_audit_budget_=timed?std::min(60.,residual_audit_budget_*2):5.;
  if(!active_residual_.apply(std::move(result)))return;
  if(selected)beginResidualNav2Check(*selected,id,evidence);
  // Bind navigation to actual dependencies, not a bare generation-only record.
  if(residual_phase_!=ResidualPhase::Idle)return;
 }
 active_residual_.recoverBudgets(now,params_.floor_cleanup_budget_s,active_observation_window_.cooldown_s);
 active_residual_.changedOrigin(pose->position.x,pose->position.y,params_.floor_cleanup_budget_s);
 if(auto id=active_residual_.next(params_.floor_cleanup_budget_s)) {
  residual_audit_stability_.reset();
  auto config=surface_goal_config_;VisibilitySensor sensor;
  try {
   auto feet=tf_buffer_->lookupTransform(params_.global_frame,"base_footprint",tf2::TimePointZero);
   if(now-rclcpp::Time(feet.header.stamp).seconds()>.5)return;
   config.support_reference_height=pose->position.z-feet.transform.translation.z;
   auto mount=tf_buffer_->lookupTransform("base_footprint",visibility_sensor_frame_,tf2::TimePointZero);
   sensor.x=mount.transform.translation.x;sensor.y=mount.transform.translation.y;sensor.z=mount.transform.translation.z;
   sensor.qx=mount.transform.rotation.x;sensor.qy=mount.transform.rotation.y;sensor.qz=mount.transform.rotation.z;sensor.qw=mount.transform.rotation.w;
   sensor.min_pitch=visibility_min_pitch_*M_PI/180.;sensor.max_pitch=visibility_max_pitch_*M_PI/180.;
  }catch(const tf2::TransformException &){return;}
  const auto region=active_residual_.regions.at(*id);const auto epoch=random_epoch_;const double budget=residual_audit_budget_;
  const ResidualPose body{pose->position.x,pose->position.y,pose->position.z,residualYaw(pose->orientation)};
  const auto boundary=params_.exploration_boundary_polygon;const double margin=params_.exploration_boundary_margin_m;
  const unsigned batch=params_.floor_cleanup_min_attempts;
  active_plan_future_=std::async(std::launch::async,[pair,region,body,config,sensor,boundary,margin,budget,batch,epoch] {
   return planSpatialRegion(*pair,region,body,config,sensor,boundary,margin,budget,batch,batch,epoch);
  });return;
 }
 auto result=active_residual_.audit(params_.floor_cleanup_budget_s,{pose->position.x,pose->position.y});
 sealResidualAudit(result,*pair,params_.exploration_boundary_polygon,params_.exploration_boundary_margin_m,padding);
 result.transition_hash=mask_hash;result.epoch=random_epoch_;
 considerResidualCompletion(result,*pair,live.get(),now);
 RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),10000,"Active residual assessment %s: %s",completionStatusName(result.status()),result.records().c_str());
}
} // namespace frontier_exploration_ros2
