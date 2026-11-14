#include "frontier_exploration_ros2/viewpoint_sampling.hpp"
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

#include "frontier_exploration_ros2/frontier_explorer_core.hpp"

#include "frontier_exploration_ros2/connected_dispatch.hpp"

#include "frontier_explorer_core_detail.hpp"

#include <action_msgs/msg/goal_status.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace frontier_exploration_ros2
{

void FrontierExplorerCore::report_frontier_validation(std::pair<double,double> reference,bool verified,
  bool completed,unsigned tested_options,unsigned available_options)
{
  if(exploration_enabled && !verified &&
     (!callbacks.floor_cleanup_data_ready || callbacks.floor_cleanup_data_ready()))
    floor_coverage_.validationEvidence(completed,tested_options,available_options);
  if(!verified && exploration_enabled && params.region_observation_tasks_enabled &&
    map && region_task_.matches(*map,reference) &&
    (!callbacks.floor_cleanup_data_ready || callbacks.floor_cleanup_data_ready()))
  {
    const auto actual=callbacks.get_current_pose();
    // Only a fully tested candidate fan contributes an origin to
    // execution-budget evidence. A timeout by itself cannot defer a region.
    const bool exhausted=completed && available_options>0 && tested_options>=available_options;
    region_task_.planningFailure(actual && exhausted?
      std::make_optional(std::make_pair(actual->position.x,actual->position.y)):std::nullopt);
  }
}

void FrontierExplorerCore::try_send_next_goal()
{
  // Main scheduler entrypoint: exits early unless maps, pose, and lifecycle gates are ready.
  if (!exploration_enabled) {
    no_next_goal_.pause(callbacks.now_ns()*1e-9);
    return;
  }

  if (return_to_start_completed) {
    // Exploration is fully finished.
    return;
  }

  if (goal_in_progress || !map.has_value() || !costmap.has_value()) {
    no_next_goal_.pause(callbacks.now_ns()*1e-9);
    floor_coverage_.pauseResidual(callbacks.now_ns()*1e-9);
    region_task_.pause(callbacks.now_ns()*1e-9);
    symmetric_relocation_.pause(callbacks.now_ns()*1e-9);
    symmetric_wait_sample_=callbacks.now_ns()*1e-9;
    // Cannot dispatch a new goal while one is active, or before required maps arrive.
    return;
  }

  commit_deferred_costmap_search_input_updates();

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    no_next_goal_.pause(callbacks.now_ns()*1e-9);
    symmetric_relocation_.pause(callbacks.now_ns()*1e-9);
    symmetric_wait_sample_=callbacks.now_ns()*1e-9;
    // Invalid TF pauses observation time even for a short outage.
    region_task_.pause(callbacks.now_ns()*1e-9);
    return;
  }

  if((params.floor_cleanup_enabled || params.region_observation_tasks_enabled) && callbacks.floor_cleanup_data_ready &&
    !callbacks.floor_cleanup_data_ready()) {
    symmetric_relocation_.pause(callbacks.now_ns()*1e-9);
    symmetric_wait_sample_=callbacks.now_ns()*1e-9;
    no_next_goal_.pause(callbacks.now_ns()*1e-9);
    floor_coverage_.waitForData(callbacks.now_ns()*1e-9);
    region_task_.pause(callbacks.now_ns()*1e-9);
    callbacks.log_debug("Floor evaluation deferred: map or sensor transform is not fresh/available");
    return;
  }
  // This must precede frontier connectivity and floor completion. A mission
  // boundary is a goal constraint, not an occupied wall that prevents reentry.
  if(!is_inside_exploration_boundary(
      {current_pose->position.x,current_pose->position.y},params.exploration_boundary_polygon,0.)) {
    no_next_goal_.pause(callbacks.now_ns()*1e-9);
    floor_coverage_.waitForData(callbacks.now_ns()*1e-9);
    region_task_.pause(callbacks.now_ns()*1e-9);
    try_random_recovery(*current_pose,true);
    return;
  }
  if (awaiting_map_refresh && !post_goal_settle_ready()) {
    no_next_goal_.pause(callbacks.now_ns()*1e-9);
    // Settle window is still active after previous goal completion.
    return;
  }

  // Contact is an immediate recovery condition, independent of idle planning.
  if(callbacks.take_clearance_recovery_request && callbacks.take_clearance_recovery_request()) {
    geometry_msgs::msg::PoseStamped measured;
    measured.header.frame_id=params.global_frame;measured.pose=*current_pose;
    if(send_pose_goal(measured,"clearance_recovery",std::nullopt,{},
      "Starting immediate supported small-step contact recovery"))return;
  }

  // Evaluate independently of all candidate-selection and validation branches.
  // A map update or a different frontier must not postpone this deadline.
  const double no_goal_now=callbacks.now_ns()*1e-9;
  no_next_goal_.timeout=params.no_next_goal_timeout_s;
  no_next_goal_.observe(no_goal_now,map->map().info.origin.position.z);
  no_next_goal_.position(no_goal_now,current_pose->position.x,current_pose->position.y);
  if(no_next_goal_.due(no_goal_now) &&
     (!params.floor_cleanup_enabled || floor_coverage_.hasActionableUnknown())) {
    no_next_goal_.attempted(no_goal_now);
    callbacks.log_warn("No executable next goal for "+std::to_string(no_next_goal_.seconds())+
      " simulated seconds; trigger recovery, historical open-area return first");
    if(callbacks.on_no_next_goal_timeout)callbacks.on_no_next_goal_timeout();
    if(try_random_recovery(*current_pose,true))return;
  }

  if(params.region_observation_tasks_enabled) {
    region_task_.boundary=params.exploration_boundary_polygon;
    region_task_.boundary_margin=params.exploration_boundary_margin_m;
    region_task_.no_gain_wait_s=params.region_no_gain_wait_s;
    if(floor_coverage_.cleanup())region_task_.no_gain_wait_s=std::max(
      region_task_.no_gain_wait_s,params.region_no_gain_budget_s/std::max(3u,static_cast<unsigned>(params.region_minimum_observation_positions)));
    region_task_.no_gain_budget_s=params.region_no_gain_budget_s;
    region_task_.minimum_positions=params.region_minimum_observation_positions;
    region_task_.cooldown_s=params.region_retry_cooldown_s;
    region_gain_pending_|=region_task_.update(*map,callbacks.now_ns()*1e-9);
    if(region_task_.planningWait(*map,callbacks.now_ns()*1e-9)) {
      symmetry_requested_=true;
      callbacks.log_warn("Region planning deferred without declaring complete: healthy_planning_wait_s="+
        std::to_string(region_task_.planningSeconds())+", failed_validation_rounds="+
        std::to_string(region_task_.planningFailures())+
        "; expanding selection to other open regions");
      region_gain_pending_=false;pending_observation_progress_.reset();
      pending_frontier_sequence.clear();observation_continuation_reference_.reset();
      frontier_snapshot.reset();raw_frontier_debug_cache.reset();mrtsp_order_cache.reset();
      decision_map_dirty=true;
    }
  }
  if (pending_observation_progress_) {
    pending_observation_progress_->observation={current_pose->position.x,current_pose->position.y};
    if(params.region_observation_tasks_enabled && region_task_.active()) {
      const auto decision=region_task_.poll(*map,
        {current_pose->position.x,current_pose->position.y},callbacks.now_ns()*1e-9,
        region_gain_pending_,observation_progress_.routeProgress(*pending_observation_progress_,*map));
      if(decision==RegionObservationTask::Decision::Waiting)return;
      if(decision==RegionObservationTask::Decision::ChangeView ||
         decision==RegionObservationTask::Decision::Deferred) {
        observation_progress_.blockObservation(
          {current_pose->position.x,current_pose->position.y},callbacks.now_ns());
        callbacks.log_info(std::string(decision==RegionObservationTask::Decision::Deferred ?
          "Region deferred without declaring complete" : "Changing regional observation position")+
          ": healthy_stationary_no_gain_s="+std::to_string(region_task_.idleSeconds())+
          ", distinct_reached_positions="+std::to_string(region_task_.triedPositions())+
          ", per_view_wait_s="+std::to_string(region_task_.no_gain_wait_s)+
          ", region_budget_s="+std::to_string(params.region_no_gain_budget_s));
      } else callbacks.log_info(decision==RegionObservationTask::Decision::Gain ?
        "Regional observation produced new information; retain spatial task" :
        "Measured approach shortened remaining route; retain spatial task");
      if(decision==RegionObservationTask::Decision::Deferred)symmetry_requested_=true;
      region_gain_pending_=false;
    }
    floor_coverage_.attempt(callbacks.now_ns()*1e-9);
    const bool approach_progress=observation_progress_.routeProgress(*pending_observation_progress_,*map);
    const auto outcome=observation_progress_.complete(*pending_observation_progress_,*map,callbacks.now_ns());
    if (outcome.cooling && !params.region_observation_tasks_enabled) {observation_continuation_reference_.reset();}
    callbacks.log_info("Observation information gain: new_area="+std::to_string(outcome.revealed_area)+
      " m2, no_gain_visits="+std::to_string(outcome.no_gain_visits)+
      ", cooling="+(outcome.cooling?"true":"false"));
    floor_coverage_.observationEvidence(
      {current_pose->position.x,current_pose->position.y},outcome.revealed_area>=.05 || approach_progress);
    pending_observation_progress_.reset();
  }
  if(callbacks.request_symmetric_relocation_step &&
     try_symmetric_relocation(*current_pose,symmetry_requested_))return;
  observation_progress_.refresh(*map,callbacks.now_ns());
  if (awaiting_map_refresh) {
    clear_post_goal_wait_state();
  }

  if(params.floor_cleanup_enabled) {
    const double now=callbacks.now_ns()*1e-9;
    floor_coverage_.idle_timeout=params.floor_no_gain_timeout_s;
    floor_coverage_.cleanup_budget=params.floor_cleanup_budget_s;
    floor_coverage_.minimum_attempts=params.floor_cleanup_min_attempts;
    const bool previous=floor_coverage_.cleanup();
    const unsigned previous_expansion=floor_coverage_.expansion();
    floor_coverage_.observe(now,*map,params.exploration_boundary_polygon,
      params.exploration_boundary_margin_m,
      std::make_pair(current_pose->position.x,current_pose->position.y));
    if(params.region_observation_tasks_enabled && region_task_.remainingAreClosed(
        *map,[&](int x,int y){return floor_coverage_.excludedCell(x,y);})) {
      callbacks.log_info("Local task released: all its remaining unknowns are independently verified stable closed regions");
      region_task_.release();region_gain_pending_=false;
      frontier_snapshot.reset();decision_map_dirty=true;
    }
    if(floor_coverage_.shouldCleanup(now)) {
      floor_coverage_.enterCleanup(now);
      callbacks.log_warn("No recent map gain: expand residual observation search; open unknowns prohibit floor handoff");
    }
    if(floor_coverage_.cleanup() && floor_coverage_.hasActionableUnknown())floor_coverage_.expand(now);
    if(previous!=floor_coverage_.cleanup() || previous_expansion!=floor_coverage_.expansion()) {
      frontier_snapshot.reset();raw_frontier_debug_cache.reset();mrtsp_order_cache.reset();
      decision_map_dirty=true;
    }
    if(callbacks.set_floor_cleanup_mode)callbacks.set_floor_cleanup_mode(floor_coverage_.cleanup());
    if(floor_coverage_.hasActionableUnknown())floor_handoff_requested_=false;
    const int64_t status_now=callbacks.now_ns();
    if(floor_coverage_.cleanup() && floor_coverage_.hasActionableUnknown() &&
       (last_residual_status_ns_<0 || status_now<last_residual_status_ns_ ||
        status_now-last_residual_status_ns_>=30'000'000'000LL)) {
      last_residual_status_ns_=status_now;
      callbacks.log_info("Unfinished floor search: unresolved_area="+
        std::to_string(floor_coverage_.actionable_unknown_area)+
        ", expansion="+std::to_string(floor_coverage_.expansion())+
        ", healthy_idle_s="+std::to_string(floor_coverage_.residualIdleSeconds())+
        ", exhausted_search_rounds="+std::to_string(floor_coverage_.residualRounds())+
        ", empty_rounds="+std::to_string(floor_coverage_.emptyRounds())+
        ", completed_failed_sets="+std::to_string(floor_coverage_.failedGroups())+
        ", tested_candidates="+std::to_string(floor_coverage_.failedOptions())+
        ", handoff_blocked=open_unknown_regions");
    }
    // Re-evaluate the current map even after requesting handoff: an opening
    // or new unknown component revokes completion. A latched request is not proof.
    if(floor_coverage_.shouldHandoff(now)) {
      floor_handoff_requested_=true;
      if(callbacks.on_floor_exhausted &&
        (last_floor_handoff_ns_==0 || callbacks.now_ns()-last_floor_handoff_ns_>=2'000'000'000LL)) {
        last_floor_handoff_ns_=callbacks.now_ns();
        callbacks.on_floor_exhausted("v1;floor_z="+std::to_string(map->map().info.origin.position.z)+
          ";stamp="+std::to_string(now)+";reason=closed_regions_excluded"+
          ";remaining_unknown_area="+std::to_string(floor_coverage_.remaining_unknown_area)+
          ";actionable_unknown_area="+std::to_string(floor_coverage_.actionable_unknown_area)+
          ";excluded_unknown_area="+std::to_string(floor_coverage_.excluded_unknown_area)+
          ";deferred_unknown_area=0"+
          ";attempts="+std::to_string(floor_coverage_.attempts()));
      }
      return;
    }
    // Count stable selection opportunities, not controller ticks or active motion.
    floor_coverage_.attempt(now);
  }
  record_start_pose(*current_pose);

  if (!pending_frontier_sequence.empty()) {
    // Replacement/preemption path already computed a concrete next target.
    if (dispatch_pending_frontier_goal(current_pose)) {return;}
  }

  FrontierSnapshot snapshot;
  auto widen_unknown_search=[&]() {
    if(!params.floor_cleanup_enabled || !floor_coverage_.hasActionableUnknown() ||
      !floor_coverage_.expand(callbacks.now_ns()*1e-9))return;
    frontier_snapshot.reset();raw_frontier_debug_cache.reset();mrtsp_order_cache.reset();
    decision_map_dirty=true;
    if(!params.region_observation_tasks_enabled)observation_continuation_reference_.reset();
    if(callbacks.set_floor_cleanup_mode)callbacks.set_floor_cleanup_mode(true);
    callbacks.log_warn("Open unknown regions remain: widening observation search, stage="+
      std::to_string(floor_coverage_.expansion())+
      ", actionable_area="+std::to_string(floor_coverage_.actionable_unknown_area)+
      ", closed_excluded_area="+std::to_string(floor_coverage_.excluded_unknown_area));
  };

  const double min_goal_distance = floor_coverage_.cleanup()?.25:
    frontier_snapshot_min_goal_distance_for_pose(*current_pose);
  try {
    snapshot = get_frontier_snapshot(
      *current_pose,
      min_goal_distance);
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier update: ") + exc.what());
    return;
  }

  FrontierSequence actionable_frontiers;
  for(const auto & frontier:snapshot.frontiers) {
    if(params.floor_cleanup_enabled &&
       floor_coverage_.closedReference(*map,frontier_reference_point(frontier)))continue;
    actionable_frontiers.push_back(frontier);
  }
  const FrontierSequence & frontiers = actionable_frontiers;
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(frontiers);
  if(relocation_arrival_anchor_) {
    if(callbacks.now_ns()*1e-9>=relocation_arrival_until_)relocation_arrival_anchor_.reset();
    else {
      FrontierSequence nearby;
      for(const auto & f:filtered_frontiers) {
        const auto p=frontier_reference_point(f);
        if(std::hypot(p.first-relocation_arrival_anchor_->first,
          p.second-relocation_arrival_anchor_->second)<=3.5)nearby.push_back(f);
      }
      if(!nearby.empty())filtered_frontiers=std::move(nearby);
      else relocation_arrival_anchor_.reset();
    }
  }
  if(params.region_observation_tasks_enabled && region_task_.active()) {
    // Attach freshly exposed adjoining unknowns before declaring the old
    // local footprint observed. A room larger than the initial radius must
    // not lose its task just because the original footprint became known.
    for(const auto & frontier:filtered_frontiers)
      if(region_task_.matches(*map,frontier_reference_point(frontier)))
        region_task_.extend(*map,frontier_reference_point(frontier));
    if(region_task_.finished(*map)) {
      callbacks.log_info("Local region observed: no remaining unknown cells in its tracked footprint");
      region_task_.release();region_gain_pending_=false;
      frontier_snapshot.reset();decision_map_dirty=true;
    } else {
      FrontierSequence regional;
      for(const auto & frontier:filtered_frontiers)
        if(region_task_.matches(*map,frontier_reference_point(frontier)))regional.push_back(frontier);
      // A new scan can remove/reject the former boundary while door-interior
      // unknowns remain. Regenerate visible observations for that footprint.
      if(regional.empty()) {
        for(auto reference:region_task_.remainingReferences(*map,
            {current_pose->position.x,current_pose->position.y})) {
          if(floor_coverage_.closedReference(*map,reference))continue;
          const std::pair<double,double> direction{
            current_pose->position.x-reference.first,current_pose->position.y-reference.second};
          auto points=sampleViewpointFan(reference,direction,{.35,.5,.8,1.1,1.4},25);
          auto expanded=sampleExpandedViewpoints(reference,direction,3);
          points.insert(points.end(),expanded.begin(),expanded.end());
          std::vector<std::pair<double,double>> safe;
          for(auto point:points) {
            int x,y;
            if(!map->worldToMapNoThrow(point.first,point.second,x,y) || map->getCost(x,y)!=0 ||
               !is_inside_exploration_boundary(point,params.exploration_boundary_polygon,
                 params.exploration_boundary_margin_m) ||
               observation_progress_.blockedObservation(point,callbacks.now_ns()) ||
               viewpointVisibleGain(*map,point,reference)<=0)continue;
            safe.push_back(point);
          }
          if(safe.empty())continue;
          FrontierCandidate candidate{reference,safe.front(),1};
          candidate.viewpoint_candidates=std::move(safe);
          candidate.requires_visible_viewpoint=true;
          regional.push_back(std::move(candidate));
        }
      }
      if(regional.empty()) {
        region_task_.noCandidate(callbacks.now_ns()*1e-9);
        throttled_debug("Regional task retained: open unknown footprint has no visible safe candidate; expanding/recovering");
        frontier_snapshot.reset();decision_map_dirty=true;
        try_random_recovery(*current_pose,true);
        return;
      }
      filtered_frontiers=std::move(regional);
    }
  }
  bool escape_mode_active = false;
  if (filtered_frontiers.empty() && !frontiers.empty()) {
    no_frontiers_reported = false;
    no_reachable_frontier_reported = false;
    publish_frontier_markers(filtered_frontiers);
    if (!all_frontiers_suppressed_reported) {
      callbacks.log_info("All currently detected frontiers are temporarily suppressed");
      all_frontiers_suppressed_reported = true;
    }
    widen_unknown_search();
    handle_all_frontiers_suppressed(*current_pose);
    return;
  }

  if (filtered_frontiers.empty() && frontiers.empty() && params.escape_enabled && min_goal_distance > 0.0) {
    FrontierSnapshot escape_snapshot;
    try {
      escape_snapshot = get_frontier_snapshot(*current_pose, 0.0);
    } catch (const std::out_of_range & exc) {
      callbacks.log_warn(std::string("Skipping escape-mode frontier update: ") + exc.what());
      return;
    }

    FrontierSequence escape_filtered_frontiers = filter_frontiers_for_suppression(escape_snapshot.frontiers);
    if (escape_filtered_frontiers.empty() && !escape_snapshot.frontiers.empty()) {
      no_frontiers_reported = false;
      no_reachable_frontier_reported = false;
      publish_frontier_markers(escape_filtered_frontiers);
      if (!all_frontiers_suppressed_reported) {
        callbacks.log_info("All currently detected frontiers are temporarily suppressed");
        all_frontiers_suppressed_reported = true;
      }
      widen_unknown_search();
    handle_all_frontiers_suppressed(*current_pose);
      return;
    }

    if (!escape_filtered_frontiers.empty()) {
      filtered_frontiers = std::move(escape_filtered_frontiers);
      escape_mode_active = true;
      callbacks.log_info(
        "No frontier survived the active minimum-distance filters (min_goal_distance=" +
        detail::format_meters(min_goal_distance) +
        "); retrying in escape mode with min-distance search and dispatch bypassed");
    }
  }

  all_frontiers_suppressed_reported = false;
  suppressed_return_to_start_started = false;
  if (filtered_frontiers.empty()) {
      // Check the raw occupancy map before declaring completion.
    // Cost filtering, preprocessing or cluster-size filtering can
    // remove all usable goals while unexplored boundaries remain.
    constexpr int directions[4][2] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}
    };

    bool has_unexplored_boundary = false;

    for (int y = 0;
      y < map->getSizeY() && !has_unexplored_boundary;
      ++y)
    {
      for (int x = 0;
        x < map->getSizeX() && !has_unexplored_boundary;
        ++x)
      {
        if (map->getCost(x, y) !=
          static_cast<int>(OccupancyGrid2d::CostValues::NoInformation))
        {
          continue;
        }

        if (!is_inside_exploration_boundary(
            map->mapToWorld(x, y),
            params.exploration_boundary_polygon,
            params.exploration_boundary_margin_m))
        {
          continue;
        }

        for (const auto & direction : directions) {
          const int nx = x + direction[0];
          const int ny = y + direction[1];

          if (nx < 0 || ny < 0 ||
            nx >= map->getSizeX() || ny >= map->getSizeY())
          {
            continue;
          }

          if (map->getCost(nx, ny) ==
            static_cast<int>(OccupancyGrid2d::CostValues::FreeSpace))
          {
            has_unexplored_boundary = true;
            break;
          }
        }
      }
    }

    if(params.floor_cleanup_enabled)
      has_unexplored_boundary=floor_coverage_.hasActionableUnknown();
    if (has_unexplored_boundary) {
      widen_unknown_search();
      floor_coverage_.exhaustedSearch(callbacks.now_ns()*1e-9,frontiers.empty(),
        callbacks.frontier_goal_validation_pending && callbacks.frontier_goal_validation_pending());
      if(floor_coverage_.shouldHandoff(callbacks.now_ns()*1e-9) &&
         (!callbacks.residual_handoff_pose_safe || callbacks.residual_handoff_pose_safe()))return;
      if(try_random_recovery(*current_pose,true)) {return;}
      publish_frontier_markers(filtered_frontiers);
      no_frontiers_reported = false;

      if (!no_reachable_frontier_reported) {
        callbacks.log_warn(
          "Open unknown regions remain, but no eligible goal is "
          "available; expanded observation search and verified recovery remain active");
        no_reachable_frontier_reported = true;
      }

      return;
    }
    no_reachable_frontier_reported = false;
    publish_frontier_markers(filtered_frontiers);

    if (!no_frontiers_reported) {
      // Log once per no-frontier streak to avoid repetitive output.
      callbacks.log_info("No more frontiers found");
      no_frontiers_reported = true;
    }

    handle_exploration_complete(*current_pose);
    return;
  }

  if (try_random_recovery(*current_pose,false)) {return;}

  no_frontiers_reported = false;
  // Keep a multi-step approach aimed at the same fresh frontier. Otherwise
  // whichever asynchronous validation finishes first can pull the robot back
  // across known space after every short observation goal.
  if (!params.region_observation_tasks_enabled && observation_continuation_reference_) {
    const auto reference=*observation_continuation_reference_;
    auto nearest=filtered_frontiers.end(); double best=.75;
    for (auto it=filtered_frontiers.begin();it!=filtered_frontiers.end();++it) {
      const auto point=frontier_reference_point(*it);
      const double distance=std::hypot(point.first-reference.first,point.second-reference.second);
      if (distance<best) {best=distance;nearest=it;}
    }
    if (nearest!=filtered_frontiers.end() &&
      callbacks.now_ns()-observation_continuation_started_ns_ < 15'000'000'000LL)
    {
      if (send_frontier_goal({*nearest},*current_pose,
          "Continuing verified observation approach: "+describe_frontier(*nearest))) {return;}
      // A timeout enters retry backoff, during which no worker is "pending".
      // Preserve this bounded approach window across that backoff as well.
      return;
    }
    observation_continuation_reference_.reset();
  }
  auto selection = select_frontier(filtered_frontiers, *current_pose);
  if (escape_mode_active && selection.frontier.has_value()) {
    selection.mode = "escape";
  }
  publish_frontier_markers(filtered_frontiers);

  if (!selection.frontier.has_value()) {
    widen_unknown_search();
    if(try_random_recovery(*current_pose,true)) {return;}
    if (!no_reachable_frontier_reported) {
      callbacks.log_info(
        "No reachable frontier candidate is available right now; waiting for a map update");
      no_reachable_frontier_reported = true;
    }
    return;
  }

  no_reachable_frontier_reported = false;
  const auto frontier_sequence = select_frontier_sequence(
    filtered_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    return;
  }

  const bool sent=send_frontier_goal(
    frontier_sequence,
    *current_pose,
    "Sending frontier goal (" + selection.mode + "): " + describe_frontier(frontier_sequence.front()),
    escape_mode_active);
  if(!sent && (!callbacks.frontier_goal_validation_pending ||
      !callbacks.frontier_goal_validation_pending()))widen_unknown_search();
}

void FrontierExplorerCore::reset_exploration_runtime_state(bool clear_maps)
{
  no_next_goal_.clear();
  observation_continuation_reference_.reset();
  clear_post_goal_wait_state();
  clear_active_goal_progress_state();

  no_frontiers_reported = false;
  no_reachable_frontier_reported = false;
  all_frontiers_suppressed_reported = false;
  return_to_start_started = false;
  return_to_start_completed = false;
  suppressed_return_to_start_started = false;
  cancel_request_in_progress = false;
  pending_cancel_reason.reset();
  pending_frontier_sequence.clear();
  pending_frontier_selection_mode.clear();
  pending_frontier_dispatch_context.clear();
  active_goal_blocked_reason.reset();
  pending_costmap_search_input_update = false;
  pending_local_costmap_search_input_update = false;
  distance_completed_frontier.reset();
  last_low_gain_reselection_time_ns.reset();
  reset_replacement_candidate_tracking();
  last_published_frontier_signature.reset();
  frontier_snapshot.reset();
  raw_frontier_debug_cache.reset();
  mrtsp_order_cache.reset();
  frontier_suppression_.reset();
  observation_progress_.clear();
  region_task_.clear();region_gain_pending_=false;
  symmetric_relocation_.clear();symmetry_requested_=false;
  relocation_arrival_anchor_.reset();relocation_arrival_until_=0;
  symmetric_no_goal_wait_=0;symmetric_wait_sample_=-1;
  floor_coverage_.clear();floor_handoff_requested_=false;last_floor_handoff_ns_=0;
  if(callbacks.set_floor_cleanup_mode)callbacks.set_floor_cleanup_mode(false);
  pending_observation_progress_.reset();
  frontier_suppression_activation_ns_.reset();

  if (clear_maps) {
    map.reset();
    decision_map.reset();
    decision_map_msg.reset();
    decision_map_workspace = DecisionMapWorkspace{};
    costmap.reset();
    local_costmap.reset();
    map_generation = 0;
    decision_map_generation = 0;
    costmap_generation = 0;
    local_costmap_generation = 0;
    decision_map_dirty = false;
    decision_map_cache_hits = 0;
    decision_map_cache_misses = 0;
    frontier_snapshot_cache_hits = 0;
    frontier_snapshot_cache_misses = 0;
    mrtsp_order_cache_hits = 0;
    mrtsp_order_cache_misses = 0;
  }
}

void FrontierExplorerCore::start_exploration_session()
{
  reset_exploration_runtime_state(true);
  exploration_enabled = true;
  if (params.frontier_suppression_enabled) {
    frontier_suppression_activation_ns_ =
      callbacks.now_ns() +
      static_cast<int64_t>(params.frontier_suppression_startup_grace_period_s * 1e9);
  }
}

void FrontierExplorerCore::stop_exploration_session(const std::string & reason)
{
  exploration_enabled = false;
  reset_exploration_runtime_state(true);
  if (goal_in_progress) {
    request_active_goal_cancel(reason);
  }
}

bool FrontierExplorerCore::ready_for_shutdown() const
{
  return !goal_in_progress && !cancel_request_in_progress;
}

void FrontierExplorerCore::set_goal_state(GoalLifecycleState state)
{
  // Keep enum state and convenience bool in sync from one write location.
  goal_state = state;
  // IDLE <-> non-IDLE mapping kept centralized to avoid inconsistent flags.
  goal_in_progress = state != GoalLifecycleState::IDLE;
}

void FrontierExplorerCore::mark_dispatch_state(int dispatch_id, GoalLifecycleState state)
{
  // Tracks lifecycle by dispatch id for stale callback filtering and diagnostics.
  dispatch_states[dispatch_id] = state;
}

void FrontierExplorerCore::reset_replacement_candidate_tracking()
{
  // Clear both candidate payload and debounce counter together to avoid stale "stable" hits.
  replacement_candidate_frontier.reset();
  replacement_candidate_hits = 0;
}

bool FrontierExplorerCore::has_stable_replacement_candidate(const FrontierSequence & frontier_sequence)
{
  if (frontier_sequence.empty()) {
    reset_replacement_candidate_tracking();
    return false;
  }

  const FrontierLike & selected_frontier = frontier_sequence.front();
  if (replacement_candidate_frontier.has_value() &&
    are_frontiers_equivalent(selected_frontier, replacement_candidate_frontier))
  {
    // Debounce only the selected replacement frontier. Later MRTSP order entries may change
    // without changing the actual replacement target, and Python reference behavior treats
    // that case as stable.
    replacement_candidate_hits += 1;
  } else {
    // New candidate observed; restart stability counter.
    replacement_candidate_frontier = selected_frontier;
    replacement_candidate_hits = 1;
  }

  // Candidate is considered stable only after configurable repeated hits.
  return replacement_candidate_hits >= replacement_required_hits;
}

std::optional<std::pair<double, double>> FrontierExplorerCore::active_goal_target_point() const
{
  if (active_goal_pose.has_value()) {
    return std::pair<double, double>{
      active_goal_pose->pose.position.x,
      active_goal_pose->pose.position.y,
    };
  }

  if (active_goal_frontier.has_value()) {
    return frontier_position(*active_goal_frontier);
  }

  return std::nullopt;
}

FrontierLike FrontierExplorerCore::frontier_with_goal_point(
  const FrontierLike & frontier,
  const std::pair<double, double> & goal_point) const
{
  FrontierLike updated_frontier = frontier;
  updated_frontier.goal_point = goal_point;
  return updated_frontier;
}

std::optional<double> FrontierExplorerCore::active_goal_visible_reveal_length() const
{
  if (!map.has_value() || !costmap.has_value() || !active_goal_frontier.has_value()) {
    return std::nullopt;
  }

  // Model the sensor at the dispatched target pose, not at the robot's current pose.
  // This keeps the gain estimate aligned with "what would be revealed if we finish this goal?"
  const auto goal_point = active_goal_target_point();
  if (!goal_point.has_value()) {
    return std::nullopt;
  }
  const auto reference_point = frontier_reference_point(*active_goal_frontier);
  double heading = 0.0;
  const double dx = reference_point.first - goal_point->first;
  const double dy = reference_point.second - goal_point->second;
  if (std::hypot(dx, dy) > 1e-6) {
    // Preferred heading points from the dispatched goal toward the frontier reference geometry.
    heading = std::atan2(dy, dx);
  } else if (active_goal_pose.has_value()) {
    // Degenerate frontier geometry falls back to the yaw of the already-dispatched goal pose.
    heading = detail::yaw_from_quaternion(active_goal_pose->pose.orientation);
  }

  const double yaw_offset_rad = params.goal_preemption_lidar_yaw_offset_deg * (detail::kPi / 180.0);
  geometry_msgs::msg::Pose sensor_pose;
  sensor_pose.position.x = goal_point->first;
  sensor_pose.position.y = goal_point->second;
  sensor_pose.orientation = detail::quaternion_from_yaw(heading + yaw_offset_rad);

  // The helper returns a local, occlusion-aware reveal length estimate around the target pose.
  const auto visible_reveal_bounds =
    active_goal_frontier->visible_reveal_bounds;

  const auto visible_gain = compute_visible_reveal_gain(
    sensor_pose,
    *map,
    *costmap,
    local_costmap,
    params.goal_preemption_lidar_range_m,
    params.goal_preemption_lidar_fov_deg,
    params.goal_preemption_lidar_ray_step_deg,
    visible_reveal_bounds);
  if (!visible_gain.has_value()) {
    return std::nullopt;
  }

  return visible_gain->visible_reveal_length_m;
}

double FrontierExplorerCore::frontier_snapshot_min_goal_distance_for_pose(
  const geometry_msgs::msg::Pose & current_pose)
{
  if (!distance_completed_frontier.has_value() ||
    params.goal_preemption_complete_if_within_m <= 0.0)
  {
    return params.frontier_candidate_min_goal_distance_m;
  }

  const auto completed_goal = frontier_position(*distance_completed_frontier);
  const double completed_distance = std::hypot(
    completed_goal.first - current_pose.position.x,
    completed_goal.second - current_pose.position.y);
  if (completed_distance > params.goal_preemption_complete_if_within_m) {
    distance_completed_frontier.reset();
    return params.frontier_candidate_min_goal_distance_m;
  }

  return std::max(
    params.frontier_candidate_min_goal_distance_m,
    params.goal_preemption_complete_if_within_m);
}

void FrontierExplorerCore::consider_preempt_active_goal(const std::string & trigger_source)
{
  // Costmap-triggered calls only update blocked reason; map-triggered calls may reseat goals.
  const bool completion_distance_enabled = params.goal_preemption_complete_if_within_m > 0.0;
  const bool preemption_allowed = (
    params.goal_preemption_enabled ||
    params.goal_skip_on_blocked_goal ||
    completion_distance_enabled);
  if (!preemption_allowed) {
    return;
  }

  if (!map.has_value() || !costmap.has_value() || !goal_in_progress || !active_goal_frontier.has_value()) {
    // Preemption logic requires an active frontier goal plus current map context.
    return;
  }

  if (!goal_handle) {
    // Goal has not been accepted yet, so cancel/reselect actions cannot be issued.
    return;
  }

  const auto current_pose = callbacks.get_current_pose();
  if (!current_pose.has_value()) {
    // Pose uncertainty: defer preemption decision.
    return;
  }

  const auto active_goal_cost_status = params.goal_skip_on_blocked_goal ?
    frontier_cost_status(active_goal_frontier) : std::optional<std::string>{};
  if (
    !active_goal_cost_status.has_value() &&
    !params.goal_preemption_enabled &&
    !completion_distance_enabled)
  {
    // Nothing to do when all active-goal update triggers are effectively inactive.
    return;
  }

  if (trigger_source != "map") {
    // Costmap/local-costmap callbacks intentionally skip full frontier reselection work.
    if (active_goal_cost_status.has_value()) {
      active_goal_blocked_reason = *active_goal_cost_status;
    }
    return;
  }

  if (cancel_request_in_progress) {
  // Avoid duplicate cancel requests while previous cancel handshake is in flight.
  return;
} 

if (!active_goal_cost_status.has_value()) {
  active_goal_blocked_reason.reset();
}

  const auto active_goal_point = active_goal_target_point();
  if (!active_goal_point.has_value()) {
    return;
  }
  const double active_goal_distance = std::hypot(
    active_goal_point->first - current_pose->position.x,
    active_goal_point->second - current_pose->position.y);
  std::optional<double> visible_reveal_length;
  // Near-goal completion is independent from visible-gain preemption; it is a close-enough guard.
  const bool revealed_completion_distance_reached =
    !active_goal_cost_status.has_value() &&
    completion_distance_enabled &&
    !active_goal_frontier->observation_goal &&
    active_goal_distance <= params.goal_preemption_complete_if_within_m;
  const std::string completion_distance_reason =
    "active frontier considered complete by goal_preemption_complete_if_within_m (distance=" +
    detail::format_meters(active_goal_distance) + ", threshold=" +
    detail::format_meters(params.goal_preemption_complete_if_within_m) + ")";

  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    !params.goal_preemption_enabled)
  {
    return;
  }

  if (revealed_completion_distance_reached) {
    distance_completed_frontier = frontier_with_goal_point(
      *active_goal_frontier,
      *active_goal_point);
    const auto context=dispatch_contexts.find(current_dispatch_id);
    if(context!=dispatch_contexts.end()) {context->second.completed_by_proximity=true;}
    request_active_goal_cancel(completion_distance_reason);
    return;
  }

  if (
    !revealed_completion_distance_reached &&
    !active_goal_sent_time_ns.has_value() &&
    !active_goal_cost_status.has_value())
  {
    // Without send timestamp we cannot evaluate time-based visible-gain preemption gate.
    return;
  }

  const int64_t now_ns = callbacks.now_ns();
  const double elapsed = active_goal_sent_time_ns.has_value() ?
    static_cast<double>(now_ns - *active_goal_sent_time_ns) / 1e9 : 0.0;

  // Time-based gate for visible-gain preemption to avoid immediate churn.
  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled &&
    elapsed < params.goal_preemption_min_interval_s)
  {
    return;
  }

  // Visible-gain gate is only meaningful on the map-triggered revealed-preemption path.
  const bool visible_gain_gate_active =
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled;
  bool visible_reveal_gain_exhausted = false;

  if (visible_gain_gate_active && !revealed_completion_distance_reached) {
    visible_reveal_length = active_goal_visible_reveal_length();
    if (!visible_reveal_length.has_value()) {
      callbacks.log_warn(
        "Skipping visible reveal gain gate for the active goal; falling back to frontier snapshot reselection");
    } else if (
      map.has_value() &&
      static_cast<double>(frontier_size(*active_goal_frontier)) * map->map().info.resolution <=
      *visible_reveal_length)
    {
      // Small frontier clusters can look fully revealable even with low absolute gain thresholds.
      // If the current visible reveal already covers the whole cluster-size proxy, skip preemption.
      reset_replacement_candidate_tracking();
      return;
    } else if (*visible_reveal_length >= params.goal_preemption_lidar_min_reveal_length_m) {
      // Keep current goal while target pose can still reveal enough unexplored boundary on arrival.
      reset_replacement_candidate_tracking();
      return;
    } else {
      // Low visible gain alone is not enough to preempt; the refreshed frontier snapshot still decides.
      visible_reveal_gain_exhausted = true;
    }
  }

  if (
    visible_reveal_gain_exhausted &&
    last_low_gain_reselection_time_ns.has_value() &&
    params.goal_preemption_min_interval_s > 0.0)
  {
    const double elapsed_since_low_gain_reselection =
      static_cast<double>(now_ns - *last_low_gain_reselection_time_ns) / 1e9;
    if (elapsed_since_low_gain_reselection < params.goal_preemption_min_interval_s) {
      return;
    }
  }
  if (visible_reveal_gain_exhausted) {
    last_low_gain_reselection_time_ns = now_ns;

    const std::string visible_gain_preemption_reason =
    visible_reveal_length.has_value() ?
    "active frontier visible reveal gain exhausted at target pose (visible=" +
    detail::format_meters(*visible_reveal_length) + ", required=" +
    detail::format_meters(params.goal_preemption_lidar_min_reveal_length_m) +
    "); canceling active goal and deferring frontier reselection until the next map refresh" :
    "active frontier visible reveal gain could not be evaluated; canceling active goal and deferring frontier reselection until the next map refresh";

  request_active_goal_cancel(visible_gain_preemption_reason);
  return;
  }

  // THIS PART MIGHT BE PROBLEMATIC FOR CPU USAGE
  // CHECK IN THE FUTURE

  FrontierSnapshot snapshot;
  try {
    snapshot = get_frontier_snapshot(*current_pose, params.frontier_candidate_min_goal_distance_m);
  } catch (const std::out_of_range & exc) {
    callbacks.log_warn(std::string("Skipping frontier reselection: ") + exc.what());
    return;
  }

  FrontierSequence actionable_frontiers;
  for(const auto & frontier:snapshot.frontiers) {
    if(params.floor_cleanup_enabled &&
       floor_coverage_.closedReference(*map,frontier_reference_point(frontier)))continue;
    actionable_frontiers.push_back(frontier);
  }
  const FrontierSequence & frontiers = actionable_frontiers;
  FrontierSequence filtered_frontiers = filter_frontiers_for_suppression(frontiers);
  if (filtered_frontiers.empty() && !frontiers.empty()) {
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    callbacks.log_info("All replacement frontiers are temporarily suppressed");
    if (active_goal_cost_status.has_value()) {
      // If current goal is blocked and no alternative frontier exists, cancel proactively.
      request_active_goal_cancel(
        *active_goal_cost_status + "; no unsuppressed replacement frontier is available");
    }
    return;
  }

  if (filtered_frontiers.empty()) {
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    if (active_goal_cost_status.has_value()) {
      // Blocked-goal path cancels proactively when no replacement target exists at all.
      request_active_goal_cancel(
        *active_goal_cost_status + "; no replacement frontier is available");
    }
    return;
  }

  if (
    !revealed_completion_distance_reached &&
    !active_goal_cost_status.has_value() &&
    params.goal_preemption_enabled &&
    frontier_exists_in_set(active_goal_frontier, filtered_frontiers))
  {
    // Snapshot membership remains the final authority for revealed-preemption.
    // If the frontier still exists in the refreshed set, keep the active goal.
    reset_replacement_candidate_tracking();
    publish_frontier_markers(filtered_frontiers);
    return;
  }

  const auto selection = select_frontier(filtered_frontiers, *current_pose);
  publish_frontier_markers(filtered_frontiers);
  const auto frontier_sequence = select_frontier_sequence(
    filtered_frontiers,
    *current_pose,
    selection.frontier);
  if (frontier_sequence.empty()) {
    // Selection produced no dispatchable target.
    reset_replacement_candidate_tracking();
    return;
  }

  if (are_frontier_sequences_equivalent(frontier_sequence, active_goal_frontiers)) {
    // Replacement candidate is effectively same as active target.
    reset_replacement_candidate_tracking();
    return;
  }

  const std::string revealed_preemption_reason = active_goal_cost_status.value_or(
    visible_reveal_gain_exhausted && visible_reveal_length.has_value() ?
    "active frontier revealed and visible reveal gain exhausted at target pose (visible=" +
    detail::format_meters(*visible_reveal_length) + ", required=" +
    detail::format_meters(params.goal_preemption_lidar_min_reveal_length_m) +
    "); preempting to the next frontier" :
    "active frontier no longer appears in the latest frontier snapshot after reveal updates; preempting to the next frontier");

  const std::string goal_update_log_prefix = active_goal_cost_status.has_value() ?
    "Skipping blocked frontier goal" :
    "Preempting active frontier goal";

  request_frontier_reselection(
    frontier_sequence,
    *current_pose,
    selection.mode,
    revealed_preemption_reason,
    goal_update_log_prefix);
}

void FrontierExplorerCore::request_frontier_reselection(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & selection_mode,
  const std::string & reselection_reason,
  const std::string & goal_update_log_prefix)
{
  if (!goal_handle || frontier_sequence.empty()) {
    // Reselection is meaningful only with an active accepted goal and a non-empty replacement.
    return;
  }

  if (!pending_frontier_sequence.empty() &&
    are_frontier_sequences_equivalent(frontier_sequence, pending_frontier_sequence))
  {
    // Keep queue compact by suppressing duplicate pending replacement requests.
    return;
  }

  if (!has_stable_replacement_candidate(frontier_sequence)) {
    // Require repeated observation before replacing active target.
    return;
  }

  pending_frontier_sequence = frontier_sequence;
  pending_frontier_selection_mode = selection_mode;
  pending_frontier_dispatch_context = "reselected";
  callbacks.log_info(
    goal_update_log_prefix + ": " + reselection_reason);
  // Use Nav2's native action preemption path by sending the replacement goal directly.
  // Post-goal settle remains reserved for terminal result paths, not live replacements.
  dispatch_pending_frontier_goal(current_pose);
}

void FrontierExplorerCore::request_active_goal_cancel(const std::string & reason)
{
  // Keep earliest cancel reason; subsequent requests should not erase context.
  if (!pending_cancel_reason.has_value()) {
    pending_cancel_reason = reason;
  }

  if (!goal_handle || cancel_request_in_progress) {
    // Cancel will be attempted later when handle exists and no request is in progress.
    return;
  }

  issue_active_goal_cancel();
}

void FrontierExplorerCore::issue_active_goal_cancel()
{
  if (!goal_handle || cancel_request_in_progress) {
    return;
  }

  const std::string reason = pending_cancel_reason.value_or("Canceling active goal");
  pending_cancel_reason.reset();
  cancel_request_in_progress = true;
  set_goal_state(GoalLifecycleState::CANCELING);
  callbacks.log_debug(reason);
  callbacks.log_info("Navigation cancel: " + reason);

  const int dispatch_id = current_dispatch_id;
  // Bind cancel response to current dispatch to ignore late/stale acknowledgements.
  goal_handle->cancel_goal_async(
    [this, dispatch_id](bool accepted, const std::string & error_message) {
      cancel_response_callback(dispatch_id, accepted, error_message);
    });
}

void FrontierExplorerCore::cancel_response_callback(
  int dispatch_id,
  bool cancel_accepted,
  const std::string & error_message)
{
  if (dispatch_id != current_dispatch_id) {
    // Ignore cancel responses from superseded goals.
    return;
  }

  if (!cancel_request_in_progress || goal_state != GoalLifecycleState::CANCELING) {
    // Result handling or a newer dispatch may have already closed this cancel path.
    // Late cancel responses must not resurrect an active goal after cleanup.
    return;
  }

  if (!error_message.empty()) {
    cancel_request_in_progress = false;
    set_goal_state(GoalLifecycleState::ACTIVE);
    callbacks.log_warn("Failed to cancel active goal: " + error_message);
    return;
  }

  if (!cancel_accepted) {
    cancel_request_in_progress = false;
    set_goal_state(GoalLifecycleState::ACTIVE);
    callbacks.log_warn("Active goal cancel request was not accepted");
  }
}

bool FrontierExplorerCore::dispatch_pending_frontier_goal(
  const std::optional<geometry_msgs::msg::Pose> & current_pose)
{
  if (pending_frontier_sequence.empty()) {
    // Nothing queued by preemption/reselection logic.
    return false;
  }

  std::optional<geometry_msgs::msg::Pose> pose = current_pose;
  if (!pose.has_value()) {
    // Keep sequence pending if TF is transiently unavailable; caller retries later.
    pose = callbacks.get_current_pose();
    if (!pose.has_value()) {
      return true;
    }
  }

  FrontierSequence frontier_sequence = filter_frontiers_for_suppression(pending_frontier_sequence);
  if(params.floor_cleanup_enabled && map)
    frontier_sequence.erase(std::remove_if(frontier_sequence.begin(),frontier_sequence.end(),
      [&](const auto & candidate) {
        return floor_coverage_.closedReference(*map,frontier_reference_point(candidate));
      }),frontier_sequence.end());
  if (frontier_sequence.empty()) {
    pending_frontier_sequence.clear();
    pending_frontier_selection_mode.clear();
    pending_frontier_dispatch_context.clear();
    reset_replacement_candidate_tracking();
    return false;
  }
  const std::string selection_mode = pending_frontier_selection_mode.empty() ?
    "mrtsp" : pending_frontier_selection_mode;

  const bool dispatched = send_frontier_goal(
    frontier_sequence,
    *pose,
    "Sending updated frontier goal (" + selection_mode + "): " +
    describe_frontier(frontier_sequence.front()));

  if (dispatched) {
    // Consume pending sequence only after successful dispatch to preserve at-least-once intent.
    pending_frontier_sequence.clear();
    pending_frontier_selection_mode.clear();
    pending_frontier_dispatch_context.clear();
    active_goal_blocked_reason.reset();
    reset_replacement_candidate_tracking();
    clear_post_goal_wait_state();
  }

  return true;
}

void FrontierExplorerCore::handle_exploration_complete(const geometry_msgs::msg::Pose & current_pose)
{
  callbacks.on_exploration_complete();

  if (return_to_start_completed) {
    return;
  }

  if (!params.return_to_start_on_complete || !start_pose.has_value())
  {
    // Mark completion when return-to-start is disabled (or unavailable) to stop repeated no-frontier scans.
    return_to_start_completed = true;
    return;
  }

  if (is_pose_within_xy_tolerance(current_pose, start_pose->pose)) {
    // Exploration already ended near start; avoid issuing a redundant return goal.
    return_to_start_completed = true;
    callbacks.log_info("Exploration finished at the start pose");
    return;
  }

  if (!return_to_start_started) {
    // Latch this flag before dispatch to prevent duplicate return goals on repeated callbacks.
    return_to_start_started = true;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2)
        << "Exploration finished, returning to start pose: ("
        << start_pose->pose.position.x << ", "
        << start_pose->pose.position.y << ")";

    send_pose_goal(
      *start_pose,
      "return_to_start",
      std::nullopt,
      {},
      oss.str());
  }
}

bool FrontierExplorerCore::is_pose_within_xy_tolerance(
  const geometry_msgs::msg::Pose & current_pose,
  const geometry_msgs::msg::Pose & goal_pose,
  double tolerance) const
{
  // XY-only proximity check intentionally ignores yaw for exploration completion decisions.
  return std::hypot(
    goal_pose.position.x - current_pose.position.x,
    goal_pose.position.y - current_pose.position.y) <= tolerance;
}

bool FrontierExplorerCore::try_symmetric_relocation(
  const geometry_msgs::msg::Pose & current,bool trigger)
{
  if(!map || !callbacks.request_symmetric_relocation_step)return false;
  const double now=callbacks.now_ns()*1e-9;
  const auto & raw=map->map();
  ObservationGrid grid;grid.width=raw.info.width;grid.height=raw.info.height;
  grid.resolution=raw.info.resolution;grid.x=raw.info.origin.position.x;
  grid.y=raw.info.origin.position.y;const auto & q=raw.info.origin.orientation;
  grid.yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));grid.cells=raw.data;
  symmetric_relocation_.observe(grid,raw.info.origin.position.z,now);
  symmetry_requested_=false;
  if(trigger && !symmetric_relocation_.active() &&
     symmetric_relocation_.begin(grid,{current.position.x,current.position.y},now,
       params.exploration_boundary_polygon,params.exploration_boundary_margin_m)) {
    const auto & t=*symmetric_relocation_.target;
    region_task_.release();region_gain_pending_=false;pending_observation_progress_.reset();
    pending_frontier_sequence.clear();observation_continuation_reference_.reset();
    callbacks.log_warn("Symmetric relocation selected: known_center=("+
      std::to_string(t.center.first)+","+std::to_string(t.center.second)+"), mirror=("+
      std::to_string(t.mirror.first)+","+std::to_string(t.mirror.second)+"), fixed_destination=("+
      std::to_string(t.destination.first)+","+std::to_string(t.destination.second)+
      "); retain destination across verified segments");
  }
  if(!symmetric_relocation_.active())return false;
  if(symmetric_relocation_.arrived({current.position.x,current.position.y})) {
    callbacks.log_info("Symmetric relocation destination reached; resume exploration with fresh observations");
    relocation_arrival_anchor_=symmetric_relocation_.target->destination;
    relocation_arrival_until_=now+60.;
    symmetric_relocation_.finish(now);symmetric_no_goal_wait_=0;
    frontier_snapshot.reset();raw_frontier_debug_cache.reset();mrtsp_order_cache.reset();decision_map_dirty=true;
    return false;
  }
  if(symmetric_relocation_.planningExpired(now)) {
    callbacks.log_warn("Symmetric relocation deferred: three failed segment validations and 30 healthy planning seconds; unknowns retained");
    symmetric_relocation_.finish(now);symmetric_no_goal_wait_=0;
    return false;
  }
  const auto step=callbacks.request_symmetric_relocation_step(current,
    symmetric_relocation_.target->destination);
  if(step.failed) {
    symmetric_relocation_.rejected();
    callbacks.log_warn("Symmetric relocation segment has no verified route; failure="+
      std::to_string(symmetric_relocation_.failures));
  }
  if(!step.goal)return true;
  const auto & goal=*step.goal;
  if(goal.header.frame_id!=params.global_frame ||
     !is_inside_exploration_boundary({goal.pose.position.x,goal.pose.position.y},
       params.exploration_boundary_polygon,params.exploration_boundary_margin_m)) {
    callbacks.log_warn("Symmetric relocation rejected before action dispatch: target frame or inset boundary");
    symmetric_relocation_.rejected();return true;
  }
  ConnectedDispatch connected(*costmap,*map,local_costmap,current,params.global_frame,
    0.,params.occ_threshold,params.exploration_boundary_polygon,params.exploration_boundary_margin_m);
  if(!connected.failure.empty() || !connected.choose(
      {goal.pose.position.x,goal.pose.position.y},1e-6,.25)) {
    callbacks.log_warn("Symmetric relocation rejected before action dispatch: "+
      (connected.failure.empty()?std::string("target disconnected or blocked in latest grids"):connected.failure));
    symmetric_relocation_.rejected();return true;
  }
  if(callbacks.publish_recovery_pose)callbacks.publish_recovery_pose(goal);
  if(!send_pose_goal(goal,"symmetry_relocation",std::nullopt,{},
      "Sending verified segment toward fixed symmetric relocation destination"))
    symmetric_relocation_.rejected();
  return true;
}

bool FrontierExplorerCore::try_random_recovery(
  const geometry_msgs::msg::Pose & current,bool blocked)
{
  if(goal_in_progress || !exploration_enabled || !map || !costmap ||
    !callbacks.request_random_recovery) return false;
  const auto goal=callbacks.request_random_recovery(current,blocked);
  if(callbacks.take_clearance_recovery_request && callbacks.take_clearance_recovery_request()) {
    geometry_msgs::msg::PoseStamped measured;
    measured.header.frame_id=params.global_frame;measured.pose=current;
    return send_pose_goal(measured,"clearance_recovery",std::nullopt,{},
      "Starting automatic supported clearance restoration before normal replanning");
  }
  if(!goal) {
    if(callbacks.random_recovery_pending && callbacks.random_recovery_pending())return true;
  if(callbacks.request_symmetric_relocation_step && blocked &&
     is_inside_exploration_boundary({current.position.x,current.position.y},
       params.exploration_boundary_polygon,0.)) {
    double now=callbacks.now_ns()*1e-9;
    if(symmetric_wait_sample_>=0) {
      double dt=now-symmetric_wait_sample_;
      if(dt>=0 && dt<=3.+1e-6)symmetric_no_goal_wait_+=dt;
    }
    symmetric_wait_sample_=now;
    if(symmetric_no_goal_wait_>=60. && try_symmetric_relocation(current,true))return true;
  }
    return false;
  }
  if(goal->header.frame_id!=params.global_frame ||
    !is_inside_exploration_boundary({goal->pose.position.x,goal->pose.position.y},
      params.exploration_boundary_polygon,params.exploration_boundary_margin_m)) {
    callbacks.log_warn("Verified random recovery rejected before action dispatch: target frame or inset boundary");
    return false;
  }
  const bool reentry=!is_inside_exploration_boundary({current.position.x,current.position.y},
    params.exploration_boundary_polygon,0.);
  ConnectedDispatch connectivity(*costmap,*map,local_costmap,current,params.global_frame,
    0.0,params.occ_threshold,params.exploration_boundary_polygon,params.exploration_boundary_margin_m,reentry);
  if(!connectivity.failure.empty()) {
    callbacks.log_warn("Verified random recovery rejected before action dispatch: "+connectivity.failure);
    return false;
  }
  if(!connectivity.choose({goal->pose.position.x,goal->pose.position.y},1e-6,reentry?.15:.5)) {
    callbacks.log_warn("Verified random recovery rejected before action dispatch: target outside inset boundary, disconnected, or blocked in latest grids");
    return false;
  }
  if(callbacks.publish_recovery_pose)callbacks.publish_recovery_pose(*goal);
  else callbacks.publish_selected_frontier_pose(*goal);
  return send_pose_goal(*goal,"random_recovery",std::nullopt,{},
    "Sending verified random recovery goal; normal exploration resumes after settling");
}

bool FrontierExplorerCore::send_pose_goal(
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & goal_kind,
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontier_sequence,
  const std::string & description)
{
  const bool clearance=goal_kind=="clearance_recovery";
  const bool available=clearance?(callbacks.wait_for_clearance_server &&
    callbacks.wait_for_clearance_server(0.0)):callbacks.wait_for_action_server(1.0);
  if (!available) {
    // Keep core non-blocking: schedule retry on future updates instead of waiting indefinitely.
    callbacks.log_info(
      "Action '" + (clearance?std::string("escape_move"):params.navigate_to_pose_action_name) + "' not ready yet, waiting...");
    if (goal_kind == "return_to_start") {
      return_to_start_started = false;
    } else if (goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    return false;
  }

  dispatch_goal_request(
    clearance?"escape_move":params.navigate_to_pose_action_name,
    goal_pose,
    goal_kind,
    frontier,
    frontier_sequence,
    description);
  return true;
}

bool FrontierExplorerCore::send_frontier_goal(
  const FrontierSequence & frontier_sequence,
  const geometry_msgs::msg::Pose & current_pose,
  const std::string & description,
  bool bypass_min_distance_dispatch)
{
  if (frontier_sequence.empty() || !map.has_value() || !costmap.has_value()) {
    return false;
  }

  // The global costmap already contains Nav2 obstacle and inflation costs.
  // A second geometric clearance would inflate obstacles again and can close
  // otherwise valid corridors, so connectivity uses zero extra clearance.
  constexpr double extra_clearance_m = 0.0;

  // Only allow a small relocation around the proposed goal. A large radius
  // could move a goal through a wall onto an unrelated reachable region.
  const double connected_goal_search_radius_m =
    std::max(0.30, 2.5 * costmap->map().info.resolution);

  ConnectedDispatch connected_dispatch(
    *costmap,
    *map,
    local_costmap,
    current_pose,
    params.global_frame,
    extra_clearance_m,
    params.occ_threshold,
    params.exploration_boundary_polygon,
    params.exploration_boundary_margin_m);

  if (!connected_dispatch.failure.empty()) {
    callbacks.log_warn(
      "Connectivity check unavailable: " + connected_dispatch.failure +
      "; waiting for map or costmap update");
    return false;
  }

  FrontierSequence dispatch_sequence;
  dispatch_sequence.reserve(frontier_sequence.size());
  std::optional<geometry_msgs::msg::PoseStamped> verified_goal;

  if(callbacks.update_observation_exclusions)
    callbacks.update_observation_exclusions(
      observation_progress_.blockedObservationCenters(callbacks.now_ns()));
  for (const auto & candidate_frontier : frontier_sequence) {
    if(params.floor_cleanup_enabled &&
       floor_coverage_.closedReference(*map,frontier_reference_point(candidate_frontier))) {
      callbacks.log_info("Skipping frontier whose local unknowns are verified closed before dispatch");
      continue;
    }
    if(candidate_frontier.viewpoint_search_exhausted) {
      callbacks.log_info("Skipping boundary with no safe fan viewpoint: "+
        describe_frontier(candidate_frontier));
      continue;
    }
    // Pending sequences and validated candidates can outlive the snapshot
    // that selected them. Recheck reference cooldown at the actual dispatch.
    if ((!params.region_observation_tasks_enabled && observation_progress_.blockedReference(
        frontier_reference_point(candidate_frontier), callbacks.now_ns())) ||
        (params.region_observation_tasks_enabled &&
          (region_task_.deferred(*map,frontier_reference_point(candidate_frontier),callbacks.now_ns()*1e-9) ||
           (region_task_.active() && !region_task_.matches(*map,frontier_reference_point(candidate_frontier)))))) {
      callbacks.log_info("Skipping cooled frontier reference before dispatch: " +
        describe_frontier(candidate_frontier));
      continue;
    }
    if (params.goal_skip_on_blocked_goal &&
      (!callbacks.validate_frontier_goal_set ||
       (candidate_frontier.viewpoint_candidates.empty() &&
        candidate_frontier.fallback_viewpoint_candidates.empty()))) {
      const auto cost_status =
        frontier_cost_status(std::optional<FrontierLike>{candidate_frontier});

      if (cost_status.has_value()) {
        callbacks.log_info(
          "Skipping blocked frontier goal before dispatch: " +
          *cost_status + "; " + describe_frontier(candidate_frontier));
        continue;
      }
    }

    const auto proposed_goal = build_dispatch_goal_pose(
      candidate_frontier,
      current_pose,
      bypass_min_distance_dispatch || floor_coverage_.cleanup());

    const std::pair<double, double> proposed_point{
      proposed_goal.pose.position.x,
      proposed_goal.pose.position.y,
    };

    const auto reachable_point = connected_dispatch.choose(
      proposed_point,
      connected_goal_search_radius_m,
      0.0);

    if (!reachable_point.has_value() &&
      (!callbacks.validate_frontier_goal_set ||
      (candidate_frontier.viewpoint_candidates.empty() &&
       candidate_frontier.fallback_viewpoint_candidates.empty()))) {
      callbacks.log_info(
        "Skipping disconnected frontier goal before dispatch: " +
        describe_frontier(candidate_frontier));
      continue;
    }

    // Bind before asynchronous validation: a region whose first route never
    // succeeds must still have a bounded planning wait.
    if(params.region_observation_tasks_enabled && !region_task_.active())
      region_task_.start(*map,frontier_reference_point(candidate_frontier),callbacks.now_ns()*1e-9);
    auto dispatch_frontier = frontier_with_goal_point(candidate_frontier, reachable_point.value_or(proposed_point));
    if(callbacks.set_observation_reference)
      callbacks.set_observation_reference(frontier_reference_point(candidate_frontier));
    if (callbacks.validate_frontier_goal || callbacks.validate_frontier_goal_set) {
      std::vector<geometry_msgs::msg::PoseStamped> observations;
      auto append_observations=[&](const auto & points) {
        for(const auto & point:points) {
          if(observations.size()>=((floor_coverage_.expansion()>0 ||
              (params.region_observation_tasks_enabled && region_task_.active()))?80u:25u))break;
          const auto reachable=connected_dispatch.choose(point,1e-6,0.0);
          if(!reachable || observation_progress_.blockedObservation(point,callbacks.now_ns()))continue;
          auto variant=frontier_with_goal_point(candidate_frontier,point);
          if(params.goal_skip_on_blocked_goal &&
            frontier_cost_status(std::optional<FrontierLike>{variant}))continue;
          const auto goal=build_goal_pose(variant,current_pose);
          const auto & q=goal.pose.orientation;
          const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
          if(region_task_.alreadyObserved(*map,frontier_reference_point(candidate_frontier),point,yaw)) {
            callbacks.log_debug("Reusing no-gain scan history; skipping the same target/view/heading");
            continue;
          }
          observations.push_back(goal);
        }
      };
      append_observations(candidate_frontier.viewpoint_candidates);
      append_observations(candidate_frontier.fallback_viewpoint_candidates);
      // Legacy/custom producers without a fan retain the connected single goal.
      if(candidate_frontier.viewpoint_candidates.empty() &&
        candidate_frontier.fallback_viewpoint_candidates.empty())
      {
        const auto goal=build_goal_pose(dispatch_frontier,current_pose);const auto & q=goal.pose.orientation;
        const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        if(!region_task_.alreadyObserved(*map,frontier_reference_point(candidate_frontier),
          {goal.pose.position.x,goal.pose.position.y},yaw))observations.push_back(goal);
      }
      if(observations.empty()) {region_task_.noCandidate(callbacks.now_ns()*1e-9);continue;}
      const auto checked = callbacks.validate_frontier_goal_set ?
        callbacks.validate_frontier_goal_set(current_pose,observations) :
        callbacks.validate_frontier_goal(current_pose,observations.front());
      if (!checked) {
        if (!callbacks.frontier_goal_validation_pending || !callbacks.frontier_goal_validation_pending()) {
          callbacks.log_info("Skipping frontier without a verified 3D route: " +
            describe_frontier(candidate_frontier));
        }
        // Missing support in this snapshot can be revealed by the next scan.
        // It is not a failed navigation attempt and must not blacklist the area.
        continue;
      }
      if (!connected_dispatch.choose(
        {checked->pose.position.x,checked->pose.position.y},1e-6,0.0)) {
        continue;
      }
      dispatch_frontier = frontier_with_goal_point(dispatch_frontier,
        {checked->pose.position.x,checked->pose.position.y});
      dispatch_frontier.goal_z = checked->pose.position.z;
      // Preserve the distinction between a frontier goal and an intermediate approach.
      dispatch_frontier.observation_goal = candidate_frontier.requires_visible_viewpoint || std::hypot(
        checked->pose.position.x - reachable_point.value_or(proposed_point).first,
        checked->pose.position.y - reachable_point.value_or(proposed_point).second) >
        std::max(0.30, params.goal_preemption_complete_if_within_m);
      verified_goal = checked;
    }
    if(params.region_observation_tasks_enabled) {
      const auto goal=verified_goal.value_or(build_goal_pose(dispatch_frontier,current_pose));
      const auto & q=goal.pose.orientation;
      const double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
      if(region_task_.alreadyObserved(*map,frontier_reference_point(candidate_frontier),
        {goal.pose.position.x,goal.pose.position.y},yaw)) {
        callbacks.log_info("Skipping previously completed no-gain scan at the same target/view/heading");
        verified_goal.reset();continue;
      }
    }
    // The 3D validator can relocate the target: apply the same final-point
    // cost test used during active navigation before accepting that relocation.
    if (params.goal_skip_on_blocked_goal) {
      const auto final_cost = frontier_cost_status(std::optional<FrontierLike>{dispatch_frontier});
      if (final_cost) {
        callbacks.log_info("Skipping blocked final observation before dispatch: " + *final_cost);
        verified_goal.reset();
        continue;
      }
    }
    if (observation_progress_.blockedObservation(
        frontier_position(dispatch_frontier),callbacks.now_ns())) {
      callbacks.log_info("Skipping repeated observation with no information gain: "+describe_frontier(dispatch_frontier));
      verified_goal.reset(); continue;
    }
    dispatch_sequence.push_back(dispatch_frontier);
    // Revalidate later observations against the fresh 3D map at dispatch time.
    if (verified_goal) {break;}
  }

  if (dispatch_sequence.empty()) {
    floor_coverage_.exhaustedSearch(callbacks.now_ns()*1e-9,false,
      callbacks.frontier_goal_validation_pending && callbacks.frontier_goal_validation_pending());
    if(floor_coverage_.shouldHandoff(callbacks.now_ns()*1e-9) &&
       (!callbacks.residual_handoff_pose_safe || callbacks.residual_handoff_pose_safe()))return true;
    if(try_random_recovery(current_pose,true)) {return true;}
    if (callbacks.frontier_goal_validation_pending && callbacks.frontier_goal_validation_pending()) {
      callbacks.log_info("3D observation validation pending; exploration executor remains responsive");
    } else {
      callbacks.log_info("No verified observation yet; candidates may be cooling down or waiting for map data");
    }
    return false;
  }

  // Build the final pose directly from the point verified by connectivity.
  // Calling build_dispatch_goal_pose() again could relocate it away from the
  // verified connected cell.
  const auto goal_pose =
    verified_goal.value_or(build_goal_pose(dispatch_sequence.front(), current_pose));

  callbacks.publish_selected_frontier_pose(goal_pose);

  const std::string dispatch_description =
    description +
    "; connected dispatch=" +
    describe_frontier(dispatch_sequence.front());

  return send_pose_goal(
    goal_pose,
    "frontier",
    dispatch_sequence.front(),
    dispatch_sequence,
    dispatch_description);
}

void FrontierExplorerCore::dispatch_goal_request(
  const std::string & action_name,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  const std::string & goal_kind,
  const std::optional<FrontierLike> & frontier,
  const FrontierSequence & frontier_sequence,
  const std::string & description)
{
  // Dispatch ids make out-of-order callbacks safe when old and new goals overlap.
  if (goal_in_progress && dispatch_states.find(current_dispatch_id) != dispatch_states.end()) {
    mark_dispatch_state(current_dispatch_id, GoalLifecycleState::SUPERSEDED);
  }

  current_dispatch_id += 1;
  const int dispatch_id = current_dispatch_id;

  active_goal_kind = goal_kind;
  active_goal_pose = goal_pose;
  active_goal_frontier = frontier;
  active_goal_frontiers = frontier_sequence;
  active_action_name = action_name;
  mark_dispatch_state(dispatch_id, GoalLifecycleState::SENDING);
  set_goal_state(GoalLifecycleState::SENDING);
  goal_handle.reset();
  // Send timestamp anchors min-interval preemption gating.
  active_goal_sent_time_ns = callbacks.now_ns();
  last_low_gain_reselection_time_ns.reset();
  awaiting_map_refresh = false;
  callbacks.log_info(description);

  dispatch_contexts[dispatch_id] = DispatchContext{
    goal_kind,
    frontier,
    frontier_sequence,
    action_name,
    goal_kind=="frontier" && frontier && map ?
      std::optional<ObservationProgress::Snapshot>(observation_progress_.capture(*map,
        frontier_reference_point(*frontier),{goal_pose.pose.position.x,goal_pose.pose.position.y})) : std::nullopt,
  };

  if(params.region_observation_tasks_enabled && goal_kind=="frontier" && frontier && map) {
    if(const auto actual=callbacks.get_current_pose())
      observation_progress_.primeRoute(*map,frontier_reference_point(*frontier),
        {actual->position.x,actual->position.y},callbacks.now_ns());
    if(region_task_.start(*map,frontier_reference_point(*frontier),callbacks.now_ns()*1e-9)) {
      callbacks.log_info("Regional task bound to unknown footprint; frontier centroid drift does not release it");
      frontier_snapshot.reset();decision_map_dirty=true;
    }
  }

  callbacks.dispatch_goal_request(GoalDispatchRequest{
    dispatch_id,
    action_name,
    goal_kind,
    goal_pose,
    frontier,
    frontier_sequence,
    description,
  });
}

void FrontierExplorerCore::clear_active_goal_state()
{
  // Clears all active lifecycle fields after terminal result/cancel.
  set_goal_state(GoalLifecycleState::IDLE);
  goal_handle.reset();
  active_goal_pose.reset();
  active_goal_frontier.reset();
  active_goal_frontiers.clear();
  active_goal_kind.clear();
  active_action_name.clear();
  active_goal_sent_time_ns.reset();
  active_goal_blocked_reason.reset();
  last_low_gain_reselection_time_ns.reset();
  clear_active_goal_progress_state();
  dispatch_states.clear();
  dispatch_contexts.clear();
}

std::optional<FrontierExplorerCore::DispatchContext> FrontierExplorerCore::dispatch_context_for(
  int dispatch_id) const
{
  // Dispatch context is best-effort metadata; stale callbacks may legitimately miss this entry.
  auto it = dispatch_contexts.find(dispatch_id);
  if (it == dispatch_contexts.end()) {
    return std::nullopt;
  }
  return it->second;
}

void FrontierExplorerCore::goal_response_callback(
  int dispatch_id,
  const std::shared_ptr<GoalHandleInterface> & received_goal_handle,
  bool accepted,
  const std::string & error_message)
{
  if (shutdown_requested) {
    // Ignore callbacks after teardown requested.
    return;
  }

  const auto context = dispatch_context_for(dispatch_id);

  if (!accepted) {
    if (dispatch_id != current_dispatch_id) {
      return;
    }

    if (context.has_value() && context->goal_kind == "return_to_start") {
      return_to_start_started = false;
    }
    if (context.has_value() && context->goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    if (context.has_value() && context->goal_kind == "frontier") {
      record_failed_frontier_attempt(context->frontier);
    }
    if(context && context->goal_kind=="symmetry_relocation")
      symmetric_relocation_.finish(callbacks.now_ns()*1e-9);
    clear_active_goal_state();
    callbacks.log_error(error_message.empty() ? "Frontier goal was rejected" : error_message);
    return;
  }

  if (dispatch_id != current_dispatch_id) {
    // Late response from superseded dispatch is ignored; Nav2 handles active-goal preemption.
    return;
  }

  no_next_goal_.accepted(callbacks.now_ns()*1e-9);
  goal_handle = received_goal_handle;
  mark_dispatch_state(dispatch_id, GoalLifecycleState::ACTIVE);
  set_goal_state(GoalLifecycleState::ACTIVE);
  if (context.has_value() && context->goal_kind == "return_to_start") {
    callbacks.log_info("Return-to-start goal accepted");
  } else if (context.has_value() && context->goal_kind == "suppressed_return_to_start") {
    callbacks.log_info("Temporary return-to-start goal accepted while frontiers are suppressed");
  } else if(context && context->goal_kind=="clearance_recovery") {
    callbacks.log_info("Automatic clearance recovery goal accepted");
  } else if(context && context->goal_kind=="symmetry_relocation") {
    callbacks.log_info("Symmetric relocation segment accepted");
  } else if(context && context->goal_kind=="random_recovery") {
    callbacks.log_info("Random recovery goal accepted");
  } else {
    callbacks.log_info("Frontier goal accepted");
  }
  start_active_goal_progress_tracking();

  if (pending_cancel_reason.has_value()) {
    // If cancel was requested while sending, execute cancel immediately after accept.
    issue_active_goal_cancel();
  }
}

void FrontierExplorerCore::get_result_callback(
  int dispatch_id,
  int status,
  int error_code,
  const std::string & error_message,
  const std::string & exception_text)
{
  const auto context = dispatch_context_for(dispatch_id);
  // Cache derived values early so later cleanup cannot invalidate log/context decisions.
  const std::string goal_kind = context.has_value() ? context->goal_kind : "";
  const FrontierSequence frontier_sequence = context.has_value() ? context->frontier_sequence : FrontierSequence{};

  if (exception_text.empty()) {
    if (dispatch_id != current_dispatch_id) {
      // Result belongs to an older superseded dispatch.
      return;
    }

    if(status==action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && goal_kind=="random_recovery") {
      const auto actual=callbacks.get_current_pose();
      if(!actual || !active_goal_pose ||
         !is_inside_exploration_boundary({actual->position.x,actual->position.y},
           params.exploration_boundary_polygon,params.exploration_boundary_margin_m) ||
         std::hypot(actual->position.x-active_goal_pose->pose.position.x,
           actual->position.y-active_goal_pose->pose.position.y)>.25) {
        status=action_msgs::msg::GoalStatus::STATUS_ABORTED;
        callbacks.log_warn("Recovery success rejected: actual pose is missing, outside safe mission inset, or too far from target");
      }
    }
    if(goal_kind=="symmetry_relocation") {
      pending_frontier_sequence.clear();pending_observation_progress_.reset();
      const auto actual=callbacks.get_current_pose();
      const bool measured=status==action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && actual &&
        active_goal_pose && std::hypot(active_goal_pose->pose.position.x-actual->position.x,
          active_goal_pose->pose.position.y-actual->position.y)<=.35;
      if(!measured || !symmetric_relocation_.stage({actual->position.x,actual->position.y})) {
        symmetric_relocation_.finish(callbacks.now_ns()*1e-9);
        callbacks.log_warn("Symmetric relocation stopped after failed/unmeasured/repeated segment; destination retained in avoidance history");
      } else {
        callbacks.log_info("Symmetric relocation segment reached; retain fixed far destination");
      }
      symmetric_no_goal_wait_=0;
    } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED && goal_kind == "return_to_start") {
      // Return path uses dedicated completion latch consumed by scheduler.
      return_to_start_completed = true;
      callbacks.log_info("Returned to start pose");
    } else if (
      status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
      goal_kind == "suppressed_return_to_start")
    {
      callbacks.log_info("Reached start pose while frontiers remain temporarily suppressed");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
        goal_kind=="clearance_recovery") {
      observation_continuation_reference_.reset();
      pending_frontier_sequence.clear();
      callbacks.log_info("Measured clearance restored; waiting for fresh map before normal replanning");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED &&
        goal_kind=="random_recovery") {
      observation_continuation_reference_.reset();
      callbacks.log_info("Random recovery point reached; resuming normal exploration after map refresh");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
      const bool observation = context && context->frontier && context->frontier->observation_goal;
      observation_continuation_reference_.reset();
      if (observation) {
        observation_continuation_reference_=frontier_reference_point(*context->frontier);
        observation_continuation_started_ns_=callbacks.now_ns();
      }
      if (context && context->observation_progress) {
        pending_observation_progress_=context->observation_progress;
        if(params.region_observation_tasks_enabled && map && region_task_.active()) {
          const auto actual=callbacks.get_current_pose();
          if(actual)region_task_.arrived(*map,{actual->position.x,actual->position.y},
            callbacks.now_ns()*1e-9,
              std::atan2(2*(actual->orientation.w*actual->orientation.z+actual->orientation.x*actual->orientation.y),
                1-2*(actual->orientation.y*actual->orientation.y+actual->orientation.z*actual->orientation.z)));
        }
      }
      callbacks.log_info(observation ?
        "Observation point reached; underlying frontier requires fresh map evaluation" :
        "Frontier goal reached");
    } else if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
      if(context && context->completed_by_proximity && context->observation_progress) {
        pending_observation_progress_=context->observation_progress;
        if(params.region_observation_tasks_enabled && map && region_task_.active()) {
          if(const auto actual=callbacks.get_current_pose())
            region_task_.arrived(*map,{actual->position.x,actual->position.y},
              callbacks.now_ns()*1e-9,
              std::atan2(2*(actual->orientation.w*actual->orientation.z+actual->orientation.x*actual->orientation.y),
                1-2*(actual->orientation.y*actual->orientation.y+actual->orientation.z*actual->orientation.z)));
        }
        callbacks.log_info("Frontier completed by proximity; scheduling information-gain evaluation");
      }
      // Expected during explicit preemption/cancel paths.
      callbacks.log_debug(goal_kind + " was canceled before completion");
      if (
        goal_kind == "frontier" &&
        frontier_suppression_ &&
        frontier_suppression_->progress_timeout_cancel_requested())
      {
        if(callbacks.on_navigation_stall) {
          const auto measured=callbacks.get_current_pose();
          if(measured)callbacks.on_navigation_stall(*measured);
        }
        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});
      }
    } else {
      if (goal_kind == "return_to_start") {
        // Failed/aborted return-to-start should permit a future retry.
        return_to_start_started = false;
      } else if (goal_kind == "suppressed_return_to_start") {
        suppressed_return_to_start_started = false;
      }
      if (goal_kind == "frontier") {
        record_failed_frontier_attempt(context.has_value() ? context->frontier : std::optional<FrontierLike>{});
      }
      callbacks.log_warn(
        goal_kind + " finished with status " + detail::status_to_string(status) +
        ", error_code=" + std::to_string(error_code) +
        ", error_msg='" + error_message + "'");
    }
  } else {
    if (dispatch_id != current_dispatch_id) {
      return;
    }

    if (goal_kind == "return_to_start") {
      // Exceptions in result wait path should also reopen return-to-start retry path.
      return_to_start_started = false;
    } else if (goal_kind == "suppressed_return_to_start") {
      suppressed_return_to_start_started = false;
    }
    if(goal_kind=="symmetry_relocation")symmetric_relocation_.finish(callbacks.now_ns()*1e-9);
    callbacks.log_error("Failed while waiting for result: " + exception_text);
  }

  if (dispatch_id != current_dispatch_id) {
    // Ignore stale results from superseded dispatches.
    return;
  }

  cancel_request_in_progress = false;
  // Cancel reason must not leak to the next goal lifecycle.
  pending_cancel_reason.reset();
  clear_active_goal_state();

  if (!exploration_enabled) {
    clear_post_goal_wait_state();
    return;
  }

  if(goal_kind=="symmetry_relocation")pending_frontier_sequence.clear();
  // A queued replacement cannot bypass a regional arrival/observation evaluation.
  if ((!params.region_observation_tasks_enabled || !pending_observation_progress_) &&
      !pending_frontier_sequence.empty()) {
    if (dispatch_pending_frontier_goal()) {
      return;
    }
  }

  if (goal_kind != "frontier" && goal_kind!="random_recovery" && goal_kind!="clearance_recovery" && goal_kind!="symmetry_relocation") {
    clear_post_goal_wait_state();
    return;
  }

  if (params.post_goal_settle_enabled) {
    start_post_goal_settle();
    return;
  }

  clear_post_goal_wait_state();
  try_send_next_goal();
}

void FrontierExplorerCore::feedback_callback(double distance_remaining, int dispatch_id)
{
  if (dispatch_id != current_dispatch_id) {
    // Ignore feedback from stale dispatches.
    return;
  }

  note_active_goal_progress(distance_remaining);

  std::ostringstream oss;
  // Keep distance precision stable to ease visual comparison in debug logs.
  oss << std::fixed << std::setprecision(3)
      << "Distance remaining: " << distance_remaining;
  callbacks.log_debug(oss.str());
}

void FrontierExplorerCore::publish_frontier_markers(const FrontierSequence & frontiers)
{
  const auto signature = frontier_signature(frontiers);
  if (last_published_frontier_signature.has_value() &&
    *last_published_frontier_signature == signature)
  {
    // Skip duplicate publish to reduce RViz/topic noise.
    return;
  }

  callbacks.publish_frontier_markers(frontiers);
  last_published_frontier_signature = signature;
}

void FrontierExplorerCore::request_shutdown()
{
  // Prevent future callback-side state transitions and release current goal handle.
  shutdown_requested = true;
  goal_handle.reset();
}

}  // namespace frontier_exploration_ros2
