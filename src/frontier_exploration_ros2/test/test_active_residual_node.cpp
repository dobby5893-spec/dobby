#include <gtest/gtest.h>
#include "frontier_exploration_ros2/frontier_explorer_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <thread>
namespace frontier_exploration_ros2 {
struct ActiveResidualNodeTestAccess {
 using Nav=nav2_msgs::action::NavigateToPose;using Compute=nav2_msgs::action::ComputePathToPose;
 static void pose(FrontierExplorerNode & n,double x,double y) {
  auto stamp=n.now();for(auto child:{n.params_.robot_base_frame,std::string("base_footprint")}) {
   geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.header.stamp=stamp;tf.child_frame_id=child;
   tf.transform.rotation.w=1;tf.transform.translation.x=x;tf.transform.translation.y=y;
   tf.transform.translation.z=child=="base_footprint"?0:.21;n.tf_buffer_->setTransform(tf,"fixture",false);
  }
  geometry_msgs::msg::TransformStamped sensor;sensor.header.frame_id="base_footprint";sensor.header.stamp=stamp;
  sensor.child_frame_id=n.visibility_sensor_frame_;sensor.transform.rotation.w=1;sensor.transform.translation.z=.2;n.tf_buffer_->setTransform(sensor,"fixture",true);
 }
 static void setup(FrontierExplorerNode & n) {
  auto p=std::make_shared<PairedFloorMap>();auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>();
  f->header.frame_id="map";f->header.stamp=n.now();f->info.width=50;f->info.height=30;f->info.resolution=.1;f->info.origin.orientation.w=1;f->data.assign(1500,0);
  f->data[15*50+35]=f->data[16*50+35]=-1;auto tree=std::make_shared<octomap::OcTree>(.1);
  for(int x=0;x<50;++x)for(int y=0;y<30;++y) {
   tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
   for(int z=0;z<10;++z)if(f->data[y*50+x]>=0)tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,(z+.5)*.1),false);
  }
  p->floor=f;p->tree=tree;p->version={1,1,1,floorDigest(*f)};p->source_stamp=n.now().seconds();
  n.paired_floor_map_=p;n.surface_map_=tree;n.surface_hint_map_=f;n.surface_map_time_s_=p->source_stamp;
  n.core_->map=OccupancyGrid2d(*f);n.core_->costmap=OccupancyGrid2d(*f);n.core_->exploration_enabled=true;
  n.floor_cleanup_mode_=false;n.runtime_state_=FrontierExplorerNode::RuntimeState::RUNNING;
  n.active_residual_epoch_=n.random_epoch_;pose(n,.5,1.5);
  auto layout=extractResidualLayout(OccupancyGrid2d(*f),{.5,1.5},{},0,std::chrono::steady_clock::now()+std::chrono::seconds(5));
  n.active_residual_.sync(layout,*p,{},0,.56);
 }
 static bool ready(FrontierExplorerNode & n){return n.residual_compute_client_->action_server_is_ready() && n.navigate_to_pose_client_->action_server_is_ready();}
 static void begin(FrontierExplorerNode & n) {
  auto id=n.active_residual_.regions.begin()->first;VisibilitySensor s;s.z=.2;
  auto plan=planSpatialRegion(*n.paired_floor_map_,n.active_residual_.regions.at(id),{.5,1.5,.21,0},n.surface_goal_config_,s,{},0,5,4,4,n.random_epoch_);
  ASSERT_TRUE(plan.selected);auto selected=*plan.selected;auto evidence=plan.evidence;ASSERT_TRUE(n.active_residual_.apply(std::move(plan)));
  n.floor_cleanup_mode_=true;n.beginResidualNav2Check(selected,id,evidence);
 }
 static int phase(FrontierExplorerNode & n){return int(n.residual_phase_);}
 static SpatialOutcome outcome(FrontierExplorerNode & n){return n.active_residual_.regions.begin()->second.attempts.begin()->second.outcome;}
 static size_t noGainCount(FrontierExplorerNode & n) {
  size_t count=0;for(auto & r:n.active_residual_.regions)for(auto & a:r.second.attempts)count+=a.second.outcome==SpatialOutcome::ObservedNoGain;return count;
 }
 static SpatialAttempt observed(FrontierExplorerNode & n) {
  for(auto & r:n.active_residual_.regions)for(auto & a:r.second.attempts)if(a.second.arrived)return a.second;
  return {};
 }
 static void map(FrontierExplorerNode & n,bool gain=false,int64_t source=-1) {
  auto p=std::make_shared<PairedFloorMap>(*n.paired_floor_map_);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);f->header.stamp=n.now();p->floor=f;
  if(gain && !n.residual_unknown_window_.keys.empty()) {
   auto tree=std::make_shared<octomap::OcTree>(*p->tree);tree->updateNode(n.residual_unknown_window_.keys.front(),false);p->tree=tree;n.surface_map_=tree;
  }
  ++p->version.octomap_generation;++p->version.floor_map_generation;p->source_stamp_ns=source<0?n.now().nanoseconds():source;p->source_stamp=p->source_stamp_ns*1e-9;
  n.paired_floor_map_=p;n.surface_hint_map_=f;
 }
 static void service(FrontierExplorerNode & n){n.serviceActiveResidual();}
 static int64_t lidar(FrontierExplorerNode & n,bool malformed=false) {
  auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();msg->header.frame_id="map";msg->header.stamp=n.tf_buffer_->lookupTransform("map",n.params_.robot_base_frame,tf2::TimePointZero).header.stamp;
  if(!malformed) {
   sensor_msgs::PointCloud2Modifier modifier(*msg);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(1);
   sensor_msgs::PointCloud2Iterator<float> x(*msg,"x"),y(*msg,"y"),z(*msg,"z");*x=1;*y=1;*z=.2;
  }else{msg->width=1;msg->height=1;}
  n.residualCloudCallback(msg);const auto stamp=rclcpp::Time(msg->header.stamp).nanoseconds();
  return n.residual_input_scans_.count(stamp)?stamp:-1;
 }
 static int emptyFrontier(FrontierExplorerNode & n) {
  auto recoveries=std::make_shared<int>(0);
  n.core_->callbacks.frontier_search=[](const auto &,const auto &,const auto &,const auto &,double,bool){return FrontierSearchResult{};};
  n.core_->callbacks.frontier_goal_validation_pending=[](){return false;};n.core_->callbacks.floor_cleanup_data_ready=[](){return true;};
  n.core_->callbacks.request_random_recovery=[recoveries](const auto &,bool)->std::optional<geometry_msgs::msg::PoseStamped>{++*recoveries;return {};};
  n.core_->try_send_next_goal();return *recoveries;
 }
 static void changeObservationSchema(FrontierExplorerNode & n,bool resolution) {
  auto p=std::make_shared<PairedFloorMap>(*n.paired_floor_map_);
  if(resolution){p->tree=std::make_shared<octomap::OcTree>(.2);n.surface_map_=p->tree;}
  else{auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);f->info.origin.position.z=3.;p->floor=f;n.surface_hint_map_=f;}
  ++p->version.octomap_generation;++p->version.floor_map_generation;p->version.floor_digest=floorDigest(*p->floor);n.paired_floor_map_=p;
 }
 static void completeFloor(FrontierExplorerNode & n) {
  auto p=std::make_shared<PairedFloorMap>(*n.paired_floor_map_);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);
  auto tree=std::make_shared<octomap::OcTree>(*p->tree);
  for(int cell=0;cell<int(f->data.size());++cell)if(f->data[cell]<0)for(int z=0;z<10;++z)tree->updateNode(octomap::point3d((cell%50+.5)*.1,(cell/50+.5)*.1,(z+.5)*.1),false);
  f->data.assign(f->data.size(),0);f->header.stamp=n.now();p->floor=f;p->tree=tree;p->version.floor_digest=floorDigest(*f);
  ++p->version.octomap_generation;++p->version.floor_map_generation;p->source_stamp=n.now().seconds();
  n.paired_floor_map_=p;n.surface_map_=tree;n.surface_hint_map_=f;n.core_->map=OccupancyGrid2d(*f);n.core_->costmap=OccupancyGrid2d(*f);
  n.floor_cleanup_mode_=true;
 }
 static msg::FloorSession enableManagedCompletion(FrontierExplorerNode & n) {
  n.managed_floor_sessions_=true;
  auto & session=n.floor_session_;
  session.task_id=newSessionIdentity();session.floor_session_id=newSessionIdentity();
  n.explorer_instance_=newSessionIdentity();session.explorer_instance=n.explorer_instance_;session.map_producer=newSessionIdentity();
  session.generation=1;session.phase=session.ACTIVE;session.started_at=n.now();session.heartbeat=session.started_at;
  n.floor_completion_v5_pub_=n.create_publisher<msg::FloorCompletion>(
    "/explore/floor_completion",rclcpp::QoS(1).reliable().transient_local());
  auto pair=std::make_shared<PairedFloorMap>(*n.paired_floor_map_);
  pair->version.scope={session.task_id,session.floor_session_id,session.map_producer,session.generation};
  n.paired_floor_map_=pair;
  return session;
 }
 static bool cleanup(FrontierExplorerNode & n){return n.floor_cleanup_mode_;}
 static void occupyLiveRoute(FrontierExplorerNode & n) {
  auto tree=std::make_shared<octomap::OcTree>(*n.surface_map_);
  tree->updateNode(octomap::point3d(.6,1.5,.35),true);
  n.surface_map_=tree;
 }
 static void replayBeforeArrival(FrontierExplorerNode & n) {
  const auto stamp=n.residual_scan_after_-1;
  n.residual_input_scans_[stamp]="map";map(n,false,stamp);
 }
 static void expireNavigation(FrontierExplorerNode & n) {
  n.residual_nav_started_=n.now().seconds()-n.params_.floor_cleanup_budget_s-1;
 }
 static bool cancelPending(FrontierExplorerNode & n){return n.residual_cancel_outcome_.has_value();}
 static int dispatch(FrontierExplorerNode & n){return n.residual_dispatch_;}
 static void lateResult(FrontierExplorerNode & n,int id){n.activeResidualNavigationResult(id,4);}
 static int64_t fused(FrontierExplorerNode & n){return n.residual_last_fused_;}
 static void staleMap(FrontierExplorerNode & n,bool expire) {
  auto p=std::make_shared<PairedFloorMap>(*n.paired_floor_map_);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);
  f->header.stamp=rclcpp::Time(n.now().nanoseconds()-5000000000LL);p->floor=f;n.paired_floor_map_=p;n.surface_hint_map_=f;n.floor_cleanup_mode_=true;
  if(expire)n.residual_phase_started_=n.now().seconds()-n.params_.floor_cleanup_budget_s-1;
 }
 static void finishedWorker(FrontierExplorerNode & n){n.active_plan_future_=std::async(std::launch::async,[]{return SpatialPlan{};});}
 static bool workerValid(FrontierExplorerNode & n){return n.active_plan_future_.valid();}
 static void stop(FrontierExplorerNode & n){n.requestStopExplorationRuntime(false,"test stop");}
 static void disableTimersForFinalCheck(FrontierExplorerNode & n){n.floor_cleanup_mode_=false;}
};
class ActiveResidualNode:public ::testing::Test {
protected:
 using A=ActiveResidualNodeTestAccess;using Nav=A::Nav;using Compute=A::Compute;
 static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 void SetUp()override {
  n=std::make_shared<FrontierExplorerNode>(rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("floor_cleanup_enabled",true),rclcpp::Parameter("geometry_scoring_enabled",false)}));
  server=std::make_shared<rclcpp::Node>("active_residual_fake_nav2");A::setup(*n);
  compute=rclcpp_action::create_server<Compute>(server,"/compute_path_to_pose",
   [](auto,const auto &){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
   [](auto){return rclcpp_action::CancelResponse::ACCEPT;},[this](auto handle) {
    EXPECT_EQ(handle->get_goal()->planner_id,"Floor3D");
    auto result=std::make_shared<Compute::Result>();result->path.header.frame_id="map";
    geometry_msgs::msg::PoseStamped start;start.header=handle->get_goal()->goal.header;start.pose.position.x=x;start.pose.position.y=y;start.pose.orientation.w=1;
    result->path.poses={start,handle->get_goal()->goal};
    if(plan_ok)handle->succeed(result);else handle->abort(result);
   });
  nav=rclcpp_action::create_server<Nav>(server,"/navigate_to_pose",
   [](auto,const auto &){return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
   [this](auto){++cancel_calls;return rclcpp_action::CancelResponse::ACCEPT;},[this](auto handle) {
    ++navigation_calls;
    if(hold_nav){held=handle;return;}
    if(move_actual){x=handle->get_goal()->pose.pose.position.x;y=handle->get_goal()->pose.pose.position.y;A::pose(*n,x,y);}
    auto result=std::make_shared<Nav::Result>();if(nav_ok)handle->succeed(result);else handle->abort(result);
   });
  executor.add_node(n);executor.add_node(server);wait([&]{return A::ready(*n);});
 }
 void TearDown()override{executor.remove_node(n);executor.remove_node(server);nav.reset();compute.reset();n.reset();server.reset();}
 void wait(std::function<bool()> condition) {
  auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(12);
  while(!condition() && std::chrono::steady_clock::now()<deadline){A::pose(*n,x,y);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  ASSERT_TRUE(condition());
 }
 void arrive(){A::begin(*n);wait([&]{return A::phase(*n)==3;});ASSERT_EQ(navigation_calls,1);}
 void baseline(){wait([&]{auto stamp=A::lidar(*n);if(stamp>0)A::map(*n,false,stamp);A::service(*n);return A::phase(*n)==4;});ASSERT_EQ(A::phase(*n),4);}
 std::shared_ptr<FrontierExplorerNode> n;std::shared_ptr<rclcpp::Node> server;
 rclcpp::executors::SingleThreadedExecutor executor;
 rclcpp_action::Server<Nav>::SharedPtr nav;rclcpp_action::Server<Compute>::SharedPtr compute;
 std::shared_ptr<rclcpp_action::ServerGoalHandle<Nav>> held;
 bool plan_ok=true,nav_ok=true,move_actual=true,hold_nav=false;int navigation_calls=0,cancel_calls=0;double x=.5,y=1.5;
};
TEST_F(ActiveResidualNode, RealNav2PlanThenNavigationArrivalAndVoxelGain) {
 arrive();baseline();A::pose(*n,x,y);const auto stamp=A::lidar(*n);ASSERT_GT(stamp,0);A::map(*n,true,stamp);A::service(*n);
 ASSERT_EQ(A::phase(*n),0);auto observed=A::observed(*n);EXPECT_TRUE(observed.nav_checked);EXPECT_TRUE(observed.fresh_cloud);
 EXPECT_EQ(observed.outcome,SpatialOutcome::ObservedGain);EXPECT_GT(observed.free_voxels,0u);A::disableTimersForFinalCheck(*n);
}
TEST_F(ActiveResidualNode, Nav2FailureCannotBecomeNoRouteCompletion) {
 plan_ok=false;A::begin(*n);wait([&]{return A::phase(*n)==0;});EXPECT_EQ(navigation_calls,0);EXPECT_EQ(A::outcome(*n),SpatialOutcome::NavigationFailed);
}
TEST_F(ActiveResidualNode, ActionSuccessWithoutMeasuredMovementCannotObserve) {
 move_actual=false;A::begin(*n);wait([&]{return A::phase(*n)==0;});EXPECT_EQ(A::outcome(*n),SpatialOutcome::LocalizationInvalid);
}
TEST_F(ActiveResidualNode, NoFreshLidarKeepsObservationPending) {
 arrive();baseline();A::map(*n,true);A::service(*n);EXPECT_EQ(A::phase(*n),4);EXPECT_FALSE(A::observed(*n).fresh_cloud);
}
TEST_F(ActiveResidualNode, MapOlderThanPostArrivalCloudDoesNotCountGain) {
 arrive();baseline();A::pose(*n,x,y);auto stamp=A::lidar(*n);ASSERT_GT(stamp,0);A::map(*n,true,stamp-10000000);A::service(*n);EXPECT_EQ(A::phase(*n),4);
}
TEST_F(ActiveResidualNode, NoGainDoesNotFinishBeforeExistingObservationWindow) {
 arrive();baseline();const auto stamp=A::lidar(*n);if(stamp>0)A::map(*n,false,stamp);A::service(*n);EXPECT_EQ(A::phase(*n),4);
}
TEST_F(ActiveResidualNode, RealNoGainWindowContinuesToAnotherSpatialNavigationGoal) {
 arrive();baseline();const double first_x=x,first_y=y;
 auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(12);
 while(navigation_calls<2 && std::chrono::steady_clock::now()<deadline) {
  A::pose(*n,x,y);const auto stamp=A::lidar(*n);if(stamp>0)A::map(*n,false,stamp);A::service(*n);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(30));
 }
 EXPECT_GE(A::noGainCount(*n),1u);EXPECT_GE(navigation_calls,2);EXPECT_GT(std::hypot(x-first_x,y-first_y),.35);
 A::disableTimersForFinalCheck(*n);
}
TEST_F(ActiveResidualNode, StopPreventsLatePlanFromLaunchingNavigation) {
 A::begin(*n);A::stop(*n);auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(150);
 while(std::chrono::steady_clock::now()<until){executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
 EXPECT_EQ(navigation_calls,0);EXPECT_EQ(A::phase(*n),0);
}
TEST_F(ActiveResidualNode, EmptyFrontierRequestsActiveObservationBeforeRandomRecovery) {
 EXPECT_EQ(A::emptyFrontier(*n),0);EXPECT_TRUE(A::cleanup(*n));
}
TEST_F(ActiveResidualNode, ResolvedUnknownsUseUnifiedStableCompletionPublisher) {
 std::vector<std::string> formal;
 std::vector<msg::FloorCompletion> typed;
 const auto qos=rclcpp::QoS(1).reliable().transient_local();
 auto sub=server->create_subscription<std_msgs::msg::String>("/explore/floor_completion_legacy",qos,
  [&](std_msgs::msg::String::ConstSharedPtr msg){formal.push_back(msg->data);});
 auto v5=server->create_subscription<msg::FloorCompletion>("/explore/floor_completion",qos,
  [&](msg::FloorCompletion::ConstSharedPtr msg){typed.push_back(*msg);});
 A::completeFloor(*n);const auto session=A::enableManagedCompletion(*n);
 auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(13);
 while((formal.empty() || typed.size()<2) && std::chrono::steady_clock::now()<deadline) {
  A::pose(*n,x,y);A::map(*n);A::service(*n);executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(50));
 }
 ASSERT_FALSE(formal.empty());auto parsed=parseVerifiedFloorCompletion(formal.back());ASSERT_TRUE(parsed);EXPECT_EQ(parsed->status,CompletionStatus::VERIFIED_COMPLETE);
 EXPECT_TRUE(completionMapEvidence(formal.back()));
 ASSERT_GE(typed.size(),2u);
 EXPECT_TRUE(completionIdentity(typed.back(),session,n->now().nanoseconds()));
 EXPECT_TRUE(verifyFloorCompletion(typed.back()));
 EXPECT_TRUE(completionMapEvidence(typed.back()));
 EXPECT_EQ(typed.front().report_id,typed.back().report_id);
 EXPECT_EQ(typed.front().sequence,typed.back().sequence);
 EXPECT_EQ(typed.front().dependencies,typed.back().dependencies);
 EXPECT_EQ(typed.front().start_x,typed.back().start_x);
 EXPECT_EQ(typed.front().start_y,typed.back().start_y);
 A::stop(*n);
 wait([&]{return typed.back().revoked;});
 EXPECT_EQ(typed.front().report_id,typed.back().report_id);
}
TEST_F(ActiveResidualNode, ChangedFloorSchemaDuringObservationRejectsOldWindow) {
 arrive();baseline();A::changeObservationSchema(*n,false);A::service(*n);EXPECT_EQ(A::phase(*n),0);EXPECT_EQ(A::observed(*n).outcome,SpatialOutcome::StaleMap);
}
TEST_F(ActiveResidualNode, ChangedOctomapResolutionCannotBecomeFalseNoGain) {
 arrive();baseline();A::changeObservationSchema(*n,true);A::service(*n);EXPECT_EQ(A::phase(*n),0);EXPECT_EQ(A::observed(*n).outcome,SpatialOutcome::StaleMap);
}
TEST_F(ActiveResidualNode, StopDrainsCompletedAsyncWorkWithoutRestart) {
 A::finishedWorker(*n);A::stop(*n);wait([&]{return !A::workerValid(*n);});EXPECT_EQ(A::phase(*n),0);
}
TEST_F(ActiveResidualNode, StaleMapCannotPreventObservationTimeoutFromReleasingOwnership) {
 arrive();baseline();A::staleMap(*n,true);A::service(*n);EXPECT_EQ(A::phase(*n),0);EXPECT_EQ(A::observed(*n).outcome,SpatialOutcome::Timeout);
}
TEST_F(ActiveResidualNode, StaleMapStillDrainsCompletedWorkerBuffers) {
 A::finishedWorker(*n);A::staleMap(*n,false);std::this_thread::sleep_for(std::chrono::milliseconds(10));A::service(*n);EXPECT_FALSE(A::workerValid(*n));
}
TEST_F(ActiveResidualNode, MalformedCloudCannotSupplyFreshObservationEvidence) {EXPECT_LT(A::lidar(*n,true),0);}
} // namespace frontier_exploration_ros2

namespace frontier_exploration_ros2 {
TEST_F(ActiveResidualNode, TimeoutRequestsCancelAndHoldsUntilTerminal) {
 hold_nav=true;A::begin(*n);wait([&]{return bool(held);});const auto old=A::dispatch(*n);
 A::expireNavigation(*n);A::service(*n);
 wait([&]{return held->is_canceling();});
 EXPECT_TRUE(A::cancelPending(*n));EXPECT_EQ(A::phase(*n),2);
 A::service(*n);EXPECT_EQ(navigation_calls,1);
 held->canceled(std::make_shared<Nav::Result>());wait([&]{return A::phase(*n)==0;});
 EXPECT_EQ(A::outcome(*n),SpatialOutcome::Timeout);
 A::lateResult(*n,old);EXPECT_EQ(A::phase(*n),0);
}
TEST_F(ActiveResidualNode, LateSuccessAfterTimeoutCannotBeginObservation) {
 hold_nav=true;A::begin(*n);wait([&]{return bool(held);});
 A::expireNavigation(*n);A::service(*n);
 // Simulate success racing ahead of the cancel request on the action server.
 held->succeed(std::make_shared<Nav::Result>());wait([&]{return A::phase(*n)==0;});
 EXPECT_EQ(A::outcome(*n),SpatialOutcome::Timeout);
}
TEST_F(ActiveResidualNode, FloorChangeCancelsNavigationAndWaitsForTerminal) {
 hold_nav=true;A::begin(*n);wait([&]{return bool(held);});
 A::changeObservationSchema(*n,false);A::service(*n);
 wait([&]{return held->is_canceling();});EXPECT_EQ(A::phase(*n),2);
 held->canceled(std::make_shared<Nav::Result>());wait([&]{return A::phase(*n)==0;});
 EXPECT_EQ(A::outcome(*n),SpatialOutcome::StaleMap);
}
TEST_F(ActiveResidualNode, StopCancelsExecutingResidualNavigation) {
 hold_nav=true;A::begin(*n);wait([&]{return bool(held);});A::stop(*n);
 wait([&]{return held->is_canceling();});EXPECT_TRUE(A::cancelPending(*n));
 held->canceled(std::make_shared<Nav::Result>());wait([&]{return A::phase(*n)==0;});
 EXPECT_EQ(navigation_calls,1);
}
TEST_F(ActiveResidualNode, DuplicateOctomapStampCannotCreditNewGain) {
 arrive();baseline();
 A::pose(*n,x,y);const auto stamp=A::lidar(*n);ASSERT_GT(stamp,0);
 A::map(*n,false,stamp);A::service(*n);EXPECT_EQ(A::fused(*n),stamp);
 A::map(*n,true,stamp);A::service(*n);EXPECT_EQ(A::phase(*n),4);
 EXPECT_EQ(A::fused(*n),stamp);
}
TEST_F(ActiveResidualNode, NewerUnmatchedOctomapCannotCreditGain) {
 arrive();baseline();A::pose(*n,x,y);const auto stamp=A::lidar(*n);ASSERT_GT(stamp,0);
 A::map(*n,true,stamp+1);A::service(*n);EXPECT_EQ(A::phase(*n),4);
 EXPECT_NE(A::fused(*n),stamp+1);
}
}

namespace frontier_exploration_ros2 {
TEST_F(ActiveResidualNode, ObstacleAddedDuringNav2PlanningRejectsOldCandidate) {
 A::begin(*n);A::occupyLiveRoute(*n);
 wait([&]{return A::phase(*n)==0;});
 EXPECT_EQ(navigation_calls,0);EXPECT_EQ(A::outcome(*n),SpatialOutcome::StaleMap);
}
TEST_F(ActiveResidualNode, PreArrivalSourceCannotEstablishBaseline) {
 arrive();A::replayBeforeArrival(*n);A::service(*n);EXPECT_EQ(A::phase(*n),3);
}
}
