#include <gtest/gtest.h>
#define main unused_multifloor_manager_main
#include "../src/multifloor_manager.cpp"
#undef main
namespace go2_multifloor_exploration {
struct MapConsistencyTestAccess {
 static void setup(MultifloorManager & m) {
  auto floor=std::make_shared<nav_msgs::msg::OccupancyGrid>();floor->header.frame_id="map";floor->header.stamp=m.now();
  floor->info.width=floor->info.height=20;floor->info.resolution=.1;floor->info.origin.position.x=floor->info.origin.position.y=-1.;
  floor->info.origin.orientation.w=1;floor->data.assign(400,0);
  auto tree=std::make_shared<octomap::OcTree>(.1);
  for(int x=-8;x<8;++x)for(int y=-8;y<8;++y)
   tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  octomap_msgs::msg::Octomap source;octomap_msgs::fullMapToMsg(*tree,source);source.header.frame_id="map";
  auto pair=std::make_shared<maps::PairedFloorMap>();pair->floor=floor;pair->tree=tree;
  pair->version={1,1,maps::octomapDigest(source),maps::floorDigest(*floor)};pair->source_stamp=m.sim();
  m.paired_floor_map_=pair;m.map_=tree;m.slice_map_=floor;m.map_stamp_=m.now();
  m.state_=MultifloorManager::State::EXPLORE_FLOOR;m.active_floor_=0;m.visited_={0};m.expected_floors_=2;
  auto & session=m.session_;
  session.task_id=maps::newSessionIdentity();session.floor_session_id=maps::newSessionIdentity();
  session.explorer_instance=maps::newSessionIdentity();session.map_producer=maps::newSessionIdentity();
  session.generation=1;session.phase=maps::msg::FloorSession::ACTIVE;
  pair->version.scope={session.task_id,session.floor_session_id,session.map_producer,session.generation};
  session.started_at=floor->header.stamp;session.heartbeat=session.started_at;
  maps::msg::FloorParticipant ack;ack.phase=session.phase;ack.generation=1;
  ack.task_id=session.task_id;ack.floor_session_id=session.floor_session_id;
  ack.role=ack.EXPLORER;ack.instance_id=session.explorer_instance;m.receiveParticipant(ack);
  ack.role=ack.SLICE;ack.instance_id=session.map_producer;m.receiveParticipant(ack);
  pose(m);
  ASSERT_TRUE(m.robot());
 }
 static void pose(MultifloorManager & m,double age=0,double x=0) {
  m.tf_->clear();
  const auto measured_stamp=m.now()-rclcpp::Duration::from_seconds(age);
  for(auto child:{m.body_frame_,m.feet_frame_}) {
   geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id=child;tf.header.stamp=measured_stamp;
   tf.transform.rotation.w=1.;tf.transform.translation.z=child==m.body_frame_?.21:0.;
   tf.transform.translation.x=x;m.tf_->setTransform(tf,"test",false);
  }
 }
 static maps::msg::FloorCompletion fresh(MultifloorManager & m,maps::msg::FloorCompletion report) {
  report.generated_at=m.now();return report;
 }
 static bool safe(MultifloorManager & m) {
  const auto measured=m.robot();return measured && m.safeStandingPose(*measured);
 }
 static bool suspended(MultifloorManager & m){return m.suspended_handoff_.has_value();}
 static auto savedMap(MultifloorManager & m){return m.map_;}
 static void replaceMap(MultifloorManager & m,std::shared_ptr<const octomap::OcTree> tree) {
  m.map_=std::move(tree);m.map_stamp_=m.now();catchUp(m);
 }
 static void contact(MultifloorManager & m) {
  auto tree=std::make_shared<octomap::OcTree>(*m.map_);
  tree->updateNode(octomap::point3d(.1,.0,.35),true);
  replaceMap(m,tree);
 }
 static void unsupported(MultifloorManager & m) {replaceMap(m,std::make_shared<octomap::OcTree>(.1));}
 static void loseLocalization(MultifloorManager & m){m.tf_->clear();}
 static void capacity(MultifloorManager & m){
  for(size_t i=0;i<4096;++i)m.handled_reports_.insert(std::to_string(i));
 }
 static void idleForMotion(MultifloorManager & m) {
  m.setPhase(MultifloorManager::Session::CLOSING);
  m.explorer_.value.phase=MultifloorManager::Session::CLOSING;
  m.idle_handoff_request_=m.pending_handoff_->request_id;
  m.closing_started_at_=m.sim();
 }
 static void assertLateNotificationsCannotReleaseNewOwner(MultifloorManager & m) {
  const auto old=++m.epoch_;
  const auto current=++m.epoch_;
  m.follow_owner_=current;m.rpc_pending_=true;
  m.followGoalResponse(old,nullptr);
  m.releaseFollow(old);
  EXPECT_EQ(m.follow_owner_,current);
  EXPECT_TRUE(m.rpc_pending_);
  m.releaseFollow(current);EXPECT_EQ(m.follow_owner_,0u);
 }
 static void assertUnknownResultDoesNotReleaseOwner(MultifloorManager & m) {
  const auto owner=++m.epoch_;
  m.follow_owner_=owner;m.rpc_pending_=true;
  EXPECT_FALSE(m.followResultIsTerminal(owner,rclcpp_action::ResultCode::UNKNOWN));
  EXPECT_EQ(m.follow_owner_,owner);
  EXPECT_TRUE(waiting(m));
  EXPECT_TRUE(m.followResultIsTerminal(owner,rclcpp_action::ResultCode::CANCELED));
  m.releaseFollow(owner);EXPECT_EQ(m.follow_owner_,0u);
 }
 static bool canFollow(MultifloorManager & m) {
  m.followPath(nav_msgs::msg::Path{},m.map_?maps::wholeOctomapDigest(*m.map_):0);
  return m.follow_owner_!=0;
 }
 static maps::msg::FloorCompletion report(MultifloorManager & m,uint64_t sequence=1) {
  auto e=maps::beginMapEvidence(*m.paired_floor_map_,0,0);maps::sealDependency(e,{0,-.5,-.5,.5,.5,0,0},*m.paired_floor_map_);
  maps::msg::FloorCompletion report;
  report.protocol_version=5;report.task_id=m.session_.task_id;report.floor_session_id=m.session_.floor_session_id;
  report.session_generation=m.session_.generation;report.explorer_instance=m.session_.explorer_instance;
  report.map_producer=m.session_.map_producer;report.report_id=maps::newSessionIdentity();
  report.sequence=sequence;report.generated_at=m.now();report.completion_status=report.VERIFIED_COMPLETE;
  report.safe_start=report.complete_views=report.complete_routes=true;report.stable_samples=3;report.stable_seconds=10;
  maps::setCompletionMapEvidence(report,e);
  return report;
 }
 static void accept(MultifloorManager & m,const maps::msg::FloorCompletion & report){m.acceptFloorCompletion(report);}
 static bool authorizeMotion(MultifloorManager & m){return m.checkBeforeMotion();}
 static void consume(MultifloorManager & m){m.consumeHandoff();}
 static void offsetContext(MultifloorManager & m){++m.context_epoch_;}
 static void rewind(MultifloorManager & m){m.clock_rewound_.store(true);m.simulation_progress_.initialized=true;m.tick();}
 static bool waiting(const MultifloorManager & m){return m.state_==MultifloorManager::State::SESSION_WAIT;}
 static void expire(maps::msg::FloorCompletion & report,MultifloorManager & m){
  m.session_.started_at=m.now()-rclcpp::Duration::from_seconds(20.);
  report.generated_at=m.now()-rclcpp::Duration::from_seconds(6.);
 }
 static bool pending(MultifloorManager & m){return m.pending_handoff_.has_value();}
 static uint64_t request(MultifloorManager & m){return m.handoff_request_id_;}
 static int phase(MultifloorManager & m){return int(m.state_);}
 static void last(MultifloorManager & m){m.expected_floors_=1;}
 static void changedFloor(MultifloorManager & m,bool relevant) {
  auto pair=std::make_shared<maps::PairedFloorMap>(*m.paired_floor_map_);
  auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*pair->floor);f->data[relevant?210:399]=100;
  pair->floor=f;++pair->version.floor_map_generation;pair->version.floor_digest=maps::floorDigest(*f);m.paired_floor_map_=pair;
 }
 static void changed3d(MultifloorManager & m,bool unrelated=false) {
  auto tree=std::make_shared<octomap::OcTree>(*m.map_);tree->updateNode(octomap::point3d(unrelated?20.:.1,.1,.4),true);
  m.map_=tree;m.map_stamp_=m.now();
 }
 static void catchUp(MultifloorManager & m) {
  auto pair=std::make_shared<maps::PairedFloorMap>(*m.paired_floor_map_);pair->tree=m.map_;
  ++pair->version.octomap_generation;++pair->version.floor_map_generation;pair->version.source_digest++;
  pair->source_stamp=m.map_stamp_.seconds();m.paired_floor_map_=pair;
 }
 static void readySearch(MultifloorManager & m) {
  MultifloorManager::SearchResult result;result.stamp=m.now();result.handoff_check=true;
  result.context_epoch=m.context_epoch_;result.handoff_request_id=m.pending_handoff_->request_id;result.source_evidence=m.pending_handoff_->evidence;
  result.source_tree_hash=maps::wholeOctomapDigest(*m.paired_floor_map_->tree);result.selected=StairCandidate{};
  std::promise<MultifloorManager::SearchResult> p;p.set_value(result);m.search_=p.get_future();
 }
 static void collect(MultifloorManager & m){m.collectSearch();}
 static bool completePending(MultifloorManager & m){return m.completion_pending_;}
 static void tick(MultifloorManager & m){m.tick();}
 static void stopping(MultifloorManager & m){m.state_=MultifloorManager::State::STOPPING_FRONTIER;}
};
class HandoffConsistency:public ::testing::Test {
protected:
 static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 void SetUp()override{manager=std::make_unique<MultifloorManager>();MapConsistencyTestAccess::setup(*manager);}
 std::unique_ptr<MultifloorManager> manager;
};
using A=MapConsistencyTestAccess;
TEST_F(HandoffConsistency, MissingMapEvidenceCannotAuthorize){auto wire=A::report(*manager);wire.dependencies.clear();A::accept(*manager,wire);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, IdenticalNotificationDoesNotReplaceRequest){auto wire=A::report(*manager);A::accept(*manager,wire);auto id=A::request(*manager);A::accept(*manager,wire);EXPECT_TRUE(A::pending(*manager));EXPECT_EQ(A::request(*manager),id);}
TEST_F(HandoffConsistency, ChangedFloorBeforeConsumptionRefusesStop){A::last(*manager);A::accept(*manager,A::report(*manager));A::changedFloor(*manager,true);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, Changed3dBeforeConsumptionRefusesStop){A::last(*manager);A::accept(*manager,A::report(*manager));A::changed3d(*manager);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, UnrelatedChangeStillAllowsLastFloorCompletion){A::last(*manager);A::accept(*manager,A::report(*manager));A::changedFloor(*manager,false);A::tick(*manager);EXPECT_EQ(A::phase(*manager),2);}
TEST_F(HandoffConsistency, OldSearchCannotConsumeNewReport){A::accept(*manager,A::report(*manager,1));A::readySearch(*manager);A::accept(*manager,A::report(*manager,2));A::collect(*manager);EXPECT_FALSE(A::completePending(*manager));EXPECT_TRUE(A::pending(*manager));EXPECT_EQ(A::phase(*manager),1);}
TEST_F(HandoffConsistency, MatchingSearchCanAuthorizeStop){A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::collect(*manager);EXPECT_TRUE(A::completePending(*manager));A::tick(*manager);EXPECT_EQ(A::phase(*manager),2);}
TEST_F(HandoffConsistency, MapChangeAfterSearchStillRefusesStop){A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::collect(*manager);ASSERT_TRUE(A::completePending(*manager));A::changed3d(*manager);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);}
TEST_F(HandoffConsistency, UnpairedLiveChangeHoldsRatherThanConsumesAuthorization){
 A::last(*manager);const auto report=A::report(*manager);A::accept(*manager,report);
 ASSERT_TRUE(A::pending(*manager));
 const auto old=A::request(*manager);
 A::changed3d(*manager,true);A::tick(*manager);
 EXPECT_EQ(A::phase(*manager),1);EXPECT_FALSE(A::pending(*manager));EXPECT_TRUE(A::suspended(*manager));
 A::accept(*manager,A::fresh(*manager,report));EXPECT_FALSE(A::pending(*manager));
 A::catchUp(*manager);A::accept(*manager,report);EXPECT_FALSE(A::pending(*manager));
 A::accept(*manager,A::fresh(*manager,report));EXPECT_TRUE(A::pending(*manager));
 EXPECT_GT(A::request(*manager),old);
 A::tick(*manager);EXPECT_EQ(A::phase(*manager),2);
}
TEST_F(HandoffConsistency, ChangedMapBeforeReportDeliveryRejectsReport){auto wire=A::report(*manager);A::changedFloor(*manager,true);A::accept(*manager,wire);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, SearchResultWithChangedLiveTreeCannotComplete){A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::changed3d(*manager,true);A::collect(*manager);EXPECT_FALSE(A::completePending(*manager));}
TEST_F(HandoffConsistency, WrongTaskSessionAndProducerAreRejected){
 auto report=A::report(*manager);
 auto stale=report;stale.task_id=maps::newSessionIdentity();A::accept(*manager,stale);EXPECT_FALSE(A::pending(*manager));
 stale=report;stale.floor_session_id=maps::newSessionIdentity();A::accept(*manager,stale);EXPECT_FALSE(A::pending(*manager));
 stale=report;stale.map_producer=maps::newSessionIdentity();A::accept(*manager,stale);EXPECT_FALSE(A::pending(*manager));
 stale=report;stale.explorer_instance=maps::newSessionIdentity();A::accept(*manager,stale);EXPECT_FALSE(A::pending(*manager));
 A::accept(*manager,report);EXPECT_TRUE(A::pending(*manager));
}
TEST_F(HandoffConsistency, RevocationAndConsumedReportsCannotBeReplayed){
 auto report=A::report(*manager);A::accept(*manager,report);ASSERT_TRUE(A::pending(*manager));
 auto revoked=report;revoked.revoked=true;A::accept(*manager,revoked);EXPECT_FALSE(A::pending(*manager));
 A::accept(*manager,report);EXPECT_FALSE(A::pending(*manager));
 auto next=A::report(*manager,2);A::accept(*manager,next);ASSERT_TRUE(A::pending(*manager));
 A::consume(*manager);A::accept(*manager,next);EXPECT_FALSE(A::pending(*manager));
 A::changed3d(*manager);EXPECT_TRUE(A::authorizeMotion(*manager));
}
TEST_F(HandoffConsistency, InvalidatedEvidenceBeforeMovementBlocksFollowPath){
 A::accept(*manager,A::report(*manager));ASSERT_TRUE(A::pending(*manager));
 A::changed3d(*manager);EXPECT_FALSE(A::authorizeMotion(*manager));EXPECT_FALSE(A::pending(*manager));
}
TEST_F(HandoffConsistency, SearchFromOldContextHasNoEffect){
 A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::offsetContext(*manager);
 A::collect(*manager);EXPECT_FALSE(A::completePending(*manager));
}
TEST_F(HandoffConsistency, ClockRewindClosesCurrentContextAndRejectsReplay){
 auto report=A::report(*manager);A::accept(*manager,report);ASSERT_TRUE(A::pending(*manager));
 A::rewind(*manager);EXPECT_TRUE(A::waiting(*manager));EXPECT_FALSE(A::pending(*manager));
 A::accept(*manager,report);EXPECT_FALSE(A::pending(*manager));
}
TEST_F(HandoffConsistency, ExpiredReportCannotStopLastFloor){
 A::last(*manager);
 auto report=A::report(*manager);A::expire(report,*manager);A::accept(*manager,report);
 EXPECT_FALSE(A::pending(*manager));
 A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);
}
TEST_F(HandoffConsistency, ValidRefreshPreservesRequestButChangedWitnessDoesNotRefresh){
 auto report=A::report(*manager);A::accept(*manager,report);ASSERT_TRUE(A::pending(*manager));
 const auto request=A::request(*manager);
 A::accept(*manager,A::fresh(*manager,report));EXPECT_EQ(A::request(*manager),request);
 A::loseLocalization(*manager);A::tick(*manager);ASSERT_TRUE(A::suspended(*manager));
 A::pose(*manager);
 auto changed=A::fresh(*manager,report);changed.dependencies.front().min_x-=.001;
 A::accept(*manager,changed);EXPECT_FALSE(A::pending(*manager));
 A::accept(*manager,A::fresh(*manager,report));EXPECT_TRUE(A::pending(*manager));
 EXPECT_GT(A::request(*manager),request);
}
TEST_F(HandoffConsistency, RevalidatedReportCannotReuseSuspendedStairSearch){
 auto report=A::report(*manager);A::accept(*manager,report);ASSERT_TRUE(A::pending(*manager));
 A::readySearch(*manager);
 A::loseLocalization(*manager);A::accept(*manager,A::fresh(*manager,report));
 ASSERT_TRUE(A::suspended(*manager));
 A::pose(*manager);A::accept(*manager,A::fresh(*manager,report));ASSERT_TRUE(A::pending(*manager));
 A::collect(*manager);EXPECT_FALSE(A::completePending(*manager));
}
TEST_F(HandoffConsistency, RevokedSuspendedIdAndSupersededIdsArePermanent){
 auto first=A::report(*manager);A::accept(*manager,first);ASSERT_TRUE(A::pending(*manager));
 A::loseLocalization(*manager);A::tick(*manager);ASSERT_TRUE(A::suspended(*manager));
 auto revoke=A::fresh(*manager,first);revoke.revoked=true;A::accept(*manager,revoke);
 A::pose(*manager);A::accept(*manager,A::fresh(*manager,first));
 EXPECT_FALSE(A::pending(*manager));EXPECT_FALSE(A::suspended(*manager));
 auto second=A::report(*manager,2);A::accept(*manager,second);ASSERT_TRUE(A::pending(*manager));
 auto third=A::report(*manager,3);A::accept(*manager,third);const auto request=A::request(*manager);
 A::accept(*manager,A::fresh(*manager,second));EXPECT_EQ(A::request(*manager),request);
 second.sequence=4;A::accept(*manager,A::fresh(*manager,second));EXPECT_EQ(A::request(*manager),request);
}
TEST_F(HandoffConsistency, ReportHistoryHasHardSessionBound){
 A::capacity(*manager);A::accept(*manager,A::report(*manager));EXPECT_FALSE(A::pending(*manager));
}
TEST_F(HandoffConsistency, MeasuredSupportClearanceAndTfMustBeValid){
 ASSERT_TRUE(A::safe(*manager));
 const auto ground=A::savedMap(*manager);
 A::contact(*manager);EXPECT_FALSE(A::safe(*manager));
 A::unsupported(*manager);EXPECT_FALSE(A::safe(*manager));
 A::replaceMap(*manager,ground);EXPECT_TRUE(A::safe(*manager));
 A::pose(*manager,1.);EXPECT_FALSE(A::safe(*manager));
 A::loseLocalization(*manager);EXPECT_FALSE(A::safe(*manager));
}
TEST_F(HandoffConsistency, FirstSendRequiresRealIdleContextAndCurrentStandingPose){
 A::accept(*manager,A::report(*manager));ASSERT_TRUE(A::pending(*manager));
 A::idleForMotion(*manager);
 ASSERT_TRUE(A::authorizeMotion(*manager));
 A::pose(*manager,0,.7);
 EXPECT_FALSE(A::canFollow(*manager));EXPECT_TRUE(A::waiting(*manager));
}
TEST_F(HandoffConsistency, LateGoalNotificationsCannotReleaseCurrentOwner){
 A::assertLateNotificationsCannotReleaseNewOwner(*manager);
}
TEST_F(HandoffConsistency, UnknownResultDoesNotConfirmMotionTerminated){
 A::assertUnknownResultDoesNotReleaseOwner(*manager);
}
TEST_F(HandoffConsistency, NoNotificationsAcceptedWhileStopping){A::stopping(*manager);A::accept(*manager,A::report(*manager));EXPECT_FALSE(A::pending(*manager));}
}
