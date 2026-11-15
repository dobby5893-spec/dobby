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
  auto tree=std::make_shared<octomap::OcTree>(.1);tree->updateNode(octomap::point3d(0,0,-.05),true);
  octomap_msgs::msg::Octomap source;octomap_msgs::fullMapToMsg(*tree,source);source.header.frame_id="map";
  auto pair=std::make_shared<maps::PairedFloorMap>();pair->floor=floor;pair->tree=tree;
  pair->version={1,1,maps::octomapDigest(source),maps::floorDigest(*floor)};pair->source_stamp=m.sim();
  m.paired_floor_map_=pair;m.map_=tree;m.slice_map_=floor;m.map_stamp_=m.now();
  m.state_=MultifloorManager::State::EXPLORE_FLOOR;m.active_floor_=0;m.visited_={0};m.expected_floors_=2;
  const auto measured_stamp=m.now();
  for(auto child:{m.body_frame_,m.feet_frame_}) {
   geometry_msgs::msg::TransformStamped tf;tf.header.frame_id="map";tf.child_frame_id=child;tf.header.stamp=measured_stamp;
   tf.transform.rotation.w=1.;tf.transform.translation.z=child==m.body_frame_?.21:0.;m.tf_->setTransform(tf,"test",false);
  }
  ASSERT_TRUE(m.robot());
 }
 static std::string report(MultifloorManager & m,int nonce=0) {
  auto e=maps::beginMapEvidence(*m.paired_floor_map_,0,0);maps::sealDependency(e,{0,-.5,-.5,.5,.5,0,0},*m.paired_floor_map_);
  std::ostringstream s;s<<std::setprecision(17)<<"floor_completion;authority=joint_residual_audit;status=VERIFIED_COMPLETE;floor_z=0;stamp="<<m.sim()
   <<";remaining=0;residual=0;transition=0;region_count=0;regions=;safe_start=1;complete_views=1;complete_routes=1;stable_samples=3;stable_seconds=10;map_evidence="
   <<maps::encodeMapEvidence(e)<<";test_nonce="<<nonce;return s.str();
 }
 static void accept(MultifloorManager & m,const std::string & report){m.acceptFloorCompletion(report);}
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
  result.handoff_request_id=m.pending_handoff_->request_id;result.source_evidence=m.pending_handoff_->evidence;
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
TEST_F(HandoffConsistency, MissingMapEvidenceCannotAuthorize){auto wire=A::report(*manager);wire=wire.substr(0,wire.find(";map_evidence="));A::accept(*manager,wire);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, IdenticalNotificationDoesNotReplaceRequest){auto wire=A::report(*manager);A::accept(*manager,wire);auto id=A::request(*manager);A::accept(*manager,wire);EXPECT_TRUE(A::pending(*manager));EXPECT_EQ(A::request(*manager),id);}
TEST_F(HandoffConsistency, ChangedFloorBeforeConsumptionRefusesStop){A::last(*manager);A::accept(*manager,A::report(*manager));A::changedFloor(*manager,true);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, Changed3dBeforeConsumptionRefusesStop){A::last(*manager);A::accept(*manager,A::report(*manager));A::changed3d(*manager);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, UnrelatedChangeStillAllowsLastFloorCompletion){A::last(*manager);A::accept(*manager,A::report(*manager));A::changedFloor(*manager,false);A::tick(*manager);EXPECT_EQ(A::phase(*manager),2);}
TEST_F(HandoffConsistency, OldSearchCannotConsumeNewReport){A::accept(*manager,A::report(*manager,1));A::readySearch(*manager);A::accept(*manager,A::report(*manager,2));A::collect(*manager);EXPECT_FALSE(A::completePending(*manager));EXPECT_TRUE(A::pending(*manager));EXPECT_EQ(A::phase(*manager),1);}
TEST_F(HandoffConsistency, MatchingSearchCanAuthorizeStop){A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::collect(*manager);EXPECT_TRUE(A::completePending(*manager));A::tick(*manager);EXPECT_EQ(A::phase(*manager),2);}
TEST_F(HandoffConsistency, MapChangeAfterSearchStillRefusesStop){A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::collect(*manager);ASSERT_TRUE(A::completePending(*manager));A::changed3d(*manager);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);}
TEST_F(HandoffConsistency, UnpairedLiveChangeHoldsRatherThanConsumesAuthorization){A::last(*manager);A::accept(*manager,A::report(*manager));A::changed3d(*manager,true);A::tick(*manager);EXPECT_EQ(A::phase(*manager),1);EXPECT_TRUE(A::pending(*manager));A::catchUp(*manager);A::tick(*manager);EXPECT_EQ(A::phase(*manager),2);}
TEST_F(HandoffConsistency, ChangedMapBeforeReportDeliveryRejectsReport){auto wire=A::report(*manager);A::changedFloor(*manager,true);A::accept(*manager,wire);EXPECT_FALSE(A::pending(*manager));}
TEST_F(HandoffConsistency, SearchResultWithChangedLiveTreeCannotComplete){A::accept(*manager,A::report(*manager));A::readySearch(*manager);A::changed3d(*manager,true);A::collect(*manager);EXPECT_FALSE(A::completePending(*manager));}
TEST_F(HandoffConsistency, NoNotificationsAcceptedWhileStopping){A::stopping(*manager);A::accept(*manager,A::report(*manager));EXPECT_FALSE(A::pending(*manager));}
}
