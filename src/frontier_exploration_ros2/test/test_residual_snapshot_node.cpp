#include <gtest/gtest.h>
#include "frontier_exploration_ros2/frontier_explorer_node.hpp"
namespace frontier_exploration_ros2 {
struct MapConsistencyNodeTestAccess {
 static void run(FrontierExplorerNode & n,int change) {
  auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>();f->header.frame_id="map";f->header.stamp=n.now();
  f->info.width=f->info.height=40;f->info.resolution=.1;f->info.origin.position.x=f->info.origin.position.y=-1.;
  f->info.origin.orientation.w=1.;f->data.assign(1600,0);
  auto tree=std::make_shared<octomap::OcTree>(.1);tree->updateNode(octomap::point3d(0,0,-.05),true);
  auto pair=std::make_shared<PairedFloorMap>();pair->floor=f;pair->tree=tree;pair->version={1,1,123,floorDigest(*f)};
  n.surface_map_=tree;n.surface_hint_map_=f;n.paired_floor_map_=pair;
  n.runtime_state_=FrontierExplorerNode::RuntimeState::RUNNING;n.floor_cleanup_mode_=true;n.residual_audit_epoch_=n.random_epoch_;
  geometry_msgs::msg::TransformStamped tf;tf.header.frame_id=n.params_.global_frame;tf.child_frame_id=n.params_.robot_base_frame;
  tf.header.stamp=n.now();tf.transform.rotation.w=1.;n.tf_buffer_->setTransform(tf,"test",true);
  ResidualAudit result;result.start={0,0};result.epoch=n.random_epoch_;result.complete=false;
  result.map_evidence=beginMapEvidence(*pair,0,0);sealDependency(result.map_evidence,{0,-.5,-.5,.5,.5,0,0},*pair);
  const auto runtime=n.core_->residual_evidence_mask(OccupancyGrid2d(*f),n.now().seconds());
  result.runtime_hash=mapHashSeed();for(auto value:runtime)mapHashMix(result.runtime_hash,uint64_t(value));
  std::promise<ResidualAudit> promise;n.residual_audit_future_=promise.get_future();
  // Mutate the live inputs while the result is still outstanding.
  if(change==1 || change==4) {
   auto newer=std::make_shared<octomap::OcTree>(*tree);newer->updateNode(octomap::point3d(change==1?.1:20.,.1,.4),true);
   n.surface_map_=newer;
  } else {
   auto current=std::make_shared<PairedFloorMap>(*pair);auto floor=std::make_shared<nav_msgs::msg::OccupancyGrid>(*f);
   if(change==2)floor->data[10*40+10]=100;
   if(change==3)floor->info.origin.position.z=3.;
   current->floor=floor;current->version.floor_map_generation++;current->version.floor_digest=floorDigest(*floor);
   n.paired_floor_map_=current;n.surface_hint_map_=floor;
  }
  n.residual_audit_next_=n.now().seconds()+100;
  promise.set_value(result);n.serviceResidualAudit();
 }
 static bool accepted(FrontierExplorerNode & n){return n.residual_audit_cache_.has_value();}
 static bool hasFuture(FrontierExplorerNode & n){return n.residual_audit_future_.valid();}
};
class ResidualSnapshotNode:public ::testing::Test {
protected:
 static void SetUpTestSuite(){int argc=0;rclcpp::init(argc,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_F(ResidualSnapshotNode, Async3dUpdateRejectedByRealService){FrontierExplorerNode n;MapConsistencyNodeTestAccess::run(n,1);EXPECT_FALSE(MapConsistencyNodeTestAccess::accepted(n));EXPECT_FALSE(MapConsistencyNodeTestAccess::hasFuture(n));}
TEST_F(ResidualSnapshotNode, Async2dUpdateRejectedByRealService){FrontierExplorerNode n;MapConsistencyNodeTestAccess::run(n,2);EXPECT_FALSE(MapConsistencyNodeTestAccess::accepted(n));EXPECT_FALSE(MapConsistencyNodeTestAccess::hasFuture(n));}
TEST_F(ResidualSnapshotNode, FloorHeightUpdateRejectedByRealService){FrontierExplorerNode n;MapConsistencyNodeTestAccess::run(n,3);EXPECT_FALSE(MapConsistencyNodeTestAccess::accepted(n));EXPECT_FALSE(MapConsistencyNodeTestAccess::hasFuture(n));}
TEST_F(ResidualSnapshotNode, Unrelated3dUpdateRetainsResultInRealService){FrontierExplorerNode n;MapConsistencyNodeTestAccess::run(n,4);EXPECT_TRUE(MapConsistencyNodeTestAccess::accepted(n));}
}
