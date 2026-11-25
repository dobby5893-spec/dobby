#include <gtest/gtest.h>
#include "frontier_exploration_ros2/active_residual.hpp"
#include "frontier_exploration_ros2/msg/floor_map_snapshot.hpp"
#include <future>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <iostream>
using namespace frontier_exploration_ros2;
using namespace go2_surface_3d_planner;
using Clock=std::chrono::steady_clock;
static std::shared_ptr<PairedFloorMap> scene(int width=50) {
 auto p=std::make_shared<PairedFloorMap>();auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>();
 f->header.frame_id="map";f->info.width=width;f->info.height=30;f->info.resolution=.1;f->info.origin.orientation.w=1.;f->data.assign(width*30,0);
 f->data[15*width+35]=f->data[16*width+35]=-1;
 auto tree=std::make_shared<octomap::OcTree>(.1);
 for(int x=0;x<width;++x)for(int y=0;y<30;++y) {
  tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
  for(int z=0;z<10;++z)if(f->data[y*width+x]>=0)tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,(z+.5)*.1),false);
 }
 p->floor=f;p->tree=tree;p->version={1,1,1,floorDigest(*f)};return p;
}
static ResidualLayout layout(const PairedFloorMap & p) {
 return extractResidualLayout(OccupancyGrid2d(*p.floor),{.5,1.5},{},0,Clock::now()+std::chrono::seconds(5));
}
static SpatialRegion region(const PairedFloorMap & p) {auto l=layout(p);SpatialRegion r;r.region=l.work.front().region;r.frontier=l.work.front().frontier;return r;}
static VisibilitySensor sensor(){VisibilitySensor s;s.z=.2;s.min_pitch=-M_PI/2;s.max_pitch=M_PI/2;return s;}
TEST(ActiveResidual, MultipleActualSafeXYViewsWithoutYawTasks) {
 auto p=scene();auto r=region(*p);SurfaceSearchConfig c;c.infer_flat_support=true;c.snap_radius=.15;
 auto plan=planSpatialRegion(*p,r,{.5,1.5,.21,0},c,sensor(),{},0,5,4,4,1);
 ASSERT_TRUE(plan.selected);EXPECT_TRUE(plan.selected->visible_cells.size()>0);
 std::set<std::pair<long,long>> xy;for(const auto & a:plan.region.attempts)if(a.second.outcome==SpatialOutcome::Ready)xy.insert({std::lround(a.second.pose.x*100),std::lround(a.second.pose.y*100)});
 EXPECT_GE(xy.size(),2u);EXPECT_GE(std::hypot(plan.selected->pose.x-.5,plan.selected->pose.y-1.5),.35);
}
TEST(ActiveResidual, MovingSpatialOriginCanRemoveOcclusion) {
 octomap::OcTree tree(.1);for(int y=-10;y<=10;++y)for(int z=-2;z<10;++z)tree.updateNode(octomap::point3d(2.05,(y+.5)*.1,(z+.5)*.1),true);
 auto s=sensor();EXPECT_FALSE(auditCellVisible(tree,{1,0,0,0},{3,0},0,s));
 EXPECT_TRUE(auditCellVisible(tree,{3.5,1.5,0,0},{3,0},0,s));
}
TEST(ActiveResidual, LidarFovFailureIsNotOcclusionProof) {
 auto p=scene();auto r=region(*p);auto s=sensor();s.min_pitch=.5;s.max_pitch=.6;
 SurfaceSearchConfig c;c.infer_flat_support=true;c.snap_radius=.15;
 auto plan=planSpatialRegion(*p,r,{.5,1.5,.21,0},c,s,{},0,5,4,4,1);
 EXPECT_FALSE(plan.selected);EXPECT_NE(plan.region.classification(180),"verified_enclosed");
}
TEST(ActiveResidual, UnknownToFreeAndOccupiedAreRealVoxelGain) {
 auto p=scene();auto r=region(*p);auto before=captureUnknownVoxels(*p,r.region,.16,.8,Clock::now()+std::chrono::seconds(5),30000);
 ASSERT_GT(before.keys.size(),2u);octomap::OcTree after(*p->tree);after.updateNode(before.keys[0],false);after.updateNode(before.keys[1],true);
 auto gain=measureVoxelGain(before,after);EXPECT_EQ(gain.free_voxels,1u);EXPECT_EQ(gain.occupied_voxels,1u);
 EXPECT_GT(gain.volume,0);EXPECT_LE(gain.union_area,gain.free_area+gain.occupied_area+1e-9);
}
TEST(ActiveResidual, DuplicateCloudsAndMovementHaveZeroMapGain) {
 auto p=scene();auto before=captureUnknownVoxels(*p,region(*p).region,.16,.8,Clock::now()+std::chrono::seconds(5),30000);
 auto gain=measureVoxelGain(before,*p->tree);EXPECT_FALSE(gain.gained());EXPECT_EQ(gain.union_area,0);
}
TEST(ActiveResidual, UnknownKeyWindowIsResourceBoundedAndTruncationUncertain) {
 auto p=scene();auto window=captureUnknownVoxels(*p,region(*p).region,.16,.8,Clock::now()+std::chrono::seconds(5),1);
 EXPECT_EQ(window.keys.size(),1u);EXPECT_FALSE(window.complete);
}
TEST(ActiveResidual, CoarseOctomapCannotBecomeEmptyLoopEnclosureProof) {
 auto p=scene();p->tree=std::make_shared<octomap::OcTree>(1.);
 EXPECT_FALSE(observedClosedFence(*p,region(*p).region,SurfaceSearchConfig{},sensor(),Clock::now()+std::chrono::seconds(5)));
 auto window=captureUnknownVoxels(*p,region(*p).region,.16,.8,Clock::now()+std::chrono::seconds(5),30000);EXPECT_GT(window.keys.size(),0u);
}
TEST(ActiveResidual, NoGainKeepsOtherValidSpatialOptionsEligible) {
 auto p=scene();ActiveResidualBook b;b.sync(layout(*p),*p,{},0,.5);auto id=b.regions.begin()->first;
 auto & r=b.regions[id];r.enumerated=true;SpatialAttempt a;a.cell=1;a.outcome=SpatialOutcome::ObservedNoGain;a.arrived=true;r.attempts[1]=a;
 a.cell=2;a.pose.x=1;a.outcome=SpatialOutcome::Ready;r.attempts[2]=a;
 EXPECT_EQ(b.next(180),id);EXPECT_EQ(b.best(id)->cell,2);EXPECT_EQ(b.audit(180,{.5,1.5}).status(),CompletionStatus::INCOMPLETE);
}
TEST(ActiveResidual, ExhaustedSameMapDoesNotRescheduleSameTimeoutForever) {
 auto p=scene();ActiveResidualBook b;b.sync(layout(*p),*p,{},0,.5);auto & r=b.regions.begin()->second;r.enumerated=true;
 SpatialAttempt a;a.cell=1;a.outcome=SpatialOutcome::Timeout;a.retries=r.retry_limit;r.attempts[1]=a;
 EXPECT_FALSE(b.next(180));EXPECT_EQ(r.classification(180),"verification_deferred");EXPECT_FALSE(b.audit(180,{.5,1.5}).handoff());
}
TEST(ActiveResidual, PartialCursorAndEvidenceSurviveAuditCycles) {
 auto p=scene();ActiveResidualBook b;auto l=layout(*p);b.sync(l,*p,{},0,.5);auto id=b.regions.begin()->first;b.regions[id].cursor=27;b.regions[id].spent=5;
 b.sync(l,*p,{},0,.5);EXPECT_EQ(b.regions[id].cursor,27);EXPECT_EQ(b.regions[id].spent,5);
}
TEST(ActiveResidual, RelatedMapChangesReevaluateHistoricalFailures) {
 auto p=scene();ActiveResidualBook b;auto l=layout(*p);b.sync(l,*p,{},0,.5);auto id=b.regions.begin()->first;b.regions[id].cursor=27;
 auto tree=std::make_shared<octomap::OcTree>(*p->tree);tree->updateNode(octomap::point3d(3,1,.3),true);p->tree=tree;++p->version.octomap_generation;
 b.sync(l,*p,{},0,.5);EXPECT_EQ(b.regions[id].cursor,27);EXPECT_FALSE(b.regions[id].verified_enclosed);EXPECT_FALSE(b.regions[id].enumerated);
}
TEST(ActiveResidual, OneBlockedRegionDoesNotStarveAnother) {
 ActiveResidualBook b;SpatialRegion first,second;first.region.id=1;first.spent=180;second.region.id=2;b.regions={{1,first},{2,second}};
 EXPECT_EQ(b.next(180),2u);EXPECT_EQ(b.next(180),2u);
}
TEST(ActiveResidual, PureOcclusionNoRouteAndNoGainRemainDifferent) {
 ResidualAudit a;a.remaining=a.deferred=1;a.regions={{1,1,"no_route",{}}};EXPECT_EQ(a.status(),CompletionStatus::BLOCKED);
 for(auto state:{"occluded","observed_no_gain","verification_deferred","support_unverified","navigation_failed"}) {
  a.regions[0].state=state;EXPECT_EQ(a.status(),CompletionStatus::INCOMPLETE);EXPECT_FALSE(a.handoff());
 }
}
TEST(ActiveResidual, ExplicitObservedFenceCanSupportResidualCompletion) {
 auto p=scene();auto r=region(*p);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);auto tree=std::make_shared<octomap::OcTree>(*p->tree);
 std::set<int> cells(r.region.cells.begin(),r.region.cells.end());
 for(int i:cells)for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
  int j=i+dy*int(f->info.width)+dx;if(cells.count(j))continue;f->data[j]=100;
  for(int z=0;z<10;++z)tree->updateNode(octomap::point3d((j%f->info.width+.5)*.1,(j/f->info.width+.5)*.1,(z+.5)*.1),true);
 }
 p->floor=f;p->tree=tree;EXPECT_TRUE(observedClosedFence(*p,r.region,SurfaceSearchConfig{},sensor(),Clock::now()+std::chrono::seconds(5)));
 ResidualAudit a;a.remaining=a.deferred=r.region.area;r.region.state="verified_enclosed";a.regions={r.region};EXPECT_EQ(a.status(),CompletionStatus::VERIFIED_WITH_RESIDUALS);
 tree->expand(); // Remove one real full-resolution fence voxel, not a pruned parent.
 ASSERT_NE(tree->search(3.45,1.55,.25),nullptr);
 tree->deleteNode(3.45,1.55,.25); // Return value propagates parent deletion, not leaf success.
 ASSERT_EQ(tree->search(3.45,1.55,.25),nullptr);
 EXPECT_FALSE(observedClosedFence(*p,r.region,SurfaceSearchConfig{},sensor(),Clock::now()+std::chrono::seconds(5)));
}
TEST(ActiveResidual, OpenFenceAndMapInsufficiencyCannotAuthorize) {
 auto p=scene();EXPECT_FALSE(observedClosedFence(*p,region(*p).region,SurfaceSearchConfig{},sensor(),Clock::now()+std::chrono::seconds(5)));
}
TEST(ActiveResidual, ZeroResidualStillCompletes) {ResidualAudit a;EXPECT_EQ(a.status(),CompletionStatus::VERIFIED_COMPLETE);}
TEST(ActiveResidual, BookDoesNotRetainHistoricalTrees) {
 auto p=scene();std::weak_ptr<const octomap::OcTree> weak=p->tree;ActiveResidualBook b;b.sync(layout(*p),*p,{},0,.5);p.reset();EXPECT_TRUE(weak.expired());EXPECT_FALSE(b.regions.empty());
}
TEST(ActiveResidual, SourceMatchedSnapshotReusesDecodedTree) {
 auto p=scene();msg::FloorMapSnapshot message;message.octomap_generation=message.floor_map_generation=message.source_octomap_generation=1;
 message.floor_map=*p->floor;octomap_msgs::fullMapToMsg(*p->tree,message.octomap);message.octomap.header.frame_id="map";
 auto result=decodeFloorSnapshot(message,p->tree,octomapDigest(message.octomap));ASSERT_TRUE(result);EXPECT_EQ(result->tree.get(),p->tree.get());
}

TEST(ActiveResidual, UnrelatedRegionalUpdateRetainsOtherRegionsCursor) {
 auto p=scene(120);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);f->data[15*120+100]=-1;p->floor=f;
 auto l=layout(*p);ASSERT_EQ(l.work.size(),2u);ActiveResidualBook b;b.sync(l,*p,{},0,.5);
 for(auto & r:b.regions){r.second.cursor=13;r.second.scan_remaining=7;}
 auto tree=std::make_shared<octomap::OcTree>(*p->tree);tree->updateNode(octomap::point3d(11.,1.5,.3),true);p->tree=tree;++p->version.octomap_generation;
 b.sync(l,*p,{},0,.5);int retained=0,reset=0;
 for(const auto & r:b.regions){retained+=r.second.cursor==13 && r.second.scan_remaining==7;reset+=r.second.cursor==13 && r.second.scan_remaining==-1;}
 EXPECT_EQ(retained,1);EXPECT_EQ(reset,1);
}
TEST(ActiveResidual, NewMeasuredOriginReevaluatesFailureWithoutErasingRealObservations) {
 ActiveResidualBook b;SpatialRegion r;r.region.id=1;r.enumerated=true;r.route_origin=std::make_pair(0.,0.);
 SpatialAttempt failed;failed.cell=1;failed.outcome=SpatialOutcome::NavigationFailed;r.attempts[1]=failed;
 SpatialAttempt observed;observed.cell=2;observed.outcome=SpatialOutcome::ObservedNoGain;observed.arrived=true;r.attempts[2]=observed;
 b.regions[1]=r;b.changedOrigin(1,0,180);EXPECT_FALSE(b.regions[1].enumerated);EXPECT_FALSE(b.regions[1].attempts.count(1));EXPECT_TRUE(b.regions[1].attempts.count(2));
}

TEST(ActiveResidual, FinishedAsyncPlanDoesNotKeepSourceTreeAlive) {
 auto p=scene();std::weak_ptr<const octomap::OcTree> weak=p->tree;auto r=region(*p);SurfaceSearchConfig c;c.infer_flat_support=true;
 auto future=std::async(std::launch::async,[p,r,c]{return planSpatialRegion(*p,r,{.5,1.5,.21,0},c,sensor(),{},0,5,4,4,1);});
 p.reset();auto completed=future.get();EXPECT_TRUE(weak.expired());EXPECT_FALSE(completed.region.region.cells.empty());
}
TEST(ActiveResidual, SnapshotTransportPayloadHasOneOctomapAndOneFloorGrid) {
 auto p=scene();msg::FloorMapSnapshot snapshot;snapshot.octomap_generation=snapshot.source_octomap_generation=snapshot.floor_map_generation=1;
 snapshot.floor_map=*p->floor;octomap_msgs::fullMapToMsg(*p->tree,snapshot.octomap);snapshot.octomap.header.frame_id="map";
 rclcpp::SerializedMessage map_bytes,snapshot_bytes;rclcpp::Serialization<octomap_msgs::msg::Octomap> map_serializer;
 rclcpp::Serialization<msg::FloorMapSnapshot> snapshot_serializer;map_serializer.serialize_message(&snapshot.octomap,&map_bytes);snapshot_serializer.serialize_message(&snapshot,&snapshot_bytes);
 auto raw=map_bytes.size(),total=snapshot_bytes.size();EXPECT_GT(total,raw);EXPECT_LT(total-raw,p->floor->data.size()+512);
 std::cout<<"snapshot_resource_fixture: octomap_bytes="<<raw<<" snapshot_bytes="<<total<<" extra_bytes="<<total-raw<<std::endl;
}

TEST(ActiveResidual, FreeCenterIsInsufficientWhenBodyOverhangsUnknownOrMapEdge) {
 auto p=scene();OccupancyGrid2d grid(*p->floor);SurfaceSearchConfig config;
 EXPECT_FALSE(knownFreeBodyFootprint(grid,{.05,.05,0,0},config));
 EXPECT_TRUE(knownFreeBodyFootprint(grid,{.5,1.5,0,0},config));
 EXPECT_FALSE(knownFreeBodyFootprint(grid,{3.45,1.55,0,0},config));
}

TEST(ActiveResidual, BudgetYieldResumesNewWorkWithoutResettingCursorOrHistory) {
 ActiveResidualBook b;SpatialRegion r;r.region.id=1;r.spent=180;r.cursor=100;r.executions=1;
 SpatialAttempt a;a.cell=2;a.arrived=true;a.outcome=SpatialOutcome::ObservedNoGain;r.attempts[2]=a;b.regions[1]=r;
 b.recoverBudgets(10,180,60);EXPECT_FALSE(b.next(180));b.recoverBudgets(69,180,60);EXPECT_FALSE(b.next(180));
 b.recoverBudgets(70,180,60);EXPECT_EQ(b.next(180),1u);EXPECT_EQ(b.regions[1].cursor,100);EXPECT_EQ(b.regions[1].attempts.size(),1u);
 b.regions[1].spent=180;b.recoverBudgets(80,180,60);b.recoverBudgets(1000,180,60);EXPECT_FALSE(b.next(180));
}

TEST(ActiveResidual, BoundedHistoryDoesNotBecomeGeometryEvidence) {
 ActiveResidualBook book;SpatialAttempt a;a.arrived=true;a.verified_at=10;
 for(size_t i=0;i<ActiveResidualBook::history_limit+10;++i)book.record(i,a);
 ASSERT_EQ(book.history.size(),ActiveResidualBook::history_limit);
 EXPECT_EQ(book.history.front().region,10u);
 EXPECT_EQ(book.recentVisits(11,60).size(),ActiveResidualBook::history_limit);
 EXPECT_TRUE(book.recentVisits(71,60).empty());
 EXPECT_TRUE(book.regions.empty());
}

TEST(ActiveResidual, CandidateToleratesUnrelatedInventoryButNotSameGenerationConflictOrLocalObstacle) {
 auto p=scene(120);auto e=beginMapEvidence(*p,.5,1.5);sealDependency(e,{1,0,0,5,3,0,0},*p);
 auto floor=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);
 floor->data[100]=-1;p->floor=floor;p->version.floor_digest=floorDigest(*floor);
 EXPECT_FALSE(spatialEvidenceCurrent(e,*p,nullptr));
 ++p->version.floor_map_generation;
 EXPECT_TRUE(spatialEvidenceCurrent(e,*p,nullptr));EXPECT_FALSE(mapEvidenceCurrent(e,*p));
 auto live=std::make_shared<octomap::OcTree>(*p->tree);
 live->updateNode(octomap::point3d(2.,1.,.35),true);
 EXPECT_FALSE(spatialEvidenceCurrent(e,*p,live.get()));
}
TEST(ActiveResidual, SplitAndMergeKeepFactsWithoutTransferringProof) {
 auto p=scene();auto l=layout(*p);ASSERT_EQ(l.work.size(),1u);
 ActiveResidualBook b;b.sync(l,*p,{},0,.5);auto original=l.work.front();
 b.regions.begin()->second.verified_enclosed=true;SpatialAttempt record;record.arrived=true;
 b.record(original.region.id,record);
 auto other=original;other.region.id++;other.region.cells.pop_back();
 l.work.front().region.id+=2;l.work.front().region.cells.erase(l.work.front().region.cells.begin());
 l.work.push_back(other);b.sync(l,*p,{},0,.5);
 ASSERT_EQ(b.regions.size(),2u);for(const auto & r:b.regions)EXPECT_FALSE(r.second.verified_enclosed);
 ASSERT_EQ(b.history.size(),1u);l.work={original};b.sync(l,*p,{},0,.5);
 EXPECT_FALSE(b.regions.at(original.region.id).verified_enclosed);
}
