#include <gtest/gtest.h>
#include "frontier_exploration_ros2/residual_region_audit.hpp"
#include "frontier_exploration_ros2/msg/floor_map_snapshot.hpp"
#include <future>
using namespace frontier_exploration_ros2;
static std::shared_ptr<const PairedFloorMap> pairMap(int generation=1,int changed=-1,bool three_d=false) {
 msg::FloorMapSnapshot s;s.octomap_generation=s.floor_map_generation=s.source_octomap_generation=generation;
 auto & f=s.floor_map;f.header.frame_id="map";f.info.width=40;f.info.height=20;f.info.resolution=1.;
 f.info.origin.orientation.w=1;f.data.assign(800,0);f.data[5*40+5]=-1;f.data[5*40+30]=-1;
 if(changed>=0)f.data[changed]=100;
 octomap::OcTree tree(.1);tree.updateNode(octomap::point3d(.5,.5,-.05),true);
 if(three_d)tree.updateNode(octomap::point3d(5.,5.,.5),true);
 octomap_msgs::fullMapToMsg(tree,s.octomap);s.octomap.header.frame_id="map";
 return decodeFloorSnapshot(s);
}
static ResidualAudit audited(const PairedFloorMap & p) {
 auto a=auditResidualRegions(OccupancyGrid2d(*p.floor),{.5,.5},{},0,{},
  [](auto,auto){return 0;},std::chrono::steady_clock::now()+std::chrono::seconds(10));
 a.start={.5,.5};sealResidualAudit(a,p,{},0,.5);return a;
}
TEST(MapConsistency, SourceGenerationMismatchRejected) {
 msg::FloorMapSnapshot s;auto pair=pairMap();s.floor_map=*pair->floor;octomap_msgs::fullMapToMsg(*pair->tree,s.octomap);s.octomap.header.frame_id="map";
 s.octomap_generation=2;s.source_octomap_generation=1;s.floor_map_generation=3;
 EXPECT_FALSE(decodeFloorSnapshot(s));
}
TEST(MapConsistency, AtomicSourceSurvivesProducerReplacement) {
 auto old=pairMap();auto a=audited(*old);auto newer=pairMap(2,-1,true);
 EXPECT_NE(wholeOctomapDigest(*old->tree),wholeOctomapDigest(*newer->tree));
 EXPECT_TRUE(mapEvidenceCurrent(a.map_evidence,*old));EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,*newer));
}
TEST(MapConsistency, RoundTripPreservesFullWidthIntegers) {
 auto p=pairMap();auto a=audited(*p);auto encoded=encodeMapEvidence(a.map_evidence);
 auto decoded=decodeMapEvidence(encoded);ASSERT_TRUE(decoded);EXPECT_EQ(decoded->version.source_digest,a.map_evidence.version.source_digest);
 EXPECT_TRUE(mapEvidenceCurrent(*decoded,*p));EXPECT_FALSE(decodeMapEvidence(encoded+"/bad"));
}
TEST(MapConsistency, RelevantFloorChangeDuringAsyncInvalidates) {
 auto old=pairMap();auto future=std::async(std::launch::async,[old]{return audited(*old);});
 auto newer=pairMap(2,5*40+6);auto a=future.get();EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,*newer));
}
TEST(MapConsistency, RelevantOctomapChangeDuringAsyncInvalidates) {
 auto old=pairMap();auto future=std::async(std::launch::async,[old]{return audited(*old);});
 auto newer=pairMap(2,-1,true);auto a=future.get();EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,*newer));
}
TEST(MapConsistency, LiveOctomapAheadOfSliceCannotCertify) {
 auto old=pairMap();auto a=audited(*old);auto newer=pairMap(2,-1,true);
 EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,*old,newer->tree.get()));
}
TEST(MapConsistency, UnrelatedFloorChangePreservesEvidenceAndStability) {
 auto old=pairMap();auto a=audited(*old);auto newer=pairMap(2,19*40+39);
 EXPECT_TRUE(mapEvidenceCurrent(a.map_evidence,*newer));
 for(auto & region:a.regions)region.state="verified_enclosed";
 ResidualAuditStability stability;EXPECT_FALSE(stability.observe(a,0,1));EXPECT_FALSE(stability.observe(a,5,2));
 auto b=audited(*newer);for(auto & region:b.regions)region.state="verified_enclosed";EXPECT_TRUE(stability.observe(b,11,3));
}
TEST(MapConsistency, UnrelatedOctomapChangePreservesEvidence) {
 auto p=pairMap();auto a=audited(*p);octomap::OcTree tree(*p->tree);tree.updateNode(octomap::point3d(100,100,0),true);
 EXPECT_TRUE(mapEvidenceCurrent(a.map_evidence,*p,&tree));
}
TEST(MapConsistency, RelevantChangeInvalidatesOnlyAffectedRegion) {
 auto p=pairMap();auto a=audited(*p);auto n=pairMap(2,-1,true);int affected=0,retained=0;
 for(const auto & d:a.map_evidence.dependencies)if(d.region) {
  if(dependencyUnchanged(d,a.map_evidence,*n))++retained;else ++affected;
 }EXPECT_EQ(affected,1);EXPECT_EQ(retained,1);
}
TEST(MapConsistency, NewUnknownElsewhereInvalidatesCompletionInventory) {
 auto p=pairMap();auto a=audited(*p);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);f->data.back()=-1;
 PairedFloorMap n=*p;n.floor=f;n.version.floor_map_generation++;n.version.floor_digest=floorDigest(*f);
 EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,n));
}
TEST(MapConsistency, ChangedSourceUnderSameGenerationRejected) {
 auto p=pairMap();auto a=audited(*p);PairedFloorMap n=*p;n.version.source_digest++;
 EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,n));
}
TEST(MapConsistency, FloorHeightAndSchemaMismatchRejected) {
 auto p=pairMap();auto a=audited(*p);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);f->info.origin.position.z=3.;
 PairedFloorMap n=*p;n.floor=f;n.version.floor_map_generation++;EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,n));
}
TEST(MapConsistency, ChangedBridgeInvalidatesReachableCandidateMembership) {
 auto p=pairMap();auto a=audited(*p);auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);
 for(int y=0;y<20;++y)f->data[y*40+15]=100;
 PairedFloorMap n=*p;n.floor=f;n.version.floor_map_generation++;n.version.floor_digest=floorDigest(*f);
 EXPECT_FALSE(mapEvidenceCurrent(a.map_evidence,n));
}
TEST(MapConsistency, CachedOcclusionRetainsThreeDimensionalEvidence) {
 auto p=pairMap();auto first=audited(*p);int calls=0;
 auto second=auditResidualRegions(OccupancyGrid2d(*p->floor),{.5,.5},{},0,{},
  [&](auto,auto){++calls;return 0;},std::chrono::steady_clock::now()+std::chrono::seconds(10),{},{},
  [&](const auto & r){return canReuseOccludedRegion(first,r,*p,.5,.5,0,0,0);});
 EXPECT_EQ(calls,0);EXPECT_EQ(second.reused_regions,2u);EXPECT_EQ(second.status(),first.status());
}
TEST(MapConsistency, UnknownAndBlockedRulesRemainUnchanged) {
 ResidualAudit a;a.remaining=a.deferred=1;a.regions={{1,1,"no_route",{}}};EXPECT_EQ(a.status(),CompletionStatus::BLOCKED);
 a.regions[0].state="verification_deferred";EXPECT_EQ(a.status(),CompletionStatus::INCOMPLETE);
}

TEST(MapConsistency, PerRegionCacheReusesOnlyUnchangedEvidence) {
 auto p=pairMap();auto first=audited(*p);
 auto f=std::make_shared<nav_msgs::msg::OccupancyGrid>(*p->floor);f->data[5*40+31]=100;
 PairedFloorMap newer=*p;newer.floor=f;newer.version.floor_map_generation++;newer.version.floor_digest=floorDigest(*f);
 int hits=0,misses=0;
 for(const auto & region:first.regions) {
  if(canReuseOccludedRegion(first,region,newer,.5,.5,0,0,0))++hits;else ++misses;
 }
 EXPECT_EQ(hits,1);EXPECT_EQ(misses,1);
}
TEST(MapConsistency, ChangedSupportContextCannotReuseOcclusion) {
 auto p=pairMap();auto first=audited(*p);ASSERT_FALSE(first.regions.empty());
 EXPECT_FALSE(canReuseOccludedRegion(first,first.regions[0],*p,.5,.5,1,0,0));
}
TEST(MapConsistency, IndependentGenerationsDoNotResetUnchangedRegionalEvidence) {
 auto p=pairMap();auto a=audited(*p);PairedFloorMap n=*p;n.version.octomap_generation+=7;n.version.floor_map_generation+=2;
 EXPECT_TRUE(mapEvidenceCurrent(a.map_evidence,n));
}

TEST(MapConsistency, UnrelatedLiveUpdateRetainsEvidenceButWaitsForMatchingSliceBeforeAuthority) {
 auto p=pairMap();auto a=audited(*p);auto tree=std::make_shared<octomap::OcTree>(*p->tree);
 tree->updateNode(octomap::point3d(100,100,0),true);
 EXPECT_TRUE(mapEvidenceCurrent(a.map_evidence,*p,tree.get()));EXPECT_FALSE(pairedSourceCaughtUp(*p,tree.get()));
 PairedFloorMap next=*p;next.tree=tree;next.version.octomap_generation++;next.version.floor_map_generation++;
 EXPECT_TRUE(mapEvidenceCurrent(a.map_evidence,next,tree.get()));EXPECT_TRUE(pairedSourceCaughtUp(next,tree.get()));
}
