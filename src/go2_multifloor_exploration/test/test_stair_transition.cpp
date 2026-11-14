#include "go2_multifloor_exploration/stair_transition.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace go2_multifloor_exploration;
using go2_surface_3d_planner::SurfaceGraph;
namespace {
void box(octomap::OcTree & tree,double xmin,double xmax,double ymin,double ymax,double top) {
 for(int ix=std::floor(xmin/.05);ix<std::ceil(xmax/.05);++ix)
  for(int iy=std::floor(ymin/.05);iy<std::ceil(ymax/.05);++iy)
   for(int iz=-2;iz<static_cast<int>(std::round(top/.05));++iz)
    tree.updateNode(octomap::point3d((ix+.5)*.05,(iy+.5)*.05,(iz+.5)*.05),true);
}
SurfaceSearchConfig config() {
 SurfaceSearchConfig c;c.xy_resolution=.1;c.snap_radius=.15;c.clearance_margin=.01;
 c.search_deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);return c;
}
void stairs(octomap::OcTree & tree,double width=1.6) {
 box(tree,-2,0,-2,2,0);
 for(int i=0;i<6;++i) box(tree,i*.25,(i+1)*.25,-width/2,width/2,(i+1)*.05);
}
}
TEST(StairDetection, VisiblePrefixDoesNotRequireACompleteUpperFloor) {
 octomap::OcTree tree(.05);stairs(tree);auto c=config();
 auto result=detectStairs(tree,c,{-.8,0,0,0},StairOptions{});
 ASSERT_FALSE(result.candidates.empty());
 auto entry=result.candidates.front();
 EXPECT_GE(entry.chain.size(),3u);EXPECT_GE(entry.target.z,.14);
 EXPECT_LT(entry.approach.x,entry.first_tread.x);EXPECT_LE(entry.target.z,.35);
 auto path=verifyStairApproach(tree,c,{-.8,0,0,0},entry);
 EXPECT_FALSE(path.empty());
}
TEST(StairDetection, TableAndIsolatedBoxAreNotStaircases) {
 octomap::OcTree tree(.05);box(tree,-2,2,-2,2,0);box(tree,0,1,-.7,.7,.75);
 EXPECT_TRUE(detectStairs(tree,config(),{-1,0,0,0},StairOptions{}).candidates.empty());
 octomap::OcTree low(.05);box(low,-2,2,-2,2,0);box(low,0,1,-.7,.7,.10);
 EXPECT_TRUE(detectStairs(low,config(),{-1,0,0,0},StairOptions{}).candidates.empty());
}
TEST(StairDetection, RejectsTreadsNarrowerThanRequiredSupportWidth) {
 octomap::OcTree tree(.05);stairs(tree,.20);
 EXPECT_TRUE(detectStairs(tree,config(),{-.8,0,0,0},StairOptions{}).candidates.empty());
}
TEST(StairDetection, SearchDeadlineIsBounded) {
 octomap::OcTree tree(.05);stairs(tree);auto c=config();c.search_deadline=std::chrono::steady_clock::now();
 auto result=detectStairs(tree,c,{-.8,0,0,0},StairOptions{});
 EXPECT_TRUE(result.timed_out);EXPECT_TRUE(result.candidates.empty());
}
TEST(StairLanding, NarrowTurnaroundIsNotCommittedAsANewFloor) {
 octomap::OcTree narrow(.05);box(narrow,-.5,.5,-2,2,1.2);
 EXPECT_FALSE(hasLandingPatch(narrow,config(),{0,0,1.2,0},.7));
 octomap::OcTree broad(.05);box(broad,-1.5,1.5,-1.5,1.5,2.4);
 EXPECT_TRUE(hasLandingPatch(broad,config(),{0,0,2.4,0},.7));
}
TEST(StairLanding, ActualHeightMustMatchEvenWhenXYHasArrived) {
 EXPECT_FALSE(reachedStairTarget({1,1,.1,0},{1,1,.4,0}));
 EXPECT_TRUE(reachedStairTarget({1.1,1,.36,0},{1,1,.4,0}));
 EXPECT_FALSE(reachedStairTarget({std::numeric_limits<double>::quiet_NaN(),1,.4,0},{1,1,.4,0}));
}
TEST(StairRegion, FlatPathsReserveFullBodyWhileStairPathsCanUseObservedTreads) {
 octomap::OcTree tree(.05);stairs(tree);auto c=config();
 c.excluded_regions.push_back({.5,0,0,.6,.6,-.1,.6});
 EXPECT_FALSE(SurfaceGraph(tree,c).isWalkableSurface({.12,0,.05,0}));
 c.excluded_regions.clear();EXPECT_TRUE(SurfaceGraph(tree,c).isWalkableSurface({.12,0,.05,0}));
 EXPECT_THROW(go2_surface_3d_planner::decodeSurfaceRegions({1,2}),std::invalid_argument);
 EXPECT_THROW(go2_surface_3d_planner::decodeSurfaceRegions({0,0,0,-1,1,0,1}),std::invalid_argument);
}
TEST(StairApproach, CannotApproachAnEntryThroughAnEnclosingWall) {
 octomap::OcTree tree(.05);stairs(tree);box(tree,-.5,-.35,-2,2,1.0);
 StairCandidate entry;entry.approach={-.1,0,0,0};
 auto c=config();c.max_iterations=5000;
 EXPECT_TRUE(verifyStairApproach(tree,c,{-1,0,0,0},entry).empty());
}
