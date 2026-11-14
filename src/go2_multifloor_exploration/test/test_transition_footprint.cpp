
#include "go2_multifloor_exploration/transition_footprint.hpp"
#include "go2_multifloor_exploration/simulation_progress.hpp"
#include <gtest/gtest.h>
using namespace go2_multifloor_exploration;
TEST(TransitionFootprint, MarksObservedConnectedStepsWithoutSwallowingAdjacentRoom) {
 octomap::OcTree tree(.05);StairCandidate stair;
 for(int x=0;x<8;++x) {
  double z=.10+x*.05;stair.chain.push_back({(x+.5)*.1,0,z,0});
  for(int y=-3;y<=3;++y)
   tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,z-.025),true);
 }
 // A floor-level room touching the stair footprint is not a stair.
 for(int x=0;x<8;++x)for(int y=4;y<10;++y)
  tree.updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.025),true);
 tree.updateInnerOccupancy();
 auto mask=transitionFootprint(tree,{stair},30,30,.1,-1,-1,0,.12,
   std::chrono::steady_clock::now()+std::chrono::seconds(2));
 ASSERT_TRUE(mask.complete);EXPECT_GT(mask.marked,0U);
 for(int y=14;y<20;++y)for(int x=10;x<18;++x)EXPECT_EQ(mask.cells[y*30+x],0);
 // Unobserved continuation and a disconnected elevated object stay unclassified.
 EXPECT_EQ(mask.cells[10*30+24],0);
}
TEST(TransitionFootprint, MissingStairEvidenceAndTimeoutNeverFillSpace) {
 octomap::OcTree tree(.05);
 auto empty=transitionFootprint(tree,{},10,10,.1,0,0,0,.12,std::chrono::steady_clock::now());
 EXPECT_TRUE(empty.complete);EXPECT_EQ(empty.marked,0U);
 StairCandidate c;c.chain.push_back({0,0,.1,0});
 auto expired=transitionFootprint(tree,{c},10,10,.1,0,0,0,.12,
  std::chrono::steady_clock::now()-std::chrono::seconds(1));
 EXPECT_FALSE(expired.complete);EXPECT_EQ(expired.marked,0U);
}
TEST(SimulationProgress, PauseFreezesFailuresAndExtendsRpcDeadlines) {
 SimulationProgress p;EXPECT_TRUE(p.sample(10,0));
 EXPECT_FALSE(p.sample(10,1));EXPECT_DOUBLE_EQ(p.extend_deadlines,1);
 EXPECT_FALSE(p.sample(10,30));EXPECT_DOUBLE_EQ(p.extend_deadlines,29);
 EXPECT_TRUE(p.sample(11,31));EXPECT_DOUBLE_EQ(p.extend_deadlines,1);
 EXPECT_TRUE(p.sample(12,32));EXPECT_DOUBLE_EQ(p.extend_deadlines,0);
 EXPECT_TRUE(p.sample(0,33));
}

TEST(SimulationProgress, NormalClockSamplingGapsDoNotDisableRpcTimeouts) {
 SimulationProgress p;EXPECT_TRUE(p.sample(10,0));
 for(int i=1;i<20;++i) {
   p.sample(10+(i/2)*.01,i*.05);
   EXPECT_DOUBLE_EQ(p.extend_deadlines,0);EXPECT_FALSE(p.paused);
 }
}
