#include "scan_planner_nav2_controller/pose_path_tracker.hpp"
#include <gtest/gtest.h>

using scan_planner_nav2_controller::PosePathTracker;
constexpr double pi = 3.14159265358979323846;

TEST(PosePathTracker, BlockedTurnCanFirstCorrectToItsPlannedAnchor)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0},{.1,0,0,0},{.1,0,0,pi/2},{.1,.2,0,pi/2}});
  tracker.command({0,0,.21,0},.2,.35,1.0);
  tracker.command({.065,0,.21,0},.2,.35,1.0);
  const auto correction=tracker.turnPositionCorrection({.065,0,.21,0},.03,1.0);
  EXPECT_GT(correction.vx,0);EXPECT_LE(correction.vx,.03+1e-9);
  EXPECT_NEAR(correction.vy,0,1e-9);EXPECT_NEAR(correction.wz,0,1e-9);
  EXPECT_NEAR(tracker.turnPositionCorrection({.1,0,.21,0},.03,1).vx,0,1e-9);
}

TEST(PosePathTracker, DoesNotTurnEarlyAtDoor)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0}, {0.1,0,0,0}, {0.1,0,0,pi/2}, {0.1,0.1,0,pi/2}});
  const auto approach = tracker.command({0,0,0.21,0}, 0.2, 0.35, 1.0);
  EXPECT_GT(approach.vx, 0);
  EXPECT_NEAR(approach.wz, 0, 1e-9);
  const auto rotate = tracker.command({0.1,0,0.21,0}, 0.2, 0.35, 1.0);
  EXPECT_NEAR(rotate.vx, 0, 1e-9);
  EXPECT_NEAR(rotate.vy, 0, 1e-9);
  EXPECT_GT(rotate.wz, 0);
  const auto enter = tracker.command({0.1,0,0.21,pi/2}, 0.2, 0.35, 1.0);
  EXPECT_GT(enter.vx, 0);
  EXPECT_NEAR(enter.vy, 0, 1e-9);
}

TEST(PosePathTracker, SmallGridConnectorDoesNotTrapGaitInCentimetreSettling)
{
  PosePathTracker tracker;
  tracker.setPlan({{-0.04,-0.10,-0.20,0.0},{0.0,-0.10,-0.20,0.0},
    {0.0,-0.10,-0.20,0.196},{-0.1,-0.10,-0.20,0.196}});
  const auto cmd=tracker.command({-0.037,-0.106,0.15,0.019},0.2,0.35,1.0);
  EXPECT_EQ(tracker.index(),2U);
  EXPECT_GT(cmd.wz,0.0);
  EXPECT_GT(cmd.vx,0.0);
  EXPECT_LE(std::hypot(cmd.vx,cmd.vy),.06);
}

TEST(PosePathTracker, SidewaysPathKeepsPlannedYaw)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,pi/2}, {0.1,0,0,pi/2}, {0.2,0,0,pi/2}});
  const auto cmd = tracker.command({0,0,0.21,pi/2}, 0.2, 0.35, 1);
  EXPECT_LT(cmd.vy, 0);
  EXPECT_NEAR(cmd.vx, 0, 1e-9);
  EXPECT_NEAR(cmd.wz, 0, 1e-9);
}

TEST(PosePathTracker, DoesNotAdvanceWithTimeOrRepeatedSetPlan)
{
  PosePathTracker tracker;
  const std::vector<scan_planner_nav2_controller::PoseReference> path{
    {0,0,0,0}, {0.1,0,0,0}, {0.1,0.1,0,pi/2}};
  tracker.setPlan(path);
  for (int i = 0; i < 100; ++i) {
    tracker.setPlan(path);
    const auto cmd = tracker.command({0,0,0.21,0}, 0.2, 0.35, 1.0);
    EXPECT_NEAR(cmd.wz, 0, 1e-9);
    EXPECT_EQ(tracker.index(), 1U);
  }
}

TEST(PosePathTracker, StairProgressUsesMeasuredHeight)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0}, {0.1,0,0.2,0}, {0.2,0,0.3,0}});
  tracker.command({0,0,0.21,0}, 0.2, 0.35, 1.0);
  tracker.command({0.1,0,0.21,0}, 0.2, 0.35, 1.0);
  EXPECT_EQ(tracker.index(), 1U);
  tracker.command({0.1,0,0.41,0}, 0.2, 0.35, 1.0);
  EXPECT_EQ(tracker.index(), 2U);
}

TEST(PosePathTracker, RecordedDoorApproachCompletesHeadingBeforeLongStraight)
{
  PosePathTracker tracker;
  tracker.setPlan({{4.3,-2.0,0,-pi*3/16},{4.3,-2.1,0,-pi/4},
    {4.3,-2.2,0,-pi/4},{4.3,-2.8,0,-pi/4}});
  tracker.command({4.3,-2.0,.21,-pi*3/16},.2,.35,1.0);
  const auto align=tracker.command({4.25388,-2.07268,.21,-.709831},.2,.35,1.0);
  EXPECT_EQ(tracker.index(),1U);
  EXPECT_LT(align.wz,-.06);
  const double world_x=std::cos(-.709831)*align.vx-std::sin(-.709831)*align.vy;
  const double world_y=std::sin(-.709831)*align.vx+std::cos(-.709831)*align.vy;
  EXPECT_GT(world_x,0.0);
  EXPECT_LT(world_y,0.0);
  EXPECT_LE(std::hypot(align.vx,align.vy),.06);
  const auto advance=tracker.command({4.25388,-2.07268,.21,-pi/4},.2,.35,1.0);
  EXPECT_EQ(tracker.index(),3U);
  EXPECT_NEAR(std::hypot(advance.vx,advance.vy),.2,1e-9);
}

TEST(PosePathTracker, CrossTrackErrorOutsideGateDoesNotSkipDoorCorner)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0},{.1,0,0,0},{.1,0,0,pi/2},{.1,.5,0,pi/2}});
  tracker.command({0,0,.21,0},.2,.35,1.0);
  const auto cmd=tracker.command({.1,.08,.21,0},.2,.35,1.0);
  EXPECT_EQ(tracker.index(),1U);
  EXPECT_NEAR(cmd.wz,0.0,1e-9);
}

TEST(PosePathTracker, CurvedEdgeTracksHeadingWithoutWaitingForLag)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0},{.1,0,0,.196}});
  tracker.command({0,0,.21,0},.2,.35,1.0);
  const auto cmd=tracker.command({.03,0,.21,.0588},.2,.35,1.0);
  EXPECT_NEAR(std::hypot(cmd.vx,cmd.vy),.07,1e-9);
  EXPECT_NEAR(cmd.wz,.1372,1e-9);
  EXPECT_LE(cmd.wz,.196-.0588);
}

TEST(PosePathTracker, CurvedEdgeCanCorrectHeadingWhenEndpointHeadingWasReachedEarly)
{
  for(double sign:{-1.,1.}) {
    PosePathTracker tracker;
    tracker.setPlan({{0,0,0,0},{1,0,0,sign*.4}});
    tracker.command({0,0,.21,0},.2,.35,1.);
    const auto command=tracker.command({.2,0,.21,sign*.4},.2,.35,1.);
    EXPECT_LT(sign*command.wz,-.1);
    EXPECT_NEAR(std::hypot(command.vx,command.vy),0,1e-9);
    EXPECT_EQ(tracker.index(),1U);
  }
}

TEST(PosePathTracker, ClockwiseCurvedEdgeTracksHeading)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0},{.1,0,0,-.196}});
  tracker.command({0,0,.21,0},.2,.35,1.0);
  const auto cmd=tracker.command({.03,0,.21,-.0588},.2,.35,1.0);
  EXPECT_NEAR(cmd.wz,-.1372,1e-9);
}

TEST(PosePathTracker, RotationCorrectsGaitDriftEvenOutsideCheckpointGate)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0},{0,0,0,pi/2},{0,.1,0,pi/2}});
  tracker.command({0,0,.21,0},.2,.35,1.0);
  const auto cmd=tracker.command({-.08,.02,.21,.7},.2,.35,1.0);
  const double wx=std::cos(.7)*cmd.vx-std::sin(.7)*cmd.vy;
  const double wy=std::sin(.7)*cmd.vx+std::cos(.7)*cmd.vy;
  EXPECT_GT(wx,0.0);
  EXPECT_LT(wy,0.0);
  EXPECT_GT(cmd.wz,0.0);
  EXPECT_LE(std::hypot(cmd.vx,cmd.vy),.06);
  EXPECT_EQ(tracker.index(),1U);
}

TEST(PosePathTracker, TurnAtCheckpointHoldsCornerPosition)
{
  PosePathTracker tracker;
  tracker.setPlan({{0,0,0,0},{.1,0,0,.196}});
  tracker.command({0,0,.21,0},.2,.35,1.0);
  const auto cmd=tracker.command({.07,.02,.21,.12},.2,.35,1.0);
  EXPECT_GT(std::cos(.12)*cmd.vx-std::sin(.12)*cmd.vy,0.0);
  EXPECT_LT(std::sin(.12)*cmd.vx+std::cos(.12)*cmd.vy,0.0);
  EXPECT_GT(cmd.wz,0.0);
}

TEST(TurnProgress, RepeatedYawJitterCannotKeepAStalledStageAlive) {
  PosePathTracker t;t.setPlan({{0,0,0,0},{0,0,0,.5},{.5,0,0,.5}});
  for(int i=0;i<8;++i) {
    scan_planner_nav2_controller::PoseReference m{0,0,.21,i%2?.02:-.02};
    t.command(m,.2,.35,1);EXPECT_FALSE(t.turnStalled(m,i));
  }
  scan_planner_nav2_controller::PoseReference m{0,0,.21,0};
  t.command(m,.2,.35,1);EXPECT_TRUE(t.turnStalled(m,8));
}
TEST(TurnProgress, RealReductionInTargetErrorKeepsSlowRotationAlive) {
  PosePathTracker t;t.setPlan({{0,0,0,0},{0,0,0,1.5}});
  for(int i=0;i<20;++i) {
    scan_planner_nav2_controller::PoseReference m{0,0,.21,i*.06};
    t.command(m,.2,.35,1);EXPECT_FALSE(t.turnStalled(m,i));
  }
}
TEST(TurnProgress, NewStageAndClockResetDoNotInheritAnExpiredTimer) {
  PosePathTracker t;t.setPlan({{0,0,0,0},{0,0,0,.5},{0,0,0,1}});
  t.command({0,0,.21,0},.2,.35,1);EXPECT_FALSE(t.turnStalled({0,0,.21,0},0));
  EXPECT_TRUE(t.turnStalled({0,0,.21,0},9));
  t.command({0,0,.21,.5},.2,.35,1);EXPECT_FALSE(t.turnStalled({0,0,.21,.5},10));
  EXPECT_FALSE(t.turnStalled({0,0,.21,.5},0));
}
TEST(TurnProgress, OrdinaryTranslationDoesNotBecomeATurnTimeout) {
  PosePathTracker t;t.setPlan({{0,0,0,0},{1,0,0,0}});
  for(int i=0;i<20;++i) {
    scan_planner_nav2_controller::PoseReference m{.1,0,.21,0};
    t.command(m,.2,.35,1);EXPECT_FALSE(t.turnStalled(m,i));
  }
}

#include "scan_planner_nav2_controller/motion_speed_limit.hpp"
TEST(MotionSpeedLimit, FlatIncreaseDoesNotIncreaseStairSpeed) {
 using scan_planner_nav2_controller::motionSpeedLimit;
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.30,1.,false),.30);
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.30,1.,true),.20);
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.30,.5,true),.10);
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.06,1.,true),.06);
}
TEST(MotionSpeedLimit, NavSpeedReductionAndStopArePreserved) {
 using scan_planner_nav2_controller::motionSpeedLimit;
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.30,0.,false),0.);
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.30,.5,false),.15);
 EXPECT_DOUBLE_EQ(motionSpeedLimit(.30,2.,false),.30);
}

TEST(PosePathTracker, LongFlatLegCanUseRaisedCeilingWithoutSkippingTurns) {
 PosePathTracker tracker;
 tracker.setPlan({{0,0,0,0},{.1,0,0,0},{.2,0,0,0},{1.,0,0,0},{1.,0,0,pi/2}});
 auto fast=tracker.command({0,0,.21,0},.30,.35,1.);
 EXPECT_NEAR(fast.vx,.30,1e-9);EXPECT_NEAR(fast.vy,0.,1e-9);
 EXPECT_LT(tracker.index(),4u);
 auto near=tracker.command({.97,0,.21,0},.30,.35,1.);
 EXPECT_LE(std::hypot(near.vx,near.vy),.06+1e-9);
 EXPECT_LE(std::abs(near.wz),.35+1e-9);
}
