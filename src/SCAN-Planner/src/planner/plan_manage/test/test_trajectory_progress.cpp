#include <gtest/gtest.h>
#include "plan_manage/trajectory_progress.hpp"

using namespace scan_planner;

static Eigen::Vector3d corner(double t)
{
  return t <= 5.0 ? Eigen::Vector3d(t * 0.2, 0, 0)
                  : Eigen::Vector3d(1, (t - 5.0) * 0.2, 0);
}

TEST(TrajectoryProgress, StationaryRobotCannotRunPastCorner)
{
  double progress = 0;
  const Eigen::Vector3d actual(0.4, 0, 0);
  for (int cycle = 0; cycle < 3000; ++cycle) {
    progress = projectProgress(corner, actual, progress, 10);
    const double reference = nearbyReference(corner, progress, 10, 0.8);
    EXPECT_LE((corner(reference) - actual).norm(), 0.071);
    EXPECT_DOUBLE_EQ(corner(reference).y(), 0.0);
  }
  EXPECT_NEAR(progress, 2.0, 0.051);
}

TEST(TrajectoryProgress, MovesThroughCornerOnlyAsRobotAdvances)
{
  double progress = 0;
  for (double actual_time = 0; actual_time <= 10; actual_time += 0.1) {
    progress = projectProgress(corner, corner(actual_time), progress, 10);
    EXPECT_NEAR(progress, actual_time, 0.051);
  }
}

TEST(TrajectoryProgress, LookaheadIsBoundedByDistance)
{
  const auto fast = [](double t) { return Eigen::Vector3d(t * 2, 0, 0); };
  EXPECT_LE(fast(nearbyReference(fast, 0, 10, 0.8)).norm(), 0.06);
}

TEST(TrajectoryProgress, DoesNotJumpAcrossHairpin)
{
  const auto hairpin = [](double t) {
    return t <= 5 ? Eigen::Vector3d(t * 0.2, 0, 0)
                  : Eigen::Vector3d(2 - t * 0.2, 0.04, 0);
  };
  const double progress = projectProgress(
    hairpin, Eigen::Vector3d(0.4, 0.035, 0), 0, 10, 100);
  EXPECT_NEAR(progress, 2.0, 0.051);
}

TEST(TrajectoryProgress, ExpiredTrajectoryDoesNotImplyArrival)
{
  EXPECT_FALSE(reachedActualGoal(Eigen::Vector3d(0.4, 0, 0), corner(10), 0.1));
  EXPECT_TRUE(reachedActualGoal(corner(10), corner(10), 0.1));
}
