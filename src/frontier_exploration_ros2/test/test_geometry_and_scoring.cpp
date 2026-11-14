#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "frontier_exploration_ros2/frontier_scorer.hpp"
#include "frontier_exploration_ros2/geometry_analyzer.hpp"

namespace frontier_exploration_ros2
{
namespace
{

FrontierCandidate make_candidate()
{
  FrontierCandidate candidate({3.0, 0.0}, {2.0, 0.0}, 20);
  candidate.goal_point = std::pair<double, double>{0.0, 0.0};
  candidate.information_gain = 0.7;
  candidate.path_cost = 3.0;
  candidate.reachable = true;
  return candidate;
}

std::vector<GeometryPoint3d> make_ground()
{
  std::vector<GeometryPoint3d> points;
  for (int x = -8; x <= 8; ++x) {
    for (int y = -8; y <= 8; ++y) {
      points.push_back({0.2 * x, 0.2 * y, 0.0});
    }
  }
  return points;
}

TEST(GeometryAnalyzerTests, OpenGroundProducesHighOpenSpaceScore)
{
  FrontierGeometryAnalyzer analyzer;
  const auto metrics = analyzer.analyze(make_candidate(), make_ground(), 0.0);
  ASSERT_TRUE(metrics.valid);
  EXPECT_GT(metrics.open_space_score, 0.9);
  EXPECT_LT(metrics.enclosure_score, 0.1);
  EXPECT_LT(metrics.traversability_risk, 0.1);
}

TEST(GeometryAnalyzerTests, CorridorWallsAndCeilingIncreaseEnclosure)
{
  auto points = make_ground();
  for (int i = -15; i <= 15; ++i) {
    const double x = 0.2 * i;
    for (int h = 2; h <= 10; ++h) {
      const double z = 0.2 * h;
      points.push_back({x, -1.0, z});
      points.push_back({x, 1.0, z});
    }
  }
  for (int x = -8; x <= 8; ++x) {
    for (int y = -5; y <= 5; ++y) {
      points.push_back({0.3 * x, 0.2 * y, 2.1});
    }
  }

  FrontierGeometryAnalyzer analyzer;
  const auto metrics = analyzer.analyze(make_candidate(), points, 0.0);
  ASSERT_TRUE(metrics.valid);
  EXPECT_GT(metrics.enclosure_score, 0.45);
  EXPECT_LT(metrics.open_space_score, 0.8);
}

TEST(GeometryAnalyzerTests, SparseCloudDoesNotClaimOutdoorOpening)
{
  FrontierGeometryAnalyzer analyzer;
  std::vector<GeometryPoint3d> points{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  const auto metrics = analyzer.analyze(make_candidate(), points, 0.0);
  EXPECT_FALSE(metrics.valid);
  EXPECT_DOUBLE_EQ(metrics.open_space_score, 0.0);
}

TEST(FrontierScorerTests, OpenSpacePenaltyLowersOtherwiseEqualCandidate)
{
  auto indoor = make_candidate();
  indoor.enclosure_score = 0.8;
  indoor.open_space_score = 0.2;
  auto outdoor = make_candidate();
  outdoor.enclosure_score = 0.0;
  outdoor.open_space_score = 1.0;

  const FrontierScoreWeights weights;
  EXPECT_GT(
    compute_frontier_total_score(indoor, weights),
    compute_frontier_total_score(outdoor, weights));
}

TEST(FrontierScorerTests, LargeInformationGainCanKeepOpenCandidateCompetitive)
{
  auto small_indoor = make_candidate();
  small_indoor.information_gain = 0.1;
  small_indoor.enclosure_score = 0.6;
  small_indoor.open_space_score = 0.2;
  auto large_open = make_candidate();
  large_open.information_gain = 1.0;
  large_open.enclosure_score = 0.0;
  large_open.open_space_score = 0.8;

  const FrontierScoreWeights weights;
  EXPECT_GT(
    compute_frontier_total_score(large_open, weights),
    compute_frontier_total_score(small_indoor, weights));
}

}  // namespace
}  // namespace frontier_exploration_ros2
