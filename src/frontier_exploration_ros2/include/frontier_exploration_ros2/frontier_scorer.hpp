#pragma once

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

struct FrontierScoreWeights
{
  double information_gain{2.0};
  double topological_gain{1.5};
  double enclosure{0.8};
  double path_cost{0.8};
  double open_space{1.0};
  double traversability_risk{0.6};
  double path_cost_scale_m{5.0};
};

double compute_frontier_total_score(
  const FrontierCandidate & candidate,
  const FrontierScoreWeights & weights);

void score_frontier_sequence(
  FrontierSequence & frontiers,
  const FrontierScoreWeights & weights);

}  // namespace frontier_exploration_ros2
