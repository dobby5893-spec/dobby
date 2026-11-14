#include "frontier_exploration_ros2/frontier_scorer.hpp"

#include <algorithm>
#include <cmath>

namespace frontier_exploration_ros2
{

double compute_frontier_total_score(
  const FrontierCandidate & candidate,
  const FrontierScoreWeights & weights)
{
  const double information_gain = std::clamp(candidate.information_gain, 0.0, 1.0);
  const double topological_gain = std::clamp(candidate.topological_gain, 0.0, 1.0);
  const double enclosure = std::clamp(candidate.enclosure_score, 0.0, 1.0);
  const double open_space = std::clamp(candidate.open_space_score, 0.0, 1.0);
  const double traversability_risk = std::clamp(candidate.traversability_risk, 0.0, 1.0);
  const double path_scale = std::max(0.1, weights.path_cost_scale_m);
  const double safe_path_cost = std::isfinite(candidate.path_cost) ?
    std::max(0.0, candidate.path_cost) : path_scale * 100.0;
  const double normalized_path_cost = safe_path_cost / (safe_path_cost + path_scale);

  return
    weights.information_gain * information_gain +
    weights.topological_gain * topological_gain +
    weights.enclosure * enclosure -
    weights.path_cost * normalized_path_cost -
    weights.open_space * open_space -
    weights.traversability_risk * traversability_risk;
}

void score_frontier_sequence(
  FrontierSequence & frontiers,
  const FrontierScoreWeights & weights)
{
  for (auto & candidate : frontiers) {
    candidate.total_score = compute_frontier_total_score(candidate, weights);
  }
}

}  // namespace frontier_exploration_ros2
