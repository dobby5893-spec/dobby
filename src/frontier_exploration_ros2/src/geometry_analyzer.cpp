#include "frontier_exploration_ros2/geometry_analyzer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace frontier_exploration_ros2
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

double clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

template<std::size_t N>
double coverage(const std::array<bool, N> & bins)
{
  return static_cast<double>(std::count(bins.begin(), bins.end(), true)) /
         static_cast<double>(N);
}

}  // namespace

FrontierGeometryAnalyzer::FrontierGeometryAnalyzer(GeometryAnalysisConfig config)
: config_(std::move(config))
{
  config_.analysis_radius_m = std::max(0.5, config_.analysis_radius_m);
  config_.local_floor_radius_m = std::clamp(
    config_.local_floor_radius_m, 0.25, config_.analysis_radius_m);
  config_.wall_min_height_m = std::max(0.0, config_.wall_min_height_m);
  config_.wall_max_height_m = std::max(
    config_.wall_min_height_m + 0.1, config_.wall_max_height_m);
  config_.ceiling_min_height_m = std::max(
    config_.wall_min_height_m, config_.ceiling_min_height_m);
  config_.ceiling_max_height_m = std::max(
    config_.ceiling_min_height_m + 0.1, config_.ceiling_max_height_m);
  config_.minimum_nearby_points = std::max<std::size_t>(1U, config_.minimum_nearby_points);
}

GeometryMetrics FrontierGeometryAnalyzer::analyze(
  const FrontierCandidate & candidate,
  const std::vector<GeometryPoint3d> & cloud,
  double reference_floor_z) const
{
  GeometryMetrics result;
  if (cloud.empty()) {
    return result;
  }

  const auto viewpoint = candidate.goal_point.value_or(candidate.center_point);
  const double heading = std::atan2(
    candidate.centroid.second - viewpoint.second,
    candidate.centroid.first - viewpoint.first);
  const double radius_sq = config_.analysis_radius_m * config_.analysis_radius_m;
  const double floor_radius_sq =
    config_.local_floor_radius_m * config_.local_floor_radius_m;

  std::vector<double> local_floor_samples;
  std::size_t nearby_count = 0U;
  for (const auto & point : cloud) {
    const double dx = point.x - viewpoint.first;
    const double dy = point.y - viewpoint.second;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq > radius_sq) {
      continue;
    }
    ++nearby_count;
    if (
      distance_sq <= floor_radius_sq &&
      point.z >= reference_floor_z - 1.5 &&
      point.z <= reference_floor_z + 1.0)
    {
      local_floor_samples.push_back(point.z);
    }
  }

  if (nearby_count < config_.minimum_nearby_points) {
    return result;
  }

  double floor_z = reference_floor_z;
  if (local_floor_samples.size() >= 5U) {
    const std::size_t floor_index = local_floor_samples.size() / 10U;
    std::nth_element(
      local_floor_samples.begin(),
      local_floor_samples.begin() + static_cast<std::ptrdiff_t>(floor_index),
      local_floor_samples.end());
    floor_z = local_floor_samples[floor_index];
  }

  std::array<bool, 12> side_bins{};
  std::array<bool, 12> front_bins{};
  std::array<bool, 16> ceiling_bins{};
  std::vector<double> ground_heights;

  for (const auto & point : cloud) {
    const double dx = point.x - viewpoint.first;
    const double dy = point.y - viewpoint.second;
    const double range = std::hypot(dx, dy);
    if (range > config_.analysis_radius_m || range < 1e-6) {
      continue;
    }

    const double relative_angle = wrap_angle(std::atan2(dy, dx) - heading);
    const double relative_height = point.z - floor_z;
    if (
      range <= config_.local_floor_radius_m &&
      relative_height >= -0.15 && relative_height <= 0.35)
    {
      ground_heights.push_back(point.z);
    }

    if (
      relative_height >= config_.wall_min_height_m &&
      relative_height <= config_.wall_max_height_m)
    {
      if (std::abs(relative_angle) <= kPi / 3.0) {
        const double unit = (relative_angle + kPi / 3.0) / (2.0 * kPi / 3.0);
        const auto index = std::min<std::size_t>(11U, static_cast<std::size_t>(unit * 12.0));
        front_bins[index] = true;
      }
      const double abs_angle = std::abs(relative_angle);
      if (abs_angle >= kPi / 3.0 && abs_angle <= 2.0 * kPi / 3.0) {
        const std::size_t side = relative_angle < 0.0 ? 0U : 6U;
        const double unit = (abs_angle - kPi / 3.0) / (kPi / 3.0);
        const auto index = std::min<std::size_t>(5U, static_cast<std::size_t>(unit * 6.0));
        side_bins[side + index] = true;
      }
    }

    if (
      relative_height >= config_.ceiling_min_height_m &&
      relative_height <= config_.ceiling_max_height_m &&
      std::abs(dx) <= config_.analysis_radius_m &&
      std::abs(dy) <= config_.analysis_radius_m)
    {
      const int x_bin = std::clamp(
        static_cast<int>(((dx / config_.analysis_radius_m) + 1.0) * 2.0), 0, 3);
      const int y_bin = std::clamp(
        static_cast<int>(((dy / config_.analysis_radius_m) + 1.0) * 2.0), 0, 3);
      ceiling_bins[static_cast<std::size_t>(y_bin * 4 + x_bin)] = true;
    }
  }

  const double side_coverage = coverage(side_bins);
  const double front_blockage = coverage(front_bins);
  const double ceiling_coverage = coverage(ceiling_bins);
  result.enclosure_score = clamp01(0.60 * side_coverage + 0.40 * ceiling_coverage);
  result.open_space_score = clamp01(
    1.0 - (0.55 * front_blockage + 0.30 * side_coverage + 0.15 * ceiling_coverage));

  if (ground_heights.size() >= 5U) {
    const auto [minimum, maximum] = std::minmax_element(
      ground_heights.begin(), ground_heights.end());
    result.traversability_risk = clamp01((*maximum - *minimum) / 0.45);
  }
  result.valid = true;
  result.floor_z = floor_z;
  return result;
}

void FrontierGeometryAnalyzer::score_frontiers(
  FrontierSequence & frontiers,
  const std::vector<GeometryPoint3d> & cloud,
  double reference_floor_z) const
{
  for (auto & frontier : frontiers) {
    const GeometryMetrics metrics = analyze(frontier, cloud, reference_floor_z);
    if (!metrics.valid) {
      frontier.enclosure_score = 0.0;
      frontier.open_space_score = 0.0;
      frontier.traversability_risk = 0.0;
      continue;
    }
    frontier.enclosure_score = metrics.enclosure_score;
    frontier.open_space_score = metrics.open_space_score;
    frontier.traversability_risk = metrics.traversability_risk;
    frontier.goal_z = metrics.floor_z;
  }
}

}  // namespace frontier_exploration_ros2
