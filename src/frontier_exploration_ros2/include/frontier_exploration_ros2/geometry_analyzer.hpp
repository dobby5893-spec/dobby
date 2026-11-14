#pragma once

#include <cstddef>
#include <vector>

#include "frontier_exploration_ros2/frontier_types.hpp"

namespace frontier_exploration_ros2
{

struct GeometryPoint3d
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GeometryAnalysisConfig
{
  double analysis_radius_m{4.0};
  double local_floor_radius_m{1.25};
  double wall_min_height_m{0.25};
  double wall_max_height_m{2.0};
  double ceiling_min_height_m{1.6};
  double ceiling_max_height_m{3.2};
  std::size_t minimum_nearby_points{30U};
};

struct GeometryMetrics
{
  bool valid{false};
  double enclosure_score{0.0};
  double open_space_score{0.0};
  double traversability_risk{0.0};
  double floor_z{0.0};
};

class FrontierGeometryAnalyzer
{
public:
  explicit FrontierGeometryAnalyzer(GeometryAnalysisConfig config = {});

  [[nodiscard]] GeometryMetrics analyze(
    const FrontierCandidate & candidate,
    const std::vector<GeometryPoint3d> & cloud,
    double reference_floor_z) const;

  void score_frontiers(
    FrontierSequence & frontiers,
    const std::vector<GeometryPoint3d> & cloud,
    double reference_floor_z) const;

private:
  GeometryAnalysisConfig config_;
};

}  // namespace frontier_exploration_ros2
