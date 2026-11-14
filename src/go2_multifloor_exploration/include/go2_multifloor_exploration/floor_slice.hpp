#ifndef GO2_MULTIFLOOR_EXPLORATION__FLOOR_SLICE_HPP_
#define GO2_MULTIFLOOR_EXPLORATION__FLOOR_SLICE_HPP_

#include <octomap/OcTree.h>

#include <cstdint>
#include <vector>

namespace go2_multifloor_exploration
{

struct FloorSliceConfig
{
  double resolution{0.10};
  double min_x{-12.0};
  double max_x{12.0};
  double min_y{-9.0};
  double max_y{9.0};
  double support_probe_depth{0.20};
  double obstacle_min_height{0.12};
  double obstacle_max_height{1.00};
  double support_xy_tolerance{0.08};
  double free_space_min_height{0.20};
  double free_space_max_height{0.80};
  int free_space_min_samples{1};
  double enclosed_unknown_max_area{1.0};
  double enclosed_unknown_max_span{1.6};
};

struct FloorSliceGrid
{
  int width{0};
  int height{0};
  double resolution{0.10};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<int8_t> data;
};

// Infer only compact unknown holes with an entirely occupied boundary.
// Eight-connectivity preserves diagonal openings; observed free cells never change.
std::size_t fillEnclosedUnknown(
  FloorSliceGrid & grid, double max_area, double max_span);

class FloorSliceBuilder
{
public:
  explicit FloorSliceBuilder(FloorSliceConfig config);
  FloorSliceGrid build(const octomap::OcTree & tree, double floor_z) const;

private:
  bool occupied(const octomap::OcTree & tree, double x, double y, double z) const;
  bool knownFree(const octomap::OcTree & tree, double x, double y, double z) const;
  bool hasSupport(const octomap::OcTree & tree, double x, double y, double floor_z) const;
  bool hasObservedFreeSpace(
    const octomap::OcTree & tree, double x, double y, double floor_z) const;

  FloorSliceConfig config_;
};

}  // namespace go2_multifloor_exploration

#endif  // GO2_MULTIFLOOR_EXPLORATION__FLOOR_SLICE_HPP_
