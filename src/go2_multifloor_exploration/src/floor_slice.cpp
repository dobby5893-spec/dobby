#include "go2_multifloor_exploration/floor_slice.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace go2_multifloor_exploration
{

std::size_t fillEnclosedUnknown(FloorSliceGrid & grid, double max_area, double max_span)
{
  if (max_area <= 0.0 || max_span <= 0.0 || grid.resolution <= 0.0 ||
    grid.width <= 0 || grid.height <= 0 ||
    grid.data.size() != static_cast<std::size_t>(grid.width) * grid.height)
  {return 0;}
  std::vector<bool> visited(grid.data.size(), false);
  std::vector<std::size_t> component;
  std::size_t filled = 0;
  for (std::size_t seed = 0; seed < grid.data.size(); ++seed) {
    if (visited[seed] || grid.data[seed] >= 0) {continue;}
    component.clear();
    component.push_back(seed); visited[seed] = true;
    bool enclosed = true;
    int min_x = grid.width, max_x = 0, min_y = grid.height, max_y = 0;
    for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
      const int x = static_cast<int>(component[cursor] % grid.width);
      const int y = static_cast<int>(component[cursor] / grid.width);
      min_x = std::min(min_x, x); max_x = std::max(max_x, x);
      min_y = std::min(min_y, y); max_y = std::max(max_y, y);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {continue;}
          const int nx = x + dx, ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= grid.width || ny >= grid.height) {
            enclosed = false; continue;
          }
          const auto next = static_cast<std::size_t>(ny) * grid.width + nx;
          if (grid.data[next] < 0) {
            if (!visited[next]) {visited[next] = true; component.push_back(next);}
          } else if (grid.data[next] != 100) {
            enclosed = false;
          }
        }
      }
    }
    const double area = component.size() * grid.resolution * grid.resolution;
    const double span = std::max(max_x - min_x + 1, max_y - min_y + 1) * grid.resolution;
    if (enclosed && area <= max_area + 1e-9 && span <= max_span + 1e-9) {
      for (const auto index : component) {grid.data[index] = 100;}
      filled += component.size();
    }
  }
  return filled;
}

FloorSliceBuilder::FloorSliceBuilder(FloorSliceConfig config)
: config_(std::move(config))
{
  if (config_.resolution <= 0.0 || config_.max_x <= config_.min_x ||
    config_.max_y <= config_.min_y)
  {
    throw std::invalid_argument("Invalid floor slice geometry");
  }
  config_.support_probe_depth = std::max(config_.resolution, config_.support_probe_depth);
  config_.obstacle_min_height = std::max(0.01, config_.obstacle_min_height);
  config_.obstacle_max_height = std::max(
    config_.obstacle_min_height, config_.obstacle_max_height);
  config_.support_xy_tolerance = std::max(0.0, config_.support_xy_tolerance);
  config_.free_space_min_height = std::max(0.01, config_.free_space_min_height);
  config_.free_space_max_height = std::max(
    config_.free_space_min_height, config_.free_space_max_height);
  config_.free_space_min_samples = std::max(1, config_.free_space_min_samples);
}

bool FloorSliceBuilder::occupied(
  const octomap::OcTree & tree, double x, double y, double z) const
{
  const auto * node = tree.search(x, y, z);
  return node != nullptr && tree.isNodeOccupied(node);
}

bool FloorSliceBuilder::knownFree(
  const octomap::OcTree & tree, double x, double y, double z) const
{
  const auto * node = tree.search(x, y, z);
  return node != nullptr && !tree.isNodeOccupied(node);
}

bool FloorSliceBuilder::hasSupport(
  const octomap::OcTree & tree, double x, double y, double floor_z) const
{
  const double t = config_.support_xy_tolerance;
  const std::array<std::pair<double, double>, 5> offsets{{
    {0.0, 0.0}, {t, 0.0}, {-t, 0.0}, {0.0, t}, {0.0, -t}}};
  const double dz = std::max(tree.getResolution(), 0.05);
  for (const auto & offset : offsets) {
    for (double depth = 0.5 * tree.getResolution();
      depth <= config_.support_probe_depth + 0.5 * tree.getResolution(); depth += dz)
    {
      if (occupied(tree, x + offset.first, y + offset.second, floor_z - depth)) {
        return true;
      }
    }
  }
  return false;
}

bool FloorSliceBuilder::hasObservedFreeSpace(
  const octomap::OcTree & tree, double x, double y, double floor_z) const
{
  const double dz = std::max(tree.getResolution(), 0.05);
  int free_samples = 0;
  for (double height = config_.free_space_min_height;
    height <= config_.free_space_max_height + 1e-9; height += dz)
  {
    if (knownFree(tree, x, y, floor_z + height) &&
      ++free_samples >= config_.free_space_min_samples)
    {
      return true;
    }
  }
  return false;
}

FloorSliceGrid FloorSliceBuilder::build(const octomap::OcTree & tree, double floor_z) const
{
  FloorSliceGrid grid;
  grid.resolution = config_.resolution;
  grid.origin_x = config_.min_x;
  grid.origin_y = config_.min_y;
  grid.width = static_cast<int>(std::ceil((config_.max_x - config_.min_x) / grid.resolution));
  grid.height = static_cast<int>(std::ceil((config_.max_y - config_.min_y) / grid.resolution));
  grid.data.assign(static_cast<std::size_t>(grid.width) * grid.height, -1);

  // Rasterize occupied leaf volumes into every output cell they overlap.
  // Sampling only a 10 cm cell centre can miss a 5 cm wall voxel entirely.
  constexpr double epsilon = 1e-9;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (!tree.isNodeOccupied(*it)) {continue;}
    const double half = it.getSize() * .5;
    if (it.getZ() + half <= floor_z + config_.obstacle_min_height ||
      it.getZ() - half >= floor_z + config_.obstacle_max_height) {continue;}
    const int min_x = std::max(0, static_cast<int>(std::floor(
      (it.getX() - half - grid.origin_x + epsilon) / grid.resolution)));
    const int max_x = std::min(grid.width - 1, static_cast<int>(std::ceil(
      (it.getX() + half - grid.origin_x - epsilon) / grid.resolution)) - 1);
    const int min_y = std::max(0, static_cast<int>(std::floor(
      (it.getY() - half - grid.origin_y + epsilon) / grid.resolution)));
    const int max_y = std::min(grid.height - 1, static_cast<int>(std::ceil(
      (it.getY() + half - grid.origin_y - epsilon) / grid.resolution)) - 1);
    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {grid.data[y * grid.width + x] = 100;}
    }
  }
  for (int y = 0; y < grid.height; ++y) {
    const double wy = grid.origin_y + (static_cast<double>(y) + 0.5) * grid.resolution;
    for (int x = 0; x < grid.width; ++x) {
      const double wx = grid.origin_x + (static_cast<double>(x) + 0.5) * grid.resolution;
      const std::size_t index = static_cast<std::size_t>(y) * grid.width + x;
      if (grid.data[index] == 100) {continue;}
      if (hasSupport(tree, wx, wy, floor_z) ||
        hasObservedFreeSpace(tree, wx, wy, floor_z))
      {
        grid.data[index] = 0;
      }
    }
  }
  fillEnclosedUnknown(grid, config_.enclosed_unknown_max_area, config_.enclosed_unknown_max_span);
  return grid;
}

}  // namespace go2_multifloor_exploration
