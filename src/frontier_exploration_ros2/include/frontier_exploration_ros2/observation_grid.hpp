#pragma once
#include <cstdint>
#include <vector>
namespace frontier_exploration_ros2 {
struct ObservationGrid {
  int width{0},height{0};
  double resolution{0},x{0},y{0},yaw{0};
  std::vector<int8_t> cells;
};
}
