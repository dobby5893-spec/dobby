#include "go2_nav2_recovery/footprint_safety.hpp"

#include <cmath>
#include <stdexcept>

using go2_nav2_recovery::assessFootprint;
using go2_nav2_recovery::clearance;

void check(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int main()
{
  nav2_costmap_2d::Costmap2D map(100, 100, 0.02, -1.0, -1.0, 0);
  std::vector<geometry_msgs::msg::Point> footprint(4);
  footprint[0].x = 0.28; footprint[0].y = 0.18;
  footprint[1].x = 0.28; footprint[1].y = -0.18;
  footprint[2].x = -0.28; footprint[2].y = -0.18;
  footprint[3].x = -0.28; footprint[3].y = 0.18;

  check(clearance(map, footprint, 0.0, 0.0, 0.0) > 0.0, "free room rejected");

  unsigned int x;
  unsigned int y;
  map.worldToMap(0.0, 0.0, x, y);
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  check(clearance(map, footprint, 0.0, 0.0, 0.0) < 0.0, "lethal cell inside body missed");
  const auto overlap = assessFootprint(map, footprint, 0.0, 0.0, 0.0);
  check(!overlap.collisionFree(), "overlapping footprint reported collision-free");
  check(overlap.hard_overlap_cells == 1, "hard overlap cell count is incorrect");
  check(overlap.first_hard_x == static_cast<int>(x) &&
    overlap.first_hard_y == static_cast<int>(y) &&
    overlap.first_hard_cost == nav2_costmap_2d::LETHAL_OBSTACLE,
    "collision diagnostic identifies the wrong hard cell");
  check(
    assessFootprint(map, footprint, 0.60, 0.0, 0.0).hard_overlap_cells == 0,
    "moving away did not clear hard overlap count");
  map.setCost(x, y, 0);

  map.worldToMap(-0.40, 0.0, x, y);
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto rear_safety = assessFootprint(map, footprint, 0.0, 0.0, 0.0);
  check(rear_safety.hard_clearance > 0.0, "hard rear clearance not measured");
  check(rear_safety.hard_clearance < 0.20, "hard rear obstacle too far away");
  check(clearance(map, footprint, 0.0, 0.0, 0.0) > 0.0, "safe rear clearance rejected");
  check(clearance(map, footprint, -0.10, 0.0, 0.0) < 0.0, "reverse into rear wall missed");
  check(
    clearance(map, footprint, 0.10, 0.0, 0.0) >
    clearance(map, footprint, 0.0, 0.0, 0.0),
    "movement away from rear obstacle did not increase clearance");
  map.setCost(x, y, 0);

  map.worldToMap(0.0, 0.29, x, y);
  map.setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  check(clearance(map, footprint, 0.0, 0.0, 0.0) > 0.0, "side obstacle falsely overlaps");
  check(clearance(map, footprint, 0.0, 0.0, M_PI_2) < 0.0, "rotation into obstacle missed");
  map.setCost(x, y, 0);

  map.worldToMap(0.0, 0.0, x, y);
  map.setCost(x, y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  const auto soft_overlap = assessFootprint(map, footprint, 0.0, 0.0, 0.0);
  check(soft_overlap.collisionFree(), "cost 253 treated as hard collision");
  check(soft_overlap.clearance == 0.0, "soft-band contact was not measured");
  check(soft_overlap.hard_clearance > 0.0, "soft-band contact reduced hard clearance");
  map.setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
  check(clearance(map, footprint, 0.0, 0.0, 0.0) < 0.0, "unknown interior accepted");

  check(clearance(map, footprint, 0.99, 0.0, 0.0) < 0.0, "out of map footprint accepted");
}
