#pragma once
#include <algorithm>
namespace scan_planner_nav2_controller {
// Raising the flat-ground ceiling must not raise the previous stair ceiling.
inline double motionSpeedLimit(double nominal,double scale,bool stairs) {
 return std::max(0.,stairs?std::min(nominal,.20):nominal)*std::clamp(scale,0.,1.);
}
}
