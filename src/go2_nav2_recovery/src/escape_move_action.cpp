#include "go2_nav2_recovery/escape_move_action.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/duration.hpp"

namespace go2_nav2_recovery
{

EscapeMoveAction::EscapeMoveAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BtActionNode<nav2_msgs::action::DriveOnHeading>(xml_tag_name, action_name, config)
{
}

void EscapeMoveAction::on_tick()
{
  double speed = 0.05;
  double time_allowance = 8.0;
  getInput("dist_x", goal_.target.x);
  getInput("dist_y", goal_.target.y);
  getInput("speed", speed);
  getInput("time_allowance", time_allowance);
  goal_.target.z = 0.0;
  goal_.speed = static_cast<float>(std::abs(speed));
  const auto duration = rclcpp::Duration::from_seconds(time_allowance);
  goal_.time_allowance.sec = static_cast<int32_t>(duration.nanoseconds() / 1000000000LL);
  goal_.time_allowance.nanosec = static_cast<uint32_t>(
    duration.nanoseconds() % 1000000000LL);
}

}  // namespace go2_nav2_recovery

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<go2_nav2_recovery::EscapeMoveAction>(
        name, "escape_move", config);
    };
  factory.registerBuilder<go2_nav2_recovery::EscapeMoveAction>("EscapeMove", builder);
}
