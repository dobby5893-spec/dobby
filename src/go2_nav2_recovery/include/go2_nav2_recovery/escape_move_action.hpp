#ifndef GO2_NAV2_RECOVERY__ESCAPE_MOVE_ACTION_HPP_
#define GO2_NAV2_RECOVERY__ESCAPE_MOVE_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/drive_on_heading.hpp"

namespace go2_nav2_recovery
{

class EscapeMoveAction : public nav2_behavior_tree::BtActionNode<
    nav2_msgs::action::DriveOnHeading>
{
public:
  EscapeMoveAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & config);

  void on_tick() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("dist_x", 0.0, "Forward/backward distance in robot frame"),
        BT::InputPort<double>("dist_y", 0.0, "Left/right distance in robot frame"),
        BT::InputPort<double>("speed", 0.05, "Planar movement speed"),
        BT::InputPort<double>("time_allowance", 8.0, "Allowed movement time")
      });
  }
};

}  // namespace go2_nav2_recovery

#endif  // GO2_NAV2_RECOVERY__ESCAPE_MOVE_ACTION_HPP_
