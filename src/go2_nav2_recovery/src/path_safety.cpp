#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace go2_nav2_recovery
{

class GoalMatches : public BT::ConditionNode
{
public:
  GoalMatches(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("path_goal")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped path_goal;
    if (!getInput("goal", goal) || !getInput("path_goal", path_goal)) {
      return BT::NodeStatus::FAILURE;
    }

    return goal.header.frame_id == path_goal.header.frame_id &&
           goal.pose == path_goal.pose ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ValidateCurrentPath : public BT::SyncActionNode
{
public:
  ValidateCurrentPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    rclcpp::SubscriptionOptions options;options.callback_group=group_;
    subscription_=node_->create_subscription<std_msgs::msg::Header>(
      "/planning/go2_replan_request",rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Header::SharedPtr message){replan_=*message;},options);
    executor_.add_callback_group(group_,node_->get_node_base_interface());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("path_goal")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path candidate;
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("candidate", candidate) ||
        !getInput("goal", goal) ||
        candidate.poses.empty() ||
        candidate.header.frame_id.empty() ||
        candidate.header.frame_id != goal.header.frame_id)
    {
      return BT::NodeStatus::FAILURE;
    }

    executor_.spin_some();
    const rclcpp::Time cutoff(replan_.stamp);
    if(replan_.frame_id==candidate.header.frame_id && cutoff.nanoseconds()>0 &&
      rclcpp::Time(candidate.header.stamp)<=cutoff) {
      return BT::NodeStatus::FAILURE;
    }
    const auto & end = candidate.poses.back().pose.position;
    const double dx = end.x - goal.pose.position.x;
    const double dy = end.y - goal.pose.position.y;
    if (std::hypot(dx, dy) > 0.25) {
      return BT::NodeStatus::FAILURE;
    }

    // The planner already uses the global costmap, the smoother checks its
    // result for collisions, and the controller checks the live local costmap.
    // This node binds the accepted path to the exact current goal so a path
    // from an earlier frontier cannot be reused.
    setOutput("path", candidate);
    setOutput("path_goal", goal);
    return BT::NodeStatus::SUCCESS;
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscription_;
  std_msgs::msg::Header replan_;
};

}  // namespace go2_nav2_recovery

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<go2_nav2_recovery::GoalMatches>("GoalMatches");
  factory.registerNodeType<go2_nav2_recovery::ValidateCurrentPath>("ValidateCurrentPath");
}
