#include <iostream>
#include <chrono>
#include <thread>
#include "std_msgs/msg/header.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("isolated_path_test");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  blackboard->set("tf_buffer", std::make_shared<tf2_ros::Buffer>(node->get_clock()));

  nav2_behavior_tree::BehaviorTreeEngine engine({"go2_path_safety_bt_node"});
  auto validator = engine.createTreeFromText(
    "<root main_tree_to_execute='Check'><BehaviorTree ID='Check'>"
    "<ValidateCurrentPath candidate='{candidate}' goal='{goal}' "
    "path='{path}' path_goal='{path_goal}'/>"
    "</BehaviorTree></root>", blackboard);
  auto goal_match = engine.createTreeFromText(
    "<root main_tree_to_execute='Check'><BehaviorTree ID='Check'>"
    "<GoalMatches goal='{goal}' path_goal='{path_goal}'/>"
    "</BehaviorTree></root>", blackboard);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 0.5;
  goal.pose.orientation.w = 1.0;

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  auto start = goal;
  start.pose.position.x = 0.0;
  path.poses = {start, goal};

  blackboard->set("candidate", path);
  blackboard->set("goal", goal);
  if (validator.tickRoot() != BT::NodeStatus::SUCCESS) {
    throw std::runtime_error("valid path rejected");
  }
  if (goal_match.tickRoot() != BT::NodeStatus::SUCCESS) {
    throw std::runtime_error("accepted path was not bound to current goal");
  }

  auto changed_goal = goal;
  changed_goal.pose.position.x = 3.0;
  blackboard->set("goal", changed_goal);
  if (goal_match.tickRoot() != BT::NodeStatus::FAILURE) {
    throw std::runtime_error("changed goal accepted stale path");
  }
  if (validator.tickRoot() != BT::NodeStatus::FAILURE) {
    throw std::runtime_error("path ending at old goal accepted for new goal");
  }

  nav_msgs::msg::Path empty;
  empty.header.frame_id = "map";
  blackboard->set("candidate", empty);
  if (validator.tickRoot() != BT::NodeStatus::FAILURE) {
    throw std::runtime_error("empty path accepted");
  }

  blackboard->set("goal",goal);
  node->set_parameter(rclcpp::Parameter("use_sim_time",true));
  path.header.stamp.sec=9;path.header.stamp.nanosec=0;
  blackboard->set("candidate",path);
  auto request_pub=node->create_publisher<std_msgs::msg::Header>(
    "/planning/go2_replan_request",rclcpp::QoS(1).reliable().transient_local());
  std_msgs::msg::Header request;request.frame_id="map";request.stamp.sec=10;
  request_pub->publish(request);
  bool rejected=false;
  for(int i=0;i<200;++i) {
    if(validator.tickRoot()==BT::NodeStatus::FAILURE) {rejected=true;break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if(!rejected) {throw std::runtime_error("recovery did not invalidate old same-goal path");}
  path.header.stamp.sec=10;blackboard->set("candidate",path);
  if(validator.tickRoot()!=BT::NodeStatus::FAILURE) {
    throw std::runtime_error("same-clock-tick path incorrectly accepted after replan request");
  }
  path.header.stamp.sec=11;blackboard->set("candidate",path);
  if(validator.tickRoot()!=BT::NodeStatus::SUCCESS) {
    throw std::runtime_error("fresh planner result rejected after recovery");
  }

  std::cout <<
    "Path binding: valid path PASS, changed goal rejects stale path, empty path rejected\n";
  rclcpp::shutdown();
  return 0;
}
