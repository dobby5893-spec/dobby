#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "tf2_ros/transform_listener.h"
#include <iostream>
int main(int argc,char ** argv){
 rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("check_navigation_tree_no_motion");
 auto bb=BT::Blackboard::create();bb->set("node",node);
 auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());bb->set("tf_buffer",tf);
 auto listener=std::make_shared<tf2_ros::TransformListener>(*tf);
 nav2_costmap_2d::FootprintSubscriber fp(node,"/global_costmap/published_footprint",*tf,"base_link",0.2);
 bb->set("server_timeout",std::chrono::milliseconds(20));
 bb->set("bt_loop_duration",std::chrono::milliseconds(10));
 bb->set("wait_for_service_timeout",std::chrono::milliseconds(3000));
 nav2_behavior_tree::BehaviorTreeEngine engine({"nav2_compute_path_to_pose_action_bt_node","nav2_smooth_path_action_bt_node","nav2_follow_path_action_bt_node","nav2_rate_controller_bt_node","nav2_recovery_node_bt_node","nav2_pipeline_sequence_bt_node","go2_escape_move_action_bt_node","go2_path_safety_bt_node"});
 auto tree=engine.createTreeFromFile("/home/dobby/scan/config/navigate_go2_recovery.xml",bb);
 // Construction validates all XML ports and plugin factories. Never tick navigation.
 auto check=engine.createTreeFromText("<root main_tree_to_execute='Check'><BehaviorTree ID='Check'><GoalMatches goal='{goal}' path_goal='{path_goal}'/></BehaviorTree></root>",bb);
 geometry_msgs::msg::PoseStamped a;a.header.frame_id="map";a.pose.orientation.w=1;bb->set("goal",a);bb->set("path_goal",a);
 if(check.tickRoot()!=BT::NodeStatus::SUCCESS)throw std::runtime_error("matching goal rejected");
 a.pose.position.x=7.95;bb->set("goal",a);
 if(check.tickRoot()!=BT::NodeStatus::FAILURE)throw std::runtime_error("stale path accepted");
 std::cout<<"Full tree constructed without motion; changed goal rejects old path: PASS\n";
 const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
 std_msgs::msg::Header header;std::vector<geometry_msgs::msg::Point> footprint;
 bool received=false;
 while(std::chrono::steady_clock::now()<deadline){rclcpp::spin_some(node);if(fp.getFootprintInRobotFrame(footprint,header)){received=true;break;}}
 if(!received) throw std::runtime_error("Live footprint transform unavailable");
 std::cout<<"Live footprint header frame="<<header.frame_id<<", vertices="<<footprint.size()<<"\n";
 rclcpp::shutdown();
}
