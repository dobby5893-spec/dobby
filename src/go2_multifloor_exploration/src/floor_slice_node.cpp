#include "go2_multifloor_exploration/floor_slice.hpp"
#include <frontier_exploration_ros2/msg/floor_map_snapshot.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace go2_multifloor_exploration
{

class FloorSliceNode : public rclcpp::Node
{
public:
  FloorSliceNode()
  : Node("floor_slice_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_full");
    output_topic_ = declare_parameter<std::string>("output_topic", "/map_floor");
    floor_height_topic_ = declare_parameter<std::string>(
      "floor_height_topic", "/multifloor/active_floor_height");
    publish_period_s_ = declare_parameter<double>("publish_period_s", 1.0);
    floor_z_ = declare_parameter<double>("initial_floor_z", 0.0);

    FloorSliceConfig config;
    config.resolution = declare_parameter<double>("resolution", 0.10);
    config.min_x = declare_parameter<double>("min_x", -12.0);
    config.max_x = declare_parameter<double>("max_x", 12.0);
    config.min_y = declare_parameter<double>("min_y", -9.0);
    config.max_y = declare_parameter<double>("max_y", 9.0);
    config.support_probe_depth = declare_parameter<double>("support_probe_depth", 0.20);
    config.obstacle_min_height = declare_parameter<double>("obstacle_min_height", 0.12);
    config.obstacle_max_height = declare_parameter<double>("obstacle_max_height", 1.00);
    config.support_xy_tolerance = declare_parameter<double>("support_xy_tolerance", 0.08);
    config.free_space_min_height = declare_parameter<double>("free_space_min_height", 0.20);
    config.free_space_max_height = declare_parameter<double>("free_space_max_height", 0.80);
    config.free_space_min_samples = declare_parameter<int>("free_space_min_samples", 1);
    config.enclosed_unknown_max_area = declare_parameter<double>("enclosed_unknown_max_area", 1.0);
    config.enclosed_unknown_max_span = declare_parameter<double>("enclosed_unknown_max_span", 1.6);
    builder_ = std::make_unique<FloorSliceBuilder>(config);

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, map_qos,
      std::bind(&FloorSliceNode::mapCallback, this, std::placeholders::_1));
    floor_sub_ = create_subscription<std_msgs::msg::Float64>(
      floor_height_topic_, map_qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (std::abs(floor_z_ - msg->data) > 1e-6) {
          floor_z_ = msg->data;
          dirty_ = true;
          RCLCPP_INFO(get_logger(), "Active floor height changed to z=%.2f", floor_z_);
        }
      });
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);
    snapshot_pub_ = create_publisher<frontier_exploration_ros2::msg::FloorMapSnapshot>(output_topic_ + "/snapshot", map_qos);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, publish_period_s_))),
      std::bind(&FloorSliceNode::publishSlice, this));
  }

private:
  void mapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(*msg));
    auto * raw = abstract ? dynamic_cast<octomap::OcTree *>(abstract.get()) : nullptr;
    if (!raw) {
      RCLCPP_ERROR(get_logger(), "Cannot deserialize OctoMap type '%s'", msg->id.c_str());
      return;
    }
    abstract.release();
    std::lock_guard<std::mutex> lock(mutex_);
    tree_.reset(raw);
    source_ = msg;
    ++octomap_generation_;
    dirty_ = true;
  }

  void publishSlice()
  {
    std::shared_ptr<const octomap::OcTree> tree;
    double floor_z = 0.0;
    octomap_msgs::msg::Octomap::ConstSharedPtr source;
    uint64_t source_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!tree_ || !dirty_) {
        return;
      }
      tree = tree_;
      source = source_;source_generation = octomap_generation_;
      floor_z = floor_z_;
      dirty_ = false;
    }

    const auto slice = builder_->build(*tree, floor_z);
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.info.map_load_time = msg.header.stamp;
    msg.info.resolution = static_cast<float>(slice.resolution);
    msg.info.width = static_cast<uint32_t>(slice.width);
    msg.info.height = static_cast<uint32_t>(slice.height);
    msg.info.origin.position.x = slice.origin_x;
    msg.info.origin.position.y = slice.origin_y;
    msg.info.origin.position.z = floor_z;
    msg.info.origin.orientation.w = 1.0;
    msg.data = slice.data;
    // One message transports the immutable pair: delayed independent topics cannot
    // accidentally associate a slice with a different OctoMap. Legacy map is kept.
    frontier_exploration_ros2::msg::FloorMapSnapshot snapshot;
    snapshot.octomap_generation = source_generation;
    snapshot.source_octomap_generation = source_generation;
    snapshot.floor_map_generation = ++floor_map_generation_;
    snapshot.octomap = *source;snapshot.floor_map = msg;
    snapshot_pub_->publish(snapshot);
    publisher_->publish(msg);
  }

  std::string frame_id_;
  std::string octomap_topic_;
  std::string output_topic_;
  std::string floor_height_topic_;
  double publish_period_s_{1.0};
  double floor_z_{0.0};
  bool dirty_{false};
  std::unique_ptr<FloorSliceBuilder> builder_;
  std::mutex mutex_;
  std::shared_ptr<const octomap::OcTree> tree_;
  octomap_msgs::msg::Octomap::ConstSharedPtr source_;
  uint64_t octomap_generation_{0},floor_map_generation_{0};
  rclcpp::Publisher<frontier_exploration_ros2::msg::FloorMapSnapshot>::SharedPtr snapshot_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr floor_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace go2_multifloor_exploration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_multifloor_exploration::FloorSliceNode>());
  rclcpp::shutdown();
  return 0;
}
