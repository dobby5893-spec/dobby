cmake_minimum_required(VERSION 3.8)
project(go2_nav2_recovery)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(go2_surface_3d_planner REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(go2_escape_move_behavior SHARED
  src/escape_move.cpp)
target_include_directories(go2_escape_move_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(go2_escape_move_behavior
  geometry_msgs std_msgs go2_surface_3d_planner octomap_msgs nav2_behaviors nav2_core nav2_costmap_2d nav2_msgs nav2_util
  pluginlib rclcpp rclcpp_action rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)

add_library(go2_escape_move_action_bt_node SHARED
  src/escape_move_action.cpp)
target_compile_definitions(go2_escape_move_action_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(go2_escape_move_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(go2_escape_move_action_bt_node
  behaviortree_cpp_v3 nav2_behavior_tree nav2_msgs rclcpp rclcpp_action)

add_library(go2_path_safety_bt_node SHARED src/path_safety.cpp)
target_compile_definitions(go2_path_safety_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(go2_path_safety_bt_node PRIVATE include)
ament_target_dependencies(go2_path_safety_bt_node
  behaviortree_cpp_v3 nav2_behavior_tree nav2_costmap_2d nav2_msgs geometry_msgs std_msgs rclcpp tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS go2_path_safety_bt_node LIBRARY DESTINATION lib)

pluginlib_export_plugin_description_file(nav2_core behavior_plugin.xml)

install(TARGETS
  go2_escape_move_behavior
  go2_escape_move_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES behavior_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(go2_escape_move_behavior go2_escape_move_action_bt_node)
ament_export_dependencies(
  behaviortree_cpp_v3 geometry_msgs std_msgs go2_surface_3d_planner octomap_msgs nav2_behavior_tree nav2_behaviors nav2_core
  nav2_costmap_2d nav2_msgs nav2_util pluginlib rclcpp rclcpp_action
  rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
ament_package()

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_footprint_safety test/test_footprint.cpp)
  target_include_directories(test_footprint_safety PRIVATE include)
  ament_target_dependencies(test_footprint_safety geometry_msgs nav2_costmap_2d)
  add_test(NAME footprint_safety COMMAND test_footprint_safety)
endif()

add_executable(check_tree_no_motion test/check_tree.cpp)
ament_target_dependencies(check_tree_no_motion nav2_behavior_tree rclcpp tf2_ros geometry_msgs nav_msgs nav2_costmap_2d)

if(BUILD_TESTING)
  add_executable(test_path_validation test/test_path_validation.cpp)
  ament_target_dependencies(test_path_validation nav2_behavior_tree rclcpp tf2_ros geometry_msgs std_msgs nav_msgs nav2_msgs)
  add_test(NAME path_validation COMMAND test_path_validation)
  set_tests_properties(path_validation PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=187" TIMEOUT 20)
endif()
