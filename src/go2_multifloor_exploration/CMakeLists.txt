cmake_minimum_required(VERSION 3.8)
project(go2_multifloor_exploration)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(frontier_exploration_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_surface_3d_planner REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(floor_slice STATIC src/floor_slice.cpp)
set_target_properties(floor_slice PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(floor_slice PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(floor_slice ${OCTOMAP_LIBRARIES})

add_executable(floor_slice_node src/floor_slice_node.cpp)
target_include_directories(floor_slice_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(floor_slice_node floor_slice ${OCTOMAP_LIBRARIES})
ament_target_dependencies(floor_slice_node
  frontier_exploration_ros2 nav_msgs octomap_msgs rclcpp std_msgs)

add_library(stair_transition STATIC src/stair_transition.cpp)
set_target_properties(stair_transition PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(stair_transition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(stair_transition go2_surface_3d_planner)
target_link_libraries(stair_transition ${OCTOMAP_LIBRARIES})

add_executable(multifloor_manager src/multifloor_manager.cpp)
target_include_directories(multifloor_manager PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(multifloor_manager stair_transition ${OCTOMAP_LIBRARIES})
ament_target_dependencies(multifloor_manager
  frontier_exploration_ros2 geometry_msgs go2_surface_3d_planner nav_msgs nav2_msgs
  octomap_msgs rclcpp rclcpp_action std_msgs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)

install(TARGETS floor_slice stair_transition floor_slice_node multifloor_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_handoff_consistency test/test_handoff_consistency.cpp)
  target_include_directories(test_handoff_consistency PRIVATE include)
  target_link_libraries(test_handoff_consistency stair_transition ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(test_handoff_consistency
    frontier_exploration_ros2 geometry_msgs go2_surface_3d_planner nav_msgs nav2_msgs
    octomap_msgs rclcpp rclcpp_action std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
  set_tests_properties(test_handoff_consistency PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=189;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_transition_footprint test/test_transition_footprint.cpp)
  target_link_libraries(test_transition_footprint stair_transition)
  ament_target_dependencies(test_transition_footprint go2_surface_3d_planner)
  ament_add_gtest(test_floor_handoff test/test_floor_handoff.cpp)
  target_include_directories(test_floor_handoff PRIVATE include)
  ament_target_dependencies(test_floor_handoff frontier_exploration_ros2)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ground_parameters test/test_ground_parameters.py TIMEOUT 30
    ENV "ROS_DOMAIN_ID=187" "ROS_LOCALHOST_ONLY=1")
  add_executable(make_stair_fixture test/make_stair_fixture.cpp)
  target_include_directories(make_stair_fixture PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(make_stair_fixture ${OCTOMAP_LIBRARIES})
  ament_target_dependencies(make_stair_fixture frontier_exploration_ros2 nav_msgs octomap_msgs)
  ament_add_pytest_test(test_floor_snapshot test/test_floor_snapshot.py TIMEOUT 40
    ENV "ROS_DOMAIN_ID=188" "ROS_LOCALHOST_ONLY=1"
      "FLOOR_SLICE_NODE=$<TARGET_FILE:floor_slice_node>"
      "STAIR_TEST_FIXTURE=$<TARGET_FILE:make_stair_fixture>")
  ament_add_pytest_test(test_stair_protocol test/test_stair_protocol.py TIMEOUT 90
    ENV "ROS_DOMAIN_ID=179" "ROS_LOCALHOST_ONLY=1"
      "STAIR_TEST_MANAGER=$<TARGET_FILE:multifloor_manager>"
      "STAIR_TEST_FIXTURE=$<TARGET_FILE:make_stair_fixture>")
  ament_add_gtest(test_stair_transition test/test_stair_transition.cpp)
  target_link_libraries(test_stair_transition stair_transition)
  ament_target_dependencies(test_stair_transition go2_surface_3d_planner)
  ament_add_gtest(test_floor_slice test/test_floor_slice.cpp)
  target_include_directories(test_floor_slice PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_floor_slice floor_slice ${OCTOMAP_LIBRARIES})
endif()

ament_export_include_directories(include)
ament_export_libraries(floor_slice)
ament_package()
