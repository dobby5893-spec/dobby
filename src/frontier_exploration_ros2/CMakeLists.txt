cmake_minimum_required(VERSION 3.8)
project(frontier_exploration_ros2)

if(POLICY CMP0069)
  cmake_policy(SET CMP0069 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_surface_3d_planner REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlExploration.srv"
  "msg/FloorMapSnapshot.msg"
  DEPENDENCIES nav_msgs octomap_msgs
)

add_library(${PROJECT_NAME}_core
  src/decision_map.cpp
  src/frontier_evaluator.cpp
  src/frontier_scorer.cpp
  src/geometry_analyzer.cpp
  src/topology_graph.cpp
  src/frontier_suppression.cpp
  src/frontier_policy.cpp
  src/frontier_search.cpp
  src/frontier_explorer_core_init.cpp
  src/frontier_explorer_core_maps.cpp
  src/frontier_explorer_core_frontiers.cpp
  src/frontier_explorer_core_suppression.cpp
  src/frontier_explorer_core_dispatch.cpp
  src/mrtsp_ordering.cpp
  src/mrtsp_solver.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_core
  action_msgs
  geometry_msgs
  nav_msgs
)

add_library(${PROJECT_NAME}_node
  src/frontier_explorer_node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_core
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node
  ${cpp_typesupport_target}
)
ament_target_dependencies(${PROJECT_NAME}_node
  go2_surface_3d_planner
  octomap_msgs
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(frontier_explorer
  src/main.cpp
)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
target_link_libraries(frontier_explorer
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_node
)
ament_target_dependencies(frontier_explorer
  rclcpp
)

add_executable(frontier_exploration_ctl
  src/frontier_exploration_ctl.cpp
)
target_compile_features(frontier_exploration_ctl PUBLIC cxx_std_17)
target_link_libraries(frontier_exploration_ctl
  ${cpp_typesupport_target}
)
ament_target_dependencies(frontier_exploration_ctl
  rclcpp
)

# Passive RViz debug observer. It links against the core library to reuse the
# same frontier extraction and scoring code, but it is installed as a separate
# executable so visualization never becomes part of the navigation control path.
add_executable(frontier_debug_observer
  src/debug/debug_observer_node.cpp
  src/debug/debug_analyzer.cpp
  src/debug/debug_markers.cpp
)
target_compile_features(frontier_debug_observer PUBLIC cxx_std_17)
target_link_libraries(frontier_debug_observer
  ${PROJECT_NAME}_core
)
ament_target_dependencies(frontier_debug_observer
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
  if(ipo_supported)
    set_property(TARGET ${PROJECT_NAME}_core PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    set_property(TARGET ${PROJECT_NAME}_node PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    set_property(TARGET frontier_explorer PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    set_property(TARGET frontier_exploration_ctl PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
    set_property(TARGET frontier_debug_observer PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  endif()
endif()

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    frontier_explorer
    frontier_exploration_ctl
    frontier_debug_observer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
    srv
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_residual_snapshot_node test/test_residual_snapshot_node.cpp)
  target_include_directories(test_residual_snapshot_node PRIVATE include)
  target_link_libraries(test_residual_snapshot_node ${PROJECT_NAME}_node octomap)
  set_tests_properties(test_residual_snapshot_node PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=190;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_map_consistency test/test_map_consistency.cpp)
  target_include_directories(test_map_consistency PRIVATE include)
  target_link_libraries(test_map_consistency ${cpp_typesupport_target} octomap)
  target_compile_features(test_map_consistency PRIVATE cxx_std_17)
  ament_target_dependencies(test_map_consistency nav_msgs octomap_msgs go2_surface_3d_planner)

  ament_add_gtest(test_region_observation_task test/test_region_observation_task.cpp)
  target_include_directories(test_region_observation_task PRIVATE include)
  target_link_libraries(test_region_observation_task ${PROJECT_NAME}_core)
  ament_target_dependencies(test_region_observation_task action_msgs geometry_msgs nav_msgs)

  ament_add_gtest(test_floor_coverage_policy test/test_floor_coverage_policy.cpp)
  target_include_directories(test_floor_coverage_policy PRIVATE include)
  target_link_libraries(test_floor_coverage_policy ${PROJECT_NAME}_core)
  ament_target_dependencies(test_floor_coverage_policy geometry_msgs nav_msgs)

  ament_add_gtest(test_viewpoint_fan test/test_viewpoint_fan.cpp)
  target_link_libraries(test_viewpoint_fan ${PROJECT_NAME}_core)
  target_include_directories(test_viewpoint_fan PRIVATE include)
  ament_target_dependencies(test_viewpoint_fan geometry_msgs nav_msgs)

  ament_add_gtest(test_observation_progress test/test_observation_progress.cpp)
  target_include_directories(test_observation_progress PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_observation_progress PRIVATE cxx_std_17)
  ament_target_dependencies(test_observation_progress geometry_msgs nav_msgs)

  ament_add_gtest(test_nearby_observation test/test_nearby_observation.cpp)
  target_include_directories(test_nearby_observation PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_nearby_observation PRIVATE cxx_std_17)
  ament_target_dependencies(test_nearby_observation go2_surface_3d_planner)
  target_link_libraries(test_nearby_observation octomap)

  ament_add_gtest(test_symmetric_relocation test/test_symmetric_relocation.cpp)
  target_link_libraries(test_symmetric_relocation ${PROJECT_NAME}_core)
  target_include_directories(test_symmetric_relocation PRIVATE include)
  target_compile_features(test_symmetric_relocation PRIVATE cxx_std_17)
  ament_target_dependencies(test_symmetric_relocation action_msgs geometry_msgs nav_msgs go2_surface_3d_planner)

  ament_add_gtest(test_random_recovery test/test_random_recovery.cpp)
  target_include_directories(test_random_recovery PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_random_recovery PRIVATE cxx_std_17)
  ament_target_dependencies(test_random_recovery go2_surface_3d_planner)
  target_link_libraries(test_random_recovery octomap)

  ament_add_gtest(test_validation_retry_queue test/test_validation_retry_queue.cpp)
  target_include_directories(test_validation_retry_queue PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_compile_features(test_validation_retry_queue PRIVATE cxx_std_17)

  ament_add_gtest(test_geometry_and_scoring
    test/test_geometry_and_scoring.cpp
  )
  target_link_libraries(test_geometry_and_scoring
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_geometry_and_scoring PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_geometry_and_scoring
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_topology_graph
    test/test_topology_graph.cpp
  )
  target_link_libraries(test_topology_graph
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_topology_graph PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_topology_graph
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_frontier_evaluator
    test/test_frontier_evaluator.cpp
  )
  target_link_libraries(test_frontier_evaluator
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_frontier_evaluator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_frontier_evaluator
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_frontier_snapshot_and_search
    test/test_frontier_snapshot_and_search.cpp
  )
  target_link_libraries(test_frontier_snapshot_and_search
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_frontier_snapshot_and_search PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_frontier_snapshot_and_search
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_preemption_flow
    test/test_preemption_flow.cpp
  )
  target_link_libraries(test_preemption_flow
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_preemption_flow PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_preemption_flow
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_qos_utils
    test/test_qos_utils.cpp
  )
  target_include_directories(test_qos_utils PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_qos_utils
    action_msgs
    geometry_msgs
    nav_msgs
    rclcpp
  )

  ament_add_gtest(test_frontier_suppression
    test/test_frontier_suppression.cpp
  )
  target_link_libraries(test_frontier_suppression
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_frontier_suppression PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_frontier_suppression
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_decision_map_and_mrtsp
    test/test_decision_map_and_mrtsp.cpp
  )
  target_link_libraries(test_decision_map_and_mrtsp
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_decision_map_and_mrtsp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_decision_map_and_mrtsp
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_mrtsp_solver
    test/test_mrtsp_solver.cpp
  )
  target_link_libraries(test_mrtsp_solver
    ${PROJECT_NAME}_core
  )
  target_include_directories(test_mrtsp_solver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_mrtsp_solver
    action_msgs
    geometry_msgs
    nav_msgs
  )

  ament_add_gtest(test_localization_freshness test/test_localization_freshness.cpp)
  target_include_directories(test_localization_freshness PRIVATE include)
  ament_add_gtest(test_control_service_and_idle
    test/test_control_service_and_idle.cpp
  )
  target_link_libraries(test_control_service_and_idle
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_node
    ${cpp_typesupport_target}
  )
  target_include_directories(test_control_service_and_idle PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  ament_target_dependencies(test_control_service_and_idle
    action_msgs
    geometry_msgs
    nav2_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    std_msgs
    tf2
    tf2_ros
    visualization_msgs
  )
  ament_add_gtest(test_connected_dispatch
  test/test_connected_dispatch.cpp
  )
  target_link_libraries(test_connected_dispatch
    ${PROJECT_NAME}_core
    octomap
  )
  target_include_directories(test_connected_dispatch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  ament_target_dependencies(test_connected_dispatch
    action_msgs
    geometry_msgs
    go2_surface_3d_planner
    nav_msgs
  )
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  action_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  octomap_msgs
  rclcpp
  rclcpp_action
  rosidl_default_runtime
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

ament_package()
