cmake_minimum_required(VERSION 3.8)
project(go2_surface_3d_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(surface_graph STATIC src/surface_graph.cpp)
set_target_properties(surface_graph PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(surface_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(surface_graph ${OCTOMAP_LIBRARIES})

add_library(${PROJECT_NAME} SHARED
  src/surface_3d_planner.cpp
  src/clearance_astar_planner.cpp
  src/goal_checker_3d.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} surface_graph ${OCTOMAP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs nav2_core nav2_costmap_2d nav_msgs octomap_msgs pluginlib
  rclcpp rclcpp_lifecycle std_msgs tf2 tf2_geometry_msgs tf2_ros)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS surface_graph ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_surface_graph test/test_surface_graph.cpp)
  target_include_directories(test_surface_graph PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_surface_graph surface_graph ${OCTOMAP_LIBRARIES})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} surface_graph)
ament_export_dependencies(
  geometry_msgs nav2_core nav2_costmap_2d nav_msgs octomap_msgs pluginlib
  rclcpp rclcpp_lifecycle std_msgs tf2 tf2_geometry_msgs tf2_ros)
ament_package()
