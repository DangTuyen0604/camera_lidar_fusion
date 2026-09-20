cmake_minimum_required(VERSION 3.8)
project(navigation_bridge)

find_package(ament_cmake REQUIRED)
find_package(fusion_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} src/detection_obstacle_bridge.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} fusion_interfaces)

add_executable(detection_obstacle_bridge_node
  src/detection_obstacle_bridge_node.cpp)
target_compile_features(detection_obstacle_bridge_node PUBLIC cxx_std_17)
target_link_libraries(detection_obstacle_bridge_node ${PROJECT_NAME})
ament_target_dependencies(
  detection_obstacle_bridge_node diagnostic_msgs fusion_interfaces geometry_msgs
  rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

install(
  TARGETS ${PROJECT_NAME} detection_obstacle_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(fusion_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_obstacle_transform test/test_obstacle_transform.cpp)
  target_link_libraries(test_obstacle_transform ${PROJECT_NAME})
  add_launch_test(test/test_bridge_node.py TIMEOUT 30)
endif()

ament_package()
