cmake_minimum_required(VERSION 3.8)
project(perception_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fusion_interfaces REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/core/calibration_loader.cpp
  src/core/coordinate_transform.cpp
  src/core/lidar_projector.cpp
  src/core/object_depth_estimator.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp)

add_executable(sensor_sync_node src/nodes/sensor_sync_node.cpp)
target_compile_features(sensor_sync_node PUBLIC cxx_std_17)
ament_target_dependencies(
  sensor_sync_node
  builtin_interfaces
  fusion_interfaces
  message_filters
  rclcpp
  sensor_msgs
)

add_executable(object_fusion_node src/nodes/object_fusion_node.cpp)
target_compile_features(object_fusion_node PUBLIC cxx_std_17)
target_link_libraries(object_fusion_node ${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_include_directories(object_fusion_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  object_fusion_node
  builtin_interfaces
  cv_bridge
  fusion_interfaces
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)

install(
  TARGETS ${PROJECT_NAME} sensor_sync_node object_fusion_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # The repository still contains intentionally empty future-work stubs.
  # Avoid treating those placeholders as production headers during linting.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_calibration_loader test/test_calibration_loader.cpp)
  target_link_libraries(test_calibration_loader ${PROJECT_NAME})
  target_compile_definitions(test_calibration_loader PRIVATE
    TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../test_data/calibration"
    FUSION_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../fusion_bringup/config")

  ament_add_gtest(test_coordinate_transform test/test_coordinate_transform.cpp)
  target_link_libraries(test_coordinate_transform ${PROJECT_NAME})

  ament_add_gtest(test_lidar_projection test/test_lidar_projection.cpp)
  target_link_libraries(test_lidar_projection ${PROJECT_NAME})

  ament_add_gtest(test_depth_estimator test/test_depth_estimator.cpp)
  target_link_libraries(test_depth_estimator ${PROJECT_NAME})
endif()

ament_package()
