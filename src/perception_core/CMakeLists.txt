cmake_minimum_required(VERSION 3.8)
project(perception_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# colcon passes no build type by default, i.e. -O0: the preprocessing
# pipeline then took 1.6 s per 130k-point scan instead of ~60 ms at -O2,
# far over a 10 Hz LiDAR's 100 ms budget.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

# The static libraries are linked into the shared component library too.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fusion_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/core/calibration_quality.cpp
  src/core/calibration_loader.cpp
  src/core/coordinate_transform.cpp
  src/core/euclidean_clustering.cpp
  src/core/ground_segmentation.cpp
  src/core/latency_tracker.cpp
  src/core/lidar_projector.cpp
  src/core/object_depth_estimator.cpp
  src/core/pointcloud_filter.cpp
  src/core/pointcloud_preprocessor.cpp
  src/core/timestamp_sync.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp)

# ROS message adapters, kept apart so the algorithms above stay ROS-free.
add_library(${PROJECT_NAME}_ros src/ros/pointcloud_io.cpp)
target_compile_features(${PROJECT_NAME}_ros PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_ros PUBLIC ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_ros PUBLIC sensor_msgs std_msgs)

add_executable(sensor_sync_node src/nodes/sensor_sync_node.cpp)
target_compile_features(sensor_sync_node PUBLIC cxx_std_17)
target_link_libraries(sensor_sync_node ${PROJECT_NAME})
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
target_link_libraries(object_fusion_node ${PROJECT_NAME}_ros ${OpenCV_LIBRARIES})
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

add_executable(lidar_projection_node src/nodes/lidar_projection_node.cpp)
target_compile_features(lidar_projection_node PUBLIC cxx_std_17)
target_link_libraries(lidar_projection_node ${PROJECT_NAME}_ros ${OpenCV_LIBRARIES})
target_include_directories(lidar_projection_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(
  lidar_projection_node
  cv_bridge
  message_filters
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

add_executable(calibration_monitor_node src/nodes/calibration_monitor_node.cpp)
target_compile_features(calibration_monitor_node PUBLIC cxx_std_17)
target_link_libraries(calibration_monitor_node ${PROJECT_NAME})
ament_target_dependencies(calibration_monitor_node fusion_interfaces rclcpp)

add_executable(metrics_node src/nodes/metrics_node.cpp)
target_compile_features(metrics_node PUBLIC cxx_std_17)
target_link_libraries(metrics_node ${PROJECT_NAME})
ament_target_dependencies(metrics_node fusion_interfaces rclcpp)

# Composable node; also installed as the standalone pointcloud_preprocess_node.
add_library(pointcloud_preprocess_component SHARED src/nodes/pointcloud_preprocess_node.cpp)
target_compile_features(pointcloud_preprocess_component PUBLIC cxx_std_17)
target_link_libraries(pointcloud_preprocess_component ${PROJECT_NAME}_ros)
ament_target_dependencies(
  pointcloud_preprocess_component
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
rclcpp_components_register_node(
  pointcloud_preprocess_component
  PLUGIN "perception_core::PointCloudPreprocessNode"
  EXECUTABLE pointcloud_preprocess_node
)

add_executable(pointcloud_preprocess_benchmark src/tools/pointcloud_preprocess_benchmark.cpp)
target_compile_features(pointcloud_preprocess_benchmark PUBLIC cxx_std_17)
target_link_libraries(pointcloud_preprocess_benchmark ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros pointcloud_preprocess_component
    pointcloud_preprocess_benchmark
    sensor_sync_node object_fusion_node
    lidar_projection_node calibration_monitor_node metrics_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 sensor_msgs std_msgs yaml-cpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
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

  ament_add_gtest(test_calibration_quality test/test_calibration_quality.cpp)
  target_link_libraries(test_calibration_quality ${PROJECT_NAME})

  ament_add_gtest(test_pointcloud_filter test/test_pointcloud_filter.cpp)
  target_link_libraries(test_pointcloud_filter ${PROJECT_NAME})

  ament_add_gtest(test_pointcloud_io test/test_pointcloud_io.cpp)
  target_link_libraries(test_pointcloud_io ${PROJECT_NAME}_ros)

  ament_add_gtest(test_pointcloud_preprocessing test/test_pointcloud_preprocessing.cpp)
  target_link_libraries(test_pointcloud_preprocessing ${PROJECT_NAME})

  ament_add_gtest(test_timestamp_sync test/test_timestamp_sync.cpp)
  target_link_libraries(test_timestamp_sync ${PROJECT_NAME})

  ament_add_gtest(test_latency_tracker test/test_latency_tracker.cpp)
  target_link_libraries(test_latency_tracker ${PROJECT_NAME})
endif()

ament_package()
