cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# colcon passes no build type by default (-O0); the filter runs at the IMU rate.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# The filter itself is ROS-free: unit tests and the offline replay tool use it directly.
add_library(${PROJECT_NAME} src/wheel_imu_ekf.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

add_library(wheel_imu_ekf_component SHARED src/wheel_imu_ekf_node.cpp)
target_compile_features(wheel_imu_ekf_component PUBLIC cxx_std_17)
target_link_libraries(wheel_imu_ekf_component ${PROJECT_NAME})
ament_target_dependencies(
  wheel_imu_ekf_component
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
)
rclcpp_components_register_node(
  wheel_imu_ekf_component
  PLUGIN "state_estimation::WheelImuEkfNode"
  EXECUTABLE wheel_imu_ekf_node
)

add_executable(ekf_replay tools/ekf_replay.cpp)
target_compile_features(ekf_replay PUBLIC cxx_std_17)
target_link_libraries(ekf_replay ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} wheel_imu_ekf_component ekf_replay
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_wheel_imu_ekf test/test_wheel_imu_ekf.cpp)
  target_link_libraries(test_wheel_imu_ekf ${PROJECT_NAME})
endif()

ament_package()
