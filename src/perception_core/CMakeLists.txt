cmake_minimum_required(VERSION 3.8)
project(perception_core)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(sensor_sync_node src/nodes/sensor_sync_node.cpp)
target_compile_features(sensor_sync_node PUBLIC cxx_std_17)
ament_target_dependencies(sensor_sync_node builtin_interfaces rclcpp sensor_msgs)

install(
  TARGETS sensor_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The repository still contains intentionally empty future-work stubs.
  # Avoid treating those placeholders as production headers during linting.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
