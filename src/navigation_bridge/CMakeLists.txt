cmake_minimum_required(VERSION 3.8)
project(navigation_bridge)

find_package(ament_cmake REQUIRED)

# The API/source files are retained as the extension point for the Nav2 bridge.
# Targets will be enabled once the implementation is added.
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
