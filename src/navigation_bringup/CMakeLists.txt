cmake_minimum_required(VERSION 3.8)
project(navigation_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
