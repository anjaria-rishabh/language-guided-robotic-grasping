cmake_minimum_required(VERSION 3.16)
project(tops_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

ament_package()
