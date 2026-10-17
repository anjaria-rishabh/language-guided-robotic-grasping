cmake_minimum_required(VERSION 3.16)
project(tops_description)

find_package(ament_cmake REQUIRED)

# meshes/ holds the real Panda geometry and its textures; urdf/ refers to it
# as package://tops_description/meshes/..., which resolves to this install.
install(
  DIRECTORY urdf launch rviz config meshes
  DESTINATION share/${PROJECT_NAME}
)

# Gazebo resolves the model:// URIs the URDF-to-SDF converter writes
# (model://tops_description/meshes/...) by searching GZ_SIM_RESOURCE_PATH
# for a folder called tops_description. colcon installs every package under
# its own prefix, so this package must put ITS share/ on that path itself;
# without it Gazebo loads the robot with no meshes at all.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/resource_paths.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The URDF check below needs a working xacro at test time; it is the
  # cheapest possible guard against a macro change that produces invalid XML.
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
