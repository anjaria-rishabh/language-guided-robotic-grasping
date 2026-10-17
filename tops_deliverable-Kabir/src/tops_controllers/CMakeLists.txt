cmake_minimum_required(VERSION 3.16)
project(tops_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wno-sign-conversion)
endif()

# -O2 by default. This is not a stylistic preference: at -O0 the Eigen
# expression templates in update() are not inlined, the 7x7 and 6x6 products
# run an order of magnitude slower, and the 1 ms deadline is genuinely at
# risk on a laptop also running Gazebo.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tops_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_DEPS
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  std_msgs
  tops_msgs
  kdl_parser
  tf2_eigen
)

add_library(${PROJECT_NAME} SHARED
  src/kinematics_model.cpp
  src/cartesian_impedance_controller.cpp
  src/cartesian_admittance_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

foreach(dep ${THIS_PACKAGE_DEPS})
  if(TARGET ${dep}::${dep})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${dep}::${dep})
  else()
    target_link_libraries(${PROJECT_NAME} PUBLIC ${${dep}_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${${dep}_INCLUDE_DIRS})
  endif()
endforeach()
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${orocos_kdl_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${orocos_kdl_INCLUDE_DIRS})

pluginlib_export_plugin_description_file(controller_interface tops_controllers_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # The unit tests cover the pieces that can be checked without a robot:
  # the passivity budget, the filter's frequency response, the gain ramp,
  # and the dynamics model against hand-computed statics. Everything else
  # needs Gazebo and lives in tops_experiments.
  ament_add_gtest(test_energy_tank test/test_energy_tank.cpp)
  target_include_directories(test_energy_tank PRIVATE include)
  target_link_libraries(test_energy_tank Eigen3::Eigen)

  ament_add_gtest(test_low_pass_filter test/test_low_pass_filter.cpp)
  target_include_directories(test_low_pass_filter PRIVATE include)
  target_link_libraries(test_low_pass_filter Eigen3::Eigen)

  ament_add_gtest(test_gain_ramp test/test_gain_ramp.cpp)
  target_include_directories(test_gain_ramp PRIVATE include)
  target_link_libraries(test_gain_ramp Eigen3::Eigen)

  ament_add_gtest(test_kinematics_model test/test_kinematics_model.cpp)
  target_include_directories(test_kinematics_model PRIVATE include)
  target_link_libraries(test_kinematics_model ${PROJECT_NAME} Eigen3::Eigen)
  foreach(dep ${THIS_PACKAGE_DEPS})
    if(TARGET ${dep}::${dep})
      target_link_libraries(test_kinematics_model ${dep}::${dep})
    else()
      target_link_libraries(test_kinematics_model ${${dep}_LIBRARIES})
      target_include_directories(test_kinematics_model PRIVATE ${${dep}_INCLUDE_DIRS})
    endif()
  endforeach()

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPS} Eigen3 orocos_kdl)

ament_package()
