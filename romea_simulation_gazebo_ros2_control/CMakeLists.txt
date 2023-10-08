cmake_minimum_required(VERSION 3.5.0)
project(romea_simulation_gazebo_ros2_control)

find_package(ament_cmake REQUIRED)
# find_package(angles REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
# find_package(gazebo_ros REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(yaml_cpp_vendor REQUIRED)
find_package(gazebo_ros2_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion -Wno-sign-conversion -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")

include_directories(include)
link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

if(${ROS_DISTRO} STREQUAL "ROS_GALACTIC")
  # Libraries
  add_library(romea_gazebo_ros2_control SHARED
    src/gazebo_ros2_control_plugin.cpp
    src/gazebo_controller_manager.cpp)
else()
  add_library(romea_gazebo_ros2_control SHARED
    src/gazebo_ros2_control_plugin.cpp)
endif()


ament_target_dependencies(romea_gazebo_ros2_control
  controller_manager
  gazebo_dev
  hardware_interface
  pluginlib
  rclcpp
  gazebo_ros2_control
)

target_include_directories(romea_gazebo_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(romea_gazebo_ros2_control PUBLIC ROS_DISTRO=${ROS_DISTRO})

## Install
install(TARGETS
  romea_gazebo_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
