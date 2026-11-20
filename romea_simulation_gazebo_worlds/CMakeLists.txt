cmake_minimum_required(VERSION 3.5)
project(romea_simulation_gazebo_worlds)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(gazebo_ros QUIET)
find_package(ros_gz_sim QUIET)

install(DIRECTORY launch media models worlds
    DESTINATION share/${PROJECT_NAME}
)


if(gazebo_ros_FOUND)
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/classic/${PROJECT_NAME}.dsv.in")
elseif(ros_gz_sim_FOUND)
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz/${PROJECT_NAME}.sh.in")
endif()


ament_python_install_module(python/romea_gazebo_world.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gazebo_world test/test_gazebo_world.py)
endif()


ament_package()
