cmake_minimum_required(VERSION 3.5)
project(romea_simulation_gazebo_worlds)

find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch media models worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_gazebo_world.py)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gazebo_world test/test_gazebo_world.py)
endif()


ament_package()