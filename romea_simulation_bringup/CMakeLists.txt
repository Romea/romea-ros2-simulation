cmake_minimum_required(VERSION 3.5)
project(romea_simulation_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_simulation_bringup.py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_simulation_bringup.yaml test_simulation_bringup.yaml)
  ament_add_pytest_test(test_simulation_bringup test/test_simulation_bringup.py)
endif()

ament_package()
