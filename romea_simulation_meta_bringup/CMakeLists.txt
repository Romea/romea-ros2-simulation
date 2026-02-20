cmake_minimum_required(VERSION 3.5)
project(romea_simulation_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(romea_simulation_meta_bringup)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_flake8)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_simulation_meta_bringup.yaml test_simulation_meta_bringup.yaml)
  ament_add_pytest_test(test_simulation_meta_bringup test/test_simulation_meta_bringup.py)
endif()

ament_package()
