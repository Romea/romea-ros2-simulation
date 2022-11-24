cmake_minimum_required(VERSION 3.5)
project(romea_simulation_gazebo_worlds)

find_package(gazebo_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch media models worlds
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/gazebo_world.py)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")


ament_package()