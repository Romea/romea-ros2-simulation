from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    simulator_type = LaunchConfiguration("simulator_type").perform(context)

    simulation_configuration_file_path = LaunchConfiguration(
        "simulation_configuration_file_path"
    ).perform(context)

    wgs84_anchor_file_path = LaunchConfiguration(
        "wgs84_anchor_file_path"
    ).perform(context)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulators/" + simulator_type + "_simulator.launch.py",
        ),
        launch_arguments={
            "wgs84_anchor_file_path": wgs84_anchor_file_path,
            "simulation_configuration_file_path": simulation_configuration_file_path
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulator_type"),
            DeclareLaunchArgument("simulation_configuration_file_path"),
            DeclareLaunchArgument("wgs84_anchor_file_path", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
