from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_simulation_bringup"),
                        "launch",
                        "simulators/" + simulator_type + "_simulator.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "wgs84_anchor_file_path": wgs84_anchor_file_path,
            "simulation_configuration_file_path": simulation_configuration_file_path
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("simulator_type", default_value="gazebo")
    )
    declared_arguments.append(
        DeclareLaunchArgument("simulation_configuration_file_path")
    )

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path", default_value=""))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
