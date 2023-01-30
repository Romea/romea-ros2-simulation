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

    simulation_configuration_filename = LaunchConfiguration(
        "simulation_configuration_filename"
    ).perform(context)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_simulation_bringup"),
                        "launch",
                        "simulators/" + simulator_type + ".launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "simulation_configuration_filename": simulation_configuration_filename
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("simulator_type", default_value="gazebo")
    )
    declared_arguments.append(
        DeclareLaunchArgument("simulation_configuration_filename")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
