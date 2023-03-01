from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_simulation_bringup import (
    get_world_package,
    get_world_name,
    has_wgs84_anchor,
    get_wgs84_anchor,
)

from romea_gazebo_world import GazeboWorld
import yaml


def get_simulation_configuration(context):
    simulation_configuration_filename = LaunchConfiguration(
        "simulation_configuration_filename"
    ).perform(context)

    with open(simulation_configuration_filename) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    configuration = get_simulation_configuration(context)

    world = GazeboWorld(get_world_package(configuration), get_world_name(configuration))

    if has_wgs84_anchor(configuration):
        world.set_wgs84_anchor(get_wgs84_anchor(configuration))

    world.save("/tmp/gazebo_world.world")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"]
                )
            ]
        ),
        launch_arguments={"world": "/tmp/gazebo_world.world", "verbose": "true"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"]
                )
            ]
        ),
        launch_arguments={"verbose": "false"}.items(),
    )

    return [GroupAction([gzserver, gzclient])]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("simulation_configuration_filename"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
