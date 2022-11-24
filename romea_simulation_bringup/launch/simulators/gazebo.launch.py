from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from gazebo_world import GazeboWorld
import yaml


    
def launch_setup(context, *args, **kwargs):


    configuration_yaml_file = LaunchConfiguration("configuration_yaml_file").perform(context)

    with open(configuration_yaml_file) as f:
        configuration = yaml.safe_load(f)

    world = GazeboWorld(configuration["world_package"],configuration["world_name"])

    if "wgs84_anchor" in configuration:
        world.set_wgs84_anchor(configuration["wgs84_anchor"])

    world.save("/tmp/gazebo_world.world")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"]
                )
            ]
        ),
        launch_arguments={"world": "/tmp/gazebo_world.world","verbose": "true"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"]
                )
            ]
        ),
        launch_arguments={"verbose": "true"}.items(),
    )

    return [gzserver, gzclient]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("configuration_yaml_file"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
