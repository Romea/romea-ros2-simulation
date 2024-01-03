from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    ExecuteProcess,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_simulation_bringup import (
    get_world_package,
    get_world_name,
    # has_wgs84_anchor,
    # get_wgs84_anchor,
)

from romea_gazebo_world import GazeboWorld
import yaml


def get_simulation_configuration(context):
    simulation_configuration_file_path = LaunchConfiguration(
        "simulation_configuration_file_path"
    ).perform(context)

    with open(simulation_configuration_file_path) as f:
        return yaml.safe_load(f)


def get_wgs84_anchor(context):
    wgs84_anchor_file_path = LaunchConfiguration(
        "wgs84_anchor_file_path"
    ).perform(context)

    if wgs84_anchor_file_path == "":
        return None

    with open(wgs84_anchor_file_path) as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):

    configuration = get_simulation_configuration(context)
    wgs84_anchor = get_wgs84_anchor(context)

    world = GazeboWorld(get_world_package(configuration), get_world_name(configuration))

    # if has_wgs84_anchor(configuration):
    #     world.set_wgs84_anchor(get_wgs84_anchor(configuration))

    if wgs84_anchor is not None:
        world.set_wgs84_anchor(wgs84_anchor)

    world.save("/tmp/gazebo_world.world")

    gzserver = ExecuteProcess(
        cmd=[
            'gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
            '/tmp/gazebo_world.world'
        ],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    return [GroupAction([gzserver, gzclient])]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("simulation_configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path", default_value=""))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
