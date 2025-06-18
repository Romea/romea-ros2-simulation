from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    ExecuteProcess,
)

from launch.substitutions import LaunchConfiguration

from romea_simulation_meta_bringup import (
    get_world_package,
    get_world_name,
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

    wgs84_anchor = get_wgs84_anchor(context)
    configuration = get_simulation_configuration(context)
    print(configuration)
    world = GazeboWorld(get_world_package(configuration), get_world_name(configuration))

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

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_configuration_file_path"),
            DeclareLaunchArgument("wgs84_anchor_file_path", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
