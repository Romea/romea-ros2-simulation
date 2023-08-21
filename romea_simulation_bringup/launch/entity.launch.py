from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from romea_mobile_base_bringup import MobileBaseMetaDescription


def get_meta_description(context):

    file_path = LaunchConfiguration("meta_description_file_path").perform(context)
    return MobileBaseMetaDescription(file_path)


def launch_setup(context, *args, **kwargs):

    simulator_type = LaunchConfiguration("simulator_type").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)
    meta_description = get_meta_description(context)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("romea_simulation_bringup"),
                        "launch",
                        "simulators/" + simulator_type + "_entity.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "robot_urdf_description": robot_urdf_description,
            "xyz": ' '.join(map(str, meta_description.get_simulation_initial_xyz())),
            "rpy": ' '.join(map(str, meta_description.get_simulation_initial_rpy())),
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument("simulator_type", default_value="gazebo")
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace")
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_urdf_description")
    )

    declared_arguments.append(
        DeclareLaunchArgument("meta_description_file_path")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
