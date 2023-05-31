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

    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

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

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
