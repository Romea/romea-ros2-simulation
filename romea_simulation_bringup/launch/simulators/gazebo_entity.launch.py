from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

from romea_common_bringup import robot_urdf_prefix


def launch_setup(context, *args, **kwargs):

    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)

    robot_description_file = "/tmp/"+robot_urdf_prefix(robot_namespace)+"description.urdf"
    with open(robot_description_file, "w") as f:
        f.write(robot_urdf_description)

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        exec_name="gazebo_spawn_entity",
        arguments=[
            "-file",
            robot_description_file,
            "-entity",
            robot_namespace,
        ],
        output={
            'stdout': 'log',
            'stderr': 'log',
        }
    )

    return [GroupAction([PushRosNamespace(robot_namespace), spawn_entity])]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument("robot_namespace")
    )

    declared_arguments.append(
        DeclareLaunchArgument("robot_urdf_description")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
