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
    xyz = LaunchConfiguration("xyz").perform(context).split(' ')
    rpy = LaunchConfiguration("rpy").perform(context).split(' ')

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
            "-x",
            xyz[0],
            "-y",
            xyz[1],
            "-z",
            xyz[2],
            "-R",
            rpy[0],
            "-P",
            rpy[1],
            "-Y",
            rpy[2],
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

    declared_arguments.append(
        DeclareLaunchArgument("xyz", default_value=['0', '0', '0'])
    )

    declared_arguments.append(
        DeclareLaunchArgument("rpy", default_value=['0', '0', '0'])
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
