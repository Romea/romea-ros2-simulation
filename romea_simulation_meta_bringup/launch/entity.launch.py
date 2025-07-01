from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)

from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import ExecutableInPackage

from romea_mobile_base_meta_bringup import MobileBaseMetaDescription
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    simulator_type = LaunchConfiguration("simulator_type").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)
    meta_description_file = LaunchConfiguration("meta_description_file_path").perform(context)
    meta_description = MobileBaseMetaDescription(meta_description_file)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulators/" + simulator_type + "_entity.launch.py",
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

    urdf_description = Command(
        [
            ExecutableInPackage("generate_urdf_description.py", "romea_mobile_base_meta_bringup"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " meta_description_file_path:",
            LaunchConfiguration("meta_description_file_path"),
            " mode:simulation",
        ],
        on_stderr="ignore",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulator_type"),
            DeclareLaunchArgument("robot_namespace"),
            DeclareLaunchArgument("meta_description_file_path"),
            DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description),
            OpaqueFunction(function=launch_setup)
        ]
    )
