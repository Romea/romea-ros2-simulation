# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import ExecutableInPackage

from romea_common_meta_bringup.meta_description import SensorMetaDescription
from romea_common_meta_bringup.utils import robot_urdf_prefix

def launch_setup(context, *args, **kwargs):

    entity_type = LaunchConfiguration("simulator_type").perform(context)
    simulator_type = LaunchConfiguration("simulator_type").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)
    meta_description_file = LaunchConfiguration("meta_description_file_path").perform(context)
    meta_description = SensorMetaDescription(entity_type, meta_description_file, robot_namespace)
    entity_name = robot_urdf_prefix(robot_namespace) + meta_description.get_name()
 
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulators/" + simulator_type + "_entity.launch.py",
        ),
        launch_arguments={
            "entity_name": entity_name,
            "robot_urdf_description": robot_urdf_description,
            "xyz": ' '.join(map(str, meta_description.get_simulation().get("xyz", [0., 0., 0.]))),
            "rpy": ' '.join(map(str, meta_description.get_simulation().get("rpy", [0., 0., 0.]))),
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    urdf_description = Command(
        [
            ExecutableInPackage(
                "generate_urdf_description.py",
                [
                    TextSubstitution(text="romea_"),
                    LaunchConfiguration("entity_type"),
                    TextSubstitution(text="_meta_bringup"),
                ],
            ),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " meta_description_file_path:",
            LaunchConfiguration("meta_description_file_path"),
            " mode:simulation",
            " standalone:true",
        ],
        on_stderr="ignore",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("entity_type"),
            DeclareLaunchArgument("simulator_type"),
            DeclareLaunchArgument("robot_namespace"),
            DeclareLaunchArgument("meta_description_file_path"),
            DeclareLaunchArgument("robot_urdf_description", default_value=urdf_description),
            OpaqueFunction(function=launch_setup)
        ]
    )
