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
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    simulator_type = LaunchConfiguration("simulator_type").perform(context)

    simulation_configuration_file_path = LaunchConfiguration(
        "simulation_configuration_file_path"
    ).perform(context)

    wgs84_anchor_file_path = LaunchConfiguration("wgs84_anchor_file_path").perform(context)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulators/"+ simulator_type + "_simulator.launch.py",
        ),
        launch_arguments={
            "wgs84_anchor_file_path": wgs84_anchor_file_path,
            "simulation_configuration_file_path": simulation_configuration_file_path,
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulator_type"),
            DeclareLaunchArgument(
                "simulation_configuration_file_path",
                default_value=get_package_share_directory("romea_simulation_meta_bringup")
                + "/config/empty_simulation.yaml",
            ),
            DeclareLaunchArgument("wgs84_anchor_file_path", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
