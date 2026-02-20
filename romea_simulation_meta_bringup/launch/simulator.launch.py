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
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

import romea_gps_meta_bringup.ros_launch as gps
import romea_simulation_meta_bringup.ros_launch as simulation


def launch_setup(context, *args, **kwargs):

    simulator_type = simulation.get_simulator_type(context)
    wgs84_anchor_file_path = gps.get_wgs84_anchor_file_path(context)    
    simulation_config_file_path = simulation.get_simulation_configuration_file_path(context)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulators/" + simulator_type + "_simulator.launch.py",
        ),
        launch_arguments={
            "wgs84_anchor_file_path": wgs84_anchor_file_path,
            "simulation_configuration_file_path": simulation_config_file_path,
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    default_simulation_configuration_file_path = (
        get_package_share_directory("romea_simulation_meta_bringup")
        + "/config/empty_simulation.yaml"
    )

    return LaunchDescription(
        [
            simulation.declare_simulator_type(),
            simulation.declare_simulation_configuration_file_path(
                default_simulation_configuration_file_path
            ),
            gps.declare_wgs84_anchor_file_path(""),
            OpaqueFunction(function=launch_setup),
        ]
    )
