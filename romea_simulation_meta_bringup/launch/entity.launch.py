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

import romea_common_meta_bringup.ros_launch as common
import romea_simulation_meta_bringup.meta_description as meta_description
import romea_simulation_meta_bringup.ros_launch as simulation


def launch_setup(context, *args, **kwargs):

    robot_namespace = common.get_robot_namespace(context)
    simulator_type = simulation.get_simulator_type(context)
    robot_urdf_description = simulation.generate_entity_urdf_description(context)
    entity_configuration = simulation.get_entity_configuration(context, robot_namespace)
    initial_x = meta_description.get_initial_x(entity_configuration)
    initial_y = meta_description.get_initial_y(entity_configuration)
    initial_z = meta_description.get_initial_z(entity_configuration)
    initial_roll = meta_description.get_initial_roll(entity_configuration)
    initial_pitch = meta_description.get_initial_pitch(entity_configuration)
    initial_yaw = meta_description.get_initial_yaw(entity_configuration)

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_meta_bringup")
            + "/launch/simulators/" + simulator_type + "_entity.launch.py",
        ),
        launch_arguments={
            "entity_name": robot_namespace,
            "robot_urdf_description": robot_urdf_description,
            "initial_x": str(initial_x),
            "initial_y": str(initial_y),
            "initial_z": str(initial_z),
            "initial_roll": str(initial_roll),
            "initial_pitch": str(initial_pitch),
            "initial_yaw": str(initial_yaw)
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
            simulation.declare_entity_type(),
            simulation.declare_simulator_type(),
            simulation.declare_simulation_configuration_file_path(
                default_simulation_configuration_file_path
            ),
            common.declare_robot_namespace(),
            common.declare_meta_description_file_path("entity"),
            OpaqueFunction(function=launch_setup)
        ]
    )
