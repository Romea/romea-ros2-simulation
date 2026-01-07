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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
)

from launch.substitutions import LaunchConfiguration

from romea_gazebo_world import GazeboWorld
from romea_simulation_meta_bringup import (
    get_simulator_configuration,
    get_world_name,
    get_world_package,
)

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
    simulation_config = get_simulation_configuration(context)
    simulator_config= get_simulator_configuration(simulation_config, "gazeo_classic")
    world = GazeboWorld(get_world_package(simulator_config), get_world_name(simulator_config))

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
