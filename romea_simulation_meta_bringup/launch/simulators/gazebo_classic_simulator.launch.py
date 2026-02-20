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
from launch.actions import ExecuteProcess, GroupAction, OpaqueFunction

from romea_gazebo_world import GazeboWorld
import romea_gps_meta_bringup.ros_launch as gps
import romea_simulation_meta_bringup.meta_description as meta_description
import romea_simulation_meta_bringup.ros_launch as simulation


def launch_setup(context, *args, **kwargs):

    wgs84_anchor = gps.get_wgs84_anchor(context)
    simulator_config = simulation.get_simulator_configuration(context, "gazebo_classic")

    world = GazeboWorld(
        "gazebo",
        meta_description.get_world_package(simulator_config),
        meta_description.get_world_name(simulator_config)
    )

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
            simulation.declare_simulation_configuration_file_path(),
            gps.declare_wgs84_anchor_file_path(""),
            OpaqueFunction(function=launch_setup)
        ]
    )
