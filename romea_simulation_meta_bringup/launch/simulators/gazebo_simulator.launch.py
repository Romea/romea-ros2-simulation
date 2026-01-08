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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    # GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from romea_gazebo_world import GazeboWorld
from romea_simulation_meta_bringup import (
    get_simulator_configuration,
    get_world_name,
    get_world_package,
)

from ros_gz_sim.actions import GzServer

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
    simulator_config= get_simulator_configuration(simulation_config, "gazebo")
    world = GazeboWorld(get_world_package(simulator_config), get_world_name(simulator_config))

    if wgs84_anchor is not None:
        world.set_wgs84_anchor(wgs84_anchor)

    world.save("/tmp/gazebo_world.world")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('ros_gz_sim')
                + "/launch/gz_sim.launch.py"
            ),
            launch_arguments={
                # 'gz_args': '/tmp/gazebo_world.world',
                'gz_args': '-g',
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        GzServer(
            world_sdf_file='/tmp/gazebo_world.world',
            world_sdf_string='world',
            container_name='gz_container',
            create_own_container=True,
            use_composition=True,
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_clock_bridge",
            parameters=[
                {"bridge_names": ["clock_bridge"]},
                {"bridges.clock_bridge.ros_topic_name": "/clock"},
                {"bridges.clock_bridge.gz_topic_name": "/clock"},
                {"bridges.clock_bridge.ros_type_name": "rosgraph_msgs/msg/Clock"},
                {"bridges.clock_bridge.gz_type_name": "gz.msgs.Clock"},
                {"bridges.clock_bridge.direction": "GZ_TO_ROS"},
                {"bridges.clock_bridge.lazy": True},
                {"bridges.clock_bridge.qos_profile": "CLOCK"},
            ],
        )
    ]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("simulation_configuration_file_path"),
            DeclareLaunchArgument("wgs84_anchor_file_path", default_value=""),
            OpaqueFunction(function=launch_setup)
        ]
    )
