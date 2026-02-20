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
from launch_ros.actions import Node

from romea_gazebo_world import GazeboWorld
import romea_gps_meta_bringup.ros_launch as gps
import romea_simulation_meta_bringup.meta_description as meta_description
import romea_simulation_meta_bringup.ros_launch as simulation

from ros_gz_sim.actions import GzServer


def launch_setup(context, *args, **kwargs):

    wgs84_anchor = gps.get_wgs84_anchor(context)
    simulator_config = simulation.get_simulator_configuration(context, "gazebo")

    world = GazeboWorld(
        "gazebo",
        meta_description.get_world_package(simulator_config),
        meta_description.get_world_name(simulator_config)
    )

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
            simulation.declare_simulation_configuration_file_path(),
            gps.declare_wgs84_anchor_file_path(""),
            OpaqueFunction(function=launch_setup)
        ]
    )
