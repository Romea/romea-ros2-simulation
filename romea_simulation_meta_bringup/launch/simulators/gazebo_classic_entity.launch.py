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

import math

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

import romea_common_meta_bringup.ros_launch as common
import romea_simulation_meta_bringup.ros_launch as simulation


def launch_setup(context, *args, **kwargs):

    robot_urdf_description = common.get_robot_urdf_description(context)
    entity_name = simulation.get_entity_name(context)
    initial_x = simulation.get_initial_x(context)
    initial_y = simulation.get_initial_y(context)
    initial_z = simulation.get_initial_z(context)
    initial_roll = simulation.get_initial_roll(context)
    initial_pitch = simulation.get_initial_pitch(context)
    initial_yaw = simulation.get_initial_yaw(context)

    robot_description_file = f"/tmp/{entity_name}_description.urdf"
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
            entity_name,
            "-x",
            initial_x,
            "-y",
            initial_y,
            "-z",
            initial_z,
            "-R",
            str(float(initial_roll)*math.pi/180.),
            "-P",
            str(float(initial_pitch)*math.pi/180.),
            "-Y",
            str(float(initial_yaw)*math.pi/180.),
        ],
        output={
            'stdout': 'log',
            'stderr': 'log',
        }
    )

    return [spawn_entity]


def generate_launch_description():

    return LaunchDescription(
        [
            simulation.declare_entity_name(),
            simulation.declare_initial_x(),
            simulation.declare_initial_y(),
            simulation.declare_initial_z(),
            simulation.declare_initial_roll(),
            simulation.declare_initial_pitch(),
            simulation.declare_initial_yaw(),
            common.declare_robot_urdf_description(),
            OpaqueFunction(function=launch_setup),
        ]
    )
