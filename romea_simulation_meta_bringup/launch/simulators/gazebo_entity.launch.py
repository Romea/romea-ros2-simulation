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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

from romea_common_meta_bringup.utils import robot_urdf_prefix


def launch_setup(context, *args, **kwargs):

    entity_name = LaunchConfiguration("entity_name").perform(context)
    robot_urdf_description = LaunchConfiguration("robot_urdf_description").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context).split(' ')
    rpy = LaunchConfiguration("rpy").perform(context).split(' ')

    robot_description_file = f"/tmp/{entity_name}_description.urdf"
    with open(robot_description_file, "w") as f:
        f.write(robot_urdf_description)

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('ros_gz_sim')
            + "/launch/gz_spawn_model.launch.py"
        ),
        launch_arguments=[
            ('file', robot_description_file),
            ('entity_name', entity_name),
            ('allow_renaming', 'false'),
            ('x', xyz[0]),
            ('y', xyz[1]),
            ('z', xyz[2]),
            ('R',  str(float(rpy[0])*math.pi/180.)),
            ('P',  str(float(rpy[1])*math.pi/180.)),
            ('Y',  str(float(rpy[2])*math.pi/180.)),
        ],
    )

    return [spawn_entity]


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("entity_name"),
            DeclareLaunchArgument("robot_urdf_description"),
            DeclareLaunchArgument("xyz", default_value='0.0 0.0 0.0'),
            DeclareLaunchArgument("rpy", default_value='0.0 0.0 0.0'),
            OpaqueFunction(function=launch_setup),
        ]
    )
