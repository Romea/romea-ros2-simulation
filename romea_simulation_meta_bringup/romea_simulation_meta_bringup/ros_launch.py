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


from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage

import romea_common_meta_bringup.ros_launch as common
import romea_simulation_meta_bringup.meta_description as meta_description

import yaml


def declare_simulation_configuration_file_path(default_value=None):
    return common.declare_argument(
        {
            "name": "simulation_configuration_file_path",
            "description": "Simulation configuration filename",
        },
        default_value,
    )


def declare_entity_type(default_value=None):
    return common.declare_argument(
        {
            "name": "entity_type",
            "description": "Type of the entity to be spawn into the simulator",
        },
        default_value,
    )


def declare_entity_name(default_value=None):
    return common.declare_argument(
        {
            "name": "entity_name",
            "description": "Name of the entity to be spawn into the simulator",
        },
        default_value,
    )


def declare_simulator_type(default_value="gazebo"):
    return common.declare_argument(
        {
            "name": "simulator_type",
            "description": "Name of the simulator that will run the demo",
            "choices": [
                "gazebo",
                "gazebo_classic",
                "isaac_sim",
                "4dv",
            ],
        },
        default_value,
    )


def declare_initial_x(default_value="0.0"):
    return common.declare_argument(
        {
            "name": "initial_x",
            "description": "Initial x position of the entity into the simulator given in meter",
        },
        default_value,
    )


def declare_initial_y(default_value="0.0"):
    return common.declare_argument(
        {
            "name": "initial_y",
            "description": "Initial y position of the entity into the simulator given in meter",
        },
        default_value,
    )


def declare_initial_z(default_value="0.0"):
    return common.declare_argument(
        {
            "name": "initial_z",
            "description": "Initial z position of the entity into the simulator given in meter",
        },
        default_value,
    )


def declare_initial_roll(default_value="0.0"):
    return common.declare_argument(
        {
            "name": "initial_roll",
            "description": (
                "Initial roll orientation of the entity into the simulator given in degree",
            )
        },
        default_value,
    )


def declare_initial_pitch(default_value="0.0"):
    return common.declare_argument(
        {
            "name": "initial_pitch",
            "description": (
                "Initial pitch orientation of the entity into the simulator given in degree",
            )
        },
        default_value,
    )


def declare_initial_yaw(default_value="0.0"):
    return common.declare_argument(
        {
            "name": "initial_yaw",
            "description": (
                "Initial yaw orientation of the entity into the simulator given in degree",
            )
        },
        default_value,
    )


def get_simulation_configuration_file_path(context):
    return LaunchConfiguration("simulation_configuration_file_path").perform(context)


def get_simulation_configuration(context):
    with open(get_simulation_configuration_file_path(context)) as f:
        return yaml.safe_load(f)


def get_simulator_configuration(context, simulator_name):
    return meta_description.get_simulator_configuration(
        get_simulation_configuration(context), simulator_name
    )


def get_entity_configuration(context, entity_name):
    return meta_description.get_entity_configuration(
        get_simulation_configuration(context), entity_name
    )


def get_entity_type(context):
    return LaunchConfiguration("entity_type").perform(context)


def get_entity_name(context):
    return LaunchConfiguration("entity_name").perform(context)


def get_simulator_type(context):
    return LaunchConfiguration("simulator_type").perform(context)


def get_initial_x(context):
    return LaunchConfiguration("initial_x").perform(context)


def get_initial_y(context):
    return LaunchConfiguration("initial_y").perform(context)


def get_initial_z(context):
    return LaunchConfiguration("initial_z").perform(context)


def get_initial_roll(context):
    return LaunchConfiguration("initial_roll").perform(context)


def get_initial_pitch(context):
    return LaunchConfiguration("initial_pitch").perform(context)


def get_initial_yaw(context):
    return LaunchConfiguration("initial_yaw").perform(context)


def generate_entity_urdf_description(context):
    entity_type = get_entity_type(context)
    simulator_type = get_simulator_type(context)
    robot_namespace = common.get_robot_namespace(context)
    meta_description_file_path = common.get_meta_description_file_path(context)
    return Command(
        [
            ExecutableInPackage(
                "generate_urdf_description.py", f"romea_{entity_type}_meta_bringup"
            ),
            " mode:",
            f"simulation_{simulator_type}",
            " standalone:",
            "true",
            " robot_namespace:",
            robot_namespace,
            " meta_description_file_path:",
            meta_description_file_path,
        ],
        on_stderr="ignore",
    ).perform(context)
