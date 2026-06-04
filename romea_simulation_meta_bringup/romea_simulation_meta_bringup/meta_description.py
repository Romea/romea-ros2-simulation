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


def get_simulator_configuration(simulation_configuration: dict, simulator_name: str):
    try:
        return simulation_configuration["simulators"][simulator_name]
    except KeyError:
        raise KeyError(
            f"{simulator_name} simulator configuration"
            + " is not found in simulation configuration file"
        )


def get_entity_configuration(simulation_configuration: dict, entity_name: str):
    for entity in simulation_configuration.get("entities", []):
        if entity_name == entity.get("name"):
            return entity
    return {}


def get_world_package(simulator_configuration: dict):
    return simulator_configuration.get("world_package")


def get_world_name(simulator_configuration: dict):
    return simulator_configuration.get("world_name")


def get_initial_xyz(entity_configuration: dict):
    return entity_configuration.get("initial_xyz", [0.0, 0.0, 0.0])


def get_initial_x_or(entity_configuration: dict, default: float):
    return entity_configuration.get("initial_x", default)


def get_initial_y_or(entity_configuration: dict, default: float):
    return entity_configuration.get("initial_y", default)


def get_initial_z_or(entity_configuration: dict, default: float):
    return entity_configuration.get("initial_z", default)


def get_initial_rpy(entity_configuration: dict):
    return entity_configuration.get("initial_rpy", [0.0, 0.0, 0.0])


def get_initial_roll_or(entity_configuration: dict, default: float):
    return entity_configuration.get("initial_roll", default)


def get_initial_pitch_or(entity_configuration: dict, default: float):
    return entity_configuration.get("initial_pitch", default)


def get_initial_yaw_or(entity_configuration: dict, default: float):
    return entity_configuration.get("initial_yaw", default)
