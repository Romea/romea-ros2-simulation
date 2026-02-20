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


def get_simulator_configuration(simulation_configuration, simulator_name):
    try:
        return simulation_configuration["simulators"][simulator_name]
    except KeyError:
        raise KeyError(
            f"{simulator_name} simulator configuration"
            + " is not found in simulation configuration file"
        )


def get_entity_configuration(simulation_configuration, entity_name):
    for entity in simulation_configuration.get("entities", []):
        if entity_name == entity.get("name"):
            return entity
    return {}


def get_world_package(simulator_configuration):
    return simulator_configuration.get("world_package")


def get_world_name(simulator_configuration):
    return simulator_configuration.get("world_name")


def get_initial_x(entity_configuration):
    return entity_configuration.get("initial_x", 0.0)


def get_initial_y(entity_configuration):
    return entity_configuration.get("initial_y", 0.0)


def get_initial_z(entity_configuration):
    return entity_configuration.get("initial_z", 0.0)


def get_initial_roll(entity_configuration):
    return entity_configuration.get("initial_roll", 0.0)


def get_initial_pitch(entity_configuration):
    return entity_configuration.get("initial_pitch", 0.0)


def get_initial_yaw(entity_configuration):
    return entity_configuration.get("initial_yaw", 0.0)
