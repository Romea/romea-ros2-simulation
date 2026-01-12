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

def get_simulator_configuration(simulation_configuration, simulator):    
    try:
        return simulation_configuration["simulators"][simulator]
    except :
        raise KeyError(f"{simulator} simulator configuration is not found in simulation configuration file")


def get_world_package(simulator_configuration):
    return simulator_configuration.get("world_package")


def get_world_name(simulator_configuration):
    return simulator_configuration.get("world_name")


# def has_wgs84_anchor(simulation_configuration):
#     return "wgs84_anchor" in simulation_configuration


# def get_wgs84_anchor(simulation_configuration):
#     return simulation_configuration["wgs84_anchor"]
