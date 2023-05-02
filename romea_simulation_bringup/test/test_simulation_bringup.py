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


import os
import pytest
import yaml

from romea_simulation_bringup import (
    get_world_package,
    get_world_name,
    get_wgs84_anchor,
)


@pytest.fixture(scope="module")
def simulation_configuration():

    configuration_filename = os.path.join(os.getcwd(), "test_simulation_bringup.yaml")
    with open(configuration_filename) as f:
        return yaml.safe_load(f)


def test_get_world_package(simulation_configuration):
    assert get_world_package(simulation_configuration) == "romea_simulation_gazebo_worlds"


def test_get_world_name(simulation_configuration):
    assert get_world_name(simulation_configuration) == "romea_small_vineyard.world"


def test_get_wgs84_anchor(simulation_configuration):
    anchor = get_wgs84_anchor(simulation_configuration)
    assert anchor["latitude"] == 45.76265802
    assert anchor["longitude"] == 3.11000985
    assert anchor["altitude"] == 405.839
