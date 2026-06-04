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
import sys
import pytest
import romea_simulation_meta_bringup.meta_description as meta_description
import yaml


@pytest.fixture(scope="module")
def simulation_configuration():

    configuration_filename = os.path.join(os.getcwd(), "test_simulation_meta_bringup.yaml")
    with open(configuration_filename) as f:
        return yaml.safe_load(f)


def test_get_gazebo_classic_world_package(simulation_configuration):
    simulator_configuration = meta_description.get_simulator_configuration(
        simulation_configuration, "gazebo_classic"
    )
    assert (
        meta_description.get_world_package(simulator_configuration)
        == "romea_simulation_gazebo_worlds"
    )


def test_get_gazebo_classic_world_name(simulation_configuration):
    simulator_configuration = meta_description.get_simulator_configuration(
        simulation_configuration, "gazebo_classic"
    )
    assert meta_description.get_world_name(simulator_configuration) == "romea_small_vineyard.world"


def test_get_gazebo_world_package(simulation_configuration):
    simulator_configuration = meta_description.get_simulator_configuration(
        simulation_configuration, "gazebo"
    )
    assert (
        meta_description.get_world_package(simulator_configuration)
        == "romea_simulation_gazebo_worlds"
    )


def test_get_gazebo_world_name(simulation_configuration):
    simulator_configuration = meta_description.get_simulator_configuration(
        simulation_configuration, "gazebo"
    )
    assert meta_description.get_world_name(simulator_configuration) == "gz_empty.sdf"


def test_failed_get_simulator_configuration(simulation_configuration):
    with pytest.raises(KeyError) as excinfo:
        meta_description.get_simulator_configuration(simulation_configuration, "simulator")

    assert "simulator simulator configuration " in str(excinfo.value)


def assert_list_equal(a: list[float], b: list[float]):
    assert len(a) == len(b)
    for x, y in zip(a, b):
        assert x == pytest.approx(y)


def test_get_robot_intial_position_and_orientation(simulation_configuration):
    entity_configuration = meta_description.get_entity_configuration(
        simulation_configuration, "robot"
    )

    assert meta_description.get_initial_x_or(entity_configuration, 0.0) == pytest.approx(1.0)
    assert meta_description.get_initial_y_or(entity_configuration, 0.0) == pytest.approx(2.0)
    assert meta_description.get_initial_z_or(entity_configuration, 0.0) == pytest.approx(3.0)
    assert meta_description.get_initial_roll_or(entity_configuration, 0.0) == pytest.approx(4.0)
    assert meta_description.get_initial_pitch_or(entity_configuration, 0.0) == pytest.approx(5.0)
    assert meta_description.get_initial_yaw_or(entity_configuration, 0.0) == pytest.approx(6.0)
    
    entity2 = meta_description.get_entity_configuration(simulation_configuration, "robot2")

    assert_list_equal(meta_description.get_initial_xyz(entity2), [7., 8., 9.])
    assert_list_equal(meta_description.get_initial_rpy(entity2), [10., 11., 12.])
