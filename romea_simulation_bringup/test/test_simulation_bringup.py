# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

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
