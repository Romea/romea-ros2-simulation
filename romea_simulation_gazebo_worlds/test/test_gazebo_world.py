# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

# import os
# import pytest

import pytest
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

from romea_gazebo_world import GazeboWorld


def test_create_world():
    world = GazeboWorld("romea_simulation_gazebo_worlds", "romea_small_vineyard.world")
    world.save("/tmp/gazebo.world")

    assert ET.parse("/tmp/gazebo.world").find("world").get("name") == "small_vineyard"


def test_create_world_with_anchor():

    world = GazeboWorld("gazebo", "empty.world")

    anchor = {"latitude": 45.76265802, "longitude": 3.11000985, "altitude": 405.839}
    world.set_wgs84_anchor(anchor)

    world_filename = (
        get_package_share_directory("romea_simulation_gazebo_worlds") +
        "/worlds/test.world"
    )
    world.save(world_filename)

    world_element = ET.parse(world_filename).find("world")
    anchor_element = world_element.find("spherical_coordinates")
    assert world_element.get("name") == "default"
    assert anchor_element.find("surface_model").text == "EARTH_WGS84"
    assert anchor_element.find("world_frame_orientation").text == "ENU"
    assert anchor_element.find("latitude_deg").text == "45.76265802"
    assert anchor_element.find("longitude_deg").text == "3.11000985"
    assert anchor_element.find("elevation").text == "405.839"
    assert anchor_element.find("heading_deg").text == "180"


def test_create_world_with_anchor_when_anchor_is_already_exist():

    world = GazeboWorld("romea_simulation_gazebo_worlds", "test.world")

    anchor = {"latitude": 45.76265802, "longitude": 3.11000985, "altitude": 405.839}
    with pytest.raises(RuntimeError):
        world.set_wgs84_anchor(anchor)
