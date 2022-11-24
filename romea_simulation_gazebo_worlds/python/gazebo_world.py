#!/usr/bin/env python3

import xml.etree.ElementTree as ET

# import os
import subprocess
import shlex
import re

from ament_index_python.packages import get_package_share_directory


class GazeboWorld:
    def __init__(self, world_package, world_name):

        self._world_filename = self._get_world_filename(world_package, world_name)
        self._world_tree = ET.parse(self._world_filename)
        
    def set_wgs84_anchor(self, wgs84_anchor):

        world_element = self._world_tree.find("world")
        spherical_coordinates_element = world_element.find("spherical_coordinates")

        if spherical_coordinates_element is not None:

            raise RuntimeError(
                "Cannot add user spherical coordinates into world "
                + self._world_filename
                + " because it's already exists."
                + " Gazebo will not be launch."
            )

        else:

            spherical_coordinates_element = ET.SubElement(
                world_element, "spherical_coordinates"
            )
            ET.SubElement(
                spherical_coordinates_element, "surface_model"
            ).text = "EARTH_WGS84"

            ET.SubElement(
                spherical_coordinates_element, "world_frame_orientation"
            ).text = "ENU"

            ET.SubElement(spherical_coordinates_element, "latitude_deg").text = str(
                wgs84_anchor["latitude"]
            )

            ET.SubElement(spherical_coordinates_element, "longitude_deg").text = str(
                wgs84_anchor["longitude"]
            )

            ET.SubElement(spherical_coordinates_element, "elevation").text = str(
                wgs84_anchor["altitude"]
            )

            ET.SubElement(spherical_coordinates_element, "heading_deg").text = "180"
        pass

    def save(self, demo_world_filename):
        self._world_tree.write(demo_world_filename)

    def _gazebo_version(self):

        cmd = shlex.split("gazebo --version")

        gazebo_version = subprocess.run(cmd, stdout=subprocess.PIPE).stdout.decode(
            "utf-8"
        )

        print(gazebo_version)
        return re.search(r"version\s*([\d.]+)", gazebo_version).group(1).split(".")[0]

    def _get_world_filename(self,world_package, world_name):

        if world_package == "gazebo":
            return (
                "/usr/share/gazebo-" + self._gazebo_version() + "/worlds/" + world_name
            )
        else:
            return get_package_share_directory(world_package) + "/worlds/" + world_name
