#!/usr/bin/env python3

import xml.etree.ElementTree as ET

import os
import sys

from ament_index_python.packages import get_package_share_directory


class GazeboWorld:
    def __init__(self, world_package, world_name):

        self._world_filename = self._get_world_filename(world_package, world_name)
        # print("world filename inner ", self._world_filename)
        self._world_tree = ET.parse(self._world_filename)

    def has_wgs84_anchor(self):
        return self._world_tree.find("world") is not None

    def get_wgs84_anchor(self):
        world_element = self._world_tree.find("world")
        spherical_coordinates_element = world_element.find(
            "spherical_coordinates")

        if spherical_coordinates_element is None:

            raise RuntimeError(
                "Cannot get spherical coordinates from world "
                + self._world_filename
                + " because it's not defined."
            )

        return {
            "latitude": float(
                ET.SubElement(spherical_coordinates_element,
                              "latitude_deg").text
            ),
            "longitude": float(
                ET.SubElement(spherical_coordinates_element,
                              "longitude_deg").text
            ),
            "altitude": float(
                ET.SubElement(spherical_coordinates_element, "elevation").text
            ),
        }

    def set_wgs84_anchor(self, wgs84_anchor):

        world_element = self._world_tree.find("world")
        spherical_coordinates_element = world_element.find("spherical_coordinates")

        if spherical_coordinates_element is not None:
            print(
                f"Warning: Spherical coordinates detected in {self._world_filename}: "
                "use of the existing one",
                file=sys.stderr
            )
            # raise RuntimeError(
            #     "Cannot add user spherical coordinates into world "
            #     + self._world_filename
            #     + " because it's already exists."
            #     + " Gazebo will not be launch."
            # )

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

    def save(self, demo_world_filename):
        self._world_tree.write(demo_world_filename)

    def _get_world_filename(self, world_package, world_name):
        if world_package is not None and world_package != 'gazebo':
            return get_package_share_directory(world_package) + "/worlds/" + world_name
        else:
            if 'GAZEBO_RESOURCE_PATH' not in os.environ:
                raise RuntimeError('Missing environment variable GAZEBO_RESOURCE_PATH')

            resource_paths = os.environ['GAZEBO_RESOURCE_PATH'].split(':')
            for path in resource_paths:
                filename = f"{path}/worlds/{world_name}"
                if os.path.isfile(filename):
                    return filename

            raise FileNotFoundError(f"No file 'worlds/{world_name}' found in GAZEBO_RESOURCE_PATH")
