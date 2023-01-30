# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license


def get_world_package(simulation_configuration):
    return simulation_configuration["world_package"]


def get_world_name(simulation_configuration):
    return simulation_configuration["world_name"]


def has_wgs84_anchor(simulation_configuration):
    return "wgs84_anchor" in simulation_configuration


def get_wgs84_anchor(simulation_configuration):
    return simulation_configuration["wgs84_anchor"]
