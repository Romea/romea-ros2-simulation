# 1) Overview #

The romea_simulation_bringup package provides  : 

 - launch files able to start simulator according a configuration file provided by user (see next section for simulation configuration file overview), for the moment only Gazebo simulator is supported.

   It is possible to launch a simulator via command line : 

    ```console
    -ros2 launch romea_simulation_bringup simulator.launch.py simulation_configuration_file_path:=path_to_demo/config/simulation.yaml  simulator_type :=gazebo
    ```

   where :

   - *simulator_type* is the name of simulator 
   - *simulation_configuration_file_path* is the absolute path of simulation configuration file    

 - a python module able to load and parse simulation configuration file 


# 2) Simulator configuration #

As seen below simulator configuration file is a yaml file constituted by three items. The first item is the name of package in which the world that will be loaded by simulator is located. This item is optional if the world name is unique. The second one is the name of world. The last item is the wgs84 anchor of the world. This item is not needed if wgs84 anchor is already defined into the world file otherwise it would set into world file before to be loaded by the simulator.  

Example :
```yaml
  world_package: romea_simulation_gazebo_worlds # optional 
  world_name: romea_small_vineyard.world
  wgs84_anchor:
    latitude: 45.76265802 #degree
    longitude: 3.11000985 #degree
    altitude: 405.839 #m
```
