# romea_ros2_simulation_bringup

### 1) Overview ###

The romea_ros2_simulation_bringup package provides  : 

 - **Launch files**  that can start a simulator based on a user-specified configuration files (see the next section for an overview of the simulation configuration files). Currently, only the Gazebo simulator is supported.

   It is possible to launch a simulator via command line : 

    ```bash
    ros2 launch romea_simulation_bringup simulator.launch.py simulation_configuration_file_path:=path_to_demo/config/simulation.yaml  wgs84_anchor_file_path:= path_to_demo/config/wgs84anchor.yaml simulator_type :=gazebo
    ```

   Where :

   - *simulator_type* is the name of simulator 
   - *simulation_configuration_file_path* is the absolute path of simulation configuration file
   - *wgs84_anchor_file_path* is the absolute path of wgs84 anchor configuration file    

 - **A Python module** able to load and parse simulation configuration file 


### 2) Simulator configuration ###

The simulator configuration file, as shown below, is a YAML file composed of two main elements. The first element specifies the name of the package that contains the world to be loaded by the simulator. This element is optional if the world name is unique. The second element is the name of the world itself.

Example :
```yaml
  world_package: romea_simulation_gazebo_worlds # optional 
  world_name: romea_small_vineyard.world
```

### 3) WGS84 anchor configuration ###

The geodetic coordinates (longitude, latitude, altitude) for the world's origin are provided in the anchor configuration file. For Gazebo worlds, this information will be stored in the spherical_coordinates XML element before the simulator loads the world (refer to the Python module in the romea_ros2_simulation_gazebo_worlds package for more details).  

Example :
```yaml
  latitude: 45.76265802 #degree
  longitude: 3.11000985 #degree
  altitude: 405.839 #m
```
