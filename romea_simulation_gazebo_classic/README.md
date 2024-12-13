# ROMEA Simulation Gazebo Classic

This package is a fork of gazebo_ros2_control that allows the launching of multiple Controler Manager Gazebo plugins to control several devices separately for the same simulation entity. The goal is to make the simulation more reflective of reality, which often involves having a separate controller for each devices. For example, a robot equipped with a robotic arm will have one controller for the mobile base and another for the robotic arm.

To declare a controller manager plugin, simply include the URDF file describing the entity within the following tag:

```xml
<gazebo>
    <plugin filename="libromea_gazebo_ros2_control.so" name="gazebo_ros2_control">
    <ros2_control_config_file>${_device_ros2_control_config_urdf_file}</ros2_control_config_file>
    <controller_manager_config_file>${device_controller_manager_config_yaml_file}</controller_manager_config_file>
    <ros>
        <namespace>${device_namespace}</namespace>
    </ros>
    </plugin>
</gazebo>
```

where:

- ​    **ros2_control_config_file**: The path to the ROS 2 control hardware configuration file in URDF format for the device to be controlled.
- ​    **controller_manager_config_file**: The path of A yaml file specifying the type of controller that can be loaded.
- ​    **ros.namespace**: The namespace for the controller manager.


To learn how to write configuration files, please refer to the documentation for [ros2_control](https://control.ros.org/master/doc/getting_started/getting_started.html). For examples, you can see the description packages for each of the robots supported in the ROMEA ecosystem.