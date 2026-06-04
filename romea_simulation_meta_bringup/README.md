# romea_simulation_meta_bringup

## 1) Overview

`romea_simulation_meta_bringup` provides the simulation orchestration layer used by robot-level meta-bringup packages.

It is responsible for:

* selecting and starting a simulator from a simulation configuration file
* loading the world associated with the selected simulator
* injecting a WGS84 anchor into Gazebo worlds when one is provided
* generating an entity URDF through the corresponding `<entity_type>_meta_bringup` package
* spawning the entity in the simulator at the pose defined in the simulation configuration

The package does not describe robots, sensors or implements itself. It delegates entity description generation to packages such as `romea_robot_meta_bringup`, then passes the generated URDF to the selected simulator integration.

Currently, the provided launch backends support:

| Simulator type | Backend |
|:---------------|:--------|
| `gazebo` | Gazebo through `ros_gz_sim` |
| `gazebo_classic` | Gazebo Classic through `gazebo_ros` |

Other simulator names may be accepted by common launch arguments, but this package only provides launch files for the backends listed above.

Additional backends are planned for future integration, notably Isaac Sim and 4D Virtualiz.

---

## 2) Simulation configuration

A simulation configuration file describes which world must be loaded for each simulator backend and where named entities must be spawned.

```yaml
simulators:
  gazebo_classic:
    world_package: romea_simulation_gazebo_worlds
    world_name: romea_small_vineyard.world
  gazebo:
    world_package: romea_simulation_gazebo_worlds
    world_name: gz_empty.sdf

entities:
  - name: robot
    initial_x: 1.0
    initial_y: 2.0
    initial_z: 0.0
    initial_roll: 0.0
    initial_pitch: 0.0
    initial_yaw: 90.0
```

The `simulators` section contains one entry per supported simulator type. Each entry provides:

| Field | Description |
|:------|:------------|
| `world_package` | package containing the world file, usually `romea_simulation_gazebo_worlds` |
| `world_name` | world file loaded by the simulator |

The `entities` section is optional. It defines the initial pose of entities spawned by `entity.launch.py`. Missing pose fields default to `0.0`.

Each entity entry is matched by its `name`. For robot simulations, this name usually matches the `robot_namespace` passed to `entity.launch.py` or to the higher-level robot simulation launch.

| Field | Unit | Description |
|:------|:----:|:------------|
| `name` | - | entity name used to select the pose and name the spawned model |
| `initial_x` | m | initial position along the world x axis |
| `initial_y` | m | initial position along the world y axis |
| `initial_z` | m | initial position along the world z axis |
| `initial_roll` | deg | initial roll angle |
| `initial_pitch` | deg | initial pitch angle |
| `initial_yaw` | deg | initial yaw angle |

---

## 3) Launch files

### Start a simulator

`simulator.launch.py` starts the selected simulator and loads the world declared in the simulation configuration.

```bash
ros2 launch romea_simulation_meta_bringup simulator.launch.py \
  simulator_type:=gazebo \
  simulation_configuration_file_path:=path/to/simulation.yaml \
  wgs84_anchor_file_path:=path/to/wgs84_anchor.yaml
```

The launch file delegates to:

* `launch/simulators/gazebo_simulator.launch.py` for `simulator_type:=gazebo`
* `launch/simulators/gazebo_classic_simulator.launch.py` for `simulator_type:=gazebo_classic`

When `wgs84_anchor_file_path` is provided, the selected Gazebo world is copied to `/tmp/gazebo_world.world` and completed with spherical coordinates before the simulator starts.

### Spawn an entity

`entity.launch.py` generates an entity URDF and spawns it in the selected simulator.

```bash
ros2 launch romea_simulation_meta_bringup entity.launch.py \
  entity_type:=robot \
  simulator_type:=gazebo \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/robot_meta_description.yaml \
  simulation_configuration_file_path:=path/to/simulation.yaml
```

`entity_type:=robot` is the usual case for full robot simulations. The same mechanism can also spawn a standalone entity, such as a sensor, an implement or a manipulator, as long as the corresponding `romea_<entity_type>_meta_bringup` package provides URDF generation.

The launch file:

* reads the entity pose from the `entities` section using `robot_namespace` as the entity name
* calls `romea_<entity_type>_meta_bringup generate_urdf_description.py` in `simulation_<simulator_type>` mode
* writes the generated URDF to `/tmp/<robot_namespace>_description.urdf`
* delegates the spawn operation to the selected simulator backend

For `entity_type:=robot`, the generated URDF comes from `romea_robot_meta_bringup` and therefore includes the mobile base reference URDF plus the URDF fragments of the devices declared in the robot meta-description.

---

## 4) WGS84 anchor

A WGS84 anchor file defines the geodetic origin of the simulated world:

```yaml
latitude: 45.76265802
longitude: 3.11000985
altitude: 405.839
```

For Gazebo worlds, this information is inserted into the world `spherical_coordinates` element before loading the world. This allows GPS simulation components to use a consistent relation between the simulated world frame and geodetic coordinates.

If the world already contains spherical coordinates, the existing values are kept.

---

## 5) Python helpers

The package provides two helper modules:

| Module | Role |
|:-------|:-----|
| `meta_description.py` | reads simulator configuration, world names and entity initial poses |
| `ros_launch.py` | declares and resolves launch arguments used by simulator and entity launch files |

These helpers are used by `romea_robot_meta_bringup/simulation_test.launch.py` to start a simulator, spawn the robot entity and then start the robot components in `simulation_<simulator_type>` mode.
