# romea_simulation_gazebo_worlds

## 1) Overview

`romea_simulation_gazebo_worlds` provides Gazebo and Gazebo Classic world resources used by ROMEA simulation launch files.

It contains:

* agricultural world files used in demos and tests
* reusable Gazebo models, such as crop rows, grass planes and ISO obstacles
* environment hooks that expose these resources to Gazebo and Gazebo Classic
* a small Python helper used to locate world files and inject WGS84 anchors

The package is used by `romea_simulation_meta_bringup` when a simulation configuration references:

```yaml
world_package: romea_simulation_gazebo_worlds
world_name: <world_file>
```

---

## 2) Package contents

| Directory | Content |
|:----------|:--------|
| `worlds/` | Gazebo and Gazebo Classic world files |
| `models/` | reusable simulation models referenced by the worlds |
| `launch/` | simple world launch file kept for standalone tests |
| `python/` | `romea_gazebo_world.py`, helper for world lookup and WGS84 anchor insertion |
| `env-hooks/` | environment hooks that expose worlds and models to Gazebo resource paths |

Example worlds include:

| World | Use |
|:------|:----|
| `gz_empty.sdf` | empty Gazebo world |
| `gz_empty_gui.sdf` | empty Gazebo world with GUI-oriented setup |
| `gz_wgs84_empty.sdf` | empty Gazebo world with WGS84 coordinates |
| `friction_cone.world` | Gazebo Classic test world |
| `romea_small_vineyard.world` | agricultural vineyard world |
| `simple_crop_field.world` | simple crop field world |

---

## 3) GazeboWorld helper

The `romea_gazebo_world.GazeboWorld` helper is used by `romea_simulation_meta_bringup` before starting Gazebo.

It keeps two operations together in one place: locating the world file and preparing a generated world file for the simulator.

### 3.1 World lookup

The helper first resolves the world file from the `world_package` and `world_name` fields of the simulation configuration.

When `world_package` is set to a package name such as `romea_simulation_gazebo_worlds`, the world is loaded from:

```text
<package_share>/worlds/<world_name>
```

When `world_package` is omitted or set to `gazebo`, the helper searches the `GAZEBO_RESOURCE_PATH`.

### 3.2 WGS84 anchor insertion

After the world file has been loaded, the helper can insert a WGS84 anchor into the world `spherical_coordinates` element.

The anchor is provided by `romea_gps_meta_bringup` and has the following format:

```yaml
latitude: 45.76265802
longitude: 3.11000985
altitude: 405.839
```

If the selected world already contains a `spherical_coordinates` element, the existing coordinates are kept.

The resulting world is then saved to a temporary file loaded by the simulator, usually `/tmp/gazebo_world.world`.

---

## 4) Integration with simulation meta-bringup

A simulation configuration can select one world per simulator backend:

```yaml
simulators:
  gazebo_classic:
    world_package: romea_simulation_gazebo_worlds
    world_name: romea_small_vineyard.world
  gazebo:
    world_package: romea_simulation_gazebo_worlds
    world_name: gz_empty.sdf
```

`romea_simulation_meta_bringup` then:

* loads the selected world through `GazeboWorld`
* applies the WGS84 anchor when provided
* writes the generated world to `/tmp/gazebo_world.world`
* starts Gazebo or Gazebo Classic with that generated world

This keeps world resources separated from robot and device meta-descriptions while still allowing robot-level launches to select a simulation environment from a single configuration file.
