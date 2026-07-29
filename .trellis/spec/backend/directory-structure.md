# Directory Structure

> How backend code is organized in this workspace today.

---

## Overview

This repository is in a mixed migration state:

- The workspace root contains `src/CMakeLists.txt`, the catkin toplevel entry.
- 26 packages under `src/` remain ROS 1 Noetic catkin packages.
- `src/dlrobot_robot/` is a ROS 2 Jazzy `ament_cmake` package built with
  `colcon`; it is not part of the root catkin build.
- Each ROS package usually lives in its own directory under `src/`.
- Most behavior is assembled through `launch/*.launch`, not a single `main()` binary.
- Package-local docs matter. Many packages have a `CLAUDE.md` file that already explains module intent.

## Directory Layout

```text
src/
├── CMakeLists.txt                     # catkin toplevel
├── FAST_LIO/                          # baseline lidar odometry / mapping
│   ├── config/
│   ├── include/
│   ├── launch/
│   ├── msg/
│   ├── src/
│   └── README.md
├── FAST_LIO_SUPER/                    # extended SLAM line with docs/ and slam_core work
│   ├── config/
│   ├── docs/
│   ├── include/
│   ├── launch/
│   ├── msg/
│   └── src/
├── greenhouse_nav/                    # integration package: launch + Python bridge nodes
│   ├── launch/
│   ├── rviz/
│   ├── scripts/
│   └── urdf/
├── far_planner/                       # planner package: include/ + src/ + config/ + launch/
├── local_planner/                     # planner package: src/ + launch/ + paths/
├── terrain_analysis/                  # mapping / terrain package
├── terrain_analysis_ext/              # extended terrain package
├── dlrobot_robot/                     # ROS 2 actuator package: include/ + src/ + launch/ + test/
├── visibility_graph_msg/              # message-only package
├── dl_msgs/                           # msg/srv package
├── slam_octvox/                       # header-heavy support package for FAST_LIO_SUPER
└── MoveBase3D/                        # container directory for several alternate subpackages
```

The previous ROS 1 chassis package is preserved outside the build source tree:

```text
other/历史归档/ros1/dlrobot_robot/
```

## Package Organization Patterns

### 1. Algorithm or runtime packages

These packages usually follow:

```text
<pkg>/
├── CMakeLists.txt
├── package.xml
├── include/
├── src/
├── launch/
├── config/        # optional
├── rviz/          # optional
└── README.md or CLAUDE.md
```

Examples:

- `src/FAST_LIO_SUPER/`
- `src/far_planner/`
- `src/graph_decoder/`
- `src/boundary_handler/`

### 2. Integration packages

Integration packages assemble existing modules rather than owning the core math.

`src/greenhouse_nav/` is the clearest example:

- `launch/` holds user-facing entry points such as `greenhouse_mapping.launch` and `greenhouse_navigation.launch`
- `scripts/` holds Python bridge or manager nodes such as `cmd_vel_stamped_bridge.py` and `localization_manager.py`
- `rviz/` stores operator-facing RViz profiles
- `urdf/` stores mount description data

### 3. Message-only packages

These packages own contracts, not runtime logic.

Examples:

- `src/visibility_graph_msg/`
- `src/dl_msgs/`

Pattern:

- `msg/` and sometimes `srv/`
- minimal `CMakeLists.txt` and `package.xml`
- no large `src/` implementation tree

### 4. Alternate stack container directories

`src/MoveBase3D/` is not one package. It is a directory that contains several catkin packages:

- `global_planner_3d`
- `local_planner_3d`
- `rover_gpr`
- `rover_launch`
- `rover_msgs`
- `rover_pcl_filter`
- `rover_teleop`
- `rviz_plugins/rviz_goal_tool_plugins/` (`package.xml` name: `rviz_plugins`)
- `rviz_plugins/rviz_multi_goals_manager_plugins/` (`package.xml` name: `rviz_multi_goals_manager_plugins`)
- `rviz_plugins/rviz_waypoint_generator_plugins/` (`package.xml` name: `waypoint_generator`)

Treat each child package as its own build and ownership boundary.

The package name must come from `package.xml`, not from the directory basename.
This also applies to the uppercase upstream directories:

- `src/FAST_LIO/` -> `fast_lio`
- `src/FAST_LIO_LOCALIZATION/` -> `fast_lio_localization`
- `src/FAST_LIO_SUPER/` -> `fast_lio_super`

See [ROS Package Inventory](./package-inventory.md) for the complete recursive
list and status classification.

## Naming Conventions

- ROS package directories are snake_case or upstream package names, for example `greenhouse_nav`, `far_planner`, `livox_ros_driver2`.
- Launch files are task-oriented and user-facing, for example:
  - `greenhouse_mapping.launch`
  - `greenhouse_navigation.launch`
  - `mapping_mid360.launch`
- Python node scripts are lower_snake_case and usually describe the bridge/manager role:
  - `cmd_vel_stamped_bridge.py`
  - `localization_manager.py`
  - `global_to_local_adapter.py`
- Config files are sensor- or mode-specific:
  - `config/mid360.yaml`
  - `config/avia.yaml`
  - `config/default.yaml`

## Rules For New Work

- Put new code in the owning ROS package, not in a shared misc directory.
- Do not create a nested `ros2_ws/` for the chassis migration. The ROS 2
  package remains at `src/dlrobot_robot` and is selected explicitly with
  `colcon`.
- Do not place the archived ROS 1 and active ROS 2 `dlrobot_robot` manifests
  in the same source tree.
- Prefer extending an existing integration package when the task is launch assembly, TF wiring, topic remapping, or a Python bridge node.
- Prefer extending a message package only when the task changes the shared ROS contract.
- Keep package-local docs close to the code when a package has non-trivial architecture, following the existing `CLAUDE.md` and `docs/` pattern in `FAST_LIO_SUPER`.
- Do not hide a new package inside `MoveBase3D/` unless it is truly part of that alternate navigation stack.

## Examples

- `src/greenhouse_nav/` is the best example of an integration package.
- `src/FAST_LIO_SUPER/` is the best example of a research-heavy runtime package with architecture docs.
- `src/visibility_graph_msg/` is the best example of a message-only package.
- `src/MoveBase3D/` shows how alternate stacks are grouped without flattening ownership into one package.
