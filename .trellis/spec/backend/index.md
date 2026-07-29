# Backend Development Guidelines

> Working conventions for the ROS 1 Noetic workspace and packages being migrated to ROS 2 Jazzy.

---

## Scope

This repository is backend-heavy. The main implementation surface is:

- ROS packages under `src/`
- C++ catkin nodes and libraries
- Python ROS nodes installed with `catkin_install_python`
- launch assembly, YAML config, RViz config, URDF, and message definitions

There is no application database layer and no web service layer in the current workspace. Read the backend guides as "how this robotics workspace is organized and changed safely".

## Package Map

The current workspace splits into a few clear groups.

One important distinction:

- the current greenhouse runtime mainline still launches baseline `FAST_LIO` + `FAST_LIO_LOCALIZATION`
- `FAST_LIO_SUPER` is the active pluggable SLAM branch and candidate replacement path, but it is not the default greenhouse launch target today

- Main SLAM line:
  - `src/FAST_LIO`
  - `src/FAST_LIO_LOCALIZATION`
  - `src/FAST_LIO_SUPER`
  - `src/slam_octvox`
- Main greenhouse navigation line:
  - `src/greenhouse_nav`
  - `src/terrain_analysis`
  - `src/terrain_analysis_ext`
  - `src/far_planner`
  - `src/local_planner`
  - `src/dlrobot_robot` (ROS 2 Jazzy chassis package; not directly buildable by the ROS 1 catkin mainline)
- Message, bridge, and support packages:
  - `src/visibility_graph_msg`
  - `src/dl_msgs`
  - `src/livox_ros_driver2`
  - `src/livox_to_pointcloud2`
  - `src/graph_decoder`
  - `src/boundary_handler`
- Alternate or reference navigation stack:
  - `src/MoveBase3D`
  - `src/dlrobot`

## Responsibility Cheatsheet

Use this quick classification before diving into package-local docs.

| Package group | Current role |
|---------------|--------------|
| `greenhouse_nav`, `FAST_LIO`, `FAST_LIO_LOCALIZATION`, `terrain_analysis`, `terrain_analysis_ext`, `far_planner`, `local_planner`, `dlrobot_robot` | Current greenhouse production-oriented runtime line |
| `FAST_LIO_SUPER`, `slam_octvox` | SLAM research / replacement branch with pluggable backend work |
| `livox_ros_driver2` | Sensor driver |
| `livox_to_pointcloud2`, `graph_decoder`, `boundary_handler` | Bridge / adapter / support tooling |
| `visibility_graph_msg`, `dl_msgs`, package-local `msg/` trees | Shared ROS message contracts |
| `MoveBase3D`, `dlrobot` | Alternate or reference stacks |

Primary evidence:

- `README.md`
- `CLAUDE.md`
- `.claude/index.json`
- package-local `CLAUDE.md` files under `src/*/`

## Pre-Development Checklist

Before editing this workspace:

1. Identify the owning ROS package first. Do not treat `src/` as a flat C++ project.
2. Check the package entry points:
   - `package.xml`
   - `CMakeLists.txt`
   - `launch/`
   - `scripts/` or `src/`
3. Check whether the package is:
   - a production line package,
   - a message/support package,
   - or an alternate/reference package.
4. Preserve existing launch names, node names, topic names, and frame names unless the task explicitly changes public behavior.
5. Run the smallest relevant verification after edits. In this repo that usually means `catkin_make`, a targeted `roslaunch ... --nodes`, or a package-level smoke test.

## Guides

| Guide | What it answers | Status |
|-------|-----------------|--------|
| [Package Inventory](./package-inventory.md) | Which catkin packages exist, what they own, and where path/package names differ | Ready |
| [Launch Topology](./launch-topology.md) | How user entrypoints recursively assemble nodes and optional branches | Ready |
| [Directory Structure](./directory-structure.md) | How ROS packages and files are laid out | Ready |
| [Module Routing](./module-routing.md) | Which package to inspect first for a given symptom or change | Ready |
| [Runtime Contracts](./runtime-contracts.md) | Canonical launch, topic, and TF contracts for the greenhouse mainline | Ready |
| [ROS Interface Contracts](./ros-interface-contracts.md) | Native node interfaces, greenhouse remaps, message types, and TF ownership | Ready |
| [Build And Message Contracts](./build-and-message-contracts.md) | Catkin dependency groups, generated interfaces, and producer/consumer ownership | Ready |
| [Bridge And Support Boundaries](./bridge-and-support-boundaries.md) | Optional adapters, decoders, converters, and file-driven helper behavior | Ready |
| [Licensing And Provenance Evidence](./licensing-and-provenance.md) | Manifest values, local license evidence, conflicts, and review boundaries | Ready |
| [Configuration And Asset Contracts](./configuration-and-assets.md) | Config precedence, maps, planner inputs, RViz, URDF, and deployment resources | Ready |
| [Runtime Artifact Contracts](./runtime-artifacts.md) | Generated PCD/log/graph/bag ownership and overwrite boundaries | Ready |
| [Documentation Evidence Rules](./documentation-evidence.md) | How current, standalone, legacy, research, and unverified claims are separated | Ready |
| [Database Guidelines](./database-guidelines.md) | File-based persistence categories and why no database abstraction applies | Ready |
| [Error Handling](./error-handling.md) | How failures are surfaced in C++ and Python ROS nodes | Ready |
| [Logging Guidelines](./logging-guidelines.md) | How ROS logging is used in runtime loops and startup | Ready |
| [Quality Guidelines](./quality-guidelines.md) | Validation expectations and review checks | Ready |
| [ROS 2 Robotics C++ Style](./ros2-robotics-code-style.md) | Direct C++17, protocol, hardware ownership, error, and test contracts for ROS 2 nodes | Ready |

## Shared Guides

Also read:

- `.trellis/spec/guides/index.md`
- `.trellis/spec/guides/code-reuse-thinking-guide.md`
- `.trellis/spec/guides/cross-layer-thinking-guide.md`
