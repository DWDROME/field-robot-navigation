# ROS Package Inventory

> Authoritative package identities and responsibility boundaries for this workspace.

---

## Inventory Rule

A directory counts as a ROS package here only when it contains a
`package.xml`. Package discovery must be recursive because `src/MoveBase3D/`
is a container with nested packages.

As of 2026-07-26, the workspace contains:

- 27 catkin package manifests under `src/`
- 17 packages outside `src/MoveBase3D/`
- 10 packages inside `src/MoveBase3D/`

Use the `<name>` value from `package.xml` for `find_package`, `roslaunch`,
`rosrun`, and dependency declarations. The directory name is not always the
package name.

## Current Greenhouse Runtime Line

These packages form the production-oriented greenhouse mapping, localization,
planning, and execution path described by the top-level `README.md` and
`src/greenhouse_nav/` launch assembly.

| Directory | Package name | Responsibility | Boundary |
|-----------|--------------|----------------|----------|
| `src/livox_ros_driver2/` | `livox_ros_driver2` | Livox Mid360 driver and custom point-cloud messages | Default greenhouse sensor input |
| `src/FAST_LIO/` | `fast_lio` | Baseline LiDAR-inertial odometry and mapping | Default greenhouse SLAM input today |
| `src/FAST_LIO_LOCALIZATION/` | `fast_lio_localization` | Localization against a prebuilt point-cloud map | Default greenhouse localization path today |
| `src/greenhouse_nav/` | `greenhouse_nav` | User-facing launch integration, RViz profiles, robot description, and Python bridge/manager nodes | Owns system assembly, not the underlying planning or SLAM algorithms |
| `src/terrain_analysis/` | `terrain_analysis` | Local terrain and elevation analysis | Mapping/perception stage used by navigation |
| `src/terrain_analysis_ext/` | `terrain_analysis_ext` | Extended-range terrain analysis | Companion terrain-processing stage |
| `src/far_planner/` | `far_planner` | FAR visibility-graph route planning | Global/route planning owner |
| `src/local_planner/` | `local_planner` | Local path planning and tracking | Local motion planning owner |
| `src/dlrobot_robot/` | `dlrobot_robot` | Chassis integration and command execution | Hardware-facing actuator boundary |

The default greenhouse runtime still uses `fast_lio` and
`fast_lio_localization`. Do not describe `fast_lio_super` as the default
greenhouse launch target unless the launch files are changed accordingly.

## SLAM Research And Replacement Line

| Directory | Package name | Responsibility | Boundary |
|-----------|--------------|----------------|----------|
| `src/FAST_LIO_SUPER/` | `fast_lio_super` | Pluggable SLAM development branch with map-backend and relocalization work | Candidate replacement/research line, not the default greenhouse runtime |
| `src/slam_octvox/` | `slam_octvox` | Header-heavy OctVoxMap and HKNN backend support | Infrastructure consumed by the `fast_lio_super` line |

Read `src/FAST_LIO_SUPER/CLAUDE.md` and its `docs/` directory before changing
backend selection, loop closure, relocalization, or map-manager code.

## Drivers, Messages, Bridges, And Support

| Directory | Package name | Responsibility | Boundary |
|-----------|--------------|----------------|----------|
| `src/livox_ros_driver2/` | `livox_ros_driver2` | Livox 3D LiDAR ROS driver | Sensor and vendor-driver boundary |
| `src/visibility_graph_msg/` | `visibility_graph_msg` | Visibility-graph message definitions | Shared ROS message contract |
| `src/dl_msgs/` | `dl_msgs` | Chassis-related custom messages and services | Shared ROS message/service contract |
| `src/livox_to_pointcloud2/` | `livox_to_pointcloud2` | Converts Livox `CustomMsg` data to standard `PointCloud2` | Sensor-message bridge |
| `src/graph_decoder/` | `graph_decoder` | Decodes and republishes visibility-graph data | Graph adapter/support package |
| `src/boundary_handler/` | `boundary_handler` | Extracts and saves visibility-graph data from boundary/trajectory files | Standalone offline planning support |

Message packages own wire contracts. A change to `msg/` or `srv/` is a
cross-package API change and must be checked against every producer and
consumer.

`graph_decoder` is reachable only when `use_graph_decoder=true`; the current
forward and reverse greenhouse entries set it to `false`. `boundary_handler`
and `livox_to_pointcloud2` are not included by those entries.

## Runtime Role Precedence

When package prose and runtime assembly disagree, classify current usage in
this order:

1. user-facing launch entry;
2. recursive launch includes, groups, arguments, and remaps;
3. source-level publishers, subscribers, and executable entrypoints;
4. top-level `README.md` and package-local documentation.

This produces the current role split:

- **default greenhouse runtime**: `greenhouse_nav`, `livox_ros_driver2`,
  `fast_lio`, `fast_lio_localization`, `terrain_analysis`,
  `terrain_analysis_ext`, `far_planner`, `local_planner`, `dlrobot_robot`;
- **optional from the greenhouse assembly**: `graph_decoder` and the
  `greenhouse_nav` global-to-local adapter;
- **SLAM candidate/research line**: `fast_lio_super`, `slam_octvox`;
- **standalone support**: `livox_to_pointcloud2`, `boundary_handler`;
- **alternate/reference line**: `dlrobot` and all ten MoveBase3D packages.

`FAST_LIO_SUPER` has a valid standalone launch path, but it is not referenced
by the current greenhouse user entries. MoveBase3D and `dlrobot` likewise have
their own entrypoints and are not recursively included by the greenhouse
launch tree.

## Alternate And Reference Stacks

`src/dlrobot/` and `src/MoveBase3D/` are retained as alternate or reference
navigation/robot stacks. Their presence does not make them part of the
default greenhouse launch path.

| Directory | Package name | Responsibility |
|-----------|--------------|----------------|
| `src/dlrobot/` | `dlrobot` | Alternate two-wheel differential-drive robot platform/driver |
| `src/MoveBase3D/global_planner_3d/` | `global_planner_3d` | Alternate 3D global planning |
| `src/MoveBase3D/local_planner_3d/` | `local_planner_3d` | Alternate 3D local planning |
| `src/MoveBase3D/rover_gpr/` | `rover_gpr` | Rover GPR/OctoMap-related processing; inspect source before assuming a narrower behavior |
| `src/MoveBase3D/rover_launch/` | `rover_launch` | Launch-only integration package for the alternate stack |
| `src/MoveBase3D/rover_msgs/` | `rover_msgs` | Messages and services for the alternate rover stack |
| `src/MoveBase3D/rover_pcl_filter/` | `rover_pcl_filter` | Point-cloud filtering |
| `src/MoveBase3D/rover_teleop/` | `rover_teleop` | Keyboard teleoperation |
| `src/MoveBase3D/rviz_plugins/rviz_goal_tool_plugins/` | `rviz_plugins` | Additional RViz goal-tool plugins |
| `src/MoveBase3D/rviz_plugins/rviz_multi_goals_manager_plugins/` | `rviz_multi_goals_manager_plugins` | RViz multi-goal management plugin |
| `src/MoveBase3D/rviz_plugins/rviz_waypoint_generator_plugins/` | `waypoint_generator` | Waypoint-generation support nested with the RViz tooling |

Treat each child of `MoveBase3D` that has a `package.xml` as an independent
catkin package and ownership boundary.

## Directory And Package Name Differences

These paths cannot be converted into package names by copying the directory
basename:

| Directory | Package name |
|-----------|--------------|
| `src/FAST_LIO/` | `fast_lio` |
| `src/FAST_LIO_LOCALIZATION/` | `fast_lio_localization` |
| `src/FAST_LIO_SUPER/` | `fast_lio_super` |
| `src/MoveBase3D/rviz_plugins/rviz_goal_tool_plugins/` | `rviz_plugins` |
| `src/MoveBase3D/rviz_plugins/rviz_waypoint_generator_plugins/` | `waypoint_generator` |

Always verify the manifest before writing package-qualified commands,
dependencies, or launch references.

## Non-Package Source Directories

`src/global_to_local_adapter/` currently has documentation and a reserved
helper-script layout but no `package.xml`. It is not an independently
buildable or launchable ROS package.

Do not add it to package dependency lists or call it with `rosrun`/`roslaunch`
unless it is deliberately converted into a package with the required catkin
metadata.

## Evidence And Refresh Procedure

Primary evidence:

- `src/**/package.xml`
- top-level `README.md` and `CLAUDE.md`
- package-local `CLAUDE.md` files
- `src/greenhouse_nav/launch/*.launch`
- `.claude/index.json`
- `.trellis/spec/backend/launch-topology.md`
- `.trellis/spec/backend/bridge-and-support-boundaries.md`

Refresh this inventory after adding, removing, or moving a manifest:

```bash
find src -name package.xml -print | sort
find src -name package.xml -print | wc -l
find src/MoveBase3D -name package.xml -print | wc -l
```

For package identity, parse XML and read `<name>`; do not infer it from a path.
