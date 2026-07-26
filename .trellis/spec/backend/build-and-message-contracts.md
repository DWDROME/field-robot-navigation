# Build And Message Contracts

> Catkin dependency boundaries and custom ROS wire contracts in this workspace.

---

## Build Model

This is one ROS1 catkin workspace with 27 recursively discovered packages.
Package identity comes from `package.xml`; build behavior comes from both
`package.xml` and `CMakeLists.txt`.

Do not assume either file is complete by itself. Several retained upstream or
research packages have narrower `catkin_package(CATKIN_DEPENDS ...)` exports
than their manifests and source usage imply.

When changing a dependency, inspect all three surfaces:

1. `package.xml` build, export, and runtime dependencies;
2. `find_package(catkin REQUIRED COMPONENTS ...)`;
3. `catkin_package(CATKIN_DEPENDS ...)` and target dependencies.

## Workspace Dependency Groups

### Message foundations

```text
visibility_graph_msg
├── far_planner
├── graph_decoder
└── boundary_handler

dl_msgs
└── dlrobot

rover_msgs
├── global_planner_3d
├── local_planner_3d
└── rviz_multi_goals_manager_plugins
```

### Livox and SLAM

```text
livox_ros_driver2
├── fast_lio
├── fast_lio_localization
├── fast_lio_super
└── livox_to_pointcloud2

slam_octvox
└── fast_lio_super
```

`slam_octvox` is header-oriented infrastructure exported through
`catkin_package(INCLUDE_DIRS include)`. `fast_lio_super` is the only current
workspace package that declares it as a catkin dependency.

### Runtime-only launch composition

`greenhouse_nav` launches FAST-LIO, localization, terrain, FAR, local planning,
and chassis packages, but its manifest only declares the standard ROS
dependencies used by its Python nodes.

Therefore:

- a launch include does not necessarily create a catkin build dependency;
- the greenhouse runtime graph is broader than the build graph expressed by
  `greenhouse_nav/package.xml`;
- package availability and resource paths must be checked separately from
  successful compilation of `greenhouse_nav`.

## Packages That Generate Custom Interfaces

| Package | Generated interfaces | Runtime role |
|---------|----------------------|--------------|
| `visibility_graph_msg` | `Node.msg`, `Graph.msg` | FAR visibility-graph serialization |
| `dl_msgs` | 16 messages and `ServoAngle.srv` | Retained `dlrobot`/external controller contracts |
| `rover_msgs` | `roverBmsStatus`, `roverRCStatus`, `roverGoalStatus` | Alternate MoveBase3D status contracts |
| `fast_lio` | `Pose6D.msg` | Internal generated C++ type, not a current topic |
| `fast_lio_localization` | `Pose6D.msg` | Generated locally, but current source includes `fast_lio/Pose6D.h` |
| `fast_lio_super` | `Pose6D.msg` | Internal generated C++ type, not a current topic |
| `livox_ros_driver2` | `CustomPoint.msg`, `CustomMsg.msg` | LiDAR driver wire contract |

`livox_to_pointcloud2` calls `generate_messages()` while its
`add_message_files()` block is commented out. Treat that as template residue,
not as ownership of a custom message contract.

## Active Message Contracts

### Livox custom point cloud

`livox_ros_driver2/CustomMsg` is published by the Livox driver and consumed by:

- baseline `fast_lio`;
- `fast_lio_localization`;
- `fast_lio_super`;
- optional `livox_to_pointcloud2`.

`CustomPoint` is nested inside `CustomMsg`; it is not published as a separate
topic type.

Changes to either message affect the sensor-driver boundary and every FAST-LIO
variant. Rebuild the complete dependent set, not only the driver.

### Visibility graph

`visibility_graph_msg/Graph` contains `Node` records and is used on a separate
graph persistence/decoding channel:

```text
far_planner /robot_vgraph
    -> graph_decoder
       -> decoded_vgraph
          -> far_planner
```

This is not the local-planner handoff. FAR separately publishes:

- `/way_point` as `geometry_msgs/PointStamped`;
- `/navigation_boundary` as `geometry_msgs/PolygonStamped`.

`localPlanner` consumes those standard messages. Do not describe
`/way_point` or `/navigation_boundary` as `visibility_graph_msg/Graph`.

`boundary_handler` has code that can encode internal nodes into
`visibility_graph_msg/Graph`, but its current runtime path does not advertise
or publish that message. It writes graph data to files and publishes RViz
markers.

### MoveBase3D status

Among the three `rover_msgs` definitions, `roverGoalStatus` has a confirmed
workspace producer/consumer loop:

- `global_planner_3d` publishes `/cur_global_goal_status`;
- `local_planner_3d` publishes `/cur_local_goal_status`;
- `rviz_multi_goals_manager_plugins` consumes both.

No repository producer or consumer was found for `roverBmsStatus` or
`roverRCStatus`. Treat them as external/reserved interfaces until runtime
evidence establishes ownership.

### `dl_msgs`

Within this workspace, `dlrobot` consumes `dl_msgs/Velocities` on `raw_vel`.
Its producer is not present under `src/` and is likely external to this
repository's ROS graph.

No repository producer/consumer was found for `ServoAngle.srv` or the other
generated `dl_msgs` messages. Do not remove or repurpose them based only on
local non-use; they may be rosserial or hardware-facing compatibility
contracts.

### FAST-LIO `Pose6D`

The three FAST-LIO directories each contain a `Pose6D.msg`, but no current ROS
publisher or subscriber transports it as a topic/service message. It is used
as a generated C++ data type.

Important asymmetry:

- baseline includes `fast_lio/Pose6D.h`;
- SUPER includes `fast_lio_super/Pose6D.h`;
- localization generates its own `fast_lio_localization/Pose6D.msg`, but its
  source includes `fast_lio/Pose6D.h`;
- localization's current CMake/catkin metadata does not declare `fast_lio`,
  so this is a cross-package generated-header dependency and build-metadata
  gap, not merely duplicate local message ownership.

Do not consolidate or delete these definitions without first resolving the
localization package's include and dependency behavior.

## Observed Build Metadata Drift

These are review hazards, not instructions to perform a broad dependency
cleanup during unrelated work.

| Package | Observed drift |
|---------|----------------|
| `boundary_handler` | Finds and uses `visibility_graph_msg`, but does not export it from `catkin_package` |
| `far_planner` | Uses `visibility_graph_msg`; exported dependency list is narrower than source/manifest use |
| `graph_decoder` | Uses `visibility_graph_msg` and declares `geometry_msgs` in the manifest; CMake exports are narrower |
| `rover_msgs` | Finds `std_srvs` but has no active service generation and does not export it |
| `rviz_multi_goals_manager_plugins` | Finds `rover_msgs`, but exports only `message_runtime` |
| `livox_to_pointcloud2` | Calls empty message generation, claims a library that is not built, and does not export all consumed packages |
| FAST-LIO variants | Find `livox_ros_driver2`, but do not export it through `catkin_package`; SUPER does export `slam_octvox` |
| `dlrobot_robot` | Manifest includes message-generation-related metadata while `catkin_package()` is empty |

For system libraries such as PCL, OpenCV, Qt, APR, Eigen, and serial support,
also compare package metadata with non-catkin `find_package`, `pkg_check_modules`,
and hard-coded include/library paths.

## Build Change Rules

- Use package names from `<name>`, not directory basenames.
- Add generated-message dependencies before compiling consumers.
- Add `${PROJECT_NAME}_EXPORTED_TARGETS` or dependency targets where generated
  headers are consumed during the same build.
- Keep `package.xml`, `find_package`, and `catkin_package` aligned when
  intentionally changing a public build boundary.
- Do not add a catkin dependency merely because two nodes exchange a topic at
  runtime; add it when code, generated types, libraries, or installed resources
  require it.
- Do not remove an apparently unused custom message without checking external
  hardware, rosserial, bag, and downstream workspace consumers.

## Validation

For a message or build-boundary change:

```bash
catkin_make
```

Then inspect:

- generated headers for every changed message package;
- direct consumer targets;
- runtime topic types with `rostopic type` when a ROS graph is available;
- recursive launch paths that expect the changed package or resource.

For documentation-only inventory work, XML/CMake inspection and Trellis
validation are sufficient; do not claim a build was tested unless
`catkin_make` actually ran.

## Primary Evidence

- `src/**/package.xml`
- `src/**/CMakeLists.txt`
- `src/**/msg/*.msg`
- `src/**/srv/*.srv`
- `src/livox_ros_driver2/src/lddc.cpp`
- `src/far_planner/src/graph_msger.cpp`
- `src/far_planner/src/far_planner.cpp`
- `src/graph_decoder/src/decoder_node.cpp`
- `src/local_planner/src/localPlanner.cpp`
- `src/dlrobot/src/dl_base.cpp`
- `src/MoveBase3D/global_planner_3d/src/global_planning_node.cpp`
- `src/MoveBase3D/local_planner_3d/scripts/local_planner_node.py`
- `src/MoveBase3D/rviz_plugins/rviz_multi_goals_manager_plugins/`
