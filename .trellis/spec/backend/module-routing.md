# Module Routing

> Where to look first when a task or bug lands in this workspace.

---

## Mainline First

The current production-oriented greenhouse line is centered on:

- `src/greenhouse_nav`
- `src/FAST_LIO`
- `src/FAST_LIO_LOCALIZATION`
- `src/terrain_analysis`
- `src/terrain_analysis_ext`
- `src/far_planner`
- `src/local_planner`
- `src/dlrobot_robot`

Do not start from `MoveBase3D` unless the task explicitly targets the alternate 3D navigation stack.

## User-Facing Entrypoints

These are the first places to inspect when the user talks about "the system" rather than one package.

| User workflow | Primary entry | Notes |
|---------------|---------------|-------|
| Mapping | `src/greenhouse_nav/launch/greenhouse_mapping.launch` | Mid360 + FAST-LIO + RViz + fixed TF |
| Navigation default | `src/greenhouse_nav/launch/greenhouse_navigation.launch` | Alias to forward navigation |
| Navigation forward | `src/greenhouse_nav/launch/greenhouse_navigation_forward.launch` | Default operational direction |
| Navigation reverse | `src/greenhouse_nav/launch/greenhouse_navigation_reverse.launch` | Same chain, different `body -> base_link` yaw |
| Internal mainline assembly | `src/greenhouse_nav/launch/system_greenhouse_base.launch` | Localization, terrain, local planner, bridge, base |
| Internal route-planner assembly | `src/greenhouse_nav/launch/system_greenhouse_with_route_planner.launch` | Adds FAR route planner layer |

If a symptom appears in a full-system launch, start at the greenhouse launch layer before diving into individual packages.

## Change Routing By Symptom

### 1. Mapping, lidar odometry, or map-backend behavior

Look first at:

- `src/FAST_LIO/`
- `src/FAST_LIO_SUPER/`
- `src/slam_octvox/`

Start with:

- `package.xml`
- `CMakeLists.txt`
- `launch/mapping_*.launch`
- `src/FAST_LIO/src/laserMapping.cpp`
- `src/FAST_LIO_SUPER/src/laserMapping.cpp`
- `config/*.yaml`

Use `FAST_LIO_SUPER` when the task mentions:

- `slam_core`
- `IMapManager`
- `INeighborSearch`
- `OctVox`
- runtime backend selection

Use baseline `FAST_LIO` when the running navigation chain still launches `fast_lio` rather than `fast_lio_super`.

### 2. Global relocalization or `/initialpose`

Look first at:

- `src/FAST_LIO_LOCALIZATION/`
- `src/greenhouse_nav/scripts/localization_manager.py`
- `src/greenhouse_nav/launch/localization_manager.launch`

Typical files:

- `scripts/global_localization.py`
- `scripts/publish_initial_pose.py`
- `src/FAST_LIO_LOCALIZATION/src/laserMapping.cpp`

Important boundary:

- `FAST_LIO_LOCALIZATION` has conflicting manifest, directory-license, and
  prose evidence
- treat it as a separate reference/runtime package and do not copy code from
  it into `FAST_LIO_SUPER` without a dedicated provenance/license review

### 3. TF, frame semantics, or `/state_estimation`

Look first at:

- `src/greenhouse_nav/launch/system_greenhouse_base.launch`
- `src/greenhouse_nav/scripts/localization_manager.py`
- `src/greenhouse_nav/launch/greenhouse_mapping.launch`

Common keywords:

- `body`
- `base_link`
- `body_foot`
- `robot_foot_init`
- `camera_init`
- `map_to_odom`

This is the main ownership point for Mid360 mount compensation and the public `map -> base_link` output.

### 4. Terrain map quality or local obstacle handling

Look first at:

- `src/terrain_analysis/`
- `src/terrain_analysis_ext/`
- `src/greenhouse_nav/launch/base_autonomy_cmu.launch`

Route by output:

- `/terrain_map` issues -> `terrain_analysis`
- `/terrain_map_ext` issues -> `terrain_analysis_ext`

Check the launch params before changing code. A large part of behavior is controlled through launch-time parameterization.

### 5. Global route planning or `goal_point -> way_point`

Look first at:

- `src/far_planner/`
- `src/visibility_graph_msg/`
- `src/greenhouse_nav/launch/far_route_planner.launch`

Supporting packages:

- `src/graph_decoder/` when the task is specifically about decoding graph messages for downstream consumers
- `src/boundary_handler/` when the task is specifically about boundary graph extraction

### 6. Local path generation or command generation

Look first at:

- `src/local_planner/`
- `src/greenhouse_nav/launch/base_autonomy_cmu.launch`

Route by topic:

- `/path` generation -> `src/local_planner/src/localPlanner.cpp`
- `/cmd_vel_stamped` generation -> `src/local_planner/src/pathFollower.cpp`
- `/cmd_vel` conversion -> `src/greenhouse_nav/scripts/cmd_vel_stamped_bridge.py`

### 7. Chassis serial control or actuator delivery

Look first at:

- `src/dlrobot_robot/`

Typical files:

- `src/dlrobot_robot/src/dlrobot_robot.cpp`
- `src/dlrobot_robot/launch/base_serial.launch`
- `src/dlrobot_robot/scripts/` for environment or udev helpers

### 8. Sensor driver or Livox message conversion

Look first at:

- `src/livox_ros_driver2/`
- `src/livox_to_pointcloud2/`

Use `livox_ros_driver2` for raw device integration or official driver launch behavior. Use `livox_to_pointcloud2` for conversion from Livox custom messages to `sensor_msgs/PointCloud2`.

### 9. Message contract changes

Look first at:

- `src/visibility_graph_msg/`
- `src/dl_msgs/`
- message definitions inside SLAM packages such as `src/FAST_LIO/msg/` or `src/FAST_LIO_SUPER/msg/`

Changing a message is a cross-package contract change. Inspect every producer and consumer before editing.

### 10. Alternate navigation stack work

Look first at:

- `src/MoveBase3D/`

But remember:

- `MoveBase3D` contains multiple child packages, not one package
- it is documented as a parallel/reference stack, not the current greenhouse mainline

### 11. Configuration, maps, and file assets

Look first at:

- `src/FAST_LIO/config/` plus the enclosing FAST-LIO launch;
- `src/greenhouse_nav/launch/` for later inline overrides;
- `src/far_planner/config/` and `src/far_planner/rviz/`;
- `src/livox_ros_driver2/config/` for device/network JSON;
- `src/local_planner/paths/` for required algorithm inputs;
- `src/far_planner/data/` for graph/boundary inputs and overwrite targets;
- `src/dlrobot_robot/urdf/` and deployment/helper scripts.

Read [Configuration And Asset Contracts](./configuration-and-assets.md) and
[Runtime Artifact Contracts](./runtime-artifacts.md) before moving, ignoring,
or regenerating these files.

### 12. Documentation or package identity

- Determine ROS package identity from `package.xml`, not directory names or
  prose indexes.
- `src/global_to_local_adapter/` contains documentation but no `package.xml`;
  it is not a catkin package. The runtime adapter belongs to
  `src/greenhouse_nav/`.
- Use [Documentation Evidence Rules](./documentation-evidence.md) when
  README, CLAUDE, launch, source, and research docs disagree.

## Routing By Task Type

| Task type | Start here |
|-----------|------------|
| "Why doesn't navigation launch work?" | `greenhouse_nav/launch/greenhouse_navigation*.launch` |
| "Why is the cloud tilted in RViz?" | `greenhouse_nav` TF and mount-compensation files |
| "Why is `/state_estimation` wrong?" | `localization_manager.py` + relocalization inputs |
| "Why is the robot not moving?" | `local_planner` -> bridge -> `dlrobot_robot` |
| "Why is the global path strange?" | `far_planner` + `terrain_analysis_ext` |
| "Why are close obstacles missed?" | `terrain_analysis` + `local_planner` |
| "How do I switch FAST-LIO map backends?" | `FAST_LIO_SUPER` + `slam_octvox` + `config/*.yaml` |
| "Why did a YAML edit have no effect?" | Trace later `<param>` overrides through the top-level launch |
| "Why is a map/RViz/path file missing?" | Configuration/assets guide + enabled launch resource paths |
| "Can we reuse code from FAST_LIO_LOCALIZATION?" | Treat license evidence as unresolved; no direct merge without review |

## Validation Routing

After changes, validate near the owning layer:

- launch assembly edits -> `roslaunch ... --nodes`
- Python bridge edits -> targeted launch or node smoke test
- C++ package edits -> `catkin_make` plus the nearest package/launch smoke path
- message changes -> rebuild plus producer/consumer launch checks

## Common Mistakes

- Starting in `MoveBase3D` when the issue belongs to the greenhouse mainline.
- Editing `FAST_LIO_SUPER` while the active launch path still runs `fast_lio`.
- Changing parameters in code when the live behavior is actually set in a launch file.
- Forgetting that a topic symptom may be owned by `greenhouse_nav` wiring rather than by the algorithm package that ultimately consumes it.
- Treating a directory with documentation but no `package.xml` as a ROS
  package.
- Assuming all `.pcd`, `.ply`, `.txt`, or `.vgh` files have the same
  source/generated ownership.
