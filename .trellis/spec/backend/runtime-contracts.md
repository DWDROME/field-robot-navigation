# Runtime Contracts

> Canonical launch, topic, and frame contracts for the current ROS1 greenhouse mainline.

---

## Overview

Treat the launch files under `src/greenhouse_nav/launch/` as the current source of truth for runtime wiring.

- The default greenhouse mainline still runs `FAST_LIO` plus `FAST_LIO_LOCALIZATION`.
- `greenhouse_nav` owns the integration contract exposed to planners and the chassis:
  - launch assembly,
  - helper TF frames,
  - `/state_estimation`,
  - command bridging.
- `FAST_LIO_SUPER` is the active pluggable SLAM branch and a compatible replacement candidate, but it is not the default greenhouse launch target today.
- Some package docs are stale. In particular, prefer `system_greenhouse_base.launch` over older examples that still point to `localization_avia.launch` or missing helper launch files.

Primary evidence:

- `README.md`
- `src/greenhouse_nav/README.md`
- `src/greenhouse_nav/launch/system_greenhouse_base.launch`
- `src/greenhouse_nav/launch/base_autonomy_cmu.launch`
- `src/greenhouse_nav/launch/far_route_planner.launch`
- `src/greenhouse_nav/scripts/localization_manager.py`
- `src/FAST_LIO/src/laserMapping.cpp`
- `src/FAST_LIO_SUPER/src/laserMapping.cpp`
- `src/FAST_LIO_SUPER/docs/ARCHITECTURE.md`
- `src/FAST_LIO_SUPER/docs/SLAM_CORE_API.md`

## Canonical Entrypoints

| Workflow | Canonical entry | Notes |
|----------|-----------------|-------|
| Mapping | `roslaunch greenhouse_nav greenhouse_mapping.launch` | Mid360 + baseline FAST-LIO + greenhouse RViz |
| Navigation default | `roslaunch greenhouse_nav greenhouse_navigation.launch` | Alias to forward mode |
| Navigation forward | `roslaunch greenhouse_nav greenhouse_navigation_forward.launch` | Default operational mode |
| Navigation reverse | `roslaunch greenhouse_nav greenhouse_navigation_reverse.launch` | Same chain, but `body -> base_link` yaw becomes `pi` |
| Internal base assembly | `src/greenhouse_nav/launch/system_greenhouse_base.launch` | Localization, helper TF, terrain, local planner, command bridge, and chassis |
| Internal route assembly | `src/greenhouse_nav/launch/system_greenhouse_with_route_planner.launch` | Base assembly plus FAR and extended terrain |
| Bag/smoke assembly | `src/greenhouse_nav/launch/system_greenhouse_bag.launch` | Disables Livox and base serial; optional bag playback |

See [Launch Topology](./launch-topology.md) for the recursive include graph and
enabled/disabled branches. See
[ROS Interface Contracts](./ros-interface-contracts.md) for the distinction
between node-native topics and the names produced by greenhouse launch remaps.

## Topic And Frame Ownership

| Contract | Producer / owner | Main consumers | Notes |
|----------|------------------|----------------|-------|
| `/livox/lidar`, `/livox/imu` | `livox_ros_driver2` and FAST-LIO YAML topic config | `FAST_LIO`, `FAST_LIO_SUPER`, `FAST_LIO_LOCALIZATION` | Current Mid360 defaults live in `config/mid360.yaml` |
| `/cloud_registered` | `FAST_LIO` or `FAST_LIO_SUPER` | `terrain_analysis`, `terrain_analysis_ext`, `far_planner` | Registered scan in `camera_init`; terrain/FAR wrappers remap their native scan topics to it |
| `/cloud_registered_body` | `FAST_LIO` or `FAST_LIO_SUPER` | operator/debug consumers | Body-frame registered scan |
| `/Odometry` | `FAST_LIO` or `FAST_LIO_SUPER` | `global_localization.py`, `localization_manager.py` | Native FAST-LIO output is `camera_init -> body` |
| `/map_to_odom` | `FAST_LIO_LOCALIZATION/global_localization.py` | `localization_manager.py`, legacy `transform_fusion.py` | Published only for ICP fitness above `0.95`; message has `frame_id=map` but no child frame |
| `/state_estimation` | `greenhouse_nav/localization_manager.py` | `terrain_analysis`, `terrain_analysis_ext`, `far_planner`, `local_planner`, adapter | Mainline public odometry contract |
| `/terrain_map` | `terrainAnalysis`, started by `base_autonomy_cmu.launch` | `local_planner`, `terrain_analysis_ext`, `far_planner` | Local obstacle/terrain layer, stamped in `map` |
| `/terrain_map_ext` | `terrainAnalysisExt`, started by `far_route_planner.launch` | `far_planner` | Extended terrain layer, stamped in `map` |
| `/goal_point` | operator or upstream goal publisher | `far_planner` | Mainline global goal input |
| `/way_point` | `far_planner` or optional adapter | `local_planner` | Local waypoint handoff |
| `/navigation_boundary` | `far_planner` | `local_planner` | Planner boundary contract |
| `/path` | `localPlanner` | `pathFollower` | `nav_msgs/Path` in frame `vehicle`; a one-point zero path is the no-feasible-path stop signal |
| `/speed` | optional `greenhouse_nav/global_to_local_adapter.py` or an external producer | `localPlanner`, `pathFollower` | Default forward/reverse entries disable the only confirmed repository producer |
| `/cmd_vel_stamped` | `pathFollower` through launch remap | `cmd_vel_stamped_bridge.py` | `geometry_msgs/TwistStamped`; node-native name is `/cmd_vel` |
| `/cmd_vel` | `cmd_vel_stamped_bridge.py` | `dlrobot_robot` | Final chassis command topic |
| `camera_init -> body` | native FAST-LIO TF | `greenhouse_nav` helper frames | Preserve this semantic in baseline and SUPER |
| `map -> camera_init` | `greenhouse_nav/localization_manager.py` | TF consumers | Broadcast from `map_to_odom` |
| `body -> body_foot` | `system_greenhouse_base.launch` static TF | RViz/operator workflows | Horizontal helper frame |
| `camera_init -> robot_foot_init` | `system_greenhouse_base.launch` static TF | greenhouse mapping RViz | Horizontal helper world frame |
| `body -> base_link` | `system_greenhouse_base.launch` static TF | navigation downstream | Mount compensation and forward/reverse mode live here |

## Scenario: Greenhouse Mainline Runtime

### 1. Scope / Trigger

- Trigger: editing launch assembly, topic remaps, TF semantics, planner handoff, or localization output.
- Trigger: investigating why mapping works but navigation does not, or why downstream planners see the wrong frame/topic.

### 2. Signatures

- Mapping entry:
  - `roslaunch greenhouse_nav greenhouse_mapping.launch`
- Navigation entry:
  - `roslaunch greenhouse_nav greenhouse_navigation.launch`
  - `roslaunch greenhouse_nav greenhouse_navigation_forward.launch`
  - `roslaunch greenhouse_nav greenhouse_navigation_reverse.launch`
- Internal assembly:
  - `src/greenhouse_nav/launch/system_greenhouse_base.launch`
  - `src/greenhouse_nav/launch/base_autonomy_cmu.launch`
  - `src/greenhouse_nav/launch/far_route_planner.launch`

### 3. Contracts

- Mapping stack contract:
  - use baseline `FAST_LIO` today, not `FAST_LIO_SUPER`
  - keep native FAST-LIO world/body semantics: `camera_init -> body`
  - greenhouse RViz uses `robot_foot_init` as the default fixed frame so the cloud looks level to operators
- Localization contract:
  - subscribe to `/Odometry`
  - subscribe to `/map_to_odom`
  - publish `/state_estimation`
  - publish odometry with `header.frame_id = map` and `child_frame_id = base_link`
  - broadcast TF from `map` to `camera_init`
  - treat `/map_to_odom` as a historical topic name for a pose matrix: its
    publisher does not set `child_frame_id` or broadcast a standard
    `map -> odom` TF
- Planner handoff contract:
  - `terrain_analysis`: native `/registered_scan`, remapped from `/cloud_registered`, -> `/terrain_map`
  - `terrain_analysis_ext`: native `/registered_scan`, remapped from `/cloud_registered`, plus `/terrain_map` -> `/terrain_map_ext`
  - terrain outputs are stamped in `map` and the terrain nodes do not transform inputs; inputs must already share map-coordinate semantics
  - `far_planner`: native `/odom_world` + `/scan_cloud` + `/terrain_local_cloud` + `/terrain_cloud` are remapped from `/state_estimation` + `/cloud_registered` + `/terrain_map` + `/terrain_map_ext`; FAR can use TF to normalize inputs to `world_frame`
  - `local_planner`: `/state_estimation` + `/terrain_map` + `/way_point` + `/navigation_boundary` -> `/path`
  - `pathFollower`: `/state_estimation` + `/path` + `/speed` -> native `/cmd_vel` (`TwistStamped`), remapped to `/cmd_vel_stamped`
  - `cmd_vel_stamped_bridge.py`: `/cmd_vel_stamped` -> `/cmd_vel`
  - the default forward/reverse entries set `use_adapter=false`; therefore
    they have no confirmed in-repository `/speed` producer and require an
    external owner or a deliberate wiring change
- Mount compensation contract:
  - do not push the greenhouse pitch compensation into Livox driver extrinsics or FAST-LIO YAML as the first choice
  - keep greenhouse-specific mount compensation in `system_greenhouse_base.launch` and `localization_manager.py`
  - default pitch compensation is `-0.471239` rad
- Optional adapter contract:
  - `global_to_local_adapter` is off by default in the main navigation entry
  - only enable it when an upstream global path publisher owns `/waypoints`

### 4. Validation & Error Matrix

| Condition | Expected behavior | Inspect first |
|-----------|-------------------|---------------|
| RViz cloud looks tilted during mapping | `camera_init` still reflects sensor startup pose; greenhouse RViz should use `robot_foot_init` | `README.md`, `greenhouse_mapping.launch`, RViz config |
| `/state_estimation` missing | localization manager waits for `/Odometry`; `/map_to_odom` is optional until relocalization succeeds | `localization_manager.py`, `system_greenhouse_base.launch` |
| Robot pose is offset or pointing backward | `body -> base_link` mount or reverse-mode yaw is wrong | `system_greenhouse_base.launch`, `greenhouse_navigation_reverse.launch` |
| Global planner has no waypoint output | FAR route planner is missing `/goal_point`, `/terrain_map_ext`, or `/state_estimation` | `far_route_planner.launch`, `terrain_analysis_ext` wiring |
| Robot has a path but chassis does not move | `/cmd_vel_stamped` is not bridged to `/cmd_vel`, or base serial layer is down | `base_autonomy_cmu.launch`, `cmd_vel_stamped_bridge.py`, `dlrobot_robot/base_serial.launch` |
| Path exists but follower speed remains zero/stale | default entry has no confirmed repository `/speed` producer | `pathFollower.cpp`, `localPlanner.cpp`, `global_to_local_adapter.py`, top-level `use_adapter` |
| A doc says to include `localization_avia.launch` directly | treat it as legacy/reference guidance, not the current greenhouse assembly | `system_greenhouse_base.launch`, `greenhouse_nav/CLAUDE.md` |
| FAR RViz repeatedly fails or opens without the expected profile | default navigation asks for missing `far_planner/rviz/greenhouse.rviz` | `greenhouse_navigation_forward.launch`, `far_route_planner.launch`, `far_planner/rviz/` |

### 5. Good / Base / Bad Cases

- Good:
  - mapping uses `greenhouse_mapping.launch`
  - navigation uses `greenhouse_navigation*.launch`
  - `/state_estimation` remains the public odometry contract for planning packages
  - `/speed` has one explicit upstream owner before motion is enabled
- Base:
  - native FAST-LIO still publishes `/Odometry` as `camera_init -> body`
  - greenhouse integration adds helper TF and `map -> base_link` semantics without rewriting FAST-LIO internals
- Bad:
  - changing planner consumers to read `/Odometry` directly
  - moving mount compensation into several unrelated packages
  - reviving the old `transform_fusion.py` path as the mainline without checking downstream assumptions
  - assuming FAR publishes `/speed` because it publishes `/way_point`

### 6. Tests Required

- Launch-only edits:
  - `roslaunch greenhouse_nav greenhouse_mapping.launch --nodes`
  - `roslaunch greenhouse_nav greenhouse_navigation.launch --nodes`
- Python bridge or localization edits:
  - rebuild/install if packaging changed
  - targeted smoke on the owning launch path
- Cross-package runtime contract edits:
  - `catkin_make`
  - both mapping and navigation launch enumeration
  - manual topic/frame smoke check on the touched contract
  - verify `/speed` type, publisher ownership, and live values before a motion
    test

### 7. Wrong vs Correct

#### Wrong

- "The greenhouse navigation stack should consume FAST-LIO `/Odometry` directly because that is what the SLAM package publishes."
- "If the mapping cloud looks tilted, fix `FAST_LIO/config/mid360.yaml` extrinsics first."

#### Correct

- The greenhouse navigation stack consumes `/state_estimation`, which is owned by `greenhouse_nav/localization_manager.py`.
- A tilted view in RViz is often a fixed-frame semantics issue; inspect `robot_foot_init`, `body_foot`, and `body -> base_link` helper TF before rewriting FAST-LIO internals.

## Scenario: FAST_LIO_SUPER Compatibility Boundary

### 1. Scope / Trigger

- Trigger: changing `FAST_LIO_SUPER`, `slam_octvox`, or deciding whether greenhouse runtime can swap from `FAST_LIO` to `FAST_LIO_SUPER`.
- Trigger: editing `slam_core/map_backend`, backend selection, or reuse boundaries across the related SLAM packages.

### 2. Signatures

- Package entry:
  - `roslaunch fast_lio_super mapping_mid360.launch`
- Backend selection params:
  - `slam_core/map_backend`
  - `slam_core/voxel_size`
  - `slam_core/capacity`
  - `slam_core/enable_lru`
  - `slam_core/omp_threads`

### 3. Contracts

- ROS compatibility contract:
  - keep node shape, topic names, TF names, and main parameters aligned with baseline `FAST_LIO`
  - continue publishing `/cloud_registered`, `/cloud_registered_body`, `/cloud_effected`, `/Laser_map`, `/Odometry`, and `/path`
  - continue using `camera_init` and `body` as the native SLAM frame pair
- `slam_core` contract:
  - `laserMapping.cpp` chooses the map backend at runtime
  - map access goes through `IMapManager`
  - neighbor lookup goes through `INeighborSearch`
  - do not reintroduce direct global `KD_TREE` ownership in `laserMapping.cpp`
- Reuse boundary:
  - `FAST_LIO_SUPER` is the active pluggable-backend branch
  - `slam_octvox` is vendored support for that branch
  - `FAST_LIO_LOCALIZATION` remains a separate reference/runtime package and should not be merged casually into `FAST_LIO_SUPER`

### 4. Validation & Error Matrix

| Condition | Expected behavior | Inspect first |
|-----------|-------------------|---------------|
| `slam_core/map_backend` is unknown | runtime warns and falls back to `ikdtree` | `FAST_LIO_SUPER/src/laserMapping.cpp` |
| A downstream package breaks after swapping to SUPER | public ROS contracts should match baseline FAST-LIO | `FAST_LIO_SUPER/docs/ARCHITECTURE.md`, topic publishers in `laserMapping.cpp` |
| A task wants code from `FAST_LIO_LOCALIZATION` copied into SUPER | stop and inspect license plus package-role boundary first | package docs, `LICENSE`, `package.xml`, `CLAUDE.md` |

### 5. Good / Base / Bad Cases

- Good:
  - greenhouse launch can reason about SUPER as a drop-in SLAM candidate because ROS contracts stay stable
  - backend-specific logic stays behind `slam_core`
- Base:
  - unknown backend names warn and fall back to `ikdtree`
  - default greenhouse runtime still launches baseline `FAST_LIO`
- Bad:
  - changing ROS topic/frame names in SUPER to expose backend internals
  - treating `package.xml` license fields as the only reuse authority
  - mixing localization-only code into SUPER without checking runtime role and license consequences

### 6. Tests Required

- `catkin_make`
- `roslaunch fast_lio_super mapping_mid360.launch --nodes`
- targeted diff review of topic publisher names and frame IDs against baseline `FAST_LIO`

### 7. Wrong vs Correct

#### Wrong

- "Because `FAST_LIO_SUPER` is in the same workspace, it is safe to absorb code from `FAST_LIO_LOCALIZATION`."
- "Switching to SUPER is the right place to rename `/Odometry` or `camera_init`."

#### Correct

- Treat `FAST_LIO_LOCALIZATION` as a separate package and license boundary until a deliberate design and legal review says otherwise.
- Keep ROS contracts stable and move backend variation behind `slam_core`.
