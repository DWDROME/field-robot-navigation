# Configuration And Asset Contracts

> Sources of runtime configuration, static algorithm inputs, and deployment
> resources for this ROS1 workspace.

---

## Source Of Truth

Configuration is assembled by ROS launch order, not by YAML alone. For a
running node, use this precedence:

```text
source-code default
< YAML loaded with rosparam
< later launch <param>
< top-level include arg only when explicitly forwarded and consumed
```

Always inspect the complete user-entry launch graph before changing a value.
The same YAML can intentionally produce different behavior in mapping and
navigation.

## Mainline Configuration Owners

| Concern | Current owner | Mainline status |
|---------|---------------|-----------------|
| Mid360 topics, lidar type, IMU/extrinsics | `src/FAST_LIO/config/mid360.yaml` | Current greenhouse baseline |
| Mapping FAST-LIO overrides | `src/FAST_LIO/launch/mapping_mid360.launch` | `point_filter_num=3` |
| Navigation FAST-LIO overrides | `src/greenhouse_nav/launch/system_greenhouse_base.launch` | `point_filter_num=1`, PCD save disabled |
| Greenhouse mount and topic wiring | `src/greenhouse_nav/launch/*.launch` | Current integration source |
| FAR greenhouse tuning | `src/far_planner/config/greenhouse.yaml` | Loaded by `far_route_planner.launch` |
| Livox device/network setup | `src/livox_ros_driver2/config/*.json` and launch args | Hardware-specific |
| Chassis serial port | `src/dlrobot_robot/launch/base_serial.launch` | Defaults to `/dev/ttyACM0` |

`src/FAST_LIO_SUPER/config/*.yaml` is a separate candidate/research
configuration family. The current greenhouse entries do not load it.
`src/FAST_LIO_LOCALIZATION/config/*` primarily supports standalone or legacy
launches; the current greenhouse path starts only `global_localization.py`.

## Static Assets

Treat these paths according to their role, not only their extension.

| Asset | Role | Required behavior |
|-------|------|-------------------|
| `src/FAST_LIO/PCD/scans.pcd` | Default greenhouse localization map | Must exist before the default navigation map publisher starts |
| `src/local_planner/paths/startPaths.ply` | Local-planner path library input | Versioned algorithm input; missing file terminates the node |
| `src/local_planner/paths/paths.ply` | Local-planner path library input | Versioned algorithm input; missing file terminates the node |
| `src/local_planner/paths/pathList.ply` | Path-to-group mapping | Versioned algorithm input; missing or malformed data terminates the node |
| `src/local_planner/paths/correspondences.txt` | Voxel/path correspondence table | Versioned algorithm input; do not classify as a runtime log |
| `src/far_planner/data/boundary.ply` | FAR boundary input/example | Versioned planner asset |
| `src/far_planner/data/trajectory.txt` | FAR trajectory input/example | Versioned planner asset |
| `src/far_planner/data/boundary_graph.vgh` | Graph input/baseline and possible overwrite target | Check provenance before replacing |
| `src/far_planner/rviz/*.rviz` | Operator visualization profiles | Validate every enabled profile path |
| `src/dlrobot_robot/urdf/*` | Robot description/reference assets | Validate referenced package and mesh resources |

The default navigation requests
`src/far_planner/rviz/greenhouse.rviz`, but that file is absent. Only
`default.rviz` and `matterport.rviz` are currently present. XML parsing or
`roslaunch --nodes` does not prove that this enabled resource is valid.

## Scenario: Change Runtime Configuration Or Assets

### 1. Scope / Trigger

- Editing YAML, JSON, launch params, RViz profiles, URDF/Xacro, map paths, or
  file-backed planner inputs.
- Moving the workspace or deploying it to another host.

### 2. Signatures

- Main mapping entry: `roslaunch greenhouse_nav greenhouse_mapping.launch`
- Main navigation entry:
  `roslaunch greenhouse_nav greenhouse_navigation.launch`
- Main map argument:
  `localization_map=$(find fast_lio)/PCD/scans.pcd`
- FAR config selector: `config=greenhouse`

### 3. Contracts

- Keep one owning value whenever practical; launch overrides must be
  intentional and documented as mode-specific.
- A top-level include argument wins over a child default only when it is
  explicitly forwarded and mapped to the consumed parameter or node argument.
- Package resources referenced through `$(find package)` must belong to a
  real catkin package identified by `package.xml`.
- The three FAST-LIO variants compile `ROOT_DIR` from
  `CMAKE_CURRENT_SOURCE_DIR`; moving source requires CMake reconfiguration or
  a clean rebuild before file output paths are trustworthy.
- Host-specific values include Mid360 network addresses, `/dev/ttyACM0`,
  `/lib/x86_64-linux-gnu/libusb-1.0.so.0`, and FAR's `/tmp` image directory.
  Do not silently replace them with generic defaults.

### 4. Validation & Error Matrix

| Condition | Expected handling |
|-----------|-------------------|
| YAML and later launch param disagree | Treat the later launch value as live behavior |
| Enabled `$(find ...)/resource` is absent | Fail the validation; do not claim a clean launch |
| Required PCD/path asset is absent | Treat as a startup blocker |
| URDF references an absent package or mesh | Mark the robot-description path incomplete |
| Device IP, serial port, or preload library differs by host | Require an explicit deployment override and visible failure |
| Source directory moved after build | Reconfigure/rebuild so compiled `ROOT_DIR` is refreshed |

### 5. Good / Base / Bad Cases

- Good: change the owning YAML or launch arg, trace every override, and
  validate the enabled resource paths.
- Base: mapping uses `point_filter_num=3`; greenhouse navigation deliberately
  overrides it to `1`.
- Bad: edit `FAST_LIO_SUPER/config/mid360.yaml` expecting the baseline
  greenhouse runtime to change.

### 6. Tests Required

- Parse changed YAML/JSON/XML with a structured parser.
- Enumerate the affected user launch with `roslaunch ... --nodes`.
- Check every enabled YAML, JSON, PCD, RViz, URDF mesh, bag, and planner asset
  path after argument expansion.
- For hardware settings, verify on the target host or state clearly that
  hardware validation was not performed.

### 7. Wrong vs Correct

#### Wrong

- "The value in `mid360.yaml` is necessarily the runtime value."
- "All PCD, PLY, TXT, and VGH files are generated output and can be ignored."

#### Correct

- Later launch overrides define the live mode-specific value.
- Classify files by ownership: map output, required algorithm input, retained
  baseline, or deployment resource.

## Known Drift To Preserve As Findings

- `src/FAST_LIO/config/ouster64.yaml` comments that Ouster is type `3`, but
  currently sets `lidar_type: 1`.
- `src/FAST_LIO_LOCALIZATION/launch/localization_avia.launch` declares
  `feature_extract_enable` twice.
- `src/dlrobot_robot/urdf/mini_tank_moveit.urdf` references
  `package://mini_tank_arm/meshes/*.STL`, while no `mini_tank_arm` package is
  present in this workspace.
- Greenhouse Xacro/URDF files are not evidence that the current user launch
  consumes them; verify launch ownership before editing them.

## Primary Evidence

- `src/FAST_LIO/config/`
- `src/FAST_LIO/launch/mapping_mid360.launch`
- `src/greenhouse_nav/launch/system_greenhouse_base.launch`
- `src/greenhouse_nav/launch/far_route_planner.launch`
- `src/far_planner/config/greenhouse.yaml`
- `src/livox_ros_driver2/config/`
- `src/local_planner/src/localPlanner.cpp`
- `src/dlrobot_robot/launch/base_serial.launch`
- `src/dlrobot_robot/urdf/mini_tank_moveit.urdf`
