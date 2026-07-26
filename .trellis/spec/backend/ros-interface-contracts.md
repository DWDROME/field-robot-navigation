# ROS Interface Contracts

> Node-native topics, greenhouse launch remaps, message types, and TF ownership.

---

## Read The Contract At Two Levels

ROS source files describe a node's native interface. The greenhouse launch
files may remap that interface into the system-wide contract.

Always record both levels:

1. inspect the publisher/subscriber name and message type in source;
2. inspect every enclosing launch file for remaps and arguments;
3. use the fully assembled greenhouse name when describing the mainline;
4. keep the native name when documenting standalone package behavior.

For example, `terrainAnalysis` natively subscribes to `/registered_scan`.
`base_autonomy_cmu.launch` remaps it to `/cloud_registered`. Calling
`/cloud_registered` the node-native topic would hide a real integration
boundary.

## Mainline Data Flow

```text
/livox/lidar + /livox/imu
    -> FAST_LIO laserMapping
       -> /cloud_registered              [PointCloud2, camera_init]
       -> /Odometry                      [Odometry, camera_init -> body]
       -> TF camera_init -> body

/map + /initialpose + /cloud_registered + /Odometry
    -> global_localization.py
       -> /map_to_odom                    [Odometry pose, frame map]

/Odometry + /map_to_odom
    -> localization_manager.py
       -> /state_estimation              [Odometry, map -> base_link]
       -> TF map -> camera_init

/cloud_registered + /state_estimation
    -> terrainAnalysis
       -> /terrain_map                   [PointCloud2, map]

/cloud_registered + /terrain_map + /state_estimation
    -> terrainAnalysisExt
       -> /terrain_map_ext               [PointCloud2, map]

/state_estimation + terrain layers + /cloud_registered + /goal_point
    -> FAR
       -> /way_point + /navigation_boundary

/state_estimation + /terrain_map + /way_point + /navigation_boundary
    -> localPlanner
       -> /path                          [Path, vehicle]

/state_estimation + /path + /speed
    -> pathFollower
       -> native /cmd_vel                [TwistStamped, vehicle]
       -> remapped /cmd_vel_stamped
    -> cmd_vel_stamped_bridge.py
       -> /cmd_vel                       [Twist]
    -> dlrobot_robot
```

Default forward/reverse user entries set `use_adapter=false`. FAR publishes
`/way_point` and `/navigation_boundary`, but not `/speed`. The only confirmed
repository producer of `/speed` is the optional
`greenhouse_nav/global_to_local_adapter.py`; otherwise an external producer is
a runtime prerequisite.

## FAST-LIO Baseline And SUPER

The baseline and SUPER executables intentionally expose the same public ROS
shape.

| Interface | Type / frame | Contract |
|-----------|--------------|----------|
| `/livox/lidar` | Livox input selected by YAML | Native default LiDAR input |
| `/livox/imu` | `sensor_msgs/Imu` | Native default IMU input |
| `/cloud_registered` | `sensor_msgs/PointCloud2`, `camera_init` | Registered world-frame scan |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2`, `body` | Registered body-frame scan |
| `/cloud_effected` | `sensor_msgs/PointCloud2`, `camera_init` | Debug/effective feature cloud |
| `/Laser_map` | `sensor_msgs/PointCloud2`, `camera_init` | Published map cloud |
| `/Odometry` | `nav_msgs/Odometry`, `camera_init -> body` | Native odometry |
| `/path` | `nav_msgs/Path`, `camera_init` | Native SLAM path |
| TF | `camera_init -> body` | Native SLAM transform |

The executable boundary differs:

- baseline: package `fast_lio`, executable `fastlio_mapping`;
- SUPER: package `fast_lio_super`, executable `fastlio_super_mapping`.

Do not run both in one graph with their default node name `laserMapping`.
Do not rename topics or frames merely to expose the SUPER map backend.

## Global Localization And State Estimation

### `global_localization.py`

Native inputs:

- subscribes to `/cloud_registered`;
- subscribes to `/Odometry`;
- blocks for `/map`;
- blocks for `/initialpose`.

Outputs:

- `/cur_scan_in_map`;
- `/submap`;
- `/map_to_odom`.

`/map_to_odom` is published only when ICP fitness is greater than `0.95`.
The message sets `header.frame_id = map` and `pose.pose`; it does not set a
child frame and the script does not broadcast TF.

The topic name is historical. Its payload is consumed directly as a pose
matrix by `localization_manager.py`; it is not a complete standard
`map -> odom` odometry/TF contract.

### `localization_manager.py`

Default inputs:

- `/Odometry`;
- `/map_to_odom`.

Outputs:

- `/state_estimation` as `nav_msgs/Odometry` with message semantics
  `map -> base_link`;
- TF `map -> camera_init`.

The composed pose is:

```text
map_to_base = map_to_odom * odom_to_body * body_to_base
```

The manager does not publish until `/Odometry` exists. Before a valid
`/map_to_odom` arrives, it deliberately uses the identity transform. The
default `body -> base_link` mount compensation is translation
`(0, -0.2, 0)` and RPY `(0, -0.471239, 0)`.

The legacy `FAST_LIO_LOCALIZATION/scripts/transform_fusion.py` also publishes
`/state_estimation` and broadcasts `map -> camera_init`. Never run it beside
the current manager without explicitly resolving duplicate ownership.

## Terrain Analysis

| Node | Native inputs | Native output | Greenhouse mainline remap |
|------|---------------|---------------|---------------------------|
| `terrainAnalysis` | `/state_estimation`, `/registered_scan` | `/terrain_map` | `/registered_scan` -> `/cloud_registered` |
| `terrainAnalysisExt` | `/state_estimation`, `/registered_scan`, `/terrain_map` | `/terrain_map_ext` | `/registered_scan` -> `/cloud_registered` |

`base_autonomy_cmu.launch` owns the mainline `terrainAnalysis` start.
`far_route_planner.launch` owns the mainline `terrainAnalysisExt` start.

Both nodes stamp output clouds with frame `map`. They do not transform incoming
clouds into `map`; their inputs must already share the same map-coordinate
semantics. A matching topic name is not sufficient proof of frame
compatibility.

## FAR Route Planner

FAR's native interface is remapped by `far_route_planner.launch`:

| Native FAR topic | Mainline topic | Role |
|------------------|----------------|------|
| `/odom_world` | `/state_estimation` | Robot state |
| `/terrain_cloud` | `/terrain_map_ext` | Extended terrain |
| `/scan_cloud` | `/cloud_registered` | Registered scan |
| `/terrain_local_cloud` | `/terrain_map` | Local terrain |
| `/goal_point` | `/goal_point` | Route goal |
| `/way_point` | `/way_point` | Local-planner handoff |
| `/navigation_boundary` | `/navigation_boundary` | Planning boundary |
| `/robot_vgraph` | `/robot_vgraph` | Visibility graph |

Unlike the terrain nodes, FAR can use TF to normalize incoming messages to its
configured `world_frame`. The greenhouse config uses `world_frame: map`.

Additional FAR control and diagnostic interfaces include `/joy`,
`/reset_visibility_graph`, `/update_visibility_graph`, `/read_file_dir`,
`/save_file_dir`, `/planning_attemptable`, `/decoded_vgraph`, metrics, debug
clouds, and visualization markers. Treat these as auxiliary interfaces, not
as replacements for the mainline planning chain.

## Local Planning And Chassis

### `localPlanner`

Native inputs include:

- `/state_estimation`;
- `/registered_scan`;
- `/terrain_map`;
- `/joy`;
- `/way_point`;
- `/speed`;
- `/navigation_boundary`;
- `/added_obstacles`;
- `/check_obstacle`.

It publishes:

- `/path` as `nav_msgs/Path` in frame `vehicle`;
- `/free_paths`.

When no feasible path exists, it publishes a single-point zero path. That is a
stop signal to the follower, not a valid route.

### `pathFollower`

Native inputs include `/state_estimation`, `/path`, `/joy`, `/speed`, and
`/stop`. Its native `/cmd_vel` publisher is
`geometry_msgs/TwistStamped`, stamped in frame `vehicle`.

`base_autonomy_cmu.launch` remaps that native `/cmd_vel` to
`/cmd_vel_stamped`. This name/type distinction is deliberate:

- `/cmd_vel_stamped`: `geometry_msgs/TwistStamped`;
- `/cmd_vel`: `geometry_msgs/Twist`.

For a path with one point or fewer, the follower commands zero linear speed.
`/stop >= 1` stops linear motion; `/stop >= 2` also stops angular motion. The
follower does not proactively publish a zero command before receiving a path,
so absence of command is not equivalent to an explicit stop.

Both `localPlanner` and `pathFollower` subscribe to `/speed`.
`far_planner` does not publish that topic. With the default adapter disabled,
the launch graph requires an external `/speed` owner; launch enumeration alone
does not prove that path tracking can command motion.

### Bridge And Chassis

`cmd_vel_stamped_bridge.py` publishes `msg.twist` from
`/cmd_vel_stamped` onto `/cmd_vel`, intentionally discarding the stamped
header and frame.

`dlrobot_robot` subscribes to relative topic `cmd_vel` as
`geometry_msgs/Twist`. It publishes relative `odom`, `imu`, and
`PowerVoltage`; its default odometry message is
`odom_combined -> base_footprint`, while IMU messages use `imu_link`.

The chassis odometry tree is separate from the greenhouse localization tree
unless another launch explicitly connects them.

## TF Ownership And Conflict Rules

The current greenhouse mainline owns this tree:

```text
map
└── camera_init                 localization_manager.py
    ├── body                    FAST_LIO or FAST_LIO_SUPER
    │   ├── body_foot           greenhouse static transform
    │   └── base_link           greenhouse static transform
    └── robot_foot_init         greenhouse static transform
```

Related message semantics are not additional TF edges:

- `/Odometry`: `camera_init -> body`;
- `/state_estimation`: `map -> base_link`;
- chassis `odom`: `odom_combined -> base_footprint`.

Before adding or enabling a transform publisher:

- identify the single owner of every parent/child pair;
- distinguish an odometry message's frame fields from a broadcast TF;
- never run both localization managers that own `map -> camera_init`;
- never run baseline and SUPER `laserMapping` together on default names;
- verify that point clouds consumed by terrain analysis already use map
  coordinates, because those nodes do not perform TF conversion.

## Review Checklist

- [ ] Native source topics and launch-remapped topics are both documented.
- [ ] Message type is checked at the publisher/subscriber, not inferred from
      the topic name.
- [ ] Point-cloud frame IDs are checked at publication sites.
- [ ] TF broadcaster ownership is unique for each parent/child pair.
- [ ] `/map_to_odom` absence and its fitness gate are handled deliberately.
- [ ] `/map_to_odom` is not described as a fully declared `map -> odom` TF.
- [ ] `/speed` has one confirmed publisher in the assembled motion graph.
- [ ] `/cmd_vel_stamped` remains `TwistStamped`; chassis `/cmd_vel` remains
      `Twist`.
- [ ] Terrain inputs already satisfy map-coordinate semantics.
- [ ] Changes are traced through the full launch include graph.

## Primary Evidence

- `src/FAST_LIO/src/laserMapping.cpp`
- `src/FAST_LIO_SUPER/src/laserMapping.cpp`
- `src/FAST_LIO_LOCALIZATION/scripts/global_localization.py`
- `src/FAST_LIO_LOCALIZATION/scripts/transform_fusion.py`
- `src/greenhouse_nav/scripts/localization_manager.py`
- `src/greenhouse_nav/scripts/cmd_vel_stamped_bridge.py`
- `src/greenhouse_nav/launch/base_autonomy_cmu.launch`
- `src/greenhouse_nav/launch/far_route_planner.launch`
- `src/terrain_analysis/src/terrainAnalysis.cpp`
- `src/terrain_analysis_ext/src/terrainAnalysisExt.cpp`
- `src/local_planner/src/localPlanner.cpp`
- `src/local_planner/src/pathFollower.cpp`
- `src/dlrobot_robot/src/dlrobot_robot.cpp`
