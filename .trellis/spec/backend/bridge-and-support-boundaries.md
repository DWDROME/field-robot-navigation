# Bridge And Support Boundaries

> Current behavior of adapter, decoder, conversion, and file-driven helper packages.

---

## Classification Rule

A support package is not part of the default greenhouse runtime merely because
it is present in `src/` or has a launch file.

Classify it from the assembled user entry:

- **mainline**: enabled by `greenhouse_navigation*.launch`;
- **optional**: reachable through an explicit launch argument, but disabled by
  the user entry;
- **standalone/reference**: not included by the greenhouse mainline.

## Current Boundary Map

| Component | Boundary | Default greenhouse status |
|-----------|----------|---------------------------|
| `cmd_vel_stamped_bridge.py` | `TwistStamped` -> `Twist` | Enabled |
| `global_to_local_adapter.py` | global `Path` + state -> local waypoint + speed | Disabled by user entry |
| `graph_decoder` | visibility graph message <-> internal graph/file form | Disabled by user entry |
| `livox_to_pointcloud2` | Livox `CustomMsg` -> `PointCloud2` | Standalone |
| `boundary_handler` | boundary/trajectory files -> saved graph + RViz markers | Standalone |

## Command Bridge

`greenhouse_nav/scripts/cmd_vel_stamped_bridge.py` is the production command
type boundary:

```text
/cmd_vel_stamped  geometry_msgs/TwistStamped
    -> publish msg.twist
/cmd_vel          geometry_msgs/Twist
    -> dlrobot_robot
```

The bridge intentionally discards header, timestamp, and frame metadata.
`base_autonomy_cmu.launch` always starts it in the current mainline.

Operational limitations:

- no command-age check;
- no stale-input timeout;
- no autonomous zero-command watchdog;
- stopping publication upstream does not make the bridge publish a stop.

The upstream `pathFollower` natively advertises `/cmd_vel` as
`TwistStamped`; the greenhouse launch remap to `/cmd_vel_stamped` is required
to avoid a type collision with the chassis `/cmd_vel` topic.

## Global-To-Local Adapter

The adapter implementation belongs to package `greenhouse_nav`; the separate
`src/global_to_local_adapter/` directory has no `package.xml`.

Inputs:

- `/state_estimation`: `nav_msgs/Odometry`;
- `/waypoints`: `nav_msgs/Path`.

Outputs:

- `/way_point`: `geometry_msgs/PointStamped`;
- `/speed`: `std_msgs/Float32`;
- `/adapter/waypoint_window`: debug `nav_msgs/Path`.

It chooses a nearest path point and a look-ahead target. If required data is
missing or path/state frame IDs disagree, it publishes stop speed and does not
publish a new local waypoint.

Runtime status:

- `system_greenhouse_base.launch` defaults `use_adapter=true` when started
  directly;
- forward and reverse user entries explicitly override it to `false`;
- FAR owns `/way_point` in the default full route-planner chain;
- `/speed` is a separate autonomy/control input consumed by the path follower;
  no FAR publisher for it was found in the repository.

Enabling the adapter is therefore an ownership choice, not merely an extra
observer. It conflicts with FAR on `/way_point` and additionally becomes a
publisher of `/speed`; ensure each topic has one intended upstream owner.

The retained `src/global_to_local_adapter/CLAUDE.md` topic description is not
the source of truth; the installed implementation and launch are under
`src/greenhouse_nav/`.

## Graph Decoder

Native interfaces:

- subscribes `/robot_vgraph` as `visibility_graph_msg/Graph`;
- publishes relative topic `decoded_vgraph` as the same type;
- publishes `/graph_decoder_viz` markers;
- subscribes `/save_file_dir` and `/read_file_dir`;
- serves `/request_graph_service`.

The default greenhouse user entries pass `use_graph_decoder=false`.
`far_route_planner.launch` can enable it explicitly.

The package-local `decoder.launch` contains a stale remap:

```text
from /planner_nav_graph to <graph_topic>
```

The node actually subscribes to `/robot_vgraph`, so that remap does not affect
the real subscription. Prefer the remap in
`greenhouse_nav/launch/far_route_planner.launch`, or repair and validate the
standalone launch before relying on it.

The package `CLAUDE.md` also names outputs not present in the current source.
Use `decoder_node.cpp` as the contract authority.

## Livox-To-PointCloud2

Native inputs:

- `/livox/lidar`: `livox_ros_driver2/CustomMsg`;
- `/livox/imu`: `sensor_msgs/Imu`.

Output:

- `/livox/lidar_pc2`: `sensor_msgs/PointCloud2`, frame `livox_frame`.

This package is not included by the current greenhouse entry. FAST-LIO
consumes the Livox custom message directly.

The converter is not a format-only bridge:

- it encodes line/time information into point intensity;
- it encodes reflectivity into curvature;
- it performs a fixed axis conversion and `sensor2novatel` transform;
- it waits for transform state derived from IMU data before publishing.

Known hazards:

- the final point's `offset_time` is used as a divisor without an explicit
  zero guard;
- IMU covariance arrays carry position/Euler data as a private protocol;
- `sensor2world` is computed but the point loop applies `sensor2novatel`;
- output frame is fixed to `livox_frame`;
- build metadata retains empty message-generation template code.

Treat any use of this package as a sensor/extrinsic contract change, not as a
transparent ROS message conversion.

## Boundary Handler

Despite older package documentation, the current implementation does not
subscribe to `/cloud_registered` or `/terrain_map`, and it does not publish
`/navigation_boundary`.

Its current runtime is file-driven:

```text
boundary PLY + trajectory text
    -> internal graph extraction
    -> graph file output
    -> /graph_decoder_viz + /free_p_viz markers
```

The standalone launch defaults its folder to `far_planner/data/`. Missing or
malformed input files cause process termination. The code can encode a
`visibility_graph_msg/Graph`, but the current run path saves files rather than
publishing that message.

Build portability hazards include OpenCV usage absent from `package.xml` and
hard-coded `/usr/local` and `/usr` search paths in CMake.

Classify `boundary_handler` as standalone/offline support until a launch path
and ROS topic contract prove otherwise.

## Review Rules

- Verify enablement from the user entry, not from an internal launch default.
- Record exact message types at both sides of every bridge.
- Treat a remap as part of the contract; standalone node behavior may differ.
- Avoid two publishers owning `/way_point`, `/speed`, or `/cmd_vel`.
- Do not assume a conversion preserves frame or timestamp metadata.
- Prefer source and assembled launch behavior over stale package prose.
- For file-driven tools, validate every default input/output path.

## Primary Evidence

- `src/greenhouse_nav/launch/greenhouse_navigation_forward.launch`
- `src/greenhouse_nav/launch/greenhouse_navigation_reverse.launch`
- `src/greenhouse_nav/launch/system_greenhouse_base.launch`
- `src/greenhouse_nav/launch/base_autonomy_cmu.launch`
- `src/greenhouse_nav/launch/far_route_planner.launch`
- `src/greenhouse_nav/scripts/cmd_vel_stamped_bridge.py`
- `src/greenhouse_nav/scripts/global_to_local_adapter.py`
- `src/graph_decoder/src/decoder_node.cpp`
- `src/graph_decoder/launch/decoder.launch`
- `src/livox_to_pointcloud2/src/livox_repub.cpp`
- `src/boundary_handler/src/graph_extractor.cpp`
- `src/boundary_handler/launch/boundary_handler.launch`
