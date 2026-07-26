# Launch Topology

> User entrypoints, recursive launch assembly, and known launch-time boundaries.

---

## Source Of Truth

For runtime composition, trust the XML under `src/greenhouse_nav/launch/`
before package-local prose. Some older examples describe a flat launch file,
but the current greenhouse navigation path is a layered include graph.

## User Entrypoints

| Workflow | Command | Role |
|----------|---------|------|
| Mapping | `roslaunch greenhouse_nav greenhouse_mapping.launch` | Livox Mid360 + baseline FAST-LIO + greenhouse RViz and helper TF |
| Navigation default | `roslaunch greenhouse_nav greenhouse_navigation.launch` | Alias to forward navigation |
| Navigation forward | `roslaunch greenhouse_nav greenhouse_navigation_forward.launch` | Full localization, planning, and chassis chain with `body_to_base_yaw=0.0` |
| Navigation reverse | `roslaunch greenhouse_nav greenhouse_navigation_reverse.launch` | Same chain with `body_to_base_yaw=pi` |
| Compatibility alias | `roslaunch greenhouse_nav system_greenhouse.launch` | Old command that includes the default navigation entry |

Do not run mapping and navigation entrypoints at the same time. Both start
`livox_lidar_publisher2` and a node named `laserMapping`.

## Mapping Include Graph

```text
greenhouse_mapping.launch
├── livox_ros_driver2/launch_ROS1/msg_MID360.launch
│   └── livox_ros_driver2_node (livox_lidar_publisher2)
├── fast_lio/launch/mapping_mid360.launch
│   └── fastlio_mapping (laserMapping)
├── rviz
├── tf_pub_mapping_body_foot
├── tf_pub_mapping_robot_foot_init
└── tf_pub_mapping_body_to_base
```

The entry disables the driver/FAST-LIO RViz flags and starts its own RViz with
`src/greenhouse_nav/rviz/greenhouse_mapping_base_link.rviz`.

## Navigation Include Graph

```text
greenhouse_navigation.launch
└── greenhouse_navigation_forward.launch
    └── system_greenhouse_with_route_planner.launch
        ├── system_greenhouse_base.launch
        │   ├── livox_ros_driver2/launch_ROS1/msg_MID360.launch
        │   ├── fast_lio/fastlio_mapping
        │   ├── fast_lio_localization/global_localization.py
        │   ├── localization_manager.launch
        │   ├── pcl_ros/pcd_to_pointcloud
        │   ├── helper static TF publishers
        │   ├── global_to_local_adapter.launch [default user entry: off]
        │   ├── base_autonomy_cmu.launch
        │   │   ├── terrain_analysis/terrainAnalysis
        │   │   ├── local_planner/localPlanner
        │   │   ├── local_planner/pathFollower
        │   │   └── greenhouse_nav/cmd_vel_stamped_bridge.py
        │   └── dlrobot_robot/base_serial.launch
        └── far_route_planner.launch
            ├── terrain_analysis_ext/terrainAnalysisExt
            ├── far_planner/far_planner
            ├── graph_decoder/graph_decoder [default user entry: off]
            └── rviz/far_rviz [default user entry: on]
```

The reverse entry uses the same graph. Its only intended mode difference is
`body_to_base_yaw=3.141592653589793`.

## Important Assembly Details

### Navigation starts FAST-LIO directly

`system_greenhouse_base.launch` does not include
`fast_lio/launch/mapping_mid360.launch`. It:

1. loads `fast_lio/config/mid360.yaml`,
2. overrides navigation-specific parameters such as `point_filter_num=1` and
   `pcd_save_enable=0`,
3. starts `fast_lio/fastlio_mapping` directly.

The mapping entry does include `mapping_mid360.launch`, whose defaults include
`point_filter_num=3`.

### Navigation avoids the legacy localization launch

The current mainline deliberately does not include
`fast_lio_localization/launch/localization_avia.launch`. That older launch
still references a missing
`fast_lio_localization/launch/PointsCloud2toLaserscan.launch` and uses the
legacy `transform_fusion.py` path.

Instead, `system_greenhouse_base.launch` explicitly starts:

- `fast_lio_localization/global_localization.py`
- `greenhouse_nav/launch/localization_manager.launch`
- `pcl_ros/pcd_to_pointcloud`

This explicit assembly is the current localization source of truth.

### Optional branches

The default forward/reverse user entries set:

- `use_adapter=false`
- `use_graph_decoder=false`
- `localization_rviz=false`
- `use_far_rviz=true`

The optional `global_to_local_adapter` only applies when another component
owns the global `/waypoints` input. It is not part of the default FAR handoff.

This also leaves a mainline prerequisite: both `localPlanner` and
`pathFollower` subscribe to `/speed`, FAR does not publish it, and the only
confirmed repository producer is the disabled adapter. A deployment must
provide `/speed` externally or deliberately enable/replace that producer
before claiming a complete motion chain.

## Internal And Diagnostic Entrypoints

| Launch | Intended use | Hardware behavior |
|--------|--------------|-------------------|
| `system_greenhouse_base.launch` | Base localization, terrain, local planning, bridge, and chassis assembly | Hardware defaults on |
| `system_greenhouse_with_route_planner.launch` | Base assembly plus FAR route planning | Hardware defaults on |
| `system_greenhouse_bag.launch` | Bag replay and module smoke testing | Forces Livox and base serial off |
| `localization_manager.launch` | Localization-manager module test | No full-system assembly |
| `base_autonomy_cmu.launch` | Terrain/local-planner module test | No sensor/localization assembly |
| `far_route_planner.launch` | FAR and extended-terrain module test | No base assembly |

`system_greenhouse_bag.launch` does not play a bag unless `play_bag=true` and
a valid `bag_path` is supplied.

## Known Launch Hazards

### Missing greenhouse FAR RViz profile

The default navigation entries set `use_far_rviz=true` and
`far_route_planner.launch` resolves the RViz argument to:

```text
src/far_planner/rviz/greenhouse.rviz
```

That file does not currently exist. The available profiles are
`default.rviz` and `matterport.rviz`. Expect the FAR RViz process to receive
an invalid config path until the launch argument or profile set is corrected.
Do not claim a clean operator-UI launch based only on XML parse success.

### Default `/speed` producer is absent

The forward/reverse entries disable `global_to_local_adapter`, while
`localPlanner` and `pathFollower` still subscribe to `/speed`. Verify a live
publisher and `std_msgs/Float32` values before treating successful node
enumeration as a motion-ready graph.

### Legacy localization include

`fast_lio_localization/launch/localization_avia.launch` references the missing
`PointsCloud2toLaserscan.launch`. Do not use it as the greenhouse mainline
entry without repairing and validating that package-local launch.

### Alternate MoveBase3D inputs are external

`roslaunch rover_launch move_base_3d.launch` starts the alternate planner
nodes but does not provide all inputs:

- `/rtabmap/cloud_map` must come from an external RTAB-Map setup.
- `/output_filter` is expected, but
  `rover_pcl_filter/pcl_filter_manager.launch` is not included automatically.

Also, `rover_gpr/launch/gpr_path.launch` names a non-workspace package
`gaussian_process_regression`; the alternate stack's main
`move_base_3d.launch` bypasses that file and starts `rover_gpr/gpr_path`
directly.

## Launch Change Rules

- Keep user entrypoints thin; put reusable assembly in internal launch files.
- Pass topics and mount parameters through the existing arg chain instead of
  hard-coding a second value in a child launch.
- Preserve the forward/reverse invariant: only the intended
  `body -> base_link` yaw differs.
- When enabling an optional branch, verify its config and resource paths
  exist, not just its included launch file.
- Validate recursive assembly, not only the file directly edited.

Minimum checks:

```bash
roslaunch greenhouse_nav greenhouse_mapping.launch --nodes
roslaunch greenhouse_nav greenhouse_navigation.launch --nodes
```

Also inspect every expanded `$(find package)/relative/path` used by an enabled
node. `--nodes` alone does not prove that RViz, map, YAML, JSON, or bag
resources are valid.

## Primary Evidence

- `src/greenhouse_nav/launch/greenhouse_mapping.launch`
- `src/greenhouse_nav/launch/greenhouse_navigation*.launch`
- `src/greenhouse_nav/launch/system_greenhouse_base.launch`
- `src/greenhouse_nav/launch/system_greenhouse_with_route_planner.launch`
- `src/greenhouse_nav/launch/system_greenhouse_bag.launch`
- `src/greenhouse_nav/launch/base_autonomy_cmu.launch`
- `src/greenhouse_nav/launch/far_route_planner.launch`
- `src/livox_ros_driver2/launch_ROS1/msg_MID360.launch`
- `src/FAST_LIO/launch/mapping_mid360.launch`
- `src/FAST_LIO_LOCALIZATION/launch/localization_avia.launch`
- `src/MoveBase3D/rover_launch/launch/move_base_3d.launch`
