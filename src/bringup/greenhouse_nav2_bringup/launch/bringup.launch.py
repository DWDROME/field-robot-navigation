#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from greenhouse_nav2_bringup.contracts import BringupContract, ContractError


def _parse_bool(value: str, label: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ContractError(f"{label} must be true or false")


def _require_file(value: str, label: str) -> str:
    path = Path(value).expanduser().resolve()
    if not path.is_file():
        raise ContractError(f"{label} is not a readable file: {path}")
    return str(path)


def _mapping_actions(
    *,
    use_sim_time: bool,
    fast_lio_params: str,
    command_gate_params: str,
    publish_static_tf: bool,
) -> list:
    local_planner_share = get_package_share_directory("cmu_local_planner")
    actions = [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="laser_mapping",
            output="screen",
            parameters=[fast_lio_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrainAnalysis",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "odometry_topic": "/localization/odometry",
                    "registered_cloud_topic": "/localization/registered_cloud",
                    "terrain_map_topic": "/perception/terrain_map",
                    "output_frame": "map",
                    "scanVoxelSize": 0.05,
                    "decayTime": 2.0,
                    "noDecayDis": 4.0,
                    "clearingDis": 8.0,
                    "useSorting": True,
                    "quantileZ": 0.25,
                    "considerDrop": False,
                    "limitGroundLift": False,
                    "maxGroundLift": 0.15,
                    "clearDyObs": False,
                    "minDyObsDis": 0.3,
                    "minDyObsAngle": 0.0,
                    "minDyObsRelZ": -0.5,
                    "absDyObsRelZThre": 0.2,
                    "minDyObsVFOV": -16.0,
                    "maxDyObsVFOV": 16.0,
                    "minDyObsPointNum": 1,
                    "noDataObstacle": False,
                    "noDataBlockSkipNum": 0,
                    "minBlockPointNum": 10,
                    "vehicleHeight": 1.5,
                    "voxelPointUpdateThre": 100,
                    "voxelTimeUpdateThre": 2.0,
                    "minRelZ": -1.5,
                    "maxRelZ": 0.2,
                    "disRatioZ": 0.2,
                }
            ],
        ),
        Node(
            package="cmu_local_planner",
            executable="local_planner_node",
            name="local_planner",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "path_folder": f"{local_planner_share}/paths",
                    "odometry_topic": "/localization/odometry",
                    "registered_cloud_topic": "/localization/registered_cloud",
                    "terrain_map_topic": "/perception/terrain_map",
                    "path_topic": "/planning/local_path",
                    "free_paths_topic": "/planning/free_paths",
                    "goal_topic": "/planning/way_point",
                    "output_frame": "base_link",
                    "use_terrain_analysis": True,
                    "autonomy_mode": True,
                    "autonomy_speed_m_s": 0.6,
                    "max_speed_m_s": 1.0,
                    "goal_x": 5.0,
                    "goal_y": 0.0,
                }
            ],
        ),
        Node(
            package="cmu_path_follower_generic",
            executable="path_follower_generic",
            name="path_follower_generic",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autonomy_mode": True,
                    "odometry_topic": "/localization/odometry",
                    "path_topic": "/planning/local_path",
                    "cmd_vel_topic": "/cmd_vel/nav",
                    "output_frame": "base_link",
                    "autonomy_speed_m_s": 0.6,
                    "max_speed_m_s": 1.0,
                    "odometry_timeout_s": 0.5,
                }
            ],
        ),
        Node(
            package="greenhouse_cmd_gate",
            executable="greenhouse_cmd_gate",
            name="greenhouse_cmd_gate",
            output="screen",
            parameters=[command_gate_params, {"use_sim_time": use_sim_time}],
        ),
    ]
    if publish_static_tf:
        actions.extend(
            [
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="greenhouse_map_to_odom_identity",
                    arguments=[
                        "--x",
                        "0",
                        "--y",
                        "0",
                        "--z",
                        "0",
                        "--roll",
                        "0",
                        "--pitch",
                        "0",
                        "--yaw",
                        "0",
                        "--frame-id",
                        "map",
                        "--child-frame-id",
                        "odom",
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                    output="screen",
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="greenhouse_livox_identity",
                    arguments=[
                        "--x",
                        "0",
                        "--y",
                        "0",
                        "--z",
                        "0",
                        "--roll",
                        "0",
                        "--pitch",
                        "0",
                        "--yaw",
                        "0",
                        "--frame-id",
                        "base_link",
                        "--child-frame-id",
                        "livox_frame",
                    ],
                    parameters=[{"use_sim_time": use_sim_time}],
                    output="screen",
                ),
            ]
        )
    return actions


def _far_actions(*, use_sim_time: bool, far_planner_params: str) -> list:
    return [
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            name="terrain_analysis_ext",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=[
                ("/state_estimation", "/localization/odometry"),
                ("/registered_scan", "/localization/registered_cloud"),
                ("/terrain_map", "/perception/terrain_map"),
                ("/terrain_map_ext", "/perception/terrain_map_ext"),
            ],
        ),
        Node(
            package="far_planner",
            executable="far_planner",
            name="far_planner",
            output="screen",
            parameters=[
                far_planner_params,
                {
                    "use_sim_time": use_sim_time,
                    "sensor_range": 8.0,
                    "terrain_range": 7.5,
                    "map_handler/map_grid_max_length": 20.0,
                    "map_handler/map_grad_max_height": 4.0,
                    "is_static_env": True,
                    "is_opencv_visual": False,
                    "is_pub_boundary": True,
                },
            ],
            remappings=[
                ("/odom_world", "/localization/odometry"),
                ("/terrain_cloud", "/perception/terrain_map_ext"),
                ("/scan_cloud", "/localization/registered_cloud"),
                ("/goal_point", "/planning/global_goal"),
                ("/terrain_local_cloud", "/perception/terrain_map"),
                ("/way_point", "/planning/way_point"),
                ("/navigation_boundary", "/planning/navigation_boundary"),
            ],
        ),
        Node(
            package="graph_decoder",
            executable="graph_decoder",
            name="graph_decoder",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]


def _launch_setup(context):
    config_dir = LaunchConfiguration("config_dir").perform(context)
    profiles = LaunchConfiguration("profiles").perform(context)
    host_profile = LaunchConfiguration("host_profile").perform(context)
    start_runtime = _parse_bool(
        LaunchConfiguration("start_runtime_nodes").perform(context),
        "start_runtime_nodes",
    )
    start_health = _parse_bool(
        LaunchConfiguration("start_health_monitor").perform(context),
        "start_health_monitor",
    )
    publish_static_tf = _parse_bool(
        LaunchConfiguration("publish_static_tf").perform(context),
        "publish_static_tf",
    )
    health_super_client = _parse_bool(
        LaunchConfiguration("health_super_client").perform(context),
        "health_super_client",
    )

    contract = BringupContract.load(config_dir)
    resolved = contract.resolve(
        profiles,
        host_profile,
        require_runtime_ready=start_runtime,
    )

    actions = [
        LogInfo(
            msg=(
                "greenhouse bringup profiles="
                + ",".join(resolved.profiles)
                + f" host={resolved.host_profile}"
                + f" use_sim_time={str(resolved.use_sim_time).lower()}"
                + f" runtime_ready={str(resolved.runtime_ready).lower()}"
            )
        )
    ]

    plan_output = LaunchConfiguration("plan_output").perform(context).strip()
    if plan_output:
        output_path = Path(plan_output).expanduser()
        if not output_path.parent.is_dir():
            raise ContractError(
                f"plan_output parent directory does not exist: {output_path.parent}"
            )
        try:
            output_path.write_text(resolved.to_json() + "\n", encoding="utf-8")
        except OSError as exc:
            raise ContractError(f"cannot write plan_output {output_path}: {exc}") from exc

    if start_runtime and "mapping-fastlio" in resolved.profiles:
        fast_lio_params = _require_file(
            LaunchConfiguration("fast_lio_params").perform(context),
            "fast_lio_params",
        )
        command_gate_params = _require_file(
            LaunchConfiguration("command_gate_params").perform(context),
            "command_gate_params",
        )
        actions.extend(
            _mapping_actions(
                use_sim_time=resolved.use_sim_time,
                fast_lio_params=fast_lio_params,
                command_gate_params=command_gate_params,
                publish_static_tf=publish_static_tf,
            )
        )

    if start_runtime and "navigation-far" in resolved.profiles:
        far_planner_params_value = LaunchConfiguration(
            "far_planner_params"
        ).perform(context).strip()
        if not far_planner_params_value:
            far_planner_params_value = (
                get_package_share_directory("far_planner")
                + "/config/default.yaml"
            )
        far_planner_params = _require_file(
            far_planner_params_value,
            "far_planner_params",
        )
        actions.extend(
            _far_actions(
                use_sim_time=resolved.use_sim_time,
                far_planner_params=far_planner_params,
            )
        )

    if start_health:
        actions.append(
            Node(
                package="greenhouse_nav2_bringup",
                executable="greenhouse_bringup_health",
                name="greenhouse_bringup_health",
                output="screen",
                parameters=[
                    {
                        "config_dir": str(contract.config_dir),
                        "profiles": ",".join(resolved.profiles),
                        "host_profile": resolved.host_profile,
                        "use_sim_time": resolved.use_sim_time,
                    }
                ],
                additional_env=(
                    {"ROS_SUPER_CLIENT": "TRUE"}
                    if health_super_client
                    else {}
                ),
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("greenhouse_nav2_bringup")
    config_dir = f"{package_share}/config"
    return LaunchDescription(
        [
            DeclareLaunchArgument("profiles", default_value="mapping-fastlio"),
            DeclareLaunchArgument("host_profile", default_value="simulation"),
            DeclareLaunchArgument("config_dir", default_value=config_dir),
            DeclareLaunchArgument("start_runtime_nodes", default_value="true"),
            DeclareLaunchArgument("start_health_monitor", default_value="true"),
            DeclareLaunchArgument("publish_static_tf", default_value="true"),
            DeclareLaunchArgument("health_super_client", default_value="false"),
            DeclareLaunchArgument(
                "fast_lio_params",
                default_value=f"{config_dir}/fast-lio-generic-sim.yaml",
            ),
            DeclareLaunchArgument(
                "command_gate_params",
                default_value=f"{config_dir}/command-gate-sim.yaml",
            ),
            DeclareLaunchArgument(
                "far_planner_params",
                default_value="",
            ),
            DeclareLaunchArgument("plan_output", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
