"""CMU local planning and following, sharing deployment, mission and command guards."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from greenhouse_mppi_navigation.deployment import load


def setup(context):
    deployment = load(LaunchConfiguration('deployment').perform(context), resources=True)
    if deployment['controllers'] != ['cmu']:
        raise ValueError('cmu.launch.py requires controllers: [cmu]')
    sim = deployment['mode'] == 'simulation'
    chassis = deployment['chassis']
    limits = chassis['limits']
    footprint = chassis['footprint']
    # Scale against the planner's declared searchRadius; keep the existing path data intact.
    scale = max(1.0, max(math.hypot(x, y) for x, y in footprint) / 0.45)
    paths = Path(get_package_share_directory('cmu_local_planner')) / 'paths'
    return [
        Node(package='greenhouse_cloud_normalizer', executable='greenhouse_cloud_normalizer',
             name='cmu_terrain_to_odom', output='screen', parameters=[{
                 'use_sim_time': sim, 'input_topic': '/perception/terrain_map',
                 'output_topic': '/perception/terrain_map_odom', 'target_frame': 'odom'}]),
        Node(package='cmu_local_planner', executable='local_planner_node',
             name='local_planner', output='screen', parameters=[{
                 'use_sim_time': sim, 'path_folder': str(paths), 'autonomy_mode': True,
                 'use_terrain_analysis': True, 'odometry_topic': '/localization/odometry',
                 'registered_cloud_topic': '/localization/registered_cloud_odom',
                 'terrain_map_topic': '/perception/terrain_map_odom',
                 'goal_topic': '/planning/cmu_waypoint', 'path_topic': '/planning/local_path',
                 'output_frame': 'base_link', 'autonomy_speed_m_s': limits['linear_velocity'],
                 'max_speed_m_s': limits['linear_velocity'],
                 'vehicle_length': 2.0 * max(abs(x) for x, _ in footprint),
                 'vehicle_width': 2.0 * max(abs(y) for _, y in footprint),
                 'path_scale': scale, 'min_path_scale': scale, 'path_scale_by_speed': False}]),
        Node(package='cmu_path_follower_generic', executable='path_follower_generic',
             name='path_follower_generic', output='screen', parameters=[{
                 'use_sim_time': sim, 'autonomy_mode': True,
                 'odometry_topic': '/localization/odometry', 'path_topic': '/planning/local_path',
                 'cmd_vel_topic': '/navigation/cmu_cmd', 'output_frame': 'base_link',
                 'autonomy_speed_m_s': limits['linear_velocity'],
                 'max_speed_m_s': limits['linear_velocity'],
                 'max_accel_m_s2': limits['linear_acceleration'],
                 'max_yaw_rate_deg_s': math.degrees(limits['angular_velocity']),
                 'stop_distance_threshold_m': min(0.2, deployment['mission']['xy_tolerance']),
                 'odometry_timeout_s': 0.5}]),
        Node(package='greenhouse_mppi_navigation', executable='terrain_grid',
             name='terrain_grid', parameters=[{'use_sim_time': sim}], output='screen'),
        Node(package='greenhouse_mppi_navigation', executable='command_adapter',
             name='cmu_command_adapter', output='screen', parameters=[{
                 'use_sim_time': sim, 'command_topic': '/navigation/cmu_cmd',
                 'estop_topic': chassis.get('estop_topic', ''),
                 'enable_topic': chassis.get('enable_topic', ''),
                 'require_dynamic_map_tf': deployment['loop_enabled']}]),
        Node(package='greenhouse_mppi_navigation', executable='mission',
             name='tracked_mission', output='screen', parameters=[{
                 'use_sim_time': sim, **deployment['mission'], 'controller_mode': 'cmu'}]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('deployment'), OpaqueFunction(function=setup)])
