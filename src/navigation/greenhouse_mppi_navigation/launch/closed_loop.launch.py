"""FAST-LIO Super, terrain and FAR with one deployment-selected controller."""
import math
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from greenhouse_mppi_navigation.deployment import load


def imu_from_body(sensor):
    # T_imu_body = T_imu_lidar * inverse(T_base_lidar); all transforms are explicit deployment inputs.
    x,y,z,roll,pitch,yaw=sensor['base_from_lidar_xyzrpy']
    cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
    rb=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
        [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    ri=[sensor['imu_from_lidar_R'][i:i+3] for i in range(0,9,3)]
    rotation=[[sum(ri[i][k]*rb[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    translation=[sensor['imu_from_lidar_T'][i]-sum(rotation[i][k]*[x,y,z][k] for k in range(3)) for i in range(3)]
    return translation,[v for row in rotation for v in row]


def setup(context):
    deployment_path=LaunchConfiguration('deployment').perform(context)
    d=load(deployment_path,resources=True); sim=d['mode']=='simulation'; sensor=d['sensor']; chassis=d['chassis']
    super_share=Path(get_package_share_directory('fast_lio_super_ros2'))
    nav_share=Path(get_package_share_directory('greenhouse_mppi_navigation'))
    body_t,body_r=imu_from_body(sensor)
    backend=LaunchConfiguration('map_backend').perform(context)
    if backend not in {'ikdtree','octvox'}: raise ValueError('map_backend must be ikdtree or octvox')
    fast={'use_sim_time':sim,'common.imu_from_body_T':body_t,'common.imu_from_body_R':body_r,
        'common.lid_topic':'/sensors/lidar/raw','common.imu_topic':'/sensors/imu/data',
        'mapping.extrinsic_T':sensor['imu_from_lidar_T'],'mapping.extrinsic_R':sensor['imu_from_lidar_R'],
        'mapping.extrinsic_est_en':False,'publish.path_en':False,
        # Preserve observed terrain density in simulation. The map backend
        # still downsamples its own map; terrain must receive actual returns.
        'publish.dense_publish_en':sim,
        'publish.registered_cloud_topic':'/localization/registered_cloud_odom',
        'slam_core.map_backend':backend,'slam_core.local_crop_radius':100.0}
    if not sim: fast.update({'preprocess.lidar_type':1,'preprocess.scan_line':4,'preprocess.blind':.5})
    limits=chassis['limits']
    gate={'use_sim_time':sim,'source_topic':'/cmd_vel/nav','output_topic':'/cmd_vel','output_frame':'base_link',
        'qualification_scope':'simulation_fixture' if sim else 'configured_hardware_pending_validation',
        'publish_rate_hz':20.0,'max_linear_velocity':limits['linear_velocity'],
        'max_angular_velocity':limits['angular_velocity'],'max_linear_acceleration':limits['linear_acceleration'],
        'max_angular_acceleration':limits['angular_acceleration'],'watchdog_timeout_sec':.3,
        'max_command_age_sec':.2,'future_tolerance_sec':.05,'auto_recover_after_watchdog':True}
    actions=[LogInfo(msg=f"closed-loop runtime_ready=true hardware_qualified=false mode={d['mode']}"),
        Node(package='fast_lio',executable='fastlio_mapping',name='laser_mapping',output='screen',
             parameters=[str(nav_share/'config/fastlio.yaml'),fast]),
        Node(package='greenhouse_cloud_normalizer',executable='greenhouse_cloud_normalizer',name='slam_cloud_to_map',output='screen',
             parameters=[{'use_sim_time':sim,'input_topic':'/localization/registered_cloud_odom',
                          'output_topic':'/localization/registered_cloud','target_frame':'map'}]),
        Node(package='greenhouse_cmd_gate',executable='greenhouse_cmd_gate',name='greenhouse_cmd_gate',parameters=[gate],output='screen')]
    if d['loop_enabled']:
        actions.append(Node(package='fast_lio_super_ros2',executable='fast_lio_super_backend',name='fast_lio_super_backend',output='screen',
            parameters=[str(super_share/'config/backend.yaml'),{'use_sim_time':sim}],remappings=[
                ('odometry','/localization/odometry'),('registered_cloud','/localization/registered_cloud_odom'),
                ('global_odometry','/localization/global_odometry'),('global_pose','/localization/global_pose'),
                ('corrected_path','/localization/corrected_path'),('corrected_map','/localization/corrected_map'),('diagnostics','/diagnostics')]))
    else:
        actions.extend([
            Node(package='tf2_ros',executable='static_transform_publisher',name='map_odom_identity',
                 arguments=['--frame-id','map','--child-frame-id','odom'],parameters=[{'use_sim_time':sim}]),
            Node(package='greenhouse_mppi_navigation',executable='identity_odometry',name='identity_global_odometry',parameters=[{'use_sim_time':sim}])])
    xyzrpy=sensor['base_from_lidar_xyzrpy']
    static_args=['--frame-id','base_link','--child-frame-id','livox_frame']
    for flag,value in zip(['--x','--y','--z','--roll','--pitch','--yaw'],xyzrpy): static_args.extend([flag,str(value)])
    actions.append(Node(package='tf2_ros',executable='static_transform_publisher',name='static_sensor_tf',arguments=static_args,parameters=[{'use_sim_time':sim}]))
    actions.extend([
        Node(package='terrain_analysis',executable='terrainAnalysis',name='terrainAnalysis',output='screen',parameters=[{
            'use_sim_time':sim,'odometry_topic':'/localization/global_odometry','registered_cloud_topic':'/localization/registered_cloud',
            'terrain_map_topic':'/perception/terrain_map','output_frame':'map','scanVoxelSize':.1,'decayTime':1.0,
            'useSorting':True,'quantileZ':.25,'considerDrop':True,'minBlockPointNum':3,'vehicleHeight':1.5}]),
        Node(package='terrain_analysis_ext',executable='terrainAnalysisExt',name='terrain_analysis_ext',output='screen',
            parameters=[{'use_sim_time':sim}],remappings=[('/state_estimation','/localization/global_odometry'),
                ('/registered_scan','/localization/registered_cloud'),('/terrain_map','/perception/terrain_map'),('/terrain_map_ext','/perception/terrain_map_ext')]),
        Node(package='far_planner',executable='far_planner',name='far_planner',output='screen',parameters=[
            str(Path(get_package_share_directory('far_planner'))/'config/default.yaml'),{'use_sim_time':sim,
            'world_frame':'map','sensor_range':8.0,'terrain_range':7.5,'map_handler/map_grid_max_length':20.0,
            'map_handler/map_grad_max_height':4.0,'is_opencv_visual':False,'g_planner/converge_distance':.3}],
            remappings=[('/odom_world','/localization/global_odometry'),('/terrain_cloud','/perception/terrain_map_ext'),
                ('/scan_cloud','/localization/registered_cloud'),('/goal_point','/planning/global_goal'),
                ('/terrain_local_cloud','/perception/terrain_map'),('/way_point','/planning/way_point')]),
        Node(package='graph_decoder',executable='graph_decoder',name='graph_decoder',parameters=[{'use_sim_time':sim}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share/'launch'/f"{d['controllers'][0]}.launch.py")),
            launch_arguments={'deployment':deployment_path}.items())])
    if not sim:
        actions.extend([
            Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',name='livox_lidar_publisher',output='screen',
                parameters=[{'xfer_format':1,'multi_topic':0,'data_src':0,'publish_freq':10.0,'output_data_type':0,
                    'frame_id':'livox_frame','user_config_path':sensor['driver_config']}],
                remappings=[('/livox/lidar','/sensors/lidar/raw'),('/livox/imu','/sensors/imu/data')]),
            Node(package='dlrobot_robot',executable='dlrobot_robot_node',name='dlrobot_robot',output='screen',parameters=[{
                'usart_port_name':chassis['device'],'serial_baud_rate':chassis['baud'],
                'odom_frame_id':'chassis_odom','robot_frame_id':'base_link','gyro_frame_id':'chassis_imu'}],
                remappings=[('odom','/chassis/odometry'),('imu','/chassis/imu')])])
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('deployment'),DeclareLaunchArgument('map_backend',default_value='octvox'),OpaqueFunction(function=setup)])
