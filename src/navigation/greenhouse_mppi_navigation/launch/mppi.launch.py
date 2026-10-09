from pathlib import Path
import json
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from greenhouse_mppi_navigation.deployment import load


def setup(context):
    deployment=load(LaunchConfiguration('deployment').perform(context),resources=True)
    if deployment['controllers']!=['mppi']:
        raise ValueError('mppi.launch.py requires controllers: [mppi]')
    sim=deployment['mode']=='simulation'
    share=Path(get_package_share_directory('greenhouse_mppi_navigation'))
    params=yaml.safe_load((share/'config/mppi.yaml').read_text())
    limits=deployment['chassis']['limits']
    controller=params['controller_server']['ros__parameters']
    mission=deployment['mission']
    controller['goal_checker']['xy_goal_tolerance']=mission['xy_tolerance']
    controller['goal_checker']['yaw_goal_tolerance']=mission['yaw_tolerance']
    for name,source in [('vx_max','linear_velocity'),('wz_max','angular_velocity'),('ax_max','linear_acceleration'),('az_max','angular_acceleration')]:
        controller['FollowPath'][name]=float(limits[source])
    controller['FollowPath']['vx_min']=-float(limits['linear_velocity'])/3
    controller['FollowPath']['ax_min']=-float(limits['linear_acceleration'])
    costmap=params['local_costmap']['local_costmap']['ros__parameters']
    costmap['footprint']=json.dumps(deployment['chassis']['footprint'])
    for section in (controller,costmap): section['use_sim_time']=sim
    # RewrittenYaml provides the nested costmap namespace without temporary hand-written files.
    from nav2_common.launch import RewrittenYaml
    source=RewrittenYaml(source_file=str(share/'config/mppi.yaml'),root_key='',param_rewrites={
        'use_sim_time':str(sim).lower(),'footprint':costmap['footprint'],
        'xy_goal_tolerance':str(mission['xy_tolerance']),'yaw_goal_tolerance':str(mission['yaw_tolerance']),
        'FollowPath.vx_max':str(controller['FollowPath']['vx_max']),
        'FollowPath.vx_min':str(controller['FollowPath']['vx_min']),
        'FollowPath.wz_max':str(controller['FollowPath']['wz_max']),
        'FollowPath.ax_max':str(controller['FollowPath']['ax_max']),
        'FollowPath.ax_min':str(controller['FollowPath']['ax_min']),
        'FollowPath.az_max':str(controller['FollowPath']['az_max'])},convert_types=True)
    return [
        Node(package='nav2_controller',executable='controller_server',name='controller_server',output='screen',
             parameters=[source,controller],remappings=[('cmd_vel','/navigation/mppi_cmd')]),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='mppi_lifecycle_manager',output='screen',
             parameters=[{'use_sim_time':sim,'autostart':True,'node_names':['controller_server'],'bond_timeout':4.0}]),
        Node(package='greenhouse_mppi_navigation',executable='terrain_grid',name='terrain_grid',parameters=[{'use_sim_time':sim}],output='screen'),
        Node(package='greenhouse_mppi_navigation',executable='command_adapter',name='mppi_command_adapter',parameters=[{
             'use_sim_time':sim,'estop_topic':deployment['chassis'].get('estop_topic',''),
             'require_dynamic_map_tf':deployment['loop_enabled'],
             'enable_topic':deployment['chassis'].get('enable_topic','')}],output='screen'),
        Node(package='greenhouse_mppi_navigation',executable='mission',name='tracked_mission',parameters=[{'use_sim_time':sim,**mission}],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('deployment'),OpaqueFunction(function=setup)])
