"""Start PX4 SITL for the x500_depth air vehicle attached to the running world.

PX4_GZ_STANDALONE=1 makes PX4 wait for and attach to the world managed by
simulation_matrix.launch.py instead of starting its own gz server, so the
matrix keeps exactly one world, one clock and one entity set.

Spawn and waypoint settings come from greenhouse_sim_air/config/x500_sites.yaml.
The image build checks the route against the delivered world geometry.
"""
from pathlib import Path
import hashlib
import json
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    world_name = LaunchConfiguration('world_name').perform(context)
    world = LaunchConfiguration('world').perform(context)
    config = Path(get_package_share_directory('greenhouse_sim_air')) / 'config/x500_sites.yaml'
    sites = yaml.safe_load(config.read_text())
    geometry = json.loads(Path('/opt/greenhouse_sim/geometry.json').read_text())
    if (geometry['air_config_sha256'] != hashlib.sha256(config.read_bytes()).hexdigest()
            or geometry['sites'][world]['air_clearance']['result'] != 'pass'):
        raise ValueError('X500 route geometry check is absent, failed or outdated; rebuild the simulation image')
    spawn = [str(value) for value in sites[world]['spawn']]

    px4_root = Path('/opt/px4')
    rootfs = Path(LaunchConfiguration('output').perform(context)) / 'px4-rootfs'
    rootfs.mkdir(parents=True, exist_ok=True)
    etc = rootfs / 'etc'
    if not etc.exists():
        etc.symlink_to(px4_root / 'build' / 'px4_sitl_default' / 'etc', target_is_directory=True)
    env = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_GZ_WORLD': world_name,
        'PX4_GZ_MODEL_POSE': ','.join(spawn),
        'PX4_SIM_MODEL': 'gz_x500_depth',
        'GZ_SIM_RESOURCE_PATH': (
            f'{px4_root}/Tools/simulation/gz/models:'
            f'{px4_root}/Tools/simulation/gz/worlds:'
            '/opt/greenhouse_sim/worlds/maize_field:'
            '/opt/maize_field/share/virtual_maize_field/models:'
            '/opt/ros/jazzy/share/clearpath_gz/meshes:'
            '/opt/ros/jazzy/share'),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': (
            '/opt/ros/jazzy/lib:'
            f'{px4_root}/build/px4_sitl_default/src/modules/simulation/gz_plugins'),
        'LD_LIBRARY_PATH': (
            f'{px4_root}/build/px4_sitl_default/src/modules/simulation/gz_plugins:'
            '/opt/xrce-agent/lib:'
            '/opt/ros/jazzy/opt/gz_utils_vendor/lib:'
            '/opt/ros/jazzy/opt/gz_transport_vendor/lib:'
            '/opt/ros/jazzy/opt/gz_msgs_vendor/lib:'
            '/opt/ros/jazzy/opt/gz_common_vendor/lib:'
            '/opt/ros/jazzy/opt/gz_math_vendor/lib:'
            '/opt/ros/jazzy/opt/gz_plugin_vendor/lib:'
            '/opt/ros/jazzy/opt/sdformat_vendor/lib:'
            '/opt/ros/jazzy/lib'),
    }
    actions = [SetEnvironmentVariable(name, value) for name, value in env.items()]
    # PX4 resolves etc/init.d-posix relative to its working directory, and
    # writes logs/dataman beside it; run from its build rootfs dir.
    actions.append(ExecuteProcess(
        cmd=[f'{px4_root}/build/px4_sitl_default/bin/px4', '-i', '0'],
        cwd=str(rootfs),
        name='px4_sitl', output='screen',
        additional_env=env))
    # ament_python installs its console entry points under lib/greenhouse_sim_air.
    # PYTHONPATH and AMENT_PREFIX_PATH come from the image environment; the agent links its
    # own libs first, then the system FastDDS stack for foonathan_memory.
    actions.append(Node(
        package='greenhouse_sim_air', executable='micro_xrce_agent',
        name='micro_xrce_agent', output='screen',
        additional_env={'LD_LIBRARY_PATH':
                        '/opt/xrce-agent/lib:/usr/lib/x86_64-linux-gnu'}))
    actions.append(Node(package='greenhouse_sim_air', executable='simulation_gcs',
                        name='simulation_gcs', output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', description='gz world name to attach'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('task', default_value='site_default'),
        DeclareLaunchArgument('output', default_value='/tmp/simulation'),
        OpaqueFunction(function=setup),
    ])
