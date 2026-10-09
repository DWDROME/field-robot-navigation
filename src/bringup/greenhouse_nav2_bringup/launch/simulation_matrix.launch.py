"""Run the three-site x three-robot simulation matrix from one entry point.

One launch manages exactly one world, one clock and one entity set:

  world    maize_field | orchard | pipeline
  robot    jackal_j100 | husky_a200 | x500
  headless true | false
  seed     maize field generation seed (build-time fixed; other worlds ignore it)
  task     task configuration name
  output   result directory root for batch runs

Ground vehicles (jackal_j100, husky_a200) spawn their Clearpath platform with
the simulation sensor suite and the command gate bridging to the platform
controller. The air vehicle (x500) starts PX4 SITL attached to the same
running world (PX4_GZ_STANDALONE) so no second world or clock is created.

The maize field world is pre-generated at image build time; this launch reads
its fixed path and never downloads resources.
"""
from pathlib import Path
import json
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

WORLDS = {
    'maize_field': {
        'file': '/opt/greenhouse_sim/worlds/maize_field/generated.world',
        'name': 'virtual_maize_field',
    },
    'orchard': {
        'file': None,  # resolved from the clearpath_gz package share
        'name': 'orchard',
    },
    'pipeline': {
        'file': None,
        'name': 'pipeline',
    },
}

ROBOTS = ('jackal_j100', 'husky_a200', 'x500')

# Ground spawn poses per site (x, y, z): maize headland corner, orchard
# staging area, pipeline base-station flat. z clears the local ground.
GROUND_SPAWNS = {
    'maize_field': ('-9.0', '-8.5', '0.5'),
    'orchard': ('0.0', '0.0', '0.3'),
    'pipeline': ('-2.0', '8.0', '0.4'),
}

# Third-person camera poses per site (x, y, z, roll, pitch, yaw): elevated
# corner views framing the site area the robot works in.
CAMERA_POSES = {
    'maize_field': ('-20.0', '-20.0', '22.0', '0.0', '0.66', '0.785'),
    'orchard': ('-30.0', '-30.0', '40.0', '0.0', '0.60', '0.725'),
    'pipeline': ('-150.0', '-180.0', '200.0', '0.0', '0.73', '0.836'),
}
TASK_CAMERA_POSES = {
    'maize_field': ('-14.0', '-14.0', '4.0', '0.0', '0.45', '0.785'),
    'orchard': ('-6.0', '-4.0', '3.0', '0.0', '0.4', '0.59'),
    'pipeline': ('-8.0', '3.0', '5.0', '0.0', '0.5', '0.56'),
}
AIR_CAMERA_POSES = {
    'maize_field': ('-14.0', '-14.0', '6.0', '0.0', '0.30', '0.785'),
    'orchard': ('-12.0', '-12.0', '10.0', '0.0', '0.23', '0.785'),
    'pipeline': ('-15.0', '-8.0', '14.0', '0.0', '0.35', '0.84'),
}


def _world_path(world: str) -> str:
    entry = WORLDS[world]
    if entry['file']:
        return entry['file']
    share = get_package_share_directory('clearpath_gz')
    return str(Path(share) / 'worlds' / f'{world}.sdf')


def _runtime_world(source: str, output: str, site: str) -> str:
    """Add sensor systems omitted by upstream worlds, retaining their geometry."""
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    present = {plugin.get('name') for plugin in world.findall('plugin')}
    for system, library in [('Sensors', 'sensors'), ('Imu', 'imu'),
                            ('NavSat', 'navsat'), ('Contact', 'contact'),
                            ('Magnetometer', 'magnetometer'), ('AirPressure', 'air-pressure')]:
        name = f'gz::sim::systems::{system}'
        if name not in present:
            plugin = ET.SubElement(world, 'plugin',
                {'name': name, 'filename': f'gz-sim-{library}-system'})
            if system == 'Sensors':
                ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    # Custom worlds must supply a magnetic reference and a geodetic origin
    # for the PX4 simulated compass/GPS. These do not change terrain geometry.
    if world.find('magnetic_field') is None:
        ET.SubElement(world, 'magnetic_field').text = '6e-06 2.3e-05 -4.2e-05'
    if world.find('spherical_coordinates') is None:
        coordinates = ET.SubElement(world, 'spherical_coordinates')
        for key, value in [('surface_model', 'EARTH_WGS84'), ('world_frame_orientation', 'ENU'),
                           ('latitude_deg', '47.397971057728974'), ('longitude_deg', '8.546163739800146'),
                           ('elevation', '0'), ('heading_deg', '0')]:
            ET.SubElement(coordinates, key).text = value
    destination = Path(output) / f'{site}-runtime.sdf'
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding='utf-8', xml_declaration=True)
    return str(destination)


def setup(context):
    world = LaunchConfiguration('world').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    if world not in WORLDS:
        raise ValueError(f'world must be one of {sorted(WORLDS)}')
    if robot not in ROBOTS:
        raise ValueError(f'robot must be one of {ROBOTS}')
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    task = LaunchConfiguration('task').perform(context)
    output = LaunchConfiguration('output').perform(context)
    if LaunchConfiguration('seed').perform(context) != '20261006':
        raise ValueError('This image contains maize seed 20261006; rebuild to change the world seed')
    if task != 'site_default':
        raise ValueError('Only the site_default task is configured')

    share = Path(get_package_share_directory('greenhouse_nav2_bringup'))
    world_file = _runtime_world(_world_path(world), output, world)
    world_name = WORLDS[world]['name']

    actions = [
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               str(share / 'config' / 'simulation' / 'dds-loopback.xml')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            f'{share}/worlds:/opt/greenhouse_sim/worlds/maize_field:'
            '/opt/maize_field/share/virtual_maize_field/models:'
            '/opt/ros/jazzy/share/clearpath_gz/meshes:'
            '/opt/px4/Tools/simulation/gz/models:'
            '/opt/ros/jazzy/share'),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
        # Pin gz-transport to loopback: the container has many docker bridge
        # interfaces, and auto-selection can bind discovery to an interface
        # where service calls time out.
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        # One entry owns the world, entities and /clock: the gz server runs
        # server-only and the ros_gz bridge publishes /clock once.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': f'-r -s {"--headless-rendering " if headless else ""}{world_file}'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='simulation_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}], output='screen'),
    ]

    if robot in ('jackal_j100', 'husky_a200'):
        x, y, z = GROUND_SPAWNS[world]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'simulation_ground_vehicle.launch.py')),
            launch_arguments={
                'robot': robot,
                'world_name': world_name,
                'world': world,
                'spawn_x': x, 'spawn_y': y, 'spawn_z': z,
                'task': task,
                'output': output,
                'navigation': LaunchConfiguration('navigation').perform(context),
                'controller': LaunchConfiguration('controller').perform(context),
            }.items()))
    else:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'simulation_x500.launch.py')),
            launch_arguments={
                'world_name': world_name,
                'world': world,
                'task': task,
                'output': output,
            }.items()))

    # Screenshot camera: spawned on request so batch runs stay lean; the
    # capture pipeline subscribes to /sim_camera/image through the bridge.
    if LaunchConfiguration('camera').perform(context) == 'true':
        task_cameras = AIR_CAMERA_POSES if robot == 'x500' else TASK_CAMERA_POSES
        camera_pose = (CAMERA_POSES if LaunchConfiguration('camera_view').perform(context) == 'panorama'
                       else task_cameras)[world]
        (Path(output) / 'camera.json').write_text(json.dumps({
            'world': world, 'robot': robot,
            'view': LaunchConfiguration('camera_view').perform(context),
            'pose': [float(value) for value in camera_pose],
            'resolution': '1920x1080', 'topic': '/sim_camera/image'}, indent=2))
        actions.append(Node(
            package='ros_gz_sim', executable='create', name='create_simulation_camera',
            arguments=['-name', 'sim_camera', '-world', world_name,
                       '-file', str(share / 'worlds' / 'simulation_camera.sdf'),
                       '-x', camera_pose[0], '-y', camera_pose[1], '-z', camera_pose[2],
                       '-R', camera_pose[3], '-P', camera_pose[4], '-Y', camera_pose[5]],
            output='screen'))
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='sim_camera_bridge',
            arguments=['/sim_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maize_field',
                              choices=sorted(WORLDS), description='Simulation site'),
        DeclareLaunchArgument('robot', default_value='husky_a200',
                              choices=list(ROBOTS), description='Simulated robot'),
        DeclareLaunchArgument('headless', default_value='true',
                              choices=['true', 'false'], description='Headless rendering'),
        DeclareLaunchArgument('seed', default_value='20261006',
                              description='Maize field generation seed (build-time fixed)'),
        DeclareLaunchArgument('task', default_value='site_default',
                              description='Task configuration name'),
        DeclareLaunchArgument('output', default_value='/tmp/simulation',
                              description='Result output root'),
        DeclareLaunchArgument('camera', default_value='false',
                              choices=['true', 'false'],
                              description='Spawn the screenshot camera'),
        DeclareLaunchArgument('camera_view', default_value='task', choices=['task', 'panorama']),
        DeclareLaunchArgument('navigation', default_value='true', choices=['true', 'false'],
                              description='Start the project ground navigation chain'),
        DeclareLaunchArgument('controller', default_value='mppi', choices=['mppi', 'cmu']),
        OpaqueFunction(function=setup),
    ])
