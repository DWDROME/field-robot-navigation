"""Run a Gazebo world with clock and sensor bridges; optionally spawn the A200."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('greenhouse_nav2_bringup')
    world = PathJoinSubstitution([share, 'worlds', LaunchConfiguration('world')])
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('world_name', default_value='greenhouse_empty'),
        DeclareLaunchArgument('vehicle', default_value='false'),
        DeclareLaunchArgument('setup_path', default_value='/opt/greenhouse_sim/robot'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
            launch_arguments={'gz_args': ['-r -s --headless-rendering ', world]}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='simulation_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'config_file': PathJoinSubstitution([
                 share, 'config', 'simulation', 'sensor-bridge.yaml'])}], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'simulation_vehicle.launch.py'])),
            condition=IfCondition(LaunchConfiguration('vehicle')),
            launch_arguments={
                'world': LaunchConfiguration('world_name'),
                'setup_path': LaunchConfiguration('setup_path'),
                'command_gate_enabled': 'true',
                'bridge_feedback_topic': '/simulation/cmd_feedback',
            }.items()),
    ])
