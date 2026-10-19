"""Route the FAST-LIO Super/FAR deployment and selected controller through bringup."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=get_package_share_directory('greenhouse_mppi_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('deployment'),DeclareLaunchArgument('map_backend',default_value='octvox'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(share+'/launch/closed_loop.launch.py'),
            launch_arguments={'deployment':LaunchConfiguration('deployment'),'map_backend':LaunchConfiguration('map_backend')}.items())])
