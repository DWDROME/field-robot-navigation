from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([Node(
        package="fast_lio_super_ros2", executable="fast_lio_super_backend",
        name="fast_lio_super_backend", output="screen",
        parameters=[get_package_share_directory("fast_lio_super_ros2") + "/config/backend.yaml"],
        remappings=[("odometry", "/localization/odometry"),
                    ("registered_cloud", "/localization/registered_cloud"),
                    ("global_odometry", "/localization/global_odometry"),
                    ("global_pose", "/localization/global_pose"),
                    ("corrected_path", "/localization/corrected_path"),
                    ("corrected_map", "/localization/corrected_map"),
                    ("diagnostics", "/diagnostics")])])
