from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("cmu_local_planner")
    return LaunchDescription(
        [
            Node(
                package="cmu_local_planner",
                executable="local_planner_node",
                name="local_planner",
                output="screen",
                parameters=[
                    {
                        "path_folder": f"{package_share}/paths",
                        "odometry_topic": "/localization/odometry",
                        "registered_cloud_topic": "/localization/registered_cloud",
                        "terrain_map_topic": "/perception/terrain_map",
                        "path_topic": "/planning/local_path",
                        "free_paths_topic": "/planning/free_paths",
                        "output_frame": "base_link",
                        "use_terrain_analysis": True,
                    }
                ],
            )
        ]
    )
