from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="cmu_path_follower_generic",
                executable="path_follower_generic",
                name="path_follower_generic",
                output="screen",
                parameters=[
                    {
                        "autonomy_mode": False,
                        "odometry_topic": "/localization/odometry",
                        "path_topic": "/planning/local_path",
                        "cmd_vel_topic": "/cmd_vel/nav",
                        "output_frame": "base_link",
                    }
                ],
            )
        ]
    )
