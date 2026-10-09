#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument(
            "odometry_topic",
            default_value="localization/odometry",
        ),
        DeclareLaunchArgument(
            "registered_cloud_topic",
            default_value="localization/registered_cloud",
        ),
        DeclareLaunchArgument(
            "terrain_map_topic",
            default_value="perception/terrain_map",
        ),
        DeclareLaunchArgument("output_frame", default_value="map"),
    ]
    terrain_node = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrainAnalysis",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "registered_cloud_topic": LaunchConfiguration(
                    "registered_cloud_topic"
                ),
                "terrain_map_topic": LaunchConfiguration("terrain_map_topic"),
                "output_frame": LaunchConfiguration("output_frame"),
                "scanVoxelSize": 0.05,
                "decayTime": 2.0,
                "noDecayDis": 4.0,
                "clearingDis": 8.0,
                "useSorting": True,
                "quantileZ": 0.25,
                "considerDrop": False,
                "limitGroundLift": False,
                "maxGroundLift": 0.15,
                "clearDyObs": False,
                "minDyObsDis": 0.3,
                "minDyObsAngle": 0.0,
                "minDyObsRelZ": -0.5,
                "absDyObsRelZThre": 0.2,
                "minDyObsVFOV": -16.0,
                "maxDyObsVFOV": 16.0,
                "minDyObsPointNum": 1,
                "noDataObstacle": False,
                "noDataBlockSkipNum": 0,
                "minBlockPointNum": 10,
                "vehicleHeight": 1.5,
                "voxelPointUpdateThre": 100,
                "voxelTimeUpdateThre": 2.0,
                "minRelZ": -1.5,
                "maxRelZ": 0.2,
                "disRatioZ": 0.2,
            }
        ],
    )
    return LaunchDescription(
        [
            *arguments,
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            terrain_node,
        ]
    )
