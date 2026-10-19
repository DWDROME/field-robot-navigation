#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    setup_path = LaunchConfiguration("setup_path")
    world = LaunchConfiguration("world")
    command_gate_enabled = LaunchConfiguration("command_gate_enabled")
    command_gate_config = LaunchConfiguration("command_gate_config")
    bridge_feedback_topic = LaunchConfiguration("bridge_feedback_topic")

    description_launch = PathJoinSubstitution(
        [
            FindPackageShare("clearpath_platform_description"),
            "launch",
            "description.launch.py",
        ]
    )
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch),
        launch_arguments={
            "setup_path": setup_path,
            "use_sim_time": "true",
            "namespace": "/",
            "use_fake_hardware": "false",
            "use_manipulation_controllers": "false",
            "use_platform_controllers": "true",
        }.items(),
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        arguments=[
            "--controller-manager-timeout",
            "60",
            "joint_state_broadcaster",
        ],
        additional_env={"ROS_SUPER_CLIENT": "True"},
        output="screen",
    )
    velocity_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_platform_velocity_controller",
        arguments=[
            "--controller-manager-timeout",
            "60",
            "platform_velocity_controller",
        ],
        additional_env={"ROS_SUPER_CLIENT": "True"},
        output="screen",
    )

    command_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="cmd_vel_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist",
            (
                "/model/robot/cmd_vel"
                "@geometry_msgs/msg/TwistStamped]gz.msgs.Twist"
            ),
        ],
        remappings=[
            ("/cmd_vel", bridge_feedback_topic),
            ("/model/robot/cmd_vel", "platform/cmd_vel"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    command_gate = Node(
        package="greenhouse_cmd_gate",
        executable="greenhouse_cmd_gate",
        name="greenhouse_cmd_gate",
        condition=IfCondition(command_gate_enabled),
        parameters=[
            command_gate_config,
            {"use_sim_time": True},
        ],
        output="screen",
    )
    command_sink = Node(
        package="greenhouse_cmd_gate",
        executable="greenhouse_cmd_sink",
        name="greenhouse_sim_cmd_sink",
        condition=IfCondition(command_gate_enabled),
        parameters=[
            command_gate_config,
            {"use_sim_time": True},
        ],
        output="screen",
    )
    odom_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odom_base_tf_bridge",
        arguments=[
            "/model/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        remappings=[
            ("/model/robot/tf", "tf"),
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    create_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="create_clearpath_a200",
        arguments=[
            "-name",
            "robot",
            "-world",
            world,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.15",
            "-Y",
            "0.0",
            "-topic",
            "robot_description",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "setup_path",
                default_value="/opt/greenhouse_sim/robot",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="greenhouse_empty",
            ),
            DeclareLaunchArgument(
                "command_gate_enabled",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "command_gate_config",
                default_value=PathJoinSubstitution([
                    FindPackageShare("greenhouse_nav2_bringup"),
                    "config", "simulation", "command-gate.yaml",
                ]),
            ),
            DeclareLaunchArgument(
                "bridge_feedback_topic",
                default_value="/cmd_vel",
            ),
            robot_description,
            joint_state_spawner,
            velocity_spawner,
            command_bridge,
            command_gate,
            command_sink,
            odom_tf_bridge,
            create_robot,
        ]
    )
