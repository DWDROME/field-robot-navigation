from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("usart_port_name", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("serial_baud_rate", default_value="115200"),
            DeclareLaunchArgument("odom_frame_id", default_value="odom_combined"),
            DeclareLaunchArgument("robot_frame_id", default_value="base_footprint"),
            DeclareLaunchArgument("gyro_frame_id", default_value="imu_link"),
            Node(
                package="dlrobot_robot",
                executable="dlrobot_robot_node",
                name="dlrobot_robot",
                output="screen",
                parameters=[
                    {
                        "usart_port_name": LaunchConfiguration("usart_port_name"),
                        "serial_baud_rate": ParameterValue(
                            LaunchConfiguration("serial_baud_rate"),
                            value_type=int,
                        ),
                        "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                        "robot_frame_id": LaunchConfiguration("robot_frame_id"),
                        "gyro_frame_id": LaunchConfiguration("gyro_frame_id"),
                    }
                ],
            ),
        ]
    )
