import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_path", description="Path to the user BLE overlay YAML file."),
        DeclareLaunchArgument("hold_seconds", default_value="3.0", description="Overlay lease refresh period in seconds."),
        Node(
            package="mrs_uav_bluetooth",
            executable="user_node",
            name="mrs_uav_bluetooth_user",
            output="screen",
            parameters=[
                {
                    "config_path": LaunchConfiguration("config_path"),
                    "hold_seconds": LaunchConfiguration("hold_seconds"),
                },
            ],
        ),
    ])