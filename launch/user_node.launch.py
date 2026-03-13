import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_path", description="Path to the user BLE overlay YAML file."),
        DeclareLaunchArgument("service_wait_timeout_sec", default_value="10.0", description="Time to wait for the BLE overlay service to become available."),
        DeclareLaunchArgument("service_call_timeout_sec", default_value="5.0", description="Time to wait for a response from the BLE overlay service after calling it."),
        DeclareLaunchArgument("deactivate_service_wait_timeout_sec", default_value="2.0", description="Time to wait for the BLE overlay service to become unavailable after calling the deactivate service."),
        DeclareLaunchArgument("sentinel_topic_suffix", default_value="overlay_keepalive", description="Suffix for the sentinel topic to publish to while the overlay is active."),
        DeclareLaunchArgument("sentinel_publish_period_sec", default_value="1.0", description="Period in seconds to publish sentinel messages while the overlay is active."),
        DeclareLaunchArgument("min_sentinel_publish_period_sec", default_value="0.2", description="Minimum period in seconds to publish sentinel messages while the overlay is active. If the service call timeout is shorter than the sentinel publish period, this should be set to a value shorter than the service call timeout to ensure that sentinel messages are published at least as frequently as the service call timeout."),
        DeclareLaunchArgument("print_source", default_value="status", description="Service text source topic suffix to print: status or log."),
        Node(
            package="mrs_uav_bluetooth",
            executable="user_node",
            name="mrs_uav_bluetooth_user",
            output="screen",
            parameters=[
                {
                    "config_path": LaunchConfiguration("config_path"),
                    "service_wait_timeout_sec": LaunchConfiguration("service_wait_timeout_sec"),
                    "service_call_timeout_sec": LaunchConfiguration("service_call_timeout_sec"),
                    "deactivate_service_wait_timeout_sec": LaunchConfiguration("deactivate_service_wait_timeout_sec"),
                    "sentinel_topic_suffix": LaunchConfiguration("sentinel_topic_suffix"),
                    "sentinel_publish_period_sec": LaunchConfiguration("sentinel_publish_period_sec"),
                    "min_sentinel_publish_period_sec": LaunchConfiguration("min_sentinel_publish_period_sec"),
                    "print_source": LaunchConfiguration("print_source"),
                },
            ],
        ),
    ])