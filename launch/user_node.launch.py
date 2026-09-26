"""Launch a user-owned overlay lease with transport-aware status output.

The background service owns the Bluetooth adapter. This helper activates one
YAML overlay, keeps its lease alive, and selects detailed GATT, advertisement,
or Mesh reporting from the overlay transport.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_path", description="Path to the user BLE overlay YAML file."),
        DeclareLaunchArgument("service_wait_timeout_sec", default_value="30.0", description="Time to wait for the Bluetooth overlay service to become available."),
        DeclareLaunchArgument("service_call_timeout_sec", default_value="45.0", description="Response timeout, including a cold Mesh/LE radio handoff."),
        DeclareLaunchArgument("deactivate_service_wait_timeout_sec", default_value="2.0", description="Time to wait for the BLE overlay service to become unavailable after calling the deactivate service."),
        DeclareLaunchArgument("sentinel_topic_suffix", default_value="overlay_keepalive", description="Suffix for the sentinel topic to publish to while the overlay is active."),
        DeclareLaunchArgument("sentinel_publish_period_sec", default_value="1.0", description="Period in seconds to publish sentinel messages while the overlay is active."),
        DeclareLaunchArgument("min_sentinel_publish_period_sec", default_value="0.2", description="Minimum period in seconds to publish sentinel messages while the overlay is active. If the service call timeout is shorter than the sentinel publish period, this should be set to a value shorter than the service call timeout to ensure that sentinel messages are published at least as frequently as the service call timeout."),
        DeclareLaunchArgument("print_source", default_value="log", description="Service text source used for GATT/generic overlays: status or log."),
        DeclareLaunchArgument("transport_report_period_sec", default_value="2.0", description="Period for advertisement and Mesh overlay detail reports."),
        Node(
            package="mrs_uav_bluetooth",
            executable="user_node",
            name="mrs_uav_bluetooth_user",
            output="screen",
            parameters=[
                {
                    "config_path": LaunchConfiguration("config_path"),
                    "service_wait_timeout_sec": ParameterValue(LaunchConfiguration("service_wait_timeout_sec"), value_type=float),
                    "service_call_timeout_sec": ParameterValue(LaunchConfiguration("service_call_timeout_sec"), value_type=float),
                    "deactivate_service_wait_timeout_sec": ParameterValue(LaunchConfiguration("deactivate_service_wait_timeout_sec"), value_type=float),
                    "sentinel_topic_suffix": LaunchConfiguration("sentinel_topic_suffix"),
                    "sentinel_publish_period_sec": ParameterValue(LaunchConfiguration("sentinel_publish_period_sec"), value_type=float),
                    "min_sentinel_publish_period_sec": ParameterValue(LaunchConfiguration("min_sentinel_publish_period_sec"), value_type=float),
                    "print_source": LaunchConfiguration("print_source"),
                    "transport_report_period_sec": ParameterValue(
                        LaunchConfiguration("transport_report_period_sec"),
                        value_type=float,
                    ),
                },
            ],
        ),
    ])