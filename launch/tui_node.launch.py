from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("adapter_alias", default_value="", description="Optional local adapter alias."),
        DeclareLaunchArgument("scan_mode", default_value="le", description="BlueZ scan transport mode."),
        DeclareLaunchArgument("uav_name_pattern", default_value="^uav[0-9]{1,2}$", description="Regex used to classify UAV names."),
        DeclareLaunchArgument("refresh_period_sec", default_value="1.0", description="Device cache refresh period."),
        DeclareLaunchArgument("render_period_sec", default_value="0.1", description="TUI render and input tick period."),
        DeclareLaunchArgument("topic_count_refresh_sec", default_value="5.0", description="How often to refresh exported-topic counts."),
        DeclareLaunchArgument("hide_non_uav", default_value="false", description="Show only devices matching the UAV name pattern."),
        Node(
            package="mrs_uav_bluetooth",
            executable="tui_node",
            name="mrs_uav_bluetooth_tui",
            output="log",
            parameters=[
                {
                    "adapter_alias": LaunchConfiguration("adapter_alias"),
                    "scan_mode": LaunchConfiguration("scan_mode"),
                    "uav_name_pattern": LaunchConfiguration("uav_name_pattern"),
                    "refresh_period_sec": LaunchConfiguration("refresh_period_sec"),
                    "render_period_sec": LaunchConfiguration("render_period_sec"),
                    "topic_count_refresh_sec": LaunchConfiguration("topic_count_refresh_sec"),
                    "hide_non_uav": LaunchConfiguration("hide_non_uav"),
                },
            ],
        ),
    ])