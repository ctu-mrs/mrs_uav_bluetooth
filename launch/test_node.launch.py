from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="odometry",
            description="Testing mode: odometry or advertisement.",
        ),
        DeclareLaunchArgument(
            "rate_hz",
            default_value="10.0",
            description="Publish rate in Hz.",
        ),
        DeclareLaunchArgument(
            "odometry_topic",
            default_value="",
            description="Override for the odometry topic. Empty uses /{hostname}/mavros/local_position/odom.",
        ),
        DeclareLaunchArgument(
            "advertisement_topic",
            default_value="",
            description="Override for the advertisement payload topic. Empty uses /{hostname}/ble/adv_local_extra.",
        ),
        DeclareLaunchArgument(
            "advertisement_observe_topic",
            default_value="",
            description="Override for the advertisement-device monitor topic. Empty uses /{hostname}/ble/advertisement.",
        ),
        DeclareLaunchArgument(
            "advertisement_observe_topic_compat",
            default_value="",
            description="Compatibility advertisement-device monitor topic. Empty uses /{hostname}/ble/advertisements.",
        ),
        DeclareLaunchArgument(
            "advertisement_log_period_sec",
            default_value="2.0",
            description="Period for logging observed advertisement user data devices in advertisement mode.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Odometry header frame_id.",
        ),
        DeclareLaunchArgument(
            "child_frame_id",
            default_value="base_link",
            description="Odometry child_frame_id.",
        ),
        Node(
            package="mrs_uav_bluetooth",
            executable="test_node",
            name="mrs_uav_bluetooth_test",
            output="screen",
            parameters=[
                {
                    "mode": LaunchConfiguration("mode"),
                    "rate_hz": LaunchConfiguration("rate_hz"),
                    "odometry_topic": LaunchConfiguration("odometry_topic"),
                    "advertisement_topic": LaunchConfiguration("advertisement_topic"),
                    "advertisement_observe_topic": LaunchConfiguration("advertisement_observe_topic"),
                    "advertisement_observe_topic_compat": LaunchConfiguration("advertisement_observe_topic_compat"),
                    "advertisement_log_period_sec": LaunchConfiguration("advertisement_log_period_sec"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "child_frame_id": LaunchConfiguration("child_frame_id"),
                },
            ],
        ),
    ])