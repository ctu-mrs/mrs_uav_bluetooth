"""Launch the optional synthetic odometry source without a config file.

Every node default is exposed as an optional launch argument so this utility is
immediately usable while remaining independent of the service overlay examples.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _argument(name, default, description):
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def generate_launch_description():
    arguments = [
        _argument("odometry_topic", "/{hostname}/mavros/local_position/odom", "Published nav_msgs/Odometry topic."),
        _argument("rate_hz", "10.0", "Publication frequency in hertz."),
        _argument("frame_id", "map", "Odometry header frame."),
        _argument("child_frame_id", "base_link", "Odometry child frame."),
        _argument("random_seed", "-1", "PRNG seed; negative selects a random seed."),
        _argument("position_xy_limit", "50.0", "Absolute X/Y position limit."),
        _argument("position_z_min", "0.0", "Minimum Z position."),
        _argument("position_z_max", "20.0", "Maximum Z position."),
        _argument("roll_pitch_limit", "0.35", "Absolute roll/pitch limit in radians."),
        _argument("linear_xy_limit", "5.0", "Absolute X/Y linear velocity limit."),
        _argument("linear_z_limit", "2.0", "Absolute Z linear velocity limit."),
        _argument("angular_xy_limit", "0.5", "Absolute X/Y angular velocity limit."),
        _argument("angular_z_limit", "1.5", "Absolute Z angular velocity limit."),
        _argument("qos_depth", "10", "Publisher QoS queue depth."),
    ]

    floating = [
        "rate_hz", "position_xy_limit", "position_z_min", "position_z_max",
        "roll_pitch_limit", "linear_xy_limit", "linear_z_limit",
        "angular_xy_limit", "angular_z_limit",
    ]
    parameters = {
        "odometry_topic": LaunchConfiguration("odometry_topic"),
        "frame_id": LaunchConfiguration("frame_id"),
        "child_frame_id": LaunchConfiguration("child_frame_id"),
        "random_seed": ParameterValue(LaunchConfiguration("random_seed"), value_type=int),
        "qos_depth": ParameterValue(LaunchConfiguration("qos_depth"), value_type=int),
    }
    parameters.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in floating
    })

    return LaunchDescription(arguments + [
        Node(
            package="mrs_uav_bluetooth",
            executable="random_odometry_publisher",
            name="random_odometry_publisher",
            output="screen",
            parameters=[parameters],
        )
    ])
