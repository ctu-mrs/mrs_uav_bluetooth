import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mrs_uav_bluetooth")
    default_config = os.path.join(pkg_share, "config", "bluetooth_node.json")

    return LaunchDescription([
        Node(
            package="mrs_uav_bluetooth",
            executable="bluetooth_node",
            name="mrs_uav_bluetooth",
            output="screen",
            parameters=[
                default_config,
                {
                    # Startup-essential params (not runtime-reloadable)
                    "advertise_mode": "peripheral",
                    "pairing_agent": "NoInputNoOutput",
                    "enable_server": True,
                    "enable_scan": True,
                    "scan_mode": "le",
                    "netplan_config_file": "/etc/netplan/01-netcfg.yaml",
                    "netplan_scripts_dir": "/etc/ctu-mrs/uav-bluetooth/netplan-scripts",
                },
            ],
        ),
    ])
