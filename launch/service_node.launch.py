import os
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	"""Launch the Bluetooth service with UAV_NAME or the PC hostname."""
	hostname = os.getenv("UAV_NAME") or socket.gethostname()
	pkg_share = get_package_share_directory("mrs_uav_bluetooth")
	default_config = os.path.join(pkg_share, "config", "default.yaml")

	return LaunchDescription([
		Node(
			package="mrs_uav_bluetooth",
			executable="service_node",
			name="mrs_uav_bluetooth",
			output="screen",
			parameters=[
				{
					"hostname": hostname,
					"default_config_path": default_config,
				},
			],
		),
	])
