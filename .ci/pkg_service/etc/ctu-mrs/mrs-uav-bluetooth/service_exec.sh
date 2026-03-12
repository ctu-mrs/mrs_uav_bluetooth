#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
source /etc/ctu-mrs/mrs-uav-bluetooth/config
exec ros2 launch mrs_uav_bluetooth service_node.launch.py
