#!/bin/bash
set -e

export HOME=${HOME:-/root}
export USER=${USER:-root}
export LOGNAME=${LOGNAME:-root}
export ROS_LOG_DIR=${ROS_LOG_DIR:-/var/log/mrs-uav-bluetooth}

mkdir -p "$ROS_LOG_DIR"

source /opt/ros/jazzy/setup.bash
exec ros2 launch mrs_uav_bluetooth bluetooth_node.launch.py
