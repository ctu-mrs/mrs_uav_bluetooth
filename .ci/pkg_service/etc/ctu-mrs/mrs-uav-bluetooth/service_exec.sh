#!/bin/bash
set -e

export USER=${USER:-$(id -un)}
if [ -z "${HOME:-}" ]; then
	HOME=$(getent passwd "$USER" | cut -d: -f6)
fi
export HOME=${HOME:-/tmp}
export LOGNAME=${LOGNAME:-$USER}
export ROS_HOME=${ROS_HOME:-$HOME/.ros}
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ROS_HOME/log}

mkdir -p "$ROS_LOG_DIR"

source /opt/ros/jazzy/setup.bash
exec ros2 launch mrs_uav_bluetooth bluetooth_node.launch.py
