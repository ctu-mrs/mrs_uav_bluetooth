#!/bin/bash
set -e

echo "[service_exec.sh] Starting mrs_uav_bluetooth service..."

export USER="${USER:-$(id -un)}"
export LOGNAME="${LOGNAME:-$USER}"

if [ -z "${HOME:-}" ]; then
	HOME=$(getent passwd "$(id -u)" | cut -d: -f6)
fi

export HOME="${HOME:-/root}"
export ROS_HOME="${ROS_HOME:-$HOME/.ros}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROS_HOME/log}"

mkdir -p "$ROS_LOG_DIR"

echo "[service_exec.sh] USER: $USER"
echo "[service_exec.sh] ROS_HOME: $ROS_HOME"
echo "[service_exec.sh] ROS_LOG_DIR: $ROS_LOG_DIR"

echo "[service_exec.sh] Sourcing ROS 2 setup..."
source /opt/ros/jazzy/setup.bash

echo "[service_exec.sh] Sourcing mrs_uav_bluetooth config..."
source /etc/ctu-mrs/mrs-uav-bluetooth/config

echo "[service_exec.sh] Launching mrs_uav_bluetooth service..."
exec ros2 launch mrs_uav_bluetooth service_node.launch.py

echo "[service_exec.sh] Script finished."
