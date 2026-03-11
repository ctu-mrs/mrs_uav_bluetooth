#!/bin/bash
set -e

TARGET_USER=${SERVICE_USER:-${SUDO_USER:-${USER:-mrs}}}

if [ "$(id -un)" = "root" ] && [ "$TARGET_USER" != "root" ]; then
	exec runuser -u "$TARGET_USER" -- env \
		HOME="${HOME:-$(getent passwd "$TARGET_USER" | cut -d: -f6)}" \
		USER="$TARGET_USER" \
		LOGNAME="$TARGET_USER" \
		SERVICE_USER="$TARGET_USER" \
		/etc/ctu-mrs/mrs-uav-bluetooth/service_exec.sh
fi

export USER=${USER:-$TARGET_USER}
if [ -z "${HOME:-}" ]; then
	HOME=$(getent passwd "$USER" | cut -d: -f6)
fi
export HOME=${HOME:-/home/mrs}
export LOGNAME=${LOGNAME:-$USER}
export ROS_HOME=${ROS_HOME:-$HOME/.ros}
export ROS_LOG_DIR=${ROS_LOG_DIR:-$ROS_HOME/log}
export ROS_WORKSPACE=${ROS_WORKSPACE:-$HOME/workspace}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}
export ZENOH_ROUTER_CONFIG_URI=${ZENOH_ROUTER_CONFIG_URI:-/opt/ros/jazzy/share/mrs_uav_deployment/config/zenoh/uav_router.json5}

echo "Running as user: $USER"
echo "Home directory: $HOME"
echo "ROS home: $ROS_HOME"
echo "ROS log directory: $ROS_LOG_DIR"
echo "ROS workspace: $ROS_WORKSPACE"
echo "RMW implementation: $RMW_IMPLEMENTATION"
echo "Zenoh router config: $ZENOH_ROUTER_CONFIG_URI"

mkdir -p "$ROS_LOG_DIR"

source /opt/ros/jazzy/setup.bash
if [ -f "$ROS_WORKSPACE/install/setup.bash" ]; then
	source "$ROS_WORKSPACE/install/setup.bash"
fi
exec ros2 launch mrs_uav_bluetooth bluetooth_node.launch.py
