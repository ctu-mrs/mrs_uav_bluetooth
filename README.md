# MRS UAV Bluetooth

This repository contains the MRS UAV Bluetooth tool. It consists of a ROS 2 node running in background as a system service. The node manages Bluetooth Low Energy (BLE) server and client for an automatic discovery and connection management among peer UAVs in a swarm.

## Installation

```bash
sudo apt update
sudo apt install mrs-bluez # optional (replaces distro's bluez)
sudo apt install ros-jazzy-mrs-uav-bluetooth mrs-uav-bluetooth-service
```

Then verify the service status:
```bash
service mrs-uav-bluetooth status
```

## Features

- 