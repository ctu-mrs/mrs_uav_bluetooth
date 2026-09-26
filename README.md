![ROS Package Build](https://github.com/ctu-mrs/mrs_uav_bluetooth/actions/workflows/ros_package_build.yml/badge.svg)
![Generic Package Build](https://github.com/ctu-mrs/mrs_uav_bluetooth/actions/workflows/generic_package_build.yml/badge.svg)

# MRS UAV Bluetooth

This package lets UAVs share ROS 2 topics over Bluetooth. It can also find nearby UAVs, help configure their Wi-Fi, and provide an SSH connection when Wi-Fi is unavailable.

On UAVs, `service_node` runs in the background and manages Bluetooth. You launch `user_node` with a configuration file to start an experiment. When `user_node` stops, the service returns to its normal settings. The `tui_node` is an interactive tool you can run on a laptop.

## Install

On a laptop used only for the TUI, install the ROS package and its library:

```bash
sudo apt update
sudo apt install mrs-libsdbus-c++ ros-jazzy-mrs-uav-bluetooth
```

On UAVs, install the MRS BlueZ build and background service. This also installs
the ROS package and its dependencies:

```bash
sudo apt install mrs-bluez mrs-uav-bluetooth-service
```

The UAV service starts in the background when installed and at each boot. Check
it with `service mrs-uav-bluetooth status`. For logs, run
`journalctl -u mrs-uav-bluetooth.service`. Its shell startup settings live in
`/etc/ctu-mrs/mrs-uav-bluetooth/startup_config`. Bluetooth and ROS settings
belong in [default.yaml](config/default.yaml) or a user overlay.


If the laptop uses its distribution's BlueZ, enable its experimental
features. Open `/etc/bluetooth/main.conf`, set `Experimental = true` in the `[General]` section, then run `sudo service bluetooth restart`. This is needed for the TUI's advanced Bluetooth controls. `mrs-bluez` on UAVs enables the setting by default.

## Quick Start

### Use Text User Interface (TUI)

The TUI finds nearby UAVs and can open their Wi-Fi settings or an SSH connection. It does not need the UAV service running on your laptop.

```bash
ros2 launch mrs_uav_bluetooth tui_node.launch.py
```

When Bluetooth serial access is enabled on a UAV, press `h` in the TUI for SSH. You can also run `ros2 run mrs_uav_bluetooth mrs-uav-bluetooth-ssh uav01`.

### Share ROS topics

Choose one Bluetooth mode for all UAVs. These modes cannot run together on one Bluetooth adapter. Ordered by increasing complexity:

| Mode | What it does | Payload limit | Sample configuration |
| --- | --- | --- | --- |
| Advertisement | Sends small messages to nearby UAVs without connecting. | 17 B | [Odometry advertisement](config/examples/swarm_odom_advertisement_overlay.yaml) |
| GATT | Connects to nearby UAVs and sends larger messages. | 512 B | [Odometry GATT](config/examples/swarm_odom_gatt_overlay.yaml) |
| Mesh | Passes messages through other UAVs to reach farther away (multi-hop). | 377 B | [Odometry Mesh](config/examples/swarm_odom_mesh_overlay.yaml) |

Copy the sample for your mode and edit the UAV names in `peer_whitelist` and the source topic in `shared_topics`. The samples use `/uavXX/mavros/local_position/odom`. Start the overlay on each participating UAV with an absolute path to your edited file:

```bash
ros2 launch mrs_uav_bluetooth user_node.launch.py config_path:=/absolute/path/to/your-overlay.yaml
```

If you do not have real odometry yet, start a test source on each UAV:

```bash
ros2 launch mrs_uav_bluetooth random_odometry_publisher.launch.py rate_hz:=5.0
```

The advertisement sample sends time and X position at 1 Hz, keeping its payload within 17 bytes. The GATT sample sends time, pose, and velocity at 10 Hz. The Mesh sample sends time, pose, and velocity at 0.2 Hz. These samples do not include frame IDs or covariance. Edit the fields and rates in the overlay if your application needs something different.

Received GATT and advertisement topics appear below `/{hostname}/bluetooth/le/peers/<peer>/`. Mesh uses `/{hostname}/bluetooth/mesh/peers/<peer>/`. The peer segment is the hostname when it can be resolved. GATT and advertisements fall back to `mac_<address>`. Mesh packets contain a unicast address but no Bluetooth MAC, so a Mesh overlay without a peer list falls back to `unicast_<address>`. The `user_node` prints status for the active mode while it runs.

#### First use of Mesh

Copy the [Mesh sample](config/examples/swarm_odom_mesh_overlay.yaml) to every UAV and put the same nonempty ordered `peer_whitelist` in each copy. Start `user_node` on each UAV. The first reachable candidate starts a private Mesh with random keys. Other listed UAVs join automatically over nearby Bluetooth provisioning. Every member can relay and provision another listed UAV. The service stores each UAV's identity and keys locally, so you do not need to create or copy key files. Each UAV number must be unique and within 1..32767.

The preferred active provisioner follows the peer list. If it disappears, the next reachable member takes over, and the preferred member takes the role again when it returns. Do not delete or copy a UAV's stored Mesh identity to another UAV. Independently formed private Mesh networks do not yet merge automatically when they meet. Start the preferred candidate first when you need one common network.

### Use your own data or ROS node

The `shared_topics` section tells the service what to send and how to rebuild it on the receiving UAV. You can copy a [sample overlay](config/examples/) and change the topic, message type, fields, and rate without writing a Bluetooth node. The [default configuration](config/default.yaml) documents every setting.

Each entry needs a `transport` of `gatt`, `advertisement`, or `mesh`, a `mode` of `export`, `import`, or `both`, and a ROS `message_type`. `export_topic` is the local source. `import_topic_suffix` names the received topic below the peer prefix. A positive `rate_hz` limits sending to the latest sample at that rate. Set it to zero to send each source sample.

`payload_format: struct` sends only the ordered fields listed in `members`. Each member has a ROS field `path` and a compact wire `type`, such as `float32` or `time_ns`. This is the best choice for small radio packets. `raw` sends a `std_msgs/msg/UInt8MultiArray` byte array. `ros2` sends the whole serialized message and is usually too large for legacy advertisements. Advertisement and Mesh bridges also need a unique `channel_id` so the receiver knows which declaration should decode the bytes. Mesh bridges may additionally set `destination`, `app_key_index`, `element_index`, `force_segmented`, `vendor_opcode`, and `company_id`.

An overlay remains active while its `user_node` is running. The service returns to [default.yaml](config/default.yaml) when that node exits. The service package also offers time sharing, Wi-Fi setup over GATT, and optional Bluetooth serial access for SSH. These features are configured in the default file or an overlay. Only one radio mode can own an adapter at a time.

The service also exposes ROS topics and services under `/{hostname}/bluetooth/le`, `/{hostname}/bluetooth/mesh`, and `/{hostname}/bluetooth/config` for advanced integrations. These are not needed to run the automatic samples. Mesh provides a send service and topics for received messages, events, and status. Use `ros2 service list`, `ros2 topic list`, and `ros2 interface show` to inspect them.

The first GATT connection may need several pairing attempts. Larger multi-hop swarms have not yet been tested.

## ROS topic reference

These names use the default prefix `/{hostname}/bluetooth`. `{hostname}` is replaced with the local machine name. An overlay can change `node_topics_prefix`.

| Topic suffix | Type | What it carries |
| --- | --- | --- |
| `/system/status` | `std_msgs/msg/String` | Periodic service status. |
| `/system/log` | `std_msgs/msg/String` | Verbose log when `log_topic_enable` is on. |
| `/config/overlay_keepalive` | `std_msgs/msg/Empty` | Overlay lifetime heartbeat from `user_node`. |
| `/le/devices` | `mrs_uav_bluetooth/msg/BleDeviceArray` | Known nearby devices. |
| `/le/advertisements` | `mrs_uav_bluetooth/msg/BleDeviceArray` | Nearby advertisement data. |
| `/le/notifications` | `mrs_uav_bluetooth/msg/BleNotification` | Raw GATT notifications. |
| `/le/advertisement` | `std_msgs/msg/UInt8MultiArray` | Optional raw advertisement input in broadcast mode. |
| `/le/peers/<peer>/time_status` | `mrs_uav_bluetooth/msg/BlePeerTimeStatus` | Peer clock and round-trip estimate. |
| `/le/peers/<peer>/<suffix>` | Configured message type | Imported GATT or advertisement bridge. |
| `/mesh/status` | `mrs_uav_bluetooth/msg/MeshStatus` | Mesh attachment, relay, and swarm state. |
| `/mesh/events` | `mrs_uav_bluetooth/msg/MeshEvent` | Provisioning and scan events. |
| `/mesh/rx` | `mrs_uav_bluetooth/msg/MeshMessage` | Raw received Mesh messages for a custom controller. |
| `/mesh/peers/<peer>/<suffix>` | Configured message type | Imported Mesh bridge, named by configured hostname or unicast fallback. |

The raw advertisement topic is an input to the service. The keepalive topic is published by `user_node`. Other fixed topics in this table are service outputs. Imported bridge topics appear only when the matching `shared_topics` declaration is active.

## ROS service reference

Prefix each suffix with `/{hostname}/bluetooth`. The interface definitions in [srv](srv/) show the full request and response fields.

| Service suffix | Type | Use |
| --- | --- | --- |
| `/config/reload` | `std_srvs/srv/Trigger` | Reload the current configuration. |
| `/config/set_active` | `mrs_uav_bluetooth/srv/SetActiveConfig` | Set or clear an overlay lease. |
| `/le/devices/list` | `mrs_uav_bluetooth/srv/ListDevices` | List known devices. |
| `/le/devices/get` | `mrs_uav_bluetooth/srv/GetDevice` | Get one device by MAC. |
| `/le/devices/connect` | `mrs_uav_bluetooth/srv/ConnectDevice` | Connect to a device. |
| `/le/devices/disconnect` | `mrs_uav_bluetooth/srv/DisconnectDevice` | Disconnect from a device. |
| `/le/devices/pair` | `mrs_uav_bluetooth/srv/PairDevice` | Pair with a device. |
| `/le/devices/set_trust` | `mrs_uav_bluetooth/srv/SetDeviceTrust` | Change device trust. |
| `/le/devices/remove` | `mrs_uav_bluetooth/srv/RemoveDevice` | Remove a cached device. |
| `/le/gatt/services/list` | `mrs_uav_bluetooth/srv/ListGattServices` | List remote GATT services. |
| `/le/gatt/characteristics/list` | `mrs_uav_bluetooth/srv/ListGattCharacteristics` | List service characteristics. |
| `/le/gatt/descriptors/list` | `mrs_uav_bluetooth/srv/ListGattDescriptors` | List characteristic descriptors. |
| `/le/gatt/path/find` | `mrs_uav_bluetooth/srv/FindGattPath` | Find a remote characteristic or descriptor. |
| `/le/gatt/value/read` | `mrs_uav_bluetooth/srv/ReadGattValue` | Read a remote value. |
| `/le/gatt/value/write` | `mrs_uav_bluetooth/srv/WriteGattValue` | Write a remote value. |
| `/le/gatt/notifications/set` | `mrs_uav_bluetooth/srv/SetNotify` | Start or stop notifications. |
| `/le/scan/set_enabled` | `mrs_uav_bluetooth/srv/SetScanEnabled` | Control ordinary LE scanning. |
| `/le/notification_bridge/configure` | `mrs_uav_bluetooth/srv/ConfigureNotificationBridge` | Change a GATT bridge at runtime. |
| `/mesh/network` | `mrs_uav_bluetooth/srv/MeshNetwork` | Join, attach, create, import, or leave a Mesh network. |
| `/mesh/send` | `mrs_uav_bluetooth/srv/MeshSend` | Send raw Mesh or model data. |
| `/mesh/manage` | `mrs_uav_bluetooth/srv/MeshManagement` | Scan, provision, and configure Mesh keys or nodes. |
| `/mesh/swarm` | `mrs_uav_bluetooth/srv/MeshSwarm` | View, join, or leave a logical swarm. |

Automatic Mesh overlays own their network identity, so their network and key management calls are restricted. Use `/mesh/swarm` to change logical membership while the physical relay stays up.

## Message reference

| Message | Main fields |
| --- | --- |
| `BleDevice` | Address, name, signal strength, connection and trust state, services, and advertisement data. |
| `BleDeviceArray` | Timestamp and devices. |
| `BleGattService` | Path, UUID, parent device, and included services. |
| `BleGattCharacteristic` | Path, UUID, flags, notification state, and MTU. |
| `BleGattDescriptor` | Path, UUID, and flags. |
| `BleNotification` | Timestamp, peer MAC, characteristic path, UUID, and bytes. |
| `BlePeerTimeStatus` | Peer name, peer timestamp, and round-trip estimate. |
| `MeshStatus` | Attachment, token, addresses, relay state, swarm members, and errors. |
| `MeshEvent` | Provisioning event, UUID, signal strength, address, and result. |
| `MeshMessage` | Mesh source, destination, key indices, and raw bytes. |
