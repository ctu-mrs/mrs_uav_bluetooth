![ROS Package Build](https://github.com/ctu-mrs/mrs_uav_bluetooth/actions/workflows/ros_package_build.yml/badge.svg)
![Generic Package Build](https://github.com/ctu-mrs/mrs_uav_bluetooth/actions/workflows/generic_package_build.yml/badge.svg)

# MRS UAV Bluetooth

This package lets UAVs share ROS 2 topics over Bluetooth. It can also find nearby UAVs, help configure their Wi-Fi, and provide an SSH connection when Wi-Fi is unavailable.

On UAVs, `service_node` runs in the background and manages Bluetooth. You launch `user_node` with a configuration file to start an experiment. When `user_node` stops, the service returns to its normal settings. The `tui_node` is an interactive tool you can run on a laptop.

## Install

On a laptop used only for the TUI, install just the ROS package:

```bash
sudo apt update
sudo apt install ros-jazzy-mrs-uav-bluetooth
```

The ROS package pulls in its runtime libraries automatically and works with
the distribution's BlueZ daemon. Do not install `mrs-bluez` or
`mrs-uav-bluetooth-service` on a TUI-only laptop.

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


The distribution's default BlueZ configuration is sufficient for TUI
discovery and GATT controls. For advanced simultaneous LE/RFCOMM bearer
handling, enable BlueZ's optional experimental interfaces by setting
`Experimental = true` in the `[General]` section of
`/etc/bluetooth/main.conf`, then run `sudo service bluetooth restart`.
`mrs-bluez` enables those interfaces on UAVs by default.

## Quick Start

### Use Text User Interface (TUI)

The TUI runs on your laptop and connects to nearby UAVs for Wi-Fi setup or SSH.

```bash
ros2 launch mrs_uav_bluetooth tui_node.launch.py
```

When Bluetooth serial access is enabled on a UAV, press `h` in the TUI for SSH. You can also run `ros2 run mrs_uav_bluetooth mrs-uav-bluetooth-ssh uav01`.

### Share ROS topics

Choose the same topic-sharing mode on the participating UAVs. The service switches the adapter to that mode when you load an overlay.

| Mode | What it does | User payload limit | Sample configuration |
| --- | --- | --- | --- |
| Advertisement | Sends small messages to nearby UAVs without connecting at small rates. | 26 B (legacy) / 246 B | [Odometry advertisement](config/examples/swarm_odom_advertisement_overlay.yaml) |
| GATT | Connects to nearby UAVs and sends larger messages at higher rates. | 512 B | [Odometry GATT](config/examples/swarm_odom_gatt_overlay.yaml) |
| Mesh | Passes small messages through other UAVs to reach farther away (multi-hop) at small rates. | 370 B | [Odometry Mesh](config/examples/swarm_odom_mesh_overlay.yaml) |

Copy the sample for your mode and edit the UAV names in `peer_whitelist` if needed. The samples are typically installed in `/opt/ros/jazzy/share/mrs_uav_bluetooth/config/examples` and use `/{hostname}/estimation_manager/odom_main`, where `{hostname}` expands to the local UAV name (for example, `/uav17/estimation_manager/odom_main`). Start the overlay on each participating UAV with an absolute path to your edited file:

```bash
ros2 launch mrs_uav_bluetooth user_node.launch.py config_path:=/absolute/path/to/your-overlay.yaml
```

When no real odometry source is available, start a test source on each UAV:

```bash
ros2 launch mrs_uav_bluetooth random_odometry_publisher.launch.py rate_hz:=10.0
```

The advertisement sample packs timestamp, orientation, and XYZ position into 26 bytes. Mesh uses 20 bytes by storing XYZ to the nearest centimetre within -327.68 to 327.67 m on each axis. GATT sends timestamp, pose, and velocity in 60 bytes. The sample message rates are 1 Hz for advertisement, 0.5 Hz for Mesh, and 10 Hz for GATT. All YAML options are listed in the [configuration table](config/README.md).

Received GATT and advertisement topics appear below `/{hostname}/bluetooth/le/peers/<peer>/`. Mesh uses `/{hostname}/bluetooth/mesh/peers/<peer>/`. The peer segment is the hostname when it can be resolved. GATT and advertisements fall back to `mac_<address>`. Mesh peer identity uses a unicast address, so an empty Mesh peer list uses the fallback `unicast_<address>`. The `user_node` prints status for the active mode while it runs.

#### First use of Mesh

Copy the [Mesh sample](config/examples/swarm_odom_mesh_overlay.yaml) to every UAV and put the same nonempty ordered `peer_whitelist` in each copy. Start `user_node` on each UAV. The first reachable candidate starts a private Mesh with random keys. Other listed UAVs join automatically over nearby Bluetooth provisioning. Every member can relay and provision another listed UAV. The service creates and stores each UAV's identity and keys automatically. Each UAV number must be unique and within 1..32767.

The preferred active provisioner follows the peer list. If it disappears, the next reachable member takes over, and the preferred member takes the role again when it returns. Keep each stored Mesh identity on its original UAV. Independently formed private Mesh networks currently remain separate when they meet.

### Use your own data or ROS node

The `shared_topics` section tells the service what to send and how to rebuild it on the receiving UAV. Copy a [sample overlay](config/examples/) and change the topic, message type, fields, and rate; configuration alone creates the Bluetooth bridge. The [default configuration](config/default.yaml) documents every setting.

Each entry needs a `transport` of `gatt`, `advertisement`, or `mesh`, a `mode` of `export`, `import`, or `both`, and a ROS `message_type`. `export_topic` is the local source. `import_topic_suffix` names the received topic below the peer prefix. A positive `rate_hz` limits sending to the latest sample at that rate. Set it to zero to send each source sample.

`payload_format: struct` uses only the ordered fields in `members_encode`. Each entry has a `target` and a `type`, which is the final transmission type such as `float32`, `uint8`, or `int16`. Without an `expression`, `target` is a ROS field copied directly. With an `expression`, `target` names the computed transmission value. In `members_decode`, each entry uses the same `target` and `expression` keys. There, `target` is the ROS field to fill and the expression refers to transmission-value names. Direct fields are reconstructed automatically.  Integer destinations are rounded when needed and always range-checked. Exporters and importers must use the same layout.

`payload_format: raw` uses a `std_msgs/msg/UInt8MultiArray` byte array. `payload_format: ros2` uses the whole serialized message and is usually too large for legacy advertisements. Mesh bridges and multi-topic advertisement bridges need a unique `channel_id` so the receiver knows which declaration should decode the bytes. The single advertisement bridge in the sample uses `framing: bare` to make all 26 data bytes available in legacy advertisements (e.g., on RPi5). Mesh destinations and keys are managed automatically. See the [configuration](config/default.yaml) for advanced settings.

An overlay remains active while its `user_node` is running. The service returns to [default.yaml](config/default.yaml) when that node exits. The service package also offers time sharing, Wi-Fi setup over GATT, and optional Bluetooth serial access for SSH. These features are configured in the default file or an overlay. The service selects one topic-sharing mode per adapter.

The service also exposes ROS topics and services under `/{hostname}/bluetooth/le`, `/{hostname}/bluetooth/mesh`, and `/{hostname}/bluetooth/config` for advanced integrations. Mesh provides a send service and topics for received messages, events, and status. Use `ros2 service list`, `ros2 topic list`, and `ros2 interface show` to inspect them.

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
