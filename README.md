# MRS UAV Bluetooth

This repository contains the MRS UAV Bluetooth tool. It consists of a ROS 2 node running in background as a system service. The node manages Bluetooth Low Energy (BLE) server and client for automatic discovery and connection management among peer UAVs in a swarm.

The package has two entrypoints:

- `service_node`: the long-running system-side node that owns the BLE adapter, publishes status, hosts built-in BLE services, and exposes ROS 2 service calls.
- `user_node`: an experiment-side helper that temporarily applies an overlay YAML configuration on top of the service default config while it stays alive.

Main capabilities:

- BLE scanning, device discovery, connect, disconnect, pair, trust, and remove operations through ROS 2 services.
- Built-in BLE services for system time sharing and Wi-Fi configuration.
- Automatic peer discovery and connection management for UAV hostnames matching a configurable pattern.
- Declarative export and import of compact ROS 2 topic payloads over BLE characteristics or descriptors.
- Periodic status reporting plus raw notification and device inventory topics.

## Installation

```bash
sudo apt update
sudo apt install mrs-bluez # optional (replaces distro's bluez)
sudo apt install ros-jazzy-mrs-uav-bluetooth mrs-uav-bluetooth-service
```

Then verify the service status using `service mrs-uav-bluetooth status`, for a full log use `journalctl -u mrs-uav-bluetooth.service`.


## Configuration Files

- `/etc/ctu-mrs/mrs-uav-bluetooth/config`: configuration file sourced by the background system service on startup.
- `config/default.yaml`: base runtime configuration loaded by `service_node` at startup. It defines scan, advertising, time and Wi-Fi services, auto-connect behavior, logging, and declarative topic bridges.

The runtime configuration model is:

1. `service_node` always starts from `config/default.yaml`.
2. `user_node` may apply an overlay YAML via `ble/set_active_config`.
3. The overlay remains active only while the user node refreshes its lease.
4. When the lease expires or the user node exits, the service reverts to the default config.

Important configuration keys in `config/default.yaml`:

- `node_topics_prefix`: root of all node-created topics, usually `/{hostname}/ble`.
- `enable_scan`, `scan_mode`, `scan_publish_period`: BLE discovery behavior.
- `enable_server`, `enable_time_service`, `enable_wifi_service`: built-in BLE server features.
- `auto_connect_enable`, `auto_connect_whitelist`, `auto_connect_pattern`, `peer_connection_timeout`: automatic peer management.
- `allowed_wifi_networks`, `netplan_config_file`, `netplan_scripts_dir`: Wi-Fi control policy and backend integration.
- `status_report_period`, `log_topic_enable`, `verbose_log_file`: observability and logging.
- `shared_topics`: declarative topic bridge definitions.

Each `shared_topics` entry may define:

- `mode`: `export`, `import`, or `both`.
- `export_topic`: local ROS topic used as the logical identity of the bridge.
- `message_type`: ROS message type, for example `nav_msgs/msg/Odometry`.
- `rate_hz`: bridge publish or poll rate. `0` means unthrottled.
- `transport_endpoint`: `characteristic` or `descriptor`.
- `members`: ordered fixed-width field mapping packed into the BLE payload.
- `name`, `key`, `import_topic_suffix`: optional overrides for logging and imported topic naming.

## Usage

Typical experiment workflow is putting the following line into the tmux script (with your own custom yaml config):

```bash
ros2 launch mrs_uav_bluetooth user_node.launch.py config_path:=/opt/ros/jazzy/share/mrs_uav_bluetooth/config/example_sharing_odometry.yaml
```

The file `config/example_sharing_odometry.yaml` shows an example configuration file for sharing of a topic with `nav_msgs/msg/Odometry`, using a fixed packet layout with timestamp, position, orientation, and twist members.

While `user_node` is running, it keeps renewing the overlay lease and prints either the bluetooth status stream (default) or log stream from `/{hostname}/ble/status` or `/{hostname}/ble/log`.

Core published topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/{hostname}/ble/devices` | `mrs_uav_bluetooth/msg/BleDeviceArray` | Current BLE device snapshot from scanning and cached device state. |
| `/{hostname}/ble/notifications` | `mrs_uav_bluetooth/msg/BleNotification` | Raw notifications received from peer GATT characteristics. |
| `/{hostname}/ble/status` | `std_msgs/msg/String` | Human-readable periodic status report. |
| `/{hostname}/ble/log` | `std_msgs/msg/String` | Optional verbose log stream when `log_topic_enable` is enabled. |
| `/{hostname}/ble/overlay_keepalive` | `std_msgs/msg/Empty` | Keep-alive heartbeat for the user node configuration overlay. |
| `/{hostname}/ble/peers/<peer>/time_status` | `mrs_uav_bluetooth/msg/BlePeerTimeStatus` | Per-peer time sharing status including the latest peer timestamp and RTT estimate. |
| Derived imported topics | configured ROS message type | Auto-created publishers for bridges declared in `shared_topics`. |

### UAV Wi-Fi configuration

When `enable_wifi_service` is enabled, the service node exposes the current Wi-Fi SSID over BLE and accepts Wi-Fi updates from a connected peer. The node applies changes through the configured netplan backend.

- Only SSIDs listed in `allowed_wifi_networks` are accepted.
- `netplan_config_file` and `netplan_scripts_dir` control how the system network configuration is updated.
- The Wi-Fi service value is refreshed periodically according to `wifi_refresh_period`.
- This feature is intended for BLE-side provisioning; there is no separate ROS topic API for Wi-Fi credentials.

### System time sharing

When `enable_time_service` is enabled, the service publishes the local system time through a BLE characteristic. When auto-connect is enabled and a peer matches `auto_connect_pattern`, the node attempts to connect, pair, trust, and subscribe to the peer time characteristic automatically.

- Per-peer status is published on `/{hostname}/ble/peers/<peer>/time_status`.
- `BlePeerTimeStatus.peer_stamp` carries the most recently observed peer timestamp.
- `BlePeerTimeStatus.last_rtt_s` carries the round-trip estimate derived from the time writeback descriptor.
- Inactive peer time bridges are cleaned up after `peer_connection_timeout`.

### ROS 2 topic sharing

Topic sharing is configured declaratively in `shared_topics`. Each bridge packs selected scalar members from a ROS message into a compact BLE payload and recreates a ROS message on the receiving side.

- `mode: export` subscribes to a local ROS topic and writes BLE payloads to a peer.
- `mode: import` reads a peer BLE endpoint and republishes decoded ROS messages locally.
- `mode: both` configures both directions for the same logical bridge.
- `transport_endpoint: characteristic` uses notifications or characteristic writes.
- `transport_endpoint: descriptor` uses a descriptor as the data endpoint when that layout is more convenient.

By default, imported topics are published under the peer namespace rooted at `/{hostname}/ble/peers/<peer>/...`. The rest of the topic name remains the same as on the origin device unless `import_topic_suffix` overrides it.

Useful service calls while testing topic bridges:

```bash
ros2 service call /ble/set_scan_enabled mrs_uav_bluetooth/srv/SetScanEnabled "{enabled: true, transport: le}"
ros2 service call /ble/list_devices mrs_uav_bluetooth/srv/ListDevices "{connected_only: false}"
ros2 service call /ble/set_active_config mrs_uav_bluetooth/srv/SetActiveConfig "{config_path: '/path/to/overlay.yaml', hold_seconds: 3.0}"
```

## ROS 2 service calls

The node exposes the following service interfaces.

| Service | Type | Request fields | Response fields | Purpose |
| --- | --- | --- | --- | --- |
| `/ble/list_devices` | `mrs_uav_bluetooth/srv/ListDevices` | `connected_only: bool` | `success: bool`, `message: string`, `devices: BleDevice[]` | Return known devices, optionally only connected ones. |
| `/ble/get_device` | `mrs_uav_bluetooth/srv/GetDevice` | `mac: string` | `success: bool`, `message: string`, `device: BleDevice` | Return one device by MAC address. |
| `/ble/connect_device` | `mrs_uav_bluetooth/srv/ConnectDevice` | `mac: string`, `timeout: float32`, `wait_for_services: bool` | `success: bool`, `message: string`, `resolved_mac: string`, `device_path: string` | Connect to a device and optionally wait for GATT services to resolve. |
| `/ble/disconnect_device` | `mrs_uav_bluetooth/srv/DisconnectDevice` | `mac: string`, `timeout: float32` | `success: bool`, `message: string` | Disconnect from a device. |
| `/ble/pair_device` | `mrs_uav_bluetooth/srv/PairDevice` | `mac: string`, `timeout: float32`, `trust_after_pair: bool` | `success: bool`, `message: string` | Pair with a device and optionally mark it trusted. |
| `/ble/set_device_trust` | `mrs_uav_bluetooth/srv/SetDeviceTrust` | `mac: string`, `trusted: bool` | `success: bool`, `message: string` | Set or clear BlueZ trust for a device. |
| `/ble/remove_device` | `mrs_uav_bluetooth/srv/RemoveDevice` | `mac: string` | `success: bool`, `message: string` | Remove a device from the adapter cache. |
| `/ble/list_gatt_services` | `mrs_uav_bluetooth/srv/ListGattServices` | `mac: string` | `success: bool`, `message: string`, `services: BleGattService[]` | List remote GATT services of a connected device. |
| `/ble/list_gatt_characteristics` | `mrs_uav_bluetooth/srv/ListGattCharacteristics` | `mac: string` | `success: bool`, `message: string`, `characteristics: BleGattCharacteristic[]` | List remote GATT characteristics of a connected device. |
| `/ble/list_gatt_descriptors` | `mrs_uav_bluetooth/srv/ListGattDescriptors` | `mac: string`, `characteristic_path: string` | `success: bool`, `message: string`, `descriptors: BleGattDescriptor[]` | List descriptors below one characteristic. |
| `/ble/find_gatt_path` | `mrs_uav_bluetooth/srv/FindGattPath` | `mac: string`, `uuid: string`, `descriptor: bool`, `characteristic_path: string` | `success: bool`, `message: string`, `path: string` | Resolve a characteristic or descriptor path by UUID. |
| `/ble/read_gatt_value` | `mrs_uav_bluetooth/srv/ReadGattValue` | `path: string`, `descriptor: bool` | `success: bool`, `message: string`, `value: uint8[]` | Read a characteristic or descriptor payload. |
| `/ble/write_gatt_value` | `mrs_uav_bluetooth/srv/WriteGattValue` | `path: string`, `descriptor: bool`, `value: uint8[]`, `with_response: bool` | `success: bool`, `message: string` | Write a characteristic or descriptor payload. |
| `/ble/set_notify` | `mrs_uav_bluetooth/srv/SetNotify` | `path: string`, `enable: bool` | `success: bool`, `message: string` | Start or stop notifications on a characteristic path. |
| `/ble/set_scan_enabled` | `mrs_uav_bluetooth/srv/SetScanEnabled` | `enabled: bool`, `transport: string` | `success: bool`, `message: string`, `scanning: bool` | Start or stop scanning with the selected transport, usually `le`. |
| `/ble/configure_notification_bridge` | `mrs_uav_bluetooth/srv/ConfigureNotificationBridge` | `direction: string`, `mac: string`, `characteristic: string`, `topic_name: string`, `message_type: string`, `member_paths: string[]`, `rate_hz: float32`, `transport_endpoint: string`, `enable: bool` | `success: bool`, `message: string`, `resolved_uuid: string`, `resolved_path: string`, `resolved_topic: string`, `resolved_message_type: string`, `resolved_member_paths: string[]`, `resolved_rate_hz: float32`, `resolved_transport_endpoint: string` | Create, update, or remove an import or export bridge without editing YAML. |
| `/ble/reload_config` | `std_srvs/srv/Trigger` | none | `success: bool`, `message: string` | Reload the currently active configuration source. |
| `/ble/set_active_config` | `mrs_uav_bluetooth/srv/SetActiveConfig` | `config_path: string`, `hold_seconds: float32` | `success: bool`, `message: string`, `active_config_path: string`, `overlay_active: bool` | Activate or clear an overlay config lease. |

## ROS 2 message types

| Message | Fields | Meaning |
| --- | --- | --- |
| `mrs_uav_bluetooth/msg/BleDevice` | `mac`, `path`, `adapter`, `address_type`, `name`, `alias`, `hostname`, `icon`, `appearance`, `rssi`, `tx_power`, `pathloss`, `connected`, `paired`, `bonded`, `trusted`, `blocked`, `services_resolved`, `uuids`, `manufacturer_data_hex`, `service_data_hex`, `last_seen` | Complete snapshot of one BLE device as seen through BlueZ and the local hostname heuristics. |
| `mrs_uav_bluetooth/msg/BleDeviceArray` | `header`, `devices` | Timestamped collection of discovered or connected BLE devices. |
| `mrs_uav_bluetooth/msg/BleGattService` | `path`, `uuid`, `primary`, `device_path`, `includes` | One remote GATT service entry. |
| `mrs_uav_bluetooth/msg/BleGattCharacteristic` | `path`, `service_path`, `uuid`, `flags`, `notifying`, `mtu` | One remote GATT characteristic entry. |
| `mrs_uav_bluetooth/msg/BleGattDescriptor` | `path`, `characteristic_path`, `uuid`, `flags` | One remote GATT descriptor entry. |
| `mrs_uav_bluetooth/msg/BleNotification` | `header`, `mac`, `path`, `uuid`, `value` | Raw notification payload received from a remote characteristic. |
| `mrs_uav_bluetooth/msg/BlePeerTimeStatus` | `header`, `mac`, `peer_name`, `peer_stamp`, `last_rtt_s` | Status of the built-in peer time bridge for one connected UAV. |

For interface introspection during development:

```bash
ros2 interface show mrs_uav_bluetooth/msg/BleDevice
ros2 interface show mrs_uav_bluetooth/srv/ConfigureNotificationBridge
```
