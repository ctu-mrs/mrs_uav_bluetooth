# MRS UAV Bluetooth

This repository contains the MRS UAV Bluetooth package. It contains custom implementations of Bluetooth Low Energy (BLE) server and client behavior, and provides ROS 2 compability interfaces for a simple use onboard UAVs.

The package providese these standalone nodes:

- `service_node`: a node running at background that owns the BLE adapter, performs periodic scans, hosts built-in GATT services, exposes ROS 2 services and topics, etc.
- `user_node`: a helper node for user TMUX sessions that temporarily applies an overlay YAML configuration on top of the service default config while it stays alive.
- `tui_node`: a node with a keyboard-controllable text user interface, which you can use on your laptop for a quick access to nearby BLE-enabled UAVs and their Wi-Fi network settings.
- `test_node`: a testing node for advertisement-based and connection-based communication modes.

Main capabilities:

- BLE scanning, device discovery, connect, disconnect, pair, trust, and remove operations through ROS 2 services.
- Built-in BLE services for system time sharing and Wi-Fi configuration.
- Automatic peer discovery and connection management for UAV hostnames matching a configurable pattern.
- Declarative export and import of compact ROS 2 topic payloads over BLE characteristics or descriptors.
- Periodic status reporting plus raw notification and device inventory topics.

## Installation

```bash
sudo apt update
sudo apt install mrs-bluez # recommended for UAVs, optional
sudo apt install mrs-libsdbus-c++ # v2 version, required
sudo apt install ros-jazzy-mrs-uav-bluetooth # the ROS 2 package
sudo apt install mrs-uav-bluetooth-service # for UAVs only
```

Then verify the service status using `service mrs-uav-bluetooth status`, for a full log use `journalctl -u mrs-uav-bluetooth.service`.

The `mrs-bluez` package has experimental mode enabled by default. It provides advanced Bluez control, which required for proper functionality of the connection-based communication between UAVs and of the TUI node on user laptops. If the official bluez package is used with this ROS package, the experimental mode must be enabled, usually by setting `Experimental = true` in `/etc/bluetooth/main.conf` and applying changes using `sudo service bluetooth restart`.

## Usage

The `tui_node` can be started simply using its launch script:
```bash
ros2 launch mrs_uav_bluetooth tui_node.launch.py
```
and does not require ROS Middleware (RMW) or Bluetooth service node running.

All other uses of this package depend on both RMW and running service node. The `service_node` is launched automatically in the systemd service when `mrs-uav-bluetooth-service` package is installed (on UAVs), otherwise it can be launched manually by user.

### Advertisement-based communication

The simplest use case requires you to create your own publisher of `std_msgs/UInt8MultiArray` data (at `/{hostname}/ble/adv_local_extra`). These bytes are used to update user data in the BLE advertisement of the local device. Custom user data of the other devices can be obtained by subscribing to `/{hostname}/ble/advertisements` topic. Note that the maximum number of bytes is quite limited by the adapter (31 B on Raspberry Pi 5, 251 B on NUCs), and data rate of this communication channel also depends on device scan availability.

To test this mode, run the test node in advertisement mode. The node will regularly update the custom advertisement data with the UAV's system timestamps (as little-endian `uint64`), and immediately log decoded timestamps advertised by the other devices.

```bash
ros2 launch mrs_uav_bluetooth test_node.launch.py mode:=advertisement rate_hz:=1.0
```
Additionally, the test node in this mode subscribes to a publisher of `nav_msgs/msg/Odometry` at path configured in the launch file. When the topic is available, the test node appends 13 floats with the odometry data to the user advertisement data. At the same time, all user data after timestamps is parsed and published in a new per-device odometry topic. The test node in this mode may be included in your TMUX session as well, to provide a simple multi-UAV positioning network.

### Connection-based communication

This is the fully-featured use case of the BLE package. Whitelisted devices get connected automatically, while establishing a two-way handshake and producing custom ROS 2 topics. Typical experiment workflow is putting the following line into the TMUX session (and creating your own custom yaml config):

```bash
ros2 launch mrs_uav_bluetooth user_node.launch.py config_path:=/opt/ros/jazzy/share/mrs_uav_bluetooth/config/example_sharing_odometry.yaml
```

The file `config/example_sharing_odometry.yaml` shows an example configuration file for sharing a topic with a message type `nav_msgs/msg/Odometry`, using a fixed packet layout with timestamp, position, orientation, and twist members. You can run a test node in an odometry mode, containing a random odometry publisher in order to test the example config setup, and the data transfer can be validated by echoing the ROS topic on the connected devices:

```bash
ros2 launch mrs_uav_bluetooth test_node.launch.py mode:=odometry rate_hz:=10.0
```

While `user_node` is running, it keeps renewing the overlay lease and prints either the bluetooth status stream (default) or log stream from `/{hostname}/ble/status` or `/{hostname}/ble/log` (configurable).

Note that when two BLE devices get connected, the advertisement-based communication stops working as the LE device discovery no longer provides the advertising information for them (data, RSSI etc.).

### UAV Wi-Fi configuration

When `enable_wifi_service` is enabled, the service node exposes three BLE characteristics: readable/writable SSID and password characteristics plus a readable/notifiable status text characteristic. The node applies changes through the configured netplan backend.

- The provided `tui_node` can be used for a simple Wi-Fi configuration of connected UAVs from your laptop.
- Only SSIDs listed in `allowed_wifi_networks` are accepted.
- The service writes `wifi_netplan_config_path` and then runs `netplan apply`.
- Writing a non-empty password updates the stored password for the selected access point.
- If writing the file or `netplan apply` fails, the previous netplan file is restored and re-applied.
- The status characteristic reports the latest Wi-Fi provisioning success or error text.
- The Wi-Fi service value is refreshed periodically according to `wifi_refresh_period`.
- This feature is intended for BLE-side provisioning; there is no separate ROS topic API for Wi-Fi credentials.

### System time sharing

When `enable_time_service` is enabled, the service publishes the local system time through a BLE characteristic. When auto-connect is enabled and a peer matches `auto_connect_pattern`, the node attempts to connect, pair, trust, and subscribe to the peer time characteristic automatically.

- Per-peer status is published on `/{hostname}/ble/peers/<peer>/time_status`.
- `BlePeerTimeStatus.peer_stamp` carries the most recently observed peer timestamp.
- `BlePeerTimeStatus.last_rtt_s` carries the round-trip estimate derived from the time writeback descriptor.
- Inactive peer time bridges are cleaned up after `peer_connection_timeout`.
- The provided `tui_node` shows the decoded time of the connected UAV.

### Declarative topic sharing

Topic sharing is configured declaratively in `shared_topics`. Each bridge packs selected scalar members from a ROS message into a compact BLE payload and recreates a ROS message on the receiving side.

- `mode: export` subscribes to a local ROS topic and writes BLE payloads to a peer characteristic.
- `mode: import` reads a peer BLE characteristic and republishes decoded ROS messages locally.
- `mode: both` configures both directions for the same logical bridge.
- Each exported bridge uses a service named `bridge:/{hostname}/...` and a single read/notify characteristic named `/{hostname}/...`.
- Bridge payloads live directly in the characteristic value; metadata stays in a small descriptor set.
- Compact bridge members may target array elements and slices, for example `position[0]`, `position[1:3]`, or `covariance[:]` for fixed-size arrays.
- Open-ended dynamic-array mappings are supported for one final array member per compact bridge payload. For full dynamic ROS messages, use `payload_format: ros2`.

By default, imported topics are published under the peer namespace rooted at `/{hostname}/ble/peers/<peer>/...`. The rest of the topic name remains the same as on the origin device unless `import_topic_suffix` overrides it.

Useful service calls while testing topic bridges:

```bash
ros2 service call /uavXX/ble/set_scan_enabled mrs_uav_bluetooth/srv/SetScanEnabled "{enabled: true, transport: le}"
ros2 service call /uavXX/ble/list_devices mrs_uav_bluetooth/srv/ListDevices "{connected_only: false}"
ros2 service call /uavXX/ble/set_active_config mrs_uav_bluetooth/srv/SetActiveConfig "{config_path: '/path/to/overlay.yaml', hold_seconds: 3.0}"
```

### Known issues

- When `auto_pair` is disabled, peer connections may reset after few minutes for no apparent reason. Thus, it is recommended to keep pairing enabled.
- The devices usually don't connect successfully on the first attempt. However, after one ore more automatic retries they should end up connected and paired successfully. It can take several minutes.


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
- `advertise_mode`, `advertise_extra_data_topic`: common advertisement settings kept in the base config. Additional BlueZ advertisement fields are also accepted in overlay YAMLs when needed.
- `auto_connect_enable`, `auto_connect_whitelist`, `auto_connect_pattern`, `peer_connection_timeout`: automatic peer management.
- `wifi_netplan_config_path`: netplan file updated by the BLE Wi-Fi service.
- `allowed_wifi_networks`: Wi-Fi SSIDs that may be selected over BLE.
- `status_report_period`, `log_topic_enable`, `verbose_log_file`: observability and logging.
- `shared_topics`: declarative topic bridge definitions.

Each `shared_topics` entry may define:

- `mode`: `export`, `import`, or `both`.
- `export_topic`: local ROS topic used as the logical identity of the bridge.
- `message_type`: ROS message type, for example `nav_msgs/msg/Odometry`.
- `rate_hz`: bridge publish or poll rate. `0` means unthrottled.
- `members`: ordered fixed-width field mapping packed into the BLE payload. Array leaves support index and slice syntax such as `data[0]`, `data[2:6]`, and `covariance[:]` for fixed-size arrays.
- `name`, `key`, `import_topic_suffix`: optional overrides for logging and imported topic naming.


## ROS 2

### Published topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/{hostname}/ble/devices` | `mrs_uav_bluetooth/msg/BleDeviceArray` | Current BLE device snapshot from scanning and cached device state. |
| `/{hostname}/ble/advertisements` | `mrs_uav_bluetooth/msg/BleDeviceArray` | Scan-side view of remote devices that currently expose advertisement user data through manufacturer data, service data, or safe raw advertising data. |
| `/{hostname}/ble/notifications` | `mrs_uav_bluetooth/msg/BleNotification` | Raw notifications received from peer GATT characteristics. |
| `/{hostname}/ble/status` | `std_msgs/msg/String` | Human-readable periodic status report. |
| `/{hostname}/ble/log` | `std_msgs/msg/String` | Optional verbose log stream when `log_topic_enable` is enabled. |
| `/{hostname}/ble/adv` | `std_msgs/msg/UInt8MultiArray` | Optional raw extra advertisement payload. The node subscribes only while publishers exist and advertises the bytes as an extra BlueZ AD block. |
| `/{hostname}/ble/overlay_keepalive` | `std_msgs/msg/Empty` | Keep-alive heartbeat for the user node configuration overlay. |
| `/{hostname}/ble/peers/<peer>/time_status` | `mrs_uav_bluetooth/msg/BlePeerTimeStatus` | Per-peer time sharing status including the latest peer timestamp and RTT estimate. |
| Derived imported topics | configured ROS message type | Auto-created publishers for bridges declared in `shared_topics`. |

### Service calls

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
| `/ble/configure_notification_bridge` | `mrs_uav_bluetooth/srv/ConfigureNotificationBridge` | `direction: string`, `mac: string`, `characteristic: string`, `topic_name: string`, `message_type: string`, `member_paths: string[]`, `rate_hz: float32`, `enable: bool` | `success: bool`, `message: string`, `resolved_uuid: string`, `resolved_path: string`, `resolved_topic: string`, `resolved_message_type: string`, `resolved_member_paths: string[]`, `resolved_rate_hz: float32` | Create, update, or remove an import or export bridge without editing YAML. |
| `/ble/reload_config` | `std_srvs/srv/Trigger` | none | `success: bool`, `message: string` | Reload the currently active configuration source. |
| `/ble/set_active_config` | `mrs_uav_bluetooth/srv/SetActiveConfig` | `config_path: string`, `hold_seconds: float32` | `success: bool`, `message: string`, `active_config_path: string`, `overlay_active: bool` | Activate or clear an overlay config lease. |

### Message types

| Message | Fields | Meaning |
| --- | --- | --- |
| `mrs_uav_bluetooth/msg/BleDevice` | `mac`, `path`, `adapter`, `address_type`, `name`, `alias`, `hostname`, `icon`, `appearance`, `rssi`, `tx_power`, `pathloss`, `connected`, `paired`, `bonded`, `trusted`, `blocked`, `services_resolved`, `uuids`, `manufacturer_data`, `service_data`, `advertising_flags`, `advertising_data`, `last_seen` | Complete snapshot of one BLE device as seen through BlueZ and the local hostname heuristics, including advertisement bytes selected by the service node. |
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
