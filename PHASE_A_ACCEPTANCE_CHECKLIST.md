# Phase A Acceptance Checklist

This document validates the current cutover plan against the original migration prompt and freezes the external contract for the remaining C++ migration work.

## Validation Against Original Prompt

- Result: aligned.
- The revised plan keeps the original prompt goals intact: same package name, unchanged ROS interfaces, unchanged launch entrypoints, and a final cutover to an async C++ runtime built on `sdbus-c++`.
- The revised plan improves the prompt by replacing generic migration guidance with repository-backed constraints from the current C++ baseline.
- The revised plan also sharpens the sequencing correctly: contract freeze first, then per-file audit, then async D-Bus/cache work, then bridge and peer event-driven cutover, then parity validation, then legacy-code cleanup.

## Frozen External Contract

### Package And Interface Contract

- Package name remains `mrs_uav_bluetooth`.
- Existing message files remain unchanged:
  - `msg/BleDevice.msg`
  - `msg/BleDeviceArray.msg`
  - `msg/BleGattService.msg`
  - `msg/BleGattCharacteristic.msg`
  - `msg/BleGattDescriptor.msg`
  - `msg/BleNotification.msg`
  - `msg/BlePeerTimeStatus.msg`
- Existing service files remain unchanged:
  - `srv/ConfigureNotificationBridge.srv`
  - `srv/ConnectDevice.srv`
  - `srv/DisconnectDevice.srv`
  - `srv/FindGattPath.srv`
  - `srv/GetDevice.srv`
  - `srv/ListDevices.srv`
  - `srv/ListGattCharacteristics.srv`
  - `srv/ListGattDescriptors.srv`
  - `srv/ListGattServices.srv`
  - `srv/PairDevice.srv`
  - `srv/ReadGattValue.srv`
  - `srv/RemoveDevice.srv`
  - `srv/SetActiveConfig.srv`
  - `srv/SetDeviceTrust.srv`
  - `srv/SetNotify.srv`
  - `srv/SetScanEnabled.srv`
  - `srv/WriteGattValue.srv`

### Launch Contract

- `launch/service_node.launch.py` launches executable `service_node` and exposes exactly one parameter override:
  - `default_config_path`
- `launch/user_node.launch.py` launches executable `user_node` and exposes exactly these launch arguments and parameter names:
  - `config_path`
  - `service_wait_timeout_sec`
  - `service_call_timeout_sec`
  - `deactivate_service_wait_timeout_sec`
  - `sentinel_topic_suffix`
  - `sentinel_publish_period_sec`
  - `min_sentinel_publish_period_sec`
  - `print_source`
- These names are frozen and must not change.

### Service Contract

- The live service registry is fixed at 18 services:
  - `ble/list_devices`
  - `ble/get_device`
  - `ble/connect_device`
  - `ble/disconnect_device`
  - `ble/pair_device`
  - `ble/set_device_trust`
  - `ble/remove_device`
  - `ble/list_gatt_services`
  - `ble/list_gatt_characteristics`
  - `ble/list_gatt_descriptors`
  - `ble/find_gatt_path`
  - `ble/read_gatt_value`
  - `ble/write_gatt_value`
  - `ble/set_notify`
  - `ble/set_scan_enabled`
  - `ble/configure_notification_bridge`
  - `ble/reload_config`
  - `ble/set_active_config`
- Note: `ble/reload_config` is implemented through `std_srvs::srv::Trigger`, but it is still part of the frozen external 18-service contract.

### Runtime Boundary Contract

- The C++ build already produces and installs `service_node` and `user_node`.
- The legacy Python runtime files under `src/` have been removed; the runtime tree is now C++-only.
- `src/app/user_overlay_node.cpp` currently depends on `ble/set_active_config` behaving synchronously from the caller perspective. That caller-visible behavior is frozen.
- Overlay behavior remains contractually visible through activation, sentinel publishing, status or log printing, and rollback to default on shutdown.

## Baseline Verification

- `CMakeLists.txt` already builds separate C++ libraries for util, config, bridge, bluez, gatt, network, ros, peer, and bridge manager domains.
- `CMakeLists.txt` installs `service_node`, `user_node`, `launch/`, `config/`, and exported headers.
- `package.xml` is already `ament_cmake`-based and already carries `yaml-cpp` and `sdbus-c++` dependencies.
- No package-level dependency forces Python to remain in the runtime, and the legacy Python implementation has been removed from `src/`.

## File-By-File Checklist

Legend:

- `preserve-shape`: existing structure is sound; only targeted parity or validation work remains.
- `parity-gap`: current C++ structure is usable, but behavior still diverges from the Python specification.
- `architecture-gap`: current implementation shape conflicts with the target async event-driven design and needs redesign.

### Minimum Audit Set

- `src/app/bluetooth_node.cpp`
  - Status: `architecture-gap`
  - Work: async redesign
  - Reason: still owns timer-driven peer progression and broader reconciliation logic; should become composition plus narrowly justified timers only.

- `src/bluez/bluez_client.cpp`
  - Status: `parity-gap`
  - Work: semantic parity
  - Reason: async operations are now reply- and cache-driven with node-owned timeout watchdogs instead of a dedicated deadline thread; remaining work is to keep shrinking broad predicate-style completion logic as peer and bridge runtime authority moves upward.

- `src/bluez/object_manager_cache.cpp`
  - Status: `parity-gap`
  - Work: semantic parity
  - Reason: the cache is already signal-driven and typed, and now cascades child removal notifications as GATT subtrees disappear; remaining work is tightening event semantics only where higher layers still need more explicit transition signals.

- `src/peer/peer_manager.cpp`
  - Status: `architecture-gap`
  - Work: async redesign
  - Reason: useful state vocabulary exists, but manager is still an adviser to node-owned reconciliation rather than the authoritative event-driven state machine.

- `src/bridge/export_bridge_manager.cpp`
  - Status: `parity-gap`
  - Work: semantic parity
  - Reason: export runtime is structurally sound, but lifecycle is still rebuilt from config application instead of cache-driven peer and service events.

- `src/bridge/import_bridge_manager.cpp`
  - Status: `parity-gap`
  - Work: semantic parity
  - Reason: descriptor polling exception is acceptable, but path refresh and notify enablement still depend on node-driven refresh instead of explicit async state transitions.

- `src/gatt/gatt_application.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: reusable exported object model is already close to the Python GATT base and should remain the generic export layer.

- `src/gatt/services/topic_bridge_service.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: metadata descriptors now follow the Python YAML-bytes serialization contract; remaining work is parity verification against real clients.

- `src/gatt/services/time_service.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: time writeback descriptor is now wired into callback semantics and peer RTT updates; remaining work is integration validation.

- `src/gatt/services/wifi_service.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: password descriptor read/write callbacks are now wired through the runtime; remaining work is deployed-client validation.

- `src/config/config_loader.cpp`
  - Status: `preserve-shape`
  - Work: semantic parity
  - Reason: deep merge, topic normalization, and bridge key derivation are largely present, but Python parity still needs explicit validation for metadata and edge-case schema handling.

- `src/config/overlay_config_manager.cpp`
  - Status: `parity-gap`
  - Work: semantic parity
  - Reason: overlay activation and rollback exist, but lease expiry still polls the ROS graph for publishers and needs a deliberate contract decision.

- `src/app/user_overlay_node.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: caller-facing overlay workflow is already present and should stay synchronous at the ROS boundary; verify end-to-end behavior after service internals go async.

- `src/ros/ros_interface_manager.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: thin assembly layer only; no architectural debt observed from current implementation shape.

- `src/ros/status_publisher.cpp`
  - Status: `preserve-shape`
  - Work: semantic parity
  - Reason: publishing structure is sound, but final topic content and status conversion still need parity validation against current Python-visible behavior.

### Additional Verified Files Outside The Minimum Set

- `src/bridge/payload_codec.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: current implementation is treated as close to Python parity and needs coverage rather than redesign.

- `src/config/shared_topic_config.cpp`
  - Status: `preserve-shape`
  - Work: test-only validation
  - Reason: keep as supporting schema logic unless parity tests expose a contract mismatch.

- `src/peer/peer_time_bridge.cpp`
  - Status: `parity-gap`
  - Work: semantic parity
  - Reason: retained as the likely home for time writeback and RTT parity once `TimeService` callback wiring is completed.

## Phase B Entry Criteria

- Contract freeze accepted.
- This checklist becomes the baseline work tracker.
- Legacy Python runtime removal is complete.
- Remaining work is C++ parity validation and runtime hardening:
  - async `ObjectManagerCache` event precision is improved where peer and bridge orchestration still need narrower signals,
  - `BluezClient` generic predicate completion paths are reduced further where explicit subsystem state transitions can replace them,
  - bridge metadata parity is validated,
  - time and Wi-Fi service callback parity is finished,
  - parity tests exist for the frozen contract.

## Immediate Next Actions

- Step 3: continue tightening `src/bluez/object_manager_cache.cpp` only where specific higher-level transitions still need richer typed signals beyond the current typed add/change/value/remove feed.
- Step 4: continue narrowing `src/bluez/bluez_client.cpp` only where operation completion still depends on generic predicate checks instead of explicit peer or bridge runtime transitions.
- Phase B progress: cache event precision has been improved enough to emit child GATT removals explicitly, and BlueZ client deadlines now run through node-owned watchdog timers rather than a dedicated worker thread; next major item is reducing `BluetoothNode` timer-driven peer progression and moving authority into peer and bridge state transitions.
- Legacy Python-removal work is complete in `src/`; remaining migration work is entirely in the C++ runtime and validation layers.