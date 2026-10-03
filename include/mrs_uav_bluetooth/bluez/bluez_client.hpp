// SPDX-License-Identifier: BSD-3-Clause
/// \file include/mrs_uav_bluetooth/bluez/bluez_client.hpp
/// \brief Declares the bluez client component of the BlueZ system-D-Bus integration layer.

#pragma once

#include "mrs_uav_bluetooth/bluez/bluez_types.hpp"
#include "mrs_uav_bluetooth/bluez/dbus_connection.hpp"
#include "mrs_uav_bluetooth/bluez/object_manager_cache.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sdbus-c++/sdbus-c++.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace mrs_uav_bluetooth::bluez {

/// Notification callback: (data, uuid, characteristic_path).
using NotificationCallback = std::function<void(const std::vector<uint8_t>&,
                                                const std::string&,
                                                const std::string&)>;

/// GATT event callback: (event_type, object_path, extra_info).
using GattEventCallback = std::function<void(const std::string& event,
                                             const std::string& path,
                                             const std::string& detail)>;

/// High-level BLE client wrapping BlueZ Device1, GattCharacteristic1, and
/// GattDescriptor1 D-Bus interfaces. Delegates device/GATT caching to
/// ObjectManagerCache and uses blocking control-path operations.
class BluezClient {
public:
    /// \brief Bind high-level device and GATT operations to an adapter and cache.
    /// \param dbus shared D-Bus connection used for BlueZ calls.
    /// \param cache Live ObjectManager cache used for peer and GATT lookups.
    /// \param adapter_path BlueZ Adapter1 object path used by this controller.
    /// \param logger ROS logger used for diagnostics.
    BluezClient(DbusConnection& dbus,
                ObjectManagerCache& cache,
                const std::string& adapter_path,
                rclcpp::Logger logger);
    /// \brief Remove signal matches and notification handlers before cache shutdown.
    ~BluezClient();

    // ---- Discovery ----
    /// \brief Apply the discovery filter and enter BlueZ discovery idempotently.
    /// \param transport BlueZ discovery filter: low energy classic or automatic.
    /// \param make_discoverable_while_scanning whether discovery temporarily exposes this adapter to peers.
    /// \return True if discovery is active after the request; otherwise false.
    bool start_scan(const std::string& transport = "le",
                    bool make_discoverable_while_scanning = false);
    /// \brief Leave BlueZ discovery idempotently and restore temporary discoverability.
    /// \return True if discovery is inactive after the request; otherwise false.
    bool stop_scan();
    /// \brief Return whether Bluetooth discovery is active.
    /// \return True when scanning; otherwise false.
    bool is_scanning() const;

    // ---- Device queries (from cache) ----
    /// \brief Snapshot all cached Bluetooth devices.
    /// \return Snapshot of all cached Bluetooth devices.
    std::vector<DeviceInfo> get_devices() const;
    /// \brief Look up one cached device by address.
    /// \param mac peer Bluetooth MAC address.
    /// \return Cached peer with the normalized address or std::nullopt when absent.
    std::optional<DeviceInfo> get_device(const std::string& mac) const;
    /// \brief Snapshot cached devices currently marked connected.
    /// \return Snapshot of cached devices currently marked connected.
    std::vector<DeviceInfo> get_connected_devices() const;

    // ---- Connection management ----
    /// Gate queued connection/pairing work before a connectionless overlay
    /// cancels pending Device1 operations. Read again before submitting a call.
    /// \param allowed whether new peer connection attempts are currently permitted.
    void set_connections_allowed(bool allowed) {
        // Gate new connection attempts during overlay handoff while allowing explicit teardown.
        connections_allowed_.store(allowed);
    }

    /// \brief Connect the selected peer and wait for BlueZ to report it connected.
    /// \param mac peer Bluetooth MAC address.
    /// \param timeout_s Maximum seconds to wait for Device1 Connected.
    /// \param prefer_le whether to prefer the LE bearer for a dual-mode peer.
    /// \return True if the peer reached Connected before the deadline; otherwise false.
    bool connect(const std::string& mac,
                 double timeout_s = 15.0,
                 bool prefer_le = false);
    /// \brief Connect only the peer's low-energy bearer and wait for a usable GATT tree.
    /// \param mac peer Bluetooth MAC address.
    /// \param timeout_s Maximum seconds to wait for the low-energy bearer.
    /// \return True if the peer's remote GATT characteristics resolved before the deadline; otherwise false.
    bool connect_le_bearer(const std::string& mac, double timeout_s = 15.0);
    /// Cancel pending connection work and wait for the link to close.
    /// Packaged BlueZ suppresses automatic reconnect after explicit Disconnect.
    /// Connect re-enables it. Trust settings and bonds are preserved.
    /// \param mac peer Bluetooth MAC address.
    /// \param timeout_s Maximum seconds to wait for Device1 to disconnect.
    /// \return True if the peer became disconnected or was already absent; otherwise false.
    bool disconnect(const std::string& mac, double timeout_s = 10.0);
    /// \brief Ask BlueZ to establish one profile UUID on an already connected peer.
    /// \param mac peer Bluetooth MAC address.
    /// \param uuid Bluetooth profile UUID to connect or disconnect.
    /// \return True if BlueZ established or already had the requested profile; otherwise false.
    bool connect_profile(const std::string& mac, const std::string& uuid);
    /// \brief Ask BlueZ to tear down one profile UUID without dropping other bearers.
    /// \param mac peer Bluetooth MAC address.
    /// \param uuid Bluetooth profile UUID to connect or disconnect.
    /// \return True if BlueZ removed or had already removed the profile; otherwise false.
    bool disconnect_profile(const std::string& mac, const std::string& uuid);

    // ---- Pairing/trust ----
    /// \brief Pair the selected peer and return BlueZ failure detail when requested.
    /// \param mac peer Bluetooth MAC address.
    /// \param timeout_s Maximum seconds to wait for pairing to complete.
    /// \param error_detail optional output receiving the concrete BlueZ failure text.
    /// \return True if the peer reached Paired before the deadline; otherwise false.
    bool pair(const std::string& mac,
              double timeout_s = 30.0,
              std::string* error_detail = nullptr);
    /// \brief Mark the selected peer trusted in BlueZ.
    /// \param mac peer Bluetooth MAC address.
    /// \return True if BlueZ stored Trusted=true; otherwise false.
    bool trust(const std::string& mac);
    /// \brief Clear the selected peer’s trusted flag in BlueZ.
    /// \param mac peer Bluetooth MAC address.
    /// \return True if BlueZ stored Trusted=false; otherwise false.
    bool untrust(const std::string& mac);
    /// \brief Block future connections from the selected peer in BlueZ.
    /// \param mac peer Bluetooth MAC address.
    /// \return True if BlueZ stored Blocked=true; otherwise false.
    bool block(const std::string& mac);
    /// \brief Allow connections from a peer previously blocked in BlueZ.
    /// \param mac peer Bluetooth MAC address.
    /// \return True if BlueZ stored Blocked=false; otherwise false.
    bool unblock(const std::string& mac);
    /// \brief Remove the selected peer and its stored bond from the adapter.
    /// \param mac peer Bluetooth MAC address.
    /// \return True if BlueZ removed the peer or it was already absent; otherwise false.
    bool remove(const std::string& mac);
    /// \brief Persist BlueZ's preferred low-energy or classic bearer for a peer.
    /// \param mac peer Bluetooth MAC address.
    /// \param bearer BlueZ preferred bearer value, such as le or bredr.
    /// \return True if BlueZ stored the requested bearer preference; otherwise false.
    bool set_preferred_bearer(const std::string& mac, const std::string& bearer);

    // ---- Services resolved waiter ----
    /// \brief Wait for BlueZ to finish discovering the selected peer’s services.
    /// \param mac peer Bluetooth MAC address.
    /// \param timeout_s Maximum seconds to wait for BlueZ service discovery.
    /// \return True if BlueZ completed remote service discovery before the deadline; otherwise false.
    bool wait_services_resolved(const std::string& mac, double timeout_s = 15.0);
    /// \brief Reload the peer's resolved GATT subtree when ObjectManager signals were missed.
    /// \param mac peer Bluetooth MAC address.
    /// \return True if the resolved peer GATT tree was loaded into the cache; otherwise false.
    bool refresh_gatt_snapshot(const std::string& mac) const;

    // ---- GATT queries (from cache) ----
    /// \brief List cached services for one peer.
    /// \param mac peer Bluetooth MAC address.
    /// \return Cached services belonging to the requested peer.
    std::vector<GattServiceInfo> list_services(const std::string& mac) const;
    /// \brief List cached characteristics for a peer or service.
    /// \param mac peer Bluetooth MAC address.
    /// \return Cached characteristics under the requested peer or service.
    std::vector<GattCharacteristicInfo> list_characteristics(const std::string& mac) const;
    /// \brief List cached descriptors for a characteristic.
    /// \param mac peer Bluetooth MAC address.
    /// \param chrc_path Optional BlueZ characteristic path limiting the returned descriptors.
    /// \return Cached descriptors under the requested characteristic.
    std::vector<GattDescriptorInfo> list_descriptors(const std::string& mac,
                                                      const std::string& chrc_path = "") const;
    /// \brief Resolve a characteristic UUID to its D-Bus path.
    /// \param mac peer Bluetooth MAC address.
    /// \param uuid Characteristic UUID to locate on the peer.
    /// \return Characteristic object path, or an empty string when not found.
    std::string find_characteristic(const std::string& mac, const std::string& uuid) const;
    /// \brief Resolve a descriptor UUID to its D-Bus path.
    /// \param mac peer Bluetooth MAC address.
    /// \param uuid Descriptor UUID to locate on the peer.
    /// \param chrc_path Optional BlueZ characteristic path limiting the UUID lookup.
    /// \return Descriptor object path, or an empty string when not found.
    std::string find_descriptor(const std::string& mac, const std::string& uuid,
                                const std::string& chrc_path = "") const;

    // ---- GATT read/write ----
    /// \brief Read the current bytes from a remote GATT characteristic.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    /// \return Characteristic bytes, or an empty vector after a D-Bus failure.
    std::vector<uint8_t> read_characteristic(const std::string& chrc_path);
    /// \brief Write bytes to a remote characteristic and wait for the D-Bus result.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    /// \param data Bytes written to the remote GATT characteristic.
    /// \param with_response True selects an acknowledged request; false selects a write command.
    /// \return True if BlueZ completed the characteristic write; otherwise false.
    bool write_characteristic(const std::string& chrc_path,
                              const std::vector<uint8_t>& data,
                              bool with_response = true);
    /// \brief Queue a characteristic write without blocking the caller thread.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    /// \param data Bytes written to the remote GATT characteristic.
    /// \param with_response True selects an acknowledged request; false selects a write command.
    /// \return True if the asynchronous characteristic write was queued; otherwise false.
    bool write_characteristic_async(const std::string& chrc_path,
                                    const std::vector<uint8_t>& data,
                                    bool with_response = true);
    /// \brief Read the current bytes from a remote GATT descriptor.
    /// \param desc_path BlueZ path of the target GATT descriptor.
    /// \return Descriptor bytes, or an empty vector after a D-Bus failure.
    std::vector<uint8_t> read_descriptor(const std::string& desc_path);
    /// \brief Write bytes to a remote descriptor and wait for the D-Bus result.
    /// \param desc_path BlueZ path of the target GATT descriptor.
    /// \param data Bytes written to the remote GATT descriptor.
    /// \return True if BlueZ completed the descriptor write; otherwise false.
    bool write_descriptor(const std::string& desc_path,
                          const std::vector<uint8_t>& data);
    /// \brief Queue a descriptor write without blocking the caller thread.
    /// \param desc_path BlueZ path of the target GATT descriptor.
    /// \param data Bytes written to the remote GATT descriptor.
    /// \return True if the asynchronous descriptor write was queued; otherwise false.
    bool write_descriptor_async(const std::string& desc_path,
                                const std::vector<uint8_t>& data);

    // ---- Notifications ----
    /// \brief Enable remote characteristic notifications and install the signal match.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    /// \return True if notifications are active after the request; otherwise false.
    bool start_notify(const std::string& chrc_path);
    /// \brief Disable remote notifications and remove the signal match idempotently.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    /// \return True if notifications are inactive after the request; otherwise false.
    bool stop_notify(const std::string& chrc_path);
    /// \brief Report whether this process tracks an active notification subscription.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    /// \return True when the characteristic path is in the active set.
    bool is_notify_active(const std::string& chrc_path) const;

    /// Register for notification value callbacks.  Returns a token for removal.
    /// \param cb Observer receiving remote characteristic values.
    /// \return Token used to remove this notification handler.
    int add_notification_handler(NotificationCallback cb);
    /// \brief Stop delivering characteristic values to the registered observer.
    /// \param token Registration token returned by add_notification_handler.
    void remove_notification_handler(int token);

    /// Register an observer for local and remote GATT operation diagnostics.
    /// \param cb Observer receiving GATT operation diagnostics.
    /// \return Token used to remove this GATT event handler.
    int add_gatt_event_handler(GattEventCallback cb);
    /// \brief Stop delivering GATT operation events to the registered observer.
    /// \param token Registration token returned by add_gatt_event_handler.
    void remove_gatt_event_handler(int token);

private:
    /// \brief Update connection waiters and notification ownership after a cache event.
    /// \param event Kind of BlueZ cache mutation being reconciled.
    /// \param object_path BlueZ object path affected by the cache event.
    void on_cache_event(CacheEvent event, const std::string& object_path);
    /// \brief Resolve a peer address to its current Device1 path.
    /// \param mac peer Bluetooth MAC address.
    /// \return Current Device1 object path, or an empty string when unknown.
    std::string device_path_for_mac(const std::string& mac) const;
    /// \brief Reload a resolved device and all of its GATT descendants into the cache.
    /// \param device_path BlueZ device path whose GATT subtree is reloaded.
    /// \return True if the device and all present GATT descendants were refreshed; otherwise false.
    bool refresh_device_gatt_cache(const std::string& device_path) const;
    /// \brief Write one Device1 property through BlueZ.
    /// \param device_path BlueZ device path whose property is changed.
    /// \param prop Device1 property name set through D-Bus.
    /// \param value D-Bus value assigned to the device property.
    void set_device_property(const std::string& device_path,
                             const std::string& prop,
                             const sdbus::Variant& value);
    /// \brief Deliver a normalized GATT operation event to a stable observer snapshot.
    /// \param event Stable GATT operation event name delivered to observers.
    /// \param path BlueZ path of the GATT object associated with the event.
    /// \param detail Optional BlueZ result text delivered with the GATT event.
    void emit_gatt(const std::string& event, const std::string& path,
                   const std::string& detail = "");

    DbusConnection& dbus_;
    ObjectManagerCache& cache_;
    std::string adapter_path_;
    rclcpp::Logger logger_;

    mutable std::mutex mutex_;
    int cache_observer_token_{0};
    // Device1.Disconnected distinguishes stale security from normal RF loss.
    std::optional<sdbus::Slot> disconnect_match_;
    std::optional<sdbus::Slot> daemon_owner_match_;
    bool scan_running_{false};
    std::atomic_bool connections_allowed_{true};
    mutable std::recursive_mutex discovery_operation_mutex_;
    // This persistent blocking connection owns discovery and remains separate
    // from the cache/event connection used beside the service state lock.
    std::unique_ptr<sdbus::IConnection> discovery_connection_;
    std::string last_scan_transport_{"le"};
    bool last_scan_discoverable_{false};
    mutable std::map<std::string, std::chrono::steady_clock::time_point> gatt_refresh_backoff_until_;

    int next_ntf_token_{1};
    std::map<int, NotificationCallback> notification_cbs_;

    int next_gatt_token_{1};
    std::map<int, GattEventCallback> gatt_event_cbs_;

    /// Tracks characteristics for which notifications are enabled through the cache observer path.
    std::set<std::string> notify_paths_;
    /// \brief Install exactly one PropertiesChanged match for a notifying characteristic.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    void ensure_notify_match(const std::string& chrc_path);
    /// \brief Remove the PropertiesChanged match owned by a characteristic.
    /// \param chrc_path BlueZ path of the target GATT characteristic.
    void remove_notify_match(const std::string& chrc_path);
};

}  // namespace mrs_uav_bluetooth::bluez
