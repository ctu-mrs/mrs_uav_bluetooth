// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bluez/bluez_client.cpp
/// \brief Implements the bluez client component of the BlueZ system-D-Bus integration layer.

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

namespace mrs_uav_bluetooth::bluez {

namespace {

using ManagedObjectMap = std::map<sdbus::ObjectPath,
                                  std::map<std::string, std::map<std::string, sdbus::Variant>>>;

// Discovery is best-effort during radio handoff. Short D-Bus deadlines keep
// overlay activation responsive while the controller changes ownership.
constexpr auto kDiscoveryTimeout = std::chrono::seconds(2);
constexpr auto kConnectPollInterval = std::chrono::milliseconds(300);
constexpr auto kPairPollInterval = std::chrono::milliseconds(500);
constexpr auto kGattRefreshRetryBackoff = std::chrono::milliseconds(3000);

/// \brief Open a synchronous system-bus connection for short-lived worker operations.
/// \return New blocking system bus.
std::unique_ptr<sdbus::IConnection> create_blocking_system_bus() {
    // Open an independent system-bus connection for bounded synchronous BlueZ calls.
    return sdbus::createSystemBusConnection();
}

/// \brief Bind a proxy to the requested BlueZ object on the caller-owned connection.
/// \param connection System-bus connection used to query BlueZ.
/// \param object_path BlueZ D-Bus object path targeted by the proxy.
/// \return New bluez proxy.
std::unique_ptr<sdbus::IProxy> create_bluez_proxy(sdbus::IConnection& connection,
                                                  const std::string& object_path) {
    // Bind a proxy to the requested BlueZ object on the caller-owned connection.
    return sdbus::createProxy(connection,
                              sdbus::ServiceName{std::string(kBluezServiceName)},
                              sdbus::ObjectPath{object_path});
}

template<typename T>
/// \brief Decode the requested variant type, returning the supplied fallback on absence or mismatch.
/// \param values D-Bus property dictionary searched for the named key.
/// \param key D-Bus property name to retrieve.
/// \param fallback value returned when the requested property is absent or ill-typed.
/// \return Requested variant or < bool >.
T get_variant_or(const std::map<std::string, sdbus::Variant>& values,
                 const std::string& key,
                 T fallback) {
    // Decode the requested variant type, returning the supplied fallback on absence or mismatch.
    auto it = values.find(key);
    if (it == values.end()) {
        return fallback;
    }
    try {
        return it->second.get<T>();
    } catch (...) {
        return fallback;
    }
}

/// \brief Match known BlueZ error fragments without depending on exact daemon wording.
/// \param message BlueZ error text to inspect.
/// \param needles accepted message fragments tested against a BlueZ error.
/// \return True when message contains; otherwise false.
bool message_contains(const std::string& message,
                      std::initializer_list<const char*> needles) {
    // Match known BlueZ error fragments without depending on exact daemon wording.
    for (const char* needle : needles) {
        if (message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// \brief Recognize BlueZ errors caused by an object vanishing mid-call.
/// \param message BlueZ error text to inspect.
/// \return True if the BlueZ error reports a vanished D-Bus object; otherwise false.
bool is_missing_object_error(const std::string& message) {
    // Accept standard D-Bus names plus BlueZ's textual GetAll race error.
    if (message_contains(message, {"NoSuchObject", "UnknownObject"})) {
        return true;
    }
    return message.find("GetAll") != std::string::npos &&
           message.find("doesn't exist") != std::string::npos;
}

/// \brief Test whether BlueZ reports a Low Energy address type.
/// \param address_type BlueZ address type used to determine LE bearer support.
/// \return True for BlueZ public or random low-energy address types; otherwise false.
bool is_le_address_type(const std::string& address_type) {
    // BlueZ uses `public` and `random` for both supported LE address forms.
    return address_type == "public" || address_type == "random";
}

/// \brief Walk from a GATT object path to its owning Device1 path.
/// \param object_path BlueZ object path from which the owning Device1 path is derived.
/// \return Owning Device1 path or an empty string for an unrelated object.
std::string device_root_path(const std::string& object_path) {
    // Walk from a GATT object path to its owning Device1 path.
    const auto device_pos = object_path.find("/dev_");
    if (device_pos == std::string::npos) {
        return {};
    }
    const auto suffix_pos = object_path.find('/', device_pos + 1);
    if (suffix_pos == std::string::npos) {
        return object_path;
    }
    return object_path.substr(0, suffix_pos);
}

/// \brief Read the peer's complete Device1 property dictionary if it still exists.
/// \param connection System-bus connection used to query BlueZ.
/// \param device_path BlueZ device path whose Device1 or GATT descendants are inspected.
/// \return Complete Device1 properties or std::nullopt if the object disappeared.
std::optional<std::map<std::string, sdbus::Variant>> read_device_properties(
    sdbus::IConnection& connection,
    const std::string& device_path) {
    // Read every Device1 property in one bounded D-Bus call.
    auto proxy = create_bluez_proxy(connection, device_path);
    std::map<std::string, sdbus::Variant> properties;
    proxy->callMethod("GetAll")
        .onInterface(std::string(kDbusPropertiesIface))
        .withArguments(std::string{kDeviceIface})
        .storeResultsTo(properties);
    return properties;
}

/// \brief Read all properties for one BlueZ interface if the object still exists.
/// \param connection System-bus connection used to query BlueZ.
/// \param object_path BlueZ object path whose interface properties are read.
/// \param interface BlueZ interface name whose properties are read or changed.
/// \return Complete interface properties or std::nullopt if the object disappeared.
std::optional<std::map<std::string, sdbus::Variant>> read_interface_properties(
    sdbus::IConnection& connection,
    const std::string& object_path,
    const std::string& interface) {
    // Read every property of the requested BlueZ interface in one bounded D-Bus call.
    auto proxy = create_bluez_proxy(connection, object_path);
    std::map<std::string, sdbus::Variant> properties;
    proxy->callMethod("GetAll")
        .onInterface(std::string(kDbusPropertiesIface))
        .withArguments(interface)
        .storeResultsTo(properties);
    return properties;
}

/// \brief Require ServicesResolved plus usable characteristics in the shared cache.
/// \param connection System-bus connection used to query BlueZ.
/// \param device_path BlueZ device path whose Device1 or GATT descendants are inspected.
/// \return True when device has resolved characteristics; otherwise false.
bool device_has_resolved_characteristics(sdbus::IConnection& connection,
                                         const std::string& device_path) {
    // Confirm both BlueZ resolution state and a populated characteristic cache.
    auto proxy = create_bluez_proxy(connection, "/");
    ManagedObjectMap objects;
    proxy->callMethod("GetManagedObjects")
        .onInterface(std::string(kDbusObjectManagerIface))
        .storeResultsTo(objects);

    for (const auto& [path, interfaces] : objects) {
        const auto path_string = static_cast<std::string>(path);
        if (path_string.find(device_path + "/") != 0) {
            continue;
        }
        if (interfaces.count(std::string(kGattCharacteristicIface)) != 0) {
            return true;
        }
    }
    return false;
}

template<typename Predicate>
/// \brief Evaluate a predicate at bounded intervals and once more at the deadline.
/// \param timeout_s Maximum seconds to evaluate the predicate.
/// \param interval delay between predicate checks.
/// \param predicate condition polled until success or timeout.
/// \return True when poll until; otherwise false.
bool poll_until(double timeout_s,
                std::chrono::milliseconds interval,
                Predicate&& predicate) {
    // Evaluate a predicate at bounded intervals and once more at the deadline.
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(static_cast<int64_t>(std::max(0.0, timeout_s) * 1000.0));
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(interval);
    }
    return predicate();
}

}  // namespace

BluezClient::BluezClient(DbusConnection& dbus,
                         ObjectManagerCache& cache,
                         const std::string& adapter_path,
                         rclcpp::Logger logger)
    : dbus_(dbus),
      cache_(cache),
      adapter_path_(adapter_path),
      logger_(logger) {
    // Forward BlueZ's explicit authentication reason as the endpoint evidence
    // that authorizes bond repair.
    disconnect_match_ = dbus_.connection().addMatch(
        "type='signal',sender='org.bluez',interface='org.bluez.Device1',"
        "member='Disconnected',path_namespace='" + adapter_path_ + "'",
        [this](sdbus::Message message) {
            // Publish BlueZ's explicit disconnect reason for bond-recovery decisions.
            std::string reason, description;
            message >> reason >> description;
            emit_gatt("device_disconnected", message.getPath(), reason);
        }, sdbus::return_slot);
    daemon_owner_match_ = dbus_.connection().addMatch(
        "type='signal',sender='org.freedesktop.DBus',"
        "interface='org.freedesktop.DBus',member='NameOwnerChanged',arg0='org.bluez'",
        [this](sdbus::Message message) {
            std::string name, previous_owner, owner;
            message >> name >> previous_owner >> owner;
            // Discovery sessions belong to the daemon instance that accepted
            // StartDiscovery. Notification subscriptions and refresh backoffs
            // are daemon-instance state as well.
            {
                std::lock_guard<std::mutex> lock(mutex_);
                scan_running_ = false;
                notify_paths_.clear();
                gatt_refresh_backoff_until_.clear();
            }
            if (!owner.empty() && owner != previous_owner) {
                emit_gatt("bluez_daemon_restarted", "/", owner);
            }
        }, sdbus::return_slot);
    cache_observer_token_ = cache_.add_observer(
        [this](CacheEvent event, const std::string& object_path) {
            // Maintain notification matches and refresh state when cached GATT objects change.
            on_cache_event(event, object_path);
        });
}

BluezClient::~BluezClient() {
    daemon_owner_match_.reset();
    disconnect_match_.reset();
    if (cache_observer_token_ != 0) {
        cache_.remove_observer(cache_observer_token_);
        cache_observer_token_ = 0;
    }

    // Release per-characteristic notify match slots.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        notify_paths_.clear();
    }
}

// ---------------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------------

bool BluezClient::start_scan(const std::string& transport,
                             bool make_discoverable_while_scanning) {
    std::lock_guard<std::recursive_mutex> operation_lock(discovery_operation_mutex_);
    if (!discovery_connection_) discovery_connection_ = create_blocking_system_bus();
    bool owned;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        owned = scan_running_;
    }
    const bool filter_changed = transport != last_scan_transport_ ||
        make_discoverable_while_scanning != last_scan_discoverable_;
    if (owned && !filter_changed) return true;

    try {
        auto proxy = sdbus::createProxy(*discovery_connection_,
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        std::map<std::string, sdbus::Variant> filter;
        filter["Transport"] = sdbus::Variant{transport};
        filter["Discoverable"] = sdbus::Variant{make_discoverable_while_scanning};
        proxy->callMethod("SetDiscoveryFilter")
            .onInterface(std::string(kAdapterIface))
            .withTimeout(kDiscoveryTimeout)
            .withArguments(filter);
        last_scan_transport_ = transport;
        last_scan_discoverable_ = make_discoverable_while_scanning;

        // A discovery session belongs to this D-Bus connection. BlueZ may
        // temporarily suspend radio scanning while the session stays owned,
        // for example during pairing or a legacy advertisement update.
        if (!owned) {
            proxy->callMethod("StartDiscovery")
                .onInterface(std::string(kAdapterIface))
                .withTimeout(kDiscoveryTimeout);
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = true;
        }
        return true;
    } catch (const sdbus::Error& error) {
        if (!owned) {
            // A timed-out StartDiscovery can still finish in the daemon.
            // Closing its owner connection cancels that request and releases
            // any late-created session before the next attempt.
            discovery_connection_.reset();
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = false;
        }
        RCLCPP_WARN(logger_, "start_scan failed: %s", error.getMessage().c_str());
        return false;
    }
}

bool BluezClient::stop_scan() {
    std::lock_guard<std::recursive_mutex> operation_lock(discovery_operation_mutex_);
    // A new client has no discovery session of its own to stop.
    if (!discovery_connection_) return true;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!scan_running_) return true;
    }
    try {
        auto proxy = sdbus::createProxy(*discovery_connection_,
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        proxy->callMethod("StopDiscovery")
            .onInterface(std::string(kAdapterIface))
            .withTimeout(kDiscoveryTimeout);
        std::lock_guard<std::mutex> lock(mutex_);
        scan_running_ = false;
        return true;
    } catch (const sdbus::Error& e) {
        std::string msg = e.getMessage();
        if (msg.find("NotAuthorized") != std::string::npos ||
            msg.find("No discovery started") != std::string::npos) {
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = false;
            return true;
        }
        RCLCPP_WARN(logger_, "stop_scan failed: %s", msg.c_str());
        return false;
    }
}

bool BluezClient::is_scanning() const {
    // Require both a live discovery request and BlueZ's cached Discovering property.
    std::lock_guard<std::recursive_mutex> operation_lock(discovery_operation_mutex_);
    bool requested;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requested = scan_running_;
    }
    if (!requested || !discovery_connection_) return false;
    try {
        auto proxy = sdbus::createProxy(*discovery_connection_,
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        return proxy->getProperty("Discovering")
            .onInterface(std::string(kAdapterIface)).get<bool>();
    } catch (const sdbus::Error& error) {
        RCLCPP_WARN(logger_, "Could not verify adapter discovery state: %s",
                    error.getMessage().c_str());
        return false;
    }
}

// ---------------------------------------------------------------------------
// Device queries
// ---------------------------------------------------------------------------

std::vector<DeviceInfo> BluezClient::get_devices() const {
    // Return a locked snapshot of all devices known by the shared object cache.
    return cache_.devices();
}

std::optional<DeviceInfo> BluezClient::get_device(const std::string& mac) const {
    // Look up one device by normalized address in the shared object cache.
    return cache_.device_by_mac(mac);
}

std::vector<DeviceInfo> BluezClient::get_connected_devices() const {
    // Return only cache records whose Device1 Connected property is true.
    return cache_.connected_devices();
}

// ---------------------------------------------------------------------------
// Connection management
// ---------------------------------------------------------------------------

bool BluezClient::connect(const std::string& mac, double timeout_s, bool prefer_le) {
    // Resolve the latest Device1 path, request a connection on the selected
    // bearer, and wait on fresh cache or property state until the deadline.
    if (!connections_allowed_.load()) return false;
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return false;
    if (dev->connected) return true;
    auto path = dev->object_path;
    const bool prefer_le_transport = prefer_le && is_le_address_type(dev->address_type);
    RCLCPP_INFO(logger_, "[client] connect(%s) path=%s timeout=%.1fs prefer_le=%s addr_type=%s",
                mac.c_str(), path.c_str(), timeout_s,
                prefer_le ? "true" : "false",
                dev->address_type.empty() ? "<unknown>" : dev->address_type.c_str());

    auto connection = create_blocking_system_bus();
    const auto device_connected_now = [&]() {
        // Prefer the fresh cache path, then verify Connected directly on Device1.
        if (const auto refreshed = cache_.device_by_mac(mac)) {
            if (!refreshed->object_path.empty()) {
                path = refreshed->object_path;
            }
            if (refreshed->connected) {
                return true;
            }
        }

        try {
            const auto properties = read_device_properties(*connection, path);
            return properties && get_variant_or<bool>(*properties, "Connected", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    };

    // This peer already has a Device1 object. Keep its connection in that
    // object's lifecycle so Device1.Disconnect cancels an unfinished Connect
    // during an overlay handoff. Adapter1.ConnectDevice opens a separate
    // discovery-free socket, outside Device1's pending connection state.
    if (prefer_le_transport) {
        (void)set_preferred_bearer(mac, "le");
    }
    if (!connections_allowed_.load()) return false;
    try {
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("Connect").onInterface(std::string(kDeviceIface));
    } catch (const sdbus::Error& error) {
        // A handoff deliberately cancels this call through Disconnect.
        if (!connections_allowed_.load()) return false;
        const auto message = error.getMessage();
        if (!message_contains(message, {"Already Connected", "AlreadyConnected", "InProgress",
                                        "In Progress", "Operation already in progress",
                                        "No more profiles to connect to", "br-connection-already-connected"})) {
            if (device_connected_now()) return true;
            RCLCPP_WARN(logger_, "connect(%s) failed: %s", mac.c_str(), message.c_str());
            return false;
        }
    }
    if (!connections_allowed_.load()) {
        // Also release a link whose completion raced the handoff.
        (void)disconnect(mac, timeout_s);
        return false;
    }

    return poll_until(timeout_s, kConnectPollInterval, [&]() {
        // Poll Device1 until BlueZ confirms that the connection completed.
        try {
            const auto properties = read_device_properties(*connection, path);
            return properties && get_variant_or<bool>(*properties, "Connected", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    });
}

bool BluezClient::connect_le_bearer(const std::string& mac, double timeout_s) {
    if (!connections_allowed_.load()) return false;
    auto dev = cache_.device_by_mac(mac);
    if (!dev || dev->object_path.empty()) {
        return false;
    }

    const auto path = dev->object_path;
    auto connection = create_blocking_system_bus();
    const auto le_connected = [&]() -> std::optional<bool> {
        // Query the LE bearer specifically; absence means this BlueZ version lacks bearer state.
        try {
            const auto properties = read_interface_properties(
                *connection, path, std::string{kLeBearerIface});
            return properties && get_variant_or<bool>(*properties, "Connected", false);
        } catch (const sdbus::Error& error) {
            if (message_contains(error.getMessage(),
                                 {"UnknownInterface", "UnknownProperty", "doesn't exist"})) {
                return std::nullopt;
            }
            throw;
        }
    };

    try {
        const auto connected = le_connected();
        if (connected && *connected) {
            return true;
        }
        if (connected.has_value()) {
            RCLCPP_INFO(logger_, "[client] connect_le_bearer(%s) path=%s timeout=%.1fs",
                        mac.c_str(), path.c_str(), timeout_s);
            try {
                auto proxy = create_bluez_proxy(*connection, path);
                proxy->callMethod("Connect")
                    .onInterface(std::string(kLeBearerIface));
            } catch (const sdbus::Error& error) {
                if (!message_contains(error.getMessage(),
                                      {"AlreadyConnected", "Already connected", "InProgress",
                                       "In Progress", "Operation already in progress"})) {
                    RCLCPP_WARN(logger_, "connect_le_bearer(%s) failed: %s",
                                mac.c_str(), error.getMessage().c_str());
                    return false;
                }
            }
            return poll_until(timeout_s, kConnectPollInterval, [&]() {
                // Wait until the LE bearer reports connected after ConnectLE returns.
                const auto state = le_connected();
                return state && *state;
            });
        }
    } catch (const sdbus::Error& error) {
        RCLCPP_WARN(logger_, "connect_le_bearer(%s) state check failed: %s",
                    mac.c_str(), error.getMessage().c_str());
        return false;
    }

    if (!connections_allowed_.load()) return false;
    // Device1.Connect selects the disconnected bearer when BR/EDR is already
    // active. PreferredBearer selects LE for the initial connection on BlueZ
    // versions whose Device1 properties omit Bearer.LE1.
    (void)set_preferred_bearer(mac, "le");
    try {
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("Connect")
            .onInterface(std::string(kDeviceIface));
        return true;
    } catch (const sdbus::Error& error) {
        if (message_contains(error.getMessage(),
                             {"AlreadyConnected", "Already connected", "InProgress",
                              "In Progress", "Operation already in progress"})) {
            return true;
        }
        RCLCPP_WARN(logger_, "connect_le_bearer(%s) Device1 fallback failed: %s",
                    mac.c_str(), error.getMessage().c_str());
        return false;
    }
}

bool BluezClient::disconnect(const std::string& mac, double timeout_s) {
    auto path = device_path_for_mac(mac);
    if (path.empty()) return true;
    auto dev = cache_.device_by_mac(mac);
    // Disconnect also cancels an outstanding Connect/Pair before Connected
    // becomes true. A cached false must not skip this cancellation on handoff.
    if (!dev) return true;
    RCLCPP_INFO(logger_, "[client] disconnect(%s) path=%s timeout=%.1fs",
                mac.c_str(), path.c_str(), timeout_s);

    auto connection = create_blocking_system_bus();
    auto proxy = create_bluez_proxy(*connection, path);
    const bool restore_trust = dev->trusted;
    const auto update_trust = [&](bool trusted) {
        // Temporarily change Trusted through Properties.Set while cancelling the connection.
        proxy->callMethod("Set")
            .onInterface(std::string(kDbusPropertiesIface))
            .withArguments(std::string(kDeviceIface),
                           std::string{"Trusted"},
                           sdbus::Variant{trusted});
    };
    if (restore_trust) {
        try {
            update_trust(false);
        } catch (const sdbus::Error& error) {
            RCLCPP_WARN(logger_,
                "disconnect(%s) could not suspend trusted reconnect policy: %s",
                mac.c_str(), error.getMessage().c_str());
            return false;
        }
    }
    bool disconnected = true;
    try {
        proxy->callMethod("Disconnect")
            .onInterface(std::string(kDeviceIface))
            .withTimeout(std::chrono::seconds(5));
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        // Private advertising addresses can expire from BlueZ between the
        // snapshot and cancellation. A vanished object has nothing to release.
        disconnected = error.getName() == "org.bluez.Error.NotConnected" ||
            error.getName() == "org.freedesktop.DBus.Error.UnknownObject";
        if (!disconnected &&
            error.getName() == "org.freedesktop.DBus.Error.UnknownMethod") {
            // BlueZ reports UnknownMethod when the device object vanished.
            // Verify absence through ObjectManager before accepting it.
            auto manager = create_bluez_proxy(*connection, "/");
            ManagedObjectMap objects;
            manager->callMethod("GetManagedObjects")
                .onInterface(std::string(kDbusObjectManagerIface))
                .storeResultsTo(objects);
            const auto object = objects.find(sdbus::ObjectPath{path});
            disconnected = object == objects.end() ||
                !object->second.contains(std::string(kDeviceIface));
        }
        if (!disconnected)
            RCLCPP_WARN(logger_, "disconnect(%s) failed: %s", mac.c_str(), message.c_str());
    }
    bool released = false;
    if (disconnected) {
        released = poll_until(timeout_s, kConnectPollInterval, [&]() {
            // Poll until Device1 disappears or reports Connected=false.
            try {
                const auto properties = read_device_properties(*connection, path);
                return !properties || !get_variant_or<bool>(*properties, "Connected", false);
            } catch (const sdbus::Error& error) {
                return is_missing_object_error(error.getMessage());
            }
        });
    }
    if (restore_trust) {
        try {
            update_trust(true);
        } catch (const sdbus::Error& error) {
            RCLCPP_WARN(logger_, "disconnect(%s) could not restore Trusted=true: %s",
                        mac.c_str(), error.getMessage().c_str());
            return false;
        }
    }
    return released;
}

bool BluezClient::connect_profile(const std::string& mac, const std::string& uuid) {
    // Connect profile.
    if (!connections_allowed_.load()) return false;
    const auto path = device_path_for_mac(mac);
    if (path.empty() || uuid.empty()) {
        return false;
    }

    try {
        auto connection = create_blocking_system_bus();
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("ConnectProfile")
            .onInterface(std::string(kDeviceIface))
            .withArguments(uuid);
        return true;
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (message_contains(message, {"AlreadyConnected", "Already connected"})) {
            return true;
        }
        RCLCPP_WARN(logger_, "connect_profile(%s, %s) failed: %s",
                    mac.c_str(), uuid.c_str(), message.c_str());
        return false;
    }
}

bool BluezClient::disconnect_profile(const std::string& mac, const std::string& uuid) {
    // Disconnect profile.
    const auto path = device_path_for_mac(mac);
    if (path.empty() || uuid.empty()) {
        return true;
    }

    try {
        auto connection = create_blocking_system_bus();
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("DisconnectProfile")
            .onInterface(std::string(kDeviceIface))
            .withArguments(uuid);
        return true;
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (message_contains(message, {"NotConnected", "NoSuchObject", "UnknownObject"})) {
            return true;
        }
        RCLCPP_WARN(logger_, "disconnect_profile(%s, %s) failed: %s",
                    mac.c_str(), uuid.c_str(), message.c_str());
        return false;
    }
}

// ---------------------------------------------------------------------------
// Pairing/trust
// ---------------------------------------------------------------------------

bool BluezClient::pair(const std::string& mac, double timeout_s, std::string* error_detail) {
    // Start one bounded Pair transaction, retain BlueZ's authentication detail,
    // and explicitly cancel a daemon operation that outlives the local timeout.
    if (!connections_allowed_.load()) return false;
    if (error_detail != nullptr) {
        error_detail->clear();
    }
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return false;
    if (dev->paired) return true;
    const auto path = dev->object_path;
    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(timeout_s);
    RCLCPP_INFO(logger_, "[client] pair(%s) path=%s timeout=%.1fs",
                mac.c_str(), path.c_str(), timeout_s);

    auto connection = create_blocking_system_bus();
    try {
        auto proxy = create_bluez_proxy(*connection, path);
        proxy->callMethod("Pair")
            .onInterface(std::string(kDeviceIface))
            .withTimeout(std::chrono::milliseconds(
                static_cast<int64_t>(std::max(0.1, timeout_s) * 1000.0)));
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (std::chrono::steady_clock::now() >= deadline) {
            // Explicitly cancel BlueZ's pending Pair operation after the D-Bus
            // deadline while retaining any established bond storage.
            try {
                auto proxy = create_bluez_proxy(*connection, path);
                proxy->callMethod("CancelPairing")
                    .onInterface(std::string(kDeviceIface));
            } catch (const sdbus::Error& cancel_error) {
                RCLCPP_WARN(logger_, "CancelPairing(%s): %s",
                            mac.c_str(), cancel_error.getMessage().c_str());
            }
        }
        if (!message_contains(message, {"AlreadyExists", "Already Paired", "AlreadyPaired", "InProgress",
                                        "In Progress", "Operation already in progress"})) {
            if (error_detail != nullptr) {
                *error_detail = message;
            }
            RCLCPP_WARN(logger_, "pair(%s) failed: %s", mac.c_str(), message.c_str());
            return false;
        }
    }

    const double remaining_s = std::max(0.0, std::chrono::duration<double>(
        deadline - std::chrono::steady_clock::now()).count());
    const bool paired = poll_until(remaining_s, kPairPollInterval, [&]() {
        // Poll Device1 until BlueZ confirms that pairing completed.
        try {
            const auto properties = read_device_properties(*connection, path);
            return properties && get_variant_or<bool>(*properties, "Paired", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    });
    if (!paired && error_detail != nullptr && error_detail->empty()) {
        *error_detail = "timed out waiting for Paired=true";
    }
    return paired;
}

bool BluezClient::trust(const std::string& mac) {
    // Trust bluez client.
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    RCLCPP_INFO(logger_, "[client] trust(%s) path=%s", mac.c_str(), path.c_str());
    try {
        set_device_property(path, "Trusted", sdbus::Variant{true});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "trust(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::untrust(const std::string& mac) {
    // Resolve Device1 and store Trusted=false through the Properties interface.
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "Trusted", sdbus::Variant{false});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "untrust(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::block(const std::string& mac) {
    // Resolve Device1 and store Blocked=true through the Properties interface.
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "Blocked", sdbus::Variant{true});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "block(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::unblock(const std::string& mac) {
    // Resolve Device1 and store Blocked=false through the Properties interface.
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "Blocked", sdbus::Variant{false});
        return true;
    } catch (const sdbus::Error& e) {
        RCLCPP_WARN(logger_, "unblock(%s) failed: %s", mac.c_str(), e.getMessage().c_str());
        return false;
    }
}

bool BluezClient::remove(const std::string& mac) {
    // Ask Adapter1 to remove the cached Device1 object and its bond idempotently.
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return true;
    RCLCPP_INFO(logger_, "[client] remove(%s) path=%s", mac.c_str(), dev->object_path.c_str());
    try {
        auto proxy = sdbus::createProxy(dbus_.connection(),
                                        sdbus::ServiceName{std::string(kBluezServiceName)},
                                        sdbus::ObjectPath{adapter_path_});
        proxy->callMethod("RemoveDevice")
            .onInterface(std::string(kAdapterIface))
            .withArguments(sdbus::ObjectPath{dev->object_path});
    } catch (const sdbus::Error& e) {
        const auto message = e.getMessage();
        if (message_contains(message, {"NoSuchObject", "UnknownObject", "doesn't exist"})) {
            return true;
        }
        RCLCPP_WARN(logger_, "remove(%s) failed: %s", mac.c_str(), message.c_str());
        return false;
    }

    return poll_until(8.0, kConnectPollInterval, [&]() {
        // Wait until the removed bond disappears from the object cache.
        const auto current = cache_.device_by_mac(mac);
        if (!current) {
            return true;
        }
        return !current->connected &&
               !current->services_resolved &&
               !current->paired &&
               !current->bonded &&
               !current->trusted;
    });
}

bool BluezClient::set_preferred_bearer(const std::string& mac, const std::string& bearer) {
    // Set Device1.PreferredBearer so the next profile connection chooses BR/EDR or LE.
    auto path = device_path_for_mac(mac);
    if (path.empty()) return false;
    try {
        set_device_property(path, "PreferredBearer", sdbus::Variant{bearer});
        return true;
    } catch (const sdbus::Error& error) {
        const auto message = error.getMessage();
        if (message_contains(message, {"UnknownProperty", "InvalidArguments", "NotSupported", "does not exist"})) {
            RCLCPP_DEBUG(logger_, "set_preferred_bearer(%s, %s) unavailable: %s",
                         mac.c_str(), bearer.c_str(), message.c_str());
            return false;
        }
        RCLCPP_WARN(logger_, "set_preferred_bearer(%s, %s) failed: %s",
                    mac.c_str(), bearer.c_str(), message.c_str());
        return false;
    }
}

// ---------------------------------------------------------------------------
// Services resolved
// ---------------------------------------------------------------------------

bool BluezClient::wait_services_resolved(const std::string& mac, double timeout_s) {
    // Poll until the peer remains connected and BlueZ finishes remote GATT discovery.
    auto dev = cache_.device_by_mac(mac);
    if (!dev) {
        return false;
    }

    const auto path = dev->object_path;
    auto connection = create_blocking_system_bus();
    return poll_until(timeout_s, kConnectPollInterval, [&]() {
        // Require both the link and remote service discovery to be complete.
        try {
            const auto properties = read_device_properties(*connection, path);
            if (!properties) {
                return false;
            }
            if (!get_variant_or<bool>(*properties, "Connected", false)) {
                return false;
            }
            return get_variant_or<bool>(*properties, "ServicesResolved", false);
        } catch (const sdbus::Error& error) {
            if (is_missing_object_error(error.getMessage())) {
                return false;
            }
            throw;
        }
    });
}

bool BluezClient::refresh_gatt_snapshot(const std::string& mac) const {
    // Reconcile the connected peer subtree from a fresh ObjectManager snapshot.
    auto dev = cache_.device_by_mac(mac);
    if (!dev || !dev->connected || !dev->services_resolved) {
        return false;
    }
    return refresh_device_gatt_cache(dev->object_path);
}

// ---------------------------------------------------------------------------
// GATT queries
// ---------------------------------------------------------------------------

std::vector<GattServiceInfo> BluezClient::list_services(const std::string& mac) const {
    // Return cached services for one peer address.
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return {};
    return cache_.services_for_device(dev->object_path);
}

std::vector<GattCharacteristicInfo> BluezClient::list_characteristics(const std::string& mac) const {
    // Return cached characteristics below the selected service or peer.
    auto dev = cache_.device_by_mac(mac);
    if (!dev) return {};
    std::vector<GattCharacteristicInfo> result;
    auto services = cache_.services_for_device(dev->object_path);
    for (const auto& svc : services) {
        auto chars = cache_.characteristics_for_service(svc.object_path);
        result.insert(result.end(), chars.begin(), chars.end());
    }
    return result;
}

std::vector<GattDescriptorInfo> BluezClient::list_descriptors(
    const std::string& mac, const std::string& chrc_path) const {
    // Return cached descriptors below the selected characteristic.
    if (!chrc_path.empty()) {
        return cache_.descriptors_for_characteristic(chrc_path);
    }
    std::vector<GattDescriptorInfo> result;
    for (const auto& ch : list_characteristics(mac)) {
        auto descs = cache_.descriptors_for_characteristic(ch.object_path);
        result.insert(result.end(), descs.begin(), descs.end());
    }
    return result;
}

int BluezClient::add_notification_handler(NotificationCallback cb) {
    // Register a subscriber under a token that can later remove it safely.
    std::lock_guard<std::mutex> lock(mutex_);
    int token = next_ntf_token_++;
    notification_cbs_[token] = std::move(cb);
    return token;
}

void BluezClient::remove_notification_handler(int token) {
    // Stop delivering values to the observer owning this registration token.
    std::lock_guard<std::mutex> lock(mutex_);
    notification_cbs_.erase(token);
}

int BluezClient::add_gatt_event_handler(GattEventCallback cb) {
    // Register an operation-event subscriber under a removable token.
    std::lock_guard<std::mutex> lock(mutex_);
    int token = next_gatt_token_++;
    gatt_event_cbs_[token] = std::move(cb);
    return token;
}

void BluezClient::remove_gatt_event_handler(int token) {
    // Stop delivering operation diagnostics to the observer owning this token.
    std::lock_guard<std::mutex> lock(mutex_);
    gatt_event_cbs_.erase(token);
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

bool BluezClient::refresh_device_gatt_cache(const std::string& device_path) const {
    // Ask the object cache to reconcile one device subtree from BlueZ ObjectManager.
    if (device_path.empty()) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = gatt_refresh_backoff_until_.find(device_path);
        if (it != gatt_refresh_backoff_until_.end() && now < it->second) {
            return false;
        }
        gatt_refresh_backoff_until_[device_path] = now + kGattRefreshRetryBackoff;
    }

    auto& cache = const_cast<ObjectManagerCache&>(cache_);
    const bool refreshed = cache.refresh_device_subtree(device_path);

    if (refreshed) {
        std::lock_guard<std::mutex> lock(mutex_);
        gatt_refresh_backoff_until_.erase(device_path);
    }

    return refreshed;
}

void BluezClient::on_cache_event(CacheEvent event, const std::string& object_path) {
    // Clear per-device refresh backoff and reconcile discovery and notification
    // ownership after BlueZ changes cached adapter or GATT objects.
    if (const auto device_path = device_root_path(object_path); !device_path.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        gatt_refresh_backoff_until_.erase(device_path);
    }

    if (event == CacheEvent::AdapterChanged && object_path == adapter_path_) {
        const auto adapter = cache_.adapter(adapter_path_);
        if (!adapter || !adapter->powered) {
            std::lock_guard<std::mutex> lock(mutex_);
            scan_running_ = false;
        }
        if (adapter) {
            // Adapter discovery is global. Only our Start/StopDiscovery calls
            // determine ownership of the discovery session on this connection.
            RCLCPP_DEBUG(logger_, "[client] adapter changed: discovering=%s",
                         adapter->discovering ? "true" : "false");
        }
    }
    if (event == CacheEvent::GattCharacteristicRemoved) {
        remove_notify_match(object_path);
    } else if (event == CacheEvent::GattCharacteristicChanged) {
        const auto characteristic = cache_.characteristic(object_path);
        if (!characteristic || !characteristic->notifying) {
            remove_notify_match(object_path);
        }
    } else if (event == CacheEvent::DeviceRemoved ||
               event == CacheEvent::DevicePropertyChanged) {
        const auto device = cache_.device(object_path);
        if (event == CacheEvent::DeviceRemoved || (device && !device->connected)) {
            const auto prefix = object_path + "/";
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto it = notify_paths_.begin(); it != notify_paths_.end();) {
                if (it->find(prefix) == 0) {
                    it = notify_paths_.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }
    if (event == CacheEvent::GattCharacteristicValueChanged) {
        const auto characteristic = cache_.characteristic(object_path);
        if (characteristic) {
            RCLCPP_DEBUG(logger_, "[client] notification from %s uuid=%s %zu bytes",
                         object_path.c_str(), characteristic->uuid.c_str(),
                         characteristic->value.size());
            std::vector<NotificationCallback> cbs;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cbs.reserve(notification_cbs_.size());
                for (auto& [_, cb] : notification_cbs_) {
                    cbs.push_back(cb);
                }
            }
            for (auto& cb : cbs) {
                try {
                    cb(characteristic->value, characteristic->uuid, object_path);
                } catch (...) {
                }
            }
        }
    }
}

std::string BluezClient::device_path_for_mac(const std::string& mac) const {
    // Resolve a normalized peer address to its current BlueZ object path.
    auto dev = cache_.device_by_mac(mac);
    return dev ? dev->object_path : std::string{};
}

void BluezClient::set_device_property(const std::string& device_path,
                                      const std::string& prop,
                                      const sdbus::Variant& value) {
    // Set one Device1 property through the standard D-Bus Properties interface.
    auto connection = create_blocking_system_bus();
    auto proxy = create_bluez_proxy(*connection, device_path);
    proxy->callMethod("Set")
        .onInterface(std::string(kDbusPropertiesIface))
        .withArguments(std::string{kDeviceIface}, prop, value);
}

void BluezClient::emit_gatt(const std::string& event,
                             const std::string& path,
                             const std::string& detail) {
    // Snapshot observers under lock, then deliver diagnostics without holding client state.
    std::vector<GattEventCallback> cbs;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cbs.reserve(gatt_event_cbs_.size());
        for (auto& [_, cb] : gatt_event_cbs_) {
            cbs.push_back(cb);
        }
    }
    for (auto& cb : cbs) {
        try {
            cb(event, path, detail);
        } catch (...) {}
    }
}

void BluezClient::ensure_notify_match(const std::string& chrc_path) {
    // Ensure notify match.
    std::lock_guard<std::mutex> lock(mutex_);
    if (!notify_paths_.insert(chrc_path).second) {
        return;
    }
    RCLCPP_DEBUG(logger_, "[client] tracking notify path via cache observer for %s", chrc_path.c_str());
}

void BluezClient::remove_notify_match(const std::string& chrc_path) {
    // Drop local ownership tracking after notification teardown or object removal.
    std::lock_guard<std::mutex> lock(mutex_);
    notify_paths_.erase(chrc_path);
}

bool BluezClient::is_notify_active(const std::string& chrc_path) const {
    // Membership means this process currently owns notification tracking.
    std::lock_guard<std::mutex> lock(mutex_);
    return notify_paths_.count(chrc_path) != 0;
}

}  // namespace mrs_uav_bluetooth::bluez
