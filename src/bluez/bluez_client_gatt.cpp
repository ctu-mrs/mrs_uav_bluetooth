// SPDX-License-Identifier: BSD-3-Clause
/// \file src/bluez/bluez_client_gatt.cpp
/// \brief Implements the bluez client gatt component of the BlueZ system-D-Bus integration layer.

#include "mrs_uav_bluetooth/bluez/bluez_client.hpp"
#include "mrs_uav_bluetooth/bluez/bluez_constants.hpp"

#include <string_view>

namespace mrs_uav_bluetooth::bluez {

namespace {

constexpr auto kGattReadTimeout = std::chrono::seconds(2);

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

/// \brief Bind a proxy to this GATT object on the shared event connection.
/// \param connection System-bus connection used to query BlueZ.
/// \param object_path BlueZ D-Bus object path targeted by the proxy.
/// \return New bluez proxy.
std::unique_ptr<sdbus::IProxy> create_bluez_proxy(sdbus::IConnection& connection,
                                                  const std::string& object_path) {
    // Bind a proxy to this GATT object on the shared event connection.
    return sdbus::createProxy(connection,
                              sdbus::ServiceName{std::string(kBluezServiceName)},
                              sdbus::ObjectPath{object_path});
}

}  // namespace

std::string BluezClient::find_characteristic(const std::string& mac,
                                             const std::string& uuid) const {
    // Resolve a characteristic by UUID below the requested peer service tree.
    const auto dev = cache_.device_by_mac(mac);
    if (!dev) {
        return {};
    }
    auto result = cache_.find_characteristic_by_uuid(dev->object_path, uuid);
    return result ? result->object_path : std::string{};
}

std::string BluezClient::find_descriptor(const std::string& mac,
                                         const std::string& uuid,
                                         const std::string& chrc_path) const {
    // Resolve a descriptor by UUID below the requested characteristic path.
    if (!chrc_path.empty()) {
        auto result = cache_.find_descriptor_by_uuid(chrc_path, uuid);
        return result ? result->object_path : std::string{};
    }
    for (const auto& ch : list_characteristics(mac)) {
        const auto result = cache_.find_descriptor_by_uuid(ch.object_path, uuid);
        if (result) {
            return result->object_path;
        }
    }
    return {};
}

std::vector<uint8_t> BluezClient::read_characteristic(const std::string& chrc_path) {
    // Issue ReadValue with a bounded timeout and publish the operation result.
    try {
        auto proxy = create_bluez_proxy(dbus_.connection(), chrc_path);
        std::map<std::string, sdbus::Variant> options;
        std::vector<uint8_t> value;
        proxy->callMethod("ReadValue")
            .onInterface(std::string(kGattCharacteristicIface))
            .withTimeout(kGattReadTimeout)
            .withArguments(options)
            .storeResultsTo(value);
        emit_gatt("client_read", chrc_path);
        return value;
    } catch (const sdbus::Error& e) {
        emit_gatt("client_read_failed", chrc_path, e.getMessage());
        return {};
    }
}

bool BluezClient::write_characteristic(const std::string& chrc_path,
                                       const std::vector<uint8_t>& data,
                                       bool with_response) {
    // Issue WriteValue as a request or command and publish the synchronous result.
    try {
        auto proxy = create_bluez_proxy(dbus_.connection(), chrc_path);
        std::map<std::string, sdbus::Variant> options;
        options["type"] = sdbus::Variant{std::string(with_response ? "request" : "command")};
        proxy->callMethod("WriteValue")
            .onInterface(std::string(kGattCharacteristicIface))
            .withArguments(data, options);
        emit_gatt("client_write", chrc_path);
        return true;
    } catch (const sdbus::Error& e) {
        emit_gatt("client_write_failed", chrc_path, e.getMessage());
        return false;
    }
}

bool BluezClient::write_characteristic_async(const std::string& chrc_path,
                                             const std::vector<uint8_t>& data,
                                             bool with_response) {
    // Start a nonblocking WriteValue while retaining its proxy through reply delivery.
    try {
        auto proxy = std::shared_ptr<sdbus::IProxy>(
            create_bluez_proxy(dbus_.connection(), chrc_path).release());
        std::map<std::string, sdbus::Variant> options;
        options["type"] = sdbus::Variant{std::string(with_response ? "request" : "command")};
        const auto path_copy = chrc_path;
        proxy->callMethodAsync("WriteValue")
            .onInterface(std::string(kGattCharacteristicIface))
            .withArguments(data, options)
            .uponReplyInvoke([this, path_copy, proxy](std::optional<sdbus::Error> err) {
                // Keep the proxy alive through completion and publish the characteristic write result.
                (void)proxy;
                if (err) {
                    emit_gatt("client_write_failed", path_copy, err->getMessage());
                } else {
                    emit_gatt("client_write", path_copy);
                }
            });
        return true;
    } catch (const sdbus::Error& e) {
        emit_gatt("client_write_failed", chrc_path, e.getMessage());
        return false;
    }
}

std::vector<uint8_t> BluezClient::read_descriptor(const std::string& desc_path) {
    // Issue Descriptor1.ReadValue with a bounded timeout and publish the result.
    try {
        auto proxy = create_bluez_proxy(dbus_.connection(), desc_path);
        std::map<std::string, sdbus::Variant> options;
        std::vector<uint8_t> value;
        proxy->callMethod("ReadValue")
            .onInterface(std::string(kGattDescriptorIface))
            .withTimeout(kGattReadTimeout)
            .withArguments(options)
            .storeResultsTo(value);
        emit_gatt("client_descriptor_read", desc_path);
        return value;
    } catch (const sdbus::Error& e) {
        emit_gatt("client_descriptor_read_failed", desc_path, e.getMessage());
        return {};
    }
}

bool BluezClient::write_descriptor(const std::string& desc_path,
                                   const std::vector<uint8_t>& data) {
    // Write descriptor bytes synchronously and publish success or failure.
    try {
        auto proxy = create_bluez_proxy(dbus_.connection(), desc_path);
        std::map<std::string, sdbus::Variant> options;
        proxy->callMethod("WriteValue")
            .onInterface(std::string(kGattDescriptorIface))
            .withArguments(data, options);
        emit_gatt("client_descriptor_write", desc_path);
        return true;
    } catch (const sdbus::Error& e) {
        emit_gatt("client_descriptor_write_failed", desc_path, e.getMessage());
        return false;
    }
}

bool BluezClient::write_descriptor_async(const std::string& desc_path,
                                         const std::vector<uint8_t>& data) {
    // Start a nonblocking descriptor write while retaining its proxy through completion.
    try {
        auto proxy = std::shared_ptr<sdbus::IProxy>(
            create_bluez_proxy(dbus_.connection(), desc_path).release());
        std::map<std::string, sdbus::Variant> options;
        const auto path_copy = desc_path;
        proxy->callMethodAsync("WriteValue")
            .onInterface(std::string(kGattDescriptorIface))
            .withArguments(data, options)
            .uponReplyInvoke([this, path_copy, proxy](std::optional<sdbus::Error> err) {
                // Keep the proxy alive through completion and publish the descriptor write result.
                (void)proxy;
                if (err) {
                    emit_gatt("client_descriptor_write_failed", path_copy, err->getMessage());
                } else {
                    emit_gatt("client_descriptor_write", path_copy);
                }
            });
        return true;
    } catch (const sdbus::Error& e) {
        emit_gatt("client_descriptor_write_failed", desc_path, e.getMessage());
        return false;
    }
}

bool BluezClient::start_notify(const std::string& chrc_path) {
    // Start remote value updates and install the matching PropertiesChanged handler.
    RCLCPP_DEBUG(logger_, "[client] start_notify path=%s", chrc_path.c_str());
    try {
        auto proxy = create_bluez_proxy(dbus_.connection(), chrc_path);
        proxy->callMethod("StartNotify")
            .onInterface(std::string(kGattCharacteristicIface));
    } catch (const sdbus::Error& e) {
        const auto msg = e.getMessage();
        if (msg.find("InProgress") == std::string::npos &&
            msg.find("In Progress") == std::string::npos &&
            msg.find("Operation already in progress") == std::string::npos &&
            msg.find("Already notifying") == std::string::npos &&
            msg.find("AlreadyNotifying") == std::string::npos) {
            emit_gatt("client_notify_failed", chrc_path, msg);
            return false;
        }
    }

    ensure_notify_match(chrc_path);
    emit_gatt("client_notify_enabled", chrc_path);
    return true;
}

bool BluezClient::stop_notify(const std::string& chrc_path) {
    // Stop remote value updates and release the local signal subscription.
    RCLCPP_DEBUG(logger_, "[client] stop_notify path=%s", chrc_path.c_str());
    if (!is_notify_active(chrc_path)) {
        return true;
    }
    try {
        auto proxy = create_bluez_proxy(dbus_.connection(), chrc_path);
        proxy->callMethod("StopNotify")
            .onInterface(std::string(kGattCharacteristicIface));
    } catch (const sdbus::Error& e) {
        const auto message = e.getMessage();
        if (message_contains(message,
                             {"doesn't exist", "UnknownObject", "NoSuchObject", "Not notifying", "NotNotifying"})) {
            remove_notify_match(chrc_path);
            emit_gatt("client_notify_disabled", chrc_path, message);
            return true;
        }
        remove_notify_match(chrc_path);
        emit_gatt("client_notify_failed", chrc_path, message);
        return false;
    }

    remove_notify_match(chrc_path);
    emit_gatt("client_notify_disabled", chrc_path);
    return true;
}

}  // namespace mrs_uav_bluetooth::bluez
